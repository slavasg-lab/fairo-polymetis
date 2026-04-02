#include "polymetis/clients/franka_hand_client.hpp"

#include "spdlog/spdlog.h"
#include <string>
#include <thread>
#include <time.h>

#include <grpc/grpc.h>

#include "polymetis.grpc.pb.h"
#include "polymetis/utils.h"

using grpc::ClientContext;

FrankaHandClient::FrankaHandClient(std::shared_ptr<grpc::Channel> channel,
                                   YAML::Node config)
    : stub_(GripperServer::NewStub(channel)) {
  // Store connection info for reconnection
  robot_ip_ = config["robot_ip"].as<std::string>();
  control_address_ = config["control_ip"].as<std::string>() + ":" +
                     config["control_port"].as<std::string>();

  // Connect to gripper
  spdlog::info("Connecting to robot_ip {}", robot_ip_);
  gripper_.reset(new franka::Gripper(robot_ip_));

  // Initialize gripper
  gripper_->homing();
  is_moving_ = false;

  // Initialize server connection
  franka::GripperState franka_gripper_state = gripper_->readOnce();

  GripperMetadata metadata;
  metadata.set_max_width(franka_gripper_state.max_width);
  metadata.set_hz(GRIPPER_HZ);

  ClientContext context;
  Empty empty;
  stub_->InitRobotClient(&context, metadata, &empty);

  spdlog::info("Connected.", robot_ip_);
}

void FrankaHandClient::getGripperState(void) {
  franka::GripperState franka_gripper_state = gripper_->readOnce();

  gripper_state_.set_width(franka_gripper_state.width);
  gripper_state_.set_is_grasped(franka_gripper_state.is_grasped);
  gripper_state_.set_is_moving(is_moving_);
  gripper_state_.set_prev_command_successful(prev_cmd_successful_);

  // gripper_state.time();  // Use current timestamp instead!
  setTimestampToNow(gripper_state_.mutable_timestamp());
}

void FrankaHandClient::applyGripperCommand(void) {
  is_moving_ = true;

  try {
    if (gripper_cmd_.grasp()) {
      spdlog::info("Grasping at width {} at speed={}", gripper_cmd_.width(),
                  gripper_cmd_.speed());
      double eps_inner = (gripper_cmd_.epsilon_inner() < 0)
                            ? EPSILON_INNER
                            : gripper_cmd_.epsilon_inner();
      double eps_outer = (gripper_cmd_.epsilon_outer() < 0)
                            ? EPSILON_OUTER
                            : gripper_cmd_.epsilon_outer();
      prev_cmd_successful_ =
          gripper_->grasp(gripper_cmd_.width(), gripper_cmd_.speed(),
                          gripper_cmd_.force(), eps_inner, eps_outer);

    } else {
      spdlog::info("Moving to width {} at speed={}", gripper_cmd_.width(),
                  gripper_cmd_.speed());
      prev_cmd_successful_ =
          gripper_->move(gripper_cmd_.width(), gripper_cmd_.speed());
    }
  } catch (const std::exception &e) {
    spdlog::warn("Gripper command exception: {}", e.what());
    prev_cmd_successful_ = false;
  }

  is_moving_ = false;
}

void FrankaHandClient::run(void) {
  int period = 1.0 / GRIPPER_HZ;
  int period_ns = period * 1.0e9;

  int timestamp_ns;

  struct timespec abs_target_time;
  clock_gettime(CLOCK_REALTIME, &abs_target_time);
  while (true) {
    try {
      // Run control step
      getGripperState();

      grpc::ClientContext context;
      status_ = stub_->ControlUpdate(&context, gripper_state_, &gripper_cmd_);

      if (!status_.ok()) {
        spdlog::warn("Gripper ControlUpdate failed: {}. Retrying...",
                     status_.error_message());
        goto sleep;
      }

      if (is_moving_ && gripper_cmd_.cancel_prev()) {
        spdlog::info("Preempting previous command with new command.");
        try {
          gripper_->stop();
        } catch (const std::exception &e) {
          spdlog::warn("Failed to stop gripper: {}", e.what());
        }
        is_moving_ = false;
      }

      if (!is_moving_) {
        // Skip if command not updated
        timestamp_ns = gripper_cmd_.timestamp().nanos();
        if (timestamp_ns != prev_cmd_timestamp_ns_ && timestamp_ns) {
          // Debounce: only execute if target changed significantly
          float width_delta =
              std::abs(gripper_cmd_.width() - prev_cmd_width_);
          bool grasp_changed = (gripper_cmd_.grasp() != prev_cmd_grasp_);

          if (width_delta > GRIPPER_WIDTH_TOLERANCE || grasp_changed ||
              prev_cmd_width_ < 0) {
            // applyGripperCommand() in separate thread
            std::thread th(&FrankaHandClient::applyGripperCommand, this);
            th.detach();
            prev_cmd_width_ = gripper_cmd_.width();
            prev_cmd_grasp_ = gripper_cmd_.grasp();
          }
          prev_cmd_timestamp_ns_ = timestamp_ns;
        }
      }
    } catch (const franka::NetworkException &e) {
      spdlog::error("Gripper network error: {}. Attempting reconnection...",
                    e.what());
      try {
        gripper_.reset(new franka::Gripper(robot_ip_));
        spdlog::info("Gripper reconnected successfully.");
      } catch (const std::exception &e2) {
        spdlog::error("Gripper reconnection failed: {}. Will retry...",
                      e2.what());
        usleep(1000000);
      }
    } catch (const std::exception &e) {
      spdlog::error("Gripper control loop error: {}. Continuing...", e.what());
      usleep(100000);
    }

  sleep:
    // Spin once
    abs_target_time.tv_nsec += period_ns;
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &abs_target_time, nullptr);
  }
}

int main(int argc, char *argv[]) {
  if (argc != 2) {
    spdlog::error("Usage: franka_hand_client /path/to/cfg.yaml");
    return 1;
  }
  YAML::Node config = YAML::LoadFile(argv[1]);

  // Launch client
  std::string control_address = config["control_ip"].as<std::string>() + ":" +
                                config["control_port"].as<std::string>();
  FrankaHandClient franka_hand_client(
      grpc::CreateChannel(control_address, grpc::InsecureChannelCredentials()),
      config);
  franka_hand_client.run();

  // Termination
  spdlog::info("Wait for shutdown; press CTRL+C to close.");

  return 0;
}
