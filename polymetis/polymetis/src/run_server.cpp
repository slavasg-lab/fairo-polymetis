// Copyright (c) Facebook, Inc. and its affiliates.

// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
#include "polymetis/polymetis_server.hpp"
#include "real_time.hpp"
#include "torch_server_ops.hpp"

#include <grpcpp/resource_quota.h>

void *RunServer(void *server_address_ptr) {
  std::string &server_address =
      *(static_cast<std::string *>(server_address_ptr));

  // Instantiate service
  PolymetisControllerServerImpl service;

  // Build service with performance tuning
  ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);

  // Message size limits — our messages are tiny (~200 bytes)
  builder.SetMaxReceiveMessageSize(64 * 1024);  // 64KB
  builder.SetMaxSendMessageSize(64 * 1024);      // 64KB

  // gRPC sync-server thread configuration.
  // The workload mixes:
  //   - 1 kHz unary ControlUpdate (hot path, most critical)
  //   - long-lived GetRobotStateStream (blocking, occupies one thread)
  //   - infrequent SetController / UpdateController / TerminateController
  // Keeping NUM_CQS=1 avoids cross-queue wakeup overhead, but we need at
  // least enough pollers and threads so the hot ControlUpdate path is never
  // starved while GetRobotStateStream occupies a thread.
  builder.SetSyncServerOption(ServerBuilder::SyncServerOption::NUM_CQS, 1);
  builder.SetSyncServerOption(ServerBuilder::SyncServerOption::MIN_POLLERS, 2);
  builder.SetSyncServerOption(ServerBuilder::SyncServerOption::MAX_POLLERS, 4);

  // Cap total threads at 8: enough for ControlUpdate + StateStream + a few
  // controller-management RPCs without letting gRPC spawn unboundedly.
  grpc::ResourceQuota rq;
  rq.SetMaxThreads(8);
  builder.SetResourceQuota(rq);

  // Channel args for low latency
  builder.AddChannelArgument(GRPC_ARG_ALLOW_REUSEPORT, 0);
  builder.AddChannelArgument(GRPC_ARG_KEEPALIVE_TIME_MS, 10000);
  builder.AddChannelArgument(GRPC_ARG_KEEPALIVE_TIMEOUT_MS, 5000);
  builder.AddChannelArgument(GRPC_ARG_HTTP2_MIN_RECV_PING_INTERVAL_WITHOUT_DATA_MS, 5000);
  builder.AddChannelArgument(GRPC_ARG_KEEPALIVE_PERMIT_WITHOUT_CALLS, 1);

  std::unique_ptr<Server> server(builder.BuildAndStart());

  // Start server
  spdlog::info("Server listening on {}", server_address);
  server->Wait();

  return NULL;
}

int main(int argc, char **argv) {
  // Parse inputs
  InputParser input(argc, argv);

  if (input.cmdOptionExists("-h")) {
    spdlog::info("Usage: polymetis_server [OPTION]");
    spdlog::info("Starts a controller manager server.");
    spdlog::info("  -h   Help");
    spdlog::info("  -r   Use real-time (requires sudo)");
    spdlog::info("  -s   Change server address");
    return 0;
  }

  bool use_real_time = false;
  if (input.cmdOptionExists("-r")) {
    use_real_time = true;
  }

  std::string ip = "0.0.0.0";
  if (input.cmdOptionExists("-s")) {
    ip = input.getCmdOption("-s");
  }
  std::string port = "50051";
  if (input.cmdOptionExists("-p")) {
    port = input.getCmdOption("-p");
  }
  std::string server_address = ip + ":" + port;

  spdlog::info("Using real time: {}", use_real_time);
  spdlog::info("Using server address: {}", server_address);

  // Start real-time thread
  void *server_address_ptr = static_cast<void *>(&server_address);

  if (!use_real_time) {
    RunServer(server_address_ptr);
  } else {
    create_real_time_thread(RunServer, server_address_ptr);
  }

  return 0;
}