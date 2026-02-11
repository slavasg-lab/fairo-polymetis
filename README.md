# Fork - Franka Hand Fix

This fork fixes the gripper deadlock issue where the Franka Hand becomes unresponsive when `goto()` commands target unreachable widths, e.g., when an object blocks the gripper from closing fully. While [#1417](https://github.com/facebookresearch/fairo/pull/1417) partially addressed this by exposing a `stop()` method, calling `stop()` alone doesn't resolve the underlying problem in real-time control scenarios (see explanation below).

## The Problem

The original code in `franka_hand_client.cpp` has two parts:

**Command execution** (runs in detached thread):
```cpp
void FrankaHandClient::applyGripperCommand(void) {
  is_moving_ = true;
  
  if (gripper_cmd_.grasp()) {
    prev_cmd_successful_ = gripper_->grasp(gripper_cmd_.width(), 
                                           gripper_cmd_.speed(),
                                           gripper_cmd_.force(), 
                                           eps_inner, eps_outer);
  } else {
    prev_cmd_successful_ = gripper_->move(gripper_cmd_.width(), 
                                          gripper_cmd_.speed());
  }
  
  is_moving_ = false;
}
```

Here, `grasp()` or `move()` are blocking calls that don't return until the motion completes, keeping `is_moving_` set to `true` until they either timeout, throw an error, or successfully reach the target width.

**Main control loop**:
```cpp
void FrankaHandClient::run(void) {
  while (true) {
    getGripperState();
    status_ = stub_->ControlUpdate(&context, gripper_state_, &gripper_cmd_);
    
    if (!is_moving_) {  // Only accepts commands when NOT moving
      timestamp_ns = gripper_cmd_.timestamp().nanos();
      if (timestamp_ns != prev_cmd_timestamp_ns_ && timestamp_ns) {
        std::thread th(&FrankaHandClient::applyGripperCommand, this);
        th.detach();
        prev_cmd_timestamp_ns_ = timestamp_ns;
      }
    }
    
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &abs_target_time, nullptr);
  }
}
```

Here, this loop **ignores any commands if the gripper is currently moving**. 

So when the gripper gets stuck trying to reach an unreachable width (as mentioned earlier, due to an object blocking it):
1. `gripper_->move()` blocks trying to reach target
2. `is_moving_` stays `true`
3. All new commands (including `stop()` exposed in [#1417](https://github.com/facebookresearch/fairo/pull/1417)) are ignored
4. Gripper stays stuck

## The Fix

This fork simply calls `stop()` to abort the previous motion before executing any new command. This makes it work for real-time applications where policies continuously stream Franka Hand commands.

## Related Issues

- [#1398](https://github.com/facebookresearch/fairo/pull/1398)
- [#1417](https://github.com/facebookresearch/fairo/pull/1417)
- [#1418](https://github.com/facebookresearch/fairo/pull/1418)

---

# Fairo

[![CircleCI](https://circleci.com/gh/facebookresearch/fairo/tree/main.svg?style=svg&circle-token=7fadbd3989ab8e76003fd5193ad62e26686bc4a6)](https://circleci.com/gh/facebookresearch/fairo/tree/main)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![codecov](https://codecov.io/gh/facebookresearch/fairo/branch/main/graph/badge.svg?token=8ERT95OC8G)](https://codecov.io/gh/facebookresearch/fairo)

**Fairo** (pronounced _"pharaoh"_) is a unified robotics platform developed by researchers at [Meta AI](https://ai.facebook.com/). It currently comprises of a set of projects:

- [Droidlet](./droidlet) is an **_early research project for AI researchers_** to explore ideas around grounded dialogue, interactive learning and human-computer interfaces. It helps you rapidly build agents (real or virtual) that perform a wide variety of tasks specified by humans. The agents can use natural language, memory and humans in the loop.
- [Polymetis](./polymetis) is a PyTorch-based real-time controller manager. It lets you write PyTorch controllers for robots, test them in simulation, and seamlessly transfer to real-time hardware.
- [Meta Robotics Platform](./mrp) lets you deploy, launch, manage, and orchestrate heterogeneous robots with ease.

Fairo is [MIT licensed](./LICENSE).
