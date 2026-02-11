# Fork - Real-Time Gripper Control Fix

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

- [#1398](https://github.com/facebookresearch/fairo/pull/1398) - Original gripper stuck issue
- [#1417](https://github.com/facebookresearch/fairo/pull/1417) - Exposed `stop()` method (partial fix)
- [#1418](https://github.com/facebookresearch/fairo/pull/1418)

---

# Fairo
...
