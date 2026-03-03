# First Demo Considerations (Franka Panda + Franka Hand + Polymetis RPC)

Date: 2026-02-19

This document is a practical go/no-go checklist for a first supervised real-robot demo using:

- Polymetis arm server/client
- Polymetis gripper server/client
- an external high-level policy bridge (for example, UMI)

---

## 1) Safety gate (must pass first)

- [ ] E-stop is physically reachable and tested.
- [ ] Franka Desk state is valid (robot unlocked, external activation device released).
- [ ] Conservative initial pose and workspace are verified on hardware.
- [ ] Collision behavior and torque limits are set conservatively in [polymetis/polymetis/python/polymetis/conf/robot_client/franka_hardware.yaml](polymetis/polymetis/python/polymetis/conf/robot_client/franka_hardware.yaml).
- [ ] Safety-controller limits/margins/stiffness are validated in the same config.
- [ ] Recovery path is rehearsed: stop, recover, restart services, re-home if required.

---

## 2) Service bring-up and interface health

- [ ] Arm server/client is up via [polymetis/polymetis/python/scripts/launch_robot.py](polymetis/polymetis/python/scripts/launch_robot.py).
- [ ] Gripper server/client is up via [polymetis/polymetis/python/scripts/launch_gripper.py](polymetis/polymetis/python/scripts/launch_gripper.py).
- [ ] Arm RPC endpoint responds (`50051` default).
- [ ] Gripper RPC endpoint responds (`50052` default).
- [ ] External bridge endpoint is reachable and mapped to correct host/port.

Recommended first pass:

- [ ] Run arm in `readonly=true` once before torque-enabled run.

---

## 3) Version and schema compatibility

- [ ] Polymetis client and server versions match (keep version enforcement enabled in [polymetis/polymetis/python/polymetis/robot_interface.py](polymetis/polymetis/python/polymetis/robot_interface.py)).
- [ ] Robot metadata fields are present and correct (DOF, URDF, EE link, gains).
- [ ] Gripper command semantics match bridge assumptions (`goto` vs `grasp`, `cancel_prev`).

---

## 4) Timing and latency alignment

- [ ] Arm action latency calibrated and applied.
- [ ] Gripper action latency calibrated and applied.
- [ ] Camera observation latency measured and applied.
- [ ] Control frequency set conservatively for first run.
- [ ] Clocks across machines are synchronized (NTP/PTP).
- [ ] Network jitter/loss on the control path is measured and acceptable.

---

## 5) Motion smoke tests (before policy)

- [ ] Manual arm motion test: smooth, stable, no oscillation.
- [ ] Manual gripper test: open/close/grasp works without chatter.
- [ ] Repeated interrupted grasps do not deadlock the hand client.
- [ ] Emergency stop and re-enable are confirmed during an active control loop.

---

## 6) Frame and scaling consistency

- [ ] Robot base frame and camera frame transforms are verified.
- [ ] Policy action space units match runtime units.
- [ ] Gripper width scaling matches training dataset assumptions.
- [ ] Joint and Cartesian command clipping is active.

---

## 7) First policy rollout protocol

Use a short, supervised run first:

- [ ] Short horizon.
- [ ] Reduced speed/frequency.
- [ ] Simple scene.
- [ ] Operator ready on E-stop.

Observe and record:

- [ ] arm lag/overshoot,
- [ ] gripper lag/chatter,
- [ ] command preemption behavior,
- [ ] unexpected stop/recovery conditions.

---

## 8) Logging and postmortem minimum

- [ ] Save command stream (arm + gripper).
- [ ] Save robot state log and timestamps.
- [ ] Save camera timestamps and effective observation latency.
- [ ] Save policy outputs and inference timing.
- [ ] Save config snapshot and git commit hashes.

---

## 9) Go / No-Go criteria

Proceed to full demo only if all are true:

- [ ] No persistent phase lag at target frequency.
- [ ] No unstable oscillation in arm or gripper.
- [ ] No deadlock in repeated gripper command cycles.
- [ ] Recovery sequence is reliable and repeatable.
- [ ] Safety operator signs off on repeatability.

---

## 10) Quick references

- Architecture overview: [polymetis/architecture.md](polymetis/architecture.md)
- Arm launcher: [polymetis/polymetis/python/scripts/launch_robot.py](polymetis/polymetis/python/scripts/launch_robot.py)
- Gripper launcher: [polymetis/polymetis/python/scripts/launch_gripper.py](polymetis/polymetis/python/scripts/launch_gripper.py)
- Franka hardware config: [polymetis/polymetis/python/polymetis/conf/robot_client/franka_hardware.yaml](polymetis/polymetis/python/polymetis/conf/robot_client/franka_hardware.yaml)
- Franka hand config: [polymetis/polymetis/python/polymetis/conf/gripper/franka_hand.yaml](polymetis/polymetis/python/polymetis/conf/gripper/franka_hand.yaml)
