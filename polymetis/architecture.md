# Polymetis Architecture Deep Dive

This document explains how Polymetis is structured, with emphasis on:

1. controlling **Franka Emika Panda** (arm) and **Franka Hand** (gripper),
2. how the **RPC/gRPC servers** are used,
3. and how to integrate this stack with **UMI**.

---

## 1) Executive summary

Polymetis uses a split architecture:

- a **controller-manager server** (gRPC service) that runs control policies,
- a **robot client** process that talks to hardware (or simulation),
- a **user client** API (`RobotInterface`, `GripperInterface`) that sends high-level commands.

For Franka:

- The Panda arm runs through `PolymetisControllerServer` on port `50051` (default).
- The hand typically runs through `GripperServer` on port `50052` (default), with a dedicated hand client process.

The key design idea is that user code is mostly the same for simulation and hardware; only the robot-client backend changes.

---

## 2) Repository map (what matters most)

### Core protocol and server

- `polymetis/polymetis/protos/polymetis.proto`
	- Defines `PolymetisControllerServer` (arm control) and `GripperServer` (gripper control).
- `polymetis/polymetis/src/polymetis_server.cpp`
	- Implementation of arm gRPC server logic.
- `polymetis/polymetis/src/run_server.cpp`
	- Server process entrypoint.
- `polymetis/polymetis/include/polymetis/polymetis_server.hpp`
	- Server class declarations and controller context structs.

### Franka Panda and hand hardware clients (C++)

- `polymetis/polymetis/src/clients/franka_panda_client/franka_panda_client.cpp`
	- Real-time client loop for Panda arm via libfranka.
- `polymetis/polymetis/src/clients/franka_panda_client/franka_hand_client.cpp`
	- Hand client loop that bridges hand state/commands through gRPC.

### Python user API and launchers

- `polymetis/polymetis/python/polymetis/robot_interface.py`
	- Main user-facing arm API.
- `polymetis/polymetis/python/polymetis/gripper_interface.py`
	- Main user-facing gripper API.
- `polymetis/polymetis/python/scripts/launch_robot.py`
	- Launches controller server + robot client (sim or hardware).
- `polymetis/polymetis/python/scripts/launch_gripper.py`
	- Launches gripper server + gripper client.

### Config

- `polymetis/polymetis/python/polymetis/conf/robot_client/franka_hardware.yaml`
	- Franka arm hardware config, metadata, limits, and safety-controller params.
- `polymetis/polymetis/python/polymetis/conf/gripper/franka_hand.yaml`
	- Franka hand client config.

---

## 3) Main runtime components

## 3.1 Controller Manager Server (arm)

Service: `PolymetisControllerServer`

Responsibilities:

- Accept metadata from robot client (`InitRobotClient`).
- Run either:
	- default controller (from metadata), or
	- user-uploaded TorchScript controller.
- Execute per-tick control updates via `ControlUpdate(RobotState) -> TorqueCommand`.
- Store robot-state logs and expose them (`GetRobotStateLog`, streams, episode intervals).

Important internal concepts:

- `RobotClientContext`
	- robot metadata + default controller + heartbeat timestamp.
- `CustomControllerContext`
	- currently active uploaded policy, episode boundaries, status (`READY`, `RUNNING`, `TERMINATING`, ...).
- Circular buffer for robot-state log (`MAX_CIRCULAR_BUFFER_SIZE`).

---

## 3.2 Robot Client (arm-side process)

For Franka Panda, this is the C++ libfranka process (`franka_panda_client`):

- Reads state from libfranka.
- Sends it over gRPC (`ControlUpdate`).
- Receives torque command.
- Applies safety checks/limits and optional filtering.
- Sends torque to robot at control rate.

This process is the real-time edge facing hardware.

---

## 3.3 User Client

### Arm API (`RobotInterface`)

Used by experiments/apps to:

- upload a scripted policy (`send_torch_policy`),
- update running policy parameters (`update_current_policy`),
- terminate policy (`terminate_current_policy`),
- stream or query robot state/log.

Also provides convenience calls for joint/EE pose goals and impedance modes.

### Gripper API (`GripperInterface`)

Used to:

- query `get_state()`,
- command `goto(...)` and `grasp(...)`.

Internally commands are queued and sent asynchronously to `GripperServer`.

---

## 4) Arm control flow for Franka Panda (end-to-end)

## 4.1 Startup

1. `launch_robot.py` starts `run_server` (gRPC server).
2. It instantiates the configured robot client (`franka_hardware.yaml` => executable C++ client).
3. Robot client reads generated metadata file and calls `InitRobotClient`.

Metadata includes:

- DOF, URDF, EE link,
- default gains (`Kq`, `Kqd`, `Kx`, `Kxd`),
- default TorchScript controller binary,
- Polymetis version.

## 4.2 Runtime tick (high frequency)

Per control cycle in Panda client:

1. Read robot state from libfranka.
2. Populate `RobotState` protobuf.
3. RPC: `ControlUpdate`.
4. Server forwards through current controller and returns torques.
5. Client applies:
	 - hard/soft safety constraints,
	 - torque clamping,
	 - optional rate limiting / low-pass behavior.
6. Send torques to robot.
7. Repeat.

## 4.3 Policy lifecycle

- `SetController(stream ControllerChunk)`
	- uploads TorchScript module in chunks,
	- swaps into active controller,
	- starts a new episode interval.
- `UpdateController(stream ControllerChunk)`
	- updates parameter dictionary of currently running controller.
- `TerminateController()`
	- transitions back to default controller,
	- closes episode interval.

---

## 5) Gripper control flow for Franka Hand

There are two practical modes in this repo:

1. **Python gripper server wrapper** (`PolymetisGripperServer`) for generic clients.
2. **Franka hand-specific C++ client** (`franka_hand_client`) that talks to Franka hand hardware and polls/updates the `GripperServer`.

Typical flow using `launch_gripper.py gripper=franka_hand`:

1. Gripper server starts (default `0.0.0.0:50052`).
2. `franka_hand_client` starts, homes the hand, and calls `InitRobotClient(GripperMetadata)`.
3. At ~30 Hz, hand client:
	 - reads hand state,
	 - sends state via `ControlUpdate`,
	 - receives latest gripper command,
	 - executes move/grasp if command timestamp is newer.
4. User-side `GripperInterface` pushes commands (`goto`, `grasp`) to server.

So the gripper path is also server-mediated, but with a much simpler command/state model than the arm.

---

## 6) Real-time and safety characteristics

Polymetis expects a real-time-capable Linux machine for deterministic hardware loops.

When `use_real_time=true`:

- server/client can be launched with elevated scheduling setup,
- `real_time.hpp` uses:
	- memory locking (`mlockall`),
	- real-time scheduling (`SCHED_FIFO`),
	- stack and allocator tuning,
	- CPU DMA latency tuning.

Franka Panda hardware client additionally enforces:

- workspace and joint limit checks,
- velocity and torque limits,
- safety reflex torques,
- collision behavior thresholds,
- optional read-only mode for dry runs.

---

## 7) Why simulation and hardware share user code

User client always talks to the same gRPC server interface.

Only robot client backend changes:

- simulation robot client (`polysim.grpc_sim_client.GrpcSimulationClient`), or
- hardware robot client (`franka_panda_client` executable).

This is the central abstraction that makes policy transfer easy.

---

## 8) RPC schema highlights (important messages)

### Arm service

- `RobotState`:
	- joint positions/velocities,
	- measured/desired torques,
	- previous command success flags,
	- timestamp and error code.
- `TorqueCommand`:
	- torque vector for all arm joints.
- `RobotClientMetadata`:
	- URDF + default controller + gains + robot model metadata.

### Gripper service

- `GripperState`: width, moving/grasp flags, status.
- `GripperCommand`: width/speed/force + grasp mode + cancel/epsilon flags.

---

## 9) How to use this with UMI

> Note: there is no first-class `umi` package inside this repo.
> Integration is done by bridging UMI outputs to Polymetis APIs.

Assuming UMI is your high-level manipulation stack (planner/policy/state machine), the integration pattern is:

## 9.1 Recommended architecture

- Keep Polymetis as the **low-level real-time execution layer** (1 kHz arm loop).
- Run UMI as a **high-level decision layer** (e.g., 10–100 Hz).
- Use a bridge process that converts UMI intents into:
	- arm commands through `RobotInterface`,
	- gripper commands through `GripperInterface`.

## 9.2 Integration modes

### A) Goal-based mode (simplest)

- UMI emits discrete EE/joint goals.
- Bridge calls `move_to_ee_pose(...)` / `move_to_joint_positions(...)`.
- Best for staged tasks and robust bring-up.

### B) Streaming setpoint mode (most common)

- Start a persistent impedance controller once (`start_joint_impedance` or `start_cartesian_impedance`).
- UMI continuously updates desired targets with:
	- `update_desired_joint_positions(...)`,
	- `update_desired_ee_pose(...)`,
	- optionally velocity updates.
- This preserves real-time stability in Polymetis while enabling responsive high-level control.

### C) Full policy handoff mode

- UMI exports a Torch policy (or wraps one).
- Bridge sends scripted policy via `send_torch_policy(...)`.
- Use `update_current_policy(...)` for online parameter adaptation.

## 9.3 Gripper with UMI

- Treat gripper as a parallel channel:
	- open/close/goto/grasp events from UMI,
	- translated to `GripperInterface.goto(...)` / `grasp(...)`.
- Keep arm and gripper sequencing in UMI state machine, but let Polymetis handle transport and device execution.

## 9.4 Practical timing guidance

- Do **not** run heavyweight UMI perception/planning in the same real-time process as hardware control.
- Keep Panda control loop on dedicated machine/process.
- Send compact setpoints/commands over network.
- Use Polymetis state logs (`GetRobotStateLog`) to align/diagnose UMI behavior.

---

## 10) Typical Franka + Franka Hand deployment

1. Start arm server+client:
	 - `launch_robot.py robot_client=franka_hardware`
2. Start gripper server+client:
	 - `launch_gripper.py gripper=franka_hand`
3. In UMI bridge process:
	 - construct `RobotInterface(ip, 50051)`
	 - construct `GripperInterface(ip, 50052)`
	 - run UMI loop and issue commands.

---

## 11) Debugging and observability tips

- If controller upload fails: verify TorchScript compatibility and protobuf chunking path.
- If commands lag: inspect `prev_controller_latency_ms` in state.
- If no motion: check safety controller constraints, read-only mode, and Franka E-stop/EAD status.
- If gripper appears stale: verify timestamp updates on gripper commands and server reachability on `50052`.

---

## 12) Key takeaway

Polymetis is best viewed as a **real-time robot execution substrate**:

- strict low-level control loop + safety on hardware client,
- policy/state orchestration on gRPC server,
- ergonomic user APIs for external systems.

UMI integration is straightforward when UMI is kept as high-level logic and Polymetis remains the deterministic low-level controller/runtime.

