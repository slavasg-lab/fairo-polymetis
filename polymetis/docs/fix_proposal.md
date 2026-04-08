# Fix Proposal: Stabilizing Polymetis gRPC + Franka 1 kHz Control

Date: 2026-04-08  
Scope: Review of the recent Polymetis server/client/runtime modifications for reducing CPU load and preventing `communication_constraints_violation` during policy execution.

---

## 1. Executive Summary

The current changes are **partially useful**, but they do **not yet address the most likely root causes** of the observed failures.

The measured network RTT to the Panda is already very good (~0.19 ms average), so the dominant remaining risk is **host-side scheduling jitter, server/client contention, and control-path overhead** rather than raw Ethernet latency.

My current assessment is:

1. **The server/client CPU pinning is likely not reliably applied as intended.**
2. **The gRPC server was tuned too aggressively for thread minimization**, which may hurt the control path under mixed load.
3. **The hot path still performs avoidable work and dynamic allocations**, especially in the Franka client safety path.
4. **The design still uses a unary RPC in a 1 kHz loop**, which is fundamentally expensive and fragile.
5. **The most important next step is to fix affinity correctness first, then relax gRPC throttling, then reduce hot-loop compute.**

This proposal separates **short-term fixes** from **structural improvements**.

---

## 2. Observed Symptoms

### Runtime symptoms

- Repeated reflex aborts with:
  - `libfranka: Move command aborted: motion aborted by reflex! ["communication_constraints_violation"]`
- Repeated server-side warning after recovery:
  - `Interrupted control update greater than threshold of 1000000000 ns. Reverting to default controller...`
- Repeated fallback in the client:
  - `ControlUpdate RPC failed ... Using local gravity compensation as fallback.`
- Success rate only around `0.74 - 0.79`
- Failures appear **after the machine has been running for a while**, not necessarily immediately after reboot

### Network evidence

Ping results are already strong:

- `rtt min/avg/max/mdev = 0.138/0.192/0.358/0.029 ms`

This strongly suggests that **the network itself is not the primary bottleneck anymore**.

---

## 3. Constraints from Official Franka / gRPC Guidance

### Franka requirements

According to the Franka troubleshooting and real-time control guidance:

- `round-trip time + control loop execution` must remain below **1 ms**.
- In a 1 kHz loop, the interval between **read and write should complete within ~500 µs**.
- Avoid in the hot loop:
  - blocking operations
  - printing/logging every cycle
  - dynamic memory allocations
  - repeated heavy model work that can be moved out of the loop
- `communication_constraints_violation` can be caused by packet loss **or** host-side timing problems.

### gRPC guidance

According to the gRPC performance guidance:

- reuse channels and stubs
- keepalive mainly helps during **idle periods**, not as a direct accelerator for already busy RPC loops
- **streaming RPCs** are recommended for long-lived logical flows because they avoid repeated RPC initiation overhead
- for performance-sensitive C++ servers, the **sync API is not recommended**
- long-lived streams and high-load RPCs can suffer from connection/stream queueing and may benefit from **separate channels**

---

## 4. Review of the Current Changes

## 4.1 `polymetis_server.cpp`

### What is good

The following changes are reasonable and beneficial:

- pre-allocating vectors for state extraction
- reusing `TorqueCommand` instead of rebuilding it every cycle
- RAII locking via `std::unique_lock`
- updating `last_update_ns` early to avoid a stale-context death loop
- unlocking before state recording so `SetController` is blocked for less time

### Assessment

These changes **do improve hot-path hygiene** and reduce some unnecessary heap churn.

### Limitation

They are **micro-optimizations**, not a solution to the main real-time risk. The dominant issue appears to be **thread scheduling / server contention / RPC architecture**, not a few vector resizes on the server alone.

### Remaining risk inside this file

`ControlUpdate()` still takes `custom_controller_context_.controller_mtx` on every cycle. At the same time:

- `SetController()` can swap in a new controller
- `UpdateController()` can run `param_dict_update_module()` under the same lock
- `TerminateController()` also touches controller state

This means the 1 kHz control path can still be **blocked by controller-management work**, exactly when the policy/action stream starts.

### Verdict

**Keep these changes.** They are fine.  
But they are **not sufficient**.

---

## 4.2 `franka_panda_client.cpp`

### What is good

- persistent stub/channel reuse is correct
- local gravity fallback on RPC failure is sensible
- model is loaded once, not inside the loop
- recovery logic is more robust than a hard crash

### What is misleading or ineffective

#### a) `GRPC_ARG_TCP_READ_CHUNK_SIZE`

The code comment says this “disables Nagle”, but that is **not what this option does**. It only changes the size of the slice used when reading from the wire.

So this tuning is **not evidence of lower latency in the intended sense**.

#### b) Keepalive tuning

Keepalive helps detect stale transports and can reduce startup delay after idle periods, but it is **not a primary latency optimization** for a continuously active 1 kHz unary RPC path.

### Hot-path issues that still matter

Inside the control loop, the code still performs work that should be minimized further:

- fresh unary `ControlUpdate()` RPC per cycle
- `ClientContext` creation per cycle
- safety processing every cycle
- Jacobian calculation even if cartesian soft-limit feedback may not actually be needed
- dynamic Eigen vectors in `checkStateLimits()`

The hot loop should be as deterministic and allocation-free as possible. At the moment, it is still heavier than ideal.

### Verdict

**Partially useful, but still too heavy in the control loop.**

---

## 4.3 `run_server.cpp`

### What is good

The intent is understandable: reduce internal gRPC background activity and keep the server lean.

### Why this is risky

The current configuration is:

- `NUM_CQS = 1`
- `MIN_POLLERS = 1`
- `MAX_POLLERS = 2`
- `ResourceQuota.SetMaxThreads(4)`

For a workload that mixes:

- a high-frequency hot RPC (`ControlUpdate`)
- long-lived stream RPCs (`GetRobotStateStream`)
- occasional controller-management RPCs (`SetController`, `UpdateController`, `TerminateController`)

this may be **too restrictive**.

Instead of reducing jitter, this can cause:

- starvation of the hot path
- queueing delays
- more sensitivity when policy/action streaming begins

Also, pinning the thread that calls `server->Wait()` does **not prove** that all gRPC worker/poller threads run on the intended isolated core.

### Verdict

**Most likely harmful or at least too aggressive in its current form.**

---

## 4.4 `launch_robot.py`

### Critical issue

The command construction for `sudo env ... taskset ...` is very likely broken:

```python
server_cmd = [
    "sudo", "-s", "env", '"PATH=$PATH"',
    f'"POLYMETIS_RT_CPU={server_cpu}"',
    "taskset", "-c", server_cpu,
] + server_cmd + ["-r"]
```

Because `subprocess.Popen()` is called with a list of arguments, there is **no shell unquoting**. The embedded quotes are therefore likely passed literally.

That means the environment variables may **not** be set the way the code assumes.

### Why this matters

The logs claim:

- launcher: server pinned to CPU 4, client uses CPU 5
- runtime thread log: `Pinned realtime thread to CPU 5`

This mismatch is a strong warning sign that the affinity setup is **not actually behaving as intended**.

### Verdict

**This should be fixed first.** It is a correctness issue, not just a tuning issue.

---

## 4.5 `real_time.hpp`

### What is good

- uses `mlockall`
- uses `SCHED_FIFO`
- sets explicit scheduling policy
- pins the created RT thread to a selected CPU

### Remaining caveat

This only pins the thread created by `create_real_time_thread()`. It does **not automatically pin** all internal worker threads created later by gRPC or third-party code.

### Verdict

**Useful foundation, but not sufficient by itself.**

---

## 5. Most Likely Root Causes

Ranked from highest to lower priority:

### 1) Affinity / isolation is not reliably applied

This is the strongest immediate concern because:

- the launcher command appears malformed
- logs suggest the intended CPU split is not cleanly enforced
- the issue worsens after the machine has been running for some time, which is consistent with increasing scheduler interference

### 2) gRPC sync server is over-constrained

The thread caps and poller caps may be creating exactly the wrong kind of fragility under mixed RPC load.

### 3) Controller-management work can interfere with the 1 kHz control loop

A shared mutex between `ControlUpdate()` and controller update/swap logic is dangerous in a hard real-time-adjacent path.

### 4) The hot loop still contains avoidable compute and allocations

The client still does more per cycle than desirable for a 1 kHz Franka loop.

### 5) Unary-RPC-at-1-kHz architecture remains fundamentally fragile

Even after local fixes, a fresh unary gRPC call per control cycle remains an expensive design choice.

---

## 6. Recommended Fixes

## 6.1 Immediate Fix A — Correct the launcher and verify real affinity

### Goal

Make sure CPU pinning actually works as intended.

### Proposed change

Replace the current quoted `env` usage with a proper environment dictionary or unquoted args.

### Safer implementation sketch

```python
server_env = os.environ.copy()
server_env["PATH"] = BUILD_DIR + os.pathsep + server_env.get("PATH", "")
server_env["POLYMETIS_RT_CPU"] = server_cpu

server_cmd = ["sudo", "taskset", "-c", server_cpu, server_exec_path, "-s", ip, "-p", port, "-r"]
server_output = subprocess.Popen(
    server_cmd,
    stdout=sys.stdout,
    stderr=sys.stderr,
    preexec_fn=os.setpgrp,
    env=server_env,
)
```

If `sudo` strips environment variables on your setup, use:

```python
server_cmd = [
    "sudo", "env",
    f"PATH={server_env['PATH']}",
    f"POLYMETIS_RT_CPU={server_cpu}",
    "taskset", "-c", server_cpu,
    server_exec_path, "-s", ip, "-p", port, "-r"
]
```

without embedded quotes.

### Mandatory validation

After launch, verify **actual** affinity of all relevant threads with:

```bash
ps -eLo pid,tid,psr,cls,rtprio,pri,comm | egrep 'run_server|franka_panda_client'
```

and optionally:

```bash
taskset -pc <pid>
cat /proc/<pid>/task/<tid>/status | grep Cpus_allowed_list
```

### Expected impact

High.

---

## 6.2 Immediate Fix B — Relax the gRPC server thread throttling

### Goal

Prevent starvation/queueing in the server.

### Proposed change

For the next test phase, remove or relax the most aggressive thread limits.

### Recommended temporary configuration

Option 1: conservative rollback

```cpp
// Remove these for now:
// builder.SetSyncServerOption(ServerBuilder::SyncServerOption::NUM_CQS, 1);
// builder.SetSyncServerOption(ServerBuilder::SyncServerOption::MIN_POLLERS, 1);
// builder.SetSyncServerOption(ServerBuilder::SyncServerOption::MAX_POLLERS, 2);
// grpc::ResourceQuota rq;
// rq.SetMaxThreads(4);
// builder.SetResourceQuota(rq);
```

Option 2: softer limit

```cpp
builder.SetSyncServerOption(ServerBuilder::SyncServerOption::NUM_CQS, 1);
builder.SetSyncServerOption(ServerBuilder::SyncServerOption::MIN_POLLERS, 2);
builder.SetSyncServerOption(ServerBuilder::SyncServerOption::MAX_POLLERS, 4);

grpc::ResourceQuota rq;
rq.SetMaxThreads(8);
builder.SetResourceQuota(rq);
```

### Expected impact

Medium to high.

### Note

This is not about maximizing throughput. It is about avoiding the self-inflicted case where one hot control RPC competes with a couple of long-lived or blocking RPC handlers in an overly constrained sync server.

---

## 6.3 Immediate Fix C — Remove avoidable work from the hot loop

### Goal

Reduce worst-case compute time in `franka_panda_client.cpp`.

### Proposed changes

#### a) Avoid dynamic Eigen allocations

Replace:

```cpp
Eigen::VectorXd force_vec(6);
Eigen::VectorXd torque_vec(NUM_DOFS);
```

with fixed-size objects:

```cpp
Eigen::Matrix<double, 6, 1> force_vec;
Eigen::Matrix<double, NUM_DOFS, 1> torque_vec;
```

#### b) Compute the Jacobian only when needed

At the moment the code computes `zeroJacobian()` every cycle before it is known whether cartesian soft-limit torque needs to be mapped.

Instead:

1. first evaluate whether cartesian soft-limit action is actually non-zero
2. only then compute Jacobian and map force to torque

This is especially valuable if cartesian safety feedback is rarely active.

#### c) Avoid repeated string/map work in the hot path if possible

The constraint bookkeeping is already better than before, but any nonessential diagnostic or map update should still be minimized in the control cycle.

### Expected impact

Medium.

---

## 6.4 Immediate Fix D — Separate control traffic from non-control traffic

### Goal

Prevent controller/state streaming activity from disturbing the hot control path.

### Proposed approaches

#### Minimum viable version

Use a **dedicated server instance or channel** for the control path if the architecture allows it.

#### Better version

Use separate channels for:

- 1 kHz `ControlUpdate`
- state streaming / logs / metadata
- controller management (`SetController`, `UpdateController`, `TerminateController`)

### Why

gRPC explicitly warns that active streams and connection-level concurrency limits can create queueing. Separating hot and non-hot traffic reduces cross-interference.

### Expected impact

Medium.

---

## 6.5 Immediate Fix E — Reduce lock contention between control and policy updates

### Goal

Ensure `ControlUpdate()` is not blocked by controller-management logic.

### Proposed change

Avoid doing heavy update work while holding the same mutex used by the 1 kHz path.

### Better design

- deserialize / prepare updates outside the hot lock
- use double-buffering or atomic pointer swap for controller parameters
- keep the lock scope as short as possible

### Risk addressed

This directly targets the symptom that failures are triggered when the policy/action stream starts.

### Expected impact

Medium to high.

---

## 6.6 Structural Fix F — Replace unary `ControlUpdate()` with streaming or in-process transport

### Goal

Remove repeated RPC initiation overhead from the 1 kHz loop.

### Recommended direction

Preferred order:

1. **bidirectional streaming RPC** for continuous state/torque exchange
2. callback/async gRPC API instead of sync server if gRPC must remain
3. shared memory / in-process control path if the server and robot client remain on the same host

### Why

The current design pays the cost of a fresh unary RPC every cycle. For a 1 kHz control loop this is an architectural liability even if local optimizations help.

### Expected impact

High, but requires more engineering work.

---

## 6.7 System-level hardening to verify in parallel

These are not code fixes, but they should be checked because Franka explicitly calls them out.

### Verify governor

I cannot confirm from the provided logs whether the CPU governor is already fixed to `performance`.

Check:

```bash
cpufreq-info | grep 'current policy'
```

If needed, set:

```bash
sudo systemctl disable ondemand
sudo systemctl enable cpufrequtils
sudo sh -c 'echo "GOVERNOR=performance" > /etc/default/cpufrequtils'
sudo systemctl daemon-reload && sudo systemctl restart cpufrequtils
```

### Verify build type

I cannot confirm from the provided logs whether the affected binaries are built with `Release` optimizations.

Check your CMake build flags and rebuild if needed.

### Run the official communication test

Beyond ping, also run the Franka `communication_test` because it stresses the path more realistically than ICMP alone.

---

## 7. Proposed Priority Order

### Phase 1 — do now

1. fix `launch_robot.py` affinity/env handling
2. verify actual thread affinity with `ps -eLo ...`
3. relax/remove gRPC thread throttling in `run_server.cpp`
4. retest without policy/action streaming
5. retest with policy/action streaming

### Phase 2 — do next

6. remove dynamic allocations and unconditional Jacobian work from the client hot loop
7. reduce lock contention in controller update paths
8. split control and non-control traffic

### Phase 3 — structural

9. migrate from unary `ControlUpdate` to a streaming or non-gRPC hot path

---

## 8. Concrete First Patch Set

If only one short patch round is possible, I recommend this exact first set:

### Patch 1

Fix `launch_robot.py` so affinity/env setup is actually correct.

### Patch 2

Rollback or soften the thread caps in `run_server.cpp`.

### Patch 3

In `franka_panda_client.cpp`:

- replace dynamic Eigen vectors with fixed-size types
- compute Jacobian only if cartesian soft-limit torque is non-zero
- keep the current fallback behavior

### Patch 4

Instrument timing for evidence instead of guessing. Add low-overhead counters/histograms for:

- server-side `ControlUpdate` duration
- client-side RPC latency
- consecutive missed cycles
- mutex wait time around `controller_mtx`

Do **not** print every cycle. Aggregate and print once per second or on shutdown.

---

## 9. Minimal Validation Plan

### Test A — affinity correctness

After launch, confirm:

- server RT thread really runs on CPU 4
- client RT thread really runs on CPU 5
- no unexpected migrations across non-isolated CPUs

### Test B — gRPC rollback effect

Compare before/after for:

- `control_command_success_rate`
- number of `ControlUpdate RPC failed`
- number of `communication_constraints_violation`
- worst-case observed RPC latency

### Test C — load interaction

Measure behavior in three conditions:

1. idle after fresh reboot
2. after machine has been running for a while
3. during policy + action streaming startup

### Success criterion

A meaningful improvement would be:

- no repeated reflex abort loop
- no persistent RPC fallback loop
- command success rate approaching 1.0
- stable operation during policy/action stream startup

---

## 10. Final Assessment

The current modifications are **not wrong overall**, but they are **insufficient and partially misdirected**:

- the server hot-path cleanups are good and should stay
- the launcher affinity setup is likely incorrect and must be fixed first
- the gRPC server thread minimization is probably too aggressive for this workload
- the client hot loop still does more work than desirable
- the unary-RPC control architecture remains the long-term weak point

In plain terms:

> The system is probably no longer losing on raw network latency. It is losing on host-side determinism.

That is annoying, of course. But at least now the enemy is in the room and not hiding in the Ethernet cable.

---

## 11. References

### Reviewed code

- `polymetis_server.cpp`
- `franka_panda_client.cpp`
- `run_server.cpp`
- `launch_robot.py`
- `real_time.hpp`
- `utils.h`

### Primary sources

1. Franka Control Interface — Troubleshooting  
   https://frankarobotics.github.io/docs/troubleshooting.html

2. gRPC Performance Best Practices  
   https://grpc.io/docs/guides/performance/

3. gRPC C++ channel/server argument keys  
   https://grpc.github.io/grpc/cpp/group__grpc__arg__keys.html

