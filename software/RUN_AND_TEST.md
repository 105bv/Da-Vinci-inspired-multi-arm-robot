# Run and test the original DaVinci arm

Use this guide for the original C++ one-arm bundle: ROS 2 Jazzy, Gazebo Harmonic, and selectable 3/4 DOF. It does not use a replacement commercial arm or the separate experimental target-control draft.

For the system explanation, command reference, direct joint-angle exercises, and file-editing map, read [README.md](README.md). This file is the sequential run and acceptance checklist. Both documents use the same manual terminal setup and model-switching checks.

Workspace: `/home/quanln/davinci_ws`  
Repository: `/home/quanln/davinci_ws/src/Da-Vinci-inspired-multi-arm-robot`

All commands below run in **Ubuntu WSL**, not PowerShell. Copy commands inside code blocks; underscores are ordinary underscores. Run one section at a time and check its expected result before continuing.

## What the demonstration does

The C++ program computes joint targets for a predefined sequence: home, approach pickup, lower, lift, transfer, lower at placement, retreat, home. It sends timed trajectories to `arm_controller`, checks fresh joint positions and completion, and writes a CSV report.

The 3-DOF arm has `base_yaw`, `shoulder_pitch`, and `elbow_pitch`. The 4-DOF arm adds `wrist_pitch`. Gripper opening is not counted here; this model has a fixed test tool. There is no physical grasp, computer vision, conveyor control, or real motor control in this demonstration. Passing the tests demonstrates simulated arm movement, not completed object sorting.

## 0. Start clean and confirm the source files

Stop earlier manual launches using Ctrl+C in their original terminals, and wait for them to exit. Close the corresponding Gazebo windows. Do not leave a mock launch and Gazebo launch running together for the manual tests. Do not run multiple motion clients together.

Your recent error already shows that the new motion executable runs. **Do not apply the original patch again just to fix joint feedback.** First check your existing files:

```bash
cd ~/davinci_ws/src/Da-Vinci-inspired-multi-arm-robot
git status --short
ls software/davinci_bringup/launch/sim.launch.py
ls software/davinci_description/urdf/arm.urdf.xacro
ls software/davinci_motion/src/motion_smoke_test.cpp
ls software/davinci_motion/include/davinci_motion/arm_model.hpp
ls software/scripts/test_both_models.sh
```

Expected: all five paths exist. Local edits in `git status` are normal after applying the bundle; preserve them. If a required path is missing, stop and resolve the missing file before building. Do not overwrite your repository or delete its build directories as an initial troubleshooting step.

## 1. Install dependencies once

Skip installation if these dependencies are already installed. This is the Jazzy/Gazebo Harmonic setup, not Gazebo Classic.

```bash
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install \
  build-essential python3-colcon-common-extensions liburdfdom-tools \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-rclcpp-action ros-jazzy-control-msgs \
  ros-jazzy-trajectory-msgs ros-jazzy-sensor-msgs ros-jazzy-std-srvs \
  ros-jazzy-ament-cmake-gtest \
  ros-jazzy-ament-lint-auto ros-jazzy-ament-lint-common

echo "$ROS_DISTRO"
ros2 pkg prefix gz_ros2_control
ros2 pkg prefix ros_gz_sim
gz sim --versions
```

Expected: `jazzy`, two valid package paths, and a Gazebo Sim version. If an installation command fails, resolve its error before continuing.

## 2. Build and run the C++ model tests

Use an Ubuntu terminal with no simulation running:

```bash
cd ~/davinci_ws
source /opt/ros/jazzy/setup.bash
colcon list --base-paths src/Da-Vinci-inspired-multi-arm-robot/software
```

Expected: the five packages `davinci_description`, `davinci_control`, `davinci_gazebo`, `davinci_motion`, and `davinci_bringup` are listed.

```bash
colcon build \
  --base-paths src/Da-Vinci-inspired-multi-arm-robot/software \
  --packages-select davinci_description davinci_control davinci_gazebo davinci_motion davinci_bringup \
  --symlink-install --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
```

Expected: all five packages finish successfully. If compilation fails, stop and retain the first compiler error. Do not treat an older installed executable as proof that this build worked.

After a successful build:

```bash
source ~/davinci_ws/install/setup.bash
ros2 pkg prefix davinci_bringup
ros2 pkg executables davinci_motion
colcon test --packages-select davinci_motion --event-handlers console_direct+
colcon test-result --test-result-base build/davinci_motion --verbose
```

Expected: a workspace installation path, `davinci_motion motion_smoke_test`, and no test failures or errors. The prepared source contains 11 C++ model cases; the aggregate colcon count may include other test bookkeeping. These cases check the mathematical model, not Gazebo contact physics.

## 3. Set up every manual-test terminal

Open **Terminal A** for the simulator and **Terminal B** for commands. Later, Terminal C is used for fault tests.

Run this entire block in **each terminal** before the manual tests:

```bash
source /opt/ros/jazzy/setup.bash
source ~/davinci_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=davinci_week1_manual
cd ~/davinci_ws
mkdir -p ~/davinci_ws/log/week1-manual
```

Domain 42 is a chosen local test domain, not a project requirement. If it is already used by another project, choose another unused domain consistently in every terminal. The matching Gazebo partition keeps these manual launches together. These settings last only for the current shell.

Sourcing Terminal A does not configure Terminal B. Package-not-found errors in a new terminal usually require repeating this setup block there.

## 4. Check both models with mock hardware first

Mock hardware checks the ROS interfaces by mirroring position commands into feedback. It has no visible Gazebo robot and does not validate physical dynamics.

### 4a. Three joints

Terminal A:

```bash
ros2 launch davinci_bringup sim.launch.py dof:=3 backend:=mock
```

Leave Terminal A running. In Terminal B:

```bash
ros2 control list_controllers
timeout 10s ros2 topic echo /joint_states --once
```

Expected: `arm_controller` and `joint_state_broadcaster` both active; the message contains positions for `base_yaw`, `shoulder_pitch`, and `elbow_pitch`. Joint ordering may differ and is harmless. If the echo command times out or positions are missing/non-finite, do not start motion.

Terminal B:

```bash
ros2 run davinci_motion motion_smoke_test --ros-args \
  -p dof:=3 -p cycles:=1 -p use_sim_time:=false \
  -p report_path:="$HOME/davinci_ws/log/week1-manual/mock-3dof.csv"
```

Expected: `MOVE`, `DONE`, then `PASS`. Stop Terminal A with Ctrl+C and wait for the launch to finish before changing models.

### 4b. Four joints

Terminal A:

```bash
ros2 launch davinci_bringup sim.launch.py dof:=4 backend:=mock
```

Terminal B:

```bash
ros2 control list_controllers
timeout 10s ros2 topic echo /joint_states --once
```

Expected: both controllers active and **all four** position entries, including `wrist_pitch`.

Only after that check passes:

```bash
ros2 run davinci_motion motion_smoke_test --ros-args \
  -p dof:=4 -p cycles:=1 -p require_vertical:=true -p use_sim_time:=false \
  -p report_path:="$HOME/davinci_ws/log/week1-manual/mock-4dof.csv"
```

Expected: `PASS`. Stop Terminal A with Ctrl+C and wait for it to exit before starting Gazebo.

## 5. Run the 3-DOF arm in Gazebo

Terminal A:

```bash
ros2 launch davinci_bringup sim.launch.py dof:=3 backend:=gazebo gui:=true
```

Wait for the robot to spawn and both controllers to activate. Keep simulation running, not paused. If the GUI is unavailable, `gui:=false` runs the same server without a visible window.

Terminal B:

```bash
ros2 control list_controllers
timeout 10s ros2 topic echo /joint_states --once
timeout 10s ros2 topic echo /clock --once
```

Expected: both controllers active; the three expected joints have finite positions; a clock message arrives. To check that time advances, run the clock command again and compare its value. A single clock message alone does not prove time continues advancing.

Then run:

```bash
ros2 run davinci_motion motion_smoke_test --ros-args \
  -p dof:=3 -p cycles:=3 -p use_sim_time:=true \
  -p report_path:="$HOME/davinci_ws/log/week1-manual/gazebo-3dof.csv"
```

Expected: eight waypoints per cycle, three completed cycles, and final `PASS`. This run permits tool tilt; do not add `require_vertical:=true` to this successful 3-DOF test.

After motion completes:

```bash
tail -n 5 ~/davinci_ws/log/week1-manual/gazebo-3dof.csv
```

Expected: `# completed_cycles=3` and `# result=PASS`. Keep this report. Reusing a report filename overwrites its previous contents.

### Optional expected-rejection test while using 3 DOF

With no other motion client running:

```bash
ros2 run davinci_motion motion_smoke_test --ros-args \
  -p dof:=3 -p cycles:=1 -p require_vertical:=true -p use_sim_time:=true \
  -p report_path:="$HOME/davinci_ws/log/week1-manual/vertical-rejection-3dof.csv"
```

Expected: failure before movement explaining that this 3-DOF sequence cannot maintain the vertical tool, with exit code 1. This intentional rejection is a passed negative test, not a failed installation. This is a limitation of the specified sequence and arm; it is not a claim that a 3-DOF arm can never point a tool downward at any position.

## 6. Stop 3 DOF, then run 4 DOF in Gazebo

**Terminal A: press Ctrl+C and wait for the previous launch to exit.** Do not merely change the motion command in Terminal B. A motion parameter cannot add a wrist to a robot already spawned in Gazebo.

Terminal A:

```bash
ros2 launch davinci_bringup sim.launch.py dof:=4 backend:=gazebo gui:=true
```

Terminal B:

```bash
ros2 control list_controllers
timeout 10s ros2 topic echo /joint_states --once
timeout 10s ros2 topic echo /clock --once
```

Before proceeding, verify that both controllers are active and the message contains finite positions for:

```text
base_yaw
shoulder_pitch
elbow_pitch
wrist_pitch
```

Their order does not matter. **If `wrist_pitch` is absent, stop here.** Your previous output contained only three joints, which explains the four-DOF motion failure. Inspect the launch log and check that the old launch has stopped. If the new launch still produces only three joints, capture these diagnostics instead of repeatedly running motion:

```bash
ros2 node list
ros2 topic info /joint_states --verbose
ros2 param get /arm_controller joints
ros2 pkg prefix davinci_description
ros2 pkg prefix davinci_bringup
```

Once four-joint feedback is confirmed, Terminal B:

```bash
ros2 run davinci_motion motion_smoke_test --ros-args \
  -p dof:=4 -p cycles:=3 -p require_vertical:=true -p use_sim_time:=true \
  -p report_path:="$HOME/davinci_ws/log/week1-manual/gazebo-4dof.csv"
```

Expected: three completed cycles and `PASS`. The wrist supports the downward tool pitch at the task waypoints. This is not a guarantee that every transition, including the home transition, maintains a constant Cartesian orientation.

```bash
tail -n 5 ~/davinci_ws/log/week1-manual/gazebo-4dof.csv
```

Expected: `# completed_cycles=3` and `# result=PASS`.

## 7. Test cancellation and feedback loss

Keep the 4-DOF Gazebo launch running. Set up Terminal C using the full block in section 3. Only one motion client should run at a time. These are simulator tests; the stop service is not a physical emergency stop.

### 7a. Request cancellation during movement

Terminal B:

```bash
ros2 run davinci_motion motion_smoke_test --ros-args \
  -p dof:=4 -p cycles:=100 -p require_vertical:=true -p use_sim_time:=true \
  -p report_path:="$HOME/davinci_ws/log/week1-manual/stop-4dof.csv"
```

Wait until Terminal B reports a `MOVE` and the arm is moving. Then Terminal C:

```bash
ros2 service call /davinci_motion/stop std_srvs/srv/Trigger '{}'
```

Expected: a stop acknowledgement, motion program termination with exit code 2, no further waypoint execution, and this report entry:

```bash
tail -n 6 ~/davinci_ws/log/week1-manual/stop-4dof.csv
```

```text
# result=FAIL
# error=Stop requested
# cancellation_confirmed=true
```

Here `FAIL` means the requested 100 cycles were deliberately interrupted. The cancellation test passes only when the stop is confirmed. Stop the simulator and inspect the logs if cancellation cannot be confirmed.

### 7b. Interrupt joint feedback

Wait for the previous motion client to exit. Terminal B:

```bash
ros2 run davinci_motion motion_smoke_test --ros-args \
  -p dof:=4 -p cycles:=100 -p require_vertical:=true -p use_sim_time:=true \
  -p report_path:="$HOME/davinci_ws/log/week1-manual/feedback-loss-4dof.csv"
```

During a `MOVE`, Terminal C:

```bash
ros2 control switch_controllers --deactivate joint_state_broadcaster --strict
```

Expected: within approximately the 0.75-second wall-time watchdog plus scheduling delay, the client detects stale feedback, attempts cancellation, and does not advance to another waypoint. If feedback is lost between moves, a missing-feedback timeout can be reported instead. Confirm the cancellation outcome in the log/report.

After the motion client exits, restore the broadcaster:

```bash
ros2 control switch_controllers --activate joint_state_broadcaster --strict
timeout 10s ros2 topic echo /joint_states --once
```

If cancellation was not confirmed, restart the simulator before further motion. A later successful three-cycle run verifies recovery. Repeat these two fault tests on the 3-DOF launch if collecting complete manual acceptance evidence: use `dof:=3`, `require_vertical:=false`, and different report filenames.

## 8. Run the supplied automated checks

Stop all manual launches and clients first. Run the following sequentially in an Ubuntu terminal; do not run both scripts at once:

```bash
cd ~/davinci_ws/src/Da-Vinci-inspired-multi-arm-robot
bash software/scripts/test_both_models.sh mock
```

Only if that succeeds:

```bash
bash software/scripts/test_both_models.sh gazebo
```

The script builds, runs model tests, checks generated URDFs, runs three cycles for each DOF, checks stop requests, and checks the expected 3-DOF vertical-task rejection. Gazebo runs headless. It uses ROS domain 73 and its own Gazebo partition inside the script, so the manual domain-42 terminals do not see these nodes.

Expected final lines:

```text
PASS: both mock models completed motion, stop, and orientation checks.
PASS: both gazebo models completed motion, stop, and orientation checks.
```

Each line appears at the end of its respective run. The script prints the results directory under `~/davinci_ws/log/week1-<timestamp>-<backend>/`. Preserve `run.log`, `launch-3dof.log`, `launch-4dof.log`, motion CSVs, and stop reports. The script does not automate the feedback-loss test in section 7b; do that separately.

The current script requests cancellation after a fixed delay. Use the manual cancellation test during visibly active motion as additional evidence that an in-progress trajectory can be cancelled. If the script exits early, inspect its printed results directory and first failure before proceeding. It has a 360-second limit per normal three-cycle motion run; a timeout is not a pass.

## 9. Completion checklist

- [ ] All five packages build successfully in WSL.
- [ ] C++ model tests report zero failures/errors.
- [ ] Mock 3/4 DOF pass.
- [ ] Gazebo 3/4 DOF each complete three cycles, with 24 waypoint data rows and `# result=PASS`.
- [ ] Each successful final joint-position error is at most 0.03 rad (about 1.72 degrees).
- [ ] Cancellation is confirmed for each model during an active movement.
- [ ] The 3-DOF vertical task is rejected before movement; the 4-DOF task completes.
- [ ] Feedback loss prevents subsequent waypoints; cancellation outcome is checked and feedback restored.
- [ ] Visual inspection shows sensible motion with the simulated arm and scene.
- [ ] Reports and a short demo video are saved.

These are acceptance checks, not a record of completed tests. The 0.03-rad check is a joint-space criterion; it does not establish a particular millimetre-level gripper accuracy.

## Troubleshooting by symptom

| Symptom | Meaning and next action |
|---|---|
| `Package ... not found` | Source both setup files in that terminal. If still missing, inspect `colcon list` and the build result. |
| `No fresh joint feedback` | The client found the action server but did not accept fresh feedback within 10 seconds. Check model DOF, expected joint names, finite positions, broadcaster, and whether simulation is running. |
| Three joints published while the client uses four | Stop the old launch and relaunch with `dof:=4`; require `wrist_pitch` in feedback before motion. |
| Controller active but no joint messages | Active status alone does not establish running physics. Check simulation pause, clock progress, and Gazebo/plugin logs. |
| `Joint feedback watchdog expired` | Feedback became stale during execution. Check simulation stalls and publisher availability; do not disable the check to hide the problem. |
| `Action server unavailable` | Check controller activation, names, and matching ROS domains. |
| Cannot write report | Create its parent folder and verify the filename is writable. |
| `.nan` in effort only | This client uses positions; missing effort values alone do not cause its fresh-feedback error. Non-finite positions do. |
| A single `A message was lost` notice | It does not explain a missing wrist joint. If loss persists, inspect `/joint_states` rate and system load. |
| GUI missing or slow | Try `gui:=false` to separate GUI problems from server/control problems. |
| Goal/path tolerance failure | Preserve the exact error and launch log; investigate tracking before loosening tolerances. |
| Old or conflicting nodes remain | Stop the specific prior launch in its terminal. Inspect `ros2 node list` and topic publishers; do not broadly kill unrelated processes. |

## What was verified by the assistant

Local checks can validate Xacro expansion for both DOFs/backends, controller-joint consistency, the link tree, inertial values, XML/SDF parsing, launch syntax, and shell-script syntax. They cannot prove ROS/Gazebo execution.

On September 20, 2026, the task's WSL command still returned `Wsl/Service/E_ACCESSDENIED`. No C++ build, C++ test execution, or Gazebo run was performed by the assistant in your WSL workspace. Your pasted output proves active controllers and three-joint feedback at that moment; it does not yet prove four-joint feedback or successful movement cycles. Keep runtime results separate from the local structural checks.
