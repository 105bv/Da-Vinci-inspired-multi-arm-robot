# DaVinci: understand, run, and control one simulated arm

This week's goal is **one simulated arm executing repeatable movements with feedback, completion checks, and failure handling**. Robot logic is C++17 on ROS 2 Jazzy. Gazebo Harmonic simulates the arm. Python in the launch file starts processes; it does not implement the robot's motion logic.

Use this README to understand the system and choose commands. Use [RUN_AND_TEST.md](RUN_AND_TEST.md) for the complete sequential acceptance procedure, including mock tests and deliberate failures. Both documents refer to the original custom arm, not the separate experimental Cartesian-target draft.

## Contents

- [1. What exists now](#1-what-exists-now)
- [2. How commands become movement](#2-how-commands-become-movement)
- [3. Correct setup, build, and launch instructions](#3-correct-setup-build-and-launch-instructions)
- [4. Command reference: inspect before moving](#4-command-reference-inspect-before-moving)
- [5. Run and understand the C++ sequence](#5-run-and-understand-the-c-sequence)
- [6. Choose your own joint-angle target](#6-choose-your-own-joint-angle-target)
- [7. Stop, test failures, and save evidence](#7-stop-test-failures-and-save-evidence)
- [8. Change the design or program](#8-change-the-design-or-program)
- [9. Learning exercises](#9-learning-exercises)
- [10. Troubleshooting](#10-troubleshooting)
- [11. Timeline and completion](#11-timeline-and-completion)

## 1. What exists now

| Capability | Current implementation |
|---|---|
| 3-DOF arm | `base_yaw`, `shoulder_pitch`, `elbow_pitch` |
| 4-DOF arm | Same three joints plus `wrist_pitch` |
| Movement | Predefined task coordinates converted to joint angles in C++; also accepts direct joint trajectories through the controller |
| Feedback | Simulated joint positions and velocities on `/joint_states` |
| Completion | Action result plus independent final-position check in the C++ client |
| Stop | C++ client's stop service requests cancellation while that client is running |
| Tool | Fixed test tool; no opening/closing gripper or physical grasp |
| Later work | Camera/vision, calibrated object coordinates, conveyor, physical motor interfaces, general collision-aware planning |

Teaching dimensions: pedestal height 0.12 m, upper arm 0.22 m, forearm 0.20 m, tool 0.06 m. These are temporary dimensions, not the team's CAD. Masses and 10 N m effort limits are placeholders. The simulation is not a validated actuator/payload model.

The sequence is: **home → approach pickup → lower → lift → transfer → lower at placement → retreat → home**. Scene markers show locations; completing this sequence does not mean an object was picked or sorted.

### Joint space versus task space

- **Joint space:** specify an angle for each joint, such as base yaw = 0.20 rad. This is what the trajectory controller accepts.
- **Task space:** specify a tool location, such as `(0.30, -0.14, 0.18)` metres in the model's world frame. Inverse kinematics (IK) converts that into joint angles.
- **Forward kinematics (FK):** use measured joint angles and geometry to calculate the tool location.

The existing C++ program has task coordinates in source code. **It has no runtime `target_xyz` parameter, keyboard jogging, or camera-target interface.** Direct joint commands below allow different angles without editing the source, but do not solve IK for arbitrary XYZ coordinates. Reach, joint limits, and obstacles constrain possible movement. Three or four arm DOF do not provide arbitrary six-dimensional tool poses.

## 2. How commands become movement

```text
motion_smoke_test (C++ task client)
  task coordinates → IK → joint/path checks → timed trajectory
                           |
                           v
/arm_controller/follow_joint_trajectory (ROS action)
                           |
                           v
arm_controller (JointTrajectoryController)
  interpolates joint setpoints over time
                           |
                           v
ros2_control hardware interface → Gazebo joints OR mock hardware
                           |
             +-------------+------------------+
             v                                v
joint_state_broadcaster                 controller action result
  /joint_states                               |
     |                                        |
     +→ C++ client checks feedback/completion ←+
     |
     +→ robot_state_publisher + URDF → /tf and /tf_static
```

The C++ client does not directly drive motors or implement a PID loop. This setup uses a position-command interface. Gazebo executes simulated joint commands; mock hardware mirrors them into feedback. Passing a mock test establishes interface behaviour, not physical tracking. The controller action supports execution monitoring; see the [official controller interface documentation](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html).

| Term | Meaning in this project |
|---|---|
| Package | A buildable unit such as `davinci_motion`; it is not necessarily a running process |
| Node | A running ROS participant, such as `/davinci_motion` or `/robot_state_publisher` |
| Topic | A stream of messages; `/joint_states` reports measurements |
| Service | A request/reply interface; `/davinci_motion/stop` acknowledges a stop request |
| Action | A longer operation with acceptance, feedback, result, and cancellation; used for movement |
| Parameter | A setting on a node, such as `cycles` |
| Launch argument | A setting used when starting a launch file, such as `backend:=gazebo` |
| Controller manager | Loads and updates controllers and connects them to hardware interfaces |
| TF | A tree of coordinate-frame transforms; available for the arm before any camera is added |

### Where the code lives

Links below are relative to this `software` folder so they work in the repository.

| Package/file | Responsibility |
|---|---|
| [davinci_description/urdf/arm.urdf.xacro](davinci_description/urdf/arm.urdf.xacro) | Links, frames, joint axes/limits, visuals, collisions, inertias, backend interfaces |
| [davinci_control/config/controllers_3dof.yaml](davinci_control/config/controllers_3dof.yaml) and [controllers_4dof.yaml](davinci_control/config/controllers_4dof.yaml) | Joint lists, controller type, command/state interfaces, tracking tolerances |
| [davinci_gazebo/worlds/one_arm_lab.sdf](davinci_gazebo/worlds/one_arm_lab.sdf) | Floor and task markers |
| [davinci_motion/include/davinci_motion/arm_model.hpp](davinci_motion/include/davinci_motion/arm_model.hpp) | Dimensions, home pose, FK/IK, waypoint sequence, limited clearance checks, duration calculation |
| [davinci_motion/src/motion_smoke_test.cpp](davinci_motion/src/motion_smoke_test.cpp) | ROS action client, feedback checks, timeouts, cancellation, CSV reports |
| [davinci_bringup/launch/sim.launch.py](davinci_bringup/launch/sim.launch.py) | Starts the selected model/backend, robot state publisher, bridge and controllers |
| [scripts/test_both_models.sh](scripts/test_both_models.sh) | Builds and checks both variants sequentially |

The Gazebo plugin provides the controller manager for the Gazebo backend. The mock launch starts `ros2_control_node` itself. Do not manually start an extra controller manager beside either launch.

## 3. Correct setup, build, and launch instructions

All commands run in **Ubuntu/Linux**

- Workspace: `/home/quanln/davinci_ws`
- Repository: `/home/quanln/davinci_ws/src/Da-Vinci-inspired-multi-arm-robot`
- Source packages: the repository's `software/` folder
- Build output: workspace `build/`, `install/`, and `log/`

### 3.1 Install once

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
```

`source` loads ROS into the current shell; it does not install or build your project. Skip installation if dependencies are already present. Stop if installation fails. Optional inspection tools used later:

```bash
sudo apt install ros-jazzy-tf2-ros ros-jazzy-tf2-tools ros-jazzy-rviz2
```

### 3.2 Discover, build, and test

Stop old simulations before rebuilding. Your already-running motion program shows you have applied at least part of the bundle; do not reapply the original patch as a troubleshooting step.

```bash
cd ~/davinci_ws
source /opt/ros/jazzy/setup.bash
colcon list --base-paths src/Da-Vinci-inspired-multi-arm-robot/software
```

Expect all five `davinci_*` packages. If one is absent, check its source directory and `package.xml` before proceeding.

```bash
colcon build \
  --base-paths src/Da-Vinci-inspired-multi-arm-robot/software \
  --packages-select davinci_description davinci_control davinci_gazebo davinci_motion davinci_bringup \
  --symlink-install --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
```

Expect five successfully built packages. Only after a successful build:

```bash
source ~/davinci_ws/install/setup.bash
ros2 pkg prefix davinci_bringup
ros2 pkg executables davinci_motion
colcon test --packages-select davinci_motion --event-handlers console_direct+
colcon test-result --test-result-base build/davinci_motion --verbose
```

Expect a workspace path, `davinci_motion motion_smoke_test`, and zero test failures/errors. The source contains 11 model test cases. `colcon build` compiles/installs; `source install/setup.bash` makes that installation discoverable; `colcon test` executes tests. An older installed binary is not proof that a failed build succeeded.

### 3.3 Set up EACH terminal

Use Terminal A for the simulator, B for motion/inspection, and C for inspection/stop during a run. Run this full block in **each** one:

```bash
source /opt/ros/jazzy/setup.bash
source ~/davinci_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=davinci_week1_manual
cd ~/davinci_ws
mkdir -p ~/davinci_ws/log/week1-manual
```

ROS domain 42 groups these ROS nodes; the Gazebo partition groups the simulator's transport. They are local test settings, not robot dimensions. Every manual-test terminal must match. If domain 42 is occupied by other work, choose another unused domain consistently. Sourcing Terminal A does not configure Terminal B.

### 3.4 Launch ONE model, then verify it

Choose **one** command in Terminal A.

For 3 DOF:

```bash
ros2 launch davinci_bringup sim.launch.py dof:=3 backend:=gazebo gui:=true
```

Or, for 4 DOF:

```bash
ros2 launch davinci_bringup sim.launch.py dof:=4 backend:=gazebo gui:=true
```

For mock control without a visible robot, substitute `backend:=mock`. For headless Gazebo, use `gui:=false`. Keep the launch terminal running and Gazebo unpaused.

In Terminal B, before sending any movement:

```bash
ros2 control list_controllers
timeout 10s ros2 topic echo /joint_states --once
```

Both `arm_controller` and `joint_state_broadcaster` must be active. Positions must be finite and include all joints for the selected model:

| Launched model | Required names in feedback |
|---|---|
| 3 DOF | `base_yaw`, `shoulder_pitch`, `elbow_pitch` |
| 4 DOF | All three above **and `wrist_pitch`** |

Order can differ; names identify joints. If the echo times out, stop here and diagnose. For Gazebo also check that simulation time advances:

```bash
timeout 10s ros2 topic echo /clock --once
```

Run it twice; the second timestamp should increase. No `/clock` is expected from the mock backend.

**To switch models:** stop the motion client, press Ctrl+C in Terminal A, wait for that launch to exit, launch the new model, then repeat the feedback check. `-p dof:=4` on the motion client cannot add a wrist to an existing 3-DOF simulation. Do not leave old mock or Gazebo launches running alongside the replacement.

## 4. Command reference: inspect before moving

These commands inspect a running system without requesting arm movement. Commands that stream output can be stopped with Ctrl+C; stopping an inspection command does not stop the robot.

### Discover processes and interfaces

| Command | What it tells you / when to use it |
|---|---|
| `ros2 node list` | Which ROS nodes are visible; use when a component seems missing |
| `ros2 node info /davinci_motion` | The client's publishers, subscribers, services, and actions; only while the executable is running |
| `ros2 topic list -t` | Available topics and message types |
| `ros2 action list -t` | Available actions; look for `/arm_controller/follow_joint_trajectory` |
| `ros2 action info /arm_controller/follow_joint_trajectory` | Action server/client discovery; not proof a movement succeeded |
| `ros2 service list -t` | Available services; the client's stop service exists only while that client is running |
| `ros2 pkg prefix davinci_motion` | Which installation ROS resolved; useful for stale/missing packages |
| `ros2 launch davinci_bringup sim.launch.py --show-args` | Supported launch arguments and defaults, without launching the arm |

### Inspect positions and controllers

| Command | Meaning |
|---|---|
| `ros2 control list_controllers` | Controller names/types and active/inactive state |
| `ros2 control list_hardware_interfaces` | Position command interfaces and position/velocity state interfaces; command interfaces should be claimed by the controller |
| `ros2 param get /arm_controller joints` | Joint list configured for the running controller |
| `ros2 topic echo /joint_states --once` | One measurement: names, joint positions, velocities, optional efforts |
| `ros2 topic hz /joint_states` | Observed message rate; leave it running briefly, then Ctrl+C |
| `ros2 topic info /joint_states --verbose` | Publishers/subscribers and QoS; useful for missing or conflicting feedback |
| `ros2 topic echo /arm_controller/controller_state --once` | Controller reference, feedback, and error information; inspect the installed message definition for exact fields |
| `ros2 interface show sensor_msgs/msg/JointState` | Explains the joint-state message fields |
| `ros2 interface show control_msgs/action/FollowJointTrajectory` | Explains the movement goal, result, and feedback fields |
| `ros2 interface show control_msgs/msg/JointTrajectoryControllerState` | Explains the controller-state message in your installed version |

Joint positions are **radians**, velocities rad/s. `0.20 rad ≈ 11.46°`; `π/2 rad = 90°`. An effort `.nan` does not cause this client's feedback failure because the client checks positions. A missing expected joint or non-finite position does.

`/joint_states` is measurement output. Publishing your own values there does not command Gazebo and can mislead the client and visualizations. Do not run a fake joint-state publisher alongside this controlled simulation.

### Inspect frames and the tool position

With the optional TF tools installed and Gazebo running:

```bash
ros2 run tf2_ros tf2_echo world tool_tip --ros-args -p use_sim_time:=true
```

This shows the tool frame expressed relative to `world`, calculated from the URDF and reported joint states. Translation is in metres. It does not command movement or measure the tool independently with a camera. For mock hardware use `use_sim_time:=false`. Stop with Ctrl+C. See the [ROS TF command tools](https://github.com/ros2/geometry2/blob/jazzy/tf2_ros/doc/cli_tools.rst).

Optional RViz inspection:

```bash
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```

Set Fixed Frame to `world`; add TF and RobotModel displays. For RobotModel use Description Source = Topic and Description Topic = `/robot_description` (Transient Local durability if required). This bundle does not include a preconfigured RViz layout. RViz visualizes the model and transforms; Gazebo runs physics. A view moving in RViz alone does not prove physical execution.

### Inspect parameters

While the C++ client is running:

```bash
ros2 param list /davinci_motion
ros2 param get /davinci_motion dof
ros2 param get /davinci_motion cycles
ros2 param get /davinci_motion use_sim_time
```

The client reads its custom settings at construction and stores them. **Restart it with new `--ros-args -p ...` values.** Do not expect `ros2 param set` during a run to change the stored DOF, cycle count, orientation requirement, or report path. Launch arguments also require relaunching; they are not live controls.

## 5. Run and understand the C++ sequence

Choose the command matching the already-launched model. First complete the controller/feedback checks in section 3.4.

### Three-joint Gazebo sequence

```bash
ros2 run davinci_motion motion_smoke_test --ros-args \
  -p dof:=3 -p cycles:=3 -p require_vertical:=false -p use_sim_time:=true \
  -p report_path:="$HOME/davinci_ws/log/week1-manual/gazebo-3dof.csv"
```

### Four-joint Gazebo sequence

```bash
ros2 run davinci_motion motion_smoke_test --ros-args \
  -p dof:=4 -p cycles:=3 -p require_vertical:=true -p use_sim_time:=true \
  -p report_path:="$HOME/davinci_ws/log/week1-manual/gazebo-4dof.csv"
```

For mock hardware set `use_sim_time:=false` and choose a `mock-3dof.csv` or `mock-4dof.csv` report name. Start learning with `cycles:=1`; use three cycles for the acceptance run.

### Understand each option

| Option | Meaning |
|---|---|
| `ros2 run davinci_motion motion_smoke_test` | Start one executable from the package; does not start the simulator |
| `--ros-args` | Following options are interpreted by ROS |
| `-p dof:=3` or `4` | Select the client's mathematical model and expected feedback names; must match launch |
| `-p cycles:=3` | Repeat the eight-waypoint sequence three times; allowed range 1–100 |
| `-p require_vertical:=true` | Reject this sequence if task-waypoint tool pitch is not downward within the model's 5° check; the 3-DOF sequence fails this check |
| `-p use_sim_time:=true` | Use ROS simulation time from `/clock`; use false for mock |
| `-p report_path:=...` | Write a CSV here; parent directory must exist and an existing file is overwritten |

The launch defaults are 3 DOF, Gazebo, GUI enabled. The client's defaults are 3 DOF, 3 cycles, `require_vertical=false`, and a report in the current directory. Use explicit values to avoid mixing models.

The 4-DOF IK already requests a downward task-waypoint pitch even with `require_vertical=false`; that option enables a preflight requirement check, not wrist activation. Neither setting adds a gripper or full tool roll/yaw control. Joint interpolation does not generally imply a straight tool path in XYZ.

### Read the result

| Output | Interpretation |
|---|---|
| `MOVE` | About to submit this waypoint; not proof it was accepted or completed |
| `DONE ... max_error=...` | Action succeeded and new final joint feedback is within 0.03 rad |
| `CYCLE 1/3 complete` | All eight waypoints of one cycle passed |
| `PASS` | Every requested cycle completed and the report was written |
| `FAIL` | Inspect the reason; do not call the run successful |

The client first waits up to 90 seconds for the action server, then up to 10 seconds for acceptable feedback. It treats feedback as fresh for 0.75 seconds of wall time. During execution it checks for stale feedback and timeouts. Those watchdogs use wall time even when `use_sim_time=true`; pausing Gazebo can therefore trigger a failure.

After completion:

```bash
tail -n 6 ~/davinci_ws/log/week1-manual/gazebo-4dof.csv
```

Use the matching report filename for 3 DOF. A three-cycle success has 24 waypoint data rows, `# completed_cycles=3`, and `# result=PASS`. The controller's configured goal tolerance is 0.02 rad; the client's extra final check is 0.03 rad. These are joint tolerances, not a guarantee of millimetre-level tool accuracy.

## 6. Choose your own joint-angle target

This is the exercise for understanding control beyond the predefined demo. A CLI action goal goes directly to `arm_controller`. It bypasses `ArmModel::inverse()`, the C++ path checks, watchdog, and CSV reporting. The controller still provides its own action result and configured tolerance checks. It is not an obstacle-aware planner.

**Exercise setup:** use the original unmodified model, start with a fresh mock launch at its initial home position, and make sure `motion_smoke_test` is not running. Inspect `/joint_states` first. These examples rotate only the base by 0.20 rad while keeping the other joints at home. After learning with mock hardware, repeat from a fresh Gazebo home state. Do not send these targets from an arbitrary edited pose and assume the intervening path has been checked.

Initial/home angles:

| Joint | Radians (rounded) | Degrees |
|---|---:|---:|
| `base_yaw` | 0.0 | 0 |
| `shoulder_pitch` | 1.134464 | 65 |
| `elbow_pitch` | -1.658063 | -95 |
| `wrist_pitch` (4 DOF only) | -1.047198 | -60 |

### 6.1 Three-joint example

Launch the 3-DOF mock model using section 3.4. In Terminal B:

```bash
ros2 action send_goal \
  /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  '{trajectory: {joint_names: [base_yaw, shoulder_pitch, elbow_pitch], points: [{positions: [0.20, 1.134464, -1.658063], time_from_start: {sec: 4, nanosec: 0}}]}, goal_time_tolerance: {sec: 3, nanosec: 0}}' \
  --feedback
```

### 6.2 Four-joint example

Stop the old launch, launch the 4-DOF mock model, and confirm `wrist_pitch` feedback first. Then:

```bash
ros2 action send_goal \
  /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  '{trajectory: {joint_names: [base_yaw, shoulder_pitch, elbow_pitch, wrist_pitch], points: [{positions: [0.20, 1.134464, -1.658063, -1.047198], time_from_start: {sec: 4, nanosec: 0}}]}, goal_time_tolerance: {sec: 3, nanosec: 0}}' \
  --feedback
```

### 6.3 Understand and modify the message

| Field | Meaning |
|---|---|
| `joint_names` | Names for this controller; all are required in the current configuration |
| `positions` | Target joint angles in radians, in exactly the same order as `joint_names` |
| `time_from_start` | Desired arrival time relative to trajectory start; 4 seconds here |
| `goal_time_tolerance` | Extra time permitted to satisfy the final controller tolerance; not a delay before movement |
| `--feedback` | Print action feedback while the goal executes |

Expect goal acceptance followed by a successful terminal status and `error_code: 0`. Acceptance alone is not completion. Check `/joint_states` afterward: base yaw should be near 0.20 rad. For this exercise, repeat the same command with the first position changed to `0.0` to return the base home. Try `sec: 8` instead of `sec: 4` and observe the timing change. Keep all other angles unchanged for this first exercise.

These position-only CLI examples do not request the same quintic endpoint interpolation used by the C++ client, which supplies position, zero velocity, and zero acceleration at both endpoints. The command format is described in the [ROS action CLI tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html).

One action goal is active at a time on this controller. Sending another can replace the current goal. Do not mix direct commands with the running demo. The `/davinci_motion/stop` service belongs to the C++ program and is **not a general stop service for these standalone CLI goals**. Do not assume Ctrl+C on the CLI guarantees that its accepted goal was cancelled. For this bounded simulation exercise let the short goal finish; stop the simulator launch if you need to end the simulation immediately.

## 7. Stop, test failures, and save evidence

### Stop a running C++ sequence

While the C++ client is executing, use a separately configured Terminal C:

```bash
ros2 service call /davinci_motion/stop std_srvs/srv/Trigger '{}'
```

The service response acknowledges the request. Verify that the motion client exits with code 2 and reports `# cancellation_confirmed=true`. It may record `# result=FAIL` because the requested cycles were intentionally interrupted. No later waypoint should start. Ctrl+C in the C++ client's terminal also requests its cooperative stop. This is simulation-level cancellation, not a hardware emergency stop.

### Automated checks

Stop manual launches and clients. From the repository root:

```bash
cd ~/davinci_ws/src/Da-Vinci-inspired-multi-arm-robot
bash software/scripts/test_both_models.sh mock
```

Only after that succeeds:

```bash
bash software/scripts/test_both_models.sh gazebo
```

The script builds, runs model tests, checks URDFs, runs three cycles for both models, requests cancellation, and checks the expected 3-DOF orientation rejection. It uses ROS domain 73 and its own Gazebo partition, and runs Gazebo headless. The script prints a timestamped results folder under `~/davinci_ws/log/`. Do not run two copies concurrently.

The script's stop check occurs after a fixed delay. Also perform a manual stop during observed movement. It does not automate feedback-loss testing. Follow [RUN_AND_TEST.md, section 7](RUN_AND_TEST.md#7-test-cancellation-and-feedback-loss) for the exact fault-test sequence, including broadcaster reactivation. Deactivating feedback intentionally causes a failure; restore it before further movement.

Save motion CSVs, launch logs, failure reasons, and a short screen recording. PASS means tested movement under these conditions, not complete hardware or grasp validation.

## 8. Change the design or program

| Desired change | Edit | Follow-up |
|---|---|---|
| Number of demo repetitions | `cycles` startup parameter | Restart only the motion executable |
| Switch 3/4 DOF | Both launch argument and client parameter | Restart simulator; verify names before motion |
| Choose a joint angle manually | Direct action goal in section 6 | Match all joints; inspect start state and motion path |
| Change pickup/place XYZ | `ArmModel::sequence()` in `arm_model.hpp` | Keep approach/lower/lift coordinates consistent; rebuild and rerun model tests |
| Change sequence order | `ArmModel::sequence()` | Recheck every connecting path; rebuild/test |
| Change home pose | `ArmModel::home()` and Xacro initial positions | Keep them consistent; rebuild and relaunch/test |
| Change lengths or axes | Xacro AND C++ kinematics | Update geometry, FK/IK, clearances, limits, targets, and tests together |
| Change controller tolerances | Correct DOF YAML; separate client threshold in C++ if justified | Understand actual tracking first; restart controllers through launch |
| Change movement speed | `ArmModel::duration()` in the C++ header | Recheck trajectory constraints; rebuild/test; there is no runtime speed parameter yet |
| Change task-marker positions | `one_arm_lab.sdf` | Keep motion targets consistent; relaunch Gazebo |
| Add a physical gripper | Model, controller, and motion/task logic | New work; opening/closing is not currently implemented |

The pitch joints use local axis `(0, -1, 0)` in this model, while base yaw uses `(0, 0, 1)`. Joint origins place joints relative to parent links. Visual size, collision geometry, inertial properties, and joint locations are different model fields; scaling a visual mesh alone does not update reach or kinematics.

The current clearance checker samples paths against the floor and approximate pedestal. It does not check mesh self-collision, arbitrary objects, conveyor geometry, or other arms. Replacing the arm with teammate CAD requires more than replacing its appearance.

### Rebuild after a C++ edit

Stop motion first. From a configured Ubuntu terminal:

```bash
cd ~/davinci_ws
colcon build \
  --base-paths src/Da-Vinci-inspired-multi-arm-robot/software \
  --packages-select davinci_motion \
  --symlink-install --event-handlers console_direct+ \
  --cmake-args -DBUILD_TESTING=ON
```

After success:

```bash
source ~/davinci_ws/install/setup.bash
colcon test --packages-select davinci_motion --event-handlers console_direct+
colcon test-result --test-result-base build/davinci_motion --verbose
```

Restart the client to load the new executable. For model, launch, world, or controller changes, use the full build in section 3.2 and restart the simulator too. `--symlink-install` does not eliminate the need to compile C++ or reload already-running nodes.

## 9. Learning exercises

Do these in order, recording a prediction, the command/change, and your observation:

1. **Discover:** launch mock hardware; identify the action server, broadcaster, and joint-state topic. Explain why no `/davinci_motion` node exists until you run its executable.
2. **Measure:** read joint feedback and convert one angle to degrees. Explain why it is a joint angle rather than tool XYZ.
3. **Command:** complete the base-only action exercise. Change 4 seconds to 8 seconds. Compare requested angles with returned positions.
4. **Locate:** use `tf2_echo world tool_tip`. Rotate the base and observe tool X/Y changing. Explain why a TF readout does not require computer vision.
5. **Trace:** read `sequence()` and then `execute()` in the C++ files. Identify preflight, goal submission, waiting, result checking, and CSV writing.
6. **Modify:** move pickup X from 0.30 to 0.28 m for approach, lower, and lift together. Update the scene marker if needed. Rebuild, test, then inspect the result.
7. **Diagnose:** request the known 3-DOF vertical sequence, observe its intentional rejection, and explain what the wrist adds. Test cancellation and feedback loss using the run guide.

You understand this week's foundation when you can choose a command for a purpose, predict its effect, locate the implementation, and explain a failure. Copying a successful command is only the first step. Full IK derivation, PID tuning, camera calibration, and MoveIt integration can follow as their tasks arise.

## 10. Troubleshooting

| Symptom | Explanation and next check |
|---|---|
| Package not found | Source both setup files in this terminal; check `colcon list`, build success, and `ros2 pkg prefix` |
| `No fresh joint feedback` | After finding the action server, the client did not receive acceptable fresh feedback; check all expected names and finite positions, broadcaster state, and running simulation |
| 4-DOF client but only three names | Restart launch with `dof:=4`; require `wrist_pitch` in feedback before retrying |
| `Joint feedback watchdog expired` | Feedback stopped/stalled during execution; check paused/slow simulation or missing broadcaster |
| Controllers active but robot does not move | Active is a lifecycle state, not proof physics is advancing or a goal was sent; check clock, feedback, and action result |
| Action server unavailable | Check active controller, action name, matching domain, and launch log |
| Unknown joints/rejected goal | Match `joint_names` to `/arm_controller` parameter and provide a position for each |
| Stop service missing | It exists only while the C++ client runs; it is not a permanent controller service |
| `.nan` effort | Unreported effort is not the same as invalid position; this client checks positions |
| `A message was lost` | One notice does not explain a missing wrist; persistent loss warrants checking rate/load/publishers |
| Cannot write report | Create the parent directory; use a writable filename |
| Tolerance failure | Preserve the exact error and inspect tracking; do not increase tolerances merely to obtain PASS |
| Edited code seems unchanged | Check build success, source the intended workspace, inspect package prefix, restart affected processes |
| Conflicting nodes/feedback | Stop the specific older launch; inspect topic publishers; avoid broadly killing unrelated processes |

## 11. Timeline and completion


| Period | Planned work | Evidence |
|---|---|---|
| Current September milestone | Both arm models; C++ motion, feedback, result checks, simulation | Checklist above; preferred arm configuration selected |
| Rest of September | Vision demo; fixed pickup versus calibrated coordinates; USB/serial and one-servo tests alongside mechanical work | Detection output, pickup arrangement, command/status exchange, servo test |
| October Week 1 | Connect gripper, vision, conveyor, and simple task state machine; full cycle in simulation | Stop conveyor → detect → pick → place → clear arm → resume |
| October Week 2 | Finalize electrical/software interfaces; calibrate joint directions, home, limits, pickup/place points | Physical fixed-position pick-and-place |
| October Week 3 | Integrate real camera, conveyor, object sensor; test failures/repeatability; begin poster/slides | Measured complete-cycle success and timing; draft presentation |
| October Week 4 | Freeze demo features; finalize poster, slides, results | Finished materials and backup demo video |
| November 1–12 | Final tests, remaining fixes, rehearsals | Reliable demonstration and explained limitations |
| November 13, 2026 | Presentation | One-arm prototype and measured results |

Multi-arm coordination remains the longer-term architecture. This prototype focuses on one reliable arm first.


### This week's evidence

- [ ] Five packages build; model tests have zero failures/errors.
- [ ] Both mock variants and both Gazebo variants pass their three-cycle automated runs.
- [ ] Four-joint feedback is confirmed for the four-DOF test.
- [ ] Successful motion reports contain 24 waypoint rows and `# result=PASS`.
- [ ] Active-motion cancellation and feedback-loss handling are checked for each model.
- [ ] The 3-DOF vertical task rejects before movement; the 4-DOF sequence completes.
- [ ] Reports and a demo recording are saved.
- [ ] You can command a small joint movement and explain the feedback/result yourself.

**Verification status:** local structural/syntax checks are separate from execution. The task cannot access WSL (`Wsl/Service/E_ACCESSDENIED`), so the assistant has not compiled C++ or run ROS/Gazebo there. The user's pasted output confirmed active controllers and three-joint feedback. Update this checklist only with actual results; newly documented commands are not claimed as runtime-tested.

### Further references

- [C++ ROS action client](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html)
- [Trajectory representation and interpolation](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/trajectory.html)
- [Gazebo/ros2_control integration](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)
