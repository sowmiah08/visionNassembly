# Dual-Arm Assembly Workcell — What's Been Built

This documents everything added to `so101_description` to turn your single SO-101
robot model into a dual-arm assembly workcell, for the *Active 3D Vision-Guided
Dual-Arm Assembly with Autonomous Failure Recovery* project. It covers what each
file does, the decisions behind them, how to launch and verify the result, and
what's intentionally left for later.

Everything here is **new, additive work**. Your original single-arm model
(`so101_new_calib.urdf`) and the dual-arm setup that already existed
(`dual_arm_final.urdf.xacro`, `dual_arm_gazebo.urdf.xacro`,
`so101_left_arm.urdf.fragment`, `so101_right_arm.urdf.fragment`) were never
modified — they still work exactly as they did before, as a fallback/reference.

## File guide

```
so101_description/
├── urdf/
│   ├── so101_new_calib.urdf                  original single-arm model (untouched)
│   ├── so101_new_calib_camera.urdf           single-arm + wrist camera (pre-existing)
│   ├── so101_left_arm.urdf.fragment          pre-existing dual-arm fragments (untouched)
│   ├── so101_right_arm.urdf.fragment
│   ├── dual_arm_final.urdf.xacro             pre-existing dual-arm display setup (untouched)
│   ├── dual_arm_gazebo.urdf.xacro            pre-existing dual-arm Gazebo setup (untouched)
│   ├── so101_scene.urdf.xacro                pre-existing single-arm + table scene (untouched)
│   │
│   ├── so101_left_arm_camera.urdf.fragment   left arm + wrist camera
│   ├── so101_right_arm_camera.urdf.fragment  right arm + wrist camera (wrist_roll fixed, see below)
│   ├── assembly_objects.urdf.xacro           reusable macros for peg/plate/fixture/tray/plug/socket
│   ├── dual_arm_workcell.urdf.xacro          active workcell for RViz (plug + socket only)
│   ├── dual_arm_workcell_gazebo.urdf.xacro   active workcell for Gazebo Harmonic (plug + socket only)
│   ├── dual_arm_workcell_peghole.urdf.xacro         preserved snapshot for RViz: fixture/plate/peg/tray + plug/socket
│   └── dual_arm_workcell_peghole_gazebo.urdf.xacro  same, for Gazebo Harmonic
│
├── meshes/
│   ├── generate_plate_with_hole.py           procedural mesh generator (see below)
│   ├── base_plate_with_hole.stl              generated output
│   ├── generate_socket_cell.py               procedural mesh generator (see below)
│   └── socket_cell_blind_hole.stl            generated output
│
├── models/
│   ├── peg.sdf                               standalone, freely-movable peg (Gazebo only)
│   └── plug.sdf                              standalone, freely-movable 4-pin plug (Gazebo only)
│
├── so101_description/
│   └── ground_truth_pose_bridge.py           custom node: real object poses -> ROS 2 (see below)
│
├── rviz/workcell_cameras.rviz                saved RViz config, RobotModel + 3 camera Image displays
│
├── launch/
│   ├── so101_display.launch.py               pre-existing (untouched)
│   ├── so101_display_camera.launch.py        pre-existing (untouched)
│   ├── so101_scene.launch.py                 pre-existing (untouched)
│   ├── dual_setup.launch.py                  pre-existing (untouched)
│   ├── dual_setup_gazebo.launch.py           pre-existing (untouched)
│   ├── workcell_display.launch.py            active workcell in RViz
│   ├── workcell_gazebo.launch.py             active workcell in Gazebo Harmonic (also opens RViz)
│   ├── workcell_peghole_display.launch.py    preserved peghole snapshot in RViz
│   └── workcell_peghole_gazebo.launch.py     preserved peghole snapshot in Gazebo Harmonic
│
├── config/controllers.yaml                   pre-existing, reused unchanged
├── worlds/test_world.sdf                      pre-existing, reused unchanged
└── assets/*.stl, *.part                       pre-existing meshes + CAD source (untouched)
```

One dead file was found and removed: `urdf/so101_prefixed.urdf.xacro` — an
unfinished template (literally contained placeholder comments like "copy
everything from so101_new_calib base_link here") with zero references
anywhere in the package. The `.part` CAD export files in `assets/` are kept
on purpose — ROS never reads them, but they're the original onshape-to-robot
source data in case you ever need to regenerate meshes.

## The dual arm

`so101_left_arm.urdf.fragment` / `so101_right_arm.urdf.fragment` already
existed: each is a copy of your single-arm model with every link, joint, and
transmission renamed with a `left_`/`right_` prefix, so two instances can
coexist in one URDF without name collisions. `so101_left_arm_camera` /
`so101_right_arm_camera` are the same thing with a wrist camera added.

**Bug found and fixed**: `right_wrist_roll`'s joint origin had a pitch of
`-1.34385` baked in, while `left_wrist_roll` and the original single-arm model
both use `0.0486795` — every other joint in the right arm fragment is
numerically identical to the left arm's, so this was an isolated stray value,
not a deliberate per-robot calibration. It was corrected to `0.0486795` in
`so101_right_arm_camera.urdf.fragment` only (the pre-existing, non-camera
`so101_right_arm.urdf.fragment` was left untouched, per "preserve originals").
This is also what was causing the right wrist to visually flicker/misalign.

## The workbench

`dual_arm_workcell(_gazebo).urdf.xacro` build an industrial aluminum-extrusion
bench instead of the original wood-plank table:

- **Tabletop**: dark non-slip surface with a brushed-aluminum edge trim band
- **Legs**: 4 square aluminum-extrusion posts (not cylinders) at the corners
- **Lower rail frame + shelf**: a rail rectangle near the floor carrying a
  dark storage shelf (replaces the old "upper shelf + 2 pillars")
- **Overhead camera gantry**: two aluminum posts rising from the tabletop,
  connected by a crossbar spanning the whole bench, with the overhead camera
  hanging from a bracket at the center — gives a stable, centered, symmetric
  look and a better vantage point than the old single cantilevered pipe mount

Both arms mount to `table_link` via fixed joints (`table_to_left_robot` /
`table_to_right_robot`), positioned at the two long edges facing the shared
central area.

## Assembly objects (`assembly_objects.urdf.xacro`)

Six reusable xacro macros, each producing one link + the fixed joint that
attaches it to a parent:

| Macro | What it is |
|---|---|
| `peg(name, parent, xyz, rpy, radius, length, material)` | A cylindrical peg (primitive geometry) |
| `base_plate_with_hole(name, parent, xyz, rpy)` | Rectangular plate with a **real cylindrical hole** (see below) |
| `assembly_fixture(name, parent, xyz, rpy, ...)` | Shallow 5-box rimmed platform that locates the plate |
| `parts_tray(name, parent, xyz, rpy, ...)` | Open 5-box tray for spare parts |
| `plug(name, parent, xyz, rpy, body_x/y/z, pin_radius, pin_length, pitch_x/y, ...)` | Connector body with 4 cylindrical pins in a 2x2 pattern (primitive geometry) |
| `socket(name, parent, xyz, rpy, material)` | Matching block with 4 **real blind pockets** (see below) |

Instantiated in the workcell: one `assembly_fixture` holding one
`base_plate_with_hole` in the central area, a loose `peg` beside it sized to
fit the plate's hole (9mm peg radius vs 12mm hole radius — clear fit), one
`parts_tray` to the side holding two smaller `spare_peg` instances lying on
their side, one `socket` fixed to the table, and one `plug` resting loose
nearby (pins up, ready to be picked up and inserted).

**Why a generated mesh, not primitives**: URDF has no boolean subtraction, so
a box with a real hole can't be built from `<box>`/`<cylinder>` primitives
alone. `meshes/generate_plate_with_hole.py` builds the plate as an explicit
triangle mesh (no external CAD library needed) by lofting between the plate's
rectangular outer boundary and the hole's circular inner boundary for the top
and bottom faces, plus the outer side walls and the inner (hole) cylindrical
wall. It self-validates before writing the STL: every edge must be shared by
exactly two triangles (watertight-closed-mesh check) and the enclosed volume
is checked against the analytic expectation (matched to ~0.8%, the remaining
gap being ordinary polygon-discretization error from approximating the
rectangle's corners and the circle). Re-run it any time with
`python3 meshes/generate_plate_with_hole.py` if you want different
dimensions (edit the constants at the top of the file).

The plate and fixture stay fixed-jointed into the one big model (deliberately
static). The peg does not: in the Gazebo workcells it's a separate, freely
movable spawned object instead, so it can actually be picked up — see
"Pickable objects" below. The `peg` xacro macro itself is unchanged and is
still what's used in the RViz-only display variants, where there's no
physics/spawning concept and "pickable" doesn't apply.

### 4-pin plug and socket

The plug is pure primitives (a box body + 4 cylinders for the pins — a
*protruding* pin needs no custom mesh, unlike a hole). The socket's pockets
do need one, and this time they're genuinely **blind** (closed bottom), not
through-holes, for better visual and physical realism than the single-peg
plate: `meshes/generate_socket_cell.py` builds one 0.03 x 0.03 x 0.025 m
"cell" with one pocket, reusing the plate generator's boundary-lofting
technique for the top face and the pocket wall, plus new flat-fan
triangulation for the solid bottom and the pocket's floor. The full
0.06 x 0.06 m, 4-pocket socket is just **four untouched copies of that same
mesh** tiled at the four quadrant offsets in the `socket` macro — since each
cell's pocket is centered within its own cell, the four copies line up
seamlessly with no extra meshing work. Self-validated the same way (the
watertight check caught a real bug during development: the first draft used
only the 4 box corners for the side walls while the top face used an
N-point sampled boundary, so the edges didn't topologically match even
though they were geometrically coincident — fixed by sampling the side walls
the same way, in the same loop). Volume matched the analytic expectation to
~0.05% once fixed.

Pin radius (5mm) vs pocket radius (7mm) gives 2mm clearance per side; pin
length (20mm) is shorter than the pocket depth (22mm) so the plug can seat
flush against the socket's top surface without its pins bottoming out.

## Pickable objects (`models/*.sdf`, Gazebo only)

The peg and plug are **not** instantiated in either Gazebo xacro file. A
fixed joint welds an object rigidly to the table — fine for a static prop,
but it means the object can never actually be picked up, since the "pick"
would have to break a joint the physics engine treats as permanent. The
plate, fixture, socket, and parts tray are deliberately kept this way
(static, fixed to the table); the peg and plug are the only objects meant to
be manipulated, so they're spawned as separate, freestanding SDF models
instead:

- `models/peg.sdf` — used by the peghole workcell
- `models/plug.sdf` — used by both the active and peghole workcells

Each is spawned by its own `ros2 run ros_gz_sim create -file ...` call in
the launch file, at the exact world-frame coordinates (`x`, `y`,
`table_top_z`) the object used to occupy when it was still welded to the
table in the URDF.

**Mass and inertia are real, computed values, not placeholders.** The peg is
a solid steel cylinder (density 7800 kg/m³): mass = 0.09924 kg, with the
standard solid-cylinder inertia formula. The plug is a composite of
aluminum body + steel pins + brass knob (densities 2700 / 7800 / 8500
kg/m³): each part's own center of mass and primitive inertia (box, cylinder,
or sphere formula) is combined about the assembly's actual center of mass
via the parallel axis theorem — mass = 0.26659 kg, CoM = (0, 0, 0.03263 m)
measured from the pin tips, Ixx = Iyy = 9.331e-05, Izz = 6.778e-05 (the
off-diagonal terms are exactly zero thanks to the 4-fold pin symmetry and
the body/knob sitting on-axis). SDF requires the `<inertial><pose>` to be at
the actual center of mass, which is why it's not just placed at a convenient
reference point. Verified live: both objects settle instantly and stay
bit-for-bit stationary over 8+ seconds of simulated time, meaning they don't
jitter or drift the way a mismatched mass/inertia/collision combination
typically would.

**Friction**: every collision surface on both objects sets
`<friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>`, per the
"high friction for pickable parts" requirement.

### Ground-truth pose (for Phase 5)

Getting an object's *true* world-frame pose out of Gazebo turned out to need
more than the obvious approach. Two things were tried and rejected before
landing on the real fix:

1. **`gz-sim-pose-publisher-system` plugin with `publish_link_pose`** — this
   reports a link's pose *relative to its own model*, which is trivially
   identity for a single-link free body (there's nothing else in the model
   to be relative to). Confirmed empirically: the published pose was all
   zeros.
2. **Bridging Gazebo's built-in `/world/<world>/dynamic_pose/info`
   (published automatically by SceneBroadcaster, with correct world-frame
   data for every model)** — the data was right, but `ros_gz_bridge`'s
   `gz.msgs.Pose_V` → `tf2_msgs/msg/TFMessage` conversion doesn't populate
   `child_frame_id` from each pose's `name` field, so every bridged
   transform came through unnamed: unusable for "look up the plug by name."

The actual fix is `so101_description/ground_truth_pose_bridge.py`, a small
custom node (installed as a `ros2 run`-able console script) that uses the
`gz.transport13` / `gz.msgs10` Python bindings (bundled with the Gazebo
Harmonic install) to subscribe to `/world/default/dynamic_pose/info`
directly, filters for whichever model names it's configured to track, and
republishes each one as a standalone, correctly-named ROS 2 topic:

| Workcell | Topic(s) |
|---|---|
| Active (`workcell_gazebo.launch.py`) | `/ground_truth/plug_pose` |
| Peghole (`workcell_peghole_gazebo.launch.py`) | `/ground_truth/peg_pose`, `/ground_truth/plug_pose` |

Each is a `geometry_msgs/msg/PoseStamped` with `frame_id: world`, timestamped
with sim time (the node runs with `use_sim_time: True` like everything else
in the launch file). Verified live: both topics stream real, correct
coordinates matching each object's actual spawn position.

**This is ground truth, not perception** — nothing that's meant to simulate
or evaluate a vision pipeline should subscribe to these topics as if they
were sensor output; they exist specifically so Phase 5 has a known-correct
answer to compare a real pose estimate against.

## Cameras

| Camera | Type | Gazebo sensor | ROS 2 topics |
|---|---|---|---|
| Overhead (gantry) | RGB-D | `rgb_camera` + `depth_camera` on `camera_link` | `/overhead_camera/rgb/{image_raw,camera_info}`, `/overhead_camera/depth/{image_raw,camera_info}` |
| Left wrist | RGB only | `left_wrist_rgb_camera` on `left_wrist_camera_link` | `/left_wrist_camera/{image_raw,camera_info}` |
| Right wrist | RGB only | `right_wrist_rgb_camera` on `right_wrist_camera_link` | `/right_wrist_camera/{image_raw,camera_info}` |

Wrist cameras are RGB-only since the mount is sized for a simple UVC webcam
module, not a depth sensor like the overhead one.

**Why a bridge is needed**: Gazebo sensors publish on Gazebo's own transport,
not ROS 2, until bridged. `workcell_gazebo.launch.py` runs one
`ros_gz_image image_bridge` node (handles all 4 image topics + automatically
republishes compressed/theora/zstd variants) and one
`ros_gz_bridge parameter_bridge` node (handles the `camera_info` topics,
which `image_bridge` doesn't cover). Each sensor has an explicit `<topic>`
tag in the xacro so these ROS topic names are stable regardless of how
Gazebo's URDF→SDF conversion names links internally (worth knowing: because
`camera_link` connects to `world` through an unbroken chain of *fixed*
joints, Gazebo's SDF conversion reduces/merges it into `table_link`
internally — harmless, but it's why `gz topic -l` shows the overhead sensor
topics nested under `.../link/table_link/...` rather than `camera_link`).

**Depth point cloud**: the overhead depth sensor also produces a 3D point
cloud (`.../depth/image_raw/points` on Gazebo's side) as an automatic
side-effect of the depth camera plugin — this is *not* bridged to ROS 2
(unneeded overhead; it ran at ~8Hz vs ~24-28Hz for plain images). It still
exists inside Gazebo if you ever want it later; just add its topic to the
`parameter_bridge` arguments:
`/overhead_camera/depth/image_raw/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked`.

## Controllers (`config/controllers.yaml`, reused unchanged)

| Controller | Joints |
|---|---|
| `so101_left_arm_controller` | left_shoulder_pan, left_shoulder_lift, left_elbow_flex, left_wrist_flex, left_wrist_roll |
| `so101_left_gripper_controller` | left_gripper |
| `so101_right_arm_controller` | right_shoulder_pan, right_shoulder_lift, right_elbow_flex, right_wrist_flex, right_wrist_roll |
| `so101_right_gripper_controller` | right_gripper |
| `joint_state_broadcaster` | all joints (state only) |

All are `joint_trajectory_controller/JointTrajectoryController`, position
command/state interfaces, driven by the `gz_ros2_control` plugin.

## How to launch

```bash
cd ~/workspaces/sow_ws/visionNassembly
colcon build --symlink-install --packages-select so101_description
source install/setup.bash

ros2 launch so101_description workcell_display.launch.py   # RViz — no physics/controllers
ros2 launch so101_description workcell_gazebo.launch.py    # Gazebo Harmonic — full sim + controllers + cameras
```

(RViz opens with no saved display config — add a `RobotModel` display and set
Fixed Frame to `table_link` or `world`.)

**Don't run two launches of the same thing at once** — during this build we
twice saw symptoms (TF flicker, a "no ros2_control tag" crash) that turned
out to be two `robot_state_publisher`/launch instances fighting over the same
topics, not real bugs. If something looks wrong, check first:
```bash
ros2 node list   # any name listed more than once means a duplicate session
```

## How to verify it's actually working

**Controllers:**
```bash
ros2 control list_controllers           # all 5 should show "active"
ros2 control list_hardware_interfaces   # all 12 joints: position [available] [claimed]

# prove it actually moves something, not just "active" in name:
ros2 action send_goal /so101_left_gripper_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [left_gripper], points: [{positions: [1.0], time_from_start: {sec: 1}}]}}"
```

**Cameras:**
```bash
ros2 topic list | grep camera          # should show all topics from the table above
ros2 topic hz /left_wrist_camera/image_raw
ros2 topic hz /overhead_camera/rgb/image_raw
rqt_image_view                         # pick a topic from the dropdown to view live
ros2 bag record /left_wrist_camera/image_raw /right_wrist_camera/image_raw \
  /overhead_camera/rgb/image_raw /overhead_camera/depth/image_raw
```
All of the above were verified live during this build: all 5 controllers
active, a trajectory goal actually moved `left_gripper` to the commanded
position, and all camera topics streaming real frames (wrist cameras
~24–28 Hz, overhead pointcloud ~8 Hz, which is expected — pointcloud
generation is heavier than a plain image).

## Explicitly out of scope (by design)

Per the original brief, this work is the workcell's visual/physical
description only. Not built yet, on purpose: perception algorithms, motion
planning, active vision, assembly controllers, or failure recovery logic.
Also not done: IK/reachability verification of the assembly-area placement
(fixture/tray coordinates are a reasonable default, not IK-checked), and the
peg/plate aren't yet free bodies for real pick-and-place.
