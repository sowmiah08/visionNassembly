# visionNassembly

**Vision-guided dual-arm assembly in simulation.** Two SO-101 robot arms
use an overhead RGB-D camera to find parts on a table, pick them up and
assemble them: a 4-pin plug into a socket, and a peg into a plate. It
recovers from failures on its own.

Built with **ROS 2 Jazzy · Gazebo Harmonic · MoveIt 2 · SAM 2.1 · Open3D**.

## How it works

```
overhead camera ──▶ find objects in depth ──▶ SAM 2 masks ──▶ fit each part's 3D model
                                                                   │
                                                          6-DoF poses (≈1 mm)
                                                                   ▼
          MoveIt plans ◀── grasp → lift → align → insert → verify (assemble.py)
                                   │
                    recovery: re-scan · next grasp · spiral search
```

1. **See:** depth separates objects from the table, and SAM 2 draws an
   exact mask around each one.
2. **Locate:** each part's 3D model (built from its simulation model) is
   fitted to the masked depth points, giving its position and rotation.
3. **Act:** MoveIt plans a straight-down grasp, a lift and a careful
   insertion.
4. **Recover:** if a part isn't found, a grasp misses, or an insertion is
   blocked, the robot re-scans, tries another grasp, or searches around
   the hole in a spiral.
5. **Evaluate:** a test harness randomises the scene and scores every
   run against the simulator's true poses.

**Adding a new part needs no new code:** just its model plus a short YAML
file describing how to grip it and how it mates. The peg/plate pair was
added this way.

## Results

6 randomised trials per pair. Each trial randomised the part's position
(within the arm's reachable area) and its full 360° rotation, partial
occlusion of the camera's view, depth/RGB noise, and a 0–3 mm injected
insertion error.

| Assembly | Success | Pose error | Insertion error |
|---|---|---|---|
| Plug → socket (left arm) | **6/6** | ≈ 1 mm | 1.2 mm mean |
| Peg → plate (right arm) | **5/6** | ≈ 1 mm | 1.6 mm mean |

The one failure had only 18% of the peg visible to the camera; it was
correctly reported as "not found" rather than grasped blindly. Small
sample sizes: treat these as a working demo, not a benchmark.

## Requirements

- Ubuntu 24.04, ROS 2 Jazzy
- An NVIDIA GPU with CUDA (for SAM 2)
- ROS packages:
  ```bash
  sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
      ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-xacro \
      ros-jazzy-moveit ros-jazzy-moveit-py ros-jazzy-pick-ik
  sudo apt upgrade     # upgrade everything together; partial upgrades break MoveIt
  ```

## Setup

```bash
git clone https://github.com/sowmiah08/visionNassembly.git
cd visionNassembly
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

**Python environment for perception** (SAM 2 + Open3D). It can still see
the ROS Python packages:
```bash
python3 -m venv --system-site-packages .venv
.venv/bin/pip install torch torchvision          # CUDA build for your GPU
.venv/bin/pip install "numpy==1.26.4" hydra-core iopath tqdm pillow open3d
SAM2_BUILD_CUDA=0 .venv/bin/pip install --no-deps "git+https://github.com/facebookresearch/sam2.git"
mkdir -p models/sam2
curl -L -o models/sam2/sam2.1_hiera_small.pt \
    https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_small.pt
```
Keep NumPy at 1.26: NumPy 2 breaks ROS's `cv_bridge`.

## Run it

In every new terminal:
```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

**1. Start everything** (Gazebo, MoveIt, the perception service):
```bash
src/vision_perception/scripts/stack.sh start      # also: stop | restart | status
```

**2. Check perception:** one scan, scored against the true poses:
```bash
.venv/bin/python -m vision_perception.scan_eval --out /tmp/scan.png
```

**3. Run an assembly** and watch it in Gazebo:
```bash
.venv/bin/python src/vision_perception/vision_perception/assemble.py --assembly plug_into_socket --arm left
.venv/bin/python src/vision_perception/vision_perception/assemble.py --assembly peg_into_plate --arm right
```
Run `stack.sh restart` between runs, since afterwards the part is
already assembled.

**4. Run the randomised evaluation:**
```bash
.venv/bin/python -m vision_perception.evaluate --assembly plug_into_socket --arm left \
    --trials 6 --out /tmp/eval_plug --seed 3 \
    --occlusion 0.5 --depth-noise 0.002 --depth-dropout 0.05 --rgb-noise 8 --insert-error-mm 3
cat /tmp/eval_plug/summary.json
```
Results land in `/tmp/eval_plug/`: `summary.json`, plus one log per trial.

Perception code must run with `.venv/bin/python`, not `ros2 run`, which
uses the system Python and can't see SAM 2.

## What's in the repo

| Package | What it is |
|---|---|
| `so101_description` | The robot, table, cameras and parts; the Gazebo launch file |
| `so101_moveit_config` | MoveIt configuration for both arms |
| `vision_perception` | Perception pipeline, scan service, assembly, evaluation |
| `vision_perception_interfaces` | ROS message/service types (`Scan.srv`, `ObjectPose.msg`) |

Part descriptions live in `src/vision_perception/config/`:
`objects/` (one file per part), `grippers/` and `assemblies/`.

## Add your own part

1. Add a Gazebo model in `so101_description/models/`, with a segmentation
   label plugin, and spawn it in `launch/workcell_gazebo.launch.py`.
2. Describe it in `vision_perception/config/objects/<name>.yaml`: its
   model, symmetry and grasp.
3. If it fits into something, add `config/assemblies/<part>_into_<target>.yaml`.
4. Rebuild, restart, and test with `scan_eval`, then `assemble.py`.

## Known limits

- The arms can only grasp straight down within a small reachable area.
  Parts have to be placed there.
- Only the part's position is randomised; targets stay in place.
- Parts that are mostly hidden from the overhead camera (≲ 20% visible)
  aren't found. Using the wrist cameras to look again is the next step.
- Simulation only.

## Next steps

- Active perception with the wrist cameras
- Larger evaluations (more trials, error sweeps, randomised targets)


