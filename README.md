# dhl_description

ROS 1 (Noetic) description package for the DHL mobile manipulator: a four-wheel
mobile base, a 4-DOF torso, two 7-DOF arms and two grippers — 22 movable joints
in total. Ships a `/cmd_vel` driver so the robot can be driven around in RViz.

The package is self-contained. Clone it into a catkin workspace and build; it
pulls in nothing but stock Noetic packages.

```bash
cd ~/catkin_ws/src
git clone git@github.com:mobinn-robotics/dhl_description.git
cd ~/catkin_ws && catkin_make && source devel/setup.bash
roslaunch dhl_description drive.launch
```

## Using it

`drive.launch` brings up everything. Two windows control the robot:

* **`joint_state_publisher_gui`** — a slider per torso and arm joint (18).
* **`rqt_robot_steering`** — linear/angular sliders that drive the base.

The wheels have no sliders; they roll only when the base is driven.

## Kinematic tree

```
base_footprint
 └ mobile_base                        fixed, z = 155 mm (= wheel radius)
   ├ wheel_fr_Link  (0.316,  0.278)   continuous, pitch
   ├ wheel_rr_Link (-0.316,  0.278)   continuous, pitch
   ├ wheel_fl_Link  (0.316, -0.278)   continuous, pitch
   ├ wheel_rl_Link (-0.316, -0.278)   continuous, pitch
   └ torso_link1                      revolute, yaw    (x=323, z=119.5)
     └ torso_link2                    revolute, pitch  (z=43)
       └ torso_link3                  revolute, pitch  (y=275)
         └ torso_link4                revolute, pitch  (y=275)
           ├ right_arm_bracket         fixed (y=260.5, z=-100, rpy 180/0/90)
           │ └ base_link_R … wrist2_Link_R   j1_R … j7_R
           │   └ dhl_hand_4_R          fixed (x=-106, rpy 90/90/0)
           │     └ tcp_R               fixed (y=-120)
           └ left_arm_bracket          fixed (y=260.5, z=+100, rpy 0/0/90)
             └ base_link_L … wrist2_Link_L   j1_L … j7_L
               └ dhl_hand_4_L
                 └ tcp_L
```

Shoulder mount height 1.128 m; overall bounding box 0.94 x 2.23 x 1.20 m with
the arms straight out (zero pose); wheels exactly on z = 0.

### Joint limits

| joint | type | range |
|---|---|---|
| `wheel_{fr,rr,fl,rl}_joint` | continuous | — |
| `torso_joint1` | revolute, yaw | ±180° *(placeholder)* |
| `torso_joint2` | revolute, pitch | ±90° *(placeholder)* |
| `torso_joint3`, `torso_joint4` | revolute, pitch | ±150° *(placeholder)* |
| `j1`, `j3`, `j5` | revolute | ±175° |
| `j2` | revolute | ±110° |
| `j4` | revolute | −75° … +110° |
| `j6` | revolute | ±70° |
| `j7` | revolute | ±85° |

Arm limits come from the SolidWorks export. Torso limits were never supplied —
override the `torso1_lower` … `torso4_upper` properties at the top of
`urdf/dhl.urdf.xacro` with the real numbers.

## Layout

```
dhl_description/
├── urdf/
│   ├── dhl.urdf.xacro       top level: base, wheels, torso, brackets, hands
│   ├── arm7_left.xacro      7-DOF arm macro (generated)
│   ├── arm7_right.xacro     7-DOF arm macro (generated)
│   └── gen_arm_macro.py     regenerates the two macros above
├── meshes/
│   ├── mobile_base.stl, wheel_*.stl, torso_joint*.stl, dhl_hand_4.stl   (mm)
│   ├── left_arm/*.STL                                                   (m)
│   └── right_arm/*.STL                                                  (m)
├── launch/drive.launch
├── rviz/dhl.rviz
└── scripts/fake_driver.py
```

## How driving works

`scripts/fake_driver.py` integrates `/cmd_vel` into rolling wheel angles and an
`odom -> base_footprint` transform. RViz cannot command anything on its own — it
only draws whatever `/joint_states` and TF say, so something has to publish
them. The fixed frame in `rviz/dhl.rviz` is `odom`, which is why the robot moves
across the grid instead of the world sliding under it.

The four wheels are fixed (no steering joints), so the base is treated as
skid-steer: the two +y wheels turn together and the two −y wheels turn together.
Verified against the closed form — a 0.3 rad/s spin in place gives each wheel
`0.3 · 0.2783 / 0.155 = 0.539 rad/s`, and 3 s of `vx=0.5, wz=0.3` lands the base
at 50.9° of yaw (51.6° exact, the rest is discrete integration).

This is **kinematics, not physics**: the commanded twist is integrated directly,
wheels never slip, nothing collides and nothing has mass.

**The wheels deliberately have no sliders.** `joint_state_publisher` reads
`~robot_description` in preference to the global one, so the GUI is handed a
copy of the model built with `wheels_fixed:=true` and creates sliders for the 18
torso and arm joints only — RViz and `robot_state_publisher` still get the real
description with rolling wheels. The driver then publishes just its 4 wheel
names on `/joint_states` while the GUI publishes the other 18;
`robot_state_publisher` remembers the last value per joint name, so two
publishers over disjoint name sets merge cleanly.

(`source_list` does not work for this. Joints arriving through it stay in the
publisher's `free_joints`, so the GUI keeps showing wheel sliders and they fight
the driver for the value.)

`fake_driver` keeps running even when you are not driving: a wheel needs *some*
publisher for its joint value or it gets no transform and disappears from RViz.

## Units and conventions

Base, wheel, torso and hand meshes are in **millimetres** and carry
`scale="0.001 0.001 0.001"`. The arm meshes are already in **metres** and are
used unscaled. In the xacro, lengths are written as `${323*mm}` and angles as
`${radians(90)}` so every number stays comparable to the source drawing.

Every revolute joint uses `axis xyz="0 0 1"`; the joint's `rpy` is what turns
that local Z into a yaw or a pitch axis.

## Provenance

The two arms were exported from SolidWorks (`sw_urdf_exporter` 1.6.0) as
`left_arm_description` / `right_arm_description`. Both exports use identical
link and joint names, which collide when you put two arms on one robot, so
`urdf/gen_arm_macro.py` rewrites each export into a xacro macro that appends
`${suffix}` (`_R` / `_L`) to every link and joint name, points the mesh paths at
this package's own copies under `meshes/left_arm` and `meshes/right_arm`, and
replaces the exporter's unnamed materials with `dhl_light`. Do not hand-edit
`arm7_left.xacro` / `arm7_right.xacro`; if an export changes, drop the updated
`*_arm_description` packages next to this one and re-run:

```bash
python3 urdf/gen_arm_macro.py
```

## Known gaps

* **`_R` / `_L` follow the source drawing, not REP-103.** `wheel_fr` and the
  `_R` arm sit at **+y**, which is the *left* side in ROS convention. The naming
  is at least self-consistent — wheels and arms use the same convention, and the
  wheel meshes are handed to match — but rename them if REP-103 matters.
* **Torso joint ranges are placeholders**, as noted above.
* **`effort` / `velocity` are placeholders.** The SolidWorks export left every
  arm joint at `effort="0" velocity="0"`, which no controller can use, so the
  macros substitute `${arm_effort}` / `${arm_velocity}`.
* **Visualization model.** Base, torso and hand links have no `<inertial>`, and
  there are no `<transmission>` tags or Gazebo plugins. Add those before using
  this in Gazebo or MoveIt.
* **`collision` reuses the full-resolution visual meshes** (126 552 triangles
  total) — replace with primitives or convex hulls before collision checking.
* **`track_width` is the geometric track** (2 × 278.3 mm). Real skid-steer
  platforms over-rotate against that number because the wheels scrub sideways;
  widen the `track_width` param until simulated yaw matches the real robot.
* **`right` upperarm2 inertia looks wrong** in the source export
  (`iyy == izz == 0.01213635`); it is carried through unchanged.
* This package deliberately does **not** use `catkin_install_python`. On a
  machine where CMake resolves `PYTHON_EXECUTABLE` to an anaconda interpreter,
  the devel-space wrapper it generates hard-codes that interpreter, and a Noetic
  node started under Python 3.12 hangs before it registers with the master.
  `install(PROGRAMS)` leaves the script's own `#!/usr/bin/env python3` in charge.
