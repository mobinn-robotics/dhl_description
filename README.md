# dhl_description

ROS 1 (Noetic) description package for the DHL mobile manipulator: a four-wheel
mobile base, a 4-DOF torso, two 7-DOF arms and two grippers — 22 movable joints.
Self-contained; it needs nothing but a stock Noetic install.

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

The storage rack is scenery, kept in its own description so it stays out of the
robot's kinematic tree. `rack:=false` drops it, `rack_xyz:="1.4 0 0"` moves it.

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

Shoulder mount height 1.128 m; wheels exactly on z = 0.

## Notes

* `urdf/arm7_left.xacro` and `urdf/arm7_right.xacro` are **generated** — both
  SolidWorks arm exports use identical link and joint names, so
  `urdf/gen_arm_macro.py` rewrites them with an `_R` / `_L` suffix. Don't edit
  them by hand; re-run the script instead.
* **Visualization model.** The base, torso and hand links have no `<inertial>`,
  and there are no transmissions or Gazebo plugins.
* **Torso joint ranges and all effort/velocity values are placeholders** — see
  the properties at the top of `urdf/dhl.urdf.xacro`. Arm ranges are real, from
  the export.
* `_R` / `_L` follow the source drawing, not REP-103: `wheel_fr` and the `_R`
  arm sit at **+y**, the *left* side in ROS convention.
