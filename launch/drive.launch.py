"""Drive the DHL base around in RViz.

The wheels are commanded through /cmd_vel, not through sliders. fake_driver
integrates the twist into rolling wheel joints + an odom transform, and the
joint_state_publisher GUI gets a wheels_fixed copy of the description so it
only offers sliders for the torso and the two arms. The two nodes publish
disjoint sets of joint names onto /joint_states. Kinematic only - no physics.

The wheels_fixed copy is fed to the GUI over its own "robot_description"
topic (published by a second, otherwise-idle robot_state_publisher whose own
/tf and /tf_static are remapped away) rather than the real robot_description,
so the two descriptions never fight over the same topic.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("dhl_description")

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=PathJoinSubstitution([pkg_share, "urdf", "dhl.urdf.xacro"]),
    )
    rvizconfig_arg = DeclareLaunchArgument(
        "rvizconfig",
        default_value=PathJoinSubstitution([pkg_share, "rviz", "dhl.rviz"]),
    )
    gui_arg = DeclareLaunchArgument(
        "gui", default_value="true",
        description="sliders for torso + arms",
    )
    steering_arg = DeclareLaunchArgument(
        "steering", default_value="true",
        description="rqt slider panel for /cmd_vel",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true",
        description="off for headless runs",
    )
    rack_arg = DeclareLaunchArgument(
        "rack", default_value="true",
        description="storage rack scenery",
    )
    rack_model_arg = DeclareLaunchArgument(
        "rack_model",
        default_value=PathJoinSubstitution([pkg_share, "urdf", "rack.urdf.xacro"]),
    )
    rack_xyz_arg = DeclareLaunchArgument("rack_xyz", default_value="1.4 0 0")
    rack_rpy_arg = DeclareLaunchArgument(
        # shelves face the robot
        "rack_rpy", default_value="0 0 3.14159265358979",
    )

    model = LaunchConfiguration("model")
    rvizconfig = LaunchConfiguration("rvizconfig")
    gui = LaunchConfiguration("gui")
    steering = LaunchConfiguration("steering")
    rviz = LaunchConfiguration("rviz")
    rack = LaunchConfiguration("rack")
    rack_model = LaunchConfiguration("rack_model")
    rack_xyz = LaunchConfiguration("rack_xyz")
    rack_rpy = LaunchConfiguration("rack_rpy")

    robot_description = ParameterValue(
        Command(["xacro ", model]), value_type=str)
    robot_description_wheels_fixed = ParameterValue(
        Command(["xacro ", model, " wheels_fixed:=true"]), value_type=str)
    rack_description = ParameterValue(
        Command(["xacro ", rack_model, ' xyz:="', rack_xyz, '" rpy:="', rack_rpy, '"']),
        value_type=str)

    fake_driver = Node(
        package="dhl_description",
        executable="fake_driver.py",
        name="fake_driver",
        output="screen",
        parameters=[{
            "wheel_radius": 0.155,
            "track_width": 0.5566,
            "rate": 50.0,
            # 0 keeps the last command instead of stopping when /cmd_vel goes quiet
            "cmd_timeout": 0.0,
        }],
    )

    # Feeds joint_state_publisher(_gui) a copy of the description with the
    # wheel joints fixed, over its own topic, so the GUI drops the four wheel
    # sliders while RViz still shows the real, rolling model. Its own /tf and
    # /tf_static are remapped to unused topics so it never broadcasts transforms.
    wheels_fixed_description_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="wheels_fixed_description_publisher",
        output="both",
        parameters=[{"robot_description": robot_description_wheels_fixed}],
        remappings=[
            ("robot_description", "robot_description_wheels_fixed"),
            ("tf", "tf_wheels_fixed_unused"),
            ("tf_static", "tf_static_wheels_fixed_unused"),
        ],
    )

    joint_state_publisher_gui = Node(
        condition=IfCondition(gui),
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        remappings=[("robot_description", "robot_description_wheels_fixed")],
    )
    joint_state_publisher = Node(
        condition=UnlessCondition(gui),
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        remappings=[("robot_description", "robot_description_wheels_fixed")],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description}],
    )

    # Scenery. Its own description and publisher, so the rack stays out of the
    # robot's kinematic tree; the remap points the publisher at rack_description.
    rack_group = GroupAction(
        condition=IfCondition(rack),
        actions=[
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="rack_state_publisher",
                output="both",
                parameters=[{"robot_description": rack_description}],
                remappings=[
                    ("robot_description", "rack_description"),
                    # The rack has one fixed joint and nothing movable, so its
                    # transform goes out on /tf_static at startup and it never
                    # needs a joint state. Point the subscription at a topic
                    # nobody publishes, or it picks up the robot's wheel
                    # states and warns about joints it has never heard of,
                    # once every 10 s, forever.
                    ("joint_states", "rack_joint_states"),
                ],
            ),
        ],
    )

    rqt_robot_steering = Node(
        condition=IfCondition(steering),
        package="rqt_robot_steering",
        executable="rqt_robot_steering",
        name="rqt_robot_steering",
    )

    rviz_node = Node(
        condition=IfCondition(rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rvizconfig],
        # mirrors roslaunch's required="true": closing RViz ends the whole launch
        on_exit=Shutdown(),
    )

    return LaunchDescription([
        model_arg,
        rvizconfig_arg,
        gui_arg,
        steering_arg,
        rviz_arg,
        rack_arg,
        rack_model_arg,
        rack_xyz_arg,
        rack_rpy_arg,
        fake_driver,
        wheels_fixed_description_publisher,
        joint_state_publisher_gui,
        joint_state_publisher,
        robot_state_publisher,
        rack_group,
        rqt_robot_steering,
        rviz_node,
    ])
