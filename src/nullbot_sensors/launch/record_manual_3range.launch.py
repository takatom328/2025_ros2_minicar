from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    # ---- args ----
    use_tof = LaunchConfiguration("use_tof")
    joy_dev = LaunchConfiguration("joy_dev")
    out_dir = LaunchConfiguration("out_dir")
    sample_hz = LaunchConfiguration("sample_hz")
    allow_partial = LaunchConfiguration("allow_partial")

    # topics
    front_in = LaunchConfiguration("front_in")
    left_in  = LaunchConfiguration("left_in")
    right_in = LaunchConfiguration("right_in")

    front_out = LaunchConfiguration("front_out")
    left_out  = LaunchConfiguration("left_out")
    right_out = LaunchConfiguration("right_out")

    cmd_topic = LaunchConfiguration("cmd_topic")

    # joy_to_cmd params
    axis_steer = LaunchConfiguration("axis_steer")
    axis_throttle = LaunchConfiguration("axis_throttle")
    invert_throttle = LaunchConfiguration("invert_throttle")

    ld = LaunchDescription()

    # ---- launch args ----
    ld.add_action(DeclareLaunchArgument("use_tof", default_value="false"))  # ハード無しなら false
    ld.add_action(DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"))

    ld.add_action(DeclareLaunchArgument("out_dir", default_value="/home/tt18/datasets"))
    ld.add_action(DeclareLaunchArgument("sample_hz", default_value="10.0"))
    ld.add_action(DeclareLaunchArgument("allow_partial", default_value="true"))

    ld.add_action(DeclareLaunchArgument("front_in", default_value="/range/front"))
    ld.add_action(DeclareLaunchArgument("left_in",  default_value="/range/left"))
    ld.add_action(DeclareLaunchArgument("right_in", default_value="/range/right"))

    ld.add_action(DeclareLaunchArgument("front_out", default_value="/range/front_filtered"))
    ld.add_action(DeclareLaunchArgument("left_out",  default_value="/range/left_filtered"))
    ld.add_action(DeclareLaunchArgument("right_out", default_value="/range/right_filtered"))

    ld.add_action(DeclareLaunchArgument("cmd_topic", default_value="/cmd_manual"))

    ld.add_action(DeclareLaunchArgument("axis_steer", default_value="4"))
    ld.add_action(DeclareLaunchArgument("axis_throttle", default_value="3"))
    ld.add_action(DeclareLaunchArgument("invert_throttle", default_value="true"))

    # ---- (optional) ToF triplet ----
    ld.add_action(
        Node(
            package="nullbot_sensors",
            executable="vl53l0x_triplet_node",
            name="vl53l0x_triplet_node",
            output="screen",
            condition=IfCondition(use_tof),
            parameters=[
                # ピンは仮。必要なら launch 引数化してもOK
                {"xshut_front": 17, "xshut_left": 27, "xshut_right": 22},
                {"addr_front": 0x30, "addr_left": 0x31, "addr_right": 0x32},
                {"publish_hz": 20.0},
                {"topic_front": "/range/front", "topic_left": "/range/left", "topic_right": "/range/right"},
                {"frame_front": "tof_front", "frame_left": "tof_left", "frame_right": "tof_right"},
            ],
        )
    )

    # ---- joy_node ----
    ld.add_action(
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[{"dev": joy_dev}],
        )
    )

    # ---- joy_to_cmd ----
    ld.add_action(
        Node(
            package="nullbot_teleop",
            executable="joy_to_cmd",
            name="joy_to_cmd",
            output="screen",
            parameters=[
                {"axis_steer": axis_steer},
                {"axis_throttle": axis_throttle},
                {"invert_throttle": invert_throttle},
            ],
            remappings=[
                # 必要なら /cmd_manual を別名にする等もここで
                # ("/cmd_manual", cmd_topic),
            ],
        )
    )

    # ---- Kalman filters (3x) ----
    ld.add_action(
        Node(
            package="nullbot_sensors",
            executable="range_kalman_filter",
            name="kf_front",
            output="screen",
            parameters=[{"input_topic": front_in, "output_topic": front_out}],
        )
    )
    ld.add_action(
        Node(
            package="nullbot_sensors",
            executable="range_kalman_filter",
            name="kf_left",
            output="screen",
            parameters=[{"input_topic": left_in, "output_topic": left_out}],
        )
    )
    ld.add_action(
        Node(
            package="nullbot_sensors",
            executable="range_kalman_filter",
            name="kf_right",
            output="screen",
            parameters=[{"input_topic": right_in, "output_topic": right_out}],
        )
    )

    # ---- recorder ----
    ld.add_action(
        Node(
            package="nullbot_sensors",
            executable="dataset_recorder_3range_ackermann",
            name="dataset_recorder",
            output="screen",
            parameters=[
                {"front_topic": front_out},
                {"left_topic": left_out},
                {"right_topic": right_out},
                {"cmd_topic": cmd_topic},
                {"out_dir": out_dir},
                {"sample_hz": sample_hz},
                {"allow_partial": allow_partial},
            ],
        )
    )

    return ld

