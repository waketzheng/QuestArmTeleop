import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    oculus_reader_pkg_dir = get_package_share_directory("oculus_reader")
    agx_arm_ctrl_pkg_dir = get_package_share_directory("agx_arm_ctrl")

    arm_ik_param_file = os.path.join(
        oculus_reader_pkg_dir, "config", "arm_ik_pose_node.piper_x.yaml"
    )

    # 1) ros2 launch agx_arm_ctrl start_single_agx_arm_rviz.launch.py ...
    left_arm_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                agx_arm_ctrl_pkg_dir,
                "launch",
                "start_single_agx_arm_rviz.launch.py",
            )
        ),
        launch_arguments={
            "can_port": "can_left",
            "namespace": "left_arm",
            "arm_type": "piper_x",
            "auto_enable": "true",
            "effector_type": "agx_gripper",
            "tcp_offset": "[0.0, 0.0, 0.13, 0.0, 0.0, 0.0]",
            "control": "false",
            "fast_mode": "true",
            "home_speed_percent": "15",
            "home_joint_tolerance": "0.03",
            "motion_done_timeout": "0.0",
            #"pub_rate": "30",
        }.items(),
    )

    right_arm_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                agx_arm_ctrl_pkg_dir,
                "launch",
                "start_single_agx_arm_rviz.launch.py",
            )
        ),
        launch_arguments={
            "can_port": "can_right",
            "namespace": "right_arm",
            "arm_type": "piper_x",
            "auto_enable": "true",
            "effector_type": "agx_gripper",
            "tcp_offset": "[0.0, 0.0, 0.13, 0.0, 0.0, 0.0]",
            "control": "false",
            "fast_mode": "true",
            "home_speed_percent": "15",
            "home_joint_tolerance": "0.03",
            "motion_done_timeout": "0.0",
            #"pub_rate": "30",
        }.items(),
    )

    # 2) ros2 run oculus_reader arm_ik_pose_node.py --ros-args --params-file ...
    left_arm_ik_pose_node = Node(
        package="oculus_reader",
        executable="arm_ik_pose_node.py",
        name="left_arm_ik_pose_node",
        output="screen",
        parameters=[
            arm_ik_param_file,
            {
                "pose_stamped_topic": "/left_delta_pose",
                "feedback_joint_topic": "/left_arm/feedback/joint_states",
                "pin_joint_status_topic": "/left_arm/control/joint_states",
            },
        ],
    )

    right_arm_ik_pose_node = Node(
        package="oculus_reader",
        executable="arm_ik_pose_node.py",
        name="right_arm_ik_pose_node",
        output="screen",
        parameters=[
            arm_ik_param_file,
            {
                "pose_stamped_topic": "/right_delta_pose",
                "feedback_joint_topic": "/right_arm/feedback/joint_states",
                "pin_joint_status_topic": "/right_arm/control/joint_states",
            },
        ],
    )

    # 3) ros2 run oculus_reader pub_pose.py --ros-args -p ros_to_arm_rpy:=...
    pub_pose_node = Node(
        package="oculus_reader",
        executable="pub_pose.py",
        name="pub_pose_node",
        output="screen",
        #parameters=[
        #    {
        #        "publish_rate_hz": 30.0,
        #    }
        #],
        # pika frame to arm ee frame
        arguments=["--ros-args", "-p", "ros_to_arm_rpy:=[-1.5708, 0.0, -1.5708]"],
    )

    # 4) ros2 run oculus_reader pub_delta_pose.py
    # Left hand -> left arm delta pose
    left_pub_delta_pose_node = Node(
        package="oculus_reader",
        executable="pub_delta_pose.py",
        name="left_pub_delta_pose_node",
        output="screen",
        parameters=[
            {
                "hand_name": "left",
                "handle_pose_topic": "/left_handle_pose",
                "feedback_tcp_pose_topic": "/left_arm/feedback/tcp_pose",
                "feedback_joint_topic": "/left_arm/feedback/joint_states",
                "delta_pose_topic": "/left_delta_pose",
                "control_joint_topic": "/left_arm/control/joint_states",
                "start_button": "X",
                "stop_button": "Y",
                "trigger_axis": "leftTrig",
                "reset_axis": "leftGrip",
                "reset_service": "/left_arm/move_home",
                "reset_joint_tolerance": 0.03,
                "reset_timeout": 8.0,
                #"control_rate_hz": 30.0,
                "gripper_max_range": 0.1,
            }
        ],
    )
    
    # Right hand -> right arm delta pose
    right_pub_delta_pose_node = Node(
        package="oculus_reader",
        executable="pub_delta_pose.py",
        name="right_pub_delta_pose_node",
        output="screen",
        parameters=[
            {
                "hand_name": "right",
                "handle_pose_topic": "/right_handle_pose",
                "feedback_tcp_pose_topic": "/right_arm/feedback/tcp_pose",
                "feedback_joint_topic": "/right_arm/feedback/joint_states",
                "delta_pose_topic": "/right_delta_pose",
                "control_joint_topic": "/right_arm/control/joint_states",
                "start_button": "A",
                "stop_button": "B",
                "trigger_axis": "rightTrig",
                "reset_axis": "rightGrip",
                "reset_service": "/right_arm/move_home",
                "reset_joint_tolerance": 0.03,
                "reset_timeout": 8.0,
                #"control_rate_hz": 30.0,
                "gripper_max_range": 0.1,
            }
        ],
    )

    lift_joystick_controller_node = Node(
        package="oculus_reader",
        executable="lift_joystick_controller.py",
        name="lift_joystick_controller",
        output="screen",
        parameters=[
            {
                "lift_can_interface": "can3",
            }
        ],
    )

    agv_joystick_controller_node = Node(
        package="oculus_reader",
        executable="agv_joystick_controller.py",
        name="agv_joystick_controller",
        output="screen",
    )

    # 5) ros2 run rviz2 rviz2 --ros-args -p config:=...
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('oculus_reader'), 'config', 'oculus_reader.rviz')]
    )
    
    return LaunchDescription(
        [
            left_arm_driver_launch,
            right_arm_driver_launch,
            left_arm_ik_pose_node,
            right_arm_ik_pose_node,
            left_pub_delta_pose_node,
            right_pub_delta_pose_node,
            pub_pose_node,
            lift_joystick_controller_node,
            agv_joystick_controller_node,
            #rviz_node,
        ]
    )
