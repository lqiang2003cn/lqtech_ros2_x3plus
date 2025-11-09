# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    
    robot_description = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ', PathJoinSubstitution([FindPackageShare("lqtech_ros2_x3plus"),"urdf","x3plus.urdf"]),
    ]), value_type=str)
    
    # # Get URDF via xacro
    # robot_description_content = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         " ",
    #         PathJoinSubstitution(
    #             [
    #                 FindPackageShare("lqtech_ros2_x3plus"),
    #                 "urdf",
    #                 "x3plus.urdf"
    #             ]
    #         ),
    #     ]
    # )
    
    robot_description = {"robot_description": robot_description}
    
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("lqtech_ros2_x3plus"), "rviz", "x3plus_ros2_control.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )
    
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("lqtech_ros2_x3plus"),
            "config",
            "x3plus_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
    )

    # # Delay rviz start after `joint_state_broadcaster`
    # delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[rviz_node],
    #     )
    # )

    # # Delay start of joint_state_broadcaster after `robot_controller`
    # # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    # delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=robot_controller_spawner,
    #         on_exit=[joint_state_broadcaster_spawner],
    #     )
    # )

    nodes = [
        robot_state_pub_node,
        rviz_node,
        control_node,
        joint_state_broadcaster_spawner,
        forward_position_controller_spawner
        # delay_rviz_after_joint_state_broadcaster_spawner,
        # delay_joint_state_broadcaster_after_robot_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
