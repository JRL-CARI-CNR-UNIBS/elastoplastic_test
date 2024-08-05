from launch.launch_description import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.substitutions import PathJoinSubstitution

from launch.event_handlers import OnExecutionComplete

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

  ros2_control_config_path = PathJoinSubstitution([FindPackageShare("elastoplastic_test"), "config", "ros2_controllers.yaml"])

  controller_manager_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[ros2_control_config_path],
    # prefix="gnome-terminal -- cgdb -ex run --args",
    output="screen",
    remappings=[("/controller_manager/robot_description","/robot_description")],
  )

  elastoplastic_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["elastoplastic_controller",
               "--controller-manager", "/controller_manager"],
  )

  joint_trajectory_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_trajectory_controller", 
               "--controller-manager", "/controller_manager"],
  )

  joint_state_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster",
               "--controller-manager","/controller_manager"],
  )

  robotiq_force_torque_sensor_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["robotiq_force_torque_sensor_broadcaster",
               "--controller-manager","/controller_manager"],
  )

  return LaunchDescription([
    controller_manager_node,
    TimerAction(
      period=2.0,
      actions=[joint_state_broadcaster_spawner,
               robotiq_force_torque_sensor_broadcaster_spawner],
    ),
    RegisterEventHandler(
      OnExecutionComplete(
        target_action=[joint_state_broadcaster_spawner],
        on_completion=[elastoplastic_controller_spawner],
      )
    ),
    RegisterEventHandler(
      OnExecutionComplete(
        target_action=[elastoplastic_controller_spawner],
        on_completion=[joint_trajectory_controller_spawner],
      )
    ),
  ])