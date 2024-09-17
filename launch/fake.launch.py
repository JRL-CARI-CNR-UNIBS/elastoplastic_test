from launch.launch_description import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  return LaunchDescription([OpaqueFunction(function=launch_setup)])

def launch_setup(context):

  launch_moveit_path = PathJoinSubstitution([FindPackageShare('elastoplastic_test'), 'launch', 'azrael_moveit.launch.py'])
  launch_moveit_and_robot_description = IncludeLaunchDescription(
    launch_description_source=PythonLaunchDescriptionSource(launch_moveit_path),
    launch_arguments=[('robot_ip', '0.0.0.0'), ('use_fake_hardware','true'), ('ft_sensor_ros2_control','true')]
  )

  launch_controllers_path = PathJoinSubstitution([FindPackageShare('elastoplastic_test'), 'launch', 'azrael_ur_controllers.launch.py'])
  launch_controllers = IncludeLaunchDescription(
    launch_description_source=PythonLaunchDescriptionSource(launch_controllers_path),
  )

  return [
    launch_moveit_and_robot_description,
    launch_controllers
  ]
