from launch.launch_description import LaunchDescription
from launch.launch_service import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.actions import OpaqueFunction, TimerAction

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  
  return LaunchDescription([OpaqueFunction(function=launch_setup)])

def launch_setup():

  azrael_moveit = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      PathJoinSubstitution([FindPackageShare("elastoplastic_test"),"launch","azrael_moveit.launch.py"])
    )
  )

  azrael_ur_bringup = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      PathJoinSubstitution([FindPackageShare("elastoplastic_test"),"launch","azrael_ur_bringup.launch.py"])
    ),
    launch_arguments={
      "robot_ip" : "192.168.254.31"
    }
  )

  azrael_mobile_base = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      PathJoinSubstitution([FindPackageShare("elastoplastic_test"),"launch","azrael_mobile_base.launch.py"])
    )
  )

  return [
    azrael_moveit,
    azrael_mobile_base,
    TimerAction(
      period=2.0,
      actions=azrael_ur_bringup,
    ),
  ]