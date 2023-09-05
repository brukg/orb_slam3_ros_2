from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition

def generate_launch_description():
  pkg_dir = get_package_share_directory('orb_slam3_ros_2')
  settings_file = LaunchConfiguration('settings_file')
  visualization = LaunchConfiguration('visualization')

  declare_settings_file_cmd = DeclareLaunchArgument(
    'settings_file',
    default_value=pkg_dir+'/params/ZED2i.yaml',
    description='Path to the settings file'
  )

  declare_visualization_cmd = DeclareLaunchArgument(
    'visualization',
    default_value='False',
    description='Enable visualization'
  )

  print("aa", IfCondition(visualization))
  mono_orb_slam_node = Node(
    package='orb_slam3_ros_2',
    executable='mono_orb_slam_node',
    name='mono_orb_slam_node',
    output='screen',
    parameters=[
      {'publish_pointcloud': False},
      {'publish_pose': True},
      {'publish_tf': True},
      {'camera_topic': '/camera/image_raw'},
      {'camera_info_topic': '/camera/camera_info'},
      {'vocabulary_file': '/home/phoenix/ros2_ws/src/open-source/ORB_SLAM3/Vocabulary/ORBvoc.txt'},
      {'settings_file': settings_file},
      {'visualization': visualization},
    ]
  )
  
  
  ld = LaunchDescription()
  ld.add_action(declare_settings_file_cmd)
  ld.add_action(declare_visualization_cmd)
  ld.add_action(mono_orb_slam_node)

  return ld