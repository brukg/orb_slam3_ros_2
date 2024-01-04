from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition

def generate_launch_description():
  pkg_dir = get_package_share_directory('orb_slam3_ros_2')
  settings_file = LaunchConfiguration('settings_file')
  vocabulary_file = LaunchConfiguration('vocabulary_file')
  visualization = LaunchConfiguration('visualization')

  declare_settings_file_cmd = DeclareLaunchArgument(
    'settings_file',
    default_value=pkg_dir+'/params/ZED2i.yaml',
    description='Path to the settings file'
  )

  declare_vocabulary_file_cmd = DeclareLaunchArgument(
    'vocabulary_file',
    default_value=pkg_dir+'/params/vocabulary/ORBvoc.txt',
    description='Path to the vocabulary file'
  )

  declare_visualization_cmd = DeclareLaunchArgument(
    'visualization',
    default_value='True',
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
      {'vocabulary_file': vocabulary_file},
      {'settings_file': settings_file},
      {'visualization': visualization},
    ],
    remappings=[
      ('/camera/image_raw', '/zed2i/zed_node/left/image_rect_color'),
      ('/right/image_rect_gray/compressed', '/zed2i/zed_node/right/image_rect_color'),
      ('/left/image_rect_gray/compressed', '/zed2i/zed_node/left/image_rect_color'),
      ('/camera/camera_info', '/camera/camera_info'),
      ('/camera/image_raw/compressed', '/zed2i/zed_node/stereo/compressed/image_rect_color'),
      ('/camera/stereo/image_raw', '/zed2i/zed_node/stereo/image_rect_color'),
    ],
  )
  
  
  ld = LaunchDescription()
  ld.add_action(declare_settings_file_cmd)
  ld.add_action(declare_visualization_cmd)
  ld.add_action(declare_vocabulary_file_cmd)
  ld.add_action(mono_orb_slam_node)

  return ld
