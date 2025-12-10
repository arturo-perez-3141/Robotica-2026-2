from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
  # --- RUTAS ---
  description_path = get_package_share_directory("example_description")
  model_path = os.path.join(description_path, "urdf", "rrr_robot.urdf")
  rviz_conf_path = os.path.join(description_path, "rviz", "rviz_config.rviz")
  
  robot_description = {"robot_description": Command(["xacro ", model_path])}

  # --- NODOS ---

  controller_manager_node = Node(
    package='example_control',
    executable="controller_manager",
    output='screen'
  )

  manipulator_controller_node = Node(
    package='example_control',
    executable="manipulator_controller",
    output='screen'
  )

  hardware_interface_node = Node(
    package='example_control',
    executable="hardware_interface",
    output='screen'
  )

  rviz_node = Node(
    package='rviz2',
    executable="rviz2",
    arguments=["-d", rviz_conf_path],
    output='log'
  )
  
  rsp_node = Node(
    package='robot_state_publisher',
    executable="robot_state_publisher",
    parameters=[robot_description],
    output='both'
  )
  
  # ¡¡ELIMINADO jsp_node!! 
  # (joint_state_publisher_gui causaba el conflicto y los NaNs)

  return LaunchDescription([
    controller_manager_node,
    manipulator_controller_node,
    #hardware_interface_node,
    rviz_node, 
    rsp_node
    ])
