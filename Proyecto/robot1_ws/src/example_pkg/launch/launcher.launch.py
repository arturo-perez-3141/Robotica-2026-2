from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
  publisher_node = Node(
    package='example_pkg',
    executable="publisher_node",
    name='publisher_node',
    output='screen'
  )

  subscriber_node = Node(
    package='example_pkg',
    executable="subscriber_node",
    name='subscriber_node',
    output='screen'
  )

  turtle_sim = Node(
    package='turtlesim',
    executable="turtlesim_node",
    name='turtlesim_node',
    output='screen'
  )

  return LaunchDescription([
    publisher_node, 
    subscriber_node,
    turtle_sim
    ])


"""# Definir nodos dentro de un paquete
  publisher_node = Node(
    package='pkg_name',
    executable=publisher_path,
    name='generic_publisher',
    output='screen',
    parameters=[
      {'topic_param': '/test_topic'}
    ]
  )

  subscriber_node = Node(
    package='pkg_name',
    executable=subscriber_path,
    name='generic_subscriber',
    output='screen',
    parameters=[
      {'topic_param': '/test_topic'}
    ]
  )"""