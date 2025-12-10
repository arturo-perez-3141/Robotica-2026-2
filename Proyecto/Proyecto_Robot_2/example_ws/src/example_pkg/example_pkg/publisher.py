#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.msg import ExampleMsg

class SimplePublisher(Node):
  def __init__(self):
    super().__init__('simple_publisher')
    topic_name = "/topic"
    self.publisher = self.create_publisher(ExampleMsg, topic_name, 10)
    self.timer = self.create_timer(1.0, self.timer_callback)
    self.get_logger().info('Nodo publicador iniciado en {}'.format(topic_name))
    self.count = 0

  def timer_callback(self):
    msg = ExampleMsg()
    msg.message = "Mensaje No. {}".format(self.count)
    msg.id = 0
    self.publisher.publish(msg)
    self.get_logger().info('Publicando mensaje: {}'.format(msg.message))
    self.count += 1

def main(args=None):
  rclpy.init(args=args)
  node = SimplePublisher()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()