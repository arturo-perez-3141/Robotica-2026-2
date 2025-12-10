#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class HardwareInterface(Node):
  def __init__(self):
    super().__init__("hardware_interface")
    
    # Suscriptor de comandos
    self.joint_hardware_objectives_subscriber = self.create_subscription(
      JointState, "/joint_hardware_objectives", self.hardware_obj_callback, 10
    )
    
    # Publicador de estado
    self.joint_states_publisher = self.create_publisher(
      JointState, "/joint_states", 10
    )
    
    # --- ESTADO INICIAL ---
    self.current_joint_state = JointState()
    # NOMBRES DEBEN SER EXACTOS AL URDF
    self.current_joint_state.name = ["joint_1_waist", "joint_2_shoulder", "joint_3_elbow"]
    # POSICIÓN INICIAL SEGURA (No usar ceros para evitar singularidad inicial visual)
    self.current_joint_state.position = [0.2, 0.2, 0.2] 
    
    self.create_timer(0.1, self.joint_states_timer_callback)
    self.get_logger().info("Hardware Interface Iniciada OK")

  def hardware_obj_callback(self, msg: JointState):
    # Actualizar estado simulado
    if len(msg.position) == 3:
        self.current_joint_state.position = msg.position

  def joint_states_timer_callback(self):
    self.current_joint_state.header.stamp = self.get_clock().now().to_msg()
    # Asegurar nombres siempre
    self.current_joint_state.name = ["joint_1_waist", "joint_2_shoulder", "joint_3_elbow"]
    self.joint_states_publisher.publish(self.current_joint_state)

def main(args=None):
  try:
    rclpy.init(args=args)
    node = HardwareInterface()
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally: 
    if 'node' in locals():
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
  main()
