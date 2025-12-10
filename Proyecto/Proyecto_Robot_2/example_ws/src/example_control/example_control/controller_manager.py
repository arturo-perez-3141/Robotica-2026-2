#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time

# Importamos la clase de cinemática que generamos anteriormente
# Asumimos que el archivo kinematics.py está en la misma carpeta del paquete
from .kinematics import RobotKinematics 

class ControlManager(Node):
  def __init__(self):
    super().__init__("control_manager")
    
    # --- 1. Inicializar y Calcular la Trayectoria ---
    self.get_logger().info("Inicializando Kinematics y calculando trayectoria...")
    self.robot_k = RobotKinematics()
    
    # Pasos de cálculo definidos en kinematics.py
    self.robot_k.direct_kinematics()
    
    # Definimos el movimiento deseado:
    # Desde q=[0,0,0] (Brazo estirado horizontalmente)
    # Hasta x=0.2, y=0.2, z=0.4 (Una posición arriba y a la izquierda)
    # Duración: 5 segundos
    self.robot_k.trajectory_generator(q_in=[0.0, 0.0, 0.0], xi_fn=[0.2, 0.2, 0.4], duration=5)
    
    # Resolver la cinemática inversa para obtener los ángulos de las juntas (q_m)
    self.robot_k.inverse_kinematics()
    
    self.get_logger().info(f"Trayectoria generada con {self.robot_k.samples} muestras.")

    # --- 2. Configuración de ROS ---
    
    # Publicador hacia el Hardware Interface (envía los objetivos de posición)
    self.hardware_command_publisher = self.create_publisher(
      JointState, "/joint_hardware_objectives", 10
    )

    # Suscriptor opcional (por si queremos ver el estado real, aunque no lo usamos para control aquí)
    self.joint_state_subscriber = self.create_subscription(
      JointState, "/joint_states", self.joint_state_callback, 10
    )

    # --- 3. Ejecución ---
    
    # Índice actual de la trayectoria
    self.current_sample = 0
    
    # Timer: Se ejecuta a la frecuencia definida en kinematics (30Hz)
    timer_period = self.robot_k.dt
    self.timer = self.create_timer(timer_period, self.control_loop)
    
    self.get_logger().info("Control Manager listo. Ejecutando movimiento...")

  def control_loop(self):
    # Verificamos si aún quedan puntos en la trayectoria
    if self.current_sample < self.robot_k.samples:
        
        # Crear mensaje JointState
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # IMPORTANTE: Nombres deben coincidir con el URDF y Hardware Interface
        msg.name = ["joint_1_waist", "joint_2_shoulder", "joint_3_elbow"]
        
        # Extraer los ángulos calculados para el instante actual
        # kinematics devuelve matrices de sympy/numpy, aseguramos float standard
        q1 = float(self.robot_k.q_m[0, self.current_sample])
        q2 = float(self.robot_k.q_m[1, self.current_sample])
        q3 = float(self.robot_k.q_m[2, self.current_sample])
        
        msg.position = [q1, q2, q3]
        
        # Publicar comando al hardware
        self.hardware_command_publisher.publish(msg)
        
        # Avanzar al siguiente punto
        self.current_sample += 1
        
    else:
        # Fin de la trayectoria
        # Opcional: Podrías reiniciar self.current_sample = 0 para hacer un bucle
        # O simplemente mantener la última posición
        pass

  def joint_state_callback(self, msg: JointState):
    # Aquí podríamos implementar un PID si quisiéramos comparar 
    # la posición deseada vs la real recibida del hardware.
    # Por ahora solo es monitoreo.
    pass

def main(args=None):
  try:
    rclpy.init(args=args)
    node = ControlManager()
    rclpy.spin(node)
  except KeyboardInterrupt:
    print("Node stopped")
  finally: 
    if 'node' in locals():
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
  main()