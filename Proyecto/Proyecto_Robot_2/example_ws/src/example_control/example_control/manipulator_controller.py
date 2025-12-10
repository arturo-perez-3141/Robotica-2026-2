#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, PointStamped
import numpy as np

# Importamos nuestros módulos
from example_control.kinematics import RobotKinematics
from example_control.dynamics import RobotDynamics

class ManipulatorController(Node):
  def __init__(self):
    super().__init__("manipulator_controller")
    
    # --- 1. CONFIGURACIÓN RÁPIDA (Solo lo vital para aparecer en Rviz) ---
    self.moving = False
    self.count = 0
    self.dynamics_loaded = False # Bandera para saber si ya calculamos la física
    
    # POSICIÓN HOME: Codo Arriba [-1.57]
    self.current_joint_states = JointState()
    self.current_joint_states.position = [0.2, 0.2, 0.2] 
    self.joint_names = ["joint_1_waist", "joint_2_shoulder", "joint_3_elbow"]

    # --- 2. Inicializar Objetos (PERO NO CALCULAR TODAVÍA) ---
    self.get_logger().info("Iniciando componentes básicos...")
    self.robot_kinematics = RobotKinematics()
    
    # Calculamos solo la cinemática (es rápido y necesario para moverse)
    self.robot_kinematics.redirect_print(self.get_logger().info)
    self.robot_kinematics.direct_kinematics()
    
    # Instanciamos Dinámica pero NO llamamos a define_dynamics() todavía (es lo lento)
    self.robot_dynamics = RobotDynamics()
    self.robot_dynamics.define_kinematics(self.robot_kinematics)
    
    # --- 3. Comunicaciones ---
    qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
    
    self.end_effector_goal_subscriber = self.create_subscription(Twist, "/end_effector_goal", self.end_effector_callback, 10)
    self.clicked_point_subscriber = self.create_subscription(PointStamped, "/clicked_point", self.clicked_point_callback, 10)
    self.joint_publisher = self.create_publisher(JointState, "/joint_states", 10)
    
    # --- 4. ENVÍO INMEDIATO ("EL GRITO") ---
    # Publicamos YA, sin esperar a nada.
    init_msg = JointState()
    init_msg.header.stamp = self.get_clock().now().to_msg()
    init_msg.name = self.joint_names
    init_msg.position = self.current_joint_states.position
    self.joint_publisher.publish(init_msg)

    # --- 5. Timer de Alta Velocidad (100 Hz) ---
    self.dt = 0.01 
    self.timer = self.create_timer(self.dt, self.control_loop)
    
    self.get_logger().info("¡ROBOT LISTO! (La dinámica se cargará al primer clic)")

  def control_loop(self):
    msg = JointState()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.name = self.joint_names
    
    if self.moving:
        # Modo Trayectoria
        th1 = float(self.robot_kinematics.q_m[0, self.count])
        th2 = float(self.robot_kinematics.q_m[1, self.count])
        th3 = float(self.robot_kinematics.q_m[2, self.count])
        
        msg.position = [th1, th2, th3]
        
        self.count += 1
        if self.count >= self.robot_kinematics.samples:
            self.moving = False
            self.count = 0
            self.current_joint_states.position = [th1, th2, th3]
            self.get_logger().info("Llegamos al destino.")
    else:
        # Modo Reposo
        msg.position = self.current_joint_states.position
        
    self.joint_publisher.publish(msg)

  def ensure_dynamics_loaded(self):
    """ Función auxiliar para cargar la matemática pesada solo cuando se necesite """
    if not self.dynamics_loaded:
        self.get_logger().warn("⚠️ Cargando Dinámica por primera vez (Esto tomará unos segundos)...")
        self.robot_dynamics.define_dynamics()
        self.dynamics_loaded = True
        self.get_logger().info("✅ Dinámica Cargada. Continuando movimiento.")

  def clicked_point_callback(self, msg: PointStamped):
    if self.moving:
      self.get_logger().warning("Robot ocupado.")
      return
    
    # 1. CARGA PEREZOSA: Aquí es donde pagamos el precio del cálculo, no al inicio.
    self.ensure_dynamics_loaded()
    
    target_pos = [msg.point.x, msg.point.y, msg.point.z]
    self.get_logger().info(f"Calculando ruta al punto: {target_pos}")
    
    start_pos = self.current_joint_states.position
    
    # Matemáticas de movimiento
    self.robot_kinematics.trajectory_generator(q_in=start_pos, xi_fn=target_pos, duration=3)
    self.robot_kinematics.inverse_kinematics()
    self.robot_dynamics.lagrange_effort_generator()
    
    # Gráficas
    self.get_logger().info(">>> MOSTRANDO GRÁFICAS... CIERRE PARA CONTINUAR <<<")
    self.robot_kinematics.ws_graph()
    self.robot_kinematics.q_graph()
    self.robot_dynamics.effort_graph()
    
    self.count = 0
    self.moving = True

  def end_effector_callback(self, msg: Twist):
    if self.moving: return
    
    # También cargamos aquí por si usan este método primero
    self.ensure_dynamics_loaded()
    
    target_pos = [msg.linear.x, msg.linear.y, msg.linear.z]
    start_pos = self.current_joint_states.position
    self.robot_kinematics.trajectory_generator(q_in=start_pos, xi_fn=target_pos, duration=3)
    self.robot_kinematics.inverse_kinematics()
    self.count = 0
    self.moving = True

def main(args=None):
  try:
    rclpy.init(args=args)
    node = ManipulatorController()
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    if 'node' in locals(): node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
  main()
