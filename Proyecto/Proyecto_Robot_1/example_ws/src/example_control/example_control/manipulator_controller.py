#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, PointStamped
from .kinematics import RobotKinematics
from .dynamics import RobotDynamics

class ManipulatorController(Node):
    def __init__(self):
        super().__init__("manipulator_controller")
        # Crear un objeto de robot
        self.robot_kinematics = RobotKinematics()
        self.robot_kinematics.redirect_print(self.get_logger().info)
        self.robot_kinematics.direct_kinematics()
        
        self.robot_dynamics = RobotDynamics()
        self.robot_dynamics.define_kinematics(self.robot_kinematics)
        self.robot_dynamics.define_dynamics() 

        # Variable de control para definir si hay una trayectoria activa
        self.moving = False
        self.current_joint_states = None 

        # Perfil de calidad de información
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, 
                                 history=HistoryPolicy.KEEP_LAST,
                                 depth=10)

        # Recibir información de una posición deseada
        self.end_effector_goal_subscriber = self.create_subscription(
            Twist, "/end_effector_goal", self.end_effector_callback, 10
        )

        # Recibir información de una posición clickeada en rviz
        self.clicked_point_subscriber = self.create_subscription(
            PointStamped, "/clicked_point", self.clicked_point_callback, 10
        )

        # Recibr información de posición actual de las juntas
        self.joint_states_subscriber = self.create_subscription(
            JointState, "/joint_states", self.joint_states_callback, 10
        )

        # Enviar información de las juntas al controller manager
        self.joint_goals_publisher = self.create_publisher(
            JointState, "/joint_goals", 10
        )
        self.get_logger().info("Controlador inicializado")

    def get_ordered_current_joints(self):
        """
        Asegura leer las juntas en el orden correcto (hombro, brazo, antebrazo).
        """
        if self.current_joint_states is None:
            return None
            
        names = self.current_joint_states.name
        positions = self.current_joint_states.position
        
        try:
            th1 = positions[names.index("shoulder_joint")]
            th2 = positions[names.index("arm_joint")]
            th3 = positions[names.index("forearm_joint")]
            return [th1, th2, th3]
        except ValueError:
            self.get_logger().warning("Nombres de juntas no coinciden en /joint_states")
            return None

    def end_effector_callback(self, msg:Twist):
        if self.moving:
            self.get_logger().warning("Trayectoria en progreso. Mensaje rechazado")
            return
        
        q_in = self.get_ordered_current_joints()
        if q_in is None:
            self.get_logger().warning("Aún no se han recibido estados de las juntas")
            return

        self.moving = True
        self.get_logger().info("Punto objetivo recibido (Twist)")
        
        # Mapeo corregido para Planar
        target_pose = [msg.linear.x, msg.linear.y, msg.angular.z]

        self.robot_kinematics.trajectory_generator(q_in, target_pose, 3)
        self.robot_kinematics.inverse_kinematics()
        self.start_publishing_trajectory()

    def clicked_point_callback(self, msg:PointStamped):
        if self.moving:
            self.get_logger().warning("Trayectoria en progreso. Mensaje rechazado")
            return
        
        q_in = self.get_ordered_current_joints()
        if q_in is None:
            self.get_logger().warning("Aún no se han recibido estados de las juntas")
            return

        self.moving = True
        self.get_logger().info(f"Clic recibido en: X={msg.point.x:.2f}, Y={msg.point.y:.2f}")
        
        # Mapeo corregido: Usamos Y en lugar de Z
        target_pose = [msg.point.x, msg.point.y, 0.0]
        
        # Generar trayectoria
        self.robot_kinematics.trajectory_generator(q_in, target_pose, 3)
        self.robot_kinematics.inverse_kinematics()
        
        # Dinámica y Gráficas (ACTIVADAS Y BLOQUEANTES)
        try:
            self.robot_dynamics.lagrange_effort_generator()
            
            # Al llamar a estas funciones, el código se detendrá aquí 
            # hasta que cierres las ventanas que aparecen.
            print("Mostrando gráficas... Cierra las ventanas para iniciar el movimiento.")
            self.robot_kinematics.ws_graph()
            self.robot_kinematics.q_graph()
            self.robot_dynamics.effort_graph()
            
        except Exception as e:
            self.get_logger().error(f"Error en dinámica o gráficas: {e}")

        # El movimiento comienza SOLO después de cerrar las gráficas
        self.start_publishing_trajectory()

    def start_publishing_trajectory(self):
        self.count = 0
        self.joint_goals = JointState()
        self.joint_goals.name = ["shoulder_joint", "arm_joint", "forearm_joint"]
        self.get_logger().info("Publicando trayectoria de las juntas")
        self.position_publisher_timer = self.create_timer(self.robot_kinematics.dt, 
                                                          self.trayectory_publisher_callback)

    def trayectory_publisher_callback(self):
        self.joint_goals.header.stamp = self.get_clock().now().to_msg()
        
        # Obtener valores
        th1 = float(self.robot_kinematics.q_m[0, self.count])
        th2 = float(self.robot_kinematics.q_m[1, self.count])
        th3 = float(self.robot_kinematics.q_m[2, self.count])
        
        self.joint_goals.position = [th1, th2, th3]
        self.joint_goals_publisher.publish(self.joint_goals)
        
        self.count += 1
        if (self.count >= self.robot_kinematics.samples):
            self.count = 0
            self.position_publisher_timer.cancel()
            self.position_publisher_timer = None
            self.get_logger().info("Trayectoria finalizada")
            self.moving = False
     
    def joint_states_callback(self, msg:JointState):
        self.current_joint_states = msg

def main(args=None):
    try:
        rclpy.init(args=args)
        node = ManipulatorController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Node stopped by user")
    except Exception as e:
        print(f"Error en runtime: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
