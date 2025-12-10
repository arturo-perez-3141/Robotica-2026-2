#!/usr/bin/env python3
from sympy import *
from example_control.kinematics import RobotKinematics 
import matplotlib.pyplot as plt
import numpy as np

class RobotDynamics():
  def __init__(self):
    self.tau_f = None 

  def define_kinematics(self, kinematics:RobotKinematics):
    self.kinematics = kinematics

  def define_dynamics(self, mass = [0.25, 0.25, 0.25]):
    print("--- INICIANDO CÁLCULO DINÁMICO SIMBÓLICO ---")
    
    # Asegurarnos de que las variables simbólicas existen
    if not hasattr(self.kinematics, 'theta_0_1'):
        theta_0_1, theta_1_2, theta_2_3 = symbols("theta_0_1, theta_1_2, theta_2_3")
        self.kinematics.theta_0_1 = theta_0_1
        self.kinematics.theta_1_2 = theta_1_2
        self.kinematics.theta_2_3 = theta_2_3

    # Variables locales para escribir menos
    l1, l2, l3 = self.kinematics.l1, self.kinematics.l2, self.kinematics.l3
    th1, th2, th3 = self.kinematics.theta_0_1, self.kinematics.theta_1_2, self.kinematics.theta_2_3

    # --- RECONSTRUCCIÓN SIMBÓLICA ---
    # Usamos funciones auxiliares para evitar contaminación de namespaces
    T_0_1 = self.trans_homo_sym(0, 0, l1, 0, 0, th1)
    T_1_2 = self.trans_homo_sym(l2, 0, 0, 0, th2, 0)
    T_2_3 = self.trans_homo_sym(l3, 0, 0, 0, th3, 0)

    T_0_2 = T_0_1 * T_1_2
    T_0_3 = T_0_2 * T_2_3

    # Centros de masa
    T_1_C1 = self.trans_homo_sym(0, 0, l1 / 2, 0, 0, 0)
    T_2_C2 = self.trans_homo_sym(l2 / 2, 0, 0, 0, 0, 0)
    T_3_C3 = self.trans_homo_sym(l3 / 2, 0, 0, 0, 0, 0)

    T_0_C1 = simplify(T_0_1 * T_1_C1)
    T_0_C2 = simplify(T_0_2 * T_2_C2)
    T_0_C3 = simplify(T_0_3 * T_3_C3)

    # Matrices de Rotación (locales para velocidad angular iterativa)
    R_0_1_loc = T_0_1[:3, :3]
    R_1_2_loc = self.trans_homo_sym(l2, 0, 0, 0, th2, 0)[:3, :3] # Solo rotación local J2
    R_2_3_loc = self.trans_homo_sym(l3, 0, 0, 0, th3, 0)[:3, :3] # Solo rotación local J3

    p_0_C1 = T_0_C1[:3, 3]
    p_0_C2 = T_0_C2[:3, 3]
    p_0_C3 = T_0_C3[:3, 3]
    
    # Velocidades Simbólicas
    theta_0_1_dot, theta_1_2_dot, theta_2_3_dot = symbols('theta_0_1_dot theta_1_2_dot theta_2_3_dot')
    theta_0_1_dot_dot, theta_1_2_dot_dot, theta_2_3_dot_dot = symbols('theta_0_1_dot_dot theta_1_2_dot_dot theta_2_3_dot_dot')

    # Inercias
    m1, m2, m3 = mass
    Ic1 = self.inertia_tensor(0.05, 0.05, l1, m1) 
    Ic2 = self.inertia_tensor(l2, 0.04, 0.04, m2)
    Ic3 = self.inertia_tensor(l3, 0.03, 0.03, m3)
    g = 9.81

    # --- Velocidades Angulares ---
    omega_0_0 = Matrix([[0], [0], [0]])
    omega_1_1 = R_0_1_loc.transpose() * (omega_0_0 + Matrix([[0], [0], [theta_0_1_dot]]))
    omega_2_2 = R_1_2_loc.transpose() * (omega_1_1 + Matrix([[0], [theta_1_2_dot], [0]]))
    omega_3_3 = R_2_3_loc.transpose() * (omega_2_2 + Matrix([[0], [theta_2_3_dot], [0]]))

    print("   ... Calculando Jacobianos de velocidad ...")
    q = [th1, th2, th3]
    q_dot = [theta_0_1_dot, theta_1_2_dot, theta_2_3_dot]
    
    v_1_C1 = p_0_C1.jacobian(q) * Matrix(q_dot)
    v_2_C2 = p_0_C2.jacobian(q) * Matrix(q_dot)
    v_3_C3 = p_0_C3.jacobian(q) * Matrix(q_dot)

    print("   ... Construyendo Lagrangiano ...")
    k1 = 0.5 * m1 * v_1_C1.dot(v_1_C1) + 0.5 * omega_1_1.dot(Ic1 * omega_1_1)
    k2 = 0.5 * m2 * v_2_C2.dot(v_2_C2) + 0.5 * omega_2_2.dot(Ic2 * omega_2_2)
    k3 = 0.5 * m3 * v_3_C3.dot(v_3_C3) + 0.5 * omega_3_3.dot(Ic3 * omega_3_3)
    K = k1 + k2 + k3

    U = m1 * g * p_0_C1[2] + m2 * g * p_0_C2[2] + m3 * g * p_0_C3[2]
    La = K - U

    print("   ... Derivando Ecuaciones de Movimiento ...")
    q_vec = Matrix(q)
    q_dot_vec = Matrix(q_dot)
    q_ddot_vec = Matrix([theta_0_1_dot_dot, theta_1_2_dot_dot, theta_2_3_dot_dot])

    dL_dq_dot = La.diff(q_dot_vec)
    dL_dq = La.diff(q_vec)
    
    ddt_dL_dq_dot = dL_dq_dot.jacobian(q_vec) * q_dot_vec + dL_dq_dot.jacobian(q_dot_vec) * q_ddot_vec
    tau = ddt_dL_dq_dot - dL_dq

    print("   ... Compilando función numérica (lambdify) ...")
    # CORRECCIÓN 1: Forzamos el backend de numpy explícitamente para evitar ambigüedad con 'sin'
    self.tau_f = lambdify(
        [th1, th2, th3,
         theta_0_1_dot, theta_1_2_dot, theta_2_3_dot,
         theta_0_1_dot_dot, theta_1_2_dot_dot, theta_2_3_dot_dot], 
        tau, 
        modules="numpy"
    )
    print("--- DINÁMICA LISTA ---")

  def lagrange_effort_generator(self):
    if self.tau_f is None:
        print("Error: Dinámica no definida.")
        return

    self.tau_m = np.zeros((3, self.kinematics.samples))
    print(f"Calculando torque para {self.kinematics.samples} muestras...")
    
    q = self.kinematics.q_m
    qd = self.kinematics.q_dot_m
    qdd = self.kinematics.q_dot_dot_m
    
    for i in range(self.kinematics.samples):
        # CORRECCIÓN 2: Convertimos explícitamente a float nativo de Python.
        # Esto elimina cualquier residuo de tipos SymPy o NumPy escalares que causan el error.
        t_val = self.tau_f(
            float(q[0, i]), float(q[1, i]), float(q[2, i]),
            float(qd[0, i]), float(qd[1, i]), float(qd[2, i]),
            float(qdd[0, i]), float(qdd[1, i]), float(qdd[2, i])
        )
        # Convertimos el resultado a array plano
        self.tau_m[:, i] = np.array(t_val).flatten()

  def effort_graph(self):
    fig, ((tau_1_g, tau_2_g, tau_3_g)) = plt.subplots(nrows=1, ncols = 3, figsize=(15, 5))
    fig.suptitle("Pares (Torque)")
    
    # Aplanamos el tiempo
    t = np.array(self.kinematics.t_m).flatten()
    
    tau1 = self.tau_m[0, :]
    tau2 = self.tau_m[1, :]
    tau3 = self.tau_m[2, :]

    tau_1_g.plot(t, tau1, 'r'); tau_1_g.set_title("Junta 1")
    tau_2_g.plot(t, tau2, 'g'); tau_2_g.set_title("Junta 2")
    tau_3_g.plot(t, tau3, 'b'); tau_3_g.set_title("Junta 3")
    
    plt.tight_layout()
    plt.show()

  def trans_homo_sym(self, x, y, z, gamma, beta, alpha):
    # Usamos cos/sin de sympy porque 'from sympy import *'
    R_z = Matrix([[cos(alpha), -sin(alpha), 0, 0], 
                  [sin(alpha), cos(alpha),  0, 0],
                  [0,          0,           1, 0],
                  [0,          0,           0, 1]])
    R_y = Matrix([[cos(beta),  0, sin(beta), 0], 
                  [0,          1, 0,         0],
                  [-sin(beta), 0, cos(beta), 0],
                  [0,          0, 0,         1]])
    R_x = Matrix([[1, 0,            0,             0], 
                  [0, cos(gamma), -sin(gamma),    0],
                  [0, sin(gamma),  cos(gamma),    0],
                  [0, 0,            0,             1]])
    
    T_pos = Matrix([[1, 0, 0, x],
                    [0, 1, 0, y],
                    [0, 0, 1, z],
                    [0, 0, 0, 1]])
    return R_x * R_y * R_z * T_pos

  def inertia_tensor(self, lx, ly, lz, mass):
    return Matrix([[(mass/12.0)*(ly**2 + lz**2), 0, 0], 
                   [0, (mass/12.0)*(lx**2 + lz**2), 0], 
                   [0, 0, (mass/12.0)*(lx**2 + ly**2)]])
