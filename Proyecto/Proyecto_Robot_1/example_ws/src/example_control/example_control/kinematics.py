#!/usr/bin/env python3
from sympy import *
import matplotlib.pyplot as plt
import numpy as np

class RobotKinematics():
  def __init__(self):
    pass

  def direct_kinematics(self):
    print("Definiendo variables del modelo en sympy (ROBOT PLANAR XY)")
    self.theta_0_1, self.theta_1_2, self.theta_2_3 = symbols("theta_0_1, theta_1_2, theta_2_3")
    self.l1 = 0.3
    self.l2 = 0.3
    self.l3 = 0.3

    # --- CONFIGURACIÓN PLANAR ---
    # T_0_1: Base fija en el plano, rotación solo en Z (theta_0_1)
    # xyz=(0,0,0) y rpy=(0,0,0) en la base para que no se incline
    self.T_0_1 = self.trans_homo(0, 0, 0, 0, 0, self.theta_0_1)
    
    # Resto de articulaciones (RRR planar)
    self.T_1_2 = self.trans_homo_xz(self.l1, 0, 0, self.theta_1_2)
    self.T_2_3 = self.trans_homo_xz(self.l2, 0, 0, self.theta_2_3)
    self.T_3_p = self.trans_homo_xz(self.l3, 0, 0, 0)
    
    T_0_p = simplify(self.T_0_1 * self.T_1_2 * self.T_2_3 * self.T_3_p)

    # --- COORDENADAS OPERACIONALES (X, Y, THETA) ---
    x_0_p = T_0_p[0, 3]
    y_0_p = T_0_p[1, 3] # Usamos Y porque estamos en el plano
    
    # Orientación final (suma de ángulos en Z)
    th_0_p = self.theta_0_1 + self.theta_1_2 + self.theta_2_3
    
    self.xi_0_p = Matrix([
      [x_0_p], 
      [y_0_p],
      [th_0_p]
    ])

    # Jacobiano
    self.J = Matrix.hstack(diff(self.xi_0_p, self.theta_0_1), 
                           diff(self.xi_0_p, self.theta_1_2), 
                           diff(self.xi_0_p, self.theta_2_3))
    
    self.J_inv = self.J.inv()

    # Vector de velocidades del espacio de trabajo
    self.x_dot, self.y_dot, self.th_dot = symbols("x_dot, y_dot, th_dot")
    self.xi_dot = Matrix([[self.x_dot], 
                          [self.y_dot], 
                          [self.th_dot]])
    
    # Ecuación de cinemática inversa: q_dot = J_inv * xi_dot
    self.q_dot = self.J_inv * self.xi_dot
    
    print("Definidas todas las variables")

  def trajectory_generator(self, q_in = [0.1, 0.1, 0.1], xi_fn = [0.5, 0.3, 1.57], duration = 4):
    self.freq = 30
    print("Definiendo trayectoria")
    
    # Polinomio de quinto orden
    self.t, a0, a1, a2, a3, a4, a5 = symbols("t, a0, a1, a2, a3, a4, a5")
    self.lam = a0 + a1*self.t + a2*(self.t)**2 + a3*(self.t)**3 + a4*(self.t)**4 + a5*(self.t)**5
    
    self.lam_dot = diff(self.lam, self.t)
    self.lam_dot_dot = diff(self.lam_dot, self.t)

    # Condiciones de frontera (Posición, Vel, Acel inicial y final)
    ec1 = self.lam.subs(self.t, 0)
    ec2 = self.lam.subs(self.t, duration) - 1
    ec3 = self.lam_dot.subs(self.t, 0)
    ec4 = self.lam_dot.subs(self.t, duration)
    ec5 = self.lam_dot_dot.subs(self.t, 0)
    ec6 = self.lam_dot_dot.subs(self.t, duration)
    
    terms = solve([ec1, ec2, ec3, ec4, ec5, ec6], [a0, a1, a2, a3, a4, a5], dict = True)
    self.lam_s          = self.lam.subs(terms[0])
    self.lam_dot_s      = self.lam_dot.subs(terms[0])
    self.lam_dot_dot_s  = self.lam_dot_dot.subs(terms[0])

    # Posición inicial calculada con FK usando q_in
    xi_in = self.xi_0_p.subs({
      self.theta_0_1: q_in[0],
      self.theta_1_2: q_in[1],
      self.theta_2_3: q_in[2]
    })
    
    # Ecuaciones de trayectoria (Línea recta en el espacio de trabajo)
    xi = xi_in + Matrix([
      [self.lam_s * (xi_fn[0] - xi_in[0])],
      [self.lam_s * (xi_fn[1] - xi_in[1])],
      [self.lam_s * (xi_fn[2] - xi_in[2])]
    ])
    
    xi_dot = Matrix([
      [self.lam_dot_s * (xi_fn[0] - xi_in[0])],
      [self.lam_dot_s * (xi_fn[1] - xi_in[1])],
      [self.lam_dot_s * (xi_fn[2] - xi_in[2])]
    ])
    
    xi_dot_dot = Matrix([
      [self.lam_dot_dot_s * (xi_fn[0] - xi_in[0])],
      [self.lam_dot_dot_s * (xi_fn[1] - xi_in[1])],
      [self.lam_dot_dot_s * (xi_fn[2] - xi_in[2])]
    ])

    # Preparar muestreo
    self.samples = int(self.freq * duration + 1)
    self.dt = 1/self.freq
    
    self.xi_m         = Matrix.zeros(3, self.samples)
    self.xi_dot_m     = Matrix.zeros(3, self.samples)
    self.xi_dot_dot_m = Matrix.zeros(3, self.samples)
    self.t_m = Matrix.zeros(1, self.samples)
    
    self.t_m[0, 0] = 0
    for a in range(self.samples - 1):
      self.t_m[0, a+1] = self.t_m[0, a] + self.dt

    # Lambdify para evaluación rápida
    xi_lam         = lambdify([self.t], xi, modules='numpy')
    xi_dot_lam     = lambdify([self.t], xi_dot, modules='numpy')
    xi_dot_dot_lam = lambdify([self.t], xi_dot_dot, modules='numpy')
    
    for a in range(self.samples):
        # Conversión segura a numpy array y asignación
        vals = xi_lam(float(self.t_m[0,a]))
        self.xi_m[:, a] = vals if isinstance(vals, np.ndarray) else np.array(vals)
        
        vals_d = xi_dot_lam(float(self.t_m[0,a]))
        self.xi_dot_m[:, a] = vals_d if isinstance(vals_d, np.ndarray) else np.array(vals_d)
        
        vals_dd = xi_dot_dot_lam(float(self.t_m[0,a]))
        self.xi_dot_dot_m[:, a] = vals_dd if isinstance(vals_dd, np.ndarray) else np.array(vals_dd)

    self.q_in = q_in

  def inverse_kinematics(self):
    print("Modelando cinemática inversa")
    self.q_m          = Matrix.zeros(3, self.samples)
    self.q_dot_m      = Matrix.zeros(3, self.samples)
    self.q_dot_dot_m  = Matrix.zeros(3, self.samples)
    
    # Condiciones iniciales de juntas
    self.q_m[:, 0]          = Matrix([[self.q_in[0]], [self.q_in[1]], [self.q_in[2]]])
    self.q_dot_m[:, 0]      = Matrix.zeros(3, 1)
    self.q_dot_dot_m[:, 0]  = Matrix.zeros(3, 1)
    
    # Lambdify de la Jacobiana Inversa
    self.q_dot_lam = lambdify([self.x_dot, self.y_dot, self.th_dot,
                               self.theta_0_1, self.theta_1_2, self.theta_2_3], self.q_dot, modules='numpy')
    
    for a in range(self.samples - 1):
      # Integración Euler: q_next = q_curr + q_dot * dt
      self.q_m[:, a+1] = self.q_m[:, a] + self.q_dot_m[:, a] * self.dt
      
      # Calcular Q_dot siguiente
      # OJO: self.xi_dot_m[1, a] es la velocidad en Y
      q_d_vals = self.q_dot_lam(float(self.xi_dot_m[0, a]), 
                                float(self.xi_dot_m[1, a]), 
                                float(self.xi_dot_m[2, a]),
                                float(self.q_m[0, a]), 
                                float(self.q_m[1, a]), 
                                float(self.q_m[2, a]))
      
      self.q_dot_m[:, a+1] = Matrix(q_d_vals)

      # Aceleración numérica
      self.q_dot_dot_m[:, a+1] = (self.q_dot_m[:, a+1] - self.q_dot_m[:, a]) / self.dt
      
    print("Trayectoria de las juntas generada")

  def ws_graph(self):
    fig, (p_g, v_g, a_g) = plt.subplots(nrows=1, ncols=3)
    fig.suptitle("Espacio de trabajo (X, Y, Theta)")
    
    p_g.set_title("Posiciones")
    p_g.plot(self.t_m.T, self.xi_m[0, :].T, color = "RED", label="X")
    p_g.plot(self.t_m.T, self.xi_m[1, :].T, color = "GREEN", label="Y")
    p_g.plot(self.t_m.T, self.xi_m[2, :].T, color = "BLUE", label="Th")
    p_g.legend()

    v_g.set_title("Velocidades")
    v_g.plot(self.t_m.T, self.xi_dot_m[0, :].T, color = "RED")
    v_g.plot(self.t_m.T, self.xi_dot_m[1, :].T, color = "GREEN")
    v_g.plot(self.t_m.T, self.xi_dot_m[2, :].T, color = "BLUE")

    a_g.set_title("Aceleraciones")
    a_g.plot(self.t_m.T, self.xi_dot_dot_m[0, :].T, color = "RED")
    a_g.plot(self.t_m.T, self.xi_dot_dot_m[1, :].T, color = "GREEN")
    a_g.plot(self.t_m.T, self.xi_dot_dot_m[2, :].T, color = "BLUE")
    
    plt.show()

  def q_graph(self):
    fig, (p_g, v_g, a_g) = plt.subplots(nrows=1, ncols=3)
    fig.suptitle("Espacio de las juntas")
    
    p_g.set_title("Posiciones")
    p_g.plot(self.t_m.T, self.q_m[0, :].T, color = "RED")
    p_g.plot(self.t_m.T, self.q_m[1, :].T, color = "GREEN")
    p_g.plot(self.t_m.T, self.q_m[2, :].T, color = "BLUE")

    v_g.set_title("Velocidades")
    v_g.plot(self.t_m.T, self.q_dot_m[0, :].T, color = "RED")
    v_g.plot(self.t_m.T, self.q_dot_m[1, :].T, color = "GREEN")
    v_g.plot(self.t_m.T, self.q_dot_m[2, :].T, color = "BLUE")

    a_g.set_title("Aceleraciones")
    a_g.plot(self.t_m.T, self.q_dot_dot_m[0, :].T, color = "RED")
    a_g.plot(self.t_m.T, self.q_dot_dot_m[1, :].T, color = "GREEN")
    a_g.plot(self.t_m.T, self.q_dot_dot_m[2, :].T, color = "BLUE")
    
    plt.show()

  def simple_graph(self, val_m, t_m):
    plt.plot(t_m.T, val_m[0, :].T)
    plt.show()
  
  def trans_homo_xz(self, x=0, z=0, gamma=0, alpha=0)->Matrix:
    R_z = Matrix([ [cos(alpha), -sin(alpha), 0], [sin(alpha), cos(alpha), 0],[0, 0, 1]])
    R_x = Matrix([ [1, 0, 0], [0, cos(gamma), -sin(gamma)],[0, sin(gamma), cos(gamma)]])

    p_x = Matrix([[x],[0],[0]])
    p_z = Matrix([[0],[0],[z]])

    T_x = Matrix.vstack(Matrix.hstack(R_x, p_x), Matrix([[0,0,0,1]]))
    T_z = Matrix.vstack(Matrix.hstack(R_z, p_z), Matrix([[0,0,0,1]]))
    return T_x * T_z
  
  def trans_homo(self, x, y, z, gamma, beta, alpha):
    R_z = Matrix([ [cos(alpha), -sin(alpha), 0], [sin(alpha), cos(alpha), 0],[0, 0, 1]])
    R_y = Matrix([ [cos(beta), 0, sin(beta)], [0, 1, 0],[-sin(beta), 0, cos(beta)]])
    R_x = Matrix([ [1, 0, 0], [0, cos(gamma), -sin(gamma)],[0, sin(gamma), cos(gamma)]])

    p_x = Matrix([[x],[0],[0]])
    p_y = Matrix([[0],[y],[0]])
    p_z = Matrix([[0],[0],[z]])
    
    T_x = Matrix.vstack(Matrix.hstack(R_x, p_x), Matrix([[0,0,0,1]]))
    T_y = Matrix.vstack(Matrix.hstack(R_y, p_y), Matrix([[0,0,0,1]]))
    T_z = Matrix.vstack(Matrix.hstack(R_z, p_z), Matrix([[0,0,0,1]]))
    return T_x * T_y * T_z

  # --- LA FUNCIÓN QUE FALTABA ---
  def redirect_print(self, new_print):
    global print
    print = new_print
  # ------------------------------

def main():
  robot = RobotKinematics()
  robot.direct_kinematics()
  # Objetivo de prueba alcanzable en el plano
  robot.trajectory_generator(xi_fn = [0.5, 0.3, 1.57])
  robot.inverse_kinematics()
  robot.ws_graph()
  robot.q_graph()

if __name__ == "__main__":
  main()
