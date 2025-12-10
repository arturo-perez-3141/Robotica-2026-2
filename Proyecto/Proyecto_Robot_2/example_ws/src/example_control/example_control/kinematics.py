#!/usr/bin/env python3
from sympy import *
import matplotlib.pyplot as plt
import numpy as np 

class RobotKinematics():
  def __init__(self):
    pass

  def direct_kinematics(self):
    print("Definiendo variables del modelo en sympy (Robot Antropomórfico 3GDL)")
    self.theta_0_1, self.theta_1_2, self.theta_2_3 = symbols("theta_0_1, theta_1_2, theta_2_3")
    
    # --- Dimensiones ---
    self.l1 = 0.05  
    self.l2 = 0.20  
    self.l3 = 0.35  

    # --- Transformaciones Homogéneas ---
    t_0_1_pos = self.trans_homo(0, 0, self.l1, 0, 0, 0)
    t_0_1_rot = self.trans_homo(0, 0, 0, 0, 0, self.theta_0_1)
    self.T_0_1 = t_0_1_pos * t_0_1_rot

    t_1_2_rot = self.trans_homo(0, 0, 0, 0, self.theta_1_2, 0)
    t_1_2_pos = self.trans_homo(self.l2, 0, 0, 0, 0, 0)
    self.T_1_2 = t_1_2_rot * t_1_2_pos

    t_2_3_rot = self.trans_homo(0, 0, 0, 0, self.theta_2_3, 0)
    t_2_3_pos = self.trans_homo(self.l3, 0, 0, 0, 0, 0)
    self.T_2_3 = t_2_3_rot * t_2_3_pos

    # Transformación total
    T_0_p = simplify(self.T_0_1 * self.T_1_2 * self.T_2_3)

    # Vector del Espacio de Trabajo
    x_0_p = T_0_p[0, 3]
    y_0_p = T_0_p[1, 3]
    z_0_p = T_0_p[2, 3]
    
    self.xi_0_p = Matrix([[x_0_p], [y_0_p], [z_0_p]])

    # Jacobiano Analítico
    self.J = Matrix.hstack(diff(self.xi_0_p, self.theta_0_1), 
                           diff(self.xi_0_p, self.theta_1_2), 
                           diff(self.xi_0_p, self.theta_2_3))
    
    print("Compilando función del Jacobiano (lambdify)...")
    self.J_lam = lambdify([self.theta_0_1, self.theta_1_2, self.theta_2_3], self.J, modules='numpy')

    print("Definidas todas las variables")

  def trajectory_generator(self, q_in = [0.01, 0.01, 0.01], xi_fn = [0.4, 0.1, 0.3], duration = 4):
    self.freq = 30
    print("Definiendo trayectoria")
    
    self.t, a0, a1, a2, a3, a4, a5 = symbols("t, a0, a1, a2, a3, a4, a5")
    self.lam = a0 + a1*self.t + a2*(self.t)**2 + a3*(self.t)**3 + a4*(self.t)**4 + a5*(self.t)**5
    
    self.lam_dot = diff(self.lam, self.t)
    self.lam_dot_dot = diff(self.lam_dot, self.t)

    # Condiciones de frontera
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

    # Posición inicial basada en q_in
    q_in = [float(val) for val in q_in]
    
    xi_in_eval = self.xi_0_p.subs({
      self.theta_0_1: q_in[0],
      self.theta_1_2: q_in[1],
      self.theta_2_3: q_in[2]
    })
    
    xi_in_np = np.array(xi_in_eval).astype(np.float64)

    # Trayectoria cartesiana (línea recta)
    lam_func = lambdify(self.t, self.lam_s, modules='numpy')
    lam_dot_func = lambdify(self.t, self.lam_dot_s, modules='numpy')
    lam_ddot_func = lambdify(self.t, self.lam_dot_dot_s, modules='numpy')

    self.samples = int(self.freq * duration + 1)
    self.dt = 1/self.freq
    
    self.xi_m          = np.zeros((3, self.samples))
    self.xi_dot_m      = np.zeros((3, self.samples))
    self.xi_dot_dot_m  = np.zeros((3, self.samples))
    self.t_m           = np.zeros((1, self.samples))

    # Generar vector de tiempo
    for a in range(self.samples):
        curr_t = a * self.dt
        self.t_m[0, a] = curr_t
        
        l = lam_func(curr_t)
        ld = lam_dot_func(curr_t)
        ldd = lam_ddot_func(curr_t)
        
        # Interpolación p = p_ini + lambda * (p_fin - p_ini)
        delta = (np.array(xi_fn).reshape(3,1) - xi_in_np)
        
        self.xi_m[:, a] = (xi_in_np + l * delta).flatten()
        self.xi_dot_m[:, a] = (ld * delta).flatten()
        self.xi_dot_dot_m[:, a] = (ldd * delta).flatten()

    self.q_in = q_in

  def inverse_kinematics(self):
    print("Modelando cinemática inversa (Jacobian Pseudo-Inverse con Límites)")
    
    self.q_m          = np.zeros((3, self.samples))
    self.q_dot_m      = np.zeros((3, self.samples))
    self.q_dot_dot_m  = np.zeros((3, self.samples))

    # Condiciones iniciales
    self.q_m[:, 0] = self.q_in
    
    # --- DEFINICIÓN DE LÍMITES DE ARTICULACIÓN (RADIANES) ---
    # Min / Max para [Cintura, Hombro, Codo]
    limits_min = np.array([-3.14, -2.5, -2.5]) 
    limits_max = np.array([ 3.14,  2.5,  2.5])
    
    print("Calculando trayectoria de juntas (Iterativo)...")
    for a in range(self.samples - 1):
      # 1. Obtener estado actual
      q_curr = self.q_m[:, a]
      
      # 2. Matriz Jacobiana numérica
      J_val = np.array(self.J_lam(q_curr[0], q_curr[1], q_curr[2])).astype(np.float64)
      
      # 3. Velocidad cartesiana deseada
      v_cartesian = self.xi_dot_m[:, a+1].reshape(3,1)
      
      # 4. Resolver q_dot = J_pinv * v_cartesian
      try:
          J_pinv = np.linalg.pinv(J_val)
          q_dot_calc = np.dot(J_pinv, v_cartesian).flatten()
      except np.linalg.LinAlgError:
          print(f"Error de álgebra lineal en iteración {a}")
          q_dot_calc = np.zeros(3)

      # 5. Integración y APLICACIÓN DE LÍMITES (Clamping)
      q_next_raw = q_curr + q_dot_calc * self.dt
      
      # Clip: Si se pasa del límite, lo dejamos en el borde
      q_next_clamped = np.clip(q_next_raw, limits_min, limits_max)
      
      self.q_m[:, a+1] = q_next_clamped
      
      # Ajuste de velocidad: Si chocamos con el límite, la velocidad real es 0
      # (Esto ayuda a que la dinámica no calcule fuerzas infinitas contra el muro)
      real_q_dot = q_dot_calc
      for i in range(3):
          # Si fue recortado (es diferente al raw), entonces chocó
          if abs(q_next_clamped[i] - q_next_raw[i]) > 1e-5:
              real_q_dot[i] = 0.0

      self.q_dot_m[:, a+1] = real_q_dot
      
      # 6. Aceleración
      if a > 0:
          acc = (self.q_dot_m[:, a+1] - self.q_dot_m[:, a]) / self.dt
          self.q_dot_dot_m[:, a+1] = acc

    # Convertir a Matrix de Sympy para compatibilidad final
    self.q_m = Matrix(self.q_m)
    self.q_dot_m = Matrix(self.q_dot_m)
    self.q_dot_dot_m = Matrix(self.q_dot_dot_m)
      
    print("Trayectoria de las juntas generada (Límites aplicados).")

  def ws_graph(self):
    t_plot = np.array(self.t_m).flatten()
    xi_plot = np.array(self.xi_m).astype(float)
    xid_plot = np.array(self.xi_dot_m).astype(float)
    xidd_plot = np.array(self.xi_dot_dot_m).astype(float)

    fig, (p_g, v_g, a_g) = plt.subplots(nrows=1, ncols=3, figsize=(15, 5))
    fig.suptitle("Espacio de Trabajo (Cartesiano)")
    
    p_g.set_title("Posiciones (X, Y, Z)")
    p_g.plot(t_plot, xi_plot[0, :], 'r', label="X")
    p_g.plot(t_plot, xi_plot[1, :], 'g', label="Y")
    p_g.plot(t_plot, xi_plot[2, :], 'b', label="Z")
    p_g.legend()
    p_g.grid()

    v_g.set_title("Velocidades")
    v_g.plot(t_plot, xid_plot[0, :], 'r')
    v_g.plot(t_plot, xid_plot[1, :], 'g')
    v_g.plot(t_plot, xid_plot[2, :], 'b')
    v_g.grid()

    a_g.set_title("Aceleraciones")
    a_g.plot(t_plot, xidd_plot[0, :], 'r')
    a_g.plot(t_plot, xidd_plot[1, :], 'g')
    a_g.plot(t_plot, xidd_plot[2, :], 'b')
    a_g.grid()
    
    plt.tight_layout()
    plt.show()

  def q_graph(self):
    t_plot = np.array(self.t_m).flatten()
    q_plot = np.array(self.q_m).astype(float)
    qd_plot = np.array(self.q_dot_m).astype(float)
    qdd_plot = np.array(self.q_dot_dot_m).astype(float)

    fig, (p_g, v_g, a_g) = plt.subplots(nrows=1, ncols=3, figsize=(15, 5))
    fig.suptitle("Espacio de las Juntas (Joint Space)")
    
    p_g.set_title("Posiciones (q1, q2, q3)")
    p_g.plot(t_plot, q_plot[0, :], 'r', label="q1")
    p_g.plot(t_plot, q_plot[1, :], 'g', label="q2")
    p_g.plot(t_plot, q_plot[2, :], 'b', label="q3")
    p_g.legend()
    p_g.grid()

    v_g.set_title("Velocidades")
    v_g.plot(t_plot, qd_plot[0, :], 'r')
    v_g.plot(t_plot, qd_plot[1, :], 'g')
    v_g.plot(t_plot, qd_plot[2, :], 'b')
    v_g.grid()

    a_g.set_title("Aceleraciones")
    a_g.plot(t_plot, qdd_plot[0, :], 'r')
    a_g.plot(t_plot, qdd_plot[1, :], 'g')
    a_g.plot(t_plot, qdd_plot[2, :], 'b')
    a_g.grid()
    
    plt.tight_layout()
    plt.show()

  def trans_homo(self, x, y, z, gamma, beta, alpha):
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
  
  def redirect_print(self, new_print):
    global print
    print = new_print

def main():
  robot = RobotKinematics()
  robot.direct_kinematics()
  
  # Posición segura para evitar división por cero
  start_pos = [0.0, 0.0, 1.57] 
  
  robot.trajectory_generator(q_in=start_pos, xi_fn=[0.3, 0.2, 0.3], duration=3)
  robot.inverse_kinematics()
  robot.ws_graph()
  robot.q_graph()

if __name__ == "__main__":
  main()
