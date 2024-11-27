import numpy as np
from copy import copy
import rbdl

pi = np.pi


class Robot(object):
    def __init__(self, q0, dq0, ndof, dt):
        self.q = q0    # numpy array (ndof x 1)
        self.dq = dq0  # numpy array (ndof x 1)
        self.M = np.zeros([ndof, ndof])
        self.b = np.zeros(ndof)
        self.dt = dt
        self.robot = rbdl.loadModel('../urdf/ur5_robot.urdf')

    def send_command(self, tau):
        rbdl.CompositeRigidBodyAlgorithm(self.robot, self.q, self.M)
        rbdl.NonlinearEffects(self.robot, self.q, self.dq, self.b)
        ddq = np.linalg.inv(self.M).dot(tau-self.b)
        self.q = self.q + self.dt*self.dq
        self.dq = self.dq + self.dt*ddq

    def read_joint_positions(self):
        return self.q

    def read_joint_velocities(self):
        return self.dq


def dh(d, theta, a, alpha):
    """
    Matriz de transformacion homogenea asociada a los parametros DH.
    Retorna una matriz 4x4
    """
    sth = np.sin(theta)
    cth = np.cos(theta)
    sa  = np.sin(alpha)
    ca  = np.cos(alpha)
    T = np.array([[cth, -ca*sth,  sa*sth, a*cth],
                   [sth,  ca*cth, -sa*cth, a*sth],
                   [0.0,      sa,      ca,     d],
                   [0.0,     0.0,     0.0,   1.0]])
    return T


def proy_fkine(q):
    """
    Calcular la cinematica directa del robot UR5 dados sus valores articulares. 
    q es un vector numpy de la forma [q1, q2, q3, q4, q5, q6]
    """
    # Matrices DH
    T1 = dh( 0.175      , q[0]+pi/2  , 0, pi/2)
    T2 = dh( 0.245      , q[1]+pi    , 0, pi/2)
    T3 = dh( 1.1375+q[2], 0          , 0, pi/2)
    T4 = dh( 0.245      , q[3]+2*pi/2, 0, pi/2)
    T5 = dh(0.735+q[4]  , 0          , 0, pi/2)
    T6 = dh( 0.175      , pi+q[5], 0, pi/2)
    T7 = dh( 0.175      , q[6]       , 0, 0)
    T8 = dh( 0.07      , 0          , 0, 0)
    # Efector final con respecto a la base
    T = T1 @ T2 @ T3 @ T4 @ T5 @ T6 @ T7
    return T

def rot2quat(R):
 """
 Convertir una matriz de rotacion en un cuaternion

 Entrada:
  R -- Matriz de rotacion
 Salida:
  Q -- Cuaternion [ew, ex, ey, ez]

 """
 dEpsilon = 1e-6
 quat = 4*[0.,]

 quat[0] = 0.5*np.sqrt(R[0,0]+R[1,1]+R[2,2]+1.0)
 if ( np.fabs(R[0,0]-R[1,1]-R[2,2]+1.0) < dEpsilon ):
  quat[1] = 0.0
 else:
  quat[1] = 0.5*np.sign(R[2,1]-R[1,2])*np.sqrt(R[0,0]-R[1,1]-R[2,2]+1.0)
 if ( np.fabs(R[1,1]-R[2,2]-R[0,0]+1.0) < dEpsilon ):
  quat[2] = 0.0
 else:
  quat[2] = 0.5*np.sign(R[0,2]-R[2,0])*np.sqrt(R[1,1]-R[2,2]-R[0,0]+1.0)
 if ( np.fabs(R[2,2]-R[0,0]-R[1,1]+1.0) < dEpsilon ):
  quat[3] = 0.0
 else:
  quat[3] = 0.5*np.sign(R[1,0]-R[0,1])*np.sqrt(R[2,2]-R[0,0]-R[1,1]+1.0)

 return np.array(quat)


def TF2xyzquat(T):
 """
 Convert a homogeneous transformation matrix into the a vector containing the
 pose of the robot.

 Input:
  T -- A homogeneous transformation
 Output:
  X -- A pose vector in the format [x y z ew ex ey ez], donde la first part
       is Cartesian coordinates and the last part is a quaternion
 """
 quat = rot2quat(T[0:3,0:3])
 res = [T[0,3], T[1,3], T[2,3], quat[0], quat[1], quat[2], quat[3]]
 return np.array(res)
####################################################################################
def jacobian(q, delta=0.0001):
 """
 Jacobiano analitico para la posicion de un brazo robotico de n grados de libertad. 
 Retorna una matriz de 3xn y toma como entrada el vector de configuracion articular 
 q=[q1, q2, q3, ..., qn]
 """
 # Crear una matriz 3xn
 n = q.size
 J = np.zeros((3,n))
 # Calcular la transformacion homogenea inicial (usando q)
 T = proy_fkine(q)
    
 # Iteracion para la derivada de cada articulacion (columna)
 for i in range(n):
  # Copiar la configuracion articular inicial
  dq = copy(q)
  # Calcular nuevamenta la transformacion homogenea e
  # Incrementar la articulacion i-esima usando un delta
  dq[i] += delta
  # Transformacion homogenea luego del incremento (q+delta)
  T_inc = proy_fkine(dq)
  # Aproximacion del Jacobiano de posicion usando diferencias finitas
  J[0:3,i]=(T_inc[0:3,3]-T[0:3,3])/delta
 return J


def ikine(xdes, q0):
 """
 Calcular la cinematica inversa de un brazo robotico numericamente a partir 
 de la configuracion articular inicial de q0. Emplear el metodo de newton.
 """
 epsilon  = 0.0001
 max_iter = 1000
 delta    = 0.00001
 q  = copy(q0)
 
 for i in range(max_iter):
  # Main loop
  xact = proy_fkine(q)
  error = xdes - xact[0:3,3]
  if np.linalg.norm(error) < epsilon:
   print(f"converge para {i+1} iteraciones.")
   return q      
  # Calcular Jacobiano en la configuración actual
  J = jacobian(q)
  delta_q = np.linalg.pinv(J).dot(error)
  q += delta_q
  # Restringir articulaciones
  for idx, value in enumerate(q):
   if idx in prism_limits:
   # Restricciones para articulaciones prismáticas
    lower, upper = prism_limits[idx]
    q[idx] = np.clip(value, lower, upper)
   else:
        # Ajustar articulaciones revolutas al rango [-pi, pi]
    q[idx] = (value + np.pi) % (2 * np.pi) - np.pi
 return q


def ik_gradient(xdes, q0):
 """
 Calcular la cinematica inversa de un brazo robotico numericamente a partir 
 de la configuracion articular inicial de q0. Emplear el metodo gradiente.
 """
 epsilon  = 0.001
 max_iter = 1000
 delta    = 0.00001
 alpha    = 0.01
 q  = copy(q0)
 for i in range(max_iter):
  # Main loop
  xact = proy_fkine(q)[0:3,3]
  error = xdes - xact
 if np.linalg.norm(error) < epsilon:
  print(f"converge para {i+1} iteraciones.")
  return q      
 # Calcular Jacobiano en la configuración actual
 grad = np.zeros(len(q))  # Gradiente tiene la misma dimensión que q
 for j in range(len(q)):
  q_temp = np.copy(q)
  q_temp[j] += delta  # Incrementar la articulación \(q_j\)
  x_temp = proy_fkine(q_temp)[0:3,3]
  grad[j] = np.dot((x_temp - xact), error) / delta  # Gradiente parcial respecto a \(q_j\)      
  # Actualizar la configuración articular en la dirección del gradiente
  q += alpha * grad  # Actualización con la tasa de aprendizaje
 return q
