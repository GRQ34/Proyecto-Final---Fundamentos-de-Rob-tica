#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from markers import *
from functions import *
from roslib import packages
import matplotlib.pyplot as plt
import rbdl


if __name__ == '__main__':



 

  rospy.init_node("control_pdg")
  pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
  bmarker_actual  = BallMarker(color['RED'])
  bmarker_deseado = BallMarker(color['GREEN'])
  # Archivos donde se almacenara los datos
  fqact = open("/home/user/qactual.txt", "w")
  fqdes = open("/home/user/qdeseado.txt", "w")
  fxact = open("/home/user/xactual.txt", "w")
  fxdes = open("/home/user/xdeseado.txt", "w")
 
  # Nombres de las articulaciones
  jnames = ['joint1', 'joint2', 'joint3',
          'joint4', 'joint5', 'joint6','joint7','joint_g_left','joint_g_right']
  # Objeto (mensaje) de tipo JointState
  jstate = JointState()
  # Valores del mensaje
  jstate.header.stamp = rospy.Time.now()
  jstate.name = jnames
 
  # =============================================================
  # Configuracion articular inicial (en radianes)
  q = np.array([0, 0, 0, 0, 0, 0, 0,0.0,0.0])
  # Velocidad inicial
  dq = np.array([0., 0., 0., 0., 0., 0.,0.,0.,0.])
  # Configuracion articular deseada
  qdes = np.array([0, 0, -0.6, -1.02, 0, 0, 0,0.,0.])
  # =============================================================
 
  # Posicion resultante de la configuracion articular deseada
  xdes = ur5_fkine(qdes)[0:3,3]
  # Copiar la configuracion articular en el mensaje a ser publicado
  jstate.position = q
  pub.publish(jstate)
 
  # Modelo RBDL
  modelo = rbdl.loadModel('/home/user/lab_ws/src/URDF2_description/urdf/URDF2.urdf')
  ndof   = modelo.q_size     # Grados de libertad
 
  # Frecuencia del envio (en Hz)
  freq = 20
  dt = 1.0/freq
  rate = rospy.Rate(freq)
 
  # Simulador dinamico del robot
  robot = Robot(q, dq, ndof, dt)
  zeros = np.zeros(9)
  g = np.zeros(9)
  tau = np.zeros(9)
  m_inercial = []
  m_gravitacional = []
  m_amortiguado = []
  m_coriolis = []
  times = []
  M = np.zeros((9, 9))

 

  # Se definen las ganancias del controlador
  Kp = np.diag([5,5,5,5,5,5,5,5,5])*0.8
  Kd = np.diag([.6,.6,.6,.6,.6,.6,.6,.6,.6])*8
 
  # Bucle de ejecucion continua
  t = 0.0
  while not rospy.is_shutdown():
 
    # Leer valores del simulador
    q  = robot.read_joint_positions()
    dq = robot.read_joint_velocities()
    # Posicion actual del efector final
    x = ur5_fkine(q)[0:3,3]
    # Tiempo actual (necesario como indicador para ROS)
    jstate.header.stamp = rospy.Time.now()
    rbdl.InverseDynamics(modelo, q, np.zeros(ndof), np.zeros(ndof), g)  # Gravedad
    c_temp = np.zeros(ndof)
    rbdl.InverseDynamics(modelo, q, dq, np.zeros(ndof), c_temp)  # Coriolis
    c = c_temp - g
    for i in range(ndof):
            ei = np.zeros(ndof)
            ei[i] = 1.0
            Mi = np.zeros(ndof)
            rbdl.InverseDynamics(modelo, q, zeros, ei, Mi)
            M[:, i] = Mi - g
   
    # Almacenamiento de datos
    fxact.write(str(t)+' '+str(x[0])+' '+str(x[1])+' '+str(x[2])+'\n')
    fxdes.write(str(t)+' '+str(xdes[0])+' '+str(xdes[1])+' '+str(xdes[2])+'\n')
    fqact.write(str(t)+' '+str(q[0])+' '+str(q[1])+' '+ str(q[2])+' '+ str(q[3])+' '+str(q[4])+' '+str(q[5])+'\n ')
    fqdes.write(str(t)+' '+str(qdes[0])+' '+str(qdes[1])+' '+ str(qdes[2])+' '+ str(qdes[3])+' '+str(qdes[4])+' '+str(qdes[5])+'\n ')
    
    print("Matriz de inercia (M):")
    print(M)
    print("\nVector de gravedad (g):")
    print(g)
    print("\nVector Coriolis/Centrífugo (C):")
    print(c)
    # ----------------------------
    # Control dinamico (COMPLETAR)
    # ----------------------------
   
    err = qdes-q
    rbdl.InverseDynamics(modelo,q,np.zeros(9),np.zeros(9),g) #Obtener vector de gravedad
    g = g.copy()
    u = Kp @ err - Kd @ dq + g# Reemplazar por la ley de control
  # Almacenar datos para gráficas
    times.append(t)
    m_gravitacional.append(np.linalg.norm(g))
    m_coriolis.append(np.linalg.norm(c))
    m_inercial.append(np.linalg.norm(M @ dq))    
   
   

    # Simulacion del robot
    robot.send_command(u)

    # Publicacion del mensaje
    jstate.position = q
    pub.publish(jstate)
    bmarker_deseado.xyz(xdes)
    bmarker_actual.xyz(x)
    t = t+dt
    # Esperar hasta la siguiente  iteracion
    rate.sleep()

  fqact.close()
  fqdes.close()
  fxact.close()
  fxdes.close()
  # ----------------------------
  # Gráficas
  # ----------------------------
# Conversión a numpy arrays
times = np.array(times)
m_inercial = np.array(m_inercial)
m_gravitacional = np.array(m_gravitacional)
m_amortiguado = np.array(m_amortiguado)
m_coriolis = np.array(m_coriolis)

# Gráficas
plt.figure(figsize=(10, 6))
plt.plot(times, m_inercial, label="Inercial", color="blue")
plt.plot(times, m_gravitacional, label="Gravitacional", color="green")
plt.plot(times, m_coriolis, label="Coriolis/Centrífugo", color="purple")
plt.xlabel("Tiempo [s]")
plt.ylabel("Módulo del efecto dinámico")
plt.title("Efectos dinámicos del manipulador vs Tiempo")
plt.legend()
plt.grid()
plt.show()
