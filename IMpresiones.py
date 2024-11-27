#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Leer el archivo y extraer las posiciones
with open("/home/victor/lab_ws/src/frlabs/lab6/src/temp/xcurrent.txt", "r") as file:
    posiciones = [list(map(float, line.split())) for line in file]
with open("/home/victor/lab_ws/src/frlabs/lab6/src/temp/xdesired.txt", "r") as file:
    posicionesd = [list(map(float, line.split())) for line in file]


# Separar los valores de x, y y z en listas
tiempo=[pos[0] for pos in posiciones]
x_values = [pos[1] for pos in posiciones]
y_values = [pos[2] for pos in posiciones]
z_values = [pos[3] for pos in posiciones]
x_valuesd = [pos[1] for pos in posicionesd]
y_valuesd = [pos[2] for pos in posicionesd]
z_valuesd = [pos[3] for pos in posicionesd]
#tiempo = range(0, len(posiciones))/freq  # Crear una lista de tiempos desde 1 hasta el número de posiciones

# Crear el gráfico
plt.figure(figsize=(10, 6))
plt.plot(tiempo, x_values, label="x", color="r")
plt.plot(tiempo, y_values, label="y", color="g")
plt.plot(tiempo, z_values, label="z", color="b")
plt.plot(tiempo, x_valuesd, label="x_ref", color="#FFA500")  # Naranja
plt.plot(tiempo, y_valuesd, label="y_ref", color="#7CFC00")  # Caña verde
plt.plot(tiempo, z_valuesd, label="z_ref", color="#00BFFF")  # Celeste
# Añadir etiquetas y leyenda
plt.xlabel("Tiempo")
plt.ylabel("Posiciones")
plt.title("Posiciones x, y, z en función del tiempo")
plt.legend()

# Mostrar el gráfico
plt.grid(True)
plt.show()
# graficando x y z
