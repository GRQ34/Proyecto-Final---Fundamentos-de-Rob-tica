# Proyecto-Final---Fundamentos-de-Rob-tica
Carlos Francia, Henry Gahona, Víctor Huanca, Germán Rosales



IMpresiones.py: muestra graficas de la posicion del efector final, comparandolas con la referencia

Robot.zip: Es el modelo del Robot diseñado para el proyecto

Lab6Functions.py: contiene un clase de tipo Robot y funciones necesarias para hacer el control cinematico

markers.py: muestra las posiciones deseadas con un marcador

test_diffnike.py: es el control cinematico de las articulaciones.

test_diffnike_pose.py: es el control cinematico de las articulaciones y la orientacion del efector final.

Carpeta Cinematica_directa_invers:
  La carpeta contiene los archivos necesarios para las pruebas de cinemática directa e inversa.
  - test_fkine: Archivo para mostrar la cinemática directa en rviz e indicar si está bien. Se colocan las articulaciones para prober.
  - test_ikine: Archivo para mostrar la cinemática inversa en rviz  e indicar si está bien. Se colocan las posiciones en espacio cartesiano.
  - functions.py: Presenta las funciones para poder utilizar fjine e ikine. Calcula la matriz de transformación DH, así como el jacobiano numérico y la cinemática inversa por metodo iterativo.
  - markers.py: Archivo con los parámetros necesarios para colocar los marers en rviz tanto para fkine cmoo para ikine.
