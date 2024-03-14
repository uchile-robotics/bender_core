#!/bin/bash

# Iniciar ROS Core
roscore &

# Esperar 3 segundos a que se inicie ROS Core
sleep 3

# Iniciar RViz
rviz rviz &

# Esperar 5 segundos a que se inicie RViz y logres entrar a los topics
sleep 5

# Reproducir el rosbag a 0.5x
rosbag play --clock --rate 0.5 demo.bag &

# Esperar 3 segundos a que se suscriban los nodos
sleep 5

# Ejecutar el tracker
rosrun basic_perception tracker.py

# Probablemente la primera vez no alcances a cargar los topicos apenas parta el video
# asi que seguramente la segunda vez los topicos ya estaran registrados y mantienes rviz abierto
# y vuelves a correr el demo.sh