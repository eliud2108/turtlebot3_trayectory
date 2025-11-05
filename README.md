This repository contains the implementation of four controllers for trajectory performance analysis on the turtlebot3 waffle robot. The controllers used are: PID, Lyapunov, Pure Pursuit, and MPC. Metrics are stored in the tb3_runs folder for later analysis. 

## Modo de ejecución:

#  Compilar paquete
cd ~/turtlebot3_pruebas
colcon build --packages-select turtlebot3_control
source install/setup.bash

export TURTLEBOT3_MODEL=waffle

# Para simulación
1. se corre ros2 launch turtlebot3_gazebo empty_world.launch.py

2. luego en otra pestaña se corre el controlador correspondiente.


# Para robot fisico
# En el robot
ros2 launch turtlebot3_bringup robot.launch.py

## CONTROLADORES

# En otra pestaña - PID
ros2 run turtlebot3_control simple_pid_controller

# En otra pestaña - Lyapunov
ros2 run turtlebot3_control simple_lyapunov_controller

# MPC
ros2 run turtlebot3_control mpc_controller_real

# PURE PERSUIT
ros2 run turtlebot3_control pure_pursuit_controller_real
