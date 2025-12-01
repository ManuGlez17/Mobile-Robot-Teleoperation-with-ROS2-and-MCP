#!/bin/bash
# start_ros_tmux.sh
SESSION="ros_all"

# cerrar sesión si existe
tmux has-session -t $SESSION 2>/dev/null && tmux kill-session -t $SESSION

# crear sesión en background
tmux new-session -d -s $SESSION -n terminalA

# Ventana A (Inicializar master de ROS1)
tmux send-keys -t $SESSION:terminalA 'source ~/ros_catkin_ws/install_isolated/setup.bash' C-m
tmux send-keys -t $SESSION:terminalA 'rosmaster --core' C-m

sleep 5

# Ventana B (Inicializar controlador del Robot)
tmux new-window -t $SESSION -n terminalB
tmux send-keys -t $SESSION:terminalB 'source ~/ros_catkin_ws/install_isolated/setup.bash' C-m
tmux send-keys -t $SESSION:terminalB 'source ~/p3dx_ws/devel/setup.bash' C-m
tmux send-keys -t $SESSION:terminalB 'rosrun rosaria RosAria _port:=/dev/p3dx' C-m

sleep 5
# Ventana C (Inicializar puente de ROS1 a ROS2)
tmux new-window -t $SESSION -n terminalC
tmux send-keys -t $SESSION:terminalC 'source ~/ros_catkin_ws/install_isolated/setup.bash' C-m
tmux send-keys -t $SESSION:terminalC 'rosparam load prueba_rosaria_bridge.yaml' C-m
tmux send-keys -t $SESSION:terminalC 'source /opt/ros/humble/setup.bash' C-m
tmux send-keys -t $SESSION:terminalC 'source ~/p3dx_project/p3dx_ws_ros2/install/setup.bash' C-m
tmux send-keys -t $SESSION:terminalC 'ros2 run ros1_bridge parameter_bridge' C-m

# Ventana D (Traductor de mensajes de sonares de ROS1 a ROS2)
tmux new-window -t $SESSION -n terminalD
tmux send-keys -t $SESSION:terminalD 'source ~/ros1_catkin_ws/install_isolated/setup.bash' C-m
tmux send-keys -t $SESSION:terminalD 'source ~/p3dx_project/p3dx_ws_ros1/devel/setup.bash' C-m
tmux send-keys -t $SESSION:terminalD 'rosrun sonar_bridge sonar_pc1_to_pc2.py' C-m

# Ventana E (Nodo de Camara)
tmux new-window -t $SESSION -n terminalE
tmux send-keys -t $SESSION:terminalE 'source /opt/ros/humble/setup.bash' C-m
tmux send-keys -t $SESSION:terminalE 'source ~/p3dx_project/p3dx_ws_ros2/install/setup.bash' C-m
tmux send-keys -t $SESSION:terminalE 'ros2 launch camera_package detect.launch.py' C-m

echo "Sesión tmux '$SESSION' creada. Usa: tmux attach -t $SESSION"