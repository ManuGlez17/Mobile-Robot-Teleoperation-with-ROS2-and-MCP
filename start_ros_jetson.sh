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
tmux send-keys -t $SESSION:terminalB 'rosrun rosaria RosAria' C-m

sleep 5
# Ventana C (Inicializar puente de ROS1 a ROS2)
tmux new-window -t $SESSION -n terminalC
tmux send-keys -t $SESSION:terminalC 'source ~/ros_catkin_ws/install_isolated/setup.bash' C-m
tmux send-keys -t $SESSION:terminalC 'rosparam load prueba_rosaria_bridge.yaml' C-m
tmux send-keys -t $SESSION:terminalC 'source /opt/ros/humble/setup.bash' C-m
tmux send-keys -t $SESSION:terminalC 'source ~/ros1_bridge_ws/install/setup.bash' C-m
tmux send-keys -t $SESSION:terminalC 'ros2 run ros1_bridge parameter_bridge' C-m

echo "Sesión tmux '$SESSION' creada. Usa: tmux attach -t $SESSION"
