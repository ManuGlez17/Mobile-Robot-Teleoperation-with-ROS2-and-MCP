# Mobile-Robot-Teleoperation-with-ROS2-and-MCP

This project implements a MCP server in ROS2 to communicate with P3DX robot. This approach allows to interact with the robot with Natural Language commands, which ranges from performing actions like moving the robot a determined distance or drawing a shape, to developing code for more complex tasks. By this way, the knowledge gap for working with a robot in ROS2 can be significantly reduced due to the power of LLMs to understand Natural Language commands.

# Hardware and System Requirements

- Host Machine, Ubuntu 22.04 
- Jetson Orin Nano, Ubuntu 22.04, CUDA available
- P3DX Mobile Robot
- OAK-D Camera (for now, any camera can be used)
- ROS1 and ROS2 installation

# Dependencies
- MCP server setup installation. Follow this tutorial link: https://github.com/robotmcp/ros-mcp-server/tree/main
- TMUX
- LLM client (Claude, GitHub Copilot, ChatGPT)
- ROS WebSocket Bridge: ```sudo apt-get install ros-<rosdistro>-rosbridge-server```
- ROSARIA Library installation. Follow this tutorial link: https://wiki.ros.org/ROSARIA/Tutorials/How%20to%20use%20ROSARIA
- OpenCV with CUDA support

# Steps in Jetson Nano
1. Create a main workspace folder.
2. Create one ROS1 workspace and one ROS2 workspace.
3. Copy the corresponding packages with their respective ROS version.
4. Build each workspace employing the following commands:
   - ROS1: ```catkin_make```
   - ROS2: ```colcon build --symlink-install --packages-skip ros1_bridge``` && ```colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure```
6. Modify the file ```start_ros_jetson.sh``` with the corresponding directories for the packages and ROS ubications. You can follow default names in the file to avoid change.
7. To run all nodes, execute the command ```\.start_ros_jetson.sh```

# Steps in Host Machine
1. Be sure you are connected to the same network as the Jetson Nano
2. Run the rosbridge web socket: ```ros2 launch rosbridge_server rosbridge_websocket_launch.xml```
3. Initialize the server as indicated in ros-mcp-server tutorial
4. Now you can communicate with the robot with Natural Language Commands!

# Command Examples
- "Check the connection with the robot"
- "List all the topics available and active nodes"
- "Subscribe to a image topic and save the image"
- "Check if the camera detects a person"
- "Move the robot forward by 5 cm"
- "Draw a 20 cm square with the robot"

# Node Structure
<img width="1177" height="779" alt="nodos_p3dx" src="https://github.com/user-attachments/assets/03a01895-2613-4d6f-a1e5-0f9d97fd1161" />
