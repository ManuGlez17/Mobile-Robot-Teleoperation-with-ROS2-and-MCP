# Mobile-Robot-Teleoperation-with-ROS2-and-MCP

This project implements a MCP server in ROS2 to communicate with P3DX robot. This approach allows to interact with the robot with Natural Language commands, which ranges from performing actions like moving the robot a determined distance or drawing a shape, to developing code for more complex tasks. By this way, the knowledge gap for working with a robot in ROS2 can be significantly reduced due to the power of LLMs to understand Natural Language commands.

# Material and System Requirements

- Host Machine, Ubuntu 22.04 
- Jetson Orin Nano, Ubuntu 22.04, CUDA available
- P3DX Mobile Robot
- OAK-D Camera (for now, any camera can be used)
- ROS1 and ROS2 installation

# Dependencies
- MCP server setup installation. Follow this tutorial link: https://github.com/robotmcp/ros-mcp-server/tree/main
- LLM client (Claude, GitHub Copilot, ChatGPT)
- ROS WebSocket Bridge: ```sudo apt-get install ros-<rosdistro>-rosbridge-server```
- ROSARIA Library installation. Follow this tutorial link: https://wiki.ros.org/ROSARIA/Tutorials/How%20to%20use%20ROSARIA
- OpenCV with CUDA support

