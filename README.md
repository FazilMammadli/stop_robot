# Stop Robot with C++ ROS2 Node

This repository contains a C++ ROS2 node that stops a robot when executed. The task is to stop the robot without using a `Twist` message or killing the node directly.

## Objective

- Create a C++ ROS2 node that, when run, will stop the robot.
- Python will not work.
- You cannot stop the robot using a `Twist` message or killing the node.


## Video Demonstration

Below is a screen recording video of the robot navigating using RViz2 and then stopping once the C++ node is activated.

![Video Demonstration](./path_to_your_video/video_demo.gif)

## How the Code Works

The C++ ROS2 node to stop the robot works by canceling all active navigation goals. Here’s a brief explanation of the key components and logic:

1. **Node Initialization:**
   The node is initialized with the name `stop_robot_node`. It creates an action client to interact with the `NavigateToPose` action server.

2. **Action Client Creation:**
   An action client is created to communicate with the `navigate_to_pose` action server, which handles navigation goals.

3. **Sending Cancel Request:**
   The node sends a request to cancel all current navigation goals using the `async_cancel_all_goals()` method of the action client. This method ensures that any active navigation goals are canceled, which effectively stops the robot.

4. **Waiting for Action Server:**
   Before sending the cancel request, the node waits for the action server to be available. If the action server is not available within a specified time, an error message is logged.

5. **Handling the Cancel Response:**
   The node waits for the result of the cancel request. If the cancel request is successful, a confirmation message is logged indicating that all goals were successfully canceled.

6. **Logging:**
   Throughout the process, informative log messages are generated to provide feedback on the status of the node and the actions being performed.

## Instructions

1. **Clone the Repository:**

   ```bash
   git clone https://github.com/FazilMammadli/stop_robot.git
   cd stop_robot

2. **Build the Package:**

   ```bash
   colcon build
   source install/setup.bash
3. **Run the Node:**
   ```bash
   ros2 run stop_robot stop_robot_node

