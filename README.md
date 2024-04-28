# Robotics-Udacity-Chase-It Project

The project is part of the [Udacity Robotics Software Engineer Nanodegree](https://www.udacity.com/course/robotics-software-engineer--nd209). This initiative focuses on creating a robot capable of autonomously chasing a white ball, implemented within a Gazebo simulation environment using the Robot Operating System (ROS).

## Project Components

### `my_robot` Package
This package forms the core of the robot's design and its simulated world. It includes:
- A URDF file defining the robot's physical structure and components.
- Simulation environments where the robot operates.

### `ball_chaser` Package
Focused on enabling the robot to detect and pursue a white ball, this package includes:
- `drive_bot`: A node that commands the robot's movement.
- `process_image`: A node responsible for the visual detection of the ball using camera data.

## Skills acquired 
- ROS
- Real-time image processing
- Robotic simulation in Gazebo
- Robotic control and sensor integration
