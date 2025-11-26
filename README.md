# UR5 Pick-and-Place Simulation with ROS2 & Gazebo

This repository contains packages for simulating a **UR5 robotic arm** using **ROS2** and **Gazebo**. The project focuses on performing pick-and-place tasks on a red cube identified by an **AprilTag**, using an **onboard camera** for detection and localization.

While the project is still a work in progress and not all components are fully implemented, it showcases my work in robotics simulation, perception, and control. Because it is a complex projects, all files are not present in here.

I advise you to build your proper ROS2 configuration with your different packages and then get some elements of this reposotiroy to get some ideas if you have a simular project

## Project Overview

* **Robot:** UR5 robotic arm, simulated in Gazebo.
* **Perception:** Detection of a red cube using **AprilTags** with an onboard camera.
* **Control:** Pick-and-place tasks implemented via ROS2 nodes controlling the UR5 arm.
* **Mathematics & Algorithms:**

  * **Quaternions** for orientation handling.
  * **SVD (Singular Value Decomposition)** used in pose estimation and coordinate transformations.
* **Simulation:** All interactions are performed in Gazebo, providing a realistic physics environment.

The archive includes the ROS2 packages I developed, which contain nodes, launch files, and scripts required to perform the simulation tasks mentioned above.

## Archive Contents

* [a compressed archive of all the ROS2 packages I worked on here] (https://github.com/ninjamath3/ur5-simulation/blob/main/packages%20developp%C3%A9s.zip)

  * Packages include control nodes, perception scripts, and Gazebo launch configurations.
  * Designed to be easily imported into a ROS2 workspace for simulation.

## Notes

* This project was primarily a learning and experimental effort to explore:

  * Robotics simulation workflows in ROS2 and Gazebo
  * Integration of computer vision (AprilTag detection) for robotic manipulation
  * Handling robot kinematics and orientation using quaternions
* Some aspects of the full pick-and-place workflow may not be fully completed.
* The code is structured to allow further development and experimentation.

## Technologies Used

* **ROS2** – Robot Operating System 2 for communication and control.
* **Gazebo** – 3D robotics simulator for realistic physics interactions.
* **AprilTag** – Visual fiducial markers for object detection.
* **Python & C++** – Nodes for perception and control.
* **Quaternions & SVD** – Mathematical tools for orientation and pose estimation.

This project demonstrates practical experience in robotic simulation, perception, and control using ROS2 and provides a foundation for extending pick-and-place tasks to more complex scenarios.
