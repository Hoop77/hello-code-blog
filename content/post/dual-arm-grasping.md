---
title: "Dual-Arm Grasp Detection of Chairs"
description: "My work from a practical course."
author:
  name: "Philipp Badenhoop"
date: 2020-07-01
draft: false
image: dual-arm-grasping.png
tags:
- Robotics
- ROS
- Simulation
- Grasping
---

During my master studies at Technical University of Munich, I attended a practical course where I made two robots grasping a chair in a ROS simulation.
Sounds easy, but was a TON of work!
I came to the conclusion that even though Gazebo is the first choice simulator in ROS, it is absolutely not ready for doing even the most basic things such as grasping!
Although it was quite painful at some points, I really learned A LOT.
Some things that I learned:

- Defining the robots in URDF
- Tuning the PID control parameters of the manipulators' joints
- Setting up the motion planning framework Moveit!
- Processing point clouds using PCL
- Developing quite an extensive pipeline to detect and execute dual-arm grasps on chairs

The code of the project can be accessed [here](https://github.com/Badenhoop/chair_manipulation).
I explained my approach as well as the results in a final paper that can be downloaded [here](/pdf/dual-arm-grasp-detection-of-chairs.pdf).