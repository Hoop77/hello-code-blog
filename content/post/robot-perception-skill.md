---
title: "Creating a Robot Perception Skill"
description: "Devlog"
author:
  name: "Philipp Badenhoop"
date: 2019-05-03
draft: true
tags:
- Robotics
- Computer Vision
markup: mmark
use: [math]
---

In this summer semester at TU Munich, I participate in a practical course where the goal is to create a perception skill which could potentially be used by a robot.
My idea is to detect cooking pots and to find their handles so that a robot can pick them up.

# Project Plan

## Deadlines

1. Goal definition - 08.05.2020
2. Research - 22.5.2020
3. Development - 03.07.2020
4. Documentation - 17.07.2020
5. Paper - 25.07.2020

## Presentations

1. Project Topic - 08.05.2020
2. State-of-the-art & proposal - 22.05.2020
3. Experiments, results & conclusions - 25.07.2020

# Goal Definition

- types of object with handle/grasps: cooking pot, pan, container
- types of handles: curved, straight, flat
- input: images, point clouds
- output: bounding box, segmentation, point cloud, object descriptor, handle descriptor
- approach: deep learning vs manual feature extraction vs reinforcement learning in simulation

# Research

## ROS handle detector
- [Paper](http://www.ccs.neu.edu/home/atp/publications/affordances_iser2014_final.pdf)
- [ROS wiki](http://wiki.ros.org/handle_detector)

This was written in 2010 so its not a deep learning approach. 
They extract handle information from point clouds.

## Review of deep learning methods in robotic grasp detection
- [Paper](https://www.mdpi.com/2414-4088/2/3/57/pdf)

### Overview

Subsystems for robot grasp detection
- **Grasp detection sub-system**: To detect grasp poses from images of the objects in their image
plane coordinates
- **Grasp planning sub-system**: To map the detected image plane coordinates to the
world coordinates
- **Control sub-system**: To determine the inverse kinematics solution of the previous sub-system

{{< figure src="/img/robot-perception-skill/grasp-representation-table.png" title="Comparison between different grasp representations ." class="autosize" >}}

Consider grasp detection vs learn robustness function.

### Datasets

- [Cornell Grasp Dataset (CGD)](http://pr.cs.cornell.edu/grasping/rect_data/data.php) - 1035 images of 280 different objects, includes point clouds
- [Washington RGB-D dataset](https://rgbd-dataset.cs.washington.edu/index.html) - includes RGB-D images
- [Dexerity Network (Dex-Net)](https://berkeleyautomation.github.io/dex-net/)
- RGB-D images has an advantage over uni-modal RGB images

{{< figure src="/img/robot-perception-skill/cdg-samples.png" title="Sample images from the CGD" class="autosize" >}}

### Architectures

{{< figure src="/img/robot-perception-skill/resnet50-grasp-predictor-architecture.png" title="Uni-modal grasp predictor for CGD" class="autosize" >}}

{{< figure src="/img/robot-perception-skill/architecture-comparison.png" title="Comparison between different transfer learning techniques in one-shot grasp detection on CGD" class="autosize" >}}

### Metrics

- point metric

    Evaluates the distance between predicted and actual grasp centre relative to a threshold (threshold generally not uniform in literature).

- rectangle metric

    A grasp is considered successful under the following conditions:

    1. Difference between grasp angles < 30°
    2. Jacquard index between grasps < 25%

    The Jacquard index between $$Grasp_{pred}$$ and $$Grasp_{true}$$ is given by:

    $$ J(Grasp_{pred}, Grasp_{true}) = \frac{| Grasp_{pred} \cap Grasp_{true}|}{| Grasp_{pred} \cup Grasp_{true} |} $$

### Future Work

- creation of larger datasets from 3D simulations and domain adaption.
- reinforcement learning approaches

## Robotic Grasp Detection using Deep Convolutional Neural Networks

[Paper](https://arxiv.org/pdf/1611.08036.pdf?source=post_page---------------------------)

# Development 

- buy/rent sensor?