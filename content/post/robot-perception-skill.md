---
title: "Multi-Arm Grasps Detection of Chairs"
description: "Creating a Robotics Perception Skill"
author:
  name: "Philipp Badenhoop"
date: 2020-05-03
draft: true
tags:
- Robotics
- Computer Vision
markup: mmark
use: [math]
---

In this summer semester at TU Munich, I participate in a practical course where the goal is to create a perception skill which could potentially be used by a robot.
My idea is to detect suitable grasping poses of chairs for dual-arm manipulation.

# Project Plan

## Deadlines

1. Goal definition - 08.05.20
2. Research - 22.5.20
3. Development - 03.07.20
4. Documentation - 17.07.20
5. Paper - 25.07.20

## Presentations

1. Project Topic - 08.05.2020
2. State-of-the-art & proposal - 22.05.2020
3. Experiments, results & conclusions - 25.07.2020

# Goal Definition

- objects: chairs
- input: point clouds generated from RGB-D images
- output: grasp descriptor/s, grasp quality values
- approach: deep learning, non-rigid registrations

# Research

## ROS handle detector

[Paper](http://www.ccs.neu.edu/home/atp/publications/affordances_iser2014_final.pdf)

[ROS wiki](http://wiki.ros.org/handle_detector)

This was written in 2010 so its not a deep learning approach. 
They extract handle information from point clouds.

## Review of deep learning methods in robotic grasp detection

[Paper](https://www.mdpi.com/2414-4088/2/3/57/pdf)

### Overview

Subsystems for robot grasp detection
- **Grasp detection sub-system**: To detect grasp poses from images of the objects in their image
plane coordinates
- **Grasp planning sub-system**: To map the detected image plane coordinates to the
world coordinates
- **Control sub-system**: To determine the inverse kinematics solution of the previous sub-system

{{< figure src="/img/robot-perception-skill/grasp-representation-table.png" title="Comparison between different grasp representations ." >}}

Consider grasp detection vs learn robustness function.

### Datasets

- [Cornell Grasp Dataset (CGD)](http://pr.cs.cornell.edu/grasping/rect_data/data.php) - 1035 images of 280 different objects, includes point clouds
- [Washington RGB-D dataset](https://rgbd-dataset.cs.washington.edu/index.html) - includes RGB-D images
- [Dexerity Network (Dex-Net)](https://berkeleyautomation.github.io/dex-net/)
- RGB-D images has an advantage over uni-modal RGB images

{{< figure src="/img/robot-perception-skill/cdg-samples.png" title="Sample images from the CGD" class="autosize" >}}

### Architectures

{{< figure src="/img/robot-perception-skill/resnet50-grasp-predictor-architecture.png" title="Uni-modal grasp predictor for CGD" >}}

{{< figure src="/img/robot-perception-skill/architecture-comparison.png" title="Comparison between different transfer learning techniques in one-shot grasp detection on CGD" >}}

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

{{< figure src="/img/robot-perception-skill/resnet50-grasp-predictor-architecture-multimodal.png" title="Multi-modal grasp predictor for CGD" >}}

### Limitations

- They assume that the input image contains only one graspable object and a single grasp has to be predicted for the object. Since there are object detection networks such as YOLO which can detect multiple objects in an image, we could think of getting rid of this limitation.

- Both ResNets were pretrained on ImageNet which may not give best results. Maybe there exists a depth dataset now which we can use to improve the results.

- Predicting a point or rectangle is not ideal for this problem because there are multiple places where the robot may grasp an object. However, it is hard to collect data where each all possible gripper poses are labeled.

## Supersizing Self-supervision: Learning to Grasp from 50K Tries and 700 Robot Hours

[Paper](https://arxiv.org/pdf/1509.06825.pdf)

### Data Collection Method

Instead of using human-labeled data they setup an environment where a robot automatically creates the data by performing random grasp trials on its own. Therefore, they setup an environment where a robot constantly picks up objects that are located on a table. A camera detects the objects on the table and chooses one at random.
Then the robot tries to grasp that object at a point randomly sampled from the object's region of interest. Thereby, the gripper angle is also selected randomly. By evaluating the force sensor, they can verify the success of the action which means that they also generate negative data.

During training they augment the dataset by employing rotation transformation.
An image may be rotated by a random angle $$\theta_{rand}$$ and the corresponding grasp orientation is then labeled $$\theta_i + \theta_{rand}$$.

### Model

Instead of regressing an angle $$ \theta $$ they formulate finding $$ \theta $$ as an 18-way binary classification problem. For every 10° from 0° to 170° they predict a classification score whether that angle bucket is graspable or not.

The network is a basic AlexNet architecture, pretrained on ImageNet.

The loss function is given by

$$L_B = \sum_{i=1}^B \sum_{j=1}^{N=18} \delta(j, \theta_i) \cdot softmax(A_{ji}, l_i)$$

where
- $$ B $$ is the batch size
- $$ l_i \in \{0, 1\} $$ is the label corresponding to the angle $$ \theta_i $$
- $$ A_{ji} $$ is the forward pass binary activation on the angle bin $$ j $$
- $$ \delta(j, \theta_i) $$ = 1 when $$ \theta_i $$ corresponds to the $$ j^{th} $$ bin and $$ 0 $$ otherwise.

After training the network with some random grasp trials, they use the trained version to continue their data collection in order to increase the number of positive grasps. So this is basically some kind of reinforcement learning. Nevertheless, this method didn't have a very large impact on the performance (it gave an increase of 3%).

### Comparison to other baselines

Their approach achieved a classification accuracy of 79.5% and this was compared to other baselines.
Note that this accuracy is not the grasp success rate of the real robot which was 66% for novel objects and 73% for previously seen objects.
The following obvious grasping heuristic resulted in a 62.11% accuracy:

1. Grasp around the center of the patch.
2. Grasp the smallest object width (by doing segmentation + eigenvector analysis).
3. Do not grasp too thin objects.

Using HoG features, a kNN based classification resulted in 69.4% accuracy and using a linear SVM they observed a 73.3% accuracy.

### Limitations

- Although this method makes it much easier to create larger datasets, the data is highly specific to the particular robot used in the paper. 
A robot with a different gripper may lead to completely different results.

- They only rely on RGB information. 
It may be worthwhile to see how depth information could impact the results.

## Dex-Net 1.0

[Paper](https://goldberg.berkeley.edu/pubs/icra2016-final-dex-net-v20.pdf)



## Dex-Net 2.0

[Paper](https://arxiv.org/pdf/1703.09312.pdf)

## Dex-Net 2.1

[Paper](http://proceedings.mlr.press/v78/mahler17a/mahler17a.pdf)

## Autonomous Dual-Arm Manipulation of Familiar Objects

[Paper](https://arxiv.org/pdf/1811.08716.pdf)

### Segmentation

- RefineNet

### Shape Registration / Grasp Planning

- non-rigid shape registration
- Coherent Point Drift (CPD) non-rigid registration method
- deformation field
- Principal Component Analysis - Expectation Maximization (PCA-EM)
- Transferring category-based functional grasping skills by latent space non-rigid registration
- Transferring grasping skills to novel instances by latent space non-rigid registration
- Part-based grasp planning for familiar objects
- Functional power grasps transferred through warping and replanning

### Trajectory Optimiziation

- kinematic chain closure constraint: simply the constraint that the relative difference in translation and orientation stays roughly the motion

## Transferring Category-based Functional Grasping Skills by Latent Space Non-Rigid Registration

[Paper 1](https://arxiv.org/pdf/1809.05390.pdf)

[Paper 2](https://arxiv.org/pdf/1809.05353.pdf)

### Related Work

- Conformal maps
- isometry
- thin-plate-splines
- elasticity
- motion coherence theory

### Method

In summary, they perform a non-rigid regstration of each training model with a canonical model. 
By doing this they obtain a transformation matrix $$ W_i $$ for each individual training model.
Then, they construct row vectors $$ y_i $$ from these matrices $$ W_i $$ which they stack to form a desgin matrix $$ Y $$.
Using Principle Component Analysis Expectation Maximization (PCA-EM) on $$ Y $$ a lower-dimensional representation of the deformations from the canonical to the observed space is obtained.
This lower-dimensional representation of the deformation is then used as a feature vector to train a linear regressor that maps these feature vectors to grasp descriptors in canonical space.

{{< figure src="/img/robot-perception-skill/training-grasping-descriptors.png" title="Training" >}}

{{< figure src="/img/robot-perception-skill/inference-grasping-descriptors.png" title="Inference" >}}

### Limitations

- The data labeling process is very cumbersome because of 3D data.

- The idea of using a sequence of 3D poses as a grasp descriptor is suboptimal because it shows only a single valid grasping pose but there could be a lot more of course.
What if the robot approaches the object from a different side? This may work well for objects that have only a limited number of valid grasp poses such as a drill but is not well suited for bowls for example.

- Quite complex approach and may be hard to reimplement. Doesn't seem to be very scalable.

- Quite low success rate indicates that there must be something wrong.

## Ideas

- Instead of predicting grasps on entire images it may be easier to employ state of the art object detectors to extract the objects in question first. Then, we crop the image to the 
predicted bounding box and perform the grasp prediction on that crop.

- Use single-shot object detection networks architectures (e.g. YOLO).

- Do we actually need RGB information? It may be much easier if we simply ignore the RGB data and only use depth/point cloud data because in the end we need to transform everything into a 3D workspace.

- Can we incorporate some sort of unsupervised/semisupervised learning into this problem? That means that we have to somehow process unlabeled data.

- Predicting probability/robustness instead of point/rectangle. Some sort of segmentation?

# Development 

## ToDo

- play around with how to represent 3D meshes and point clouds in python and which data formats there exist
- visualize 3D data
- perform PCA on 3D data
- install PyCPD and play around with registration methods
- implement [this paper](https://arxiv.org/pdf/1809.05390.pdf)
- install Blender
- collect 3D meshes of chairs
- align the collected meshes
- transform chair meshes into point clouds
- install Ubuntu Mate 18 on SSD on my PC
- get ROS ARI and REEM-C simulations running
- work through the ARI and REEM-C tutorials
- figure out how to let a robot grasp an object in the simulation
- buy/rent RGB-D sensor
- collect real-life RGB-D images of chairs