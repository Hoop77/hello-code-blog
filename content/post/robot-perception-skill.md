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

### Creating the database

*Grasp Generation*

Sample up to $$ K = 250 $$ parallel-jaw grasps for each object by doing the following:

1. Sample random point $$ \textbf{c}_1 $$ on object surface (using signed disance function (SDF))
2. Sample grasping axis $$ \textbf{v} $$ from the friction cone around $$ \textbf{c}_1 $$
3. Find antipodal contact $$ \textbf{c}_2 $$ on the line $$ \textbf{c}_1 + t \textbf{v} $$
4. Define the grasp point $$ \textbf{g} $$ the center point between $$ c_1 $$ and $$ c_2 $$
5. Evaluate $$ P_F(\textbf{g}) $$ (Probability of force closure using soft finger contact model) using Monte-Carlo integration by sampling object pose, gripper pose and friction random variables $$ N = 500 $$ times and record the numbers of samples for which $$ \textbf{g}_k $$ achieved force closure

{{< figure src="/img/robot-perception-skill/dex-net-1-0-contact-model.png" title="Contact model" class="autosize" >}}

*Depthmap Gradient Features*

To measure grasp similarity they create a depthmap for each grasp which represents the feature space.
The depthmap is a local projection of the object surface around a contact point $$ c_i $$ (given by the grap $$ \textbf{g}_i$$). In order to make the map orientation-invariant they orient its axes along the eigenvectos of a weighted covariance matrix of the 3D surface points that generate the depthmap and refer to [this paper](https://s3.amazonaws.com/academia.edu.documents/42972440/SHOT_Unique_Signatures_of_Histograms_for20160223-22934-hz4swl.pdf?response-content-disposition=inline%3B%20filename%3DSHOT_Unique_signatures_of_histograms_for.pdf&X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Credential=ASIATUSBJ6BACFHCSTAZ%2F20200507%2Fus-east-1%2Fs3%2Faws4_request&X-Amz-Date=20200507T124036Z&X-Amz-Expires=3600&X-Amz-SignedHeaders=host&X-Amz-Security-Token=IQoJb3JpZ2luX2VjEIX%2F%2F%2F%2F%2F%2F%2F%2F%2F%2FwEaCXVzLWVhc3QtMSJGMEQCIFYStAhdcnGaPaLOXahS%2BMNLKNg8xEU1baRjgO%2FORvkNAiBF7LbBLVmWMHoSoqOzafVykssdVdJruav6Nncyi2Rnyyq9Awi9%2F%2F%2F%2F%2F%2F%2F%2F%2F%2F8BEAAaDDI1MDMxODgxMTIwMCIMmiL5CIIhFaQg9LJ7KpEDXymuN%2BcWFrYBxkP3NLOwDISoWi2SGdguJ%2FNy4zvvDUfVTmpWYIe6B9kPrjzBkJ%2BMSSnEIc6n8YzBNrlnzEYwUi1ddHe1Bo7jeUBlBomGUvjIcOi5ofhHMqgg0y2ci2zFfnyCySjvToULz%2BJhWUQSFdp6Jo%2Frv%2BPRJ1jCxYiZvQZU9lJxF3%2FrZoUUQWHds%2BVqo6RRqRh8V0DULiLzUFUjXpvK67HNlWJsVgbgnqA3QuXk5Qp7F3XpBPCmDF3%2FUlPdL8E6Y6SDqB76sRuI9gwDovHs8d4J9D3LG4wwx1lbNV5dHCcRxMsv0u7of7Q5NTNjIH9Zuv%2FTcRFf32bOEvCM1W%2FKKcHiIRfScnqa2SMB48SYXt7v6aFsppazUJEXGcVDDtJazzAPalKCoj%2BwdHbevIrInEz4ZElkAoBRl2n2spPU3VFNopF%2Fj3YOpZc1qHbXkLe4ajfYv5iIzD8HpOBFOJUVhK8DNEMdTnUsaDILPNUMTQOVDEcTCBVO9VFr6czYKRj8BbCSSYKJuSbM4ukLLFswsPjP9QU67AH9%2FIYIfinaCJ9M23WP7btgERZ9EdLJdUWtLS4YuBscpOFOzaIIqrLbkjai0PYJtLgYJIEBoJPdAsfBUCvCdnVF3hCUuxFTkPTSDifsjeTZOTWCk1cOnymvjR5j07AfdkpjL%2BAmQtw3CmVHz9AHMlfflYW4vteb868h1I%2BrL7YFtte%2FxVLX4iqRtCqIEaEiZxqgDv7eDPS3esu83hGQTIGqpSXgVnXlRUenipB%2FKriDmoEXQ6f2CpQai3QSoVPuZ89EfmO2NyraWjmGRJ8KLXgzzZ6ScoUpAHtk1ahcC5xRW4rKLtmQdJxxyyIuzw%3D%3D&X-Amz-Signature=bf5da3159fa60b416cdb980c4317301b798482d5b31999ca3a01d8495788b394).
Finally, they take the gradients $$ \nabla \textbf{d}_i = (\nabla_x \textbf{d}, \nabla_y \textbf{d}_i ) $$ and store them in the dataset.

{{< figure src="/img/robot-perception-skill/dex-net-1-0-local-surface-depthmaps.png" title="Local surface depthmaps" class="autosize" >}}

*Multi-View Convolutional Neural Network*

**Goal**: Index prior 3D object and grasp data by embedding each object in a vector space where distance represents object similarity.

**Method**: Render objects in multiple virtual camera views and use the resulting images to train a CNN to predict the corresponding object class.
When forwarding an image to the CNN, they maxpool the output of the last fully connected layer before the prediction.
Then, they use Principle Component Analysis to reduce the max-pooled output to a 100 dimensional vecture vector $$ \psi(\mathcal{O}) $$.
Now, they measure the dissimilarity between two objects $$ \mathcal{O}_i $$ and $$ \mathcal{O}_j $$ by the euclidean distance $$ \| \psi(\mathcal{O}_i) - \psi(\mathcal{O}_j) \| $$.

{{< figure src="/img/robot-perception-skill/dex-net-1-0-mvcnn.png" title="Mutli-View Convolutional Neural Network" class="autosize" >}}

### Predicting grasp for new objects

In order to find a suitable grasp pose for a new object, the Dex-Net 1.0 algorithm starts by using the same algorithm as described above to sample a set of $$ K $$ random grasp poses $$ \Gamma $$.
Now, the idea is to score these samples based on the chosen quality metric (in this case the probability of force closure $$ P_F $$) and information obtained from the grasp database.
So the goal is to combine the knowledge from the two sources.
This is achieved by rephrasing the objective of finding the best grasp as a reinforcement learning problem, namely a [Multi-Armed Bandit](https://en.wikipedia.org/wiki/Multi-armed_bandit) problem.

Each generated grasp $$ \textbf{g}_j $$ is associated with a Bernoulli random variable whoose parameter $$ \theta_j $$ represents the probability that the grasp succeeds.
To understand the analogy to the typical example of the Multi-Arm Bandit: the probability of winning at slot machine $$ j $$ would now be understood as the probability of making a successful grasp using $$ \textbf{g}_j $$.
Since the parameter of winning at a particular slot machine is not known, the agent has to make a series of trials to estimate the machines' expected reward.
Coming back to grasp predictions, the confusing part is that we somehow already have a probabilistic measure of success, namely the probability of force closure.
However, the actual goal is to incorporate prior grasp knowledge into the score.
What is a good to tool to combine different sources of knowledge? *Bayesian Inference*.

Therefore, we model each individual parameter $$ \theta_j $$ as a Beta distribution $$ \theta_j \sim Beta(\alpha_j, \beta_j) $$.
The $$ \alpha $$ parameter in a Beta distribution usually stands for the number of observed successful outcomes while $$ \beta $$ represents the number of failed observed outcomes of a random process.
Given these two numbers, the Beta distribution tells us how likely the **true** success rate $$ \theta $$ would be.
For example, if $$ \alpha = 8000 $$ and $$ \beta = 2000 $$ (8000 successful outcomes and 2000 failed outcomes), then $$ Beta(0.8, \alpha, \beta) $$ gives a high probability, meaning it is very likely that the true expected success rate would be 80%.
Likewise, $$ Beta(0.1, \alpha, \beta) $$ yields a very small probability value since it is extremely unlikely that the true expected success rate would be 10%, given the observed data.

In this case, the parameters $$ \alpha_j $$ and $$ \beta_j $$ quantize how many successful/failed grasps there are that are similar to the grasp $$ \textbf{g}_j $$.
The similarity between two grasps is measured using the distance of grasp poses, the depthmap gradients and the object similarity obtained from the Multi-View CNN.
In the course of performing Bayesian inference we want to iteratively update the Beta distributions for each individual grasp which in the end means to update $$ \alpha_j $$ and $$ \beta_j $$ in order to estimate the grasp success rate for each $$ \textbf{g}_j $$.
To do so, we need to define a prior distribution which means to find reasonable initial values $$ \alpha_{j, 0} $$ and $$ \beta_{j, 0} $$:

$$ \alpha_{j, 0} = \alpha_0 + \sum_{\textbf{g}_l \in \mathcal{D}} k(\textbf{g}_j, \textbf{g}_l) Z_l $$

$$ \beta_{j, 0} = \beta_0 + \sum_{\textbf{g}_l \in \mathcal{D}} k(\textbf{g}_j, \textbf{g}_l) (N_l - Z_l) $$

where $$ \mathcal{D} $$ is our grasp database and $$ \alpha_0 $$ and $$ \beta_0 $$ are just some constant parameters and can safely ignored at this point. The kernel $$ k $$ gives the similarity between two grasps and returns a values between 0 and 1 where higher values represent greater similiarity.
$$ Z_l $$ is the number of successful grasps that were observed for grasp $$ \textbf{g}_l $$ while $$ N_l $$ is the number of grasp trials performed with $$ \textbf{g}_l $$.
These numbers stem from our database and were determined offline.
Therefore, $$ \alpha_{j, 0} $$ and $$ \beta_{j, 0} $$ represent the number of successful and failed grasp trials weighted by grasp similarity.

Note that until now we made sense of our database as a prior for estimating the distribution of $$ \theta_j $$.
But of course, we also want to directly evaluate the quality metric for the newly seen grasps.
So why not just iterate through every $$ \textbf{g}_j $$ and compute its corresponding $$ P_F $$ value?
Because by doing so we could barely make use of our nice probabilistic methodology.
Instead, we want to it the Bayesian way and get a *posterior distribution* for the success rates $$ \theta_j $$.

A nice property of choosing a Beta distribution as the prior is that the posterior distribution is again a Beta distribution, characterized by some updated parameters $$ \alpha $$ and $$ \beta $$.
But how are these new parameters determined?
What the authors do is to choose a single grasp $$ \textbf{g}_j $$ from the $$ K $$ grasp samples using a method called Thompson sampling (which we will get to in a moment).
Then, they use the quality metric/simulation to figure out whether $$ \textbf{g}_j $$ would be a successful or a failed grasp and use this outcome to compute the updated posterior distribution parameters for every grasp $$ \textbf{g}_l $$:

$$ \alpha_{l,\ t} = \alpha_{l,\ t - 1} + k(\textbf{g}_j, \textbf{g}_l) S_j $$

$$ \beta_{l,\ t} = \beta_{l,\ t - 1} + k(\textbf{g}_j, \textbf{g}_l) (1 - S_j) $$

where $$ S_j \in \{ 0, 1 \} $$ represent the outcome.
Note that they computed the quality metric for only one particular grasp but transferred the gained knowledge across all other samples.
Furthermore, this sample $$ \textbf{g}_j $$ was retreived using Thompson sampling.
A grasp success rate $$ \hat{\theta_l} \sim p(\theta_l \ | \ \alpha_{l,\ t},\ \beta_{l,\ t}) $$ is sampled for each grasp $$ \textbf{g}_l \in \Gamma $$.
Then, we simply pick the grasp $$ \textbf{g}_j $$ for which the sampled success rate is highest. 
That's it. 
It is simple but an effective way of handling the exploration vs exploitation trade-off in a Multi-Armed Bandit problem.
This sampling and posterior update is repeated $$ T = 200 $$ times.

This pretty much covers the Dex-Net 1.0 algorithm leaving out a couple of details (for example on how the kernel $$ k $$ is computed), however, it hopefully unveils the core ideas.

{{< figure src="/img/robot-perception-skill/dex-net-1-0-algorithm.png" title="The Dex-Net 1.0 Algorithm for finding suitable grasp poses on a new object" >}}

## Dex-Net 2.0

[Paper](https://arxiv.org/pdf/1703.09312.pdf)

{{< figure src="/img/robot-perception-skill/dex-net-2-0-dataset-generation-pipeline.png" title="Dex-Net 2.0 pipeline for training dataset generation" class="autosize" >}}

{{< figure src="/img/robot-perception-skill/dex-net-2-0-gqcnn.png" title="Architecture of the GQ-CNN" class="autosize" >}}

### Method

dex-net-1-0-contact-model.png

## Dex-Net 2.1

[Paper](http://proceedings.mlr.press/v78/mahler17a/mahler17a.pdf)

## Dex-Net 4.0

[Paper](https://robotics.sciencemag.org/content/robotics/4/26/eaau4984.full.pdf)

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

## Method

- What kind of gripper/hand?
- How to represent grasps?
- How to generate data? simulation vs hand-labeled data
- How to get 3D models?

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