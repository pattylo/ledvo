# LEDVO ("LE"arning "D"ynamic Factors for Optimization-based "V"isual "O"dometry)

<!-- ## UNDER CONSTRUCTION! -->

## Learning Dynamic Factors for Optimization-based Visual Odometry

This is a project for COMP5212 Machine Learning.

<!-- ### Abstract
<div align="justify">
This research addresses the relative pose estimation problem of aerial vehicles; in particular from the perspective of ground-based sensing and control. Over the years, a wide range of research could be found addressing this problem, nevertheless, we tackle the problem with more information: vision, dynamic model, and control inputs. Such adopted method is believed to increase the robustness of state estimation and hence further ensure the stability of a offboard control framework. We first formulate the problem as a factor graph optimization and restrict the dimension size with sliding window. Successively, through minimizing the visual residuals and dynamic residuals, the optimal state could be then retrieved. Specifically, we depose IMU information due to the requirement of bideirectional communication, which could potentially cause transmition congestion. Furthermore, we demonstrate the performance by conducting a series of experiments, in which comparisons are also made with other algorithms. The results show that with the designed configuration while omitting IMU data from bidirectional communication, promosing results could be achieved. 
</div>

### Video

### Software Installation
#### Pre-Requisite

```
# we tested on system with ubuntu 18.04 and 20.04 
# install Ubuntu 18.04 || 20.04
# install ROS
# install OpenCV (should come along with ROS)

sudo apt-get install ros-noetic-sophus
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen #if neccessary

```

This repo utilize some third-party libraries,
- [g2o](https://github.com/RainerKuemmerle/g2o) to solve the non-linear least square optimization problem.


# If you are using scount_ros, feel free to use our scout_ros package
```
mkdir -p {name scout}_ws/src
cd {name scout}_ws/src
git clone https://github.com/HKPolyU-UAV/scout_ros
catkin_make
```
For scout_ros, click [here](https://github.com/agilexrobotics/scout_ros.git) to know more. They have done a fantastic job on packaging their hardware platform.

The above packages salute the contribution of the following academic paper: -->

