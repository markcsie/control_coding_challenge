# Intelligent Flying Machines 
We make humans as efficient as robots.

At the core, IFM is a Data Analytics Company using Machine Learning,
Computer Vision, and Robotics to automate indoor data capture. We develop
autonomous systems that know where they are and what they see using no
external infrastructure.

[![Intelligent Flying Machines](https://img.youtube.com/vi/AMDiR61f86Y/0.jpg)](https://www.youtube.com/watch?v=AMDiR61f86Y)

## flying with IFM
We are currently hiring Robotics Control Software Engineers to join our team full time in Chicago. A big part of our hiring process is designed around working through and implementing a control & state estimation pipeline from scratch. You will start with the basics and as you progress through the different stages in our interview process, you and your software will grow. If you make it all the way through to the very end, we will fly you in for a day, work hands on with you to implement the last parts and finally test and run your code on our robots in a real warehouse. 

## the challenge, stage 1
There are many types of sensors installed on our robots and systems. Much like smartphones, our robots have inertial measurement units (IMUs). In addition to these IMUs, our robots also carry distance sensors that tell them how far they are from the ground (distance sensors). In this challenge, you will implement a ROS node that takes measurements from both these sensors and fuses them together for an estimated height (z) from the ground. The key thing here is to ensure that your program can account for the differences in rates and noise (an IMU is much more noisy than a distance sensor) as well as compensate for tilt in the distance measurements. 

We provide you with a ROS bag file to test and develop your code with. [ROS Bag File](https://raw.githubusercontent.com/ifm-tech/control_coding_challenge/data/ifm_z_data.bag)

### the inputs to your program are: 
ground distance measurements (ROS topic: /ifm_sys/distance/ground)

raw imu data (ROS topic: /ifm_sys/imu/data) 

You will note that the IMU is quite noisy.

### the outputs of your program should be: 
the estimated height (z) of the robot based on the two inputs published on a ROS topic

Throughout our interview process, we will build upon this initial code to see how well you handle building complete control & state estimation pipelines, so make sure to keep everything organized. Since you will try to make this pipeline work with other datasets, it is very advisable to implement all parameters that you need as dynamic reconfigure options so you can change them on the fly (literally) with rqt. 

### here's a few things we like: 
1) robustness to outliers and timeouts 

2) very few parameters

3) efficient computations and comments that explain the math behind your implementation

4) if you decide to implement a specific type of filter, please explain why

## submission
To submit, please fork the repo and email us at jobs@ifm-tech.com

## what we look for
We look for elegantly written, maintainable, and reusable code. We want you to leverage test-driven principles that quantify your implementation's performance. 

And... 

<img src="https://img.devrant.io/devrant/rant/r_109448_5NyDp.jpg" >  
