---
title: "Building a self-driving car - Part I"
description: "From part selection to teleoperation."
author:
  name: "Philipp Badenhoop"
date: 2019-05-03
draft: true
tags:
- Autonomous Driving
- Robotics
image: yoshicar-dark.jpeg
markup: mmark
---

# Part Selection

I really started to miss the times hacking on our cute little robocar from college... 

... so I decided to build my own.
This time I want to experiment with indoor navigation using LIDAR. As poor students don't have much money, I focused on keeping the costs as low as possible.
Here is the list of its parts:

{.table .table-striped}
Part | Product Name
--- | ---
The car itself | H-King Rattler 1/8 4WD Buggy V2 (ARR) with 60A ESC
The brain | Raspberry Pi 3 b+
Micro-SD card | 64GB SanDisk Ultra microSDHC Memory Card
Voltage converter | DC to DC 4.5-30V to 1-30V 12A Buck Converter Step Down
PWM board | AZDelivery PCA9685 16 Channel 12 Bit PWM Servoid Driver
Wifi-adapter | Edimax EW-7811UN 150Mbps Wireless Nano USB Adapter
LIDAR sensor | RPLidar A1M8
IMU sensor | Adafruit BNO055
Battery (2x) | FLOUREON RC Battery 2s 7.4 V 5200 mAh
Charging station | VOLTCRAFT V-Charge 60

Altogether, the car costs about 400€.

# Wiring

# Software Setup

## Install Ubuntu image

I used [this](https://downloads.ubiquityrobotics.com/) Ubuntu 16 image which you can easily write on your SD-card using GParted.
The image has ROS Kinetic already installed.

## Configure static IP address

I recommend setting up a static IP address. 
TODO

## Controlling PWM signals

Let's start by setting up a fresh ROS workspace.
Therefore, just type the following into your terminal.

{{< highlight bash "linenos=inline" >}}
$ mkdir ros_ws
$ cd ros_ws
$ mkdir src
$ catkin_make
{{< / highlight >}}

By running **catkin_make**, we intialize the workspace which assumes that there exists a src folder in the current working directory.
Next, we will create a ROS package in which we create a python node that runs the PWM related stuff.
To create the package, just type the following into your command prompt.

{{< highlight bash "linenos=inline" >}}
$ catkin_create_pkg pwm rospy std_msgs
$ cd src/pwm
$ mkdir src
{{< / highlight >}}

In the following pwm.py file, 

{{< highlight python "linenos=inline" >}}
#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import Adafruit_PCA9685

PWM_FREQ = 50
MOTOR_CHANNEL = 13
SERVO_CHANNEL = 14

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(PWM_FREQ)

def set_pulse(channel, pulse):
    pulse = PWM_FREQ * pulse * 4096
    pwm.set_pwm(channel, 0, int(pulse))

def set_value(channel, value):
    value = min(max(value, -1.0), 1.0)
    pulse = 0.0015 + value * 0.0005
    set_pulse(channel, pulse)
    rospy.logdebug("set pulse: %f on channel channel %d", pulse, channel)

def motor_callback(data):
    value = data.data
    rospy.logdebug(rospy.get_caller_id() + "motor: %f", value)
    set_value(MOTOR_CHANNEL, value)

def servo_callback(data):
    value = data.data
    rospy.logdebug(rospy.get_caller_id() + "servo %f", value)
    set_value(SERVO_CHANNEL, value)
    
def main():
    rospy.init_node('pwm')
    rospy.Subscriber("pwm/motor", Float64, motor_callback)
    rospy.Subscriber("pwm/servo", Float64, servo_callback)
    rospy.spin()
  
if __name__ == '__main__':
    main()
{{< / highlight >}}

TODO: Before this can run, you have to configure i2c interface and pip install the adafruit driver.

{{< figure src="/img/yoshicar.jpeg" title="" class="autosize" >}}