/**
 * @file botmotion.cpp
 * @author Ajinkya Parwekar
 * @brief The botmotion.cpp file.
 * It contains functions implementation for moving the turtlebot3 in a simulated environment.
 * @Copyright "Copyright 2020" <Ajinkya Parwekar>
 *
 * @section LICENSE
 *  
 * MIT License
 * Copyright (c) 2020 Ajinkya Parwekar
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include "geometry_msgs/Twist.h"
#include "botmotion.hpp"

void mtb::Motion::laserCallback(const sensor_msgs::LaserScan::ConstPtr& data) {
    dist = data->ranges[180];
    ROS_INFO_STREAM("Distance from the obstacle is: " << dist);
}

    mtb::Motion::Motion(ros::NodeHandle node) {
    // Subscriber for topic LaserScan
    ros::Subscriber laserSubscriber =
      node.subscribe("/scan", 1000, &Motion::laserCallback, this);

    // Publisher for topic velocity
    ros::Publisher velocityPublisher =
      node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // Setting looprate of 4 Hz
    ros::Rate rate(4);

    while (ros::ok()) {
        geometry_msgs::Twist twist;

        // Initializing all values to zero
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = 0.0;

        // Movement logic: if distance from the sensor data is more than 45 cm
        // Consider no obstacle and keep moving forward at the steps of 12 cm
        if (dist > 0.45) {
            ROS_INFO_STREAM("Moving ahead...");
            twist.linear.x = -0.12;
        // If obstacle is detected, rotate by 80 degrees about the z axis
        } else {
            ROS_INFO_STREAM("Obstacle Detected; Changing direction...");
            twist.angular.z = 1.4;
        }

        velocityPublisher.publish(twist);
        ros::spinOnce();
        rate.sleep();
    }
}
