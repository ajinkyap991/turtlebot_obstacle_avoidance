#pragma once
/**
 * @file botmotion.hpp
 * @author Ajinkya Parwekar
 * @brief The botmotion.hpp file.
 * It contains function definitions for moving the turtlebot3 in a simulated environment.
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

namespace mtb {

/**
 * @brief The Motion class
 */

class Motion {
 private:
    /**
     * @brief The distance of the robot from the obstacle in meters.
     */
    double dist;

 public:
    /**
     * @brief The base constructor for Motion class.
     * @param node ROS Nodehandle
     */
    explicit Motion(ros::NodeHandle node);

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& data);
};
}  // namespace mtb
