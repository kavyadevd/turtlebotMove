/*
 * Copyright (C) BSD3.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file move.cpp
 * @author Kavyashree Devadiga
 * @date 28th November 2021
 * @copyright All rights reserved
 * @brief Move turtlebot methods file.
 */

// Include required headers
#include "../include/move.h"

#include <sstream>

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"

Move::Move(ros::NodeHandle nh) {
    ROS_INFO_STREAM("Initialized node.");
    ROS_DEBUG_STREAM("Move object created.");
    send_velocity = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 150);
    turtle_vel = 150;
    laser_scan = nh.subscribe("scan", 50, &Move::getLaserData, this);
}

    std::vector<double> collision_;
void Move::getLaserData(const sensor_msgs::LaserScan::ConstPtr &laser_data) {
    move_yn = true;
    auto start_ = laser_data->ranges.begin();
    auto end_ = laser_data->ranges.end() - 30;
    std::vector<double> laser_range;
            ROS_INFO_STREAM("blah");

    collision_ = std::vector<double>(end_, end_ + 30);
    laser_range = std::vector<double>(start_, start_ + 30);
    collision_.insert(collision_.end(), laser_range.begin(),
                      laser_range.end());
    lase_range_val = collision_[collision_.size()-1];
    for (auto &range : collision_) {
        ROS_INFO_STREAM("Laser Scan value:" << range << "\n");
        if (range < 0.4) {
            ROS_DEBUG_STREAM("Lase value: " << range);
            ROS_INFO_STREAM("Obstacle close");
            collision_yn = true;
            move_yn = false;
        }
        ROS_INFO_STREAM("Path clear.");
    }
}

void Move::stopMoving(ros::Publisher turtle_vel_) {
    robot_vel.linear.x = 0.0;
    robot_vel.linear.y = 0.0;
    robot_vel.linear.z = 0.0;
    robot_vel.angular.x = 0.0;
    robot_vel.angular.y = 0.0;
    robot_vel.angular.z = 0.0;
    turtle_vel_.publish(robot_vel);
}

void Move::turnBot(ros::NodeHandle nh, ros::Publisher turtle_vel_) {
    ROS_INFO_STREAM("Turning Robot.");
    robot_vel.angular.z = 0.5;
    turtle_vel_.publish(robot_vel);
    ros::Rate publish_rate(10);
    while (!move_yn) {
        ROS_DEBUG_STREAM("Move ahead flag false.");
        publish_rate.sleep();
        ros::spinOnce();
    }
    stopMoving(turtle_vel_);
    collision_yn = false;
}

void Move::startMoving(ros::NodeHandle nh, ros::Publisher turtle_vel_,
                       ros::Rate publish_rate) {
    if (collision_yn) {
        stopMoving(turtle_vel_);
        turnBot(nh, turtle_vel_);
    } else {
        robot_vel.linear.x = 0.5;
        robot_vel.angular.z = 0.0;
    }
    turtle_vel_.publish(robot_vel);
    ros::spinOnce();
    publish_rate.sleep();
    ROS_INFO_STREAM("Velocity linear:" << robot_vel.linear.x << " Velocity angular: " << robot_vel.angular.z);


    ROS_INFO_STREAM("Laser Scan value:" << lase_range_val << "\n");
}

Move::~Move() {}
