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
 * @file move.h
 * @author Kavyashree Devadiga
 * @date 28th November 2021
 * @copyright All rights reserved
 * @brief Header file for moving turtlebot
 */

#ifndef INCLUDE_MOVE_
#define INCLUDE_MOVE_

// Include required headers
#include <sstream>

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"

class Move {
   private:
    double turtle_vel;               // move velocity of turtlebot
    ros::Publisher send_velocity;    // variable to publish velocity
    ros::Subscriber laser_scan;      // object for laser scan
    geometry_msgs::Twist robot_vel;  // Robot twist message object
    bool collision_yn = false;
    bool turn_yn = false;
    bool move_yn = true;
    float lase_range_val;

   public:
    /**
     * @brief Constructor for move class
     * @param ROS node handler
     */
    explicit Move(ros::NodeHandle);

    /**
     * @brief Initialize turtlebot move
     *
     * @param ROS nodehandler
     * @param ROS Publisher
     * @param Rate of publisher
     */
    void startWalk(ros::NodeHandle, ros::Publisher, ros::Rate);

    /**
     * @brief Read data from laser scan
     */
    void getLaserData(const sensor_msgs::LaserScan::ConstPtr&);

    /**
     * @brief Stops turtlebot by setting all velocities to zero
     */
    void stopMoving(ros::Publisher);

    /**
     * @brief Function to turn turtlebot
     */
    void turnBot(ros::NodeHandle, ros::Publisher);

    /**
     * @brief Start moving turtlebot
     */
    void startMoving(ros::NodeHandle, ros::Publisher, ros::Rate);

    /**
     * @brief Destructor
     *
     */
    ~Move();
};

#endif  //   INCLUDE_MOVE_
