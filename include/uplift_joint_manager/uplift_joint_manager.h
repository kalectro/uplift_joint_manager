/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

//\Author Kai Franke, Robert Bosch LLC

#ifndef UPLIFT_JOINT_MANAGER_H_
#define UPLIFT_JOINT_MANAGER_H_

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

#include <dynamic_reconfigure/server.h>
#include <joint_driver/joint_driver.h>
#include <new_arduino_interface.hpp> 
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <moveit_msgs/MoveGroupActionResult.h>
#include <uplift_joint_manager/JointConfig.h>
#include <std_msgs/Float64.h>

// general defines
#define CREATE_NEW_ENCODER 255
#define ARDUINO_ARRIVED 0

// defines for spine
#define SPINE_PWM_PIN 6
#define SPINE_PWM_FREQUENCY 490
#define SPINE_DIRECTION_CONTROL1_PIN 8
#define SPINE_DIRECTION_CONTROL2_PIN 9
#define SPINE_ENCODER1_PIN 4
#define SPINE_ENCODER2_PIN 3
#define SPINE_ENCODER_MARKS_ON_STROKE 20000 //TODO: find correct number
#define SPINE_LENGTH 0.4 // TODO: find out correct length

// defines for arm
#define ARM_PWM_PIN 5
#define ARM_PWM_FREQUENCY 490
#define ARM_DIRECTION_CONTROL1_PIN 2
#define ARM_DIRECTION_CONTROL2_PIN 6
#define IMU_CHAIN_ID 1

#define JointPtr boost::shared_ptr< JointDriver >
#define TrajectoryMsg trajectory_msgs::JointTrajectory

#define IMU 1 // defines which IMU in the chain to use
//TODO: buy a longer cable so there is no need to attach a chain of IMUs

using namespace bosch_drivers_common;

std::vector<std::string> joint_names_;

float target_ = 0;
bool arm_disabled_ = false;
bool spine_disabled_ = false;
JointPtr arm_driver_position_;
JointPtr arm_driver_velocity_;
uint8_t arm_encoder_id_;
JointPtr spine_driver_position_;
JointPtr spine_driver_velocity_;
uint8_t spine_encoder_id_;
boost::shared_ptr< TrajectoryMsg > trajectory_desired_;
double position_influence_ = 0.5;
double velocity_influence_ = 0.5;
ros::Time start_time_trajectory_;
double roll_, pitch_, yaw_;

// maps joint names in message to known joints
std::vector<int> lookup;

int32_t point_counter_ = -1;

enum { SPINE, ARM };
/**
 * \brief Joint manager for the uplift
 *
 * This joint manager controls all available joints on the uplift robot. It will subscribe to the a planned
 * trajectory message and try to follow this trajectory using multiple instances of JointDriver to control
 * the position and the velocity of all joints
 */

int main(int argc, char **argv);

void arm_callback( uplift_joint_manager::JointConfig &config, uint32_t level );

void spine_callback( uplift_joint_manager::JointConfig &config, uint32_t level );

void trajectory_cb ( const moveit_msgs::MoveGroupActionResultConstPtr& desired );

#endif // UPLIFT_JOINT_MANAGER_H_
