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

#include <uplift_joint_manager/uplift_joint_manager.h>

void callback( uplift_joint_manager::SpineConfig &config, uint32_t level ) 
{ 
  // convert enums from cfg file into the in the h files
  bosch_drivers_common::motor_drive_mode motor_mode = (bosch_drivers_common::motor_drive_mode)config.motor_mode;
  
  switch ( motor_mode )
  {
    case 0: _spine_disabled = false; break;
    case 1: _spine_disabled = true; break;
    case 2: _spine_disabled = true; break;
    default: ROS_ERROR("bad motor mode");
  }
  _spine_driver_position->setMode( motor_mode );

  _spine_driver_position->pid->setParams( config.p_gain, config.i_gain, config.d_gain, config.max_integral );
  
  // if influence has changed, adjust the other influence value
  if ( config.position_influence > _position_influence + 0.01 || config.position_influence < _position_influence - 0.01 )
  {
    config.velocity_influence = 1.0 - config.position_influence;
  }
  if ( config.velocity_influence > _velocity_influence + 0.01 || config.velocity_influence < _velocity_influence - 0.01 )
  {
    config.position_influence = 1.0 - config.velocity_influence;
  }
  
  ROS_INFO("New control parameters set to  P_gain:%f I_gain:%f D_gain:%f", config.p_gain, config.i_gain, config.d_gain);
}

void trajectory_cb ( const moveit_msgs::MoveGroupActionResultConstPtr& desired )
{
  ROS_INFO("new trajectory received");
  // copy message
  _trajectory_desired = desired->result.planned_trajectory.joint_trajectory;
    
  // map joint names in message to known joints
  for( size_t i = 0; i < _joint_names.size(); ++i)
  {
    for( size_t j = 0; j < _trajectory_desired.joint_names.size(); ++j )
    {
      if( _trajectory_desired.joint_names[j] == _joint_names[i] )
      {
        lookup->at(i) = j;
        break;
      }
    }
    if (lookup->at(i) == -1)
    {
      ROS_ERROR("Unable to locate joint %s in the commanded trajectory.", _joint_names[i].c_str());
      return;
    }
  }
  
  // reset counter for active point in trajectory
  _point_counter = 0;
}


int main(int argc, char **argv) 
{
  ros::init(argc, argv, "uplift_joint_manager");
  ros::NodeHandle nh;
  
  std::string hw_id;
  std::string hw_id2;  
  nh.param<std::string>( "hardware_id1", hw_id, "/dev/ttyACM0" );
  nh.param<std::string>( "hardware_id2", hw_id2, "/dev/ttyACM1" );
  
  _joint_names.resize(2);
  _joint_names[SPINE] = "spine";
  _joint_names[ARM] = "arm";
  
  // create lookup table with the same size as the number of joints 
  lookup( new std::vector<int>(_joint_names.size(), -1));

  ArduinoInterface Arduino( hw_id );
  if( Arduino.initialize() == false)
  {
    ROS_ERROR("Error initializing Arduino");
  }  


  // create joint driver for position control for the spine
	_spine_driver_position( new JointDriver( 
                            &Arduino,                                                                                                 // hardware interface
                            SPINE_PWM_PIN, SPINE_PWM_FREQUENCY, SPINE_DIRECTION_CONTROL1_PIN, SPINE_DIRECTION_CONTROL2_PIN,           // Motor pins and settings
                            SPINE_ENCODER1_PIN, SPINE_ENCODER2_PIN, SPINE_ENCODER_MARKS_ON_STROKE, CREATE_NEW_ENCODER, SPINE_LENGTH,  // Encoder pins and settings
                            JointDriver::POSITION)); 	                                                                                // Joint control mode
  // initalize joint and connected hardware
  _spine_driver_position->initialize(); 
  // retrieve encoder id to use in next joint driver
  _spine_encoder_id = _spine_driver_position->getEncoderID();
  
  // creat joint driver for velocity control for the spine
	_spine_driver_velocity( new JointDriver( 
	                          &Arduino,                                                                                                 // hardware interface
	                          SPINE_PWM_PIN, SPINE_PWM_FREQUENCY, SPINE_DIRECTION_CONTROL1_PIN, SPINE_DIRECTION_CONTROL2_PIN,           // Motor pins and settings
	                          SPINE_ENCODER1_PIN, SPINE_ENCODER2_PIN, SPINE_ENCODER_MARKS_ON_STROKE, _spine_encoder_id, SPINE_LENGTH,   // Encoder pins and settings
	                          JointDriver::VELOCITY)); 	                                                                                // Joint control mode
	// initalize joint and connected hardware
	_spine_driver_velocity->initialize();
	
	
	ros::Subscriber sub = nh.subscribe ("/move_group/result", 5, trajectory_cb);
	ros::Publisher pub = nh.advertise<sensor_msgs::JointState> ("real_state", 5);
	
	dynamic_reconfigure::Server<uplift_joint_manager::SpineConfig> server;
  dynamic_reconfigure::Server<uplift_joint_manager::SpineConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  
  ROS_INFO("Running control loop");
  ros::Rate loop_rate_Hz(100);
  ros::Duration duration_between_points;
  
  while ( ros::ok() )
  {
    if( _point_counter < 0 ) // no trajectory to follow
    {
      ros::spinOnce();
      loop_rate_Hz.sleep();
      continue;
    }
      
    if( (uint32_t)_point_counter < _trajectory_desired.points.size() - 1)
    {
      _point_counter++;
      duration_between_points = _trajectory_desired.points[_point_counter].time_from_start - _trajectory_desired.points[_point_counter - 1].time_from_start;
    }
    double current_velocity = _spine_driver_velocity->getPosition();
    double output_velocity_control = _spine_driver_velocity->compute( current_velocity, _trajectory_desired.points[_point_counter].velocities[lookup->at(SPINE)] );
    ROS_INFO("velocity: current:%f  target:%f  output:%f", current_velocity, _trajectory_desired.points[_point_counter].velocities[lookup->at(SPINE)], output_velocity_control );
    
    double current_position = _spine_driver_position->getPosition();//TODO getPosition correct?
    double output_position_control = _spine_driver_position->compute( _trajectory_desired.points[_point_counter].position[lookup->at(SPINE)] );
    ROS_INFO("position: current:%f  target:%f  output:%f", current_position, _trajectory_desired.points[_point_counter].position[lookup->at(SPINE)], output_position_control );
    
    _spine_driver_velocity->applyOutput( (output_velocity_control * _velocity_influence) + (output_position_control * _position_influence) );
    
    ros::spinOnce();
    duration_between_points.sleep();
  }    
  
  return 0;
}
