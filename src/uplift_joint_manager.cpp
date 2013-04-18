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

void spine_cb( uplift_joint_manager::JointConfig &config, uint32_t level ) 
{ 
  static int32_t control_mode_ = 0;
  
  if ( config.control_mode != control_mode_ )
  {
    double p, i, d, max_integral;
    control_mode_ = config.control_mode;
    switch ( control_mode_ )
    {
      case 0: 
      {
        spine_driver_position_->pid->getParams( p, i, d, max_integral );
        config.p_gain = p;
        config.i_gain = i;
        config.d_gain = d;
        config.max_integral = max_integral;
      }break;
      
      case 1:
      {
        spine_driver_velocity_->pid->getParams( p, i, d, max_integral );
        config.p_gain = p;
        config.i_gain = i;
        config.d_gain = d;
        config.max_integral = max_integral;
      }break;
      
      default: ROS_ERROR("bad joint control mode"); return;
    }   
  }
  
  // convert enums from cfg file into the in the h files
  motor_drive_mode motor_mode = (motor_drive_mode)config.motor_mode;
  
  switch ( motor_mode )
  {
    case DRIVE: spine_disabled_ = false; break;
    case FREE_RUNNING: spine_disabled_ = true; break;
    case BRAKE: spine_disabled_ = true; break;
    default: ROS_ERROR("bad motor mode");
  }
  spine_driver_position_->setMode( motor_mode );

  switch ( control_mode_ )
  {
    case JointDriver::POSITION: 
    {
      spine_driver_position_->pid->setParams( config.p_gain, config.i_gain, config.d_gain, config.max_integral );
    }break;
    
    case JointDriver::VELOCITY:
    {
      spine_driver_velocity_->pid->setParams( config.p_gain, config.i_gain, config.d_gain, config.max_integral );
    }break;
  }
  
  // if influence has changed, adjust the other influence value
  if ( config.position_influence > position_influence_ + 0.01 || config.position_influence < position_influence_ - 0.01 )
  {
    config.velocity_influence = 1.0 - config.position_influence;
    position_influence_ = config.position_influence;
    velocity_influence_ = config.velocity_influence;
  }
  if ( config.velocity_influence > velocity_influence_ + 0.01 || config.velocity_influence < velocity_influence_ - 0.01 )
  {
    config.position_influence = 1.0 - config.velocity_influence;
    position_influence_ = config.position_influence;
    velocity_influence_ = config.velocity_influence;
  }
  
  ROS_INFO("New control parameters set to  P_gain:%f I_gain:%f D_gain:%f", config.p_gain, config.i_gain, config.d_gain);
}

void arm_cb( uplift_joint_manager::JointConfig &config, uint32_t level ) 
{ 
  static int32_t control_mode_ = 0;
  
  if ( config.control_mode != control_mode_ )
  {
    double p, i, d, max_integral;
    control_mode_ = config.control_mode;
    switch ( control_mode_ )
    {
      case 0: 
      {
        arm_driver_position_->pid->getParams( p, i, d, max_integral );
        config.p_gain = p;
        config.i_gain = i;
        config.d_gain = d;
        config.max_integral = max_integral;
      }break;
      
      case 1:
      {
        arm_driver_velocity_->pid->getParams( p, i, d, max_integral );
        config.p_gain = p;
        config.i_gain = i;
        config.d_gain = d;
        config.max_integral = max_integral;
      }break;
      
      default: ROS_ERROR("bad joint control mode"); return;
    }   
  }
  
  // convert enums from cfg file into the in the h files
  motor_drive_mode motor_mode = (motor_drive_mode)config.motor_mode;
  
  switch ( motor_mode )
  {
    case DRIVE: arm_disabled_ = false; break;
    case FREE_RUNNING: arm_disabled_ = true; break;
    case BRAKE: arm_disabled_ = true; break;
    default: ROS_ERROR("bad motor mode");
  }
  arm_driver_position_->setMode( motor_mode );

  switch ( control_mode_ )
  {
    case JointDriver::POSITION: 
    {
      arm_driver_position_->pid->setParams( config.p_gain, config.i_gain, config.d_gain, config.max_integral );
    }break;
    
    case JointDriver::VELOCITY:
    {
      arm_driver_velocity_->pid->setParams( config.p_gain, config.i_gain, config.d_gain, config.max_integral );
    }break;
  }
  
  // if influence has changed, adjust the other influence value
  if ( config.position_influence > position_influence_ + 0.01 || config.position_influence < position_influence_ - 0.01 )
  {
    config.velocity_influence = 1.0 - config.position_influence;
    position_influence_ = config.position_influence;
    velocity_influence_ = config.velocity_influence;
  }
  if ( config.velocity_influence > velocity_influence_ + 0.01 || config.velocity_influence < velocity_influence_ - 0.01 )
  {
    config.position_influence = 1.0 - config.velocity_influence;
    position_influence_ = config.position_influence;
    velocity_influence_ = config.velocity_influence;
  }
  
  ROS_INFO("New control parameters set to  P_gain:%f I_gain:%f D_gain:%f", config.p_gain, config.i_gain, config.d_gain);
}

void trajectory_cb ( const moveit_msgs::MoveGroupActionResultConstPtr& desired )
{
  ROS_INFO("new trajectory received");
  // copy message
  trajectory_desired_.reset( new TrajectoryMsg( desired->result.planned_trajectory.joint_trajectory ) );
    
  // map joint names in message to known joints
  for( size_t i = 0; i < joint_names_.size(); ++i)
  {
    for( size_t j = 0; j < trajectory_desired_->joint_names.size(); ++j )
    {
      if( trajectory_desired_->joint_names[j] == joint_names_[i] )
      {
        lookup[i] = j;
        break;
      }
    }
    if (lookup[i] == -1)
    {
      ROS_ERROR("Unable to locate joint %s in the commanded trajectory.", joint_names_[i].c_str());
      return;
    }
  }
  
  // reset counter for active point in trajectory
  point_counter_ = 0;
  // specifies the time the trajectory was started
  start_time_trajectory_ = ros::Time::now();
}

int main(int argc, char **argv) 
{
  //
  // initialize ROS
  //
  ros::init(argc, argv, "uplift_joint_manager");
  ros::NodeHandle nh;
  ros::NodeHandle nh_feedback("~");
  
  std::string hw_id_arm;
  std::string hw_id_spine;  
  nh.param<std::string>( "hardware_id_arm", hw_id_arm, "/dev/ttyACM0" );
  nh.param<std::string>( "hardware_id_spine", hw_id_spine, "/dev/ttyACM1" );
  
  joint_names_.resize(2);
  joint_names_[SPINE] = "spine";
  joint_names_[ARM] = "arm";
  
  double current_position[joint_names_.size()], current_velocity[joint_names_.size()];
  
  sensor_msgs::JointState robot_state;
  robot_state.name.resize(2);
  robot_state.position.resize(2);
  robot_state.velocity.resize(2);
  robot_state.name[SPINE] ="spine";
  robot_state.name[ARM] ="arm";
  //robot_state.name[2] ="single_finger";
  //robot_state.name[3] ="double_finger";
  
  //robot_state.position[2] = 0.0;
  //robot_state.velocity[2] = 0.0;
  //robot_state.position[3] = 0.0;
  //robot_state.velocity[3] = 0.0;
  
  
  // create lookup table with the same size as the number of joints 
  lookup.resize( joint_names_.size() );

  if( ARDUINO_ARRIVED )
  {  
    NewArduinoInterface Arduino_Spine( hw_id_spine );
    if( Arduino_Spine.initialize() == false)
    {
      ROS_ERROR("Error initializing Arduino_Spine");
    }  

    // create joint driver for position control for the spine
	  spine_driver_position_.reset( new JointDriver(
            &Arduino_Spine,                                                                                           // hardware interface
            SPINE_PWM_PIN, SPINE_PWM_FREQUENCY, SPINE_DIRECTION_CONTROL1_PIN, SPINE_DIRECTION_CONTROL2_PIN,           // Motor pins and settings
            SPINE_ENCODER1_PIN, SPINE_ENCODER2_PIN, SPINE_ENCODER_MARKS_ON_STROKE, CREATE_NEW_ENCODER, SPINE_LENGTH,  // Encoder pins and settings
            JointDriver::POSITION)); 	                                                                                // Joint control mode
    // initalize joint and connected hardware
    spine_driver_position_->initialize(); 
    // invert encoder values
    spine_driver_position_->encoder_->invertOutput();
    // retrieve encoder id to use in next joint driver
    spine_encoder_id_ = spine_driver_position_->getEncoderID();
    
    // creat joint driver for velocity control for the spine
	  spine_driver_velocity_.reset( new JointDriver( 
            &Arduino_Spine,                                                                                           // hardware interface
            SPINE_PWM_PIN, SPINE_PWM_FREQUENCY, SPINE_DIRECTION_CONTROL1_PIN, SPINE_DIRECTION_CONTROL2_PIN,           // Motor pins and settings
            SPINE_ENCODER1_PIN, SPINE_ENCODER2_PIN, SPINE_ENCODER_MARKS_ON_STROKE, spine_encoder_id_, SPINE_LENGTH,   // Encoder pins and settings
            JointDriver::VELOCITY)); 	                                                                                // Joint control mode
	  // initalize joint and connected hardware
	  spine_driver_velocity_->initialize();
	  // invert encoder values
	  spine_driver_velocity_->encoder_->invertOutput();
	}
	
	NewArduinoInterface Arduino_Arm( hw_id_arm );
  if( Arduino_Arm.initialize() == false)
  {
    ROS_ERROR("Error initializing Arduino_Arm");
  }
  
  // create joint driver for position control for the arm
  arm_driver_position_.reset( new JointDriver(
            &Arduino_Arm,                                                                             // hardware interface
            ARM_PWM_PIN, ARM_PWM_FREQUENCY, ARM_DIRECTION_CONTROL1_PIN, ARM_DIRECTION_CONTROL2_PIN,   // Motor pins and settings
            IMU_CHAIN_ID,                                                                             // IMU settings
            JointDriver::POSITION)); 	                                                                // Joint control mode
    // initalize joint and connected hardware
    arm_driver_position_->initialize(); 
    
  // create joint driver for velocity control for the arm
  arm_driver_velocity_.reset( new JointDriver(
            &Arduino_Arm,                                                                             // hardware interface
            ARM_PWM_PIN, ARM_PWM_FREQUENCY, ARM_DIRECTION_CONTROL1_PIN, ARM_DIRECTION_CONTROL2_PIN,   // Motor pins and settings
            IMU_CHAIN_ID,                                                                             // IMU settings
            JointDriver::VELOCITY)); 	                                                                // Joint control mode
	  // initalize joint and connected hardware
	  // arm_driver_velocity_->initialize();  // not neccessay because IMU was already initialized
	  
	  
	ros::Subscriber sub = nh.subscribe ("/move_group/result", 1, trajectory_cb);
	ros::Publisher pub = nh.advertise<sensor_msgs::JointState> ("/joint_states", 1);

  ros::Publisher pub_position_target_spine = nh_feedback.advertise<std_msgs::Float64> ("feedback/spine/position/target", 5);
  ros::Publisher pub_velocity_target_spine = nh_feedback.advertise<std_msgs::Float64> ("feedback/spine/velocity/target", 5);

	ros::Publisher pub_position_target_arm   = nh_feedback.advertise<std_msgs::Float64> ("feedback/arm/position/target", 5);
	ros::Publisher pub_velocity_target_arm   = nh_feedback.advertise<std_msgs::Float64> ("feedback/arm/velocity/target", 5);
	
	//
	// dynamic reconfigure settings
	//
	if( ARDUINO_ARRIVED )
	{
	  dynamic_reconfigure::Server<uplift_joint_manager::JointConfig> server_spine;
	  dynamic_reconfigure::Server<uplift_joint_manager::JointConfig>::CallbackType spine_config;
	  spine_config = boost::bind(&spine_cb, _1, _2);
    server_spine.setCallback(spine_config);
  }
  
	dynamic_reconfigure::Server<uplift_joint_manager::JointConfig> server_arm;
  dynamic_reconfigure::Server<uplift_joint_manager::JointConfig>::CallbackType arm_config;
  arm_config = boost::bind(&arm_cb, _1, _2);
  server_arm.setCallback(arm_config);
  
  
  ROS_INFO("Running control loop");
  ros::Rate loop_rate_Hz(100);
  ros::Duration duration_between_points;

  while ( ros::ok() )
  {
    // record time of measurements
    ros::Time now = ros::Time::now();
    
    // check if any trajectory has been received yet (point_counter_ is initialized with -1)
    if( point_counter_ < 0 )
    {
      ROS_DEBUG("Waiting for first trajectory to be published");
      ros::spinOnce();
      loop_rate_Hz.sleep();
      continue;
    }
    if( (uint32_t)point_counter_ < trajectory_desired_->points.size() - 1)
    {
      if( now > start_time_trajectory_ + trajectory_desired_->points[point_counter_].time_from_start )
      {
        point_counter_++;
      }
    }
    
    
    //
    // measure current angles, positions and velocities of the robot and publish robot state
    //
    if( ARDUINO_ARRIVED )
	  {
      current_velocity[SPINE] = spine_driver_velocity_->getState();
      current_position[SPINE] = spine_driver_position_->getState();
      robot_state.position[SPINE] = current_position[SPINE];
      robot_state.velocity[SPINE] = current_velocity[SPINE];
    }

    current_velocity[ARM] = arm_driver_velocity_->getState();
    current_position[ARM] = arm_driver_position_->getState();
    robot_state.position[ARM] = current_position[ARM];
    robot_state.velocity[ARM] = current_velocity[ARM];
    
    robot_state.header.stamp = now;
    
    pub.publish( robot_state );
    

    //
    // SPINE control
    // extract trajectory target information and apply new control to spine motor
    //    
    if( ARDUINO_ARRIVED )
	  {
      double target_velocity = trajectory_desired_->points[point_counter_].velocities[lookup[SPINE]];
      double output_velocity_control = spine_driver_velocity_->compute( current_velocity[SPINE], target_velocity ); 
                    
      ROS_DEBUG("spine velocity: current:%f  target:%f  output:%f", current_velocity[SPINE], target_velocity, output_velocity_control );
      
      double target_position = trajectory_desired_->points[point_counter_].positions[lookup[SPINE]];
      double output_position_control = spine_driver_position_->compute( current_position[SPINE], target_position );
      
      ROS_DEBUG("spine position: current:%f  target:%f  output:%f", current_position[SPINE], target_position, output_position_control );
      
      // weigh position and velocity output and apply
      spine_driver_velocity_->applyOutput( (output_velocity_control * velocity_influence_) + (output_position_control * position_influence_) );
      
      // publish feedback information
      std_msgs::Float64 temp;
      temp.data = target_position;
      pub_position_target_spine.publish( temp );
      temp.data = target_velocity;
      pub_velocity_target_spine.publish( temp );
    }
    
    
    //
    // ARM control
    // extract trajectory target information and apply new control to arm motor
    //
    double target_velocity = (-1.0) * trajectory_desired_->points[point_counter_].velocities[lookup[ARM]];  // adjust rotation direction
    double output_velocity_control = arm_driver_velocity_->compute( current_velocity[ARM], target_velocity ); 
                  
    ROS_DEBUG("arm velocity: current:%f  target:%f  output:%f", current_velocity[ARM], target_velocity, output_velocity_control );
    
    double target_position = (-1.0) * trajectory_desired_->points[point_counter_].positions[lookup[ARM]];  // adjust rotation direction
    double output_position_control = arm_driver_position_->compute( current_position[ARM], target_position );
    
    ROS_DEBUG("arm position: current:%f  target:%f  output:%f", current_position[ARM], target_position, output_position_control );
    
    // weigh position and velocity output and apply
    arm_driver_velocity_->applyOutput( (output_velocity_control * velocity_influence_) + (output_position_control * position_influence_) );
    
    // publish feedback information
    std_msgs::Float64 temp;
    temp.data = target_position;
    pub_position_target_arm.publish( temp );
    temp.data = target_velocity;
    pub_velocity_target_arm.publish( temp );
    
    ros::spinOnce();
    loop_rate_Hz.sleep();
  }    
  
  return 0;
}
