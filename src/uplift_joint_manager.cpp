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


//
//  dynamic reconfigure callback function
//
void dynamic_cb( uplift_joint_manager::JointConfig &config, uint32_t level ) 
{ 
  static int32_t arm_control_mode = 0;
  static int32_t spine_control_mode = 0;
  
  calibrate_height_ = config.calibrate_height;
  
  if ( config.spine_control_mode != spine_control_mode )
  {
    double p, i, d, max_integral;
    spine_control_mode = config.spine_control_mode;
    switch ( spine_control_mode )
    {
      case 0: 
      {
        spine_driver_position_->getPidPtr()->getParams( p, i, d, max_integral );
        config.spine_p_gain = p;
        config.spine_i_gain = i;
        config.spine_d_gain = d;
        config.spine_max_integral = max_integral;
      }break;
      
      case 1:
      {
        spine_driver_velocity_->getPidPtr()->getParams( p, i, d, max_integral );
        config.spine_p_gain = p;
        config.spine_i_gain = i;
        config.spine_d_gain = d;
        config.spine_max_integral = max_integral;
      }break;
      
      default: ROS_ERROR("bad joint control mode"); return;
    }   
  }
  if ( config.arm_control_mode != arm_control_mode )
  {
    double p, i, d, max_integral;
    arm_control_mode = config.arm_control_mode;
    switch ( arm_control_mode )
    {
      case 0: 
      {
        arm_driver_position_->getPidPtr()->getParams( p, i, d, max_integral );
        config.arm_p_gain = p;
        config.arm_i_gain = i;
        config.arm_d_gain = d;
        config.arm_max_integral = max_integral;
      }break;
      
      case 1:
      {
        arm_driver_velocity_->getPidPtr()->getParams( p, i, d, max_integral );
        config.arm_p_gain = p;
        config.arm_i_gain = i;
        config.arm_d_gain = d;
        config.arm_max_integral = max_integral;
      }break;
      
      default: ROS_ERROR("bad joint control mode"); return;
    }   
  }
  
  
  // convert enums from cfg file into the in the h files
  H_bridgeDriver::motor_drive_mode spine_motor_mode = (H_bridgeDriver::motor_drive_mode)config.spine_motor_mode;
  H_bridgeDriver::motor_drive_mode arm_motor_mode = (H_bridgeDriver::motor_drive_mode)config.arm_motor_mode;
  
  switch ( spine_motor_mode )
  {
    case H_bridgeDriver::DRIVE: spine_disabled_ = false; break;
    case H_bridgeDriver::FREE_RUNNING: spine_disabled_ = true; break;
    case H_bridgeDriver::BRAKE: spine_disabled_ = true; break;
    default: ROS_ERROR("bad motor mode");
  }
  switch ( arm_motor_mode )
  {
    case H_bridgeDriver::DRIVE: arm_disabled_ = false; break;
    case H_bridgeDriver::FREE_RUNNING: arm_disabled_ = true; break;
    case H_bridgeDriver::BRAKE: arm_disabled_ = true; break;
    default: ROS_ERROR("bad motor mode");
  }
  arm_driver_position_->setMode( arm_motor_mode );
  spine_driver_position_->setMode( spine_motor_mode );


  switch ( spine_control_mode )
  {
    case JointDriver::POSITION: 
    {
      spine_driver_position_->getPidPtr()->setParams( config.spine_p_gain, config.spine_i_gain, config.spine_d_gain, config.spine_max_integral );
    }break;
    
    case JointDriver::VELOCITY:
    {
      spine_driver_velocity_->getPidPtr()->setParams( config.spine_p_gain, config.spine_i_gain, config.spine_d_gain, config.spine_max_integral );
    }break;
  }
  switch ( arm_control_mode )
  {
    case JointDriver::POSITION: 
    {
      arm_driver_position_->getPidPtr()->setParams( config.arm_p_gain, config.arm_i_gain, config.arm_d_gain, config.arm_max_integral );
    }break;
    
    case JointDriver::VELOCITY:
    {
      arm_driver_velocity_->getPidPtr()->setParams( config.arm_p_gain, config.arm_i_gain, config.arm_d_gain, config.arm_max_integral );
    }break;
  }
  
  
  // if influence has changed, adjust the other influence value
  if ( config.spine_position_influence > spine_position_influence_ + 0.01 || config.spine_position_influence < spine_position_influence_ - 0.01 )
  {
    config.spine_velocity_influence = 1.0 - config.spine_position_influence;
    spine_position_influence_ = config.spine_position_influence;
    spine_velocity_influence_ = config.spine_velocity_influence;
  }
  if ( config.arm_position_influence > arm_position_influence_ + 0.01 || config.arm_position_influence < arm_position_influence_ - 0.01 )
  {
    config.arm_velocity_influence = 1.0 - config.arm_position_influence;
    arm_position_influence_ = config.arm_position_influence;
    arm_velocity_influence_ = config.arm_velocity_influence;
  }
  
  if ( config.spine_velocity_influence > spine_velocity_influence_ + 0.01 || config.spine_velocity_influence < spine_velocity_influence_ - 0.01 )
  {
    config.spine_position_influence = 1.0 - config.spine_velocity_influence;
    spine_position_influence_ = config.spine_position_influence;
    spine_velocity_influence_ = config.spine_velocity_influence;
  }
  if ( config.arm_velocity_influence > arm_velocity_influence_ + 0.01 || config.arm_velocity_influence < arm_velocity_influence_ - 0.01 )
  {
    config.arm_position_influence = 1.0 - config.arm_velocity_influence;
    arm_position_influence_ = config.arm_position_influence;
    arm_velocity_influence_ = config.arm_velocity_influence;
  }
  
  if( config.gripper_status == true )
  {
    gripper_target_ = GRIPPER_CLOSE;
    ROS_INFO("Closing gripper");
    pickup = false;
  }
  else
  {
    gripper_target_ = GRIPPER_OPEN;
    ROS_INFO("Opening gripper");
    pickup = true;
  }
  reached_gripper_limit_ = false;
  successful_grab_ = false;
  force_counter = 0;
  // set action time to infinity
  gripper_action_time_ = ros::Time((uint32_t)ULONG_MAX, 0);
  ROS_INFO("New control parameters set");
}



//
//  Callback function when a new height calibration is received
//
void heigth_cb( const std_msgs::Float32ConstPtr& height )
{
  if( calibrate_height_ )
  {
    height_spine_ = (-1.0) * height->data + CAMERA_TO_SPINE;  // height of camera + offset
    double temp = (height_spine_ - SPINE_MIN_HEIGHT) / (SPINE_LENGTH) * SPINE_ENCODER_TICKS_ON_STROKE;
    int32_t encoder_ticks = (int32_t) temp;
    ROS_INFO("camera calibrated at %fm, spine height set to %u encoder ticks", height_spine_, encoder_ticks);
    spine_driver_position_->getEncoderPtr()->setPosition(encoder_ticks);
  }
}



//
//  Callback function when a new Trajectroy is received from moveit
//
void trajectory_cb ( const moveit_msgs::MoveGroupActionResultConstPtr& desired )
{
  if( desired->status.status != 3 ) // trajectory execution not complete
  {
    ROS_WARN("Motion plan was not computed successfully and will therefore not be executed. Code %i", desired->status.status);
    return;
  }
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
  
  // set the time when the gripper is supposed to open or close
  gripper_action_time_ = start_time_trajectory_
                         // last trajectory time stamp
                       + trajectory_desired_->points[trajectory_desired_->points.size() - 1].time_from_start
                         // user selected offset TODO: get rid of define GRIPPER_TIME_DELAY
                       + ros::Duration(0, GRIPPER_TIME_DELAY * 1e6);
}



//
//  Main 
//
int main(int argc, char **argv) 
{
  //
  // initialize ROS
  //
  ros::init(argc, argv, "uplift_joint_manager");
  ros::NodeHandle nh;
  ros::NodeHandle nh_feedback("~");
  
  //
  // set up Arduino
  //
  std::string hw_id_arm;
  std::string hw_id_spine;  
  nh.param<std::string>( "hardware_id_arm", hw_id_arm, "/dev/ttyACM0" );
  nh.param<std::string>( "hardware_id_spine", hw_id_spine, "/dev/ttyACM1" );
  
  //
  // set up controlable joints and robot states
  //
  joint_names_.resize(2);
  joint_names_[SPINE] = "spine";
  joint_names_[ARM] = "arm";
  // stores current position and velocity readings for all controlable joints
  double current_position[joint_names_.size()];
  double current_velocity[joint_names_.size()];
  
  // set up robot joint state
  sensor_msgs::JointState robot_state;
  robot_state.name.resize(2);
  robot_state.position.resize(2);
  robot_state.velocity.resize(2);
  robot_state.name[SPINE] = joint_names_[SPINE];
  robot_state.name[ARM] = joint_names_[ARM];
  
  // create lookup table with the same size as the number of joints 
  lookup.resize( joint_names_.size() );


  //
  //  Create joint drivers for Spine
  //
  // create Arduino object and initialize
  NewArduinoInterface Arduino_Spine( hw_id_spine );
  if( Arduino_Spine.initialize() == false)
  {
    ROS_ERROR("Error initializing Arduino_Spine");
    return -1;
  }  

  // create joint driver for position control for the spine
  spine_driver_position_.reset( new JointDriver(
          &Arduino_Spine,                                                                                  // hardware interface
          SPINE_PWM_PIN, SPINE_PWM_FREQUENCY, SPINE_DIRECTION_CONTROL1_PIN, SPINE_DIRECTION_CONTROL2_PIN,  // Motor pins and settings
          SPINE_ENCODER1_PIN, SPINE_ENCODER2_PIN, SPINE_ENCODER_TICKS_ON_STROKE / 4.0, CREATE_NEW_ENCODER, // Encoder pins and settings; ticks=4*marks
          SPINE_LENGTH, JointDriver::POSITION)); 	                                                         // Joint settings
  // initalize joint and connected hardware
  spine_driver_position_->initialize(); 
  // retrieve encoder id to use in next joint driver
  spine_encoder_id_ = spine_driver_position_->getEncoderPtr()->getEncoderID();
  
  // creat joint driver for velocity control for the spine
  spine_driver_velocity_.reset( new JointDriver( 
          &Arduino_Spine,                                                                                  // hardware interface
          SPINE_PWM_PIN, SPINE_PWM_FREQUENCY, SPINE_DIRECTION_CONTROL1_PIN, SPINE_DIRECTION_CONTROL2_PIN,  // Motor pins and settings
          SPINE_ENCODER1_PIN, SPINE_ENCODER2_PIN, SPINE_ENCODER_TICKS_ON_STROKE / 4.0, spine_encoder_id_,  // Encoder pins and settings; ticks=4*marks
          SPINE_LENGTH, JointDriver::VELOCITY)); 	                                                         // Joint setting
  // initalize joint and connected hardware
  spine_driver_velocity_->initialize();

	
	//
  // create joint drivers for arm
	//
	// create Arduino object and initialize
	NewArduinoInterface Arduino_Arm( hw_id_arm );
  if( Arduino_Arm.initialize() == false)
  {
    ROS_ERROR("Error initializing Arduino_Arm");
    return -1;
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
  arm_driver_velocity_->initialize(); 
	  
	  
  //
  // create joint driver for gripper
  //
  gripper_driver_position_.reset( new JointDriver(
            &Arduino_Arm,                                                                                           // hardware interface
            GRIPPER_PWM_PIN, GRIPPER_PWM_FREQUENCY, GRIPPER_DIRECTION_CONTROL1_PIN, GRIPPER_DIRECTION_CONTROL2_PIN, // Motor pins and settings
            GRIPPER_ADC_PIN, ADC_REFERENCE_VOLTAGE, GRIPPER_LENGTH,                                                 // ADC settings
            JointDriver::POSITION)); 	                                                                              // Joint control mode
  //initialize joint and connected hardware
  gripper_driver_position_->initialize();
  gripper_driver_position_->getPidPtr()->setParams(500,0,0,0);
  
  
  
	ros::Subscriber sub = nh.subscribe( "/move_group/result", 1, trajectory_cb);
	ros::Subscriber sub_height = nh.subscribe( "/height_detector/height", 1, heigth_cb );
	
	ros::Publisher pub = nh.advertise<sensor_msgs::JointState> ("/joint_states", 1);

  ros::Publisher pub_position_target_spine = nh_feedback.advertise<std_msgs::Float64> ("feedback/spine/position/target", 5);
  ros::Publisher pub_velocity_target_spine = nh_feedback.advertise<std_msgs::Float64> ("feedback/spine/velocity/target", 5);

	ros::Publisher pub_position_target_arm   = nh_feedback.advertise<std_msgs::Float64> ("feedback/arm/position/target", 5);
	ros::Publisher pub_velocity_target_arm   = nh_feedback.advertise<std_msgs::Float64> ("feedback/arm/velocity/target", 5);
	
	//
	// dynamic reconfigure settings
	//
  dynamic_reconfigure::Server<uplift_joint_manager::JointConfig> server;
  dynamic_reconfigure::Server<uplift_joint_manager::JointConfig>::CallbackType config;
  config = boost::bind(&dynamic_cb, _1, _2);
  server.setCallback(config);
  
  ROS_INFO("Running control loop");
  ros::Rate loop_rate_Hz(100);
  ros::Duration duration_between_points;


  while ( ros::ok() )
  {
    // record time of measurements
    ros::Time now = ros::Time::now();
    
    //
    // measure current angles, positions and velocities of the robot and publish robot state
    //
    current_velocity[SPINE] = spine_driver_velocity_->getState();
    current_position[SPINE] = spine_driver_position_->getState();
    robot_state.position[SPINE] = current_position[SPINE];
    robot_state.velocity[SPINE] = current_velocity[SPINE];

    current_velocity[ARM] = arm_driver_velocity_->getState();
    current_position[ARM] = arm_driver_position_->getState();
    robot_state.position[ARM] = (-1) * current_position[ARM];
    robot_state.velocity[ARM] = (-1) * current_velocity[ARM];
    
    robot_state.header.stamp = now;
    
    pub.publish( robot_state );
    
    
    // check if any trajectory has been received yet (point_counter_ is initialized with -1)
    if( point_counter_ < 0 )
    {
      ROS_DEBUG("Waiting for first trajectory to be published");
      ros::spinOnce();
      loop_rate_Hz.sleep();
      continue;
    }
    
    // check if trajectory is not completed yet
    if( (size_t)point_counter_ < trajectory_desired_->points.size() - 1)
    {
      // read time stamp of current trajectory point
      ros::Time target_time = start_time_trajectory_ + trajectory_desired_->points[point_counter_].time_from_start;
      // find next trajectory point which has a time stamp not in the past
      // also make sure to not increase the counter above the trajectroy size
      while( ( now > target_time )  && ( (size_t)point_counter_ < trajectory_desired_->points.size() - 1) )
      {
        ++point_counter_;
        // set target time to new trajectory point time stamp
        target_time = start_time_trajectory_ + trajectory_desired_->points[point_counter_].time_from_start;
      }
    }
    
    
    // check if gripper is supposed to open or close
    if( now > gripper_action_time_ )
    {
      if( pickup )
      {
        gripper_target_ = GRIPPER_CLOSE;
        ROS_INFO("Closing gripper");
      }
      else
      {
        gripper_target_ = GRIPPER_OPEN;
        ROS_INFO("Opening gripper");
      }
      pickup = !pickup;
      reached_gripper_limit_ = false;
      successful_grab_ = false;
      force_counter = 0;
      // set action time to infinity
      gripper_action_time_ = ros::Time((uint32_t)ULONG_MAX, 0);
    }

    
    //
    // SPINE control
    // extract trajectory target information and apply new control to spine motor
    //    
    double target_velocity_spine = trajectory_desired_->points[point_counter_].velocities[lookup[SPINE]];
    double output_velocity_control_spine = spine_driver_velocity_->compute( current_velocity[SPINE], target_velocity_spine ); 
                  
    ROS_DEBUG("spine velocity: current:%f  target:%f  output:%f", current_velocity[SPINE], target_velocity_spine, output_velocity_control_spine );
    
    double target_position_spine = trajectory_desired_->points[point_counter_].positions[lookup[SPINE]];
    double output_position_control_spine = spine_driver_position_->compute( current_position[SPINE], target_position_spine );
    
    ROS_DEBUG("spine position: current:%f  target:%f  output:%f", current_position[SPINE], target_position_spine, output_position_control_spine );
    
    // weigh position and velocity output and apply
    spine_driver_velocity_->applyOutput( (output_velocity_control_spine * spine_velocity_influence_) + (output_position_control_spine * spine_position_influence_) );
    
    // publish feedback information
    std_msgs::Float64 temp;
    temp.data = target_position_spine;
    pub_position_target_spine.publish( temp );
    temp.data = target_velocity_spine;
    pub_velocity_target_spine.publish( temp );

    
    //
    // ARM control
    // extract trajectory target information and apply new control to arm motor
    //
    double target_velocity_arm = (-1.0) * trajectory_desired_->points[point_counter_].velocities[lookup[ARM]];  // adjust rotation direction
    double output_velocity_control_arm = arm_driver_velocity_->compute( current_velocity[ARM], target_velocity_arm ); 
                  
    ROS_DEBUG("arm velocity: current:%f  target:%f  output:%f", current_velocity[ARM], target_velocity_arm, output_velocity_control_arm );
    
    double target_position_arm = (-1.0) * trajectory_desired_->points[point_counter_].positions[lookup[ARM]];  // adjust rotation direction
    double output_position_control_arm = arm_driver_position_->compute( current_position[ARM], target_position_arm );
    
    ROS_DEBUG("arm position: current:%f  target:%f  output:%f", current_position[ARM], target_position_arm, output_position_control_arm );
    
    // weigh position and velocity output and apply
    arm_driver_velocity_->applyOutput( (output_velocity_control_arm * arm_velocity_influence_) + (output_position_control_arm * arm_position_influence_) );
    
    
    //
    //  Gripper control
    //
    double gripper_position = gripper_driver_position_->getState( );
    double gripper_delta = gripper_last_position_ - gripper_position;
    double gripper_control  = gripper_driver_position_->compute( gripper_position, gripper_target_ );
    gripper_last_position_ = gripper_position;
    
    // check if endpoint has been reached
    if( !pickup )  // gripper's target is CLOSE
    {
      if( gripper_position > GRIPPER_CLOSE - GRIPPER_DELTA )
      {
        ROS_DEBUG("gripper closed");
        reached_gripper_limit_ = true;
        successful_grab_ = false;
        gripper_driver_position_->applyOutput(0.0);
      }
    }
    else  // gripper's target is open
    {
      if( gripper_position < GRIPPER_OPEN + GRIPPER_DELTA )
      {
        ROS_DEBUG("gripper open");
        reached_gripper_limit_ = true;
        successful_grab_ = false;
        gripper_driver_position_->applyOutput(0.0);
      }
    }
    
    // check if maximum motor torque is used but motor does not move -> object grabbed
    if( gripper_control >= 1.0 && !pickup)
    {
      if( (-1) * gripper_delta < GRIPPER_DELTA )
      {
        ROS_DEBUG("gripper delta %f", gripper_delta);
        if( ++force_counter > GRIPPER_FORCE_LIMIT )
        {
          ROS_DEBUG("gripper limit reached");
          reached_gripper_limit_ = true;
          gripper_driver_position_->applyOutput(0.0);
          successful_grab_ = true;
        }
      }
      else
      {
        force_counter = 0;
      }
    }
    
    // apply new PWM duty cycle
    if( !reached_gripper_limit_ )
    {
       gripper_driver_position_->applyOutput( gripper_control );
       ROS_DEBUG("gripper target: %f   real: %f", gripper_target_, gripper_position);
    }
    
    
    // publish feedback information
    std_msgs::Float64 temp_arm;
    temp_arm.data = target_position_arm;
    pub_position_target_arm.publish( temp_arm );
    temp_arm.data = target_velocity_arm;
    pub_velocity_target_arm.publish( temp_arm );
    
    ros::spinOnce();
    loop_rate_Hz.sleep();
  }    
  
  return 0;
}
