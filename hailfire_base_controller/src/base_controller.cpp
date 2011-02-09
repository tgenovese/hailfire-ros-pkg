/*
 * Copyright (c) 2011, Thierry Genovese.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * The name of the author may not be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "hailfire_robo_claw/robo_claw.h"

namespace hailfire_base_controller
{

struct MotorParams
{
  int id;
  struct
  {
    int p;
    int i;
    int d;
    int qpps;
  } gains;
};

struct EncoderParams
{
  int id;
  double gain;
};

/**
 * @class BaseController
 * @brief Message subscriber node class listening to velocity commands.
 *
 * This class subscribes to velocity command messages and uses the RoboClaw
 * C++ API (hailfire_robo_claw package) to drive the left and right motors.
 */
class BaseController
{
public:

  /**
   * @brief Constructor.
   *
   * Creates and configures the node, then subscribes to cmd_vel messages.
   *
   * Upon creation, the following parameters are fetched from the parameter
   * server:
   *
   * To create the RoboClaw instance:
   * ~robo_claw/dev_name        Name of the serial device file
   * ~robo_claw/baud_rate       Baud rate to use
   * ~robo_claw/address         Address of the RoboClaw on the bus
   *
   * For the left motor:
   * ~left_motor/id             Identifier (1 or 2) of the left motor
   * ~left_motor/gains/p        Proportional gain of the left PID
   * ~left_motor/gains/i        Integral gain of the left PID
   * ~left_motor/gains/d        Derivate gain of the left PID
   * ~left_motor/gains/qpps     Speed of the motor at 100% power in ticks/s
   *
   * For the right motor:
   * ~right_motor/id            Identifier (1 or 2) of the right motor
   * ~right_motor/gains/p       Proportional gain of the right PID
   * ~right_motor/gains/i       Integral gain of the right PID
   * ~right_motor/gains/d       Derivate gain of the right PID
   * ~right_motor/gains/qpps    Speed of the motor at 100% power in ticks/s
   *
   * For both motors:
   * ~motors_max_vel_qpps       Maximum allowed velocity in ticks/s
   * ~motors_max_acc_qpps2      Maximum allowed acceleration in ticks/s^2
   *
   * For the left encoder:
   * ~left_encoder/id           Identifier (1 or 2) of the left encoder
   * ~left_encoder/gain         Corrective gain for the left encoder
   *
   * For the right encoder:
   * ~right_encoder/id          Identifier (1 or 2) of the right encoder
   * ~right_encoder/gain        Corrective gain for the right encoder
   *
   * For both encoders:
   * ~encoders_ticks_per_m      Number of encoder ticks per m
   * ~encoders_track_m          Length between encoders in m
   */
  BaseController();

  /**
   * @brief Destructor: cleans up
   */
  ~BaseController();

private:

  /**
   * @brief Create the RoboClaw instance from ROS parameters.
   */
  void createRoboClawFromParams(ros::NodeHandle nh_param);

  /**
   * @brief Configure a motor from ROS parameters.
   */
  void configureMotorFromParams(ros::NodeHandle nh_param, MotorParams *out);

  /**
   * @brief Configure an encoder from ROS parameters.
   */
  void configureEncoderFromParams(ros::NodeHandle nh_param, EncoderParams *out);

  /**
   * @brief Message handler.
   *
   * This method is called by ROS when a message is published on the cmd_vel
   * topic. It transforms the received geometry_msgs/Twist message in a call to
   * the driveMotorsWithSpeedAndAcceleration method of the RoboClaw instance.
   *
   */
  void twistMsgHandler(const geometry_msgs::Twist::ConstPtr &cmd);

  ros::NodeHandle nh_;              /**< The ROS node handle */
  ros::Subscriber msg_sub_;         /**< The ROS subscriber to Twist messages */

  hailfire_robo_claw::RoboClaw *robo_claw_;     /**< The RoboClaw instance */

  MotorParams left_motor_;          /**< Params of the left motor */
  MotorParams right_motor_;         /**< Params of the right motor */

  int motors_max_vel_qpps_;         /**< Max allowed velocity in ticks/s */
  int motors_max_acc_qpps2_;        /**< Max allowed acceleration in ticks/s^2 */

  EncoderParams left_encoder_;      /**< Params of the left encoder */
  EncoderParams right_encoder_;     /**< Params of the right encoder */

  double encoders_ticks_per_m_;     /**< Number of encoder ticks per m */
  double encoders_track_m_;         /**< Length between encoders in m */
};

BaseController::BaseController(void)
{
  ros::NodeHandle nh_robo_claw_param("~robo_claw");
  createRoboClawFromParams(nh_robo_claw_param);

  ros::NodeHandle nh_left_motor_param("~left_motor");
  configureMotorFromParams(nh_left_motor_param, &left_motor_);

  ros::NodeHandle nh_right_motor_param("~right_motor");
  configureMotorFromParams(nh_right_motor_param, &right_motor_);

  ros::NodeHandle nh_left_encoder_param("~left_encoder");
  configureEncoderFromParams(nh_left_encoder_param, &left_encoder_);

  ros::NodeHandle nh_right_encoder_param("~right_encoder");
  configureEncoderFromParams(nh_right_encoder_param, &right_encoder_);

  // Get other params
  ros::NodeHandle nh_param("~");

  if (!nh_param.getParam("motors_max_vel_qpps", motors_max_vel_qpps_) ||
      !nh_param.getParam("motors_max_acc_qpps2", motors_max_acc_qpps2_))
  {
    ROS_ERROR("Failed to retrieve all motors parameters.");
    exit(1);
  }

  if (!nh_param.getParam("encoders_ticks_per_m", encoders_ticks_per_m_) ||
      !nh_param.getParam("encoders_track_m", encoders_track_m_))
  {
    ROS_ERROR("Failed to retrieve all encoders parameters.");
    exit(1);
  }

  // Subscribe to cmd_vel topic
  ROS_INFO("Subscribing to cmd_vel topic");
  msg_sub_ = nh_.subscribe("cmd_vel", 1, &BaseController::twistMsgHandler, this);

  ROS_INFO("Ready to handle velocity commands");
}

BaseController::~BaseController()
{
  if (robo_claw_)
  {
    delete robo_claw_;
  }
}

void BaseController::createRoboClawFromParams(ros::NodeHandle nh_param)
{
  // Should we proceed with the creation of the RoboClaw instance?
  bool inhibit;
  nh_param.param("inhibit", inhibit, false);
  if (inhibit)
  {
    ROS_INFO("~robo_claw/inhibit set: RoboClaw instance will not be created");
    robo_claw_ = NULL;
    return;
  }

  // Get params
  std::string dev_name;
  int baud_rate_int;
  int address;

  if (!nh_param.getParam("dev_name", dev_name) ||
      !nh_param.getParam("baud_rate", baud_rate_int) ||
      !nh_param.getParam("address", address))
  {
    ROS_ERROR("Failed to retrieve all RoboClaw parameters.");
    exit(1);
  }

  // Validate baud rate and get actual constant
  speed_t baud_rate;
  switch (baud_rate_int)
  {
    case 2400:
      baud_rate = B2400;
      break;
    case 9600:
      baud_rate = B9600;
      break;
    case 19200:
      baud_rate = B19200;
      break;
    case 38400:
      baud_rate = B38400;
      break;
    default:
      ROS_ERROR("Unsupported baud rate: %u", baud_rate_int);
      exit(1);
  }

  robo_claw_ = new hailfire_robo_claw::RoboClaw(dev_name.c_str(), baud_rate, address);
}

void BaseController::configureMotorFromParams(ros::NodeHandle nh_param, MotorParams *out)
{
  // Get params
  if (!nh_param.getParam("id", out->id) ||
      !nh_param.getParam("gains/p", out->gains.p) ||
      !nh_param.getParam("gains/i", out->gains.i) ||
      !nh_param.getParam("gains/d", out->gains.d) ||
      !nh_param.getParam("gains/qpps", out->gains.qpps))
  {
    ROS_ERROR("Failed to retrieve all motor parameters.");
    exit(1);
  }

  // Apply PID settings
  if (robo_claw_)
  {
    robo_claw_->setPIDConstants(out->id, out->gains.p, out->gains.i, out->gains.d, out->gains.qpps);
  }
}

void BaseController::configureEncoderFromParams(ros::NodeHandle nh_param, EncoderParams *out)
{
  // Get params
  if (!nh_param.getParam("id", out->id) ||
      !nh_param.getParam("gain", out->gain))
  {
    ROS_ERROR("Failed to retrieve all encoder parameters.");
    exit(1);
  }
}

void BaseController::twistMsgHandler(const geometry_msgs::Twist::ConstPtr &cmd)
{
  // Converts rad/s and m/s to ticks/s
  int angular_consign = encoders_ticks_per_m_ * encoders_track_m_ * cmd->angular.z / 2;
  int linear_consign  = encoders_ticks_per_m_ * cmd->linear.x;

  // Compute left/right consigns from angular/linear consigns
  int left_consign  = linear_consign - angular_consign;
  int right_consign = linear_consign + angular_consign;

  // Compute motor1/motor2 consigns from left/right consigns
  int speed_m1 = (left_motor_.id == 1 ? left_consign : right_consign);
  int speed_m2 = (right_motor_.id == 2 ? right_consign : left_consign);

  // Crop speeds
  speed_m1 = (speed_m1 > +motors_max_vel_qpps_ ? +motors_max_vel_qpps_ : speed_m1);
  speed_m1 = (speed_m1 < -motors_max_vel_qpps_ ? -motors_max_vel_qpps_ : speed_m1);
  speed_m2 = (speed_m2 > +motors_max_vel_qpps_ ? +motors_max_vel_qpps_ : speed_m2);
  speed_m2 = (speed_m2 < -motors_max_vel_qpps_ ? -motors_max_vel_qpps_ : speed_m2);

  // Send to RoboClaw
  if (robo_claw_)
  {
    robo_claw_->driveMotorsWithSpeedAndAcceleration(speed_m1, speed_m2, motors_max_acc_qpps2_);
  }
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "base_controller");

  hailfire_base_controller::BaseController base_controller;

  ros::spin();

  return 0;
}
