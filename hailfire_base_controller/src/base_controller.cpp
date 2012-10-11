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
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <tf/transform_broadcaster.h>
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

struct PolarCounts
{
  double angular; // corrected angular ticks
  double linear;  // corrected linear ticks
};

struct OdomPosition
{
  double x; // m
  double y; // m
  double th; // rad
};

struct PolarSpeed
{
  double angular; // rad/s
  double linear;  // m/s
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

  /**
   * @brief Main loop
   */
  void spin();

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
   * @brief Twist message handler.
   *
   * This method is called by ROS when a message is published on the cmd_vel
   * topic. It transforms the received geometry_msgs/Twist message in a call to
   * the driveMotorsWithSpeedAndAcceleration method of the RoboClaw instance.
   *
   */
  void twistMsgHandler(const geometry_msgs::Twist::ConstPtr &cmd);

  /**
   * @brief Stops both motors immediately, regardless of their current speed.
   *
   * This method is called by ROS when a message is published on the estop
   * topic. It stops both motors with a call to the driveMotor method of the
   * RoboClaw instance.
   */
  void hardStop(const std_msgs::Empty::ConstPtr &empty);

  /**
   * @brief Reads the encoder counts and updates the position_ member.
   */
  void refreshPosition();

  /**
   * @brief Reads the encoder speeds and updates the speed_ member.
   */
  void refreshSpeed();

  /**
   * @brief Publish odometry transform over tf and odometry message on /odom topic.
   */
  void publishOdometry();

  ros::NodeHandle nh_;              /**< The ROS node handle */
  ros::Subscriber twist_sub_;       /**< The ROS subscriber to velocity command messages */
  ros::Subscriber estop_sub_;       /**< The ROS subscriber to emergency stop messages */
  ros::Publisher odom_pub_;         /**< The ROS publisher of Odometry messages */
  tf::TransformBroadcaster odom_bc_; /**< The TF broadcaster of Odometry messages */

  hailfire_robo_claw::RoboClaw *robo_claw_;     /**< The RoboClaw instance */

  MotorParams left_motor_;          /**< Params of the left motor */
  MotorParams right_motor_;         /**< Params of the right motor */

  int motors_max_vel_qpps_;         /**< Max allowed velocity in ticks/s */
  int motors_max_acc_qpps2_;        /**< Max allowed acceleration in ticks/s^2 */

  EncoderParams left_encoder_;      /**< Params of the left encoder */
  EncoderParams right_encoder_;     /**< Params of the right encoder */

  double encoders_ticks_per_m_;     /**< Number of encoder ticks per m */
  double encoders_track_m_;         /**< Length between encoders in m */

  PolarCounts prev_counts_;         /**< Previous polar odometers value */
  OdomPosition position_;           /**< Position computed from odometers */
  PolarSpeed speed_;                /**< Speed from odometers */
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
  twist_sub_ = nh_.subscribe("cmd_vel", 1, &BaseController::twistMsgHandler, this);

  // Subscribe to estop topic
  ROS_INFO("Subscribing to estop topic");
  estop_sub_ = nh_.subscribe("estop", 1, &BaseController::hardStop, this);

  // Publish to /odom topic
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 50);

  ROS_INFO("Ready to handle velocity commands");
}

BaseController::~BaseController()
{
  const std_msgs::Empty::ConstPtr empty;
  hardStop(empty);

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

  ROS_DEBUG("Commanding m1: %d, m2: %d", speed_m1, speed_m2);

  // Send to RoboClaw
  if (robo_claw_)
  {
    robo_claw_->driveMotorsWithSpeedAndAcceleration(speed_m1, speed_m2, motors_max_acc_qpps2_);
  }
}

void BaseController::hardStop(const std_msgs::Empty::ConstPtr &empty)
{
  ROS_DEBUG("Commanding a hard stop");

  if (robo_claw_)
  {
    robo_claw_->driveMotor(left_motor_.id, 0);
    robo_claw_->driveMotor(right_motor_.id, 0);
  }
}

void BaseController::refreshPosition()
{
  // get polar odometers counts
  PolarCounts odometers;
  if (robo_claw_) {
    // bit 0: 1 if underflow occurred
    // bit 1: current direction 0=forward 1=backward
    // bit 2: 1 if overflow occurred
    // bit 7: 1 if encoder is OK
    // FIXME: could publish these to /diagnostics
    uint8_t l_status;
    uint8_t r_status;

    // Read encoders counts (ticks)
    int32_t l_count_raw;
    int32_t r_count_raw;
    robo_claw_->readEncoderCount(left_encoder_.id, &l_count_raw, &l_status);
    robo_claw_->readEncoderCount(right_encoder_.id, &r_count_raw, &r_status);

    // Apply gains
    double l_count_g = l_count_raw * left_encoder_.gain;
    double r_count_g = r_count_raw * right_encoder_.gain;

    // Convert to polar counts
    odometers.angular = (r_count_g - l_count_g) / 2;
    odometers.linear = (r_count_g + l_count_g) / 2;
  }
  // Not running on real hardware
  else {
    odometers.angular = 0;
    odometers.linear = 0;
  }

  // compute differences since last measure
  PolarCounts delta;
  delta.angular = odometers.angular - prev_counts_.angular;
  delta.linear = odometers.linear - prev_counts_.linear;

  // keep the new measures for later
  prev_counts_.angular = odometers.angular;
  prev_counts_.linear = odometers.linear;

  // update position
  if (delta.angular == 0) {
    // we are going really straight (very unlikely)
    position_.x += cos(position_.th) * delta.linear / encoders_ticks_per_m_;
    position_.y += sin(position_.th) * delta.linear / encoders_ticks_per_m_;
  } else {
    // approximate the trajectory to an arc of a circle: compute its radius and angle
    int arc_r = delta.linear * encoders_track_m_ / (delta.angular * 2);
    int arc_a = 2 * delta.angular / (encoders_ticks_per_m_ * encoders_track_m_);

    position_.x += arc_r * (-sin(position_.th) + sin(position_.th + arc_a));
    position_.y += arc_r * (cos(position_.th) - cos(position_.th + arc_a));
    position_.th += arc_a;
  }
}

void BaseController::refreshSpeed()
{
  if (robo_claw_) {
    // Read encoders speeds (ticks/s)
    int32_t l_speed_raw;
    int32_t r_speed_raw;
    robo_claw_->readEncoderSpeed(left_encoder_.id, &l_speed_raw);
    robo_claw_->readEncoderSpeed(right_encoder_.id, &r_speed_raw);

    // Apply gains
    double l_speed_g = l_speed_raw * left_encoder_.gain;
    double r_speed_g = r_speed_raw * right_encoder_.gain;

    // Convert to polar speeds
    double a_speed = (r_speed_g - l_speed_g) / 2;
    double l_speed = (r_speed_g + l_speed_g) / 2;

    // Convert to m and rad
    speed_.angular = 2 * a_speed / (encoders_ticks_per_m_ * encoders_track_m_);
    speed_.linear = l_speed / encoders_ticks_per_m_;
  }
  // Not running on real hardware
  else {
    speed_.angular = 0;
    speed_.linear = 0;
  }
}

void BaseController::publishOdometry()
{
  refreshPosition();
  refreshSpeed();

  ros::Time current_time = ros::Time::now();

  // since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(position_.th);

  // first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = position_.x;
  odom_trans.transform.translation.y = position_.y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  // send the transform
  odom_bc_.sendTransform(odom_trans);

  // next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

  //  set the position
  odom.pose.pose.position.x = position_.x;
  odom.pose.pose.position.y = position_.y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  // set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = speed_.linear;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = speed_.angular;

  // publish the message
  odom_pub_.publish(odom);
}

void BaseController::spin()
{
  ros::Rate r(50.0); // Hz
  while (nh_.ok()) {
    ros::spinOnce(); // check for incoming messages
    publishOdometry();
    r.sleep();
  }
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "base_controller");

  hailfire_base_controller::BaseController base_controller;
  base_controller.spin();

  return 0;
}
