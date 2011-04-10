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
#include <joy/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>

namespace hailfire_teleop_joy
{

/**
 * @class TeleopJoy
 * @brief Transforms Joy messages in Twist messages.
 */
class TeleopJoy
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
   * ~teleop_joy/axis_linear    Index of axis to use for linear speed
   * ~teleop_joy/axis_angular   Index of axis to use for angular speed
   * ~teleop_joy/scale_linear   Scale coefficient for linear speed
   * ~teleop_joy/scale_angular  Scale coefficient for angular speed
   * ~teleop_joy/button_estop   Index of button to use for emergency stop
   */
  TeleopJoy();

private:

  /**
   * @brief Message handler.
   *
   * This method is called by ROS when a message is published on the
   * joy topic. It transforms the received joy/Joy message in a new
   * geometry_msgs/Twist message.
   *
   */
  void joyCallback(const joy::Joy::ConstPtr &joy);

  ros::NodeHandle nh_;          /**< The ROS node handle */
  ros::Publisher twist_pub_;    /**< The ROS publisher of velocity command messages */
  ros::Publisher estop_pub_;    /**< The ROS publisher of emergency stop messages */
  ros::Subscriber joy_sub_;     /**< The ROS subscriber to Joy messages */

  int linear_, angular_;        /**< Index of axis to use for linear and angular speeds */
  double l_scale_, a_scale_;    /**< Scale coefficient for linear and angular speeds */
  int estop_;                   /**< Index of button to use for emergency stop */
};

TeleopJoy::TeleopJoy():
  linear_(1),
  angular_(2),
  l_scale_(1.0),
  a_scale_(1.0),
  estop_(1)
{
  ros::NodeHandle nh_param("~");

  nh_param.param("axis_linear", linear_, linear_);
  nh_param.param("axis_angular", angular_, angular_);
  nh_param.param("scale_angular", a_scale_, a_scale_);
  nh_param.param("scale_linear", l_scale_, l_scale_);
  nh_param.param("button_estop", estop_, estop_);

  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  estop_pub_ = nh_.advertise<std_msgs::Empty>("estop", 10);
  joy_sub_ = nh_.subscribe("joy", 10, &TeleopJoy::joyCallback, this);
}

void TeleopJoy::joyCallback(const joy::Joy::ConstPtr &joy)
{
  geometry_msgs::Twist cmd;
  cmd.linear.x = l_scale_ * joy->axes[linear_];
  cmd.linear.y = 0;
  cmd.linear.z = 0;
  cmd.angular.x = 0;
  cmd.angular.y = 0;
  cmd.angular.z = a_scale_ * joy->axes[angular_];
  twist_pub_.publish(cmd);

  if (joy->buttons[estop_] == 1)
  {
    ROS_INFO("Asking for emergency stop");
    std_msgs::Empty empty;
    estop_pub_.publish(empty);
  }
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teleop_joy");
  hailfire_teleop_joy::TeleopJoy teleop_joy;
  ros::spin();
  return 0;
}
