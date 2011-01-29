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
#include <hailfire_fpga_proxy/FPGAKeyValue.h>
#include <hailfire_fpga_proxy/FPGATransfer.h>

namespace hailfire_base_controller
{

/**
 * @class BaseController
 * @brief Message subscriber node class listening to velocity commands.
 *
 * This class subscribes to velocity command messages and converts them to
 * calls to the fpga_proxy service.
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
   * ~odometer_imp_per_m
   * ~odometer_track_m
   *
   */
  BaseController();

  /**
   * @brief Configures the angle control system in the FPGA.
   *
   * The following parameters are fetched from the parameter server and are
   * used to configure the angle control systems implemented in the FPGA:
   *
   * ~angle_cs/max_acceleration
   * ~angle_cs/max_deceleration
   * ~angle_cs/correct_filter_gain_p
   * ~angle_cs/correct_filter_gain_i
   * ~angle_cs/correct_filter_gain_d
   * ~angle_cs/correct_filter_out_shift
   *
   * The max_acceleration and max_deceleration params are converted from real-
   * world units to odometer ticks before being sent to the FPGA.
   *
   */
  void configureAngleControlSystem();

  /**
   * @brief Configures the distance control system in the FPGA.
   *
   * The following parameters are fetched from the parameter server and are
   * used to configure the distance control systems implemented in the FPGA:
   *
   * ~distance_cs/max_acceleration
   * ~distance_cs/max_deceleration
   * ~distance_cs/correct_filter_gain_p
   * ~distance_cs/correct_filter_gain_i
   * ~distance_cs/correct_filter_gain_d
   * ~distance_cs/correct_filter_out_shift
   *
   * The max_acceleration and max_deceleration params are converted from real-
   * world units to odometer ticks before being sent to the FPGA.
   *
   */
  void configureDistanceControlSystem();

private:

  /**
   * @brief Message handler.
   *
   * This method is called by ROS when a message is published on the cmd_vel
   * topic. It transforms the received geometry_msgs/Twist message in an
   * fpga_proxy service call.
   *
   * The angular and linear velocity commands are converted from real-world
   * units to odometer ticks before being sent to the FPGA.
   *
   */
  void twistMsgHandler(const geometry_msgs::Twist::ConstPtr &cmd);

  /**
   * @brief Converts from real-world angular units to odometer ticks.
   */
  int angularOdometerTicksFromRealWorldUnits(double const &val)
  {
    return odometer_imp_per_m_ * odometer_track_m_ * val / 2;
  }

  /**
   * @brief Converts from real-world linear units to odometer ticks.
   */
  int linearOdometerTicksFromRealWorldUnits(double const &val)
  {
    return odometer_imp_per_m_ * val;
  }

  ros::NodeHandle nh_;              /**< The ROS node handle */
  ros::ServiceClient srv_client_;   /**< The ROS service client for fpga_proxy service */
  ros::Subscriber msg_sub_;         /**< The ROS subscriber to Twist messages */

  double odometer_imp_per_m_;       /**< Number of odometer ticks per robot meter */
  double odometer_track_m_;         /**< Length between odometers in meters */
};

BaseController::BaseController(void)
{
  // Get odometer params
  ros::NodeHandle nh_param("~");
  if (nh_param.getParam("odometer_imp_per_m", odometer_imp_per_m_) &&
      nh_param.getParam("odometer_track_m", odometer_track_m_))
  {
    ROS_INFO("Will use odometer_imp_per_m: %f", odometer_imp_per_m_);
    ROS_INFO("Will set odometer_track_m: %f", odometer_track_m_);
  }
  else
  {
    ROS_ERROR("Failed to retrieve all odometer parameters.");
    exit(1);
  }

  // Creates client for fpga_proxy service. Waits until that service is available.
  ROS_INFO("Connecting to fpga_proxy service");
  srv_client_ = nh_.serviceClient<hailfire_fpga_proxy::FPGATransfer>("fpga_proxy");
  srv_client_.waitForExistence();

  // Subscribe to cmd_vel topic (we still have the time to configure the angle
  // and distance control systems since ros::spin() hasn't been called yet).
  ROS_INFO("Subscribing to cmd_vel topic");
  msg_sub_ = nh_.subscribe("cmd_vel", 1, &BaseController::twistMsgHandler, this);

  ROS_INFO("Ready to handle velocity commands");
}

void BaseController::configureAngleControlSystem()
{
  ros::NodeHandle nh_param("~angle_cs");

  double max_acceleration_d;
  double max_deceleration_d;
  int max_acceleration;
  int max_deceleration;
  int correct_filter_gain_p;
  int correct_filter_gain_i;
  int correct_filter_gain_d;
  int correct_filter_out_shift;

  if (nh_param.getParam("max_acceleration", max_acceleration_d) &&
      nh_param.getParam("max_deceleration", max_deceleration_d) &&
      nh_param.getParam("correct_filter_gain_p", correct_filter_gain_p) &&
      nh_param.getParam("correct_filter_gain_i", correct_filter_gain_i) &&
      nh_param.getParam("correct_filter_gain_d", correct_filter_gain_d) &&
      nh_param.getParam("correct_filter_out_shift", correct_filter_out_shift))
  {
    max_acceleration = angularOdometerTicksFromRealWorldUnits(max_acceleration_d);
    max_deceleration = angularOdometerTicksFromRealWorldUnits(max_deceleration_d);

    ROS_INFO("Will set angle max_acceleration: %u", max_acceleration);
    ROS_INFO("Will set angle max_deceleration: %u", max_deceleration);
    ROS_INFO("Will set angle correct_filter_gain_p: %u", correct_filter_gain_p);
    ROS_INFO("Will set angle correct_filter_gain_i: %u", correct_filter_gain_i);
    ROS_INFO("Will set angle correct_filter_gain_d: %u", correct_filter_gain_d);
    ROS_INFO("Will set angle correct_filter_out_shift: %u", correct_filter_out_shift);
  }
  else
  {
    ROS_ERROR("Failed to retrieve all angle control system parameters.");
    exit(1);
  }

  hailfire_fpga_proxy::FPGATransfer transfer;
  hailfire_fpga_proxy::FPGAKeyValue pair;

  pair.key = hailfire_fpga_proxy::FPGAKeyValue::ANGLE_MAX_ACCELERATION;
  pair.value.clear();
  pair.value.push_back((max_acceleration >> 8) & 0xFF);
  pair.value.push_back((max_acceleration >> 0) & 0xFF);
  transfer.request.pairs.push_back(pair);

  pair.key = hailfire_fpga_proxy::FPGAKeyValue::ANGLE_MAX_DECELERATION;
  pair.value.clear();
  pair.value.push_back((max_deceleration >> 8) & 0xFF);
  pair.value.push_back((max_deceleration >> 0) & 0xFF);
  transfer.request.pairs.push_back(pair);

  pair.key = hailfire_fpga_proxy::FPGAKeyValue::ANGLE_CORRECT_FILTER_GAIN_P;
  pair.value.clear();
  pair.value.push_back(correct_filter_gain_p);
  transfer.request.pairs.push_back(pair);

  pair.key = hailfire_fpga_proxy::FPGAKeyValue::ANGLE_CORRECT_FILTER_GAIN_I;
  pair.value.clear();
  pair.value.push_back(correct_filter_gain_i);
  transfer.request.pairs.push_back(pair);

  pair.key = hailfire_fpga_proxy::FPGAKeyValue::ANGLE_CORRECT_FILTER_GAIN_D;
  pair.value.clear();
  pair.value.push_back(correct_filter_gain_d);
  transfer.request.pairs.push_back(pair);

  pair.key = hailfire_fpga_proxy::FPGAKeyValue::ANGLE_CORRECT_FILTER_OUT_SHIFT;
  pair.value.clear();
  pair.value.push_back(correct_filter_out_shift);
  transfer.request.pairs.push_back(pair);

  if (srv_client_.call(transfer))
  {
    ROS_INFO("Configured angle control system");
  }
  else
  {
    ROS_ERROR("Failed to configure angle control system");
    exit(1);
  }
}

void BaseController::configureDistanceControlSystem()
{
  ros::NodeHandle nh_param("~distance_cs");

  double max_acceleration_d;
  double max_deceleration_d;
  int max_acceleration;
  int max_deceleration;
  int correct_filter_gain_p;
  int correct_filter_gain_i;
  int correct_filter_gain_d;
  int correct_filter_out_shift;

  if (nh_param.getParam("max_acceleration", max_acceleration_d) &&
      nh_param.getParam("max_deceleration", max_deceleration_d) &&
      nh_param.getParam("correct_filter_gain_p", correct_filter_gain_p) &&
      nh_param.getParam("correct_filter_gain_i", correct_filter_gain_i) &&
      nh_param.getParam("correct_filter_gain_d", correct_filter_gain_d) &&
      nh_param.getParam("correct_filter_out_shift", correct_filter_out_shift))
  {
    max_acceleration = linearOdometerTicksFromRealWorldUnits(max_acceleration_d);
    max_deceleration = linearOdometerTicksFromRealWorldUnits(max_deceleration_d);
  
    ROS_INFO("Will set distance max_acceleration: %u", max_acceleration);
    ROS_INFO("Will set distance max_deceleration: %u", max_deceleration);
    ROS_INFO("Will set distance correct_filter_gain_p: %u", correct_filter_gain_p);
    ROS_INFO("Will set distance correct_filter_gain_i: %u", correct_filter_gain_i);
    ROS_INFO("Will set distance correct_filter_gain_d: %u", correct_filter_gain_d);
    ROS_INFO("Will set distance correct_filter_out_shift: %u", correct_filter_out_shift);
  }
  else
  {
    ROS_ERROR("Failed to retrieve all distance control system parameters.");
    exit(1);
  }

  hailfire_fpga_proxy::FPGATransfer transfer;
  hailfire_fpga_proxy::FPGAKeyValue pair;

  pair.key = hailfire_fpga_proxy::FPGAKeyValue::DISTANCE_MAX_ACCELERATION;
  pair.value.clear();
  pair.value.push_back((max_acceleration >> 8) & 0xFF);
  pair.value.push_back((max_acceleration >> 0) & 0xFF);
  transfer.request.pairs.push_back(pair);

  pair.key = hailfire_fpga_proxy::FPGAKeyValue::DISTANCE_MAX_DECELERATION;
  pair.value.clear();
  pair.value.push_back((max_deceleration >> 8) & 0xFF);
  pair.value.push_back((max_deceleration >> 0) & 0xFF);
  transfer.request.pairs.push_back(pair);

  pair.key = hailfire_fpga_proxy::FPGAKeyValue::DISTANCE_CORRECT_FILTER_GAIN_P;
  pair.value.clear();
  pair.value.push_back(correct_filter_gain_p);
  transfer.request.pairs.push_back(pair);

  pair.key = hailfire_fpga_proxy::FPGAKeyValue::DISTANCE_CORRECT_FILTER_GAIN_I;
  pair.value.clear();
  pair.value.push_back(correct_filter_gain_i);
  transfer.request.pairs.push_back(pair);

  pair.key = hailfire_fpga_proxy::FPGAKeyValue::DISTANCE_CORRECT_FILTER_GAIN_D;
  pair.value.clear();
  pair.value.push_back(correct_filter_gain_d);
  transfer.request.pairs.push_back(pair);

  pair.key = hailfire_fpga_proxy::FPGAKeyValue::DISTANCE_CORRECT_FILTER_OUT_SHIFT;
  pair.value.clear();
  pair.value.push_back(correct_filter_out_shift);
  transfer.request.pairs.push_back(pair);

  if (srv_client_.call(transfer))
  {
    ROS_INFO("Configured distance control system");
  }
  else
  {
    ROS_ERROR("Failed to configure distance control system");
    exit(1);
  }
}

void BaseController::twistMsgHandler(const geometry_msgs::Twist::ConstPtr &cmd)
{
  // Converts rad/s to ticks/s
  int angle_consign = angularOdometerTicksFromRealWorldUnits(cmd->angular.z);

  // Converts m/s to ticks/s
  int distance_consign = linearOdometerTicksFromRealWorldUnits(cmd->linear.x);

  // Send to FPGA
  hailfire_fpga_proxy::FPGATransfer transfer;
  hailfire_fpga_proxy::FPGAKeyValue pair;

  pair.key = hailfire_fpga_proxy::FPGAKeyValue::ANGLE_SPEED_CONSIGN;
  pair.value.clear();
  pair.value.push_back((angle_consign >> 24) & 0xFF);
  pair.value.push_back((angle_consign >> 16) & 0xFF);
  pair.value.push_back((angle_consign >> 8) & 0xFF);
  pair.value.push_back((angle_consign >> 0) & 0xFF);
  transfer.request.pairs.push_back(pair);

  pair.key = hailfire_fpga_proxy::FPGAKeyValue::DISTANCE_SPEED_CONSIGN;
  pair.value.clear();
  pair.value.push_back((distance_consign >> 24) & 0xFF);
  pair.value.push_back((distance_consign >> 16) & 0xFF);
  pair.value.push_back((distance_consign >> 8) & 0xFF);
  pair.value.push_back((distance_consign >> 0) & 0xFF);
  transfer.request.pairs.push_back(pair);

  if (srv_client_.call(transfer))
  {
    ROS_DEBUG("Sent angle (%d) and distance (%d) consigns to the fpga_proxy", angle_consign, distance_consign);
  }
  else
  {
    ROS_ERROR("Failed to send angle and distance consigns to the fpga_proxy");
    exit(1);
  }
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "base_controller");

  hailfire_base_controller::BaseController base_controller;
  base_controller.configureAngleControlSystem();
  base_controller.configureDistanceControlSystem();

  ros::spin();

  return 0;
}
