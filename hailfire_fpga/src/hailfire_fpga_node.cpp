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
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include "hailfire_spi/spi_device.h"
#include "hailfire_fpga_msgs/ExtPort.h"
#include "hailfire_fpga_msgs/ExtPortArray.h"
#include "hailfire_fpga_msgs/IRSensor.h"
#include "hailfire_fpga_msgs/IRSensorArray.h"
#include "hailfire_fpga_msgs/Motor.h"
#include "hailfire_fpga_msgs/MotorArray.h"
#include "hailfire_fpga_msgs/Odometer.h"
#include "hailfire_fpga_msgs/OdometerArray.h"
#include "hailfire_fpga_msgs/Servo.h"
#include "hailfire_fpga_msgs/ServoArray.h"
#include "hailfire_fpga_msgs/TestRegisters.h"

#define HAILFIRE_FPGA_MAX_MSG 50

// Read odometer counts: 0x11 to 0x14
#define HAILFIRE_FPGA_ODOMETER_NB 4
#define HAILFIRE_FPGA_ODOMETER_COUNT_BASE 0x10

// Read extension ports: 0x31 to 0x37
#define HAILFIRE_FPGA_EXT_PORT_NB 7
#define HAILFIRE_FPGA_EXT_PORT_BASE 0x30

// Fixed value (0xDEADCODE) for testing
#define HAILFIRE_FPGA_FIXED_VALUE 0x42

// Read IR sensors: 0x50 to 0x57
#define HAILFIRE_FPGA_IR_SENSOR_NB 8
#define HAILFIRE_FPGA_IR_SENSOR_BASE 0x50

// Read test registers
#define HAILFIRE_FPGA_TEST_READ_UINT8 0x71
#define HAILFIRE_FPGA_TEST_READ_UINT16 0x72
#define HAILFIRE_FPGA_TEST_READ_UINT32 0x73
#define HAILFIRE_FPGA_TEST_READ_INT8 0x74
#define HAILFIRE_FPGA_TEST_READ_INT16 0x75
#define HAILFIRE_FPGA_TEST_READ_INT32 0x76

// Reset FPGA: 0x81
#define HAILFIRE_FPGA_RESET 0x81

// Set LEDs
#define HAILFIRE_FPGA_LED_GREEN 0x82
#define HAILFIRE_FPGA_LED_YELLOW 0x83

// Set motor speeds: 0x91 to 0x98
#define HAILFIRE_FPGA_MOTOR_NB 8
#define HAILFIRE_FPGA_MOTOR_SPEED_BASE 0x90

// Set servo consigns: 0xA1 to 0xA8
#define HAILFIRE_FPGA_SERVO_NB 8
#define HAILFIRE_FPGA_SERVO_CONSIGN_BASE 0xA0

// Set test registers
#define HAILFIRE_FPGA_TEST_WRITE_UINT8 0xF1
#define HAILFIRE_FPGA_TEST_WRITE_UINT16 0xF2
#define HAILFIRE_FPGA_TEST_WRITE_UINT32 0xF3
#define HAILFIRE_FPGA_TEST_WRITE_INT8 0xF4
#define HAILFIRE_FPGA_TEST_WRITE_INT16 0xF5
#define HAILFIRE_FPGA_TEST_WRITE_INT32 0xF6


namespace hailfire_fpga
{

struct FPGAKeyValue
{
  uint8_t key;
  uint8_t _length;            // used by doTransfer
  std::vector<uint8_t> value; // byte 0 is the MSB
};

/**
 * @brief Returns an hexadecimal dump of the given byte array.
 */
std::string dump_bytes(std::vector<uint8_t> const& bytes);

/**
 * @brief Returns an hexadecimal dump of the given FPGAKeyValue array.
 */
std::string dump_bytes(std::vector<FPGAKeyValue> const& kv_pairs);

/**
 * @brief Returns a vector of bytes from an (u)int(8|16|32) value.
 */
std::vector<uint8_t> bytes_from_uint8(uint8_t const& value);
std::vector<uint8_t> bytes_from_uint16(uint16_t const& value);
std::vector<uint8_t> bytes_from_uint32(uint32_t const& value);
std::vector<uint8_t> bytes_from_int8(int8_t const& value);
std::vector<uint8_t> bytes_from_int16(int16_t const& value);
std::vector<uint8_t> bytes_from_int32(int32_t const& value);

/**
 * @brief Returns an (u)int(8|16|32) value from a vector of bytes.
 */
uint8_t uint8_from_bytes(std::vector<uint8_t> const& bytes);
uint16_t uint16_from_bytes(std::vector<uint8_t> const& bytes);
uint32_t uint32_from_bytes(std::vector<uint8_t> const& bytes);
int8_t int8_from_bytes(std::vector<uint8_t> const& bytes);
int16_t int16_from_bytes(std::vector<uint8_t> const& bytes);
int32_t int32_from_bytes(std::vector<uint8_t> const& bytes);

/**
 * @class FPGANode
 * @brief Message-based ROS interface to the FPGA
 *
 * This class publishes messages on advertised topics with values fetched from
 * the FPGA, and forwards messages received on subscribed topics to the FPGA.
 *
 */
class FPGANode
{
public:

  /**
   * @brief Constructor.
   *
   * Creates and configures the SPI device, then advertises topics for sensors
   * and subscribes to topics for actuators.
   *
   * Uses the following parameters from the parameter server:
   * ~spidev/dev_name        used for SPI device instance (defaults to "spidev1.1")
   * ~spidev/mode            appropriate setter is called if present
   * ~spidev/lsb_first       appropriate setter is called if present
   * ~spidev/bits_per_word   appropriate setter is called if present
   * ~spidev/max_speed       appropriate setter is called if present
   * ~odometer_rate          publish rate for /odometer* topics (in Hz)
   * ~ext_rate               publish rate for /ext* topics (in Hz)
   * ~ir_sensor_rate         publish rate for /ir_sensor* topics (in Hz)
   * ~test_rate              publish rate for /test* topics (in Hz)
   *
   */
  FPGANode();

  /**
   * @brief Destructor: cleans up
   */
  ~FPGANode();

private:

  /**
   * @brief Advertises the topics this node is capable of publishing.
   */
  void setupAdvertisements();

  /**
   * @brief Subscribes to topics this node is capable of handling.
   */
  void setupSubscriptions();

  /**
   * @brief Publishes odometer counts.
   *
   * This method is called regularly (at ~odometer_rate) to manage the
   * following topics: /odometer/port[1-4] and /odometer/all.
   *
   * No messages are published on topics without subscribers, neither are the
   * unused values fetched from the FPGA.
   */
  void publishOdometers(ros::TimerEvent const& event);

  /**
   * @brief Publishes ext port values.
   *
   * This method is called regularly (at ~ext_rate) to manage the
   * following topics: /ext/port[1-7] and /ext/all.
   *
   * No messages are published on topics without subscribers, neither are the
   * unused values fetched from the FPGA.
   */
  void publishExtPorts(ros::TimerEvent const& event);

  /**
   * @brief Publishes IR sensor values.
   *
   * This method is called regularly (at ~ir_sensor_rate) to manage the
   * following topics: /ir_sensor/port[1-8] and /ir_sensor/all.
   *
   * No messages are published on topics without subscribers, neither are the
   * unused values fetched from the FPGA.
   */
  void publishIRSensors(ros::TimerEvent const& event);

  /**
   * @brief Publishes test register values.
   *
   * This method is called regularly (at ~test_rate) to manage the
   * /test/fixed and /test/reg_read topics.
   *
   * No messages are published on topics without subscribers, neither are the
   * unused values fetched from the FPGA.
   */
  void publishTestRegisters(ros::TimerEvent const& event);

  /**
   * @brief Handle reset message.
   *
   * This method is called by ROS when a message is published on the /reset
   * topic. It transforms and forwards the message to the FPGA.
   */
  void handleReset(std_msgs::Empty::ConstPtr const& empty);

  /**
   * @brief Handles LED on/off messages.
   *
   * This method is called by ROS when a message is published on the /led/green
   * and /led/yellow topics. It transforms and forwards the message to the FPGA.
   *
   * In the std_msgs::Bool message, set data to true for "on", false for "off".
   */
  void handleLedMsg(uint8_t led_key, std_msgs::Bool::ConstPtr const& msg);

  /**
   * @brief Handles motor consign messages.
   *
   * This method is called by ROS when a message is published on the
   * /motor/port[1-8] topics. It transforms and forwards the message to the FPGA.
   *
   * In the std_msgs::Int16 message, set data to the signed motor speed consign.
   *
   * NB: -1024 <= speed consign < 1024
   */
  void handleMotorMsg(uint8_t motor_nb, std_msgs::Int16::ConstPtr const& msg);

  /**
   * @brief Handles servo consign messages.
   *
   * This method is called by ROS when a message is published on the
   * /servo/port[1-8] topics. It transforms and forwards the message to the FPGA.
   *
   * In the std_msgs::UInt16 message, set data to the servo position consign.
   *
   * Note: 12500 < position consign < 62500 (for most servos), 0 to disable
   */
  void handleServoMsg(uint8_t servo_nb, std_msgs::UInt16::ConstPtr const& msg);

  /**
   * @brief Handles multi-motor consign messages.
   *
   * This method is called by ROS when a message is published on the
   * /motor/combined topic. It transforms and forwards the message to the FPGA.
   */
  void handleMotorCombinedMsg(hailfire_fpga_msgs::MotorArray::ConstPtr const& msg);

  /**
   * @brief Handles multi-servo consign messages.
   *
   * This method is called by ROS when a message is published on the
   * /servo/combined topic. It transforms and forwards the message to the FPGA.
   */
  void handleServoCombinedMsg(hailfire_fpga_msgs::ServoArray::ConstPtr const& msg);

  /**
   * @brief Handles test register write messages.
   *
   * This method is called by ROS when a message is published on the
   * /test/reg_write topic. It transforms and forwards the message to the FPGA.
   */
  void handleTestRegistersMsg(hailfire_fpga_msgs::TestRegisters::ConstPtr const& msg);

  /**
   * @brief Handles the encoding/decoding of FPGA messages and .
   *
   * This method is called internally: it transforms requests to a format
   * accepted by the FPGA (Key, Length, Value encoded byte array), sends it to
   * the FPGA via SPI and transforms the FPGA response back to a ROS format.
   */
  void doTransfer(std::vector<FPGAKeyValue> &kv_pairs);

  ros::NodeHandle nh_;                          /**< The ROS node handle */

  ros::Publisher odometers_pub_;                /**< /odometer/all publisher */
  std::vector<ros::Publisher> odometer_pub_;    /**< /odometer/port[1-4] publishers */
  ros::Timer odometer_timer_;                  /**< Timer for publishOdometers */

  ros::Publisher ext_ports_pub_;                /**< /ext/all publisher */
  std::vector<ros::Publisher> ext_port_pub_;    /**< /ext/port[1-7] publishers */
  ros::Timer ext_port_timer_;                  /**< Timer for publishExtPorts */

  ros::Publisher ir_sensors_pub_;               /**< /ir_sensor/all publisher */
  std::vector<ros::Publisher> ir_sensor_pub_;   /**< /ir_sensor/port[1-8] publishers */
  ros::Timer ir_sensor_timer_;                  /**< Timer for publishIRSensors */

  ros::Subscriber reset_sub_;                   /**< /reset subscriber */

  ros::Subscriber led_green_sub_;               /**< /led/green subscriber */
  ros::Subscriber led_yellow_sub_;              /**< /led/yellow subscriber */

  ros::Subscriber motors_sub_;                  /**< /motor/combined subscriber */
  std::vector<ros::Subscriber> motor_sub_;      /**< /motor/port[1-8] subscriber */

  ros::Subscriber servos_sub_;                  /**< /servo/combined subscriber */
  std::vector<ros::Subscriber> servo_sub_;      /**< /servo/port[1-8] subscriber */

  ros::Publisher fixed_pub_;                    /**< /test/fixed publisher */
  ros::Publisher reg_read_pub_;                 /**< /test/reg_read publisher */
  ros::Subscriber reg_write_sub_;               /**< /test/reg_write subscriber */
  ros::Timer test_timer_;                       /**< Timer for publishTestRegisters */

  hailfire_spi::SPIDevice *spi_device_;         /**< The SPI device instance */
};

FPGANode::FPGANode()
{
  ros::NodeHandle nh_spidev("~spidev");

  std::string dev_name;
  nh_spidev.param("dev_name", dev_name, std::string("/dev/spidev1.1"));

  bool inhibit;
  nh_spidev.param("inhibit", inhibit, false);
  if (inhibit)
  {
    ROS_INFO("~spidev/inhibit set: SPI will not be used");
    spi_device_ = NULL;
  }
  else
  {
    ROS_INFO("SPI device: %s", dev_name.c_str());
    spi_device_ = new hailfire_spi::SPIDevice(dev_name.c_str());
  }

  int mode;
  if (spi_device_ && nh_spidev.getParam("mode", mode))
  {
    ROS_INFO("SPI mode: %u", mode);
    spi_device_->setMode(mode);
  }

  bool lsb_first;
  if (spi_device_ && nh_spidev.getParam("lsb_first", lsb_first))
  {
    ROS_INFO("SPI lsb_first: %s", (lsb_first ? "true" : "false"));
    spi_device_->setLSBFirst(lsb_first);
  }

  int bits_per_word;
  if (spi_device_ && nh_spidev.getParam("bits_per_word", bits_per_word))
  {
    ROS_INFO("SPI bits_per_word: %u", bits_per_word);
    spi_device_->setBitsPerWord(bits_per_word);
  }

  int max_speed;
  if (spi_device_ && nh_spidev.getParam("max_speed", max_speed))
  {
    ROS_INFO("SPI max_speed: %u", max_speed);
    spi_device_->setMaxSpeed(max_speed);
  }

  setupAdvertisements();
  setupSubscriptions();

  ros::NodeHandle nh_param("~");
  int odometer_rate;
  if (!nh_param.getParam("odometer_rate", odometer_rate))
  {
    ROS_ERROR("Missing ~odometer_rate publish rate param.");
    exit(1);
  }
  odometer_timer_ = nh_.createTimer(ros::Rate(odometer_rate), &FPGANode::publishOdometers, this);

  int ext_rate;
  if (!nh_param.getParam("ext_rate", ext_rate))
  {
    ROS_ERROR("Missing ~ext_rate publish rate param.");
    exit(1);
  }
  ext_port_timer_ = nh_.createTimer(ros::Rate(ext_rate), &FPGANode::publishExtPorts, this);

  int ir_sensor_rate;
  if (!nh_param.getParam("ir_sensor_rate", ir_sensor_rate))
  {
    ROS_ERROR("Missing ~ir_sensor_rate publish rate param.");
    exit(1);
  }
  ir_sensor_timer_ = nh_.createTimer(ros::Rate(ir_sensor_rate), &FPGANode::publishIRSensors, this);

  int test_rate;
  if (!nh_param.getParam("test_rate", test_rate))
  {
    ROS_ERROR("Missing ~test_rate publish rate param.");
    exit(1);
  }
  test_timer_ = nh_.createTimer(ros::Rate(test_rate), &FPGANode::publishTestRegisters, this);

  ROS_INFO("Ready to service FPGA requests");
}

FPGANode::~FPGANode()
{
  if (spi_device_)
  {
    delete spi_device_;
  }
}

void FPGANode::setupAdvertisements()
{
  unsigned int i;

  // /odometer/all
  ros::NodeHandle nh_odometer("~odometer");
  odometers_pub_ =
    nh_odometer.advertise<hailfire_fpga_msgs::OdometerArray>("all", HAILFIRE_FPGA_MAX_MSG);

  // /odometer/port[1-4]
  odometer_pub_.reserve(HAILFIRE_FPGA_ODOMETER_NB);
  for (i = 0; i < HAILFIRE_FPGA_ODOMETER_NB; ++i)
  {
    std::ostringstream oss;
    oss << "port" << (i + 1);
    odometer_pub_.push_back(
      nh_odometer.advertise<hailfire_fpga_msgs::Odometer>(oss.str(), HAILFIRE_FPGA_MAX_MSG));
  }

  // /ext/all
  ros::NodeHandle nh_ext_port("~ext");
  ext_ports_pub_ =
    nh_ext_port.advertise<hailfire_fpga_msgs::ExtPortArray>("all", HAILFIRE_FPGA_MAX_MSG);

  // /ext/port[1-7]
  ext_port_pub_.reserve(HAILFIRE_FPGA_EXT_PORT_NB);
  for (i = 0; i < HAILFIRE_FPGA_EXT_PORT_NB; ++i)
  {
    std::ostringstream oss;
    oss << "port" << (i + 1);
    ext_port_pub_.push_back(
      nh_ext_port.advertise<hailfire_fpga_msgs::ExtPort>(oss.str(), HAILFIRE_FPGA_MAX_MSG));
  }

  // /ir_sensor/all
  ros::NodeHandle nh_ir_sensor("~ir_sensor");
  ir_sensors_pub_ =
    nh_ir_sensor.advertise<hailfire_fpga_msgs::IRSensorArray>("all", HAILFIRE_FPGA_MAX_MSG);

  // /ir_sensor/port[1-8]
  ir_sensor_pub_.reserve(HAILFIRE_FPGA_IR_SENSOR_NB);
  for (i = 0; i < HAILFIRE_FPGA_IR_SENSOR_NB; ++i)
  {
    std::ostringstream oss;
    oss << "port" << (i + 1);
    ir_sensor_pub_.push_back(
      nh_ir_sensor.advertise<hailfire_fpga_msgs::IRSensor>(oss.str(), HAILFIRE_FPGA_MAX_MSG));
  }

  // /test/fixed
  ros::NodeHandle nh_test("~test");
  fixed_pub_ = nh_test.advertise<std_msgs::UInt32>("fixed", HAILFIRE_FPGA_MAX_MSG);

  // /test/reg_read
  reg_read_pub_ =
    nh_test.advertise<hailfire_fpga_msgs::TestRegisters>("reg_read", HAILFIRE_FPGA_MAX_MSG);
}

void FPGANode::setupSubscriptions()
{
  unsigned int i;

  // /reset
  ros::NodeHandle nh_reset("~");
  reset_sub_ = nh_reset.subscribe("reset", 1, &FPGANode::handleReset, this);

  // /led/green and /led/yellow
  ros::NodeHandle nh_led("~led");
  led_green_sub_  = nh_led.subscribe<std_msgs::Bool>("green", 1,
    boost::bind(&FPGANode::handleLedMsg, this, HAILFIRE_FPGA_LED_GREEN, _1));
  led_yellow_sub_ = nh_led.subscribe<std_msgs::Bool>("yellow", 1,
    boost::bind(&FPGANode::handleLedMsg, this, HAILFIRE_FPGA_LED_YELLOW, _1));

  // /motor/combined
  ros::NodeHandle nh_motor("~motor");
  motors_sub_ = nh_motor.subscribe("combined", 1, &FPGANode::handleMotorCombinedMsg, this);

  // /motor/port[1-8]
  motor_sub_.reserve(HAILFIRE_FPGA_MOTOR_NB);
  for (i = 0; i < HAILFIRE_FPGA_MOTOR_NB; ++i)
  {
    std::ostringstream oss;
    oss << "port" << (i + 1);
    motor_sub_.push_back(nh_motor.subscribe<std_msgs::Int16>
        (oss.str(), 1, boost::bind(&FPGANode::handleMotorMsg, this, i + 1, _1)));
  }

  // /servo/combined
  ros::NodeHandle nh_servo("~servo");
  servos_sub_ = nh_servo.subscribe("combined", 1, &FPGANode::handleServoCombinedMsg, this);

  // /servo/port[1-8]
  servo_sub_.reserve(HAILFIRE_FPGA_SERVO_NB);
  for (i = 0; i < HAILFIRE_FPGA_SERVO_NB; ++i)
  {
    std::ostringstream oss;
    oss << "port" << (i + 1);
    servo_sub_.push_back(nh_servo.subscribe<std_msgs::UInt16>
        (oss.str(), 1, boost::bind(&FPGANode::handleServoMsg, this, i + 1, _1)));
  }

  // /test/reg_write
  ros::NodeHandle nh_test("~test");
  reg_write_sub_ = nh_test.subscribe("reg_write", 1, &FPGANode::handleTestRegistersMsg, this);
}

void FPGANode::publishOdometers(ros::TimerEvent const& event)
{
  unsigned int i;

  // Need to know which topics must be published
  bool publish_all = odometers_pub_.getNumSubscribers() > 0;
  std::vector<bool> publish_needed (HAILFIRE_FPGA_ODOMETER_NB, false);
  for (i = 0; i < HAILFIRE_FPGA_ODOMETER_NB; ++i)
  {
    publish_needed[i] = odometer_pub_[i].getNumSubscribers() > 0;
  }

  // Need to know which odometers must be read
  std::vector<uint8_t> reading_needed;
  for (i = 0; i < HAILFIRE_FPGA_ODOMETER_NB; ++i)
  {
    if (publish_all || publish_needed[i])
      reading_needed.push_back((uint8_t) i + 1);
  }

  // Nothing to do?
  if (reading_needed.size() == 0)
    return;

  // Prepare FPGA request
  std::vector<FPGAKeyValue> kv_pairs;
  kv_pairs.reserve(2 * reading_needed.size());
  for (i = 0; i < reading_needed.size(); ++i)
  {
    // Read int32 count
    FPGAKeyValue read_count;
    read_count.key = HAILFIRE_FPGA_ODOMETER_COUNT_BASE + reading_needed[i];
    read_count.value.assign(4, 0);
    kv_pairs.push_back(read_count);
  }

  doTransfer(kv_pairs);

  // Gather values and publish to subscribed topics
  hailfire_fpga_msgs::OdometerArray all_msg;
  unsigned int pair_i = 0;
  for (i = 0; i < HAILFIRE_FPGA_ODOMETER_NB; ++i)
  {
    if (publish_all || publish_needed[i])
    {
      hailfire_fpga_msgs::Odometer single_msg;
      single_msg.count = int32_from_bytes(kv_pairs[pair_i++].value);

      if (publish_needed[i])
        odometer_pub_[i].publish(single_msg);

      if (publish_all)
        all_msg.odometers.push_back(single_msg);
    }
  }

  if (publish_all)
    odometers_pub_.publish(all_msg);
}

void FPGANode::publishExtPorts(ros::TimerEvent const& event)
{
  unsigned int i;

  // Need to know which topics must be published
  bool publish_all = ext_ports_pub_.getNumSubscribers() > 0;
  std::vector<bool> publish_needed (HAILFIRE_FPGA_EXT_PORT_NB, false);
  for (i = 0; i < HAILFIRE_FPGA_EXT_PORT_NB; ++i)
  {
    publish_needed[i] = ext_port_pub_[i].getNumSubscribers() > 0;
  }

  // Need to know which ext_ports must be read
  std::vector<uint8_t> reading_needed;
  for (i = 0; i < HAILFIRE_FPGA_EXT_PORT_NB; ++i)
  {
    if (publish_all || publish_needed[i])
      reading_needed.push_back((uint8_t) i + 1);
  }

  // Nothing to do?
  if (reading_needed.size() == 0)
    return;

  // Prepare FPGA request
  std::vector<FPGAKeyValue> kv_pairs;
  kv_pairs.reserve(reading_needed.size());
  for (i = 0; i < reading_needed.size(); ++i)
  {
    // Read uint8 port
    FPGAKeyValue read_port;
    read_port.key = HAILFIRE_FPGA_EXT_PORT_BASE + reading_needed[i];
    read_port.value.assign(1, 0);
    kv_pairs.push_back(read_port);
  }

  doTransfer(kv_pairs);

  // Gather values and publish to subscribed topics
  hailfire_fpga_msgs::ExtPortArray all_msg;
  unsigned int pair_i = 0;
  for (i = 0; i < HAILFIRE_FPGA_EXT_PORT_NB; ++i)
  {
    if (publish_all || publish_needed[i])
    {
      hailfire_fpga_msgs::ExtPort single_msg;
      uint8_t port = uint8_from_bytes(kv_pairs[pair_i++].value);
      single_msg.pins[0] = (port & 0x01) ? true : false;
      single_msg.pins[1] = (port & 0x02) ? true : false;
      single_msg.pins[2] = (port & 0x04) ? true : false;
      single_msg.pins[3] = (port & 0x08) ? true : false;
      single_msg.pins[4] = (port & 0x10) ? true : false;
      single_msg.pins[5] = (port & 0x20) ? true : false;
      single_msg.pins[6] = (port & 0x40) ? true : false;
      single_msg.pins[7] = (port & 0x80) ? true : false;

      if (publish_needed[i])
        ext_port_pub_[i].publish(single_msg);

      if (publish_all)
        all_msg.ext_ports.push_back(single_msg);
    }
  }

  if (publish_all)
    ext_ports_pub_.publish(all_msg);
}

void FPGANode::publishIRSensors(ros::TimerEvent const& event)
{
  unsigned int i;

  // Need to know which topics must be published
  bool publish_all = ir_sensors_pub_.getNumSubscribers() > 0;
  std::vector<bool> publish_needed (HAILFIRE_FPGA_IR_SENSOR_NB, false);
  for (i = 0; i < HAILFIRE_FPGA_IR_SENSOR_NB; ++i)
  {
    publish_needed[i] = ir_sensor_pub_[i].getNumSubscribers() > 0;
  }

  // Need to know which IR sensors must be read
  std::vector<uint8_t> reading_needed;
  for (i = 0; i < HAILFIRE_FPGA_IR_SENSOR_NB; ++i)
  {
    if (publish_all || publish_needed[i])
      reading_needed.push_back((uint8_t) i);
  }

  // Nothing to do?
  if (reading_needed.size() == 0)
    return;

  // Prepare FPGA request
  std::vector<FPGAKeyValue> kv_pairs;
  kv_pairs.reserve(2 * reading_needed.size());
  for (i = 0; i < reading_needed.size(); ++i)
  {
    // Read uint16 value (really, 10-bit value)
    FPGAKeyValue read_ir_sensor;
    read_ir_sensor.key = HAILFIRE_FPGA_IR_SENSOR_BASE + reading_needed[i];
    read_ir_sensor.value.assign(2, 0);
    kv_pairs.push_back(read_ir_sensor);
  }

  doTransfer(kv_pairs);

  // Gather values and publish to subscribed topics
  hailfire_fpga_msgs::IRSensorArray all_msg;
  unsigned int pair_i = 0;
  for (i = 0; i < HAILFIRE_FPGA_IR_SENSOR_NB; ++i)
  {
    if (publish_all || publish_needed[i])
    {
      hailfire_fpga_msgs::IRSensor single_msg;
      single_msg.value = int16_from_bytes(kv_pairs[pair_i++].value);

      if (publish_needed[i])
        ir_sensor_pub_[i].publish(single_msg);

      if (publish_all)
        all_msg.ir_sensors.push_back(single_msg);
    }
  }

  if (publish_all)
    ir_sensors_pub_.publish(all_msg);
}

void FPGANode::publishTestRegisters(ros::TimerEvent const& event)
{
  std::vector<FPGAKeyValue> kv_pairs;

  if (reg_read_pub_.getNumSubscribers() > 0)
  {
    // Read uint8 test register
    FPGAKeyValue read_uint8;
    read_uint8.key = HAILFIRE_FPGA_TEST_READ_UINT8;
    read_uint8.value.assign(1, 0);

    // Read uint16 test register
    FPGAKeyValue read_uint16;
    read_uint16.key = HAILFIRE_FPGA_TEST_READ_UINT16;
    read_uint16.value.assign(2, 0);

    // Read uint32 test register
    FPGAKeyValue read_uint32;
    read_uint32.key = HAILFIRE_FPGA_TEST_READ_UINT32;
    read_uint32.value.assign(4, 0);

    // Read int8 test register
    FPGAKeyValue read_int8;
    read_int8.key = HAILFIRE_FPGA_TEST_READ_INT8;
    read_int8.value.assign(1, 0);

    // Read int16 test register
    FPGAKeyValue read_int16;
    read_int16.key = HAILFIRE_FPGA_TEST_READ_INT16;
    read_int16.value.assign(2, 0);

    // Read int32 test register
    FPGAKeyValue read_int32;
    read_int32.key = HAILFIRE_FPGA_TEST_READ_INT32;
    read_int32.value.assign(4, 0);

    kv_pairs.reserve(7);
    kv_pairs.push_back(read_uint8);
    kv_pairs.push_back(read_uint16);
    kv_pairs.push_back(read_uint32);
    kv_pairs.push_back(read_int8);
    kv_pairs.push_back(read_int16);
    kv_pairs.push_back(read_int32);
  }

  if (fixed_pub_.getNumSubscribers() > 0)
  {
    // Read fixed uint32 test register
    FPGAKeyValue read_fixed;
    read_fixed.key = HAILFIRE_FPGA_FIXED_VALUE;
    read_fixed.value.assign(4, 0);
    kv_pairs.push_back(read_fixed);
  }

  // Nothing to do?
  if (kv_pairs.size() == 0)
    return;

  doTransfer(kv_pairs);

  // Get the responses
  unsigned int pair_i = 0;
  if (reg_read_pub_.getNumSubscribers() > 0)
  {
    hailfire_fpga_msgs::TestRegisters msg;
    msg.uint8_value  = uint8_from_bytes(kv_pairs[pair_i++].value);
    msg.uint16_value = uint16_from_bytes(kv_pairs[pair_i++].value);
    msg.uint32_value = uint32_from_bytes(kv_pairs[pair_i++].value);
    msg.int8_value   = int8_from_bytes(kv_pairs[pair_i++].value);
    msg.int16_value  = int16_from_bytes(kv_pairs[pair_i++].value);
    msg.int32_value  = int32_from_bytes(kv_pairs[pair_i++].value);
    reg_read_pub_.publish(msg);
  }

  if (fixed_pub_.getNumSubscribers() > 0)
  {
    std_msgs::UInt32 msg;
    msg.data = uint32_from_bytes(kv_pairs[pair_i++].value);
    fixed_pub_.publish(msg);
  }
}

void FPGANode::handleReset(std_msgs::Empty::ConstPtr const& empty)
{
  FPGAKeyValue reset;
  reset.key = HAILFIRE_FPGA_RESET;
  reset.value = bytes_from_uint8(1);

  std::vector<FPGAKeyValue> kv_pairs;
  kv_pairs.push_back(reset);

  doTransfer(kv_pairs);
}

void FPGANode::handleLedMsg(uint8_t led_key, std_msgs::Bool::ConstPtr const& msg)
{
  // Set led on/off bool
  FPGAKeyValue set_led;
  set_led.key = led_key;
  set_led.value = bytes_from_uint8(msg->data ? 1 : 0);

  std::vector<FPGAKeyValue> kv_pairs;
  kv_pairs.push_back(set_led);

  doTransfer(kv_pairs);
}

void FPGANode::handleMotorMsg(uint8_t motor_nb, std_msgs::Int16::ConstPtr const& msg)
{
  // Set int16 motor speed
  FPGAKeyValue set_motor;
  set_motor.key = HAILFIRE_FPGA_MOTOR_SPEED_BASE + motor_nb;
  set_motor.value = bytes_from_int16(msg->data);

  std::vector<FPGAKeyValue> kv_pairs;
  kv_pairs.push_back(set_motor);

  doTransfer(kv_pairs);
}

void FPGANode::handleServoMsg(uint8_t servo_nb, std_msgs::UInt16::ConstPtr const& msg)
{
  // Set uint16 servo consign speed
  FPGAKeyValue set_servo;
  set_servo.key = HAILFIRE_FPGA_SERVO_CONSIGN_BASE + servo_nb;
  set_servo.value = bytes_from_uint16(msg->data);

  std::vector<FPGAKeyValue> kv_pairs;
  kv_pairs.push_back(set_servo);

  doTransfer(kv_pairs);
}

void FPGANode::handleMotorCombinedMsg(hailfire_fpga_msgs::MotorArray::ConstPtr const& msg)
{
  std::vector<FPGAKeyValue> kv_pairs;
  kv_pairs.reserve(msg->motors.size());

  for (unsigned int i = 0; i < msg->motors.size(); ++i)
  {
    // Set int16 motor speed
    FPGAKeyValue set_motor;
    set_motor.key = HAILFIRE_FPGA_MOTOR_SPEED_BASE + msg->motors[i].key;
    set_motor.value = bytes_from_int16(msg->motors[i].speed);
    kv_pairs.push_back(set_motor);
  }

  doTransfer(kv_pairs);
}

void FPGANode::handleServoCombinedMsg(hailfire_fpga_msgs::ServoArray::ConstPtr const& msg)
{
  std::vector<FPGAKeyValue> kv_pairs;
  kv_pairs.reserve(msg->servos.size());

  for (unsigned int i = 0; i < msg->servos.size(); ++i)
  {
    // Set uint16 servo consign speed
    FPGAKeyValue set_servo;
    set_servo.key = HAILFIRE_FPGA_SERVO_CONSIGN_BASE + msg->servos[i].key;
    set_servo.value = bytes_from_uint16(msg->servos[i].consign);
    kv_pairs.push_back(set_servo);
  }

  doTransfer(kv_pairs);
}

void FPGANode::handleTestRegistersMsg(hailfire_fpga_msgs::TestRegisters::ConstPtr const& msg)
{
  std::vector<FPGAKeyValue> kv_pairs;

  // Set uint8 test register with new value
  FPGAKeyValue set_uint8;
  set_uint8.key = HAILFIRE_FPGA_TEST_WRITE_UINT8;
  set_uint8.value = bytes_from_uint8(msg->uint8_value);

  // Set uint16 test register with new value
  FPGAKeyValue set_uint16;
  set_uint16.key = HAILFIRE_FPGA_TEST_WRITE_UINT16;
  set_uint16.value = bytes_from_uint16(msg->uint16_value);

  // Set uint32 test register with new value
  FPGAKeyValue set_uint32;
  set_uint32.key = HAILFIRE_FPGA_TEST_WRITE_UINT32;
  set_uint32.value = bytes_from_uint32(msg->uint32_value);

  // Set int8 test register with new value
  FPGAKeyValue set_int8;
  set_int8.key = HAILFIRE_FPGA_TEST_WRITE_INT8;
  set_int8.value = bytes_from_int8(msg->int8_value);

  // Set int16 test register with new value
  FPGAKeyValue set_int16;
  set_int16.key = HAILFIRE_FPGA_TEST_WRITE_INT16;
  set_int16.value = bytes_from_int16(msg->int16_value);

  // Set int32 test register with new value
  FPGAKeyValue set_int32;
  set_int32.key = HAILFIRE_FPGA_TEST_WRITE_INT32;
  set_int32.value = bytes_from_int32(msg->int32_value);

  kv_pairs.reserve(6);
  kv_pairs.push_back(set_uint8);
  kv_pairs.push_back(set_uint16);
  kv_pairs.push_back(set_uint32);
  kv_pairs.push_back(set_int8);
  kv_pairs.push_back(set_int16);
  kv_pairs.push_back(set_int32);

  doTransfer(kv_pairs);
}

void FPGANode::doTransfer(std::vector<FPGAKeyValue> &kv_pairs)
{
  unsigned int i, j;

  ROS_DEBUG("doTransfer");

  ROS_DEBUG_STREAM("Request pairs:" << dump_bytes(kv_pairs));

  // Count required number of bytes for KLV-encoded vector
  unsigned int nb_bytes = 0;
  for (i = 0; i < kv_pairs.size(); ++i)
  {
    // The length being 1-byte only, the value must not be longer than 256
    // bytes, or it will be truncated.
    kv_pairs[i]._length = 0xFF & kv_pairs[i].value.size();

    // 1-byte key + 1-byte length + value
    nb_bytes += 2 + kv_pairs[i]._length;
  }

  // KLV-encode given keys and values
  std::vector<uint8_t> tx_rx_bytes;
  tx_rx_bytes.reserve(nb_bytes);

  // Create KLV-encoded vector
  for (i = 0; i < kv_pairs.size(); ++i)
  {
    tx_rx_bytes.push_back(kv_pairs[i].key);
    tx_rx_bytes.push_back(kv_pairs[i]._length);
    for (j = 0; j < kv_pairs[i]._length; ++j) // byte 0 is the MSB
      tx_rx_bytes.push_back(kv_pairs[i].value[j]);
  }

  ROS_DEBUG_STREAM("tx bytes:" << dump_bytes(tx_rx_bytes));

  // Synchronous SPI transfer, using the same vector to store the received
  // bytes (same length so valid memory space)
  if (spi_device_)
    spi_device_->doSyncTransfer(&tx_rx_bytes[0], &tx_rx_bytes[0], nb_bytes);

  ROS_DEBUG_STREAM("rx bytes:" << dump_bytes(tx_rx_bytes));

  // Insert received values in pairs vector
  unsigned int rx_offset = 0;
  for (i = 0; i < kv_pairs.size(); ++i)
  {
    // account for key and length (null) bytes in response
    rx_offset += 2;

    // Value
    for (j = 0; j < kv_pairs[i]._length; ++j, ++rx_offset)
      kv_pairs[i].value[j] = tx_rx_bytes[rx_offset];
  }

  ROS_DEBUG_STREAM("Response pairs:" << dump_bytes(kv_pairs));
}

std::string dump_bytes(std::vector<uint8_t> const& bytes)
{
  std::stringstream ss;
  ss << std::setfill('0');
  ss << std::hex;
  for (unsigned int i = 0; i < bytes.size(); ++i)
  {
    ss << std::endl << "  0x" << std::setw(2) << +bytes[i];
  }
  return ss.str();
}

std::string dump_bytes(std::vector<FPGAKeyValue> const& kv_pairs)
{
  std::stringstream ss;
  ss << std::setfill('0');
  ss << std::hex;
  for (unsigned int i = 0; i < kv_pairs.size(); ++i)
  {
    ss << std::endl << "  key: 0x" << std::setw(2) << +kv_pairs[i].key;
    ss << std::endl << "  value:";
    for (unsigned int j = 0; j < kv_pairs[i].value.size(); ++j)
    {
      ss << std::endl << "    0x" << std::setw(2) << +kv_pairs[i].value[j];
    }
  }
  return ss.str();
}

uint8_t uint8_from_bytes(std::vector<uint8_t> const& bytes)
{
  uint8_t value = bytes[0];
  return value;
}

uint16_t uint16_from_bytes(std::vector<uint8_t> const& bytes)
{
  uint16_t value = (bytes[0] << 8) |
                   (bytes[1] << 0);
  return value;
}

uint32_t uint32_from_bytes(std::vector<uint8_t> const& bytes)
{
  uint32_t value = (bytes[0] << 24) |
                   (bytes[1] << 16) |
                   (bytes[2] << 8) |
                   (bytes[3] << 0);
  return value;
}

int8_t int8_from_bytes(std::vector<uint8_t> const& bytes)
{
  int8_t value = (int8_t) uint8_from_bytes(bytes);
  return value;
}

int16_t int16_from_bytes(std::vector<uint8_t> const& bytes)
{
  int16_t value = (int16_t) uint16_from_bytes(bytes);
  return value;
}

int32_t int32_from_bytes(std::vector<uint8_t> const& bytes)
{
  int32_t value = (int32_t) uint32_from_bytes(bytes);
  return value;
}

std::vector<uint8_t> bytes_from_uint8(uint8_t const& value)
{
  std::vector<uint8_t> bytes (1, 0);
  bytes[0] = value;
  return bytes;
}

std::vector<uint8_t> bytes_from_uint16(uint16_t const& value)
{
  std::vector<uint8_t> bytes (2, 0);
  bytes[0] = (value >> 8) & 0xFF;
  bytes[1] = (value >> 0) & 0xFF;
  return bytes;
}

std::vector<uint8_t> bytes_from_uint32(uint32_t const& value)
{
  std::vector<uint8_t> bytes (4, 0);
  bytes[0] = (value >> 24) & 0xFF;
  bytes[1] = (value >> 16) & 0xFF;
  bytes[2] = (value >> 8) & 0xFF;
  bytes[3] = (value >> 0) & 0xFF;
  return bytes;
}

std::vector<uint8_t> bytes_from_int8(int8_t const& value)
{
  uint8_t uvalue = (uint8_t) value;
  return bytes_from_uint8(uvalue);
}

std::vector<uint8_t> bytes_from_int16(int16_t const& value)
{
  uint16_t uvalue = (uint16_t) value;
  return bytes_from_uint16(uvalue);
}

std::vector<uint8_t> bytes_from_int32(int32_t const& value)
{
  uint32_t uvalue = (uint32_t) value;
  return bytes_from_uint32(uvalue);
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hailfire_fpga");

  hailfire_fpga::FPGANode fpga_node;
  ros::spin();

  return 0;
}
