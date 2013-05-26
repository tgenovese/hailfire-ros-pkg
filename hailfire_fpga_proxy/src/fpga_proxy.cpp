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
#include "hailfire_spi/spi_device.h"
#include "hailfire_fpga_msgs/FPGAKeyValue.h"
#include "hailfire_fpga_msgs/FPGATransfer.h"
#include "hailfire_fpga_msgs/FPGATestRegisters.h"
#include "hailfire_fpga_msgs/ExtPort.h"
#include "hailfire_fpga_msgs/LED.h"
#include "hailfire_fpga_msgs/Motor.h"
#include "hailfire_fpga_msgs/Odometer.h"
#include "hailfire_fpga_msgs/Servo.h"
#include "hailfire_fpga_msgs/FPGA.h"

/**
 * @brief Returns an hexadecimal dump of the given byte array.
 */
std::string dump_bytes(std::vector<uint8_t> const &bytes);

/**
 * @brief Returns an hexadecimal dump of the given FPGAKeyValue array.
 */
std::string dump_bytes(std::vector<hailfire_fpga_msgs::FPGAKeyValue> const &bytes);

std::vector<uint8_t> bytes_from_uint8(uint8_t const &value);
std::vector<uint8_t> bytes_from_uint16(uint16_t const &value);
std::vector<uint8_t> bytes_from_uint32(uint32_t const &value);
std::vector<uint8_t> bytes_from_int8(int8_t const &value);
std::vector<uint8_t> bytes_from_int16(int16_t const &value);
std::vector<uint8_t> bytes_from_int32(int32_t const &value);

uint8_t uint8_from_bytes(std::vector<uint8_t> const &bytes);
uint16_t uint16_from_bytes(std::vector<uint8_t> const &bytes);
uint32_t uint32_from_bytes(std::vector<uint8_t> const &bytes);
int8_t int8_from_bytes(std::vector<uint8_t> const &bytes);
int16_t int16_from_bytes(std::vector<uint8_t> const &bytes);
int32_t int32_from_bytes(std::vector<uint8_t> const &bytes);

/**
 * @class FPGAProxy
 * @brief Service node class to proxy key-value requests to the FPGA.
 *
 * This class implements a service allowing key-value requests (in ROS messages)
 * to be forwarded to the FPGA (through SPI), whose responses are then returned
 * in the response to the ROS service call.
 *
 */
class FPGAProxy
{
public:

  /**
   * @brief Constructor.
   *
   * Creates and configures the SPI device, then advertises the service.
   *
   * Uses the following parameters from the parameter server:
   * ~spidev/dev_name        used for SPI device instance (defaults to "spidev1.0")
   * ~spidev/mode            appropriate setter is called if present
   * ~spidev/lsb_first       appropriate setter is called if present
   * ~spidev/bits_per_word   appropriate setter is called if present
   * ~spidev/max_speed       appropriate setter is called if present
   *
   */
  FPGAProxy();

  /**
   * @brief Destructor: cleans up
   */
  ~FPGAProxy();

private:

  /**
   * @brief Service handler.
   *
   * This method is called by ROS when a request to the service is made.
   * It transforms the ROS message in a format accepted by the FPGA (Key,
   * Length, Value encoded byte array), sends it to the FPGA via SPI and
   * transforms the FPGA response back to a ROS format.
   */
  bool doTransfer(hailfire_fpga_msgs::FPGATransfer::Request &req,
                  hailfire_fpga_msgs::FPGATransfer::Response &res);

  /**
   * @brief Service handler.
   *
   * This method is called by ROS when a request to the service is made.
   * It transforms the ROS message in a format accepted by the FPGA (Key,
   * Length, Value encoded byte array), sends it to the FPGA via SPI and
   * transforms the FPGA response back to a ROS format.
   */
  bool doTestRegisters(hailfire_fpga_msgs::FPGATestRegisters::Request &req,
                       hailfire_fpga_msgs::FPGATestRegisters::Response &res);

  /**
   * @brief Service handler.
   *
   * This method is called by ROS when a request to the service is made.
   * It transforms the ROS message in a format accepted by the FPGA (Key,
   * Length, Value encoded byte array), sends it to the FPGA via SPI and
   * transforms the FPGA response back to a ROS format.
   */
  bool doHighLevel(hailfire_fpga_msgs::FPGA::Request &req,
                   hailfire_fpga_msgs::FPGA::Response &res);

  ros::NodeHandle nh_;                  /**< The ROS node handle */
  ros::ServiceServer srv1_;             /**< A ROS service handle */
  ros::ServiceServer srv2_;             /**< A ROS service handle */
  ros::ServiceServer srv3_;             /**< A ROS service handle */
  hailfire_spi::SPIDevice *spi_device_; /**< The SPI device instance */
};

FPGAProxy::FPGAProxy()
{
  ros::NodeHandle nh_param("~spidev");

  std::string dev_name;
  nh_param.param("dev_name", dev_name, std::string("/dev/spidev1.1"));

  bool inhibit;
  nh_param.param("inhibit", inhibit, false);
  if (inhibit)
  {
    ROS_INFO("~spidev/inhibit set: SPI will not be used");
    spi_device_ = NULL;
  }
  else
  {
    ROS_INFO("SPI device: %s", dev_name.c_str());
    spi_device_ = new hailfire_fpga_msgs::SPIDevice(dev_name.c_str());
  }

  int mode;
  if (spi_device_ && nh_param.getParam("mode", mode))
  {
    ROS_INFO("SPI mode: %u", mode);
    spi_device_->setMode(mode);
  }

  bool lsb_first;
  if (spi_device_ && nh_param.getParam("lsb_first", lsb_first))
  {
    ROS_INFO("SPI lsb_first: %s", (lsb_first ? "true" : "false"));
    spi_device_->setLSBFirst(lsb_first);
  }

  int bits_per_word;
  if (spi_device_ && nh_param.getParam("bits_per_word", bits_per_word))
  {
    ROS_INFO("SPI bits_per_word: %u", bits_per_word);
    spi_device_->setBitsPerWord(bits_per_word);
  }

  int max_speed;
  if (spi_device_ && nh_param.getParam("max_speed", max_speed))
  {
    ROS_INFO("SPI max_speed: %u", max_speed);
    spi_device_->setMaxSpeed(max_speed);
  }

  srv1_ = nh_.advertiseService("fpga_proxy_raw", &FPGAProxy::doTransfer, this);
  srv2_ = nh_.advertiseService("fpga_proxy_test", &FPGAProxy::doTestRegisters, this);
  srv3_ = nh_.advertiseService("fpga_proxy", &FPGAProxy::doHighLevel, this);
  ROS_INFO("Ready to proxy requests to FPGA");
}

FPGAProxy::~FPGAProxy()
{
  if (spi_device_)
  {
    delete spi_device_;
  }
}

bool FPGAProxy::doTransfer(hailfire_fpga_msgs::FPGATransfer::Request &req,
                           hailfire_fpga_msgs::FPGATransfer::Response &res)
{
  ROS_INFO("doTransfer");

  ROS_DEBUG_STREAM("Request pairs:" << dump_bytes(req.pairs));

  // Count required number of bytes for KLV-encoded vector
  unsigned int nb_bytes = 0;
  for (unsigned int i = 0; i < req.pairs.size(); ++i)
  {
    nb_bytes += 2 + req.pairs[i].value.size(); // key + length + value
  }

  // KLV-encode given keys and values
  std::vector<uint8_t> tx_rx_bytes;
  tx_rx_bytes.reserve(nb_bytes);

  // Create KLV-encoded vector
  for (unsigned int i = 0; i < req.pairs.size(); ++i)
  {
    // Key
    tx_rx_bytes.push_back(req.pairs[i].key);

    // Length
    uint8_t length = 0xFF & req.pairs[i].value.size();
    tx_rx_bytes.push_back(length);

    // Value
    for (unsigned int j = 0; j < length; ++j)
    {
      tx_rx_bytes.push_back(req.pairs[i].value[j]);
    }
  }

  ROS_DEBUG_STREAM("tx bytes:" << dump_bytes(tx_rx_bytes));

  // Synchronous SPI transfer, using the same vector to store the received
  // bytes (same length so valid memory space)
  if (spi_device_)
  {
    spi_device_->doSyncTransfer(&tx_rx_bytes[0], &tx_rx_bytes[0], nb_bytes);
  }

  ROS_DEBUG_STREAM("rx bytes:" << dump_bytes(tx_rx_bytes));

  // Reconcile received values with the keys and length of the request
  unsigned int rx_offset = 0;
  for (unsigned int i = 0; i < req.pairs.size(); ++i)
  {
    hailfire_fpga_msgs::FPGAKeyValue pair_i;

    // Key
    pair_i.key = req.pairs[i].key;
    ++rx_offset;

    // Length
    uint8_t length = 0xFF & req.pairs[i].value.size();
    ++rx_offset;

    // Value
    pair_i.value.reserve(length);
    for (unsigned int j = 0; j < length; ++j, ++rx_offset)
    {
      pair_i.value.push_back(rx_offset < nb_bytes ? tx_rx_bytes[rx_offset] : 0);
    }

    res.pairs.push_back(pair_i);
  }

  ROS_DEBUG_STREAM("Response pairs:" << dump_bytes(res.pairs));

  return true;
}

bool FPGAProxy::doTestRegisters(hailfire_fpga_msgs::FPGATestRegisters::Request &req,
                                hailfire_fpga_msgs::FPGATestRegisters::Response &res)
{
  ROS_INFO("doTestRegisters");

  // Read fixed uint32 test register
  hailfire_fpga_msgs::FPGAKeyValue read_fixed;
  read_fixed.key = hailfire_fpga_msgs::FPGAKeyValue::TEST_VALUE;
  read_fixed.value.assign(4, 0);

  // Read uint8 test register
  hailfire_fpga_msgs::FPGAKeyValue read_uint8;
  read_uint8.key = hailfire_fpga_msgs::FPGAKeyValue::READ_TEST_REGISTER_UINT8;
  read_uint8.value.assign(1, 0);

  // Read uint16 test register
  hailfire_fpga_msgs::FPGAKeyValue read_uint16;
  read_uint16.key = hailfire_fpga_msgs::FPGAKeyValue::READ_TEST_REGISTER_UINT16;
  read_uint16.value.assign(2, 0);

  // Read uint32 test register
  hailfire_fpga_msgs::FPGAKeyValue read_uint32;
  read_uint32.key = hailfire_fpga_msgs::FPGAKeyValue::READ_TEST_REGISTER_UINT32;
  read_uint32.value.assign(4, 0);

  // Read int8 test register
  hailfire_fpga_msgs::FPGAKeyValue read_int8;
  read_int8.key = hailfire_fpga_msgs::FPGAKeyValue::READ_TEST_REGISTER_INT8;
  read_int8.value.assign(1, 0);

  // Read int16 test register
  hailfire_fpga_msgs::FPGAKeyValue read_int16;
  read_int16.key = hailfire_fpga_msgs::FPGAKeyValue::READ_TEST_REGISTER_INT16;
  read_int16.value.assign(2, 0);

  // Read int32 test register
  hailfire_fpga_msgs::FPGAKeyValue read_int32;
  read_int32.key = hailfire_fpga_msgs::FPGAKeyValue::READ_TEST_REGISTER_INT32;
  read_int32.value.assign(4, 0);

  // Set uint8 test register with new value
  hailfire_fpga_msgs::FPGAKeyValue set_uint8;
  set_uint8.key = hailfire_fpga_msgs::FPGAKeyValue::WRITE_TEST_REGISTER_UINT8;
  set_uint8.value = bytes_from_uint8(req.new_uint8);

  // Set uint16 test register with new value
  hailfire_fpga_msgs::FPGAKeyValue set_uint16;
  set_uint16.key = hailfire_fpga_msgs::FPGAKeyValue::WRITE_TEST_REGISTER_UINT16;
  set_uint16.value = bytes_from_uint16(req.new_uint16);

  // Set uint32 test register with new value
  hailfire_fpga_msgs::FPGAKeyValue set_uint32;
  set_uint32.key = hailfire_fpga_msgs::FPGAKeyValue::WRITE_TEST_REGISTER_UINT32;
  set_uint32.value = bytes_from_uint32(req.new_uint32);

  // Set int8 test register with new value
  hailfire_fpga_msgs::FPGAKeyValue set_int8;
  set_int8.key = hailfire_fpga_msgs::FPGAKeyValue::WRITE_TEST_REGISTER_INT8;
  set_int8.value = bytes_from_int8(req.new_int8);

  // Set int16 test register with new value
  hailfire_fpga_msgs::FPGAKeyValue set_int16;
  set_int16.key = hailfire_fpga_msgs::FPGAKeyValue::WRITE_TEST_REGISTER_INT16;
  set_int16.value = bytes_from_int16(req.new_int16);

  // Set int32 test register with new value
  hailfire_fpga_msgs::FPGAKeyValue set_int32;
  set_int32.key = hailfire_fpga_msgs::FPGAKeyValue::WRITE_TEST_REGISTER_INT32;
  set_int32.value = bytes_from_int32(req.new_int32);

  // Prepare transfer
  hailfire_fpga_msgs::FPGATransfer tr;
  tr.request.pairs.reserve(13);
  tr.request.pairs.push_back(read_fixed);
  tr.request.pairs.push_back(read_uint8);
  tr.request.pairs.push_back(read_uint16);
  tr.request.pairs.push_back(read_uint32);
  tr.request.pairs.push_back(read_int8);
  tr.request.pairs.push_back(read_int16);
  tr.request.pairs.push_back(read_int32);
  tr.request.pairs.push_back(set_uint8);
  tr.request.pairs.push_back(set_uint16);
  tr.request.pairs.push_back(set_uint32);
  tr.request.pairs.push_back(set_int8);
  tr.request.pairs.push_back(set_int16);
  tr.request.pairs.push_back(set_int32);

  if (!doTransfer(tr.request, tr.response))
  {
    return false;
  }

  // Get the responses
  res.fixed_val = uint32_from_bytes(tr.response.pairs[0].value);
  res.prev_uint8 = uint8_from_bytes(tr.response.pairs[1].value);
  res.prev_uint16 = uint16_from_bytes(tr.response.pairs[2].value);
  res.prev_uint32 = uint32_from_bytes(tr.response.pairs[3].value);
  res.prev_int8 = int8_from_bytes(tr.response.pairs[4].value);
  res.prev_int16 = int16_from_bytes(tr.response.pairs[5].value);
  res.prev_int32 = int32_from_bytes(tr.response.pairs[6].value);

  return true;
}

bool FPGAProxy::doHighLevel(hailfire_fpga_msgs::FPGA::Request &req,
                            hailfire_fpga_msgs::FPGA::Response &res)
{
  ROS_INFO("doHighLevel");
  unsigned int i;

  hailfire_fpga_msgs::FPGATransfer tr;

  for (i = 0; i < req.odometers.size(); ++i)
  {
    // Read int32 count
    hailfire_fpga_msgs::FPGAKeyValue read_count;
    read_count.key = hailfire_fpga_msgs::FPGAKeyValue::ODOMETER_COUNT_BASE + req.odometers[i].key;
    read_count.value.assign(4, 0);
    tr.request.pairs.push_back(read_count);

    // Read int32 speed
    hailfire_fpga_msgs::FPGAKeyValue read_speed;
    read_speed.key = hailfire_fpga_msgs::FPGAKeyValue::ODOMETER_SPEED_BASE + req.odometers[i].key;
    read_speed.value.assign(4, 0);
    tr.request.pairs.push_back(read_speed);
  }

  for (i = 0; i < req.ext_ports.size(); ++i)
  {
    // Read uint8 port
    hailfire_fpga_msgs::FPGAKeyValue read_port;
    read_port.key = hailfire_fpga_msgs::FPGAKeyValue::EXT_PORT_VALUE_BASE + req.ext_ports[i].key;
    read_port.value.assign(1, 0);
    tr.request.pairs.push_back(read_port);
  }

  for (i = 0; i < req.leds.size(); ++i)
  {
    // Set bool led
    hailfire_fpga_msgs::FPGAKeyValue set_led;
    set_led.key = hailfire_fpga_msgs::FPGAKeyValue::LED_BASE + req.leds[i].key;
    set_led.value = bytes_from_uint8(req.leds[i].on ? 1 : 0);
    tr.request.pairs.push_back(set_led);
  }

  for (i = 0; i < req.motors.size(); ++i)
  {
    // Set int16 motor speed
    hailfire_fpga_msgs::FPGAKeyValue set_motor;
    set_motor.key = hailfire_fpga_msgs::FPGAKeyValue::MOTOR_SPEED_BASE + req.motors[i].key;
    set_motor.value = bytes_from_int16(req.motors[i].speed);
    tr.request.pairs.push_back(set_motor);
  }

  for (i = 0; i < req.servos.size(); ++i)
  {
    // Set uint16 servo consign speed
    hailfire_fpga_msgs::FPGAKeyValue set_servo;
    set_servo.key = hailfire_fpga_msgs::FPGAKeyValue::SERVO_CONSIGN_BASE + req.servos[i].key;
    set_servo.value = bytes_from_uint16(req.servos[i].consign);
    tr.request.pairs.push_back(set_servo);
  }

  if (!doTransfer(tr.request, tr.response))
  {
    return false;
  }

  unsigned int pair_i = 0;
  for (i = 0; i < req.odometers.size(); ++i)
  {
    hailfire_fpga_msgs::Odometer odometer;
    odometer.key = req.odometers[i].key;
    odometer.count = int32_from_bytes(tr.response.pairs[pair_i++].value);
    odometer.speed = int32_from_bytes(tr.response.pairs[pair_i++].value);
    res.odometers.push_back(odometer);
  }

  for (i = 0; i < req.ext_ports.size(); ++i)
  {
    hailfire_fpga_msgs::ExtPort ext_port;
    ext_port.key = req.ext_ports[i].key;
    uint8_t port = uint8_from_bytes(tr.response.pairs[pair_i++].value);
    ext_port.pins[0] = (port & 0x01) ? true : false;
    ext_port.pins[1] = (port & 0x02) ? true : false;
    ext_port.pins[2] = (port & 0x04) ? true : false;
    ext_port.pins[3] = (port & 0x08) ? true : false;
    ext_port.pins[4] = (port & 0x10) ? true : false;
    ext_port.pins[5] = (port & 0x20) ? true : false;
    ext_port.pins[6] = (port & 0x40) ? true : false;
    ext_port.pins[7] = (port & 0x80) ? true : false;
    res.ext_ports.push_back(ext_port);
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fpga_proxy");
  FPGAProxy fpga_proxy;
  ros::spin();

  return 0;
}

std::string dump_bytes(std::vector<uint8_t> const &bytes)
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

std::string dump_bytes(std::vector<hailfire_fpga_msgs::FPGAKeyValue> const &bytes)
{
  std::stringstream ss;
  ss << std::setfill('0');
  ss << std::hex;
  for (unsigned int i = 0; i < bytes.size(); ++i)
  {
    ss << std::endl << "  key: 0x" << std::setw(2) << +bytes[i].key;
    ss << std::endl << "  value:";
    for (unsigned int j = 0; j < bytes[i].value.size(); ++j)
    {
      ss << std::endl << "    0x" << std::setw(2) << +bytes[i].value[j];
    }
  }
  return ss.str();
}

uint8_t uint8_from_bytes(std::vector<uint8_t> const &bytes)
{
  uint8_t value = bytes[0];
  return value;
}

uint16_t uint16_from_bytes(std::vector<uint8_t> const &bytes)
{
  uint16_t value = (bytes[0] << 8) |
                   (bytes[1] << 0);
  return value;
}

uint32_t uint32_from_bytes(std::vector<uint8_t> const &bytes)
{
  uint32_t value = (bytes[0] << 24) |
                   (bytes[1] << 16) |
                   (bytes[2] << 8) |
                   (bytes[3] << 0);
  return value;
}

int8_t int8_from_bytes(std::vector<uint8_t> const &bytes)
{
  int8_t value = (int8_t) uint8_from_bytes(bytes);
  return value;
}

int16_t int16_from_bytes(std::vector<uint8_t> const &bytes)
{
  int16_t value = (int16_t) uint16_from_bytes(bytes);
  return value;
}

int32_t int32_from_bytes(std::vector<uint8_t> const &bytes)
{
  int32_t value = (int32_t) uint32_from_bytes(bytes);
  return value;
}

std::vector<uint8_t> bytes_from_uint8(uint8_t const &value)
{
  std::vector<uint8_t> bytes (1, 0);
  bytes[0] = value;
  return bytes;
}

std::vector<uint8_t> bytes_from_uint16(uint16_t const &value)
{
  std::vector<uint8_t> bytes (2, 0);
  bytes[0] = (value >> 8) & 0xFF;
  bytes[1] = (value >> 0) & 0xFF;
  return bytes;
}

std::vector<uint8_t> bytes_from_uint32(uint32_t const &value)
{
  std::vector<uint8_t> bytes (4, 0);
  bytes[0] = (value >> 24) & 0xFF;
  bytes[1] = (value >> 16) & 0xFF;
  bytes[2] = (value >> 8) & 0xFF;
  bytes[3] = (value >> 0) & 0xFF;
  return bytes;
}

std::vector<uint8_t> bytes_from_int8(int8_t const &value)
{
  uint8_t uvalue = (uint8_t) value;
  return bytes_from_uint8(uvalue);
}

std::vector<uint8_t> bytes_from_int16(int16_t const &value)
{
  uint16_t uvalue = (uint16_t) value;
  return bytes_from_uint16(uvalue);
}

std::vector<uint8_t> bytes_from_int32(int32_t const &value)
{
  uint32_t uvalue = (uint32_t) value;
  return bytes_from_uint32(uvalue);
}

