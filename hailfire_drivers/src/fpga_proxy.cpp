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
#include "hailfire_drivers/spi_device.h"
#include "hailfire_drivers/FPGAKeyValue.h"
#include "hailfire_drivers/FPGATransfer.h"

/**
 * @brief Returns an hexadecimal dump of the given byte array.
 */
std::string dump_bytes(std::vector<uint8_t> const &bytes);

/**
 * @brief Returns an hexadecimal dump of the given FPGAKeyValue array.
 */
std::string dump_bytes(std::vector<hailfire_drivers::FPGAKeyValue> const &bytes);

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
   * It transforms the keys and values in a format accepted by the FPGA (Key,
   * Length, Value encoded byte array), sends it to the FPGA via SPI and
   * transforms the FPGA response back to a ROS format.
   */
  bool doTransfer(hailfire_drivers::FPGATransfer::Request &req,
                  hailfire_drivers::FPGATransfer::Response &res);

  ros::NodeHandle nh_;                      /**< The ROS node handle */
  ros::ServiceServer srv_;                  /**< The ROS service handle */
  hailfire_drivers::SPIDevice *spi_device_; /**< The SPI device instance */
};

FPGAProxy::FPGAProxy()
{
  ros::NodeHandle nh_param("~spidev");

  std::string dev_name;
  nh_param.param(std::string("dev_name"), dev_name, std::string("/dev/spidev1.0"));
  ROS_INFO("SPI device: %s", dev_name.c_str());

  spi_device_ = new hailfire_drivers::SPIDevice(dev_name.c_str());

  int mode;
  if (nh_param.getParam("mode", mode))
  {
    ROS_INFO("SPI mode: %u", mode);
    spi_device_->setMode(mode);
  }

  bool lsb_first;
  if (nh_param.getParam("lsb_first", lsb_first))
  {
    ROS_INFO("SPI lsb_first: %s", (lsb_first ? "true" : "false"));
    spi_device_->setLSBFirst(lsb_first);
  }

  int bits_per_word;
  if (nh_param.getParam("bits_per_word", bits_per_word))
  {
    ROS_INFO("SPI bits_per_word: %u", bits_per_word);
    spi_device_->setBitsPerWord(bits_per_word);
  }

  int max_speed;
  if (nh_param.getParam("max_speed", max_speed))
  {
    ROS_INFO("SPI max_speed: %u", max_speed);
    spi_device_->setMaxSpeed(max_speed);
  }

  srv_ = nh_.advertiseService("fpga_proxy", &FPGAProxy::doTransfer, this);
  ROS_INFO("Ready to proxy requests to FPGA");
}

FPGAProxy::~FPGAProxy()
{
  delete spi_device_;
}

bool FPGAProxy::doTransfer(hailfire_drivers::FPGATransfer::Request &req,
                           hailfire_drivers::FPGATransfer::Response &res)
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
  spi_device_->doSyncTransfer(&tx_rx_bytes[0], &tx_rx_bytes[0], nb_bytes);

  ROS_DEBUG_STREAM("rx bytes:" << dump_bytes(tx_rx_bytes));

  // Reconcile received values with the keys and length of the request
  unsigned int rx_offset = 0;
  for (unsigned int i = 0; i < req.pairs.size(); ++i)
  {
    hailfire_drivers::FPGAKeyValue pair_i;

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

std::string dump_bytes(std::vector<hailfire_drivers::FPGAKeyValue> const &bytes)
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
