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

#include "ros/ros.h"
#include "hailfire_drivers/spi_device.h"
#include "hailfire_drivers/FPGAKeyValue.h"
#include "hailfire_drivers/FPGATransfer.h"
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

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

bool do_sync_transfer(hailfire_drivers::FPGATransfer::Request &req,
                      hailfire_drivers::FPGATransfer::Response &res)
{
  ROS_INFO("do_sync_transfer");

  ROS_DEBUG_STREAM("Request pairs:" << dump_bytes(req.pairs));

  // Count required number of bytes for KLV-encoded vector
  unsigned int nb_bytes = 0;
  for (unsigned int i = 0; i < req.pairs.size(); ++i)
  {
    nb_bytes += 2 + req.pairs[i].value.size(); // key + length + value
  }

  // KLV-encode given keys and values
  std::vector<uint8_t> tx_bytes;
  tx_bytes.reserve(nb_bytes);

  // Create KLV-encoded vector
  for (unsigned int i = 0; i < req.pairs.size(); ++i)
  {
    // Key
    tx_bytes.push_back(req.pairs[i].key);

    // Length
    uint8_t length = 0xFF & req.pairs[i].value.size();
    tx_bytes.push_back(length);

    // Value
    for (unsigned int j = 0; j < length; ++j)
    {
      tx_bytes.push_back(req.pairs[i].value[j]);
    }
  }

  ROS_DEBUG_STREAM("tx bytes:" << dump_bytes(tx_bytes));

  std::vector<uint8_t> rx_bytes = tx_bytes; // FIXME

  ROS_DEBUG_STREAM("rx bytes:" << dump_bytes(rx_bytes));

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
      pair_i.value.push_back(rx_offset < nb_bytes ? rx_bytes[rx_offset] : 0);
    }

    res.pairs.push_back(pair_i);
  }

  ROS_DEBUG_STREAM("Response pairs:" << dump_bytes(res.pairs));

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fpga_proxy");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("fpga_proxy", do_sync_transfer);
  ROS_INFO("Ready to proxy requests to FPGA");
  ros::spin();

  return 0;
}
