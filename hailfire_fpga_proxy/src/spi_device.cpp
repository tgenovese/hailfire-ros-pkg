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

#include "hailfire_fpga_proxy/spi_device.h"
#include <ros/assert.h>

#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>

namespace hailfire_fpga_proxy
{

SPIDevice::SPIDevice(const char *dev_name)
{
  fd_ = open(dev_name, O_RDWR);
  ROS_ASSERT(fd_ >= 0);
}

SPIDevice::~SPIDevice()
{
  close(fd_);
}

uint8_t SPIDevice::getMode()
{
  uint8_t mode;
  int ret = ioctl(fd_, SPI_IOC_RD_MODE, &mode);
  ROS_ASSERT(ret != -1);
  return mode;
}

void SPIDevice::setMode(uint8_t mode)
{
  int ret = ioctl(fd_, SPI_IOC_WR_MODE, &mode);
  ROS_ASSERT(ret != -1);
}

bool SPIDevice::getLSBFirst()
{
  uint8_t lsb_first;
  int ret = ioctl(fd_, SPI_IOC_RD_LSB_FIRST, &lsb_first);
  ROS_ASSERT(ret != -1);
  return (lsb_first == SPI_LSB_FIRST ? true : false);
}

void SPIDevice::setLSBFirst(bool lsb_first)
{
  uint8_t lsb_first_int = (lsb_first ? SPI_LSB_FIRST : 0);
  int ret = ioctl(fd_, SPI_IOC_WR_LSB_FIRST, &lsb_first_int);
  ROS_ASSERT(ret != -1);
}

uint8_t SPIDevice::getBitsPerWord()
{
  uint8_t bits_per_word;
  int ret = ioctl(fd_, SPI_IOC_RD_BITS_PER_WORD, &bits_per_word);
  ROS_ASSERT(ret != -1);
  return bits_per_word;
}

void SPIDevice::setBitsPerWord(uint8_t bits_per_word)
{
  int ret = ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word);
  ROS_ASSERT(ret != -1);
}

uint32_t SPIDevice::getMaxSpeed()
{
  uint32_t max_speed;
  int ret = ioctl(fd_, SPI_IOC_RD_MAX_SPEED_HZ, &max_speed);
  ROS_ASSERT(ret != -1);
  return max_speed;
}

void SPIDevice::setMaxSpeed(uint32_t max_speed)
{
  int ret = ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &max_speed);
  ROS_ASSERT(ret != -1);
}

void SPIDevice::doSyncTransfer(uint8_t *tx, uint8_t *rx, unsigned int len)
{
  struct spi_ioc_transfer tr;
  memset(&tr, 0, sizeof(tr));
  tr.tx_buf = (unsigned long)tx;
  tr.rx_buf = (unsigned long)rx;
  tr.len = len;

  int ret = ioctl(fd_, SPI_IOC_MESSAGE(1), &tr);
  ROS_ASSERT(ret != -1);
}

}
