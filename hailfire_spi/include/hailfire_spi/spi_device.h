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

#ifndef HAILFIRE_FPGA_PROXY_SPI_DEVICE_H
#define HAILFIRE_FPGA_PROXY_SPI_DEVICE_H

#include <linux/spi/spidev.h>
#include <stdint.h>

namespace hailfire_fpga_proxy
{

/**
 * @class SPIDevice
 * @brief User space interface of spidev kernel module.
 *
 * This class provides an interface for the user space API of the spidev kernel
 * module in C++, thus hiding the gory details involved.
 *
 */
class SPIDevice
{
public:

  /**
   * @brief Constructor
   * @param dev_name The name of the spidev device file (eg: "/dev/spidev1.0")
   */
  SPIDevice(const char *dev_name);

  /**
   * @brief Destructor: cleans up
   */
  ~SPIDevice();

  /**
   * @brief Reads the SPI mode
   * @return The SPI mode (SPI_MODE_0..SPI_MODE_3)
   */
  uint8_t getMode();

  /**
   * @brief Sets the SPI mode
   * @param mode The SPI mode (SPI_MODE_0..SPI_MODE_3)
   */
  void setMode(uint8_t mode);

  /**
   * @brief Reads the SPI bit justification
   * @return True if the least significant bit is sent first
   */
  bool getLSBFirst();

  /**
   * @brief Sets the SPI bit justification
   * @param lsb_first True to send the least significant bit first
   */
  void setLSBFirst(bool lsb_first);

  /**
   * @brief Reads the SPI device word length
   * @return The SPI device word length (1..N)
   */
  uint8_t getBitsPerWord();

  /**
   * @brief Sets the SPI device word length
   * @param bits_per_word The SPI device word length (1..N)
   */
  void setBitsPerWord(uint8_t bits_per_word);

  /**
   * @brief Reads the SPI device default max speed
   * @return The SPI device default max speed (Hz)
   */
  uint32_t getMaxSpeed();

  /**
   * @brief Sets the SPI device default max speed
   * @param max_speed The SPI device default max speed (Hz)
   */
  void setMaxSpeed(uint32_t max_speed);

  /**
   * @brief Does a synchronous full-duplex transfer of arbitrary length.
   * @param tx Array of bytes to be sent (pointer to first element)
   * @param rx Array of bytes to fill with received bytes
   * @param len Number of bytes to send and receive
   */
  void doSyncTransfer(uint8_t *tx, uint8_t *rx, unsigned int len);

private:

  int fd_; /**< The file descriptor for the spidev device file */

};

}

#endif
