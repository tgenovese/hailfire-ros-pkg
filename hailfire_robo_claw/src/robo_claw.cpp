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

#include "hailfire_robo_claw/robo_claw.h"
#include <ros/assert.h>

namespace hailfire_robo_claw
{

RoboClaw::RoboClaw(const char *dev_name, int baud_rate, uint8_t address) : address_(address)
{
  dev_.open(dev_name, baud_rate);
}

RoboClaw::~RoboClaw()
{
  dev_.close();
}

void RoboClaw::driveMotor(uint8_t motor, int8_t speed)
{
  uint8_t command = (motor == 1 ? (speed >= 0 ? 0 : 1) : (speed >= 0 ? 4 : 5));
  uint8_t uspeed  = (speed >= 0 ? speed : -speed);

  sendUint8(command, uspeed);
}

void RoboClaw::moveForwardOrBackward(int8_t speed)
{
  uint8_t command = (speed >= 0 ? 8 : 9);
  uint8_t uspeed  = (speed >= 0 ? speed : -speed);

  sendUint8(command, uspeed);
}

void RoboClaw::turnLeftOrRight(int8_t speed)
{
  uint8_t command = (speed >= 0 ? 11 : 10);
  uint8_t uspeed  = (speed >= 0 ? speed : -speed);

  sendUint8(command, uspeed);
}

bool RoboClaw::readEncoderCount(uint8_t encoder, int32_t *count, uint8_t *status)
{
  uint8_t command = (encoder == 1 ? 16 : 17);
  uint8_t bytes[6];
  if (readBytes(command, bytes, 6))
  {
    uint32_t ucount = (bytes[0] << 24) |
                      (bytes[1] << 16) |
                      (bytes[2] << 8) |
                      (bytes[3] << 0);
    *count = (int32_t) ucount;
    *status = bytes[4];
    return true;
  }
  return false;
}

bool RoboClaw::readEncoderSpeed(uint8_t encoder, int32_t *speed)
{
  uint8_t command = (encoder == 1 ? 18 : 19);
  uint8_t bytes[6];
  if (readBytes(command, bytes, 6))
  {
    uint32_t uspeed = (bytes[0] << 24) |
                      (bytes[1] << 16) |
                      (bytes[2] << 8) |
                      (bytes[3] << 0);
    uspeed &= 0x7FFFFFFF;
    *speed = (bytes[4] == 0 ? uspeed : -uspeed);
    return true;
  }
  return false;
}

void RoboClaw::resetEncoders()
{
  uint8_t command = 20;

  uint8_t bytes[3];
  bytes[0] = address_;
  bytes[1] = command;
  bytes[2] = computeCRC(bytes, 2);

  dev_.write((char *)bytes, 3);
}

void RoboClaw::setPIDConstants(uint8_t motor, uint32_t p, uint32_t i, uint32_t d, uint32_t qpps)
{
  uint8_t command = (motor == 1 ? 28 : 29);

  uint8_t bytes[19];
  bytes[0] = address_;
  bytes[1] = command;

  bytes[2] = (d >> 24) & 0xFF;
  bytes[3] = (d >> 16) & 0xFF;
  bytes[4] = (d >> 8) & 0xFF;
  bytes[5] = (d >> 0) & 0xFF;

  bytes[6] = (p >> 24) & 0xFF;
  bytes[7] = (p >> 16) & 0xFF;
  bytes[8] = (p >> 8) & 0xFF;
  bytes[9] = (p >> 0) & 0xFF;

  bytes[10] = (i >> 24) & 0xFF;
  bytes[11] = (i >> 16) & 0xFF;
  bytes[12] = (i >> 8) & 0xFF;
  bytes[13] = (i >> 0) & 0xFF;

  bytes[14] = (qpps >> 24) & 0xFF;
  bytes[15] = (qpps >> 16) & 0xFF;
  bytes[16] = (qpps >> 8) & 0xFF;
  bytes[17] = (qpps >> 0) & 0xFF;

  bytes[18] = computeCRC(bytes, 18);

  dev_.write((char *)bytes, 19);
}

bool RoboClaw::readCurrentSpeed(uint8_t motor, int32_t *speed)
{
  uint8_t command = (motor == 1 ? 30 : 31);
  uint8_t bytes[5];
  if (readBytes(command, bytes, 5))
  {
    uint32_t uspeed = (bytes[0] << 24) |
                      (bytes[1] << 16) |
                      (bytes[2] << 8) |
                      (bytes[3] << 0);
    *speed = (int32_t) uspeed;
    return true;
  }
  return false;
}

void RoboClaw::driveMotorWithDutyCycle(uint8_t motor, int16_t duty_cycle)
{
  uint8_t command = (motor == 1 ? 32 : 33);
  uint16_t uduty = duty_cycle;

  uint8_t bytes[5];
  bytes[0] = address_;
  bytes[1] = command;
  bytes[2] = (uduty >> 8) & 0xFF;
  bytes[3] = (uduty >> 0) & 0xFF;
  bytes[4] = computeCRC(bytes, 4);

  dev_.write((char *)bytes, 5);
}

void RoboClaw::driveMotorsWithDutyCycle(int16_t duty_m1, int16_t duty_m2)
{
  uint8_t command = 34;
  uint16_t uduty_m1 = duty_m1;
  uint16_t uduty_m2 = duty_m2;

  uint8_t bytes[7];
  bytes[0] = address_;
  bytes[1] = command;
  bytes[2] = (uduty_m1 >> 8) & 0xFF;
  bytes[3] = (uduty_m1 >> 0) & 0xFF;
  bytes[4] = (uduty_m2 >> 8) & 0xFF;
  bytes[5] = (uduty_m2 >> 0) & 0xFF;
  bytes[6] = computeCRC(bytes, 6);

  dev_.write((char *)bytes, 7);
}

void RoboClaw::driveMotorWithSpeed(uint8_t motor, int32_t speed)
{
  uint8_t command = (motor == 1 ? 35 : 36);
  uint16_t uspeed = speed;

  uint8_t bytes[7];
  bytes[0] = address_;
  bytes[1] = command;
  bytes[2] = (uspeed >> 24) & 0xFF;
  bytes[3] = (uspeed >> 16) & 0xFF;
  bytes[4] = (uspeed >> 8) & 0xFF;
  bytes[5] = (uspeed >> 0) & 0xFF;
  bytes[6] = computeCRC(bytes, 6);

  dev_.write((char *)bytes, 7);
}

void RoboClaw::driveMotorsWithSpeed(int32_t speed_m1, int32_t speed_m2)
{
  uint8_t command = 37;
  uint16_t uspeed_m1 = speed_m1;
  uint16_t uspeed_m2 = speed_m2;

  uint8_t bytes[11];
  bytes[0] = address_;
  bytes[1] = command;

  bytes[2] = (uspeed_m1 >> 24) & 0xFF;
  bytes[3] = (uspeed_m1 >> 16) & 0xFF;
  bytes[4] = (uspeed_m1 >> 8) & 0xFF;
  bytes[5] = (uspeed_m1 >> 0) & 0xFF;

  bytes[6] = (uspeed_m2 >> 24) & 0xFF;
  bytes[7] = (uspeed_m2 >> 16) & 0xFF;
  bytes[8] = (uspeed_m2 >> 8) & 0xFF;
  bytes[9] = (uspeed_m2 >> 0) & 0xFF;

  bytes[10] = computeCRC(bytes, 10);

  dev_.write((char *)bytes, 11);
}

void RoboClaw::driveMotorWithSpeedAndAcceleration(uint8_t motor, int32_t speed, uint32_t accel)
{
  uint8_t command = (motor == 1 ? 38 : 39);
  uint16_t uspeed = speed;

  uint8_t bytes[11];
  bytes[0] = address_;
  bytes[1] = command;

  bytes[2] = (accel >> 24) & 0xFF;
  bytes[3] = (accel >> 16) & 0xFF;
  bytes[4] = (accel >> 8) & 0xFF;
  bytes[5] = (accel >> 0) & 0xFF;

  bytes[6] = (uspeed >> 24) & 0xFF;
  bytes[7] = (uspeed >> 16) & 0xFF;
  bytes[8] = (uspeed >> 8) & 0xFF;
  bytes[9] = (uspeed >> 0) & 0xFF;

  bytes[10] = computeCRC(bytes, 10);

  dev_.write((char *)bytes, 11);
}

void RoboClaw::driveMotorsWithSpeedAndAcceleration(int32_t speed_m1, int32_t speed_m2, uint32_t accel)
{
  uint8_t command = 40;
  uint16_t uspeed_m1 = speed_m1;
  uint16_t uspeed_m2 = speed_m2;

  uint8_t bytes[15];
  bytes[0] = address_;
  bytes[1] = command;

  bytes[2] = (accel >> 24) & 0xFF;
  bytes[3] = (accel >> 16) & 0xFF;
  bytes[4] = (accel >> 8) & 0xFF;
  bytes[5] = (accel >> 0) & 0xFF;

  bytes[6] = (uspeed_m1 >> 24) & 0xFF;
  bytes[7] = (uspeed_m1 >> 16) & 0xFF;
  bytes[8] = (uspeed_m1 >> 8) & 0xFF;
  bytes[9] = (uspeed_m1 >> 0) & 0xFF;

  bytes[10] = (uspeed_m2 >> 24) & 0xFF;
  bytes[11] = (uspeed_m2 >> 16) & 0xFF;
  bytes[12] = (uspeed_m2 >> 8) & 0xFF;
  bytes[13] = (uspeed_m2 >> 0) & 0xFF;

  bytes[14] = computeCRC(bytes, 14);

  dev_.write((char *)bytes, 15);
}

void RoboClaw::setPWMResolution(uint8_t resolution)
{
  uint8_t command = 48;

  resolution = (resolution > 14 ? 14 : (resolution < 8) ? 8 : resolution);
  resolution -= 8;

  sendUint8(command, resolution);
}

void RoboClaw::setMinimumMainVoltage(float voltage)
{
  uint8_t command = 2;

  voltage = (voltage > 30 ? 30 : (voltage < 6) ? 6 : voltage);
  uint8_t uvoltage = (voltage - 6) * 5;

  sendUint8(command, uvoltage);
}

void RoboClaw::setMaximumMainVoltage(float voltage)
{
  uint8_t command = 3;

  voltage = (voltage > 30 ? 30 : (voltage < 0) ? 0 : voltage);
  uint8_t uvoltage = voltage * 5.12;

  sendUint8(command, uvoltage);
}

bool RoboClaw::readMainVoltageLevel(float *voltage)
{
  uint8_t command = 24;
  uint16_t value = 0;
  if (readUint16(command, &value))
  {
    *voltage = value / 10.0;
    return true;
  }
  return false;
}

void RoboClaw::setMinimumLogicVoltage(float voltage)
{
  uint8_t command = 26;

  voltage = (voltage > 28 ? 28 : (voltage < 6) ? 6 : voltage);
  uint8_t uvoltage = (voltage - 6) * 5;

  sendUint8(command, uvoltage);
}

void RoboClaw::setMaximumLogicVoltage(float voltage)
{
  uint8_t command = 27;

  voltage = (voltage > 28 ? 28 : (voltage < 0) ? 0 : voltage);
  uint8_t uvoltage = voltage * 5.12;

  sendUint8(command, uvoltage);
}

bool RoboClaw::readLogicVoltageLevel(float *voltage)
{
  uint8_t command = 25;
  uint16_t value = 0;
  if (readUint16(command, &value))
  {
    *voltage = value / 10.0;
    return true;
  }
  return false;
}

bool RoboClaw::readFirmwareVersion(char *version_string)
{
  uint8_t command = 21;

  uint8_t req[2];
  req[0] = address_;
  req[1] = command;
  dev_.write((char *)req, 2);

  dev_.read(version_string, 32);
  return true;
}

void RoboClaw::sendUint8(uint8_t command, uint8_t value)
{
  uint8_t bytes[4];
  bytes[0] = address_;
  bytes[1] = command;
  bytes[2] = value;
  bytes[3] = computeCRC(bytes, 3);

  dev_.write((char *)bytes, 4);
}

bool RoboClaw::readBytes(uint8_t command, uint8_t *bytes, unsigned int nb_bytes)
{
  uint8_t req[2];
  req[0] = address_;
  req[1] = command;
  dev_.write((char *)req, 2);

  dev_.readBytes((char *)bytes, nb_bytes);

  return checkCRC(bytes, nb_bytes - 1, bytes[nb_bytes - 1]);
}

bool RoboClaw::readUint16(uint8_t command, uint16_t *value)
{
  uint8_t bytes[3];
  if (readBytes(command, bytes, 3))
  {
    *value = (bytes[0] << 8) | bytes[1];
    return true;
  }
  return false;
}

uint8_t RoboClaw::computeCRC(uint8_t *bytes, unsigned int nb_bytes)
{
  uint8_t crc = 0;
  for (unsigned int i = 0; i < nb_bytes; ++i)
  {
    crc += bytes[i];
  }
  crc &= 0x7F;

  return crc;
}

bool RoboClaw::checkCRC(uint8_t *bytes, unsigned int nb_bytes, uint8_t crc)
{
  return computeCRC(bytes, nb_bytes) == crc ? true : false;
}

}
