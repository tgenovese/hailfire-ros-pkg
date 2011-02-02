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

#ifndef HAILFIRE_ROBO_CLAW_ROBO_CLAW_H
#define HAILFIRE_ROBO_CLAW_ROBO_CLAW_H

#include <cereal_port/CerealPort.h>
#include <stdint.h>

namespace hailfire_robo_claw
{

/**
 * @class RoboClaw
 * @brief C++ class for Basic Micro's Robo Claw motor controller.
 *
 * This class provides an interface for the serial API of Robo Claw motor
 * controller board in C++, thus hiding the gory details involved.
 *
 */
class RoboClaw
{
public:

  /**
   * @brief Constructor
   * @param dev_name  The name of the serial device file (eg: "/dev/tty.USB1")
   * @param baud_rate The baud rate to use, configured on the board with DIP
   *                  switches. Valid baud rates are: 2400, 9600, 19200, 38400.
   * @param address   The address to use, configured on the board with DIP
   *                  switches (multiple Robo Claw boards could share the same
   *                  bus, although we don't support this). Valid addresses
   *                  are: 128 to 135 (included).
   */
  RoboClaw(const char *dev_name, int baud_rate, uint8_t address);

  /**
   * @brief Destructor: cleans up
   */
  ~RoboClaw();

  /**
   * @brief Drive a motor forward or backward.
   * @param motor Motor number (1 or 2)
   * @param speed Signed speed consign.
   *              - +127: full speed forward
   *              -  +64: about half speed forward
   *              -    0: full stop
   *              -  -64: about half speed backward
   *              - -127: full speed backward
   *
   * For motor 1, this is a combination of commands 0 and 1, which gives an
   * 8-bit version of command 6.
   *
   * For motor 2, this is a combination of commands 4 and 5, which gives an
   * 8-bit version of command 7.
   */
  void driveMotor(uint8_t motor, int8_t speed);

  /**
   * @brief Drive both motors to move forward or backward.
   * @param speed Signed speed consign.
   *              - +127: full speed forward
   *              -  +64: about half speed forward
   *              -    0: full stop
   *              -  -64: about half speed backward
   *              - -127: full speed backward
   *
   * This is a combination of commands 8 and 9, which gives an 8-bit version
   * of command 12.
   */
  void moveForwardOrBackward(int8_t speed);

  /**
   * @brief Drive both motors to turn left or right.
   * @param speed Signed speed consign.
   *              - +127: full speed turn left
   *              -  +64: about half speed turn left
   *              -    0: full stop
   *              -  -64: about half speed turn right
   *              - -127: full speed turn right
   *
   * This is a combination of commands 10 and 11, which gives an 8-bit version
   * of command 13.
   */
  void turnLeftOrRight(int8_t speed);

  /**
   * @brief Reads the current count of a quadrature encoder.
   * @param encoder Encoder number (1 or 2).
   * @param count   Output count.
   * @param status  Output status byte.
   *                - bit 0: 1 if underflow occurred
   *                - bit 1: current direction 0=forward 1=backward
   *                - bit 2: 1 if overflow occurred
   *                - bit 7: 1 if encoder is OK
   * @return true if the count was read correctly.
   *
   * This is command 16 (encoder 1) and 17 (encoder 2), with a more friendly
   * signed count output.
   */
  bool readEncoderCount(uint8_t encoder, int32_t *count, uint8_t *status);

  /**
   * @brief Reads the current speed of a quadrature encoder.
   * @param encoder Encoder number (1 or 2).
   * @param speed   Output speed in pulses per second.
   * @return true if the speed was read correctly.
   *
   * This is command 18 (encoder 1) and 19 (encoder 2), with a more friendly
   * signed speed output.
   */
  bool readEncoderSpeed(uint8_t encoder, int32_t *speed);

  /**
   * @brief Resets both quadrature encoder counters to zero (command 20).
   */
  void resetEncoders();

  /**
   * @brief Sets the PID constants for a motor (commands 28 and 29).
   * @param motor Motor number (1 or 2)
   * @param p     Proportianal gain.
   * @param i     Integral gain.
   * @param d     Derivate gain.
   * @param qpps  Speed of the encoder when the motor is at 100% power,
   *              in quad pulses per second (QPPS).
   */
  void setPIDConstants(uint8_t motor, uint32_t p, uint32_t i, uint32_t d, uint32_t qpps);

  /**
   * @brief Reads the current speed of a motor (commands 30 and 31).
   * @param motor Motor number (1 or 2)
   * @param speed Output speed in pulses per 125th of a second.
   * @return true if the speed was read correctly.
   */
  bool readCurrentSpeed(uint8_t motor, int32_t *speed);

  /**
   * @brief Drive a motor with a signed duty cycle (commands 32 and 33).
   * @param motor       Motor number (1 or 2)
   * @param duty_cycle  Speed as a duty cycle (default resolution is 8 bits).
   */
  void driveMotorWithDutyCycle(uint8_t motor, int16_t duty_cycle);

  /**
   * @brief Drive both motors with signed duty cycles (command 34).
   * @param duty_m1 M1 speed as a duty cycle (default resolution is 8 bits).
   * @param duty_m2 M2 speed as a duty cycle (default resolution is 8 bits).
   */
  void driveMotorsWithDutyCycle(int16_t duty_m1, int16_t duty_m2);

  /**
   * @brief Drive a motor with a signed speed consign (commands 35 and 36).
   * @param motor Motor number (1 or 2)
   * @param speed Signed speed in quad pulses per second.
   *
   * Once a value is sent the motor will begin to accelerate as fast as possible
   * until the defined speed is reached.
   */
  void driveMotorWithSpeed(uint8_t motor, int32_t speed);

  /**
   * @brief Drive both motors with signed speed consigns (command 37).
   * @param speed_m1 Signed speed in quad pulses per second.
   * @param speed_m2 Signed speed in quad pulses per second.
   *
   * Once a value is sent the motors will begin to accelerate as fast as possible
   * until the defined speed is reached.
   */
  void driveMotorsWithSpeed(int32_t speed_m1, int32_t speed_m2);

  /**
   * @brief Drive a motor with speed and acceleration consigns (com. 38 and 39).
   * @param motor Motor number (1 or 2)
   * @param speed Signed speed in quad pulses per second.
   * @param accel Unsigned acceleration or deceleration.
   *
   * Once a value is sent the motor will begin to accelerate incrementally
   * until the defined speed is reached.
   */
  void driveMotorWithSpeedAndAcceleration(uint8_t motor, int32_t speed, uint32_t accel);

  /**
   * @brief Drive both motors with speed and acceleration consigns (com. 40).
   * @param speed_m1 Signed speed in quad pulses per second.
   * @param speed_m2 Signed speed in quad pulses per second.
   * @param accel    Unsigned acceleration or deceleration (for both motors).
   *
   * Once a value is sent the motors will begin to accelerate incrementally
   * until the defined speed is reached.
   */
  void driveMotorsWithSpeedAndAcceleration(int32_t speed_m1, int32_t speed_m2, uint32_t accel);

  /**
   * @brief Adjusts the resolution of the PWM (command 48).
   * @param resolution PWM resolution in bits. Valid values: 8 to 14 included.
   */
  void setPWMResolution(uint8_t resolution);

  /**
   * @brief Sets main input minimum voltage level.
   * @param voltage Value in volts. Valid range is 6V - 30V.
   *
   * If the input voltage drops below the set voltage level Robo Claw will
   * shut down.
   *
   * This is command 2 with a more friendly float interface.
   */
  void setMinimumMainVoltage(float voltage);

  /**
   * @brief Sets main input maximum voltage level.
   * @param voltage Value in volts. Valid range is 0V - 30V.
   *
   * If you are using a input of any type you can ignore this setting.
   *
   * During regenerative breaking a back voltage is applied to charge the
   * battery. When using an ATX type power supply if it senses anything over
   * 16V it will shut down. By setting the maximum voltage level, Robo Claw
   * before exceeding it will go into hard breaking mode until the voltage
   * drops below the maximum value set.
   *
   * This is command 3 with a more friendly float interface.
   */
  void setMaximumMainVoltage(float voltage);

  /**
   * @brief Reads the main input voltage level.
   * @param voltage Output value.
   * @return true if main input voltage level was read correctly.
   *
   * This is command 24 with a more friendly float interface.
   */
  bool readMainVoltageLevel(float *voltage);

  /**
   * @brief Sets logic input minimum voltage level.
   * @param voltage Value in volts. Valid range is 6V - 28V.
   *
   * If the input voltage drops below the set voltage level Robo Claw will
   * shut down.
   *
   * This is command 26 with a more friendly float interface.
   */
  void setMinimumLogicVoltage(float voltage);

  /**
   * @brief Sets logic input maximum voltage level.
   * @param voltage Value in volts. Valid range is 0V - 28V.
   *
   * If the input voltage rises about the set voltage level Robo Claw will
   * shut down and require a hard reset to recover..
   *
   * This is command 27 with a more friendly float interface.
   */
  void setMaximumLogicVoltage(float voltage);

  /**
   * @brief Reads the logic input voltage level.
   * @param voltage Output value.
   * @return true if logic input voltage level was read correctly.
   *
   * This is command 25 with a more friendly float interface.
   */
  bool readLogicVoltageLevel(float *voltage);

  /**
   * @brief Reads Robo Claw firmware version.
   * @param version_string Output value (up to 32 bytes, null-terminated).
   * @return true if the firmware version was read correctly.
   *
   * This is command 21.
   */
  bool readFirmwareVersion(char *version_string);

private:

  /**
   * @brief Sends a command with a single unsigned byte param
   * @param command  The command id
   * @param value    The value of the command's argument
   */
  void sendUint8(uint8_t command, uint8_t value);

  /**
   * @brief Sends a command, reads a number of bytes and checks the CRC.
   * @param command  The command id
   * @param bytes    The output bytes
   * @param nb_bytes The expected number of bytes, CRC included
   * @return true if the bytes were read correctly and the CRC was good.
   */
  bool readBytes(uint8_t command, uint8_t *bytes, unsigned int nb_bytes);

  /**
   * @brief Sends a command and reads the 16-bit unsigned response (+CRC).
   * @param command  The command id
   * @param value    The output value
   * @return true if the value was read correctly.
   */
  bool readUint16(uint8_t command, uint16_t *value);

  /**
   * @brief Computes the CRC byte for Robo Claw.
   * @param bytes    An array of bytes (address, command, params)
   * @param nb_bytes The number of bytes to consider in the array
   * @return the CRC byte (more like a checksum really).
   */
  uint8_t computeCRC(uint8_t *bytes, unsigned int nb_bytes);

  /**
   * @brief Checks the given CRC against the CRC of the given bytes.
   * @param bytes    An array of bytes
   * @param nb_bytes The number of bytes to consider in the array
   * @param crc      The expected value of the CRC
   * @return true if the CRCs match, false otherwise.
   */
  bool checkCRC(uint8_t *bytes, unsigned int nb_bytes, uint8_t crc);

  cereal::CerealPort dev_; /**< The serial port to the Robo Claw */
  uint8_t address_;        /**< The address of the Robo Claw */

};

}

#endif
