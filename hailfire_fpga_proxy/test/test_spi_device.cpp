#include "hailfire_fpga_proxy/spi_device.h"
#include <gtest/gtest.h>
#include <iostream>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

using namespace hailfire_fpga_proxy;

static char dev_filename[] = "/dev/spidev1.1";

TEST(SPIDevice, DISABLED_foo)
{
  SPIDevice spi_device(dev_filename);

  // mode
  uint8_t old_mode = spi_device.getMode();
  spi_device.setMode(SPI_MODE_1);
  uint8_t new_mode = spi_device.getMode();
  EXPECT_EQ(new_mode, SPI_MODE_1);
  spi_device.setMode(old_mode);

  // lsb_first
  bool old_lsb_first = spi_device.getLSBFirst();
  spi_device.setLSBFirst(!old_lsb_first);
  bool new_lsb_first = spi_device.getLSBFirst();
  EXPECT_EQ(new_lsb_first, !old_lsb_first);
  spi_device.setLSBFirst(old_lsb_first);

  // bits_per_word
  uint8_t old_bits_per_word = spi_device.getBitsPerWord();
  spi_device.setBitsPerWord(2 * old_bits_per_word);
  uint8_t new_bits_per_word = spi_device.getBitsPerWord();
  EXPECT_EQ(new_bits_per_word, 2 * old_bits_per_word);
  spi_device.setBitsPerWord(old_bits_per_word);

  // max_speed
  uint32_t old_max_speed = spi_device.getMaxSpeed();
  spi_device.setMaxSpeed(2 * old_max_speed);
  uint32_t new_max_speed = spi_device.getMaxSpeed();
  EXPECT_EQ(new_max_speed, 2 * old_max_speed);
  spi_device.setMaxSpeed(old_max_speed);

  // transfer
  uint8_t tx[] =
  {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x95,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xDE, 0xAD, 0xBE, 0xEF, 0xBA, 0xAD,
    0xF0, 0x0D,
  };
  uint8_t rx[ARRAY_SIZE(tx)] = {0, };
  spi_device.doSyncTransfer(tx, rx, ARRAY_SIZE(tx));

  // response should be equal to send data
  for (unsigned int i = 0; i < ARRAY_SIZE(tx); ++i)
  {
    EXPECT_EQ(rx[i], tx[i]);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  // Show a message about disabled tests unless already asked to run them
  if (!testing::GTEST_FLAG(also_run_disabled_tests))
  {
    std::cout << "Test suite skipped." << std::endl
              << "Run " << argv[0] << " --gtest_also_run_disabled_tests "
              << "to run SPI tests if " << dev_filename << " is available "
              << "and MOSI and MISO are connected (loopback)" << std::endl;
  }

  return RUN_ALL_TESTS();
}
