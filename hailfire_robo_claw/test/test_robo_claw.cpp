#include "hailfire_robo_claw/robo_claw.h"
#include <iostream>
#include <getopt.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>

#define ARRAY_LEN(x) (sizeof(x) / sizeof(x[0]))

struct option g_options[] =
{
  // option       A  Flag   V  (has_arg, flag, val)
  // -----------  -  ----  ---
  { "port",       1, NULL, 'p' },
  { "baud",       1, NULL, 'b' },
  { "debug",      0, NULL, 'd' },
  { "verbose",    0, NULL, 'v' },
  { "help",       0, NULL, 'h' },
  { 0,            0, 0,    0   }
};

struct operation_t
{
  const char *name;
  int argc;
}
g_operations[22] =
{
  { "driveMotor",                          2 },
  { "moveForwardOrBackward",               1 },
  { "turnLeftOrRight",                     1 },
  { "readEncoderCount",                    1 },
  { "readEncoderSpeed",                    1 },
  { "resetEncoders",                       0 },
  { "setPIDConstants",                     5 },
  { "readCurrentSpeed",                    1 },
  { "driveMotorWithDutyCycle",             2 },
  { "driveMotorsWithDutyCycle",            2 },
  { "driveMotorWithSpeed",                 2 },
  { "driveMotorsWithSpeed",                2 },
  { "driveMotorWithSpeedAndAcceleration",  3 },
  { "driveMotorsWithSpeedAndAcceleration", 3 },
  { "setPWMResolution",                    1 },
  { "setMinimumMainVoltage",               1 },
  { "setMaximumMainVoltage",               1 },
  { "readMainVoltageLevel",                0 },
  { "setMinimumLogicVoltage",              1 },
  { "setMaximumLogicVoltage",              1 },
  { "readLogicVoltageLevel",               0 },
  { "readFirmwareVersion",                 0 },
};

int usage(int argc, char **argv)
{
  std::cerr << "Usage: " << argv[0] << " [option(s)] op [args]" << std::endl
            << "  Calls a method of RoboClaw C++ class" << std::endl
            << std::endl
            << "  -p, --port=port   Set the I/O port" << std::endl
            << "  -b, --baud=baud   Set the baudrate used" << std::endl
            << "  -d, --debug       Turn on debug output" << std::endl
            << "  -v, --verbose     Turn on verbose messages" << std::endl
            << "  -h, --help        Display this message" << std::endl;
  return 1;
}

int main(int argc, char **argv)
{
  const char *port = NULL;
  speed_t baud = B0;
  bool debug = 0;
  bool verbose = 0;

  // Parse the command line options
  int opt;
  while ((opt = getopt_long(argc, argv, "p:b:dvh", g_options, NULL)) > 0)
  {
    switch (opt)
    {
      case 'p':
      {
        port = optarg;
        break;
      }
      case 'b':
      {
        switch (atoi(optarg))
        {
          case 2400:
            baud = B2400;
            break;
          case 9600:
            baud = B9600;
            break;
          case 19200:
            baud = B19200;
            break;
          case 38400:
            baud = B38400;
            break;
          default:
            std::cerr << "Unsupported baud rate: " << optarg << std::endl;
            return 1;
        }
        break;
      }
      case 'd':
      {
        debug = true;
        break;
      }
      case 'v':
      {
        verbose = true;
        break;
      }
      case '?':
      case 'h':
      {
        return usage(argc, argv);
      }
    }
  }

  // Port and baud must be given
  if (!port || baud == B0)
  {
    std::cerr << "Missing --port and/or --baud option(s)" << std::endl;
    return 1;
  }

  std::cout << "Hello World" << std::endl
            << "Port: " << port << std::endl
            << "Baud: " << baud << std::endl;

  std::cout << "Remaining args:" << std::endl;
  for (int i = optind; i < argc; ++i)
  {
    std::cout << argv[i] << std::endl;
  }

  // At least 1 non-option argument must be given
  if (optind == argc)
  {
    std::cerr << "Missing operation" << std::endl;
    return 1;
  }

  // Get the operation description from g_operations
  struct operation_t *op = NULL;
  for (unsigned int i = 0; i < ARRAY_LEN(g_operations); ++i)
  {
    if (strcmp(g_operations[i].name, argv[optind]) == 0)
    {
      op = &g_operations[i];
      break;
    }
  }

  // Bail if the operation could not be found
  if (!op)
  {
    std::cerr << "Unknown operation: " << argv[optind] << std::endl;
    return 1;
  }

  // There must be just enough non-option arguments for the operation name
  // and the operation params.
  if ((argc - optind) != (1 + op->argc))
  {
    std::cerr << "Too many or too few arguments: "
              << op->name << " needs " << op->argc << " arguments"
              << std::endl;
    return 1;
  }

  int args_idx = optind + 1;

  hailfire_robo_claw::RoboClaw robo_claw(port, baud, 128);

  // Call method
  if (strcmp(op->name, "driveMotor") == 0)
  {
    uint8_t motor = atoi(argv[args_idx++]);
    int8_t speed  = atoi(argv[args_idx++]);

    printf("driveMotor(%u, %d)\n", motor, speed);
    robo_claw.driveMotor(motor, speed);
  }
  else if (strcmp(op->name, "moveForwardOrBackward") == 0)
  {
    int8_t speed = atoi(argv[args_idx++]);

    printf("moveForwardOrBackward(%d)\n", speed);
    robo_claw.moveForwardOrBackward(speed);
  }
  else if (strcmp(op->name, "turnLeftOrRight") == 0)
  {
    int8_t speed = atoi(argv[args_idx++]);

    printf("turnLeftOrRight(%d)\n", speed);
    robo_claw.turnLeftOrRight(speed);
  }
  else if (strcmp(op->name, "readEncoderCount") == 0)
  {
    uint8_t encoder = atoi(argv[args_idx++]);
    int32_t count   = 0;
    uint8_t status  = 0;

    printf("readEncoderCount(%u)\n", encoder);
    bool ret = robo_claw.readEncoderCount(encoder, &count, &status);

    printf("readEncoderCount returned: %s\n", (ret ? "true" : "false"));
    printf("count: %d\n", count);
    printf("status: %u\n", status);
  }
  else if (strcmp(op->name, "readEncoderSpeed") == 0)
  {
    uint8_t encoder = atoi(argv[args_idx++]);
    int32_t speed   = 0;

    printf("readEncoderSpeed(%u)\n", encoder);
    bool ret = robo_claw.readEncoderSpeed(encoder, &speed);

    printf("readEncoderSpeed returned: %s\n", (ret ? "true" : "false"));
    printf("speed: %d\n", speed);
  }
  else if (strcmp(op->name, "resetEncoders") == 0)
  {
    printf("resetEncoders()\n");
    robo_claw.resetEncoders();
  }
  else if (strcmp(op->name, "setPIDConstants") == 0)
  {
    uint8_t motor = atoi(argv[args_idx++]);
    uint32_t p    = atoi(argv[args_idx++]);
    uint32_t i    = atoi(argv[args_idx++]);
    uint32_t d    = atoi(argv[args_idx++]);
    uint32_t qpps = atoi(argv[args_idx++]);

    printf("setPIDConstants(%u, %u, %u, %u, %u)\n", motor, p, i, d, qpps);
    robo_claw.setPIDConstants(motor, p, i, d, qpps);
  }
  else if (strcmp(op->name, "readCurrentSpeed") == 0)
  {
    uint8_t motor = atoi(argv[args_idx++]);
    int32_t speed = 0;

    printf("readCurrentSpeed(%u)", motor);
    bool ret = robo_claw.readCurrentSpeed(motor, &speed);

    printf("readCurrentSpeed returned: %s\n", (ret ? "true" : "false"));
    printf("speed: %d\n", speed);
  }
  else if (strcmp(op->name, "driveMotorWithDutyCycle") == 0)
  {
    uint8_t motor      = atoi(argv[args_idx++]);
    int16_t duty_cycle = atoi(argv[args_idx++]);

    printf("driveMotorWithDutyCycle(%u, %d)\n", motor, duty_cycle);
    robo_claw.driveMotorWithDutyCycle(motor, duty_cycle);
  }
  else if (strcmp(op->name, "driveMotorsWithDutyCycle") == 0)
  {
    int16_t duty_m1 = atoi(argv[args_idx++]);
    int16_t duty_m2 = atoi(argv[args_idx++]);

    printf("driveMotorsWithDutyCycle(%d, %d)\n", duty_m1, duty_m2);
    robo_claw.driveMotorsWithDutyCycle(duty_m1, duty_m2);
  }
  else if (strcmp(op->name, "driveMotorWithSpeed") == 0)
  {
    uint8_t motor = atoi(argv[args_idx++]);
    int32_t speed = atoi(argv[args_idx++]);

    printf("driveMotorWithSpeed(%u, %d)\n", motor, speed);
    robo_claw.driveMotorWithSpeed(motor, speed);
  }
  else if (strcmp(op->name, "driveMotorsWithSpeed") == 0)
  {
    int32_t speed_m1 = atoi(argv[args_idx++]);
    int32_t speed_m2 = atoi(argv[args_idx++]);

    printf("driveMotorsWithSpeed(%d, %d)\n", speed_m1, speed_m2);
    robo_claw.driveMotorsWithSpeed(speed_m1, speed_m2);
  }
  else if (strcmp(op->name, "driveMotorWithSpeedAndAcceleration") == 0)
  {
    uint8_t motor  = atoi(argv[args_idx++]);
    int32_t speed  = atoi(argv[args_idx++]);
    uint32_t accel = atoi(argv[args_idx++]);

    printf("driveMotorWithSpeedAndAcceleration(%u, %d, %u)\n", motor, speed, accel);
    robo_claw.driveMotorWithSpeedAndAcceleration(motor, speed, accel);
  }
  else if (strcmp(op->name, "driveMotorsWithSpeedAndAcceleration") == 0)
  {
    int32_t speed_m1 = atoi(argv[args_idx++]);
    int32_t speed_m2 = atoi(argv[args_idx++]);
    uint32_t accel   = atoi(argv[args_idx++]);

    printf("driveMotorsWithSpeedAndAcceleration(%d, %d, %u)\n", speed_m1, speed_m2, accel);
    robo_claw.driveMotorsWithSpeedAndAcceleration(speed_m1, speed_m2, accel);
  }
  else if (strcmp(op->name, "setPWMResolution") == 0)
  {
    uint8_t resolution = atoi(argv[args_idx++]);

    printf("setPWMResolution(%u)\n", resolution);
    robo_claw.setPWMResolution(resolution);
  }
  else if (strcmp(op->name, "setMinimumMainVoltage") == 0)
  {
    float voltage = atof(argv[args_idx++]);

    printf("setMinimumMainVoltage(%f)\n", voltage);
    robo_claw.setMinimumMainVoltage(voltage);
  }
  else if (strcmp(op->name, "setMaximumMainVoltage") == 0)
  {
    float voltage = atof(argv[args_idx++]);

    printf("setMaximumMainVoltage(%f)\n", voltage);
    robo_claw.setMaximumMainVoltage(voltage);
  }
  else if (strcmp(op->name, "readMainVoltageLevel") == 0)
  {
    float voltage = 0;

    printf("readMainVoltageLevel()\n");
    bool ret = robo_claw.readMainVoltageLevel(&voltage);

    printf("readMainVoltageLevel returned: %s\n", (ret ? "true" : "false"));
    printf("voltage: %f\n", voltage);
  }
  else if (strcmp(op->name, "setMinimumLogicVoltage") == 0)
  {
    float voltage = atof(argv[args_idx++]);

    printf("setMinimumLogicVoltage(%f)\n", voltage);
    robo_claw.setMinimumLogicVoltage(voltage);
  }
  else if (strcmp(op->name, "setMaximumLogicVoltage") == 0)
  {
    float voltage = atof(argv[args_idx++]);

    printf("setMaximumLogicVoltage(%f)\n", voltage);
    robo_claw.setMaximumLogicVoltage(voltage);
  }
  else if (strcmp(op->name, "readLogicVoltageLevel") == 0)
  {
    float voltage = 0;

    printf("readLogicVoltageLevel()\n");
    bool ret = robo_claw.readLogicVoltageLevel(&voltage);

    printf("readLogicVoltageLevel returned: %s\n", (ret ? "true" : "false"));
    printf("voltage: %f\n", voltage);
  }
  else if (strcmp(op->name, "readFirmwareVersion") == 0)
  {
    char version[32];

    printf("readFirmwareVersion()\n");
    bool ret = robo_claw.readFirmwareVersion(version);

    printf("readFirmwareVersion returned: %s\n", (ret ? "true" : "false"));
    printf("version: %s\n", version);
  }
  else
  {
    std::cerr << "Unhandled operation: " << op->name << std::endl;
    return 1;
  }

  return 0;
}
