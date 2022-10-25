#include "rpg_single_board_io/gpio.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>

namespace rpg_single_board_io
{

GPIO::GPIO(const unsigned int gpio, const GpioDirection& dir) :
    fd_value_(-1), num_gpio_(-1), direction_(GpioDirection::In), edge_(
        GpioEdge::None)
{
  gpioSetup(gpio, dir);
}

GPIO::GPIO(const unsigned int gpio, const GpioEdge& edge) :
    fd_value_(-1), num_gpio_(-1), direction_(GpioDirection::In), edge_(
        GpioEdge::None)
{
  gpioSetup(gpio, edge);
}

GPIO::GPIO() :
    fd_value_(-1), num_gpio_(-1), direction_(GpioDirection::Unset), edge_(
        GpioEdge::None)
{
}

GPIO::~GPIO()
{
  gpioClose();
}

int GPIO::gpioSetup(const unsigned int gpio, const GpioDirection& dir)
{
  gpioUnexport(gpio);

  if (gpioExport(gpio) < 0)
  {
    perror("gpio/init: export");
    return -1;
  }
  num_gpio_ = gpio;

  if (gpioSetDir(gpio, dir) < 0)
  {
    perror("gpio/init: direction");
    return -1;
  }
  direction_ = dir;

  if (gpioOpen() < 0)
  {
    perror("gpio/init: open");
    return -1;
  }

  return 0;
}

int GPIO::gpioSetup(const unsigned int gpio, const GpioEdge& edge)
{
  gpioUnexport(gpio);

  if (gpioExport(gpio) < 0)
  {
    perror("gpio/init: export");
    return -1;
  }
  num_gpio_ = gpio;

  if (gpioSetDir(gpio, GpioDirection::In) < 0)
  {
    perror("gpio/init: direction");
    return -1;
  }
  direction_ = GpioDirection::In;

  if (gpioSetEdge(edge) < 0)
  {
    perror("gpio/init: edge");
    return -1;
  }
  edge_ = edge;

  if (gpioOpen() < 0)
  {
    perror("gpio/init: open");
    return -1;
  }

  return 0;
}

int GPIO::gpioSetValue(const GpioValue& value) const
{
  if (fd_value_ < 0)
  {
    return fd_value_;
  }

  if (direction_ != GpioDirection::Out)
  {
    perror("gpio/set-value");
    return -1;
  }

  if (value == GpioValue::High)
  {
    if (write(fd_value_, "1", 2) < 0)
    {
      return -1;
    }
  }
  else
  {
    if (write(fd_value_, "0", 2) < 0)
    {
      return -1;
    }
  }

  return 0;
}

int GPIO::gpioGetValue(GpioValue *value) const
{
  char ch;
  if (fd_value_ < 0)
  {
    return fd_value_;
  }

  if (direction_ != GpioDirection::In)
  {
    perror("gpio/get-value");
    return -1;
  }

  if (read(fd_value_, &ch, 1) < 0)
  {
    return -1;
  }

  if (ch != '0')
  {
    *value = GpioValue::High;
  }
  else
  {
    *value = GpioValue::Low;
  }

  return 0;
}

bool GPIO::gpioIsOpen() const
{
  return fd_value_ > -1;
}

int GPIO::gpioGetFd() const
{
  if (fd_value_ < 0)
  {
    perror("gpio/fd_open");
  }

  return fd_value_;
}

int GPIO::gpioGetGpioNum() const
{
  return num_gpio_;
}

GpioDirection GPIO::gpioGetDirection() const
{
  return direction_;
}

GpioEdge GPIO::gpioGetEdge() const
{
  return edge_;
}

int GPIO::gpioExport(const unsigned int gpio) const
{
  int fd, len;
  char buf[kMaxBufLen_];

  fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
  if (fd < 0)
  {
    perror("gpio/export: open failed");
    return fd;
  }

  len = snprintf(buf, sizeof(buf), "%d", gpio);
  int res;
  res = write(fd, buf, len);

  printf("Export res: %i\n\r", res);

  if (res < 0)
  {
    perror("gpio/export: write failed");
    return -1;
  }

  printf("Export successful!\n\r");

  close(fd);

  return 0;
}

int GPIO::gpioUnexport(const unsigned int gpio) const
{
  int fd, len;
  char buf[kMaxBufLen_];

  fd = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);
  if (fd < 0)
  {
    return fd;
  }

  len = snprintf(buf, sizeof(buf), "%d", gpio);
  if (write(fd, buf, len) < 0)
  {
    perror("gpio/unexport");
    return -1;
  }

  printf("Unexport successful!\n\r");

  close(fd);

  return 0;
}

int GPIO::gpioSetDir(const unsigned int gpio, const GpioDirection& dir) const
{
  int fd;
  char buf[kMaxBufLen_];

  snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/direction", gpio);

  fd = open(buf, O_WRONLY);
  if (fd < 0)
  {
    return fd;
  }

  // After exporting the GPIO pin, udev rules (if used) need a moment to
  // apply. So we will retry writing the direction for about 2 seconds
  // before giving up.
  const size_t max_attempts = 2000;
  const size_t sleep_between_attempts = 1000; // [microseconds]
  for (size_t attempt = 0; attempt <= max_attempts; attempt++)
  {
    if (dir == GpioDirection::Out)
    {
      if (write(fd, "out", 4) >= 0)
      {
        // Direction set successfully
        break;
      }
    }
    else
    {
      if (write(fd, "in", 3) >= 0)
      {
        // Direction set successfully
        break;
      }
    }

    // Chill for a moment
    usleep(sleep_between_attempts);

    if (attempt >= max_attempts)
    {
      return -1;
    }
  }

  printf("Set dir successful!\n\r");
  close(fd);

  return 0;
}

int GPIO::gpioSetEdge(const GpioEdge& edge) const
{
  int fd;
  char buf[kMaxBufLen_];

  snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/edge", num_gpio_);

  fd = open(buf, O_WRONLY);
  if (fd < 0)
  {
    perror("gpio/set-edge");
    return fd;
  }

  std::string edge_name;
  switch (edge)
  {
    case GpioEdge::Rising:
      edge_name = "rising";
      break;
    case GpioEdge::Falling:
      edge_name = "falling";
      break;
    case GpioEdge::Both:
      edge_name = "both";
      break;
  }

  // After exporting the GPIO pin, udev rules (if used) need a moment to
  // apply. So we will retry writing the direction for about 2 seconds
  // before giving up.
  const size_t max_attempts = 2000;
  const size_t sleep_between_attempts = 1000; // [microseconds]
  for (size_t attempt = 0; attempt <= max_attempts; attempt++)
  {
    if (write(fd, edge_name.c_str(), edge_name.length() + 1) >= 0)
    {
      // Edge set successfully
      break;
    }

    // Chill for a moment
    usleep(sleep_between_attempts);

    if (attempt >= max_attempts)
    {
      return -1;
    }
  }

  printf("Set edge successful!\n\r");
  close(fd);

  return 0;
}

int GPIO::gpioOpen()
{
  char buf[kMaxBufLen_];

  snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", num_gpio_);

  if (direction_ == GpioDirection::In)
  {
    fd_value_ = open(buf, O_RDONLY);
  }
  else
  {
    fd_value_ = open(buf, O_WRONLY);
  }
  if (fd_value_ < 0)
  {
    perror("gpio/init: Value");
    return fd_value_;
  }

  return 0;
}

int GPIO::gpioClose()
{
  if (num_gpio_ != -1)
  {
    gpioUnexport(num_gpio_);
    num_gpio_ = -1;
  }

  if (fd_value_ > -1)
  {
    close(fd_value_);
    fd_value_ = -1;
  }

  direction_ = GpioDirection::In;
  edge_ = GpioEdge::None;

  return 0;
}

} // namespace rpg_single_board_io
