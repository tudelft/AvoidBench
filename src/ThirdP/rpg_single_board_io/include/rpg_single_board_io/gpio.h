#pragma once

#define SYSFS_GPIO_DIR "/sys/class/gpio"

namespace rpg_single_board_io
{

enum class GpioDirection
{
  Unset, In, Out
};

enum class GpioEdge
{
  None, Rising, Falling, Both
};

enum class GpioValue
{
  High, Low
};

class GPIO
{
public:
  GPIO(const unsigned int gpio, const GpioDirection& dir);
  GPIO(const unsigned int gpio, const GpioEdge& edge);
  GPIO(); // gpioSetup needs to be called manually
  ~GPIO();

  int gpioSetValue(const GpioValue& value) const;
  int gpioGetValue(GpioValue *value) const;

  bool gpioIsOpen() const;
  int gpioGetFd() const;
  int gpioGetGpioNum() const;
  GpioDirection gpioGetDirection() const;
  GpioEdge gpioGetEdge() const;

  int gpioSetup(const unsigned int gpio, const GpioDirection& dir);
  int gpioSetup(const unsigned int gpio, const GpioEdge& edge);

private:
  int gpioExport(const unsigned int gpio) const;
  int gpioUnexport(const unsigned int gpio) const;
  int gpioSetDir(const unsigned int gpio, const GpioDirection& dir) const;
  int gpioSetEdge(const GpioEdge& edge) const;

  int gpioOpen();
  int gpioClose();

  int fd_value_;
  int num_gpio_;
  GpioDirection direction_;
  GpioEdge edge_;

  // Constants
  static constexpr int kMaxBufLen_ = 64;
};

} // namespace rpg_single_board_io
