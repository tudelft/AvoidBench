#pragma once

#include <stdint.h>
#include <string>

#define XU4_ADC0_PATH "/sys/devices/12d10000.adc/iio:device0/in_voltage0_raw"
#define XU4_ADC3_PATH "/sys/devices/12d10000.adc/iio:device0/in_voltage3_raw"

#define UP_ADC_PATH "/sys/bus/iio/devices/iio:device0/in_voltage_raw"

namespace rpg_single_board_io
{

class ADCReader
{
public:
  ADCReader(const std::string& board_name, const unsigned int adc_id);
  ADCReader();
  ~ADCReader();

  int adcSetup(const std::string& board_name, const unsigned int adc_id);

  int adcReadRaw(unsigned int* value) const;
  int getMaxAdcValue();
  double getMaxAdcVoltage();

private:
  int setBoardName(const std::string& board_name);
  int adcConnect(const unsigned int adc_id);
  int adcDisconnect();

  enum class BoardNames
  {
    NONE, ODROID, UP
  } board_name_;

  int fd_;
  bool is_setup_ = false;

  // Constants
  static constexpr int kMaxAdcValueOdroid_ = 4095;
  static constexpr int kMaxAdcValueUp_ = 255;
  static constexpr double kMaxAdcVoltageOdroid_ = 1.8;
  static constexpr double kMaxAdcVoltageUp_ = 3.3;
};

} // namespace rpg_single_board_io
