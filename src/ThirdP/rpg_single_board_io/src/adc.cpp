#include "rpg_single_board_io/adc.h"

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

namespace rpg_single_board_io
{

ADCReader::ADCReader(const std::string& board_name, const unsigned int adc_id) :
    fd_(-1)
{
  adcSetup(board_name, adc_id);
}

ADCReader::ADCReader() :
    board_name_(BoardNames::NONE), fd_(-1)
{
}

ADCReader::~ADCReader()
{
  adcDisconnect();
}

int ADCReader::adcSetup(const std::string& board_name,
                        const unsigned int adc_id)
{
  if (setBoardName(board_name) < 0)
  {
    return -1;
  }

  if (adcConnect(adc_id) < 0)
  {
    return -1;
  }

  is_setup_ = true;
  return 0;
}

int ADCReader::adcReadRaw(unsigned int* value) const
{
  if (!is_setup_)
  {
    perror("ADC has not been setup!");
    return -1;
  }

  if (fd_ < 0)
  {
    perror("ADC is not open");
    return fd_;
  }

  // Make sure we read from the beginning of the file
  lseek(fd_, 0, SEEK_SET);

  char buf[4];
  for (int i = 0; i < 4; i++)
  {
    char ch[1];
    int ret = read(fd_, ch, sizeof(ch));
    if (ret > 0)
    {
      buf[i] = ch[0];
    }
    else if (ret == 0)
    {
      // If we read less than 4 bytes, the last red byte seems to be rubbish
      buf[i - 1] = 0;
      break;
    }
    else
    {
      return -1;
    }
  }

  *value = atoi(buf);

  return 0;
}

int ADCReader::getMaxAdcValue()
{
  switch (board_name_)
  {
    case BoardNames::ODROID:
      return kMaxAdcValueOdroid_;
    case BoardNames::UP:
      return kMaxAdcValueUp_;
    case BoardNames::NONE:
    default:
      return -1;
  }
}

double ADCReader::getMaxAdcVoltage()
{
  switch (board_name_)
  {
    case BoardNames::ODROID:
      return kMaxAdcVoltageOdroid_;
    case BoardNames::UP:
      return kMaxAdcVoltageUp_;
    case BoardNames::NONE:
    default:
      return 0.0;
  }
}

int ADCReader::setBoardName(const std::string& board_name)
{
  const std::string odroid_name = "odroid";
  const std::string up_name = "up";

  if (odroid_name.compare(board_name) == 0)
  {
    board_name_ = BoardNames::ODROID;
  }
  else if (up_name.compare(board_name) == 0)
  {
    board_name_ = BoardNames::UP;
  }
  else
  {
    perror("No valid board name provided! [odroid, up]");
    return -1;
  }
  return 0;
}

int ADCReader::adcConnect(const unsigned int adc_id)
{
  if (board_name_ == BoardNames::NONE)
  {
    perror("Board name was not set before connecting to ADC!");
    return -1;
  }

  if (board_name_ == BoardNames::ODROID && adc_id != 0 && adc_id != 3)
  {
    perror("On the Odroid XU4, the ADC ID must be either 0 or 3!");
    return -1;
  }

  std::string adc_path;

  switch (board_name_)
  {
    case BoardNames::ODROID:
      if (adc_id == 0)
      {
        adc_path = XU4_ADC0_PATH;
      }
      if (adc_id == 3)
      {
        adc_path = XU4_ADC3_PATH;
      }
      break;
    case BoardNames::UP:
      adc_path = UP_ADC_PATH;
      break;
    case BoardNames::NONE:
    default:
      perror("Unknown board name set!");
      return -1;
  }

  fd_ = open(adc_path.c_str(), O_RDONLY);
  if (fd_ < 0)
  {
    perror("Could not open adc!");
    return fd_;
  }

  return 0;
}

int ADCReader::adcDisconnect()
{
  if (fd_ > -1)
  {
    close(fd_);
    fd_ = -1;
  }

  is_setup_ = false;

  return 0;
}

} // namespace rpg_single_board_io
