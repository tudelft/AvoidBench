#include "rpg_common/gnuplot.h"

#include <glog/logging.h>

namespace rpg_common {

Gnuplot::Gnuplot(const bool persist)
{
  pipe_ = popen(persist ? "gnuplot --persist" : "gnuplot", "w");
  // Prevent gnuplot from stealing focus all the time.
  operator <<("set term x11 noraise");
}

void Gnuplot::operator <<(const std::string& string)
{
  fputs((string + std::string("\n")).c_str(), pipe_);
  fflush(pipe_);
}

void Gnuplot::operator <<(const std::vector<size_t>& data)
{
  for (const size_t i : data)
  {
    fprintf(pipe_, "%zu\n", i);
  }
  fprintf(pipe_, "e\n");
  fflush(pipe_);
}

void Gnuplot::feedAsXy(
    const std::vector<double>& x, const std::vector<double>& y)
{
  CHECK_EQ(x.size(), y.size());
  for (size_t i = 0u; i < x.size(); ++i)
  {
    fprintf(pipe_, "%lf %lf\n", x[i], y[i]);
  }
  fprintf(pipe_, "e\n");
  fflush(pipe_);
}

}  // namespace rpg_common
