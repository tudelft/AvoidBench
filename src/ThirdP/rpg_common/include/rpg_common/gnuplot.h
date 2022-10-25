#pragma once

#include <Eigen/Dense>

#include "rpg_common/aligned.h"

namespace rpg_common {

// Launches an instance of Gnuplot to talk to. To plot data:
//
// rpg::Gnuplot plot;
// plot << "plot '-' w p";
// plot << your_matrix;
//
// See gnuplot documentation for commands.
//
// Needs `sudo apt-get install gnuplot-x11`
class Gnuplot
{
public:
  explicit Gnuplot(const bool persist = false);

  // Adds new line at end!
  void operator <<(const std::string& string);

  void operator <<(const std::vector<size_t>& data);

  template <int Rows, int Cols>
  void operator <<(const Eigen::Matrix<double, Rows, Cols>& data)
  {
    for (int row_i = 0; row_i < data.rows(); ++row_i)
    {
      for (int col_i = 0; col_i < data.cols(); ++col_i)
      {
        fprintf(pipe_, "%lf ", data(row_i, col_i));
      }
      fprintf(pipe_, "\n");
    }
    fprintf(pipe_, "e\n");
    fflush(pipe_);
  }

  template <int Cols>
  void operator <<(
      const Aligned<std::vector, Eigen::Matrix<double, 1, Cols>>& data)
  {
    for (const Eigen::Matrix<double, 1, Cols>& row : data)
    {
      for (int col_i = 0; col_i < row.cols(); ++col_i)
      {
        fprintf(pipe_, "%lf ", row(col_i));
      }
      fprintf(pipe_, "\n");
    }
    fprintf(pipe_, "e\n");
    fflush(pipe_);
  }

  void feedAsXy(const std::vector<double>& x, const std::vector<double>& y);

private:
  FILE* pipe_;
};

}  // namespace rpg_common
namespace rpg = rpg_common;
