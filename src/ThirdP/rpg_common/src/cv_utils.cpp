#include "rpg_common/cv_utils.h"

#include <Eigen/Dense>
#include <glog/logging.h>
#include <iostream>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

namespace rpg_common
{
namespace cv_utils
{

void printKeyPoints(const std::vector<cv::KeyPoint>& key_points)
{
  for (unsigned int i = 0; i < key_points.size(); ++i)
  {
    std::cout << "Point: x: " << key_points[i].pt.x << " y: "
              << key_points[i].pt.y;
  }
}

void getCvKeyPointsFromVector2d(const Eigen::Vector2d& keypoint,
                                const int level,
                                cv::KeyPoint* cv_key_points,
                                const int feature_size)
{
  CHECK_NOTNULL(cv_key_points);

  cv_key_points->pt = cv::Point_<float>(keypoint[0], keypoint[1]);
  // By default, FAST feature are using a radius of 3.
  cv_key_points->size = feature_size;
  cv_key_points->octave = level;
}

void getCvKeyPointsFromEigen(const Eigen::Matrix2Xd& coordinates,
                             const Eigen::VectorXi& key_point_levels,
                             std::vector<cv::KeyPoint>* key_points,
                             bool all_key_point_levels_are_empty)
{
  CHECK_NOTNULL(key_points);
  if (!all_key_point_levels_are_empty)
  {
    CHECK_EQ(coordinates.cols(), key_point_levels.rows());
  }

  key_points->resize(coordinates.cols());
  for (int i = 0; i < coordinates.cols(); ++i)
  {
    getCvKeyPointsFromVector2d(coordinates.col(i),
                               (all_key_point_levels_are_empty) ? 0 : key_point_levels(i),
                               &key_points->at(i));
  }
}

void getCvKeyPointsFromFrame(const Eigen::Matrix2Xd& eigen_key_points,
                             std::vector<cv::KeyPoint>* cv_key_points,
                             const int feature_size)
{
  CHECK_NOTNULL(cv_key_points);

  cv_key_points->resize(eigen_key_points.cols());
  for (int i = 0; i < eigen_key_points.cols(); ++i)
  {
    getCvKeyPointsFromVector2d(eigen_key_points.col(i),
                               0,
                               &cv_key_points->at(i),
                               feature_size);
  }
}

void showImage(const cv::Mat& image, const std::string &window_name)
{
  // Create a window for display.
  cv::namedWindow(window_name, cv::WINDOW_NORMAL);

  // Show our image inside it. Normalize floating point values.
  if (image.depth() == CV_32F || image.depth() == CV_64F)
  {
    cv::Mat image_normalized;
    cv::normalize(image, image_normalized, 0,1, cv::NORM_MINMAX);
    cv::imshow(window_name, image_normalized);
  }
  else
  {
    cv::imshow(window_name, image);
  }
  cv::waitKey(1);
}

void showImageWithKeyPoints(const cv::Mat& image,
                            const std::vector<cv::KeyPoint>& key_points)
{
  cv::Mat out_image;
  cv::drawKeypoints(image, key_points, out_image);

  showImage(out_image);
}

void showImageWithKeyPoints(const cv::Mat& image,
                            const Eigen::Matrix2Xd& key_points)
{
  std::vector<cv::KeyPoint> cv_key_points;
  const Eigen::VectorXi dummy_ref;
  getCvKeyPointsFromEigen(key_points, dummy_ref, &cv_key_points, true);
  showImageWithKeyPoints(image, cv_key_points);
}

// The four solutions are (R1/R2, +/-t).
void decomposeEssentialMatrix(
    const Eigen::Matrix3d& E, Eigen::Matrix3d* R1, Eigen::Matrix3d* R2,
    Eigen::Vector3d* t)
{
  CHECK_NOTNULL(R1);
  CHECK_NOTNULL(R2);
  CHECK_NOTNULL(t);

  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      E, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d Vt = svd.matrixV().transpose();

  if (U.determinant() < 0)
  {
    U *= -1;
  }
  if (Vt.determinant() < 0)
  {
    Vt *= -1;
  }

  Eigen::Matrix3d W;
  W << 0, 1, 0, -1, 0, 0, 0, 0, 1;

  *R1 = U * W * Vt;
  *R2 = U * W.transpose() * Vt;

  *t = U.col(2);
}

void concatVertical(const cv::Mat& A, const cv::Mat& B, cv::Mat* C)
{
  CHECK_EQ(A.cols, B.cols) << "Input arguments must have same number of columns";
  CHECK_EQ(A.type(), B.type()) << "Input arguments must have the same type";

  *C = cv::Mat(A.rows + B.rows, A.cols, A.type());
  cv::Mat C_top = C->rowRange(0, A.rows);
  A.copyTo(C_top);
  cv::Mat C_bottom = C->rowRange(A.rows, A.rows + B.rows);
  B.copyTo(C_bottom);
}


void concatHorizontal(const cv::Mat& A, const cv::Mat& B, cv::Mat* C)
{
  CHECK_EQ(A.rows, B.rows) << "Input arguments must have same number of rows";
  CHECK_EQ(A.type(), B.type()) << "Input arguments must have the same type";

  *C = cv::Mat(A.rows, A.cols + B.cols, A.type());
  cv::Mat C_left = C->colRange(0, A.cols);
  A.copyTo(C_left);
  cv::Mat C_right = C->colRange(A.cols, A.cols + B.cols);
  B.copyTo(C_right);
}

void drawArrow(cv::Mat* img, const cv::Point& pt1, const cv::Point& pt2,
               const cv::Scalar& color,  const int thickness, const int line_type,
               const int shift, const double tip_length_ratio)
{
  // Code adapted from: http://stackoverflow.com/a/24466638
  CHECK_NOTNULL(img);

  const double tipSize = cv::norm(pt1 - pt2) * tip_length_ratio;
  cv::line(*img, pt1, pt2, color, thickness, line_type, shift);
  const double angle = atan2((double) pt1.y - pt2.y, (double) pt1.x - pt2.x);
  cv::Point p(std::round(pt2.x + tipSize * std::cos(angle + CV_PI / 4)),
              std::round(pt2.y + tipSize * std::sin(angle + CV_PI / 4)));
  cv::line(*img, p, pt2, color, thickness, line_type, shift);
  p.x = std::round(pt2.x + tipSize * std::cos(angle - CV_PI / 4));
  p.y = std::round(pt2.y + tipSize * std::sin(angle - CV_PI / 4));
  cv::line(*img, p, pt2, color, thickness, line_type, shift);
}

} // namespace cv_utils
} // namespace rpg_common
