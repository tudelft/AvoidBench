#include <iostream>
#include <vector>

#include <Eigen/Core>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace rpg_common
{
namespace cv_utils
{
// Prints the key point coordinates to the standard output
void printKeyPoints(const std::vector<cv::KeyPoint>& key_points);

// By default, FAST feature are using a radius of 3.
void getCvKeyPointsFromVector2d(const Eigen::Vector2d& keypoint,
                                const int level,
                                cv::KeyPoint* cv_key_points,
                                const int feature_size = 3);

// if all_key_point_levels_are_empty is set to true.
// 0 will be set for all levels
void getCvKeyPointsFromEigen(const Eigen::Matrix2Xd& coordinates,
                             const Eigen::VectorXi& key_point_levels,
                             std::vector<cv::KeyPoint>* key_points,
                             bool all_key_point_levels_are_empty = false);

// By default, FAST feature are using a radius of 3.
void getCvKeyPointsFromFrame(const Eigen::Matrix2Xd& eigen_key_points,
                             std::vector<cv::KeyPoint>* cv_key_points,
                             const int feature_size = 3);

// Display the cv image (imshow).
// Replaces the old image if called multiple times.
void showImage(const cv::Mat& image,
               const std::string& window_name = std::string("Display window"));

// Display the cv image, with the highlighted key points coordinates.
// Replaces the old image if called multiple times.
void showImageWithKeyPoints(const cv::Mat& image,
                            const std::vector<cv::KeyPoint>& key_points);

// Display the cv image, with the highlighted key points coordinates.
// Replaces the old image if called multiple times.
void showImageWithKeyPoints(const cv::Mat& image,
                            const Eigen::Matrix2Xd& key_points);

// The four solutions are (R1/R2, +/-t).
void decomposeEssentialMatrix(
    const Eigen::Matrix3d& E, Eigen::Matrix3d* R1, Eigen::Matrix3d* R2,
    Eigen::Vector3d* t);

// Concatenates two matrices vertically.
void concatVertical(const cv::Mat& A, const cv::Mat& B, cv::Mat* C);

// Concatenates two matrices horizontally.
void concatHorizontal(const cv::Mat& A, const cv::Mat& B, cv::Mat* C);

// Draws an arrow between two points on an image.
// For OpenCV >= 2.4.10, you should use the function arrowedLine from OpenCV
// (http://stackoverflow.com/a/33841805).
void drawArrow(cv::Mat* img, const cv::Point& pt1, const cv::Point& pt2,
               const cv::Scalar& color,  const int thickness = 1,
               const int line_type = 8, const int shift = 0,
               const double tip_length_ratio = 0.1);

} // namespace cv_utils
} // namespace rpg_common
namespace rpg = rpg_common;
