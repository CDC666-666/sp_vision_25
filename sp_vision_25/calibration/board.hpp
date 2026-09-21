#ifndef CALIBRATION__BOARD_HPP
#define CALIBRATION__BOARD_HPP

#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace calibration
{
// 棋盘格尺寸是内角点数量；圆点板尺寸是圆心数量。
inline bool find_board_points(
  const cv::Mat & image, const cv::Size & pattern_size, const std::string & pattern_type,
  std::vector<cv::Point2f> & points)
{
  if (pattern_type == "circles")
    return cv::findCirclesGrid(image, pattern_size, points, cv::CALIB_CB_SYMMETRIC_GRID);

  if (pattern_type != "chessboard")
    throw std::runtime_error("pattern_type must be 'chessboard' or 'circles'");

  cv::Mat gray;
  if (image.channels() == 1)
    gray = image;
  else
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

  const auto found = cv::findChessboardCorners(
    gray, pattern_size, points, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
  if (found)
    cv::cornerSubPix(
      gray, points, cv::Size(11, 11), cv::Size(-1, -1),
      cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.001));
  return found;
}
}  // namespace calibration

#endif  // CALIBRATION__BOARD_HPP
