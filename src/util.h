#ifndef UTIL_H_
#define UTIL_H_

#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <iostream>
#include <string>
#include <iterator>
#include <fstream>
#include <map>
#include <iomanip>

namespace util {

/// \brief Add zero padding in the file name. It's used in the case that you should type '000001.png'.
/// \param[in] value The number of file name.
/// \param[in] precision Zero padding precision.
/// \return Zero padded string.
std::string AddZeroPadding(const int value, const unsigned precision);

/// \brief This method checks whether the rotations matrix is valid or not.
/// \param[in] R Rotation matrix.
/// \return Boolean of the result. If R is the rotation matrix, return true.
bool IsRotMat(cv::Mat &R);

/// \brief Convert rotation matrix to euler angle.
/// \param[in] R Rotation matrix.
/// \return Euler angle.
cv::Vec3f RotMatToEuler(cv::Mat &R);

}// namespace util

#endif /* UTIL_H_ */
