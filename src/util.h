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

std::string AddZeroPadding(const int value, const unsigned precision);

cv::Vec3f RotMatToEuler(cv::Mat &R);

}// namespace util

#endif /* UTIL_H_ */
