#include "util.h"

namespace util {

std::string AddZeroPadding(const int value, const unsigned precision) {
  std::ostringstream oss;
  oss << std::setw(precision) << std::setfill('0') << value;
  return oss.str();
}

bool IsRotMat(cv::Mat &R) {
  cv::Mat Rt;
  transpose(R, Rt);
  cv::Mat shouldBeIdentity = Rt * R;
  cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());

  return norm(I, shouldBeIdentity) < 1e-6;
}

cv::Vec3f RotMatToEuler(cv::Mat &R) {
  assert(IsRotMat(R));
  float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
  bool singular = sy < 1e-6;
  float x, y, z;

  if (!singular) {
    x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
    y = atan2(-R.at<double>(2,0), sy);
    z = atan2(R.at<double>(1,0), R.at<double>(0,0));
  }
  else {
    x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
    y = atan2(-R.at<double>(2,0), sy);
    z = 0;
  }

  return cv::Vec3f(x, y, z);
}

}  // namespace util
