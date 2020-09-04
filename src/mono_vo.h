/*

  The MIT License

  Copyright (c) 2020 Edward Im

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.

*/
/*

  The MIT License

  Copyright (c) 2015 Avi Singh

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.

*/
#ifndef MONO_VO_H_
#define MONO_VO_H_

#include "util.h"

#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <iostream>
#include <string>
#include <iterator>
#include <fstream>
#include <map>

/// \class MonoVO
/// \brief This class performs the visual odometry algorithm using mono image sequences.
class MonoVO {
 public:
  /// \brief The constructor.
  /// \param[in] max_frame Maximum number of frame to play.
  /// \param[in] min_num_pts Minimum number of features.
  /// \param[in] fn_kitti Path to kitti dataset.
  MonoVO(int max_frame, int min_num_pts, std::string fn_kitti);

  /// \brief Tract the features by using lucas-kanade optical flow.
  /// \param[in] img1 Input first image.
  /// \param[in] img2 Input second image.
  /// \param[in] p1 Features tracked by the feature extractor (e.g., FAST).
  /// \param[out] p2 Features being tracked by the lucas-kanade optical flow.
  void FeatureTracking(cv::Mat img1, cv::Mat img2, std::vector<cv::Point2f> &p1, std::vector<cv::Point2f> &p2);

  /// \brief Detection features.
  /// \param[in] img Input image.
  /// \param[out] p Vector of feature points.
  void FeatureDetection(cv::Mat img, std::vector<cv::Point2f> &p);

  /// \brief Perform the intialization process with the first two frames.
  void Initialization();

  /// \brief Pose tracking using lucas-kanade optical flow.
  /// \param[in] The number of current frame.
  void PoseTracking(int nframe);

  /// \brief Visualize the result.
  /// \param[in] nframe The number of current frame.
  void Visualize(int nframe);

  /// \brief Reduce the size of vector after tracking.
  /// \param[in/out] v The vector to be reduced.
  void ReduceVector(std::vector<int> &v);

  /// \brief This method gets the focal length and principal point from the calibration file.
  void FetchIntrinsicParams();

  /// \brief Compute the absolute scale.
  /// \param[in] nframe Number of frame.
  void ComputeAbsoluteScale(int nframe);

  /// \brief Get the last camera rotation.
  cv::Mat GetRotation() { return prev_R_; }

  /// \brief Get the last camera translation.
  cv::Mat GetTranslation() { return prev_t_; }

  /// \brief Get the result image.
  cv::Mat GetResultImage() { return dst_; }

 private:
  /// \brief Maximum number of frame to play.
  int max_frame_;

  /// \brief Minimum number of features.
  int min_num_pts_;

  /// \brief Path to kitti dataset.
  std::string fn_kitti_;

  /// \brief Path to calibration file.
  std::string fn_calib_;

  /// \brief Path to ground truth pose file.
  std::string fn_poses_;

  /// \brief Path to image file.
  std::string fn_images_;

  /// \brief Previous gray image.
  cv::Mat prev_img_;

  /// \brief Current gray image.
  cv::Mat curr_img_;

  /// \brief Result image to be shown.
  cv::Mat dst_;

  /// \brief Previous features.
  std::vector<cv::Point2f> prev_pts_;

  /// \brief Current features.
  std::vector<cv::Point2f> curr_pts_;

  /// \brief Previous rotation.
  cv::Mat prev_R_;

  /// \brief Previous translation.
  cv::Mat prev_t_;

  /// \brief Current rotation.
  cv::Mat curr_R_;

  /// \brief Current translation.
  cv::Mat curr_t_;

  /// \brief Scale parameter.
  double scale_;

  /// \brief Focal length.
  double f_;

  /// \brief Principal point.
  cv::Point2f pp_;

  /// \brief Essential matrix.
  cv::Mat E_;

  /// \brief Optical flow status.
  std::vector<unsigned char> status_;

  /// \brief Masking matrix.
  cv::Mat mask_;

  /// \brief Trajectory windows.
  cv::Mat img_traj_;

  /// \brief The vector of feature point id.
  std::vector<int> idx_;

  /// \brief Unique id of each feature point.
  unsigned int id_;

  /// \brief map of previous feature points and its id.
  std::map<int, cv::Point2f> prev_pts_map_;
};

#endif /* MONO_VO_H_ */
