#include "mono_vo.h"

MonoVO::MonoVO(int max_frame, int min_num_pts, std::string fn_kitti)
    : max_frame_(max_frame), min_num_pts_(min_num_pts), fn_kitti_(fn_kitti), id_(0)
{
  // Set several paths.
  fn_calib_ = fn_kitti_ + "/sequences/00/calib.txt";
  fn_poses_ = fn_kitti_ + "/poses/00.txt";
  fn_images_ = fn_kitti_ + "/sequences/00/image_2/";

  // Set the trajectory image.
  img_traj_ = cv::Mat::zeros(600, 600, CV_8UC3);

  // Fetch the focal length and pricipal point.
  FetchIntrinsicParams();

  Initialization();
}

void MonoVO::ReduceVector(std::vector<int> &v) {
  int j=0;

  for(int i=0; i<int(v.size()); i++) {
    if(status_[i]) {
      v[j++] = v[i];
    }
  }
  v.resize(j);
}

void MonoVO::FeatureTracking(cv::Mat img1,
                             cv::Mat img2,
                             std::vector<cv::Point2f> &p1,
                             std::vector<cv::Point2f> &p2)
{
  std::vector<float> error;
  cv::Size window_size = cv::Size(21,21);
  cv::TermCriteria tc = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);

  // Feature tracking using lucas-kanade tracker.
  cv::calcOpticalFlowPyrLK(img1, img2, p1, p2, status_, error, window_size, 3, tc, 0, 0.001);

  // Getting rid of points for which the KLT tracking failed or those who have gone outside the frame.
  int index_correction=0;

  for(int i=0; i<status_.size(); i++) {
    cv::Point2f pt = p2.at(i - index_correction);

    if((status_.at(i)==0) || (pt.x<0) || (pt.y<0)) {
      if((pt.x<0) || (pt.y<0)) {
        status_.at(i) = 0;
      }

      p1.erase(p1.begin() + (i-index_correction));
      p2.erase(p2.begin() + (i-index_correction));
      index_correction++;
    }
  }
}

void MonoVO::FeatureDetection(cv::Mat img, std::vector<cv::Point2f> &p)
{
  std::vector<cv::KeyPoint> kpt;

  // Extract features using FAST algorithm.
  cv::FAST(img, kpt, 20, true);

  // Sort keypoints ascending by response.
  std::sort(kpt.begin(), kpt.end(), [](const cv::KeyPoint &a, const cv::KeyPoint &b) {
                                      return a.response > b.response;
                                    });

  // Take the minimum number of feature point.
  for(int i=0; i<min_num_pts_; i++) {
    p.push_back(kpt[i].pt);
  }
}

void MonoVO::FetchIntrinsicParams() {
  // Read the calib.txt file.
  std::ifstream fin(fn_calib_);
  std::string line;

  if(fin.is_open()) {
    std::getline(fin, line);
    std::istringstream iss(line);

    for(int j=0; j<13; j++) {
      std::string token;
      iss >> token;
      if(j==1) {
        f_ = std::stod(token);   // focal length
      }
      if(j==3) {
        pp_.x = std::stod(token); // principal point x
      }
      if(j==7) {
        pp_.y = std::stod(token); // printcipal point y
      }
    }
    fin.close();
  }
  else {
    std::cout << "[-] Cannot read the calibration file: " << fn_calib_ << std::endl;
    f_ = 0;
    pp_ = cv::Point2f(0,0);
  }
}

void MonoVO::ComputeAbsoluteScale(int nframe)
{
  std::string line;
  int i=0;
  std::ifstream fin(fn_poses_);

  double x=0,y=0,z=0;
  double prev_x, prev_y, prev_z;

  if(fin.is_open()) {
    while((std::getline(fin, line)) && (i<=nframe)) {
      prev_z = z;
      prev_y = y;
      prev_x = x;

      std::istringstream iss(line);
      for(int j=0; j<12; j++) {
        iss>>z;
        if(j==7) y=z;
        if(j==3) x=z;
      }
      i++;
    }
    fin.close();
  }
  else {
    std::cout << "[-] Unable to open file: " << fn_poses_ << std::endl;
    scale_ = 0;
    return;
  }

  scale_ = std::sqrt( (x-prev_x)*(x-prev_x) +
                      (y-prev_y)*(y-prev_y) +
                      (z-prev_z)*(z-prev_z));
}

void MonoVO::Initialization() {
  // initialize (using first two frames).
  scale_ = 1.0;

  std::string fn1 = fn_images_ + util::AddZeroPadding(0, 6) + ".png";
  std::string fn2 = fn_images_ + util::AddZeroPadding(1, 6) + ".png";

  // Load the fist two images.
  cv::Mat img1 = cv::imread(fn1, cv::IMREAD_GRAYSCALE);
  cv::Mat img2 = cv::imread(fn2, cv::IMREAD_GRAYSCALE);

  // Feature extraction and tracking.
  std::vector<cv::Point2f> p1, p2;
  FeatureDetection(img1, p1);
  FeatureTracking(img1, img2, p1, p2);

  // Find essential matrix E and recover R,t from E.
  E_ = cv::findEssentialMat(p2, p1, f_, pp_, cv::RANSAC, 0.999, 1.0, mask_);
  cv::recoverPose(E_, p2, p1, curr_R_, curr_t_, f_, pp_, mask_);

  prev_img_ = img2;
  prev_pts_ = p2;

  prev_R_ = curr_R_.clone();
  prev_t_ = curr_t_.clone();
}

void MonoVO::PoseTracking(int nframe) {
  // pose tracking starts from the third image.
  if(nframe > max_frame_) {
    return;
  }

  std::string fn = fn_images_ + util::AddZeroPadding(nframe, 6) + ".png";
  cv::Mat curr_img_color = cv::imread(fn);
  cv::cvtColor(curr_img_color, curr_img_, cv::COLOR_BGR2GRAY);

  // Feature tracking.
  FeatureTracking(prev_img_, curr_img_, prev_pts_, curr_pts_);

  ReduceVector(idx_);

  // Find essential matrix E and recover R,t from E.
  E_ = cv::findEssentialMat(curr_pts_, prev_pts_, f_, pp_, cv::RANSAC, 0.999, 1.0, mask_);
  cv::recoverPose(E_, curr_pts_, prev_pts_, curr_R_, curr_t_, f_, pp_, mask_);

  // Get the scale.
  ComputeAbsoluteScale(nframe);

  if((scale_>0.1) &&
     (curr_t_.at<double>(2) > curr_t_.at<double>(0)) &&
     (curr_t_.at<double>(2) > curr_t_.at<double>(1)))
  {
    prev_t_ += scale_ * (prev_R_*curr_t_);
    prev_R_ = curr_R_ * prev_R_;
  }

  // If we have not enough features, try feature extraction again.
  if(prev_pts_.size() < min_num_pts_) {
    FeatureDetection(prev_img_, prev_pts_);
    FeatureTracking(prev_img_, curr_img_, prev_pts_, curr_pts_);
  }

  for(int i=0; i<curr_pts_.size(); i++) {
    idx_.push_back(id_++);
  }

  for(int i=0; i<curr_pts_.size(); i++) {
    prev_pts_map_.insert(std::make_pair(idx_[i], curr_pts_[i]));
  }

  dst_ = curr_img_color.clone();

  // Draw result.
  Visualize(nframe);

  prev_img_ = curr_img_.clone();
  prev_pts_ = curr_pts_;
}

void MonoVO::Visualize(int nframe) {
  cv::Mat effect = cv::Mat::zeros(cv::Size(dst_.cols, dst_.rows), CV_8UC3);

  // Draw tracking image.
  int visual_limit = 5000;
  if(visual_limit > min_num_pts_) {
    visual_limit = min_num_pts_;
  }

  for(size_t i=0; i<visual_limit; i++) {
    cv::circle(effect, prev_pts_[i], 2, cv::Scalar(0,0,255), 2);  // previous features (red).
    cv::circle(effect, curr_pts_[i], 2, cv::Scalar(255,0,0), 2);  // current features (blue).
  }

  std::map<int, cv::Point2f>::iterator mit;
  for(size_t i=0; i<visual_limit; i++) {
    int id = idx_[i];
    mit = prev_pts_map_.find(id);

    if(mit != prev_pts_map_.end()) {
      cv::arrowedLine(effect, curr_pts_[i], mit->second, cv::Scalar(255,255,255),1,16,0,0.1); // LK optical flow (white)
    }
  }

  // Blending image and effect.
  cv::addWeighted(dst_, 1.0, effect, 0.6, 0, dst_);

  // Add text.
  const std::string text2 = "Red: Previous Features";
  const std::string text3 = "Blue: Current Features";
  const std::string text4 = "Arrow: LK Optical Flow";
  const std::string text5 = "Frames: " + std::to_string(nframe);
  cv::putText(dst_, text2, cv::Point(dst_.cols-230, dst_.rows-100), 1, 1.0, cv::Scalar(255,255,255),2);
  cv::putText(dst_, text3, cv::Point(dst_.cols-230, dst_.rows-75), 1, 1.0, cv::Scalar(255,255,255),2);
  cv::putText(dst_, text4, cv::Point(dst_.cols-230, dst_.rows-50), 1, 1.0, cv::Scalar(255,255,255),2);
  cv::putText(dst_, text5, cv::Point(dst_.cols-230, dst_.rows-25), 1, 1.0, cv::Scalar(255,255,255),2);

#if 0
  cv::imshow("Result Image", dst_);
  cv::namedWindow("Result Image", cv::WINDOW_AUTOSIZE);
  cv::waitKey(1);
#endif
}
