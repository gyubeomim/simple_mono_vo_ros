#include "mono_vo.h"

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <tf/transform_broadcaster.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "mono_vo_node");
  ros::NodeHandle nh, priv_nh("~");

  ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 1);
  ros::Publisher pub_image = nh.advertise<sensor_msgs::Image>("image", 1);

  int max_frame;
  int min_num_feat;
  std::string fn_kitti;

  priv_nh.getParam("max_frame", max_frame); // 1000
  priv_nh.getParam("min_num_feat", min_num_feat); // 2000
  priv_nh.getParam("fn_kitti", fn_kitti);

  tf::TransformBroadcaster tf_broadcaster;
  tf::StampedTransform transform;
  transform.frame_id_ = "/world";
  transform.child_frame_id_ = "/camera";

  MonoVO* vo = new MonoVO(max_frame, min_num_feat, fn_kitti);

  int nframe=2;
  while(ros::ok()) {
    if(nframe >= max_frame) {
      break;
    }

    vo->PoseTracking(nframe);

    cv::Mat R = vo->GetRotation();
    cv::Mat t_wc = vo->GetTranslation();
    cv::Vec3f euler = util::RotMatToEuler(R);
    tf::Matrix3x3 _R(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                     R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                     R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
    tf::Quaternion quat;
    _R.getRotation(quat);

    cv::Ptr<cv::Formatter> round = cv::Formatter::get(cv::Formatter::FMT_DEFAULT);
    round->set64fPrecision(3);
    round->set32fPrecision(3);

    std::cout << std::endl << "[+] " << util::AddZeroPadding(nframe,6) << std::endl;
    std::cout << "Rotation(euler):  " <<  std::setprecision(3) << "[" << euler.val[0] << ", " << euler.val[1] << ", " << euler.val[2] << "]" << std::endl;
    std::cout << "Translate(x,y,z): " << round->format(t_wc.t()) << std::endl;

    transform.stamp_ = ros::Time::now();
    transform.setRotation(tf::Quaternion(quat[0], quat[1], quat[2], quat[3]));
    transform.setOrigin(tf::Vector3(t_wc.at<double>(0), t_wc.at<double>(1), t_wc.at<double>(2)));
    tf_broadcaster.sendTransform(transform);

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "/world";
    odom.child_frame_id = "/camera";

    geometry_msgs::Quaternion odom_quat;
    odom_quat.x = quat[0];
    odom_quat.y = quat[1];
    odom_quat.z = quat[2];
    odom_quat.w = quat[3];

    //set the position
    odom.pose.pose.position.x = t_wc.at<double>(0);
    odom.pose.pose.position.y = t_wc.at<double>(1);
    odom.pose.pose.position.z = t_wc.at<double>(2);
    odom.pose.pose.orientation = odom_quat;

    // publish to /odom.
    pub_odom.publish(odom);

    cv::Mat dst = vo->GetResultImage();

    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

    cv_ptr->encoding = "bgr8";
    cv_ptr->header.stamp = ros::Time::now();
    cv_ptr->header.frame_id = "/world";
    cv_ptr->image = dst;

    // publish to /image.
    pub_image.publish(cv_ptr->toImageMsg());

    nframe++;
  }

  return 0;
}
