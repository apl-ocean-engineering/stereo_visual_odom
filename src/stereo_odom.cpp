#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include "stereo_visual_odom/input.h"

#include "g3_to_ros_logger/ROSLogSink.h"
#include "g3_to_ros_logger/g3logger.h"

#include "stereo_visual_odom/configuration.h"

using namespace std;
using namespace message_filters;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image>
    ImageSyncPolicy;

void loadConfig(ros::NodeHandle nh_) {

  int downsample, optical_flow_win_size, ORB_edge_threshold, ORB_patch_size,
      bucket_size, features_per_bucket;

  bool display_image, display_matched_features;

  ros::param::param<int>(nh_.resolveName("image_downsample"), downsample, 1);
  ros::param::param<int>(nh_.resolveName("optical_flow_win_size"),
                         optical_flow_win_size, 55);
  ros::param::param<int>(nh_.resolveName("ORB_edge_threshold"),
                         ORB_edge_threshold, 62);
  ros::param::param<int>(nh_.resolveName("ORB_patch_size"), ORB_patch_size, 62);
  ros::param::param<int>(nh_.resolveName("bucket_size"), ORB_patch_size, 15);
  ros::param::param<int>(nh_.resolveName("features_per_bucket"),
                         features_per_bucket, 10);
  ros::param::param<bool>(nh_.resolveName("display_image"), display_image,
                          false);
  ros::param::param<bool>(nh_.resolveName("display_matched_features"),
                          display_matched_features, false);

  Conf().downsample = downsample;
  Conf().optical_flow_win_size = optical_flow_win_size;
  Conf().ORB_edge_threshold = ORB_edge_threshold;
  Conf().ORB_patch_size = ORB_patch_size;
  Conf().bucket_size = ORB_patch_size;
  Conf().features_per_bucket = features_per_bucket;
  Conf().display_image = display_image;
  Conf().display_matched_features = display_matched_features;
}

void loadCalibration(cv::Mat &K, cv::Mat &P, cv::Mat &d, std::string name,
                     ros::NodeHandle nh_) {
  std::vector<float> _K;
  std::vector<float> _P;
  std::vector<float> _d;

  Eigen::Matrix3f camera_matrix = Eigen::Matrix3f::Identity();
  Eigen::Matrix<float, 3, 4> projection_matrix;
  Eigen::Matrix<float, 5, 1> distorition_coeffs;

  // Load rotation matrix
  int idx = 0;
  if (nh_.getParam(name + "/camera_matrix/data", _K)) {
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        if (idx != 8)
          camera_matrix(i, j) = _K.at(idx) / Conf().downsample;
        idx++;
      }
    }
    // LOG(DEBUG) << "Loaded rotations: " << R;
  } else {
    ROS_FATAL("Failed to load camera_matrix");
  }
  idx = 0;
  // Load translation vector
  if (nh_.getParam(name + "/projection_matrix/data", _P)) {
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 4; j++) {
        if (idx != 10)
          projection_matrix(i, j) = _P.at(idx) / Conf().downsample;
        else
          projection_matrix(i, j) = _P.at(idx);
        idx++;
      }
    }
  }

  idx = 0;
  // Load translation vector
  if (nh_.getParam(name + "/distortion_coefficients/data", _d)) {
    for (int i = 0; i < 5; i++) {
      distorition_coeffs(i, 0) = _d.at(idx);
      idx++;
    }
  } else {
    ROS_FATAL("Failed to load projection_matrix.");
  }

  eigen2cv(camera_matrix, K);
  eigen2cv(projection_matrix, P);

  eigen2cv(distorition_coeffs, d);
}

int main(int argc, char **argv) {
  libg3logger::G3Logger<ROSLogSink> logWorker(argv[0]);
  logWorker.logBanner();
  logWorker.verbose(2);

  std::string name = "stereo_odom";

  ros::init(argc, argv, name);
  ros::NodeHandle nh_(name);

  loadConfig(nh_);

  // -----------------------------------------
  // Load images and calibration parameters
  // -----------------------------------------
  bool display_ground_truth = false;
  std::vector<Matrix> pose_matrix_gt;

  cv::Mat_<float> Kl(3, 3);
  cv::Mat_<float> Kr(3, 3);
  cv::Mat_<float> dl(5, 1);
  cv::Mat_<float> dr(5, 1);
  cv::Mat_<float> projMatrl(3, 4);
  cv::Mat_<float> projMatrr(3, 4);


  loadCalibration(Kl, projMatrl, dl, "/" + name + "/left", nh_);
loadCalibration(Kr, projMatrr, dr, "/" + name + "/right", nh_);

  Input input(projMatrl, projMatrr, Kl, Kr, dl, dr, pose_matrix_gt,
              display_ground_truth);

  std::string left_topic = "/left/image";
  std::string right_topic = "/right/image";

  message_filters::Subscriber<sensor_msgs::Image> left_image_sub(nh_,
                                                                 left_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> right_image_sub(
      nh_, right_topic, 1);
  message_filters::Synchronizer<ImageSyncPolicy> imageSync(
      ImageSyncPolicy(10), left_image_sub, right_image_sub);
  //
  imageSync.registerCallback(
      boost::bind(&Input::imageSyncCallback, &input, _1, _2));

  dynamic_reconfigure::Server<stereo_visual_odom::StereoVisualOdomConfig>
      reconfigure_callback(nh_);
  dynamic_reconfigure::Server<
      stereo_visual_odom::StereoVisualOdomConfig>::CallbackType f;

  f = boost::bind(&Input::reconfigureCallback, &input, _1, _2);
  reconfigure_callback.setCallback(f);

  ros::spin();

  return 0;
}
