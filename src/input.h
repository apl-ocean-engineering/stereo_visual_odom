#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"

#include <algorithm>
#include <ctime>
#include <ctype.h>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include "Frame.h"
#include "evaluate/evaluate_odometry.h"
#include "feature.h"
#include "utils.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

#include "visualOdometry.h"

#include <cv_bridge/cv_bridge.h>

#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Image.h"

#include "stereo_visual_odom/StereoVisualOdomConfig.h"
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

class Input {
public:
  Input(cv::Mat projleft, cv::Mat projright, cv::Mat Kleft, cv::Mat Kright,
        cv::Mat dleft, cv::Mat dright, std::vector<Matrix> gt, bool display_gt);
  void imageSyncCallback(const sensor_msgs::ImageConstPtr &imgL,
                         const sensor_msgs::ImageConstPtr &imgR);
  void readImages(std::string filepath);
  cv::Mat rosImage2CvMat(sensor_msgs::ImageConstPtr img);
  void run();
  void
  reconfigureCallback(const stereo_visual_odom::StereoVisualOdomConfig &config,
                      uint32_t level);

private:
  clock_t t_a, t_b;
  cv::Mat imageLeft_t0, imageLeft_t1, imageRight_t0, imageRight_t1;
  bool initalized;
  FeatureSet currentVOFeatures;
  cv::Mat projMatrl, projMatrr;
  cv::Mat Kl, Kr;
  cv::Mat dl, dr;

  cv::Mat points4D, points3D;
  int init_frame_id = 0;
  cv::Mat trajectory;
  std::vector<Matrix> pose_matrix_gt;

  bool display_ground_truth;
  cv::Mat rotation;
  cv::Mat translation;

  cv::Mat pose;
  cv::Mat Rpose;

  cv::Mat frame_pose;
  cv::Mat frame_pose32;

  ros::NodeHandle nh_;
  std::string pose_channel;
  ros::Publisher pose_publisher;

  bool new_image = false;

  int frame_id;
};
