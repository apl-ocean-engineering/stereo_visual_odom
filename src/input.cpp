#include "input.h"

#include "g3_to_ros_logger/ROSLogSink.h"
#include "g3_to_ros_logger/g3logger.h"

using namespace std;

Input::Input(cv::Mat leftK, cv::Mat rightK, std::vector<Matrix> gt,
             bool display_gt)
    : projMatrl(leftK), projMatrr(rightK), pose_matrix_gt(gt), frame_id(0),
      initalized(false), display_ground_truth(display_gt) {
  trajectory = cv::Mat::zeros(600, 1200, CV_8UC3);
  rotation = cv::Mat::eye(3, 3, CV_64F);
  translation = cv::Mat::zeros(3, 1, CV_64F);

  pose = cv::Mat::zeros(3, 1, CV_64F);
  Rpose = cv::Mat::eye(3, 3, CV_64F);

  frame_pose = cv::Mat::eye(4, 4, CV_64F);
  frame_pose32 = cv::Mat::eye(4, 4, CV_32F);

  LOG(INFO) << "frame_pose " << frame_pose;

  pose_channel = nh_.resolveName("manipulation_slam/pose");
  pose_publisher = nh_.advertise<geometry_msgs::PoseStamped>(pose_channel, 1);
}

void Input::imageSyncCallback(const sensor_msgs::ImageConstPtr &imgL,
                              const sensor_msgs::ImageConstPtr &imgR) {
  // std::cout << "here" << std::endl;
  imageLeft_t1 = rosImage2CvMat(imgL);
  imageRight_t1 = rosImage2CvMat(imgR);

  // cv::resize(imageLeft_t1, imageLeft_t1,
  //            cv::Size(imageLeft_t1.cols / 4, imageLeft_t1.rows / 4));
  // cv::resize(imageRight_t1, imageRight_t1,
  //            cv::Size(imageRight_t1.cols / 4, imageRight_t1.rows / 4));

  if (frame_id > 0) {
    run();
  }

  imageLeft_t0 = imageLeft_t1;
  imageRight_t0 = imageRight_t1;

  frame_id++;
}
//
void Input::readImages(std::string filepath) {
  //   cv::Mat imageLeft_t0_color, imageLeft_t0;
  //   loadImageLeft(imageLeft_t0_color, imageLeft_t0, init_frame_id, filepath);
  //
  //   cv::Mat imageRight_t0_color, imageRight_t0;
  //   loadImageRight(imageRight_t0_color, imageRight_t0, init_frame_id,
  //   filepath); clock_t t_a, t_b;
  //
  //   // -----------------------------------------
  //   // Run visual odometry
  //   // -----------------------------------------
  //   std::vector<FeaturePoint> oldFeaturePointsLeft;
  //   std::vector<FeaturePoint> currentFeaturePointsLeft;
  //
  //   for (int frame_id = init_frame_id + 1; frame_id < 9000; frame_id++) {
  //
  //     std::cout << std::endl << "frame_id " << frame_id << std::endl;
  //     // ------------
  //     // Load images
  //     // ------------
  //     cv::Mat imageLeft_t1_color, imageLeft_t1;
  //     loadImageLeft(imageLeft_t1_color, imageLeft_t1, frame_id, filepath);
  //     cv::Mat imageRight_t1_color, imageRight_t1;
  //     loadImageRight(imageRight_t1_color, imageRight_t1, frame_id, filepath);
  //
  //     t_a = clock();
  //     std::vector<cv::Point2f> oldPointsLeft_t0 = currentVOFeatures.points;
  //
  //     std::vector<cv::Point2f> pointsLeft_t0, pointsRight_t0, pointsLeft_t1,
  //         pointsRight_t1;
  //
  //     matchingFeatures(imageLeft_t0, imageRight_t0, imageLeft_t1,
  //     imageRight_t1,
  //                      currentVOFeatures, pointsLeft_t0, pointsRight_t0,
  //                      pointsLeft_t1, pointsRight_t1);
  //
  //     imageLeft_t0 = imageLeft_t1;
  //     imageRight_t0 = imageRight_t1;
  //
  //     std::vector<cv::Point2f> &currentPointsLeft_t0 = pointsLeft_t0;
  //     std::vector<cv::Point2f> &currentPointsLeft_t1 = pointsLeft_t1;
  //
  //     std::vector<cv::Point2f> newPoints;
  //     std::vector<bool> valid; // valid new points are ture
  //
  //     // ---------------------
  //     // Triangulate 3D Points
  //     // ---------------------
  //     cv::Mat points3D_t0, points4D_t0;
  //     cv::triangulatePoints(projMatrl, projMatrr, pointsLeft_t0,
  //     pointsRight_t0,
  //                           points4D_t0);
  //     cv::convertPointsFromHomogeneous(points4D_t0.t(), points3D_t0);
  //
  //     cv::Mat points3D_t1, points4D_t1;
  //     cv::triangulatePoints(projMatrl, projMatrr, pointsLeft_t1,
  //     pointsRight_t1,
  //                           points4D_t1);
  //     cv::convertPointsFromHomogeneous(points4D_t1.t(), points3D_t1);
  //
  //     // ---------------------
  //     // Tracking transfomation
  //     // ---------------------
  //     trackingFrame2Frame(projMatrl, projMatrr, pointsLeft_t0, pointsLeft_t1,
  //                         points3D_t0, rotation, translation, false);
  //     displayTracking(imageLeft_t1, pointsLeft_t0, pointsLeft_t1);
  //
  //     points4D = points4D_t0;
  //     frame_pose.convertTo(frame_pose32, CV_32F);
  //     points4D = frame_pose32 * points4D;
  //     cv::convertPointsFromHomogeneous(points4D.t(), points3D);
  //     //
  //     // // ------------------------------------------------
  //     // // Intergrating and display
  //     // // ------------------------------------------------
  //     //
  //     cv::Vec3f rotation_euler = rotationMatrixToEulerAngles(rotation);
  //
  //     cv::Mat rigid_body_transformation;
  //
  //     if (abs(rotation_euler[1]) < 0.1 && abs(rotation_euler[0]) < 0.1 &&
  //         abs(rotation_euler[2]) < 0.1) {
  //       integrateOdometryStereo(frame_id, rigid_body_transformation,
  //       frame_pose,
  //                               rotation, translation);
  //
  //     } else {
  //
  //       std::cout << "Too large rotation" << std::endl;
  //     }
  //     t_b = clock();
  //     float frame_time = 1000 * (double)(t_b - t_a) / CLOCKS_PER_SEC;
  //     float fps = 1000 / frame_time;
  //     cout << "[Info] frame times (ms): " << frame_time << endl;
  //     cout << "[Info] FPS: " << fps << endl;
  //     cout << "frame_pose " << frame_pose << endl;
  //
  //     // std::cout << "rigid_body_transformation" <<
  //     rigid_body_transformation
  //     // <<
  //     // std::endl; std::cout << "rotation: " << rotation_euler << std::endl;
  //     // std::cout << "translation: " << translation.t() << std::endl;
  //     // std::cout << "frame_pose" << frame_pose << std::endl;
  //
  //     cv::Mat xyz = frame_pose.col(3).clone();
  //     display(frame_id, trajectory, xyz, pose_matrix_gt, fps,
  //             display_ground_truth);
  //   }
}
//
cv::Mat Input::rosImage2CvMat(sensor_msgs::ImageConstPtr img) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    // LOGF(WARNING, "cv_bridge exception: %s", e.what());
    return cv::Mat();
  }

  return cv_ptr->image;
}

void Input::run() {
  // if (new_image) {
  LOG(INFO) << std::endl << "frame_id " << frame_id;
  t_a = clock();
  std::vector<cv::Point2f> oldPointsLeft_t0 = currentVOFeatures.left_points;

  std::vector<cv::Point2f> pointsLeft_t0, pointsRight_t0, pointsLeft_t1,
      pointsRight_t1;

  matchingFeatures(imageLeft_t0, imageRight_t0, imageLeft_t1, imageRight_t1,
                   currentVOFeatures, pointsLeft_t0, pointsRight_t0,
                   pointsLeft_t1, pointsRight_t1);

  std::vector<cv::Point2f> &currentPointsLeft_t0 = pointsLeft_t0;
  std::vector<cv::Point2f> &currentPointsLeft_t1 = pointsLeft_t1;
  //
  std::vector<cv::Point2f> newPoints;
  std::vector<bool> valid; // valid new points are ture

  // ---------------------
  // Triangulate 3D Points
  // ---------------------
  cv::Mat points3D_t0, points4D_t0;
  // if (pointsLeft_t0.size() > 5) {
  if (pointsLeft_t0.size() < 6) {
    LOG(INFO) << "Not enough points";
    return;
  }

  // OLD FORMAT

  cv::triangulatePoints(projMatrl, projMatrr, pointsLeft_t0, pointsRight_t0,
                        points4D_t0);

  cv::convertPointsFromHomogeneous(points4D_t0.t(), points3D_t0);

  cv::Mat points3D_t1, points4D_t1;

  cv::triangulatePoints(projMatrl, projMatrr, pointsLeft_t1, pointsRight_t1,
                        points4D_t1);

  cv::convertPointsFromHomogeneous(points4D_t1.t(), points3D_t1);

  // // NEW FORMAT
  // std::vector<cv::Point3f> points3D_t0V, points3D_t1V;
  // cv::Mat points4D_t0V, points4D_t1V;
  //
  // cv::triangulatePoints(projMatrl, projMatrr, points3D_t0V, points3D_t1V,
  //                       points4D_t0V);
  //
  // cv::convertPointsFromHomogeneous(points4D_t0V.t(), points3D_t0V);
  //
  // LOG(INFO) << points3D_t0V;
  //
  // // ---------------------
  // // Tracking transfomation
  // // ---------------------
  // trackingFrame2Frame(points3D_t0, points3D_t1, rotation, translation);

  trackingFrame2Frame(projMatrl, projMatrr, pointsLeft_t0, pointsLeft_t1,
                      points3D_t0, rotation, translation, false);
  displayTracking(imageLeft_t1, pointsLeft_t0, pointsLeft_t1);
  frame_pose.convertTo(frame_pose32, CV_32F);
  //
  LOG(INFO) << "rotation: " << std::endl << rotation;

  // ------------------------------------------------
  // Intergrating and display
  // ------------------------------------------------

  // cv::Vec3f rotation_euler = rotationMatrixToEulerAngles(rotation);

  cv::Mat rigid_body_transformation;

  // if (abs(rotation_euler[1]) < 0.1 && abs(rotation_euler[0]) < 0.1 &&
  //     abs(rotation_euler[2]) < 0.1) {
  integrateOdometryStereo(frame_id, rigid_body_transformation, frame_pose,
                          rotation, translation);
  //
  // // } else {
  // //   std::cout << "Too large rotation" << std::endl;
  // // }
  t_b = clock();
  float frame_time = 1000 * (double)(t_b - t_a) / CLOCKS_PER_SEC;
  float fps = 1000 / frame_time;
  LOG(INFO) << "frame times (ms): " << frame_time;
  LOG(INFO) << "FPS: " << fps;

  Eigen::Matrix4f eigen_fp;

  cv2eigen(frame_pose, eigen_fp);

  Eigen::Matrix3f R = eigen_fp.block<3, 3>(0, 0);
  Eigen::Quaternionf q(R);
  //
  cv::Mat xyz = frame_pose.col(3).clone();
  geometry_msgs::PoseStamped poseStamped;
  poseStamped.header.frame_id = "map";
  poseStamped.pose.position.x = xyz.at<double>(2);
  poseStamped.pose.position.y = xyz.at<double>(0);
  poseStamped.pose.position.z = xyz.at<double>(1);

  poseStamped.pose.orientation.x = q.z();
  poseStamped.pose.orientation.y = q.x();
  poseStamped.pose.orientation.z = q.y();
  poseStamped.pose.orientation.w = q.w();
  pose_publisher.publish(poseStamped);
  display(frame_id, trajectory, xyz, pose_matrix_gt, fps, display_ground_truth);
  new_image = false;
}
// }
//}
