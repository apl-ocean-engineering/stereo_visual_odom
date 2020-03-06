#include "input.h"

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

  std::cout << "frame_pose " << frame_pose << std::endl;
}

void Input::imageSyncCallback(const sensor_msgs::ImageConstPtr &imgL,
                              const sensor_msgs::ImageConstPtr &imgR) {
  // std::cout << "here" << std::endl;
  imageLeft_t1 = rosImage2CvMat(imgL);
  imageRight_t1 = rosImage2CvMat(imgR);

  if (initalized) {
    imageLeft_t0 = imageLeft_t1;
    imageRight_t0 = imageRight_t1;
    new_image = true;
  }

  if (!initalized)
    initalized = true;
  frame_id++;

  cv::imshow("imageRight_t1", imageRight_t1);
  cv::waitKey(1);
}

cv::Mat Input::rosImage2CvMat(sensor_msgs::ImageConstPtr img) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    // LOGF(WARNING, "cv_bridge exception: %s", e.what());
    return cv::Mat();
  }

  return cv_ptr->image;
}

void Input::run() {
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    if (initalized && new_image) {
      t_a = clock();
      std::vector<cv::Point2f> oldPointsLeft_t0 = currentVOFeatures.points;

      std::vector<cv::Point2f> pointsLeft_t0, pointsRight_t0, pointsLeft_t1,
          pointsRight_t1;

      // cv::imshow("imageLeft_t0", imageLeft_t0);
      // cv::imshow("imageRight_t0", imageRight_t0);
      // cv::imshow("imageLeft_t1", imageLeft_t1);
      // cv::imshow("imageRight_t1", imageRight_t1);
      // cv::waitKey(1000);

      matchingFeatures(imageLeft_t0, imageRight_t0, imageLeft_t1, imageRight_t1,
                       currentVOFeatures, pointsLeft_t0, pointsRight_t0,
                       pointsLeft_t1, pointsRight_t1);

      std::vector<cv::Point2f> &currentPointsLeft_t0 = pointsLeft_t0;
      std::vector<cv::Point2f> &currentPointsLeft_t1 = pointsLeft_t1;

      std::vector<cv::Point2f> newPoints;
      std::vector<bool> valid; // valid new points are ture

      // ---------------------
      // Triangulate 3D Points
      // ---------------------
      cv::Mat points3D_t0, points4D_t0;
      cv::triangulatePoints(projMatrl, projMatrr, pointsLeft_t0, pointsRight_t0,
                            points4D_t0);
      cv::convertPointsFromHomogeneous(points4D_t0.t(), points3D_t0);

      cv::Mat points3D_t1, points4D_t1;
      cv::triangulatePoints(projMatrl, projMatrr, pointsLeft_t1, pointsRight_t1,
                            points4D_t1);
      cv::convertPointsFromHomogeneous(points4D_t1.t(), points3D_t1);

      // ---------------------
      // Tracking transfomation
      // ---------------------
      trackingFrame2Frame(projMatrl, projMatrr, pointsLeft_t0, pointsLeft_t1,
                          points3D_t0, rotation, translation, false);
      displayTracking(imageLeft_t1, pointsLeft_t0, pointsLeft_t1);

      points4D = points4D_t0;
      frame_pose.convertTo(frame_pose32, CV_32F);
      points4D = frame_pose32 * points4D;
      cv::convertPointsFromHomogeneous(points4D.t(), points3D);

      // ------------------------------------------------
      // Intergrating and display
      // ------------------------------------------------

      cv::Vec3f rotation_euler = rotationMatrixToEulerAngles(rotation);

      cv::Mat rigid_body_transformation;

      if (abs(rotation_euler[1]) < 0.1 && abs(rotation_euler[0]) < 0.1 &&
          abs(rotation_euler[2]) < 0.1) {
        integrateOdometryStereo(frame_id, rigid_body_transformation, frame_pose,
                                rotation, translation);

      } else {
        std::cout << "Too large rotation" << std::endl;
      }
      t_b = clock();
      float frame_time = 1000 * (double)(t_b - t_a) / CLOCKS_PER_SEC;
      float fps = 1000 / frame_time;
      cout << "[Info] frame times (ms): " << frame_time << endl;
      cout << "[Info] FPS: " << fps << endl;

      cv::Mat xyz = frame_pose.col(3).clone();
      display(frame_id, trajectory, xyz, pose_matrix_gt, fps,
              display_ground_truth);
      new_image = false;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
