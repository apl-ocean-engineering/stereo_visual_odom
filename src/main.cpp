#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include "input.h"

using namespace std;
using namespace message_filters;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image>
    ImageSyncPolicy;

int main(int argc, char **argv) {
  ros::init(argc, argv, "stereo_odom");
  ros::NodeHandle nh_("stereo_odom");

  // -----------------------------------------
  // Load images and calibration parameters
  // -----------------------------------------
  bool display_ground_truth = false;
  std::vector<Matrix> pose_matrix_gt;
  if (argc == 4) {
    display_ground_truth = true;
    cerr << "Display ground truth trajectory" << endl;
    // load ground truth pose
    string filename_pose = string(argv[3]);
    pose_matrix_gt = loadPoses(filename_pose);
  }
  if (argc < 3) {
    cerr << "Usage: ./run path_to_sequence path_to_calibration "
            "[optional]path_to_ground_truth_pose"
         << endl;
    return 1;
  }

  // Sequence
  string filepath = string(argv[1]);
  cout << "Filepath: " << filepath << endl;

  // Camera calibration
  string strSettingPath = string(argv[2]);
  cout << "Calibration Filepath: " << strSettingPath << endl;

  cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

  float fx = fSettings["Camera.fx"];
  float fy = fSettings["Camera.fy"];
  float cx = fSettings["Camera.cx"];
  float cy = fSettings["Camera.cy"];
  float bf = fSettings["Camera.bf"];

  cv::Mat projMatrl =
      (cv::Mat_<float>(3, 4) << fx, 0., cx, 0., 0., fy, cy, 0., 0, 0., 1., 0.);
  cv::Mat projMatrr =
      (cv::Mat_<float>(3, 4) << fx, 0., cx, bf, 0., fy, cy, 0., 0, 0., 1., 0.);
  cout << "P_left: " << endl << projMatrl << endl;
  cout << "P_right: " << endl << projMatrr << endl;

  // -----------------------------------------
  // Initialize variables
  // -----------------------------------------

  Input input(projMatrl, projMatrr, pose_matrix_gt, display_ground_truth);

  // input.readImages(filepath);

  // std::string left_topic = "/camera/left/image_raw";
  // std::string right_topic = "/camera/right/image_raw";
  std::string left_topic = "/zed/zed_node/left_raw/image_raw_color";
  std::string right_topic = "/zed/zed_node/right_raw/image_raw_color";

  message_filters::Subscriber<sensor_msgs::Image> left_image_sub(nh_,
                                                                 left_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> right_image_sub(
      nh_, right_topic, 1);
  message_filters::Synchronizer<ImageSyncPolicy> imageSync(
      ImageSyncPolicy(10), left_image_sub, right_image_sub);
  //
  imageSync.registerCallback(
      boost::bind(&Input::imageSyncCallback, &input, _1, _2));

  ros::spin();
  // input.run();

  // ------------------------
  // Load first images
  // ------------------------
  // cv::Mat imageLeft_t0_color, imageLeft_t0;
  // loadImageLeft(imageLeft_t0_color, imageLeft_t0, init_frame_id,
  // filepath);
  //
  // cv::Mat imageRight_t0_color, imageRight_t0;
  // loadImageRight(imageRight_t0_color, imageRight_t0, init_frame_id,
  // filepath);
  // clock_t t_a, t_b;
  //
  // // -----------------------------------------
  // // Run visual odometry
  // // -----------------------------------------
  // std::vector<FeaturePoint> oldFeaturePointsLeft;
  // std::vector<FeaturePoint> currentFeaturePointsLeft;
  //
  // for (int frame_id = init_frame_id + 1; frame_id < 9000; frame_id++) {
  //
  //   std::cout << std::endl << "frame_id " << frame_id << std::endl;
  //   // ------------
  //   // Load images
  //   // ------------
  //   cv::Mat imageLeft_t1_color, imageLeft_t1;
  //   loadImageLeft(imageLeft_t1_color, imageLeft_t1, frame_id, filepath);
  //   cv::Mat imageRight_t1_color, imageRight_t1;
  //   loadImageRight(imageRight_t1_color, imageRight_t1, frame_id,
  //   filepath);
  //
  //   t_a = clock();
  //   std::vector<cv::Point2f> oldPointsLeft_t0 = currentVOFeatures.points;
  //
  //   std::vector<cv::Point2f> pointsLeft_t0, pointsRight_t0,
  //   pointsLeft_t1,
  //       pointsRight_t1;
  //
  //   matchingFeatures(imageLeft_t0, imageRight_t0, imageLeft_t1,
  //   imageRight_t1,
  //                    currentVOFeatures, pointsLeft_t0, pointsRight_t0,
  //                    pointsLeft_t1, pointsRight_t1);
  //
  //   imageLeft_t0 = imageLeft_t1;
  //   imageRight_t0 = imageRight_t1;
  //
  //   std::vector<cv::Point2f> &currentPointsLeft_t0 = pointsLeft_t0;
  //   std::vector<cv::Point2f> &currentPointsLeft_t1 = pointsLeft_t1;
  //
  //   std::vector<cv::Point2f> newPoints;
  //   std::vector<bool> valid; // valid new points are ture
  //
  //   // ---------------------
  //   // Triangulate 3D Points
  //   // ---------------------
  //   cv::Mat points3D_t0, points4D_t0;
  //   cv::triangulatePoints(projMatrl, projMatrr, pointsLeft_t0,
  //   pointsRight_t0,
  //                         points4D_t0);
  //   cv::convertPointsFromHomogeneous(points4D_t0.t(), points3D_t0);
  //
  //   cv::Mat points3D_t1, points4D_t1;
  //   cv::triangulatePoints(projMatrl, projMatrr, pointsLeft_t1,
  //   pointsRight_t1,
  //                         points4D_t1);
  //   cv::convertPointsFromHomogeneous(points4D_t1.t(), points3D_t1);
  //
  //   // ---------------------
  //   // Tracking transfomation
  //   // ---------------------
  //   trackingFrame2Frame(projMatrl, projMatrr, pointsLeft_t0,
  //   pointsLeft_t1,
  //                       points3D_t0, rotation, translation, false);
  //   displayTracking(imageLeft_t1, pointsLeft_t0, pointsLeft_t1);
  //
  //   points4D = points4D_t0;
  //   frame_pose.convertTo(frame_pose32, CV_32F);
  //   points4D = frame_pose32 * points4D;
  //   cv::convertPointsFromHomogeneous(points4D.t(), points3D);
  //
  //   // ------------------------------------------------
  //   // Intergrating and display
  //   // ------------------------------------------------
  //
  //   cv::Vec3f rotation_euler = rotationMatrixToEulerAngles(rotation);
  //
  //   cv::Mat rigid_body_transformation;
  //
  //   if (abs(rotation_euler[1]) < 0.1 && abs(rotation_euler[0]) < 0.1 &&
  //       abs(rotation_euler[2]) < 0.1) {
  //     integrateOdometryStereo(frame_id, rigid_body_transformation,
  //     frame_pose,
  //                             rotation, translation);
  //
  //   } else {
  //
  //     std::cout << "Too large rotation" << std::endl;
  //   }
  //   t_b = clock();
  //   float frame_time = 1000 * (double)(t_b - t_a) / CLOCKS_PER_SEC;
  //   float fps = 1000 / frame_time;
  //   cout << "[Info] frame times (ms): " << frame_time << endl;
  //   cout << "[Info] FPS: " << fps << endl;
  //
  //   cv::Mat xyz = frame_pose.col(3).clone();
  //   display(frame_id, trajectory, xyz, pose_matrix_gt, fps,
  //           display_ground_truth);
  // }

  return 0;
}
