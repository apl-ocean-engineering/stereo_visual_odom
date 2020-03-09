#include "visualOdometry.h"

#include "g3_to_ros_logger/ROSLogSink.h"
#include "g3_to_ros_logger/g3logger.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

#include <pcl/common/angles.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/transformation_estimation_svd.h>

cv::Mat euler2rot(cv::Mat &rotationMatrix, const cv::Mat &euler) {

  double x = euler.at<double>(0);
  double y = euler.at<double>(1);
  double z = euler.at<double>(2);

  // Assuming the angles are in radians.
  double ch = cos(z);
  double sh = sin(z);
  double ca = cos(y);
  double sa = sin(y);
  double cb = cos(x);
  double sb = sin(x);

  double m00, m01, m02, m10, m11, m12, m20, m21, m22;

  m00 = ch * ca;
  m01 = sh * sb - ch * sa * cb;
  m02 = ch * sa * sb + sh * cb;
  m10 = sa;
  m11 = ca * cb;
  m12 = -ca * sb;
  m20 = -sh * ca;
  m21 = sh * sa * cb + ch * sb;
  m22 = -sh * sa * sb + ch * cb;

  rotationMatrix.at<double>(0, 0) = m00;
  rotationMatrix.at<double>(0, 1) = m01;
  rotationMatrix.at<double>(0, 2) = m02;
  rotationMatrix.at<double>(1, 0) = m10;
  rotationMatrix.at<double>(1, 1) = m11;
  rotationMatrix.at<double>(1, 2) = m12;
  rotationMatrix.at<double>(2, 0) = m20;
  rotationMatrix.at<double>(2, 1) = m21;
  rotationMatrix.at<double>(2, 2) = m22;

  return rotationMatrix;
}

void checkValidMatch(std::vector<cv::Point2f> &points,
                     std::vector<cv::Point2f> &points_return,
                     std::vector<bool> &status, int threshold) {
  int offset;
  for (int i = 0; i < points.size(); i++) {
    offset = std::max(std::abs(points[i].x - points_return[i].x),
                      std::abs(points[i].y - points_return[i].y));
    // std::cout << offset << ", ";

    if (offset > threshold) {
      status.push_back(false);
    } else {
      status.push_back(true);
    }
  }
}

void removeInvalidPoints(std::vector<cv::Point2f> &points,
                         const std::vector<bool> &status) {
  int index = 0;
  for (int i = 0; i < status.size(); i++) {
    if (status[i] == false) {
      points.erase(points.begin() + index);
    } else {
      index++;
    }
  }
}

void matchingFeatures(cv::Mat &imageLeft_t0, cv::Mat &imageRight_t0,
                      cv::Mat &imageLeft_t1, cv::Mat &imageRight_t1,
                      std::vector<cv::Point2f> &pointsLeft_t0,
                      std::vector<cv::Point2f> &pointsRight_t0,
                      std::vector<cv::Point2f> &pointsLeft_t1,
                      std::vector<cv::Point2f> &pointsRight_t1){

    cv::Mat descriptorsL_t0, descriptorsL_t1, descriptorsR_t0, descriptorsR_t1;

    std::vector<cv::KeyPoint> keypointsL_t0, keypointsL_t1,
      keypointsR_t0, keypointsR_t1;
    featureDetectionORB(imageLeft_t0, pointsLeft_t0,
      keypointsL_t0, descriptorsL_t0);
    featureDetectionORB(imageLeft_t1, pointsLeft_t1,
      keypointsL_t1, descriptorsL_t1);
    featureDetectionORB(imageRight_t0, pointsRight_t0,
      keypointsR_t0, descriptorsR_t0);
    featureDetectionORB(imageRight_t1, pointsRight_t1,
      keypointsR_t1, descriptorsR_t1);


  std::vector<cv::DMatch> matchesL_R_t0, matchesL_R_t1, matchesL_t0_t1, matchesR_t0_t1;


  match(imageLeft_t0, imageRight_t0, keypointsL_t0, keypointsR_t0,
    descriptorsL_t0, descriptorsR_t0, matchesL_R_t0);

  // match(imageLeft_t0, imageLeft_t1, keypointsL_t0, keypointsL_t1,
  //   descriptorsL_t0, descriptorsL_t1, matchesL_t0_t1);

  match(imageLeft_t1, imageRight_t1, keypointsL_t1, keypointsR_t1,
    descriptorsL_t1, descriptorsR_t1, matchesL_R_t1);

  // match(imageRight_t0, imageRight_t1, keypointsR_t0, keypointsR_t1,
  //   descriptorsR_t0, descriptorsR_t1, matchesR_t0_t1);



  cv::KeyPoint::convert(keypointsL_t0, pointsLeft_t0, std::vector<int>());
  cv::KeyPoint::convert(keypointsR_t0, pointsRight_t0, std::vector<int>());
  cv::KeyPoint::convert(keypointsL_t1, pointsLeft_t1, std::vector<int>());
  cv::KeyPoint::convert(keypointsR_t1, pointsRight_t1, std::vector<int>());
}
void match(cv::Mat img1, cv::Mat img2, std::vector<cv::KeyPoint> &kpoints1,
          std::vector<cv::KeyPoint> &kpoints2, cv::Mat descriptors1,
          cv::Mat descriptors2, std::vector<cv::DMatch> &matches){

  featureDetectionORB(img1, kpoints1, descriptors1);
  featureDetectionORB(img2, kpoints2, descriptors2);


  cv::Ptr<cv::DescriptorMatcher> matcher =
      cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
  std::vector<std::vector<cv::DMatch>> knn_matches;
  matcher->knnMatch(descriptors1, descriptors2, knn_matches, 2);

  const float ratio_thresh = 0.7f;
  std::vector<cv::KeyPoint> returnKp1, returnKp2;
  for (size_t i = 0; i < knn_matches.size(); i++) {
    if (knn_matches[i][0].distance <
        ratio_thresh * knn_matches[i][1].distance) {
      matches.push_back(knn_matches[i][0]);
    }
  }

  for (int i=0; i<matches.size(); i++){
    // std::cout << matches.at(i).trainIdx << " " << matches.at(i).queryIdx << std::endl;
    returnKp1.push_back(kpoints1.at(matches.at(i).queryIdx));
    returnKp2.push_back(kpoints2.at(matches.at(i).trainIdx));
  }
  //-- Draw matches
  cv::Mat img_matches, outImg1, outImg2;
  // cv::drawMatches(img1, returnKp1, img2, returnKp2, matches,
  //                 img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
  //                 std::vector<char>(),
  //                 cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  //
  kpoints1 = returnKp1;
  kpoints2 = returnKp2;

  std::cout << kpoints1.size() << " " <<  kpoints2.size() << std::endl;
  // std::cout << keypointsR_t0.size() << " " <<  keypointsR_t1.size() << std::endl;

  cv::drawKeypoints(img1, returnKp1, outImg1);
  cv::drawKeypoints(img2, returnKp2, outImg2);
  // //
  // // //-- Show detected matches
  // cv::imshow("Good img1", outImg1);
  // cv::imshow("Good img2", outImg2);
  // cv::waitKey(1);

  }

void matchingFeatures(cv::Mat &imageLeft_t0, cv::Mat &imageRight_t0,
                      cv::Mat &imageLeft_t1, cv::Mat &imageRight_t1,
                      FeatureSet &currentVOFeatures,
                      std::vector<cv::Point2f> &pointsLeft_t0,
                      std::vector<cv::Point2f> &pointsRight_t0,
                      std::vector<cv::Point2f> &pointsLeft_t1,
                      std::vector<cv::Point2f> &pointsRight_t1) {
  // ----------------------------
  // Feature detection using FAST
  // ----------------------------
  std::vector<cv::Point2f> pointsLeftReturn_t0; // feature points to check
                                                // cicular mathcing validation

  if (currentVOFeatures.size() < 2000) {

    // append new features with old features
    appendNewFeatures(imageLeft_t0, imageRight_t0, currentVOFeatures);
    // std::cout << "Current feature set size: " <<
    // currentVOFeatures.points.size() << std::endl;
  }

  // --------------------------------------------------------
  // Feature tracking using KLT tracker, bucketing and circular matching
  // --------------------------------------------------------
  int bucket_size = 25;
  int features_per_bucket = 2;
  bucketingFeatures(imageLeft_t0, currentVOFeatures, bucket_size,
                    features_per_bucket);

  pointsLeft_t0 = currentVOFeatures.left_points;

  circularMatching(imageLeft_t0, imageRight_t0, imageLeft_t1, imageRight_t1,
                   pointsLeft_t0, pointsRight_t0, pointsLeft_t1, pointsRight_t1,
                   pointsLeftReturn_t0, currentVOFeatures);

  std::vector<bool> status;
  checkValidMatch(pointsLeft_t0, pointsLeftReturn_t0, status, 0);

  removeInvalidPoints(pointsLeft_t0, status);
  removeInvalidPoints(pointsLeft_t1, status);
  removeInvalidPoints(pointsRight_t0, status);
  removeInvalidPoints(pointsRight_t1, status);

  // cv::Mat imgL = imageLeft_t1;
  // cv::Mat imgR = imageLeft_t0;
  // for (int i = 0; i < pointsLeft_t0.size(); i++) {
  //   cv::Point2f l0 = pointsLeft_t1.at(i);
  //   cv::Point2f r0 = pointsLeft_t0.at(i);
  //   cv::circle(imgL, cv::Point2i(int(l0.x), int(l0.y)), 10,
  //              cv::Scalar(255, 255, 255));
  //   cv::circle(imgR, cv::Point2i(int(r0.x), int(r0.y)), 10,
  //              cv::Scalar(255, 255, 255));
  //   cv::imshow("imgL", imgL);
  //   cv::imshow("imgR", imgR);
  //   cv::waitKey();
  // }

  currentVOFeatures.left_points = pointsLeft_t1;
}

void trackingFrame2Frame(cv::Mat points3D_t0, cv::Mat points3D_t1,
                         cv::Mat &rotation, cv::Mat &translation) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(
      new pcl::PointCloud<pcl::PointXYZ>());
  cloud_in->width = points3D_t0.rows;
  cloud_in->height = 1;
  cloud_in->is_dense = false;
  cloud_in->resize(cloud_in->width * cloud_in->height);

  cloud_out->width = points3D_t0.rows;
  cloud_out->height = 1;
  cloud_out->is_dense = false;
  cloud_out->resize(cloud_out->width * cloud_out->height);

  for (int i = 0; i < points3D_t0.rows; i++) {
    cv::Point3f point0 = points3D_t0.at<cv::Vec3f>(i, 0);
    cv::Point3f point1 = points3D_t1.at<cv::Vec3f>(i, 0);

    Eigen::Vector3d v0(point0.x, point0.y, point0.z);
    Eigen::Vector3d v1(point1.x, point1.y, point1.z);

    if ((v0 - v1).norm() < 0.1) {

      cloud_in->points[i].x = point0.x;
      cloud_in->points[i].y = point0.y;
      cloud_in->points[i].z = point0.z;

      cloud_out->points[i].x = point1.x;
      cloud_out->points[i].y = point1.y;
      cloud_out->points[i].z = point1.z;
    } else {
      std::cout << (v0 - v1).norm() << std::endl;
    }
  }
  // LOG(WARNING) << p0 << std::endl << std::endl << p1;
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>
      TESVD;
  pcl::registration::TransformationEstimationSVD<
      pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transformation2;
  TESVD.estimateRigidTransformation(*cloud_in, *cloud_out, transformation2);
  LOG(WARNING) << "The Estimated Rotation and translation matrices(using "
                  "getTransformation function) are : \n"
               << transformation2;

  Eigen::Matrix3d R = transformation2.block<3, 3>(0, 0).cast<double>();
  Eigen::Vector3d t = transformation2.block<3, 1>(0, 3).cast<double>();

  eigen2cv(R, rotation);
  eigen2cv(t, translation);
}

void trackingFrame2Frame(cv::Mat &projMatrl, cv::Mat &projMatrr,
                         std::vector<cv::Point2f> &pointsLeft_t0,
                         std::vector<cv::Point2f> &pointsLeft_t1,
                         cv::Mat &points3D_t0, cv::Mat &rotation,
                         cv::Mat &translation, bool mono_rotation) {

  // Calculate frame to frame transformation

  // -----------------------------------------------------------
  // Rotation(R) estimation using Nister's Five Points Algorithm
  // -----------------------------------------------------------
  double focal = projMatrl.at<float>(0, 0);
  cv::Point2d principle_point(projMatrl.at<float>(0, 2),
                              projMatrl.at<float>(1, 2));

  // //recovering the pose and the essential cv::matrix
  cv::Mat E, mask;
  cv::Mat translation_mono = cv::Mat::zeros(3, 1, CV_64F);
  E = cv::findEssentialMat(pointsLeft_t0, pointsLeft_t1, focal, principle_point,
                           cv::RANSAC, 0.999, 1.0, mask);
  if (E.rows != 3 || E.cols != 3) {
    LOG(WARNING) << "E not proper size";
    LOG(INFO) << E;
    LOG(INFO) << pointsLeft_t0;
    return;
  }
  // std::cout << E << std::endl;
  cv::recoverPose(E, pointsLeft_t0, pointsLeft_t1, rotation, translation_mono,
                  focal, principle_point, mask);
  // std::cout << "recoverPose rotation: " << rotation << std::endl;

  // ------------------------------------------------
  // Translation (t) estimation by use solvePnPRansac
  // ------------------------------------------------
  cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);
  cv::Mat inliers;
  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
  cv::Mat intrinsic_matrix =
      (cv::Mat_<float>(3, 3) << projMatrl.at<float>(0, 0),
       projMatrl.at<float>(0, 1), projMatrl.at<float>(0, 2),
       projMatrl.at<float>(1, 0), projMatrl.at<float>(1, 1),
       projMatrl.at<float>(1, 2), projMatrl.at<float>(1, 1),
       projMatrl.at<float>(1, 2), projMatrl.at<float>(1, 3));
  //
  int iterationsCount = 500;    // number of Ransac iterations.
  float reprojectionError = .5; // maximum allowed distance to
  // consider it an inlier.
  float confidence = 0.999; // RANSAC
  // // successful confidence.
  bool useExtrinsicGuess = true;
  int flags = cv::SOLVEPNP_ITERATIVE;
  // //
  cv::solvePnPRansac(points3D_t0, pointsLeft_t1, intrinsic_matrix, distCoeffs,
                     rvec, translation, useExtrinsicGuess, iterationsCount,
                     reprojectionError, confidence, inliers, flags);

  if (!mono_rotation) {
    cv::Rodrigues(rvec, rotation);
  }
  //
  // std::cout << "[trackingFrame2Frame] inliers size: " << inliers.size()
  // << std::endl;
}

void displayTracking(cv::Mat &imageLeft_t1,
                     std::vector<cv::Point2f> &pointsLeft_t0,
                     std::vector<cv::Point2f> &pointsLeft_t1) {
  // -----------------------------------------
  // Display feature racking
  // -----------------------------------------
  cv::namedWindow("vis", cv::WINDOW_NORMAL);
  int radius = 2;
  cv::Mat vis;

  cv::cvtColor(imageLeft_t1, vis, CV_GRAY2BGR, 3);

  for (int i = 0; i < pointsLeft_t0.size(); i++) {
    cv::circle(vis, cvPoint(pointsLeft_t0[i].x, pointsLeft_t0[i].y), radius,
               CV_RGB(0, 255, 0));
  }

  for (int i = 0; i < pointsLeft_t1.size(); i++) {
    cv::circle(vis, cvPoint(pointsLeft_t1[i].x, pointsLeft_t1[i].y), radius,
               CV_RGB(255, 0, 0));
  }

  for (int i = 0; i < pointsLeft_t1.size(); i++) {
    cv::line(vis, pointsLeft_t0[i], pointsLeft_t1[i], CV_RGB(0, 255, 0));
  }

  cv::imshow("vis ", vis);
}
