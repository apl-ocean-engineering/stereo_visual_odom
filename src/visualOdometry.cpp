#include "visualOdometry.h"
#include "configuration.h"

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

#include <math.h>

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
                     std::vector<bool> &status, int threshold,
                     int criteria = 1) {
  int offset;
  for (int i = 0; i < points.size(); i++) {
    if (criteria == 1) {
      offset = std::max(std::abs(points[i].x - points_return[i].x),
                        std::abs(points[i].y - points_return[i].y));
    } else
      offset = std::abs(points[i].y - points_return[i].y);

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

void matchingFeatures(cv::Mat imageLeft_t0, cv::Mat imageRight_t0,
                      cv::Mat imageLeft_t1, cv::Mat imageRight_t1,
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
  }

  // --------------------------------------------------------
  // Feature tracking using KLT tracker, bucketing and circular matching
  // --------------------------------------------------------
  int bucket_size = Conf().bucket_size;
  int features_per_bucket = Conf().features_per_bucket;
  LOG(WARNING) << bucket_size << " " << features_per_bucket;
  bucketingFeatures(imageLeft_t0, currentVOFeatures, bucket_size,
                    features_per_bucket);

  pointsLeft_t0 = currentVOFeatures.left_points;
  if (pointsLeft_t0.size() < 6) {
    return;
  }

  circularMatching(imageLeft_t0, imageRight_t0, imageLeft_t1, imageRight_t1,
                   pointsLeft_t0, pointsRight_t0, pointsLeft_t1, pointsRight_t1,
                   pointsLeftReturn_t0, currentVOFeatures);

  std::vector<bool> status1, status2, status3;
  checkValidMatch(pointsLeft_t0, pointsLeftReturn_t0, status1, 0);

  int status1Before = pointsLeft_t0.size();

  removeInvalidPoints(pointsLeft_t0, status1);
  removeInvalidPoints(pointsLeft_t1, status1);
  removeInvalidPoints(pointsRight_t0, status1);
  removeInvalidPoints(pointsRight_t1, status1);
  LOG(WARNING) << "Points removed by status 1 removal: "
               << status1Before - pointsLeft_t0.size();
  //
  //
  //
  // checkValidMatch(pointsLeft_t0, pointsRight_t0, status2, 55, 2);
  // // //
  // int status2Before = pointsLeft_t0.size();
  // // //
  // // //
  // removeInvalidPoints(pointsLeft_t0, status2);
  // removeInvalidPoints(pointsLeft_t1, status2);
  // removeInvalidPoints(pointsRight_t0, status2);
  // removeInvalidPoints(pointsRight_t1, status2);
  //
  // LOG(WARNING) << "Points removed by status 2 removal: "
  //              << status2Before - pointsLeft_t0.size();
  // //
  // checkValidMatch(pointsLeft_t0, pointsRight_t0, status3, 25);
  // //
  // int status3Before = pointsLeft_t0.size();
  //
  //
  // removeInvalidPoints(pointsLeft_t0, status3);
  // removeInvalidPoints(pointsLeft_t1, status3);
  // removeInvalidPoints(pointsRight_t0, status3);
  // removeInvalidPoints(pointsRight_t1, status3);
  //
  // LOG(WARNING) << "Points removed by status 3 removal: " << status3Before -
  // pointsLeft_t0.size();

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

  cv::Point3f mean_diff;
  mean_diff.x = 0;
  mean_diff.y = 0;
  mean_diff.z = 0;

  int idx = 0;

  for (int i = 0; i < points3D_t0.rows; i++) {
    cv::Point3f point0 = points3D_t0.at<cv::Vec3f>(i, 0);
    cv::Point3f point1 = points3D_t1.at<cv::Vec3f>(i, 0);


    mean_diff.x += (point0.x - point1.x);
    mean_diff.y += (point0.y - point1.y);
    mean_diff.z += (point0.z - point1.z);

    idx++;

  }
  mean_diff.x = mean_diff.x / idx;
  mean_diff.y = mean_diff.y / idx;
  mean_diff.z = mean_diff.z / idx;

  int count = 0;

  for (int i = 0; i < points3D_t0.rows; i++) {
    cv::Point3f point0 = points3D_t0.at<cv::Vec3f>(i, 0);
    cv::Point3f point1 = points3D_t1.at<cv::Vec3f>(i, 0);

    float normalized_diff = pow((point0.x - point1.x) / mean_diff.x +
                                    (point0.y - point1.y) / mean_diff.y +
                                    (point0.z - point1.z) / mean_diff.z,
                                2);
    if (normalized_diff < 5) {

      count += 1;
      cloud_in->points[i].x = point0.x;
      cloud_in->points[i].y = point0.y;
      cloud_in->points[i].z = point0.z;

      cloud_out->points[i].x = point1.x;
      cloud_out->points[i].y = point1.y;
      cloud_out->points[i].z = point1.z;
    }
  }

  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>
      TESVD;
  pcl::registration::TransformationEstimationSVD<
      pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transformation2;
  TESVD.estimateRigidTransformation(*cloud_in, *cloud_out, transformation2);

  Eigen::Matrix3d R = transformation2.block<3, 3>(0, 0).cast<double>();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(R, Eigen::ComputeThinU |
                                               Eigen::ComputeThinV);
  R = svd.matrixU() * (svd.matrixV().transpose()); // Final rotation

  Eigen::Vector3d t = transformation2.block<3, 1>(0, 3).cast<double>();
  eigen2cv(R, rotation);
  eigen2cv(t, translation);
}

void trackingFrame2Frame(cv::Mat &projMatrl, cv::Mat &projMatrr, cv::Mat Kl,
                         cv::Mat dl, std::vector<cv::Point2f> &pointsLeft_t0,
                         std::vector<cv::Point2f> &pointsLeft_t1,
                         cv::Mat &points3D_t0, cv::Mat &rotation,
                         cv::Mat &translation, cv::Point3f &meanPoint, bool mono_rotation) {

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
  cv::recoverPose(E, pointsLeft_t0, pointsLeft_t1, rotation, translation_mono,
                  focal, principle_point, mask);

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

  int iterationsCount = 500;        // number of Ransac iterations.
  float reprojectionError = 0.0001; // maximum allowed distance to
  // consider it an inlier.
  float confidence = 0.999; // RANSAC
  // // successful confidence.
  bool useExtrinsicGuess = true;
  int flags = cv::SOLVEPNP_EPNP;

  cv::Mat translation_init = cv::Mat::zeros(3, 1, CV_64F);
  translation_init.at<double>(0,0) = meanPoint.x;
  translation_init.at<double>(1,0) = meanPoint.y;
  translation_init.at<double>(2,0) = meanPoint.z;


  cv::solvePnPRansac(points3D_t0, pointsLeft_t1, intrinsic_matrix, dl, rvec,
                     translation_init, useExtrinsicGuess, iterationsCount,
                     reprojectionError, confidence, inliers, flags);

  if (!mono_rotation) {
    cv::Rodrigues(rvec, rotation);
  }
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

  cv::imshow("vis", vis);
}
