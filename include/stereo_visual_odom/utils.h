#ifndef UTILS_H
#define UTILS_H

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

#include "evaluate/matrix.h"
#include "feature.h"

// --------------------------------
// Visualization
// --------------------------------
void drawFeaturePoints(cv::Mat image, std::vector<cv::Point2f> &points);

void display(int frame_id, cv::Mat &trajectory, cv::Mat &pose,
             std::vector<Matrix> &pose_matrix_gt, float fps, bool showgt);

void integrateOdometryStereo(cv::Mat &frame_pose, const cv::Mat rotation,
                             const cv::Mat translation);

// --------------------------------
// Transformation
// --------------------------------
void integrateOdometryStereo(int frame_id, cv::Mat &rigid_body_transformation,
                             cv::Mat &frame_pose, const cv::Mat &rotation,
                             const cv::Mat &translation_stereo);

bool isRotationMatrix(cv::Mat &R);

cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R);

void drawPoints(cv::Mat img, std::vector<cv::Point2f> points, int i);

cv::Vec3d
CalculateMean(const cv::Mat_<cv::Vec3d> &points);

cv::Mat_<double>
FindRigidTransform(const cv::Mat_<cv::Vec3d> &points1, const cv::Mat_<cv::Vec3d> points2);

// --------------------------------
// I/O
// --------------------------------

void loadImageLeft(cv::Mat &image_color, cv::Mat &image_gary, int frame_id,
                   std::string filepath);

void loadImageRight(cv::Mat &image_color, cv::Mat &image_gary, int frame_id,
                    std::string filepath);

void loadGyro(std::string filename,
              std::vector<std::vector<double>> &time_gyros);

void displayMatches(cv::Mat img1, cv::Mat img2,
                    std::vector<cv::Point2f> points1,
                    std::vector<cv::Point2f> points2);

// read time gyro txt file with format of timestamp, gx, gy, gz

#endif
