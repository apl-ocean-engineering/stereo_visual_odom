#ifndef VISUAL_ODOM_H
#define VISUAL_ODOM_H

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
#include "bucket.h"
#include "feature.h"
#include "utils.h"

void matchingFeatures(cv::Mat imageLeft_t0, cv::Mat imageRight_t0,
                      cv::Mat imageLeft_t1, cv::Mat imageRight_t1,
                      FeatureSet &currentVOFeatures,
                      std::vector<cv::Point2f> &pointsLeft_t0,
                      std::vector<cv::Point2f> &pointsRight_t0,
                      std::vector<cv::Point2f> &pointsLeft_t1,
                      std::vector<cv::Point2f> &pointsRight_t1);

// void trackingFrame2Frame(cv::Mat points3D_t0, cv::Mat points3D_t1,
//                          cv::Mat &rotation, cv::Mat &translation);

void trackingFrame2Frame(cv::Mat &projMatrl, cv::Mat &projMatrr, cv::Mat Kl,
                         cv::Mat dl, std::vector<cv::Point2f> &pointsLeft_t0,
                         std::vector<cv::Point2f> &pointsLeft_t1,
                         cv::Mat &points3D_t0, cv::Mat &rotation,
                         cv::Mat &translation, cv::Point3f &meanPoint, bool mono_rotation = true);

void displayTracking(cv::Mat &imageLeft_t1,
                     std::vector<cv::Point2f> &pointsLeft_t0,
                     std::vector<cv::Point2f> &pointsLeft_t1);

#endif
