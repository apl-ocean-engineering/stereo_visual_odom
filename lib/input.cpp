#include "stereo_visual_odom/input.h"

#include "g3_to_ros_logger/ROSLogSink.h"
#include "g3_to_ros_logger/g3logger.h"

#include "stereo_visual_odom/configuration.h"
#include <math.h>

using namespace std;

Input::Input(cv::Mat projleft, cv::Mat projright, cv::Mat Kleft, cv::Mat Kright,
             cv::Mat dleft, cv::Mat dright, std::vector<Matrix> gt,
             bool display_gt)
        : projMatrl(projleft), projMatrr(projright), Kl(Kleft), Kr(Kright),
        dl(dleft), dr(dright), pose_matrix_gt(gt), frame_id(0), initalized(false),
        display_ground_truth(display_gt) {
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
        imageLeft_t1 = rosImage2CvMat(imgL);
        imageRight_t1 = rosImage2CvMat(imgR);

        if (Conf().downsample != 1) {
                cv::resize(imageLeft_t1, imageLeft_t1,
                           cv::Size(imageLeft_t1.cols / Conf().downsample,
                                    imageLeft_t1.rows / Conf().downsample));
                cv::resize(imageRight_t1, imageRight_t1,
                           cv::Size(imageRight_t1.cols / Conf().downsample,
                                    imageRight_t1.rows / Conf().downsample));
        }

        cv::Mat out1, out2;

        if (frame_id > 0) { //&& frame_id % 4 == 0
                run();
        }

        imageLeft_t0 = imageLeft_t1;
        imageRight_t0 = imageRight_t1;

        frame_id++;
}
//
void Input::readImages(std::string filepath) {
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

        cv::Mat imL0_unEdited = imageLeft_t0.clone();
        cv::Mat imL1_unEdited = imageLeft_t1.clone();
        cv::Mat imR0_unEdited = imageRight_t0.clone();
        cv::Mat imR1_unEdited = imageRight_t1.clone();

        matchingFeatures(imageLeft_t0, imageRight_t0, imageLeft_t1, imageRight_t1,
                         currentVOFeatures, pointsLeft_t0, pointsRight_t0,
                         pointsLeft_t1, pointsRight_t1);

        if (pointsLeft_t0.size() < 6) {
                return;
        }

        std::vector<cv::Point2f> &currentPointsLeft_t0 = pointsLeft_t0;
        std::vector<cv::Point2f> &currentPointsLeft_t1 = pointsLeft_t1;
        //
        std::vector<cv::Point2f> newPoints;
        std::vector<bool> valid; // valid new points are ture

        displayMatches(imL0_unEdited, imR0_unEdited, pointsLeft_t0, pointsRight_t0);

        // ---------------------
        // Triangulate 3D Points
        // ---------------------
        cv::Mat points4D_t0;
        std::vector<cv::Point3f> points3D_t0V;
        if (pointsLeft_t0.size() < 6) {
                LOG(INFO) << "Not enough points";
                return;
        }

        cv::triangulatePoints(projMatrl, projMatrr, pointsLeft_t0, pointsRight_t0,
                              points4D_t0);

        cv::convertPointsFromHomogeneous(points4D_t0.t(), points3D_t0V);

        cv::Mat points4D_t1;
        std::vector<cv::Point3f> points3D_t1V;

        cv::triangulatePoints(projMatrl, projMatrr, pointsLeft_t1, pointsRight_t1,
                              points4D_t1);

        cv::convertPointsFromHomogeneous(points4D_t1.t(), points3D_t1V);

        cv::Mat temp1, temp2;
        cv::Point3f mean_t0, mean_t1;
        for (int i = 0; i < pointsLeft_t0.size(); i++) {
                cv::Point2f p0 = pointsLeft_t0.at(i);
                cv::Point2f p1 = pointsRight_t0.at(i);
                cv::Point3f p_3 = points3D_t0V.at(i);
                cv::Point3f p_31 = points3D_t1V.at(i);
                if (cv::norm(p_31 - p_3) > Conf().motion_threshold) {
                        pointsLeft_t0.erase(pointsLeft_t0.begin() + i);
                        pointsRight_t0.erase(pointsRight_t0.begin() + i);
                        points3D_t0V.erase(points3D_t0V.begin() + i);
                        points3D_t1V.erase(points3D_t1V.begin() + i);
                        pointsLeft_t1.erase(pointsLeft_t1.begin() + i);
                        pointsRight_t1.erase(pointsRight_t1.begin() + i);
                }
        }




        cv::Mat points3D_t1 = cv::Mat(points3D_t1V);
        cv::Mat points3D_t0 = cv::Mat(points3D_t0V);


        cv::Mat_<double> RBT = FindRigidTransform(points3D_t1, points3D_t0);

        rotation = RBT(cv::Range(0,3),cv::Range(0,3));
        LOG(WARNING) << "rot" << rotation;
        translation.at<double>(0,0) = RBT.at<double>(0,3);
        translation.at<double>(1,0) = RBT.at<double>(1,3);
        translation.at<double>(2,0) = RBT.at<double>(2,3);

        LOG(WARNING) << "translation norm: "
                     << pow(pow(translation.at<double>(0, 0), 2) +
               pow(translation.at<double>(0, 1), 2) +
               pow(translation.at<double>(0, 2), 2),
               1.0 / 2.0);

        displayTracking(imageLeft_t1, pointsLeft_t0, pointsLeft_t1);
        frame_pose.convertTo(frame_pose32, CV_32F);
        // ------------------------------------------------
        // Intergrating and display
        // ------------------------------------------------
        cv::Vec3f rotation_euler = rotationMatrixToEulerAngles(rotation);

        cv::Mat rigid_body_transformation;

        if (abs(rotation_euler[1]) < 0.1 && abs(rotation_euler[0]) < 0.1 &&
            abs(rotation_euler[2]) < 0.1 && abs(translation.at<double>(0, 0)) < 0.1 &&
            abs(translation.at<double>(0, 1)) < 0.1 &&
            abs(translation.at<double>(0, 2)) < 0.1) {
                integrateOdometryStereo(frame_id, rigid_body_transformation, frame_pose,
                                        rotation, translation);
        } else {
                LOG(WARNING) << "Translation too large" << std::endl;
        }
        t_b = clock();
        float frame_time = 1000 * (double)(t_b - t_a) / CLOCKS_PER_SEC;
        float fps = 1000 / frame_time;
        LOG(INFO) << "frame times (ms): " << frame_time;
        LOG(INFO) << "FPS: " << fps;

        Eigen::Matrix4f eigen_fp;

        cv2eigen(frame_pose, eigen_fp);
        //
        Eigen::Matrix3f R = eigen_fp.block<3, 3>(0, 0);
        Eigen::Quaternionf q(R);
        //
        cv::Mat xyz = frame_pose.col(3).clone();
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header.frame_id = "map";
        poseStamped.pose.position.x = xyz.at<double>(2);
        poseStamped.pose.position.y = xyz.at<double>(0);
        poseStamped.pose.position.z = xyz.at<double>(1);
        //
        poseStamped.pose.orientation.x = q.z();
        poseStamped.pose.orientation.y = q.x();
        poseStamped.pose.orientation.z = q.y();
        poseStamped.pose.orientation.w = q.w();

        pose_publisher.publish(poseStamped);
        display(frame_id, trajectory, xyz, pose_matrix_gt, fps, display_ground_truth);
        new_image = false;
}
void Input::reconfigureCallback(
        const stereo_visual_odom::StereoVisualOdomConfig &config, uint32_t level) {
        Conf().downsample = config.image_downsample;

        if (config.optical_flow_win_size % 2 != 0) {
                Conf().optical_flow_win_size = config.optical_flow_win_size;
        } else {
                Conf().optical_flow_win_size = config.optical_flow_win_size - 1;
        }

        Conf().ORB_edge_threshold = config.edge_threshold;
        Conf().ORB_patch_size = config.patch_size;
        Conf().bucket_size = config.bucket_size;
        Conf().features_per_bucket = config.features_per_bucket;
}

// }
//}
