#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include "input.h"

#include "g3_to_ros_logger/ROSLogSink.h"
#include "g3_to_ros_logger/g3logger.h"

using namespace std;
using namespace message_filters;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image>
        ImageSyncPolicy;


void loadCalibration(cv::Mat &K, cv::Mat &P, std::string name, ros::NodeHandle nh_, int downsample=1){
        std::vector<float> _K;
        std::vector<float> _P;

        Eigen::Matrix3f camera_matrix = Eigen::Matrix3f::Identity();
        Eigen::Matrix<float, 3, 4> projection_matrix;

        // Load rotation matrix
        int idx = 0;
        std::cout << nh_.getParam(name + "/camera_matrix/data", _K) <<  std::endl;
        if (nh_.getParam(name + "/camera_matrix/data", _K)) {
                for (int i = 0; i < 3; i++) {
                        for (int j = 0; j < 3; j++) {
                                camera_matrix(i, j) = _K.at(idx)/downsample;
                                idx++;
                        }
                }
                // LOG(DEBUG) << "Loaded rotations: " << R;
        }
        else {
                ROS_FATAL("Failed to load camera_matrix");

        }
        idx = 0;
        // Load translation vector
        if (nh_.getParam(name + "/projection_matrix/data", _P)) {
                for (int i = 0; i < 3; i++) {
                        for (int j = 0; j < 4; j++) {
                                //std::cout << _P.at(idx) <<std::endl;
                                projection_matrix(i, j) = _P.at(idx)/downsample;
                                idx++;
                        }
                }
        }

        else {
                ROS_FATAL("Failed to load projection_matrix.");
        }
        projection_matrix(3,3) = 1;
        eigen2cv(camera_matrix,K);
        eigen2cv(projection_matrix,P);
}


int main(int argc, char **argv) {
        libg3logger::G3Logger<ROSLogSink> logWorker(argv[0]);
        logWorker.logBanner();
        logWorker.verbose(2);

        std::string name = "stereo_odom";

        ros::init(argc, argv, name);
        ros::NodeHandle nh_(name);

        int downsample = 1;

        // -----------------------------------------
        // Load images and calibration parameters
        // -----------------------------------------
        bool display_ground_truth = false;
        std::vector<Matrix> pose_matrix_gt;
        // if (argc == 4) {
        //   display_ground_truth = true;
        //   cerr << "Display ground truth trajectory" << endl;
        //   // load ground truth pose
        //   string filename_pose = string(argv[3]);
        //   pose_matrix_gt = loadPoses(filename_pose);
        // }
        // if (argc < 3) {
        //   cerr << "Usage: ./run path_to_sequence path_to_calibration "
        //           "[optional]path_to_ground_truth_pose"
        //        << endl;
        //   return 1;
        // }

        // Sequence
        // string filepath = string(argv[1]);
        // LOG(DEBUG) << "Filepath: " << filepath;
        //
        // // Camera calibration
        // string strSettingPath = string(argv[2]);
        // LOG(DEBUG) << "Calibration Filepath: " << strSettingPath;

        // cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        //
        // float fx = fSettings["Camera.fx"];
        // float fy = fSettings["Camera.fy"];
        // float cx = fSettings["Camera.cx"];
        // float cy = fSettings["Camera.cy"];
        // float bf = fSettings["Camera.bf"];



        //cv::Mat projMatrl;
        // =
        //     (cv::Mat_<float>(3, 4) << fx/downsample, 0.,
        //      cx/downsample, 0., 0., fy/downsample, cy/downsample, 0., 0, 0., 1., 0.);
        //cv::Mat projMatrr;
        // =
        //    (cv::Mat_<float>(3, 4) << fx/downsample, 0., cx/downsample, bf/downsample,
        //    0., fy/downsample, cy/downsample, 0., 0, 0., 1., 0.);


        cv::Mat Kl, Kr;
        cv::Mat_<float> projMatrl(3, 4); // projMatrl; //(3, 4, CV_32F);
        cv::Mat_<float> projMatrr(3, 4); //(3, 4, CV_32F);
        //cv::Mat_<float>(3, 4) projMatrl, projMatrr;

        loadCalibration(Kl, projMatrl, "/" + name + "/left", nh_, downsample);
        loadCalibration(Kr, projMatrr, "/" + name + "/right", nh_, downsample);

        //std::cout << projMatrl << std::endl;

        //LOG(INFO) << "P_left: " << endl << projMatrl;
        //LOG(INFO) << "P_right: " << endl << projMatrr;

        // -----------------------------------------
        // Initialize variables
        // -----------------------------------------

        Input input(projMatrl, projMatrr, pose_matrix_gt, display_ground_truth, downsample);

        // input.readImages(filepath);
        // //
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

        return 0;
}
