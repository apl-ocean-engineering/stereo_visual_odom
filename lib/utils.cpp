#include "stereo_visual_odom/utils.h"
#include "evaluate/evaluate_odometry.h"

#include "g3_to_ros_logger/ROSLogSink.h"
#include "g3_to_ros_logger/g3logger.h"

#include "stereo_visual_odom/configuration.h"

// --------------------------------
// Visualization
// --------------------------------
void drawFeaturePoints(cv::Mat image, std::vector<cv::Point2f> &points) {
		int radius = 2;

		for (int i = 0; i < points.size(); i++) {
				circle(image, cvPoint(points[i].x, points[i].y), radius,
				       CV_RGB(255, 255, 255));
		}
}

void display(int frame_id, cv::Mat &trajectory, cv::Mat &pose,
             std::vector<Matrix> &pose_matrix_gt, float fps, bool show_gt) {
		// draw estimated trajectory
		int x = int(pose.at<double>(0)) + 300;
		int y = int(pose.at<double>(2)) + 100;

		circle(trajectory, cv::Point(x, y), 1, CV_RGB(255, 0, 0), 2);
		if (show_gt) {

				cv::Mat pose_gt = cv::Mat::zeros(1, 3, CV_64F);

				pose_gt.at<double>(0) = pose_matrix_gt[frame_id].val[0][3];
				pose_gt.at<double>(1) = pose_matrix_gt[frame_id].val[0][7];
				pose_gt.at<double>(2) = pose_matrix_gt[frame_id].val[0][11];
				x = int(pose_gt.at<double>(0)) + 300;
				y = int(pose_gt.at<double>(2)) + 100;
				circle(trajectory, cv::Point(x, y), 1, CV_RGB(255, 255, 0), 2);
		}
}

// --------------------------------
// Transformation
// --------------------------------

void displayMatches(cv::Mat img1, cv::Mat img2,
                    std::vector<cv::Point2f> points1,
                    std::vector<cv::Point2f> points2) {
		cv::Mat dst;

		cv::hconcat(img1, img2, dst);
		cv::cvtColor(dst, dst, CV_GRAY2BGR);
		// Conf().display_matched_features = true;
		cv::RNG rng(12345);
		for (int i = 0; i < points1.size(); i++) {
				cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
				                              rng.uniform(0, 255));
				cv::Point2i p1 = cv::Point2i(int(points1.at(i).x), int(points1.at(i).y));
				cv::Point2i p2 =
						cv::Point2i(int(img1.cols + points2.at(i).x), int(points2.at(i).y));

				cv::circle(dst, p1, 5, color);
				cv::circle(dst, p2, 5, color);
				cv::line(dst, p1, p2, color);
		}
		if (Conf().display_matched_features) {
				cv::imshow("dst", dst);
				cv::waitKey(1);
		}
}

void drawPoints(cv::Mat img, std::vector<cv::Point2f> points, int idx) {
		for (int i = 0; i < points.size(); i++) {
				cv::Point2f p = points.at(i);
				cv::Point2i pi(int(p.x), int(p.y));
				cv::circle(img, pi, 2, cv::Scalar(255, 255, 255));
		}
		if (Conf().display_matched_features) {
				cv::imshow("features" + std::to_string(idx), img);
		}
}

void integrateOdometryStereo(cv::Mat &frame_pose, const cv::Mat rotation,
                             const cv::Mat translation) {
		cv::Mat addup = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);
		cv::Mat G;
		cv::hconcat(rotation, translation, G);
		cv::vconcat(G, addup, G);
}

cv::Vec3d CalculateMean(const cv::Mat_<cv::Vec3d> &points) {
		cv::Mat_<cv::Vec3d> result;
		cv::reduce(points, result, 0, CV_REDUCE_AVG);
		return result(0, 0);
}

cv::Mat_<double> FindRigidTransform(const cv::Mat_<cv::Vec3d> &points1,
                                    const cv::Mat_<cv::Vec3d> points2) {
		// taken from
		// here:https://stackoverflow.com/questions/21206870/opencv-rigid-transformation-between-two-3d-point-clouds
		/* Calculate centroids. */
		cv::Vec3d t1 = -CalculateMean(points1);
		cv::Vec3d t2 = -CalculateMean(points2);

		cv::Mat_<double> T1 = cv::Mat_<double>::eye(4, 4);
		T1(0, 3) = t1[0];
		T1(1, 3) = t1[1];
		T1(2, 3) = t1[2];

		cv::Mat_<double> T2 = cv::Mat_<double>::eye(4, 4);
		T2(0, 3) = -t2[0];
		T2(1, 3) = -t2[1];
		T2(2, 3) = -t2[2];

		/* Calculate covariance matrix for input points. Also calculate RMS deviation
		 * from centroid which is used for scale calculation.
		 */
		cv::Mat_<double> C(3, 3, 0.0);
		double p1Rms = 0, p2Rms = 0;
		for (int ptIdx = 0; ptIdx < points1.rows; ptIdx++) {
				cv::Vec3d p1 = points1(ptIdx, 0) + t1;
				cv::Vec3d p2 = points2(ptIdx, 0) + t2;
				p1Rms += p1.dot(p1);
				p2Rms += p2.dot(p2);
				for (int i = 0; i < 3; i++) {
						for (int j = 0; j < 3; j++) {
								C(i, j) += p2[i] * p1[j];
						}
				}
		}

		cv::Mat_<double> u, s, vh;
		cv::SVD::compute(C, s, u, vh);

		cv::Mat_<double> R = u * vh;

		if (cv::determinant(R) < 0) {
				R -= u.col(2) * (vh.row(2) * 2.0);
		}

		// double scale = sqrt(p2Rms / p1Rms);
		// R *= scale;

		cv::Mat_<double> M = cv::Mat_<double>::eye(4, 4);
		R.copyTo(M.colRange(0, 3).rowRange(0, 3));

		cv::Mat_<double> result = T2 * M * T1;
		result /= result(3, 3);

		// cv::Mat trans = -R*cv::Mat(t2) + t1;

		return result.rowRange(0, 3);
}

void integrateOdometryStereo(int frame_i, cv::Mat &rigid_body_transformation,
                             cv::Mat &frame_pose, const cv::Mat &rotation,
                             const cv::Mat &translation_stereo) {

		cv::Mat addup = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);

		cv::hconcat(rotation, translation_stereo, rigid_body_transformation);

		cv::vconcat(rigid_body_transformation, addup, rigid_body_transformation);

		double scale = sqrt(
				(translation_stereo.at<double>(0)) * (translation_stereo.at<double>(0)) +
				(translation_stereo.at<double>(1)) * (translation_stereo.at<double>(1)) +
				(translation_stereo.at<double>(2)) * (translation_stereo.at<double>(2)));

		rigid_body_transformation = rigid_body_transformation.inv();
		if (scale < 10) {
				frame_pose = frame_pose * rigid_body_transformation;

		} else {
				LOG(WARNING) << "scale below 0.1, or incorrect translation";
		}
}

bool isRotationMatrix(cv::Mat &R) {
		cv::Mat Rt;
		transpose(R, Rt);
		cv::Mat shouldBeIdentity = Rt * R;
		cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());
		return norm(I, shouldBeIdentity) < 1e-6;
}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R) {

		assert(isRotationMatrix(R));

		float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) +
		                R.at<double>(1, 0) * R.at<double>(1, 0));

		bool singular = sy < 1e-6; // If

		float x, y, z;
		if (!singular) {
				x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
				y = atan2(-R.at<double>(2, 0), sy);
				z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
		} else {
				x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
				y = atan2(-R.at<double>(2, 0), sy);
				z = 0;
		}
		return cv::Vec3f(x, y, z);
}

// --------------------------------
// I/O
// --------------------------------

void loadGyro(std::string filename,
              std::vector<std::vector<double> > &time_gyros)
// read time gyro txt file with format of timestamp, gx, gy, gz
{
		std::ifstream file(filename);

		std::string value;
		double timestamp, gx, gy, gz;

		while (file.good()) {

				std::vector<double> time_gyro;

				getline(file, value, ' ');
				timestamp = stod(value);
				time_gyro.push_back(timestamp);

				getline(file, value, ' ');
				gx = stod(value);
				time_gyro.push_back(gx);

				getline(file, value, ' ');
				gy = stod(value);
				time_gyro.push_back(gy);

				getline(file, value);
				gz = stod(value);
				time_gyro.push_back(gz);

				// printf("t: %f, gx: %f, gy: %f, gz: %f\n" , timestamp, gx, gy, gz);

				time_gyros.push_back(time_gyro);
		}
}

void loadImageLeft(cv::Mat &image_color, cv::Mat &image_gary, int frame_id,
                   std::string filepath) {
		char file[200];
		sprintf(file, "image_0/%06d.png", frame_id);

		// sprintf(file, "image_0/%010d.png", frame_id);
		std::string filename = filepath + std::string(file);

		image_color = cv::imread(filename, cv::IMREAD_COLOR);
		cvtColor(image_color, image_gary, cv::COLOR_BGR2GRAY);
}

void loadImageRight(cv::Mat &image_color, cv::Mat &image_gary, int frame_id,
                    std::string filepath) {
		char file[200];
		sprintf(file, "image_1/%06d.png", frame_id);

		// sprintf(file, "image_0/%010d.png", frame_id);
		std::string filename = filepath + std::string(file);

		image_color = cv::imread(filename, cv::IMREAD_COLOR);
		cvtColor(image_color, image_gary, cv::COLOR_BGR2GRAY);
}
