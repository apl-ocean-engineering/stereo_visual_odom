#include "stereo_visual_odom/feature.h"
#include "stereo_visual_odom/bucket.h"
#include "stereo_visual_odom/configuration.h"
#include "stereo_visual_odom/utils.h"

#include "g3_to_ros_logger/ROSLogSink.h"
#include "g3_to_ros_logger/g3logger.h"

void deleteUnmatchFeatures(std::vector<cv::Point2f> &points0,
                           std::vector<cv::Point2f> &points1,
                           std::vector<uchar> &status) {
		// getting rid of points for which the KLT tracking failed or those who have
		// gone outside the frame
		int indexCorrection = 0;
		for (int i = 0; i < status.size(); i++) {
				cv::Point2f pt = points1.at(i - indexCorrection);
				if ((status.at(i) == 0) || (pt.x < 0) || (pt.y < 0)) {
						if ((pt.x < 0) || (pt.y < 0)) {
								status.at(i) = 0;
						}
						points0.erase(points0.begin() + (i - indexCorrection));
						points1.erase(points1.begin() + (i - indexCorrection));
						indexCorrection++;
				}
		}
}

void featureDetectionORB(cv::Mat image, std::vector<cv::Point2f> &points,
                         std::vector<cv::KeyPoint> &keypoints,
                         cv::Mat &descriptors) {
		// uses FAST as for feature dection, modify parameters as necessary
		// cv::ORB orb = cv::ORB::create();
		int features = 500;
		float scaleFactor = 1.2f;
		int levels = 8;
		int edgeThreshold = 31;
		int firstLevel = 0;
		int WTA_K = 2;
		int scoreType = cv::ORB::HARRIS_SCORE;
		int patchSize = 31;
		int fastThreshold = 20;

		cv::Ptr<cv::ORB> orb = cv::ORB::create();

		// cv::Ptr<cv::ORB> orb = cv::ORB::create(
		//   features, scaleFactor, levels,
		//   edgeThreshold, WTA_K, scoreType, patchSize, fastThreshold);
		orb->setMaxFeatures(100000);
		// orb->setEdgeThreshold(35);
		// orb->setPatchSize(35);


		orb->detectAndCompute(image, cv::noArray(), keypoints, descriptors);

		orb->setEdgeThreshold(Conf().ORB_edge_threshold);
		orb->setPatchSize(Conf().ORB_patch_size);

		cv::Mat outImg;
		cv::drawKeypoints(image, keypoints, outImg);
		// LOG(INFO) << Conf().display_matched_features;
		if (Conf().display_matched_features) {
				LOG(INFO) << "outImg.size" << outImg.size();
				cv::imshow("outImg", outImg);
				cv::waitKey(1);
		}
		cv::KeyPoint::convert(keypoints, points, std::vector<int>());
}

void featureDetectionFast(cv::Mat image, std::vector<cv::Point2f> &points) {
		// uses FAST as for feature dection, modify parameters as necessary

		std::vector<cv::KeyPoint> keypoints;
		int fast_threshold = 20;
		bool nonmaxSuppression = true;
		cv::FAST(image, keypoints, fast_threshold, nonmaxSuppression);

		cv::Mat outImg;
		cv::drawKeypoints(image, keypoints, outImg);
		cv::KeyPoint::convert(keypoints, points, std::vector<int>());
}

void featureDetectionGoodFeaturesToTrack(cv::Mat image,
                                         std::vector<cv::Point2f> &points) {
		// uses GoodFeaturesToTrack for feature dection, modify parameters as
		// necessary
		std::cout << "featureDetectionGoodFeaturesToTrack" << std::endl;
		int maxCorners = 5000;
		double qualityLevel = 0.0000000000000001;
		double minDistance = 5.;
		int blockSize = 1;
		bool useHarrisDetector = true;
		double k = 0.004;
		cv::Mat mask;

		cv::goodFeaturesToTrack(image, points, maxCorners, qualityLevel, minDistance,
		                        mask, blockSize, useHarrisDetector, k);
}

void featureTracking(cv::Mat img_1, cv::Mat img_2,
                     std::vector<cv::Point2f> &points1,
                     std::vector<cv::Point2f> &points2,
                     std::vector<uchar> &status) {
		// this function automatically gets rid of points for which tracking fails

		std::vector<float> err;
		cv::Size winSize = cv::Size(21, 21);
		cv::TermCriteria termcrit = cv::TermCriteria(
				cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);

		calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3,
		                     termcrit, 0, 0.001);
		deleteUnmatchFeatures(points1, points2, status);
}

void deleteUnmatchFeaturesCircle(
		std::vector<cv::Point2f> &points0, std::vector<cv::Point2f> &points1,
		std::vector<cv::Point2f> &points2, std::vector<cv::Point2f> &points3,
		std::vector<cv::Point2f> &points0_return, std::vector<uchar> &status0,
		std::vector<uchar> &status1, std::vector<uchar> &status2,
		std::vector<uchar> &status3, std::vector<int> &ages) {
		// getting rid of points for which the KLT tracking failed or those who have
		// gone outside the frame
		for (int i = 0; i < ages.size(); ++i) {
				ages[i] += 1;
		}

		int indexCorrection = 0;
		for (int i = 0; i < status3.size(); i++) {
				cv::Point2f pt0 = points0.at(i - indexCorrection);
				cv::Point2f pt1 = points1.at(i - indexCorrection);
				cv::Point2f pt2 = points2.at(i - indexCorrection);
				cv::Point2f pt3 = points3.at(i - indexCorrection);
				cv::Point2f pt0_r = points0_return.at(i - indexCorrection);

				if ((status3.at(i) == 0) || (pt3.x < 0) || (pt3.y < 0) ||
				    (status2.at(i) == 0) || (pt2.x < 0) || (pt2.y < 0) ||
				    (status1.at(i) == 0) || (pt1.x < 0) || (pt1.y < 0) ||
				    (status0.at(i) == 0) || (pt0.x < 0) || (pt0.y < 0)) {
						if ((pt0.x < 0) || (pt0.y < 0) || (pt1.x < 0) || (pt1.y < 0) ||
						    (pt2.x < 0) || (pt2.y < 0) || (pt3.x < 0) || (pt3.y < 0)) {
								status3.at(i) = 0;
						}
						points0.erase(points0.begin() + (i - indexCorrection));
						points1.erase(points1.begin() + (i - indexCorrection));
						points2.erase(points2.begin() + (i - indexCorrection));
						points3.erase(points3.begin() + (i - indexCorrection));
						points0_return.erase(points0_return.begin() + (i - indexCorrection));

						ages.erase(ages.begin() + (i - indexCorrection));
						indexCorrection++;
				}
		}
}

void circularMatching(cv::Mat img_l_0, cv::Mat img_r_0, cv::Mat img_l_1,
                      cv::Mat img_r_1, std::vector<cv::Point2f> &points_l_0,
                      std::vector<cv::Point2f> &points_r_0,
                      std::vector<cv::Point2f> &points_l_1,
                      std::vector<cv::Point2f> &points_r_1,
                      std::vector<cv::Point2f> &points_l_0_return,
                      FeatureSet &current_features) {

		// this function automatically gets rid of points for which tracking fails

		std::vector<float> err1, err2, err3, err4;
		// cv::Size winSize = cv::Size(21, 21);
		cv::Size winSize =
				cv::Size(Conf().optical_flow_win_size, Conf().optical_flow_win_size);
		cv::TermCriteria termcrit = cv::TermCriteria(
				cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);

		std::vector<uchar> status0;
		std::vector<uchar> status1;
		std::vector<uchar> status2;
		std::vector<uchar> status3;

		clock_t tic = clock();

		calcOpticalFlowPyrLK(img_l_0, img_r_0, points_l_0, points_r_0, status0, err1,
		                     winSize, 3, termcrit, 0, 0.001);
		calcOpticalFlowPyrLK(img_r_0, img_r_1, points_r_0, points_r_1, status1, err2,
		                     winSize, 3, termcrit, 0, 0.001);
		calcOpticalFlowPyrLK(img_r_1, img_l_1, points_r_1, points_l_1, status2, err3,
		                     winSize, 3, termcrit, 0, 0.001);
		calcOpticalFlowPyrLK(img_l_1, img_l_0, points_l_1, points_l_0_return, status3,
		                     err4, winSize, 3, termcrit, 0, 0.001);

		drawPoints(img_l_0, points_l_0, 0);
		cv::waitKey(1);
		clock_t toc = clock();
		std::cerr << "calcOpticalFlowPyrLK time: "
		          << float(toc - tic) / CLOCKS_PER_SEC * 1000 << "ms" << std::endl;

		deleteUnmatchFeaturesCircle(points_l_0, points_r_0, points_r_1, points_l_1,
		                            points_l_0_return, status0, status1, status2,
		                            status3, current_features.ages);

		// std::cout << "points : " << points_l_0.size() << " "<< points_r_0.size() <<
		// " "<< points_r_1.size() << " "<< points_l_1.size() << " "<<std::endl;
}

void bucketingFeatures(cv::Mat &image, FeatureSet &current_features,
                       int bucket_size, int features_per_bucket) {
		// This function buckets features
		// image: only use for getting dimension of the image
		// bucket_size: bucket size in pixel is bucket_size*bucket_size
		// features_per_bucket: number of selected features per bucket
		int image_height = image.rows;
		int image_width = image.cols;
		int buckets_nums_height = image_height / bucket_size;
		int buckets_nums_width = image_width / bucket_size;
		int buckets_number = buckets_nums_height * buckets_nums_width;

		std::vector<Bucket> Buckets;

		// initialize all the buckets
		for (int buckets_idx_height = 0; buckets_idx_height <= buckets_nums_height;
		     buckets_idx_height++) {
				for (int buckets_idx_width = 0; buckets_idx_width <= buckets_nums_width;
				     buckets_idx_width++) {
						Buckets.push_back(Bucket(features_per_bucket));
				}
		}

		// bucket all current features into buckets by their location
		int buckets_nums_height_idx, buckets_nums_width_idx, buckets_idx;
		for (int i = 0; i < current_features.left_points.size(); ++i) {
				buckets_nums_height_idx = current_features.left_points[i].y / bucket_size;
				buckets_nums_width_idx = current_features.left_points[i].x / bucket_size;
				buckets_idx =
						buckets_nums_height_idx * buckets_nums_width + buckets_nums_width_idx;
				Buckets[buckets_idx].add_feature(current_features.left_points[i],
				                                 current_features.ages[i]);
		}

		// get features back from buckets
		current_features.clear();
		for (int buckets_idx_height = 0; buckets_idx_height <= buckets_nums_height;
		     buckets_idx_height++) {
				for (int buckets_idx_width = 0; buckets_idx_width <= buckets_nums_width;
				     buckets_idx_width++) {
						buckets_idx = buckets_idx_height * buckets_nums_width + buckets_idx_width;
						Buckets[buckets_idx].get_features(current_features);
				}
		}

		std::cout << "current features number after bucketing: "
		          << current_features.size() << std::endl;
}

void appendNewFeatures(cv::Mat &imageL, cv::Mat &imageR,
                       FeatureSet &current_features) {
		cv::Mat descriptorsL, descriptorsR;
		std::vector<cv::Point2f> points_new;
		std::vector<cv::KeyPoint> keypointsL, keypointsR;

		cv::Mat outImgL, outImgR;
		featureDetectionORB(imageL, points_new, keypointsL, descriptorsL);

		current_features.left_points.insert(current_features.left_points.end(),
		                                    points_new.begin(), points_new.end());
		std::vector<int> ages_new(points_new.size(), 0);
		current_features.ages.insert(current_features.ages.end(), ages_new.begin(),
		                             ages_new.end());
}

void appendNewFeatures(cv::Mat &image, FeatureSet &current_features) {
		std::vector<cv::Point2f> points_new;
		featureDetectionFast(image, points_new);
		current_features.left_points.insert(current_features.left_points.end(),
		                                    points_new.begin(), points_new.end());
		std::vector<int> ages_new(points_new.size(), 0);
		current_features.ages.insert(current_features.ages.end(), ages_new.begin(),
		                             ages_new.end());
}

void appendNewFeatures(std::vector<cv::Point2f> points_new,
                       FeatureSet &current_features) {
		current_features.left_points.insert(current_features.left_points.end(),
		                                    points_new.begin(), points_new.end());
		std::vector<int> ages_new(points_new.size(), 0);
		current_features.ages.insert(current_features.ages.end(), ages_new.begin(),
		                             ages_new.end());
}
