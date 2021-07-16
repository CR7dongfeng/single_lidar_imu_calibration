#include <dirent.h>
#include <iostream>
#include <string>
#include <vector>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "calib_options.h"
#include "calibration_once.h"
#include "frame.h"

namespace jf {

void createFrames(
		std::vector<std::shared_ptr<Frame>> &frames,
		const vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &local_lidar_scans,
		const vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>
		&vec_relative_imu,
		const CalibrationOptions &calib_options) {
	if (local_lidar_scans.size() != vec_relative_imu.size()) {
		LOG(FATAL) << "local_lidar_scans.size() != vec_relative_imu.size()";
	}
	
	vector<pcl::PointCloud<pcl::PointNormal>::Ptr> processed_local_lidar_scans;
	for (size_t i = 0; i < local_lidar_scans.size(); i++) {
		processed_local_lidar_scans.emplace_back(
				cloudPreProcess(local_lidar_scans[i]));
	}
	
//	cout << "local_lidar_scans.size()" << local_lidar_scans.size() << endl;
	
	for (size_t i = 0; i < local_lidar_scans.size() - 1; i++) {
		Eigen::Isometry3d relative_imu = Eigen::Isometry3d::Identity();
		relative_imu = vec_relative_imu[i].inverse() * vec_relative_imu[i + 1];
		if (relative_imu.translation().norm() > calib_options.min_move_distance) {
			std::shared_ptr<Frame> f(new Frame());
			f->move_t_ = relative_imu;
			
			f->cloud1_ = processed_local_lidar_scans[i];
			f->cloud2_ = processed_local_lidar_scans[i + 1];
			f->copyToEigen();
			
			f->cloud1_.reset();
			f->cloud2_.reset();
			
			frames.push_back(f);
		}
	}
	
	cout << "frames.size()" << frames.size() << endl;
}

Eigen::Isometry3d calibrationOnce(
		vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &local_lidar_scans,
		const vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>
		&vec_relative_imu,
		const Eigen::Isometry3d &extrinsic_input,
		const CalibrationOptions &calib_options) {
	Eigen::Isometry3d ret;
	
	// 对frames进行初始化
	std::vector<std::shared_ptr<Frame>> frames;
	createFrames(frames, local_lidar_scans, vec_relative_imu, calib_options);
	local_lidar_scans.clear();
	
	Frame::extrinsic_ = extrinsic_input;
	
	double thresh = calib_options.max_correspondence_distance_upper;
	int iteration_counter = 0;
	
	for (auto f : frames) {
		f->filterPtsByCorrespondences(1.5 * thresh);
	}
	
	// 开始标定
	for (int i = 0; i < calib_options.max_iterations; i++) {
		// 1、对frame进行滤波
		for (auto f : frames) {
			f->getClosestPoints(thresh);
			f->filterCorrespondences();
			f->filterCorrespondencesByNormal();
		}
		
		// 2、进行ceres优化
		Eigen::Quaterniond q_former(Frame::extrinsic_.linear());
		Eigen::Vector3d t_former(Frame::extrinsic_.translation());
		
		ICP_Ceres::ceresOptimizer(frames, true, true, calib_options.calib_stage);
		
		Eigen::Quaterniond q_latter(Frame::extrinsic_.linear());
		Eigen::Vector3d t_latter(Frame::extrinsic_.translation());
		
		std::cout << "iterator: i = " << i << std::endl;
		std::cout << "thresh = " << thresh << std::endl;
		
		// 3、判断阈值并决定是否继续
		double rot_residual =
				fabs(1 - ((q_former * q_latter.conjugate()).normalized()).w());
		std::cout << "inner: rot_residual is " << rot_residual << endl;
		double trans_residual = (t_former - t_latter).norm();
		std::cout << "inner: trans_residual is " << trans_residual << endl;
		
		iteration_counter++;
		
		bool R_converge = ((calib_options.calib_stage == ICP_Ceres::CALIB_R_STEP) &&
		                   (rot_residual < calib_options.rotation_epsilon));
		bool t_converge =
				((calib_options.calib_stage == ICP_Ceres::CALIB_XY_STEP) &&
				 (trans_residual < calib_options.translation_epsilon));
		
		if (iteration_counter >= calib_options.delay_iterations || R_converge ||
		    t_converge) {
			thresh -= calib_options.distance_step;
			iteration_counter = 0;
			
			if (thresh < calib_options.max_correspondence_distance_lower -
			             calib_options.distance_step / 2) {
				if (R_converge || t_converge)
					break;
				else
					thresh = calib_options.max_correspondence_distance_lower;
			}
		}
	}
	
	ret = Frame::extrinsic_;
	std::cout << "calibration_once: return final extrinsic is :\n"
	          << ret.matrix() << std::endl;
	
	return ret;
}
}

