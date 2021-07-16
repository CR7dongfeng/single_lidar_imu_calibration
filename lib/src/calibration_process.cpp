//
// Created by ubuntu on 19-7-3.
//
#include "calibration_process.h"
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <boost/filesystem.hpp>
#include <iostream>
#include <string>
#include <thread>
#include <future>
#include <vector>
#include "calib_options.h"
#include "calibration_once.h"
#include "scan_input_calib.h"


#include "graph_slam_options.h"

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>

#include <fstream>

#include <pcl/io/pcd_io.h>

namespace jf {

Eigen::Isometry3d calibrationProcess(
		const vector<DeformedLidarScan>& deformed_scans, const PpkFileReader& ppk,
		const Eigen::Isometry3d& extrinsic_input, const GraphSlamOptions& options) {
	Eigen::Isometry3d out_extrinsic = extrinsic_input;

	std::cout << "initial extrinsic:\n" << out_extrinsic.matrix() << endl;

	CalibrationOptions calib_options;
	
//	calib_options.calib_stage = 2;

	int iterations = 0;

	// 主循环
	while (iterations <= calib_options.max_iterations) {
//		vector<DeformedLidarScan> local_scans;
//		vector<future<LidarScanWithOrigin>> future_vec(deformed_scans.size());
//		for(size_t i=0; i<deformed_scans.size(); i++) {
//			future_vec[i] = async(transformLidarScanToIMUOrigin, deformed_scans[i], &ppk,
//			                       &out_extrinsic.matrix());
//		}
//		for(size_t i=0; i<deformed_scans.size(); i++) {
//			local_scans.emplace_back(std::move(future_vec[i].get()));
//		}

//		for(auto deformed_scan : deformed_scans) {
//			local_scans.emplace_back(
//				std::move(transformLidarScanToIMUOrigin(deformed_scan, &ppk, &out_extrinsic.matrix()))
//					);
//		}
		
		// 1、以IMU为原点，将点云数据转换到ENU坐标系，并进行运动补偿
		auto local_scans = transformLidarScansToIMUOrigin(
				deformed_scans, ppk, out_extrinsic.matrix(), options.cpu_threads);

		LOG(INFO) << "size of local_scans" << local_scans.size();

		// 2、计算IMU坐标系下的相对运动
		vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>
				vec_relative_imu;

		for (size_t i = 0; i < local_scans.size(); i++) {
			Eigen::Isometry3d relative_imu = getRelativeTransPoseOfIMU(
					local_scans.at(0).pose, local_scans.at(i).pose);
			vec_relative_imu.emplace_back(relative_imu);
		}
		
		// 3、将运动补偿后的点云从ENU坐标系转回到Lidar坐标系下
		vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> local_lidar_scans;
		
		for (size_t i = 0; i < local_scans.size(); i++) {
			local_lidar_scans.emplace_back(std::move(
					transformLocalScanToLidarOrigin(local_scans[i], &out_extrinsic.matrix())
					));
            local_lidar_scans[i]->height = 1;
            local_lidar_scans[i]->width = local_lidar_scans[i]->points.size();
            local_lidar_scans[i]->is_dense = false;
			pcl::io::savePCDFileBinary("/home/qcraft/task/qcraft/offboard/calibration/lidar_extrinsics/lidar_calibration_record/20210611_195708_Q8021_2/test_pcds2/" + to_string(iterations) + "-" + to_string(i) + ".pcd", *(local_lidar_scans[i]));
		}

		local_scans.clear();

		std::cout << "iterations = " << iterations << std::endl;
		std::cout << "before calibration: \n"
		          << out_extrinsic.matrix() << std::endl;

		Eigen::Quaterniond q_former(out_extrinsic.linear());
		Eigen::Vector3d t_former(out_extrinsic.translation());

		// 4、进行标定并判断阈值，决定是否继续
		out_extrinsic = calibrationOnce(local_lidar_scans, vec_relative_imu,
		                                out_extrinsic, calib_options);

		Eigen::Quaterniond q_latter(out_extrinsic.linear());
		Eigen::Vector3d t_latter(out_extrinsic.translation());

		double rot_residual =
				fabs(1 - ((q_former * q_latter.conjugate()).normalized()).w());
		;
		std::cout << "outer: rot_residual is " << rot_residual << endl;
		double trans_residual = (t_former - t_latter).norm();
		std::cout << "outer: trans_residual is " << trans_residual << endl;

		if (calib_options.calib_stage == ICP_Ceres::CALIB_R_STEP) {
			if (rot_residual < calib_options.outer_rotation_epsilon) {
				calib_options.calib_stage = ICP_Ceres::CALIB_XY_STEP;
				LOG(INFO) << "Extrinsic R converges! Iterations is :" << iterations;
				iterations = 0;
			} else if (iterations == calib_options.max_iterations) {
				calib_options.calib_stage = ICP_Ceres::CALIB_XY_STEP;
				iterations = 0;
				LOG(INFO) << "Extrinsic R - max iterations!";
			}
		} else if (calib_options.calib_stage == ICP_Ceres::CALIB_XY_STEP) {
			if (trans_residual < calib_options.outer_translation_epsilon) {
				LOG(INFO) << "Extrinsic XY converges! Iterations is :" << iterations;
				std::cout << "final extrinsic: \n"
				          << out_extrinsic.matrix() << std::endl;
				
				Extrinsic xyzrpa = matrix4dToExtrinsic(out_extrinsic.matrix());
				std::cout << "x y z r p a : \n"
									<< xyzrpa.x << " " << xyzrpa.y << " " << xyzrpa.z << " "
									<< xyzrpa.r << " " << xyzrpa.p << " " << xyzrpa.a << endl;
//				ofstream ofs("/home/limingbo/data/extrinsic.txt");
//				ofs << xyzrpa.x << " " << xyzrpa.y << " " << xyzrpa.z << " "
//				    << xyzrpa.r << " " << xyzrpa.p << " " << xyzrpa.a << endl;
				return out_extrinsic;
			}
		}
		iterations++;
	}

	return out_extrinsic;
}
}

/*

 */