#pragma once

#include <Eigen/Geometry>
#include <string>
#include "calib_options.h"
#include "icp-ceres.h"


namespace jf {

/*
 * Description: execute calibration one iteration
 * Input:
 * local_lidar_scans: point cloud scans in local lidar frames
 * vec_relative_imu: relative motion tranformation matrix in imu frames
 * extrinsic_input: input lidar-imu extrinsic
 * calib_options: calib options
 * Return: out lidar-imu extrinsic
 */
Eigen::Isometry3d calibrationOnce(
		vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &local_lidar_scans,
		const vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>
		&vec_relative_imu,
		const Eigen::Isometry3d &extrinsic_input,
		const CalibrationOptions &calib_options);
}
