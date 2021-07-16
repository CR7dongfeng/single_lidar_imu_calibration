#pragma once

#include "ppk_file_reader.h"
#include "lidar_type.h"
#include "graph_slam_options.h"

namespace jf {

using DeformedLidarScan = LidarScanWithOrigin;
using namespace std;

/*
 * Description: execute calibration
 * Input:
 * deformed_scans: point cloud scans in local lidar frames
 * ppk: 后解算GNSS数据
 * extrinsic_input: input lidar-imu extrinsic
 * options: calib options
 * Return: output lidar-imu extrinsic
 */
Eigen::Isometry3d calibrationProcess(
		const vector<DeformedLidarScan>& deformed_scans, const PpkFileReader& ppk,
		const Eigen::Isometry3d& extrinsic_input, const GraphSlamOptions& options);

}
