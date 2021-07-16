#pragma once

#include "lidar_type.h"
#include "scan_input_calib.h"
#include "ppk_file_reader.h"
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include "graph_slam_options.h"

namespace jf {
using DeformedLidarScan = LidarScanWithOrigin;
using namespace std;

/*
 * check calibration result
 */
void checkCalibration(
		const vector <DeformedLidarScan> &deformed_scans, const PpkFileReader &ppk,
		const Eigen::Isometry3d &extrinsic_input, const GraphSlamOptions &options);
	
}