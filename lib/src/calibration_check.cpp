#include "calibration_check.h"

#include "calibration_process.h"
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <boost/filesystem.hpp>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "calib_options.h"
#include "calibration_once.h"
#include "scan_input_calib.h"


#include "graph_slam_options.h"

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <pcl/visualization/pcl_visualizer.h>

namespace jf {

void checkCalibration (
		const vector<DeformedLidarScan>& deformed_scans, const PpkFileReader& ppk,
		const Eigen::Isometry3d& extrinsic_input, const GraphSlamOptions& options) {
	Eigen::Isometry3d out_extrinsic = extrinsic_input;
	
	std::cout << "initial extrinsic:\n" << out_extrinsic.matrix() << endl;
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> p(new pcl::visualization::PCLVisualizer("vp1"));
	int vp_1;
	
	auto local_scans = transformLidarScansToIMUOrigin(
			deformed_scans, ppk, out_extrinsic.matrix(), options.cpu_threads);
	
//	vector<DeformedLidarScan> local_scans;
//
////	int counter = 0;
////	for(auto deformed_scan : deformed_scans) {
////		local_scans.emplace_back(
////				std::move(transformLidarScanToIMUOrigin(deformed_scan, &ppk, &out_extrinsic.matrix()))
////		);
////		counter++;
////
////		if( counter >= 10 ) {
////			break;
////		}
////	}
	
	vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> local_lidar_scans;
	
	for (size_t i = 0; i < local_scans.size(); i++) {
		local_lidar_scans.emplace_back(std::move(
				transformLocalScanToLidarOrigin(local_scans[i], &out_extrinsic.matrix())
		));
		
		p->removePointCloud("vp1");
		pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>
				src_h(local_lidar_scans[i], "intensity");
		p->addCoordinateSystem(1.0, 0, 0, 0);
		p->addPointCloud(local_lidar_scans[i], src_h, "vp1", vp_1);
		
		p->spin();
	}
	
	local_scans.clear();
	
}
}

