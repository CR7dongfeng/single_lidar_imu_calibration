#include "scan_input_calib.h"
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <boost/make_shared.hpp>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>

#include "concurrent_worker.h"


#define radianToDegree(radian___) (radian___ * 180.0 / M_PI)
#define degreeToRadian(degree___) (degree___ * M_PI / 180.0)

namespace jf {

using namespace std;
using namespace GeographicLib;
// using namespace api;
using namespace pcl;

void rotationToYawPitchRoll(const Eigen::Matrix3d& rotation, double& yaw,
                            double& pitch, double& roll) {
	yaw = std::atan2(rotation(1, 0), rotation(0, 0));
	pitch = std::atan2(-rotation(2, 0), std::sqrt(std::pow(rotation(2, 1), 2) +
	                                              std::pow(rotation(2, 2), 2)));
	roll = std::atan2(rotation(2, 1), rotation(2, 2));
}

Extrinsic matrix4dToExtrinsic(const Eigen::Matrix4d& trans_matrix) {
	//Eigen::Matrix4d trans_matrix = trans_matrix_origin.inverse();
	Extrinsic ex;
	rotationToYawPitchRoll(trans_matrix.block(0, 0, 3, 3), ex.a, ex.p, ex.r);
	ex.x = trans_matrix(0, 3);
	ex.y = trans_matrix(1, 3);
	ex.z = trans_matrix(2, 3);
	ex.r = ex.r * 180.0 / M_PI;
	ex.p = ex.p * 180.0 / M_PI;
	ex.a = ex.a * 180.0 / M_PI;
	return ex;
}

LidarScanWithOrigin transformLidarScanToIMUOrigin(
		const LidarScanWithOrigin &original_scan, const PpkFileReader *ppk,
		const Eigen::Matrix4d *lidar_extrinsic_matrix) {
	LidarScanWithOrigin local_scan_with_origin;
	local_scan_with_origin.origin_timestamp = original_scan.origin_timestamp;
	
	std::cout << "local_scan_with_origin.origin_timestamp : " << local_scan_with_origin.origin_timestamp << std::endl;
	local_scan_with_origin.scan.reserve(original_scan.scan.size());
	
	// 读取ppk(已经插值)
	auto track_point_pos = ppk->read(original_scan.origin_timestamp);  //这是关键
	
	// 以下是操作坐標系變換
	auto C_ob_n =
			yprDegrees2C_b_n(track_point_pos.imu.yaw, track_point_pos.imu.pitch,
			                 track_point_pos.imu.roll);
	
	LocalCartesian local_cartesian(track_point_pos.gps.lat,
	                               track_point_pos.gps.lng,
	                               track_point_pos.gps.height);
	
	for (const auto &original_point : original_scan.scan) {
		local_scan_with_origin.scan.emplace_back(original_point);
		auto &point = local_scan_with_origin.scan.back();
		
		auto lidar_POS = ppk->read(point.getTimestamp());
		
//		std::cout << "point.getTimestamp() : " << point.getTimestamp() << std::endl;
		if (lidar_POS.utc_time < 0.0) {
			LOG_EVERY_N(WARNING, 9999) << "NO ppk data found for lidar point : "
			                           << point.getTimestampMili();
			local_scan_with_origin.scan.pop_back();
			continue;
		}
		
		double dx, dy, dz = lidar_POS.gps.height - track_point_pos.gps.height;
		
		// geographicLib, 5 us
		local_cartesian.Forward(lidar_POS.gps.lat, lidar_POS.gps.lng,
		                        lidar_POS.gps.height, dx, dy, dz);
		
//		Eigen::Vector3d relative_T(dy, dx, -1.0 * dz);  // 东北天变成北东地
		Eigen::Vector3d relative_T(dx, dy, dz);  // 东北天变成北东地
		
		auto C_n_lb = yprDegrees2C_n_b(lidar_POS.imu.yaw, lidar_POS.imu.pitch,
		                               lidar_POS.imu.roll);
		
		point.vector4d = *lidar_extrinsic_matrix * point.vector4d;  //
		
		point.vector3d = C_n_lb * point.vector3d + relative_T;
	}
	
	local_scan_with_origin.pose = track_point_pos;
	
	std::cout << "local_scan_with_origin.scan.size()" << local_scan_with_origin.scan.size() << std::endl;
	
	return local_scan_with_origin;
}

vector<LocalLidarScan> transformLidarScansToIMUOrigin(
		const vector<LidarScanWithOrigin>& deformed_scans, const PpkFileReader& ppk,
		const Eigen::Matrix4d& lidar_extrinsic_matrix, int thread_nums) {
	auto local_scans = concurrentWorker(
			deformed_scans, std::function<decltype(transformLidarScanToIMUOrigin)>(
					transformLidarScanToIMUOrigin),
			thread_nums, &ppk, &lidar_extrinsic_matrix);
	
	sort(local_scans.begin(), local_scans.end(),
	     [](const LidarScanWithOrigin& a, const LidarScanWithOrigin& b) {
		     return a.origin_timestamp < b.origin_timestamp;
	     });
	
	return local_scans;
}

Eigen::Isometry3d getRelativeTransPoseOfIMU(const UtcPOS& pose1,
                                            const UtcPOS& pose2) {
	Eigen::Matrix3d C_b1_n, C_n_b2;
	Eigen::Isometry3d ret = Eigen::Isometry3d::Identity();
	
	LocalCartesian local_cartesian(pose1.gps.lat, pose1.gps.lng,
	                               pose1.gps.height);
	double dx, dy, dz;
	local_cartesian.Forward(pose2.gps.lat, pose2.gps.lng, pose2.gps.height, dx,
	                        dy, dz);
//	Eigen::Vector3d relative_T_nav(dy, dx, -1.0 * dz);
	Eigen::Vector3d relative_T_nav(dx, dy, dz);
	
	C_b1_n = yprDegrees2C_b_n(pose1.imu.yaw, pose1.imu.pitch, pose1.imu.roll);
	C_n_b2 = yprDegrees2C_n_b(pose2.imu.yaw, pose2.imu.pitch, pose2.imu.roll);
	
	Eigen::Matrix3d relative_R_body = C_b1_n * C_n_b2;
	Eigen::Vector3d relative_T_body = C_b1_n * relative_T_nav;
	
	ret.linear() = relative_R_body;
	ret.translation() = relative_T_body;
	
	//    LOG(INFO) << "relative_imu is :\n" << ret.matrix();
	
	return ret;
}

PointCloud<PointXYZI> lidarScan2PclXYZICloud(const LidarScan& scan) {
	PointCloud<PointXYZI> cloud;
	for (const auto& pt : scan) cloud.points.emplace_back(pt);
	return cloud;
}

inline pcl::PointCloud<pcl::PointXYZI>::Ptr lidarScan2PclXYZICloudPtr(
		const LidarScan& scan) {
	return boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(
			lidarScan2PclXYZICloud(scan));
}

pcl::PointCloud<pcl::PointXYZI>::Ptr toProcessedCloud(
		const LidarScanWithOrigin& scan) {
	auto cloud = lidarScan2PclXYZICloudPtr(scan.scan);
	cloud->header.stamp = scan.origin_timestamp * 1e6;
	
	// 100ms per cloud, optional?
	// FIXME: this function will remove points on light pole, maybe statistic
	// filter won't?
	//  radiusOutlierRemoval<PointXYZI>(cloud, 1.0, 10);
	
	return cloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr transformLocalScanToLidarOrigin(
		const LocalLidarScan& local_scan,
		const Eigen::Matrix4d* lidar_extrinsic_matrix) {
	auto& pose = local_scan.pose;
	auto scan = toProcessedCloud(local_scan);
	
	Eigen::Isometry3d extrinsic = Eigen::Isometry3d::Identity();
	extrinsic.matrix() = *lidar_extrinsic_matrix;
	Eigen::Isometry3d T_ol_n = Eigen::Isometry3d::Identity();
	T_ol_n.linear() =
			extrinsic.linear().transpose() *
			yprDegrees2C_b_n(pose.imu.yaw, pose.imu.pitch, pose.imu.roll);
	T_ol_n.translation() =
			-extrinsic.linear().transpose() * extrinsic.translation();  // -RT * t
	pcl::transformPointCloud(*scan, *scan, T_ol_n.matrix());
	
	return scan;
}



}
