#pragma once

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "lidar_type.h"
#include "ppk_file_reader.h"

#define radianToDegree(radian___) (radian___ * 180.0 / M_PI)
#define degreeToRadian(degree___) (degree___ * M_PI / 180.0)

namespace jf {

struct Extrinsic {
	double x;
	
	double y;
	
	double z;
	
	// roll in degrees
	double r;
	// pitch in degrees
	double p;
	// azimuth(yaw) in degrees
	double a;
};

Extrinsic matrix4dToExtrinsic(const Eigen::Matrix4d& trans_matrix);

inline Eigen::Matrix3d ypr2RotationZXY(double yaw, double pitch, double roll) {
	return (Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()) *
	        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX()) *
	        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitY()))
			.toRotationMatrix();
}

inline Eigen::Matrix3d ypr2RotationZXYInverse(double yaw, double pitch, double roll) {
	return (Eigen::AngleAxisd(-roll, Eigen::Vector3d::UnitY()) *
	        Eigen::AngleAxisd(-pitch, Eigen::Vector3d::UnitX()) *
	        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()))
			.toRotationMatrix();
}

inline Eigen::Matrix3d ypr2C_n_b(double yaw, double pitch, double roll) {
	return ypr2RotationZXY(yaw, pitch, roll);
}

inline Eigen::Matrix3d ypr2C_b_n(double yaw, double pitch, double roll) {
	return ypr2RotationZXYInverse(yaw, pitch, roll);
}

inline Eigen::Matrix3d yprDegrees2C_n_b(double yaw, double pitch, double roll) {
	return ypr2C_n_b(degreeToRadian(yaw), degreeToRadian(pitch),
	                 degreeToRadian(roll));
}

inline Eigen::Matrix3d yprDegrees2C_b_n(double yaw, double pitch, double roll) {
	return ypr2C_b_n(degreeToRadian(yaw), degreeToRadian(pitch),
	                 degreeToRadian(roll));
}

LidarScanWithOrigin transformLidarScanToIMUOrigin(
		const LidarScanWithOrigin &original_scan, const PpkFileReader *ppk,
		const Eigen::Matrix4d *lidar_extrinsic_matrix);

std::vector<LocalLidarScan> transformLidarScansToIMUOrigin(
		const std::vector<LidarScanWithOrigin>& deformed_scans, const PpkFileReader& ppk,
		const Eigen::Matrix4d& lidar_extrinsic_matrix, int thread_nums);

pcl::PointCloud<pcl::PointXYZI>::Ptr transformLocalScanToLidarOrigin(
		const LocalLidarScan& local_scan,
		const Eigen::Matrix4d* lidar_extrinsic_matrix);

Eigen::Isometry3d getRelativeTransPoseOfIMU(const UtcPOS& pose1,
                                            const UtcPOS& pose2);

pcl::PointCloud<pcl::PointXYZI>::Ptr transformLocalScanToLidarOrigin(
		const LocalLidarScan& local_scan,
		const Eigen::Matrix4d* lidar_extrinsic_matrix);

}