#include <iostream>
#include <pcl/io/pcd_io.h>
#include "lidar_type.h"
#include "ppk_file_reader.h"
#include "graph_slam_options.h"
#include "calibration_process.h"
#include "scan_input_calib.h"
#include "calibration_check.h"

using namespace jf;

typedef struct PointXYZIT {
	PCL_ADD_POINT4D
	uint8_t intensity;
	double timestamp;
	uint16_t ring;                   // laser ring number
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16 PPoint;

// pcl point cloud
POINT_CLOUD_REGISTER_POINT_STRUCT(
		PointXYZIT, (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)
		(double, timestamp, timestamp)(std::uint16_t, ring, ring))
typedef pcl::PointCloud<PPoint> PPointCloud;

int main(int argc, char** argv) {
	
	PpkFileReader ppk("/home/juefx/tasks/rs_calib/ppk.txt", 200);
	
//	const size_t cloud_num = 50;
	const size_t cloud_num = 4;
	
	std::vector<PPointCloud::Ptr> cloud_vec;
	cloud_vec.reserve(cloud_num);
	cloud_vec.resize(cloud_num);
	
	std::vector<LidarScanWithOrigin> lidar_scans_with_origin;
	lidar_scans_with_origin.reserve(cloud_num);
//	lidar_scans_with_origin.resize(cloud_num);
	
	for(size_t i=0; i<cloud_num; i++) {
		cloud_vec[i].reset(new PPointCloud());
		std::string filename;
		
		filename = std::to_string(2) + std::to_string(i) + ".pcd";

		std::string fileheader = "/home/juefx/tasks/rs_calib/pcds/";
		std::string file = fileheader + filename;
		
		std::cout << file << std::endl;
		
		//
//		file = "/home/limingbo/data/hesai_pcap2pcl/cmake-build-release/2.pcd";
//		file = "/home/limingbo/data/raw_data/pointclouds/20.pcd";
		//
		
		pcl::io::loadPCDFile(file, *cloud_vec[i]);
		
		LidarScan lidar_scan;
		lidar_scan.reserve(cloud_vec[i]->size());
//		lidar_scan.resize(cloud_vec[i]->size());
		
		for(size_t j=0; j<cloud_vec[i]->size(); j++) {
			LidarPoint lidar_point;
			lidar_point.x = cloud_vec[i]->points[j].x;
			lidar_point.y = cloud_vec[i]->points[j].y;
			lidar_point.z = cloud_vec[i]->points[j].z;
			lidar_point.intensity = cloud_vec[i]->points[j].intensity;
			
//			cloud_vec[i]->points[j].timestamp = cloud_vec[i]->points[j].timestamp -1;
			
			lidar_point.secs = static_cast<uint32_t>(cloud_vec[i]->points[j].timestamp);
			lidar_point.nsecs = static_cast<uint32_t>((cloud_vec[i]->points[j].timestamp - lidar_point.secs) * 1e9);
			double point_dis = lidar_point.getDistance();
			if (point_dis > 3 && point_dis < 50)
				lidar_scan.emplace_back(std::move(lidar_point));

//			std::cout.precision(28);
//			std::cout << "cloud_vec[i]->points[j].timestamp" << cloud_vec[i]->points[j].timestamp << std::endl;
//			std::cout << "lidar_point.secs" << lidar_point.secs << std::endl;
//			std::cout << "lidar_point.nsecs" << lidar_point.nsecs << std::endl;
		}

//		std::cout << "origin points num = " << lidar_scan.capacity() << std::endl;
//		std::cout << "in distance num = " << lidar_scan.size() << std::endl << std::endl;

//		std::cout.precision(20);
//		std::cout << "cloud_vec[i]->points[0].timestamp" << cloud_vec[i]->points[0].timestamp << std::endl;
		
		cout.precision(16);
		double origin_timestamp = (long long unsigned int)(cloud_vec[i]->points[0].timestamp * 1000) / 1000.0;
		cout << "origin_timestamp" << origin_timestamp << endl;
		
		LidarScanWithOrigin lidar_scan_with_origin(origin_timestamp, lidar_scan);
		lidar_scans_with_origin.emplace_back(std::move(lidar_scan_with_origin));
		
		std::cout << "i = " << i << std::endl;
		std::cout << "lidar_scans_with_origin.size() in cycle : " << lidar_scans_with_origin.size() << std::endl;
	}
	
	GraphSlamOptions options;
	options.cpu_threads = 5;
	options.lidar_max_radius = 50;
	
	Eigen::Isometry3d lidar_imu_extrinsic_matrix = Eigen::Isometry3d::Identity();
#if 1 // RSM1
    lidar_imu_extrinsic_matrix.linear() = (Eigen::AngleAxisd(90*M_PI/180.0, Eigen::Vector3d::UnitZ())).toRotationMatrix();
    lidar_imu_extrinsic_matrix.translation() << 0.04, 0.59885, 0.015;
#endif
	
//	lidar_imu_extrinsic_matrix.linear() = yprDegrees2C_n_b(190, 0, 0);
//	lidar_imu_extrinsic_matrix.translation() = Eigen::Vector3d(-0.02, 0.16, 0.368);

//	lidar_imu_extrinsic_matrix.matrix() <<   -0.982222 ,   0.186033  , 0.0251425 ,      -0.02,
//	-0.186206 ,  -0.982499, -0.00471127 ,         16,
//	0.0238261,  -0.0093092,    0.999673 ,       36.8,
//	0   ,        0     ,      0  ,         1;
	
//	lidar_imu_extrinsic_matrix.matrix() <<  -0.982173 ,  0.186241,  0.0255089 ,     -0.02,
//			-0.186526 , -0.982406, -0.0092733   ,      16,
//			0.023333, -0.0138661,   0.999632 ,      36.8,
//			0,          0 ,         0  ,        1;
	
//	lidar_imu_extrinsic_matrix.matrix() <<    -0.877109 , -0.480042 , 0.0155161 ,     -0.02,
//	0.480068 , -0.877229 ,-0.0022596  ,     0.16,
//	0.0146959, 0.00546685,   0.999877 ,     0.368,
//	0    ,      0   ,       0      ,    1;
	
//	lidar_imu_extrinsic_matrix.linear() = (Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ()) *
//			Eigen::AngleAxisd(8.0*M_PI/180.0, Eigen::Vector3d::UnitY()) *
//			Eigen::AngleAxisd(-42.0*M_PI/180.0, Eigen::Vector3d::UnitZ())).toRotationMatrix();
//
//	lidar_imu_extrinsic_matrix.translation() = Eigen::Vector3d(0.03, -0.504, 0.094);

//	lidar_imu_extrinsic_matrix.linear() = (Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ()) *
//			Eigen::AngleAxisd(8.0*M_PI/180.0, Eigen::Vector3d::UnitY()) *
//			Eigen::AngleAxisd(-42.0*M_PI/180.0, Eigen::Vector3d::UnitZ())).toRotationMatrix();
//
//	lidar_imu_extrinsic_matrix.translation() = Eigen::Vector3d(0.03, -0.504, 0.094);

//	lidar_imu_extrinsic_matrix.matrix() <<  -0.608174 ,   0.79303, -0.0350427 ,      0.03,
//	-0.785784 , -0.607702 , -0.115069 ,    -0.504,
//	 -0.112549, -0.0424462 ,  0.992739 ,     0.094,
//	0      ,    0   ,       0 ,         1;

// initial value:
//	lidar_imu_extrinsic_matrix.linear() = ( Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) *
//			Eigen::AngleAxisd(-20.0*M_PI/180.0, Eigen::Vector3d::UnitX()) ).toRotationMatrix();
//
//	lidar_imu_extrinsic_matrix.translation() = Eigen::Vector3d(-0.0035, -0.37, 0.279);

// after calibration:
//	lidar_imu_extrinsic_matrix.matrix() <<   -0.991263 ,  0.129704 , 0.0239887 ,  -0.0035,
//	-0.130143 , -0.932095 , -0.338027 , -0.37,
//	-0.0214838,  -0.338195 ,  0.940831 , 0.279,
//	0 ,         0   ,       0   ,       1;
	
	// after calibration:
//	lidar_imu_extrinsic_matrix.matrix() <<    -0.999279 ,  0.0373536 , 0.00673704  ,    -0.0035,
//			-0.0374448 ,   -0.94112 ,  -0.335992  ,     -0.37,
//			-0.00621015 ,  -0.336002 ,   0.941841  ,     0.279,
//			0 ,          0 ,          0  ,         1;
	
//	lidar_imu_extrinsic_matrix.matrix() << -0.99966,   -0.0226293 ,  -0.0129158, -0.000770697,
//			0.0256501 ,   -0.941814 ,   -0.335155 ,   -0.362999,
//			-0.00458 ,   -0.335373  ,   0.942074    ,   0.279,
//			0    ,        0       ,     0      ,      1;
	
//	lidar_imu_extrinsic_matrix.matrix() <<
//    -0.9995755041069729 ,-0.02887770670189058 , 0.00385871026808635 , 0.00491639701191993,
//    0.02583805071641327 , -0.9398672216394188 , -0.3405613025917335 , -0.3859514342577523,
//    0.01346130470903569 , -0.3403170341658521 ,  0.9402143955141767 ,             0.27803,
//    0      ,              0           ,         0       ,             1;
	
	
	
	cloud_vec.clear();

//	pcl::io::loadPCDFile(filename, *cloud1);
	/*
	std::vector<UtcPOS>& buf = ppk.getBuf();
	std::cout << "buf.size()" << buf.size() << std::endl;
	
	for(size_t i=0; i<10; i++) {
		std::cout << "buf[i].utc_time" << buf[i].utc_time << std::endl;
		std::cout << "buf[i].gps.lat" << buf[i].gps.lat << std::endl;
		std::cout << "buf[i].gps.lng" << buf[i].gps.lng << std::endl;
		std::cout << "buf[i].gps.height" << buf[i].gps.height << std::endl;
		std::cout << "buf[i].imu.roll" << buf[i].imu.roll << std::endl;
		std::cout << "buf[i].imu.pitch" << buf[i].imu.pitch << std::endl;
		std::cout << "buf[i].imu.yaw" << buf[i].imu.yaw << std::endl << std::endl;
	}
	 */
//	std::cout << lidar_scans_with_origin.size() << lidar_scans_with_origin.size() << std::endl;
//	for(auto lidar_scan : lidar_scans_with_origin) {
//		std::cout << "lidar_scan.origin_timestamp" << lidar_scan.origin_timestamp << std::endl;
//	}
	
	 checkCalibration(
			lidar_scans_with_origin, ppk, lidar_imu_extrinsic_matrix, options);
	
	return 0;
}