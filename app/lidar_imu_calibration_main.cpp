#include <iostream>
#include <pcl/io/pcd_io.h>
#include "lidar_type.h"
#include "ppk_file_reader.h"
#include "graph_slam_options.h"
#include "calibration_process.h"
#include "scan_input_calib.h"



using namespace jf;

typedef struct PointXYZIT {
	PCL_ADD_POINT4D
	float intensity;
	double timestamp;
	uint16_t ring;                   // laser ring number
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16 PPoint;

// pcl point cloud
POINT_CLOUD_REGISTER_POINT_STRUCT(
		PointXYZIT, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
		(double, timestamp, timestamp)(std::uint16_t, ring, ring))
typedef pcl::PointCloud<PPoint> PPointCloud;

bool isInsect(double src_start, double src_end, double tgt_start, double tgt_end) {
    return !(src_end < tgt_start || src_start > tgt_end);
}
/*
 * 搞清楚输入有哪些即可
 * 1、后解算文件，也即IE文件（以前的格式，21列或22列的形式）
 * 2、pcd文件夹，存放由pcap转来的pcd，一般来说，每个pcd点云时间长度为两秒
 * 3、pcd文件夹里面有多少pcd。例如60个pcd，其名称为10.pcd - 69.pcd
 * 4、外参初值
 */


int main(int argc, char** argv) {
//    Eigen::Matrix4d trans_matrix;
//    trans_matrix <<  0.9940555604812841, -0.1083991201668767,  0.0101574319238297,               7.345,
//    0.08058182026851791 , 0.7952666942992368 , 0.6008805664860335 ,                1.3,
//    -0.0732127920410941 ,-0.5964901639469098 , 0.7992742779522786,   2.731860160827637,
//    0       ,            0      ,             0   ,                1;
//    Extrinsic ex = matrix4dToExtrinsic(trans_matrix);
//    cout << ex.x << "," << ex.y << "," << ex.z << "," << ex.r << "," << ex.p << "," << ex.a << endl;
//
//    return 0 ;

	// 1、解析后解算原始数据
	// 输入一：后解算文件
	int pose_frequency = 20;
	PpkFileReader ppk("/home/qcraft/task/qcraft/offboard/calibration/lidar_extrinsics/lidar_calibration_record/20210611_195708_Q8021_2/gnss_pose_ie_new.txt", pose_frequency);
	// 输入二：pcd文件夹
	std::string fileheader = "/home/qcraft/task/qcraft/offboard/calibration/lidar_extrinsics/lidar_calibration_record/20210611_195708_Q8021_2/pcds/";
	// 输入三：定义用于标定的点云组数
	const size_t cloud_num = 60;

	std::vector<PPointCloud::Ptr> cloud_vec;
	cloud_vec.reserve(cloud_num);
	cloud_vec.resize(cloud_num);
	
	std::vector<LidarScanWithOrigin> lidar_scans_with_origin;
	lidar_scans_with_origin.reserve(cloud_num);
//	lidar_scans_with_origin.resize(cloud_num);
	
	// 2、组织点云数据
	for(size_t i=0; i<cloud_num; i++) {
		cloud_vec[i].reset(new PPointCloud());
		std::string filename;
		
//		filename = std::to_string(1) + std::to_string(i+10) + ".pcd";
		filename = std::to_string(i+30) + ".pcd";
		
		std::string file = fileheader + filename;

		std::cout << file << std::endl;

		pcl::io::loadPCDFile(file, *cloud_vec[i]);

		LidarScan lidar_scan;
		lidar_scan.reserve(cloud_vec[i]->size());
//		lidar_scan.resize(cloud_vec[i]->size());

		for(size_t j=0; j<cloud_vec[i]->size(); j++) {
			LidarPoint lidar_point;
			lidar_point.x = cloud_vec[i]->points[j].x;
			lidar_point.y = cloud_vec[i]->points[j].y;
			lidar_point.z = cloud_vec[i]->points[j].z;
			float sqr_dis = lidar_point.x*lidar_point.x + lidar_point.y*lidar_point.y + lidar_point.z*lidar_point.z;
			if(sqr_dis > 50*50 || sqr_dis < 2*2) continue;
			lidar_point.intensity = cloud_vec[i]->points[j].intensity;
			lidar_point.secs = static_cast<uint32_t>(cloud_vec[i]->points[j].timestamp);
			lidar_point.nsecs = static_cast<uint32_t>((cloud_vec[i]->points[j].timestamp - lidar_point.secs) * 1e9);
            lidar_point.intensity = cloud_vec[i]->points[j].intensity;
				lidar_scan.emplace_back(std::move(lidar_point));
				
		}
		
//		std::cout << "origin points num = " << lidar_scan.capacity() << std::endl;
//		std::cout << "in distance num = " << lidar_scan.size() << std::endl << std::endl;
		
//		std::cout << "cloud_vec[i]->points[0].timestamp" << cloud_vec[i]->points[0].timestamp << std::endl;
		cout.precision(16);
//		double origin_timestamp = (long long unsigned int)(cloud_vec[i]->points[0].timestamp * 1000) / 1000.0;
        double origin_timestamp = std::round(cloud_vec[i]->points[0].timestamp * pose_frequency) / (double)pose_frequency;
		cout << "origin_timestamp" << origin_timestamp << endl;
//	LidarScanWithOrigin lidar_scan_with_origin(pcd_cloud_input->points[0].timestamp, lidar_scan);
		LidarScanWithOrigin lidar_scan_with_origin(origin_timestamp, lidar_scan);
		lidar_scans_with_origin.emplace_back(std::move(lidar_scan_with_origin));
		
		std::cout << "i = " << i << std::endl;
		std::cout << "lidar_scans_with_origin.size() in cycle : " << lidar_scans_with_origin.size() << std::endl;
	}
	
	// 3、标定选项
	GraphSlamOptions options;
	options.cpu_threads = 5;
	options.lidar_max_radius = 50;
	
	// 4、输入外参初值
	// 输入四：外参初值
	Eigen::Isometry3d lidar_imu_extrinsic_matrix = Eigen::Isometry3d::Identity();
#if 0
//	lidar_imu_extrinsic_matrix.linear() = yprDegrees2C_n_b(190, 0, 0);
//	lidar_imu_extrinsic_matrix.translation() = Eigen::Vector3d(-0.02, 0.16, 0.368);
	
//	lidar_imu_extrinsic_matrix.linear() = (Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ()) *
//	                                       Eigen::AngleAxisd(8.0*M_PI/180.0, Eigen::Vector3d::UnitY()) *
//	                                       Eigen::AngleAxisd(-42.0*M_PI/180.0, Eigen::Vector3d::UnitZ())).toRotationMatrix();
//
//	lidar_imu_extrinsic_matrix.translation() = Eigen::Vector3d(0.03, -0.504, 0.094);
#endif

#if 0 // 下倾
	lidar_imu_extrinsic_matrix.linear() = ( Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) *
	                                        Eigen::AngleAxisd(20.0*M_PI/180.0, Eigen::Vector3d::UnitX()) ).toRotationMatrix();

	lidar_imu_extrinsic_matrix.translation() = Eigen::Vector3d(0.1595, -0.482014, 0.040823);

//    lidar_imu_extrinsic_matrix.linear() << -1, 0, 0, 0, -1, 0, 0, 0, 1;
//    lidar_imu_extrinsic_matrix.translation() << -0.014, 0.166, 0.334;
//    lidar_imu_extrinsic_matrix.matrix() <<        0.9999220141488692, -0.001504247000996202,  -0.01239769581126953 ,              -0.0045,
//                                                  -0.002854603504427214 ,   0.9389084463919692,   -0.3441551692662063,               0.02534,
//    0.01215799569424533 ,   0.3441637205383197,    0.9388309307888828    ,           0.20771,
//    0             ,        0      ,               0   ,                  1;
#endif

#if 0 // XT32
    lidar_imu_extrinsic_matrix.linear() = (Eigen::AngleAxisd(180*M_PI/180.0, Eigen::Vector3d::UnitZ())).toRotationMatrix();
//    lidar_imu_extrinsic_matrix.translation() << -(0.125-(0.0477-0.027)), -(0.18-0.058), 0.244-0.015+0.0464;
    lidar_imu_extrinsic_matrix.translation() << -30.6/1000, 20.34/1000, (349.89 + 46.4)/1000;
#endif
#if 0 // RSM1
//    lidar_imu_extrinsic_matrix.linear() = (Eigen::AngleAxisd(90*M_PI/180.0, Eigen::Vector3d::UnitZ())).toRotationMatrix();
//    lidar_imu_extrinsic_matrix.translation() << 0.04, 0.59885, 0.015;
//    lidar_imu_extrinsic_matrix.matrix() <<   -0.01421382870798138 ,  -0.9998898150095205 ,-0.004280760877685615   ,               0.04,
//    0.9998069903923746 , -0.01415432006939832 , -0.01362487379464196   ,            0.59885,
//    0.01356278127844927,  -0.00447359627199314 ,   0.9998980137495964  ,               0.015,
//    0     ,                0    ,                 0    ,                 1;

    lidar_imu_extrinsic_matrix.matrix() << -0.01421382870826693  , -0.9998898150095207 ,-0.004280760877684309 ,   0.2064854108612717,
    0.9998069903923749 , -0.01415432006968365  , -0.0136248737946445  ,  0.6842966480010559,
    0.01356278127845057, -0.004473596271995681  ,  0.9998980137495964  ,               0.015,
    0           ,          0      ,               0     ,                1;
#endif
	cloud_vec.clear();
//4.5149674415588379, 0.9898494482040405, 2.3150002956390381, 0.7136061582910318, 0.0012423763502774, -0.5665602429494437
    lidar_imu_extrinsic_matrix.linear() = (Eigen::AngleAxisd(-0.5665602429494437*M_PI/180.0, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(0.0012423763502774*M_PI/180.0, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(0.7136061582910318*M_PI/180.0, Eigen::Vector3d::UnitX())).toRotationMatrix();
//    lidar_imu_extrinsic_matrix.translation() << -(0.125-(0.0477-0.027)), -(0.18-0.058), 0.244-0.015+0.0464;
    lidar_imu_extrinsic_matrix.translation() << 4.5149674415588379, 0.9898494482040405, 2.3150002956390381;

    lidar_imu_extrinsic_matrix.matrix() <<  0.999897890439518 ,   -0.0142728868095511,  0.0007024220406961285  ,    4.514967441558838,
    0.01426850072653189  ,   0.9998808129887184 ,  0.005896584098983925   ,  0.9898494482040405,
    -0.0007864995985202361 , -0.005885959491975228 ,    0.9999823682941816  ,    2.315000295639038,
    0                  ,    0   ,                   0    ,                  1;

//    lidar_imu_extrinsic_matrix.matrix() <<    0.9999134283940964 , -0.01314581755799396 ,0.0005685049659784202  ,   4.514967441558838,
//    0.01314172342617605 ,   0.9998912727150937 , 0.006688636152465057,    0.9898494482040405,
//     -0.000656370744549122, -0.006680585971462737 ,   0.9999774692204438 ,    2.315000295639038,
//    0         ,            0   ,                  0    ,                 1;
//
//    lidar_imu_extrinsic_matrix.matrix() <<    0.9999133952115635 ,  -0.01315041879894527,  0.0005182295784047853 ,     4.563047941860715,
//    0.01314673659597109   ,  0.9998920403799924 ,  0.006562842494821804  ,   0.9584420999554403,
//    -0.0006044777578548455 ,  -0.00655546109347247 ,    0.9999783300334524  ,    2.315000295639038,
//    0             ,         0     ,                 0            ,          1;

//    final extrinsic:
//    0.9999133952115635   -0.01315041879894527  0.0005182295784047853      4.572589910619315
//    0.01314673659597109     0.9998920403799924   0.006562842494821804     0.9450076835841004
//    -0.0006044777578548455   -0.00655546109347247     0.9999783300334524      2.315000295639038
//    0                      0                      0                      1
//    x y z r p a :
//    4.572589910619315 0.9450076835841004 2.315000295639038 -0.3756030122712321 0.03463402644378769 0.7532743589136763


//    0.9998979215879413   -0.01427026665961454  0.0007112619620843963      4.577217949202573
//    0.01426582384971635     0.9998808160172024   0.005902544422739029     0.9422026533969032
//   -0.0007954080739336491  -0.005891795162515151     0.9999823268817098      2.315000295639038
//    0                      0                      0                      1
//    x y z r p a :
//    4.577217949202573 0.9422026533969032 2.315000295639038 -0.3375770564418105 0.04557353043256003 0.8173994833835148


    // 5、进入标定程序
	Eigen::Isometry3d final_lidar_imu_extrinsic = calibrationProcess(
			lidar_scans_with_origin, ppk, lidar_imu_extrinsic_matrix, options);
	
	return 0;
}

