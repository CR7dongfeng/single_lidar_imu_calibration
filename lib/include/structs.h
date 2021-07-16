#pragma once

#include <string>
#include <vector>

namespace jf {

struct LidarRectifiedTrackPoint {
	std::string trackPointId;
	
	//定位时间，单位：毫秒
	long locTime;
	
	//经度，坐标单位：度，10*8
	double x;
	//纬度，坐标单位：度，10*8
	double y;
	//高程，单位：米，
	double z;
	// 度
	double roll;
	// 度
	double pitch;
	// 度
	double azimuth;
	
	//经度，坐标单位：度，10*8
	double xDelta;
	
	//纬度，坐标单位：度，10*8
	double yDelta;
	
	//高程，单位：米，
	double zDelta;
	// 度
	double rollDelta;
	// 度
	double pitchDelta;
	// 度
	double azimuthDelta;
};

struct LidarRectifiedTrack {
	std::string surveyTaskId;
	
	std::string trackId;
	
	bool isRectified;
	
	std::vector<LidarRectifiedTrackPoint> pointList;
};

struct TrackExtend {
	std::string trackId;
	
	double cameraHeight;
	
	double rollDelta;
	
	double pitchDelta;
	
	double azimuthDelta;
	
	long uploadTime;
	
	//  uint8_t cameraCalibrationType;
	//
	//  uint8_t heightCalibrationType;
};

struct HardwareDevice {
	std::string id;
	
	std::string surveyDeviceId;
	
	// to be completed
};

struct SurveyDevice {
	std::string id;
	
	std::string name;
	
	bool isCalibrated;
	
	bool delFlag;
	
	std::string remark;
	
	std::string operUser;
	
	long createTime;
	
	long updateTime;
	
	std::vector<HardwareDevice> hardwareDevices;
	
	// to be completed
};

struct CameraCalibrationInfo {
	std::string id;
	
	double focusX;
	
	double focusY;
	
	double principlePointX;
	
	double principlePointY;
	
	double k1;
	
	double k2;
	
	double k3;
	
	double p1;
	
	double p2;
	
	int imageWidth;
	
	int imageHeight;
	
	// to be completed
};

struct LeverArmCalibrationInfo {
	std::string id;
	
	double xt;
	
	double yt;
	
	double zt;
	
	double xj;
	
	double yj;
	
	double zj;
	
	uint8_t imuOrientation;
	
	double apllyVehicleBodyRotationX;
	
	double apllyVehicleBodyRotationY;
	
	double apllyVehicleBodyRotationZ;
	
	uint8_t version;
	
	long calibrationTime;
	
	std::string calibrationUser;
	
	bool valid;
	
	bool delFlag;
	
	std::string remark;
	
	std::string operUser;
	
	long createTime;
	
	long updateTime;
};

struct LidarCameraCalibrationInfo {
	std::string id;
	
	double lidarCameraX;
	
	double lidarCameraY;
	
	double lidarCameraZ;
	
	double lidarCameraRoll;
	
	double lidarCameraPitch;
	
	double lidarCameraAzimuth;
	
	// to be completed
};

struct LidarBodyCalibrationInfoSingle {
	std::string id;
	
	int hubport;
	
	int calibratedSurveyDeviceId;
	
	double lidarBodyX;
	
	double lidarBodyY;
	
	double lidarBodyZ;
	
	double lidarBodyRoll;
	
	double lidarBodyPitch;
	
	double lidarBodyAzimuth;
	
	// to be completed
};

struct CalibratedSurveyDevice {
	std::string id;
	
	std::string surveyDeviceId;
	
	std::string cameraCalibrationId;
	
	std::string leverArmCalibrationId;
	
	std::string lidarCalibrationId;
	
	std::string inUser;
	
	std::string remark;
	
	bool delFlag;
	
	std::string operUser;
	
	long createTime;
	
	long updateTime;
	
	double rollDelta;
	
	double pitchDelta;
	
	double azimuthDelta;
	
	SurveyDevice surveyDevice;
	
	CameraCalibrationInfo cameraCalibrationInfo;
	
	LeverArmCalibrationInfo leverArmCalibrationInfo;
	
	LidarCameraCalibrationInfo lidarCameraCalibrationInfo;
	
	std::vector<LidarBodyCalibrationInfoSingle> lidarBodyCalibrationInfo;
};

class Coordinate {
 public:
	double x;
	double y;
	double z;
};

class KRSTrackPoint {
 public:
	// id
	std::string trackPointId;
	std::string trackId;
	//任务ID
	std::string taskId;
	//
	std::string seq;
	//设备号
	std::string deviceId;
	//
	std::string deviceType;
	//经度，坐标单位：度，10*8
	double x;
	//纬度，坐标单位：度，10*8
	double y;
	//高程，单位：米， 10*3
	double z;
	//
	Coordinate location;
	Coordinate coordinate;
	//定位时间，单位：毫秒
	long locTime;
	std::string status;
	std::string northVelocity;
	std::string eastVelocity;
	std::string upVelocity;
	std::string roll;
	std::string pitch;
	std::string azimuth;
	std::string picH;
	std::string picW;
	std::string longitudeSigma;
	std::string latitudeSigma;
	std::string heightSigma;
	std::string rollSigma;
	std::string pitchSigma;
	std::string azimuthSigma;
	//
	std::string posType;
	std::string version;
	//照片状态 0：正常
	std::string delFlag;
	// R矩阵
	double R[3][3];
	// C矩阵
	double C[3];
	// T矩阵
	double T[3];
	
	//是否为后差分数据，true代表后差分数据，false代表实时差分数据
	bool postDifference;
	//位置点后差分质量，范围1-6
	long qualityNum;
	//后处理状态， 1代表fixed, 2代表其他状态
	long ambStatus;
	//卫星状态
	long gnssQuality;
};

class KRSTrack {
 public:
	// id
	std::string trackId;
	//任务ID
	std::string taskId;
	//设备号
	std::string deviceId;
	//
	std::string seq;
	//原始轨迹名称，比如track6
	std::string surveyTrackName;
	//
	double totalMileage;
	//
	double effectiveMileage;
	
	//轨迹线开始时间，utc, 时间单位，毫秒
	long long startTime;
	
	//轨迹线结束时间，utc, 时间单位，毫秒
	long long endTime;
	
	//坐标数组
	std::vector<Coordinate> coordBuffer;
	
	//轨迹点列表
	std::vector<KRSTrackPoint> pointList;
};

struct DaLaneGeometry {
 public:
	std::string type;
	std::vector<std::vector<double>> coordinates;
	std::vector<double> nodeCoordinates;
};

struct DaLaneProperties {
 public:
	std::string lineId;
	std::string id;
	std::string
			type;             //!< 0-实线形点,1-虚线起点,2-虚线终点,3-虚线形点，9-补充形点
	std::string segType;  //!< 识别分类
	std::string source;   //!< 每个结点的来源
	
	// only da line
	std::string
			lineType;  //!< 1-总线减速，2-公交车专用线，3-实线，4-虚线，99-其他
	
	// only da line node
	std::string nodeIdx;  //!< 节点序号
	/// 0-正常 1-裁剪点 2-强制打断点 3-Complex Node RNode, 4-Complex Node LNode
	std::string extraType;
	std::vector<double> coord;     //!< 经纬度坐标
	std::vector<double> utmCoord;  //!< UTM坐标
	std::string trackId;           //!< 原始轨迹id
	std::string newTrackId;
	std::string measureId;          //!< 测量帧
	std::string measureFrameIndex;  //!< 测量帧序号
	std::string heightId;           //!< 高度帧
	std::string heightFrameIndex;   //!< 高度帧序号
	double measureDistance;         //!< 测量距离
	double heightDistance;          //!< 高度帧距离
	double curvature;               //!< 离散曲率
	std::vector<double> pixel;      //!< 测量像素点
};

struct DaLaneFeature {
 public:
	std::string type;
	DaLaneGeometry geometry;
	DaLaneProperties properties;
};

class DaLane {
 public:
	std::string type;
	
	std::vector<DaLaneFeature> feature;
};

struct TrackAll : KRSTrack {
	CalibratedSurveyDevice device;
	
	TrackExtend extend;
};
}
