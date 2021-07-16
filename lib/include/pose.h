#pragma once

#include <Eigen/Dense>

#include "structs.h"

namespace jf {

/// POSE with timestamp

struct BLH {
	double lat;
	double lng;
	double height;
	
	friend std::ostream& operator<<(std::ostream& os, const BLH& glh);
};

struct XY {
	XY() = default;
	XY(double x, double y) : x(x), y(y) {}
	
	double x;
	double y;
};

struct BL {
	BL() = default;
	BL(double lat, double lon) : lat(lat), lon(lon) {}
	
	double lat;
	double lon;
};

struct XYZ {
	XYZ() = default;
	XYZ(double x, double y, double z) : x(x), y(y), z(z) {}
	explicit XYZ(const Eigen::Vector3d& xyz) {
		x = xyz[0];
		y = xyz[1];
		z = xyz[2];
	}
	
	double x;
	double y;
	double z;
	
	friend std::ostream& operator<<(std::ostream& os, const XYZ& xyz);
};

template <typename Archive>
inline void serialize(Archive& ar, XYZ& xyz, const unsigned int version) {
	ar& xyz.x;
	ar& xyz.y;
	ar& xyz.z;
}

struct UtmXYZ : XYZ {
	char UTMZone[4] = {0};
};

/**
 * Always ZYX order
 * In radians
 */
struct IMU {
	IMU() = default;
	IMU(double yaw, double pitch, double roll)
			: yaw(yaw), pitch(pitch), roll(roll) {}
	explicit IMU(const Eigen::Vector3d& ypr) {
		yaw = ypr[0];
		pitch = ypr[1];
		roll = ypr[2];
	}
	
	double yaw;
	double pitch;
	double roll;
	
	friend std::ostream& operator<<(std::ostream& os, const IMU& pos);
	inline explicit operator Eigen::Quaterniond() const;
};

inline IMU::operator Eigen::Quaterniond() const {
	return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
	       Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
	       Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
}

template <typename Archive>
inline void serialize(Archive& ar, IMU& imu, const unsigned int version) {
	ar& imu.yaw;
	ar& imu.pitch;
	ar& imu.roll;
}

/// POSE
struct POSE {
	POSE() = default;
	
	POSE(const Eigen::Vector3d& xyz, const Eigen::Vector3d& ypr)
			: xyz(xyz), imu(ypr) {}
	
	POSE(const Eigen::Matrix4d& tf)
			: xyz(tf.topRightCorner<3, 1>()),
			  imu(tf.topLeftCorner<3, 3>().eulerAngles(2, 1, 0)) {}
	
	POSE(const Eigen::Isometry3d& iso)
			: xyz(iso.translation()), imu(iso.rotation().eulerAngles(2, 1, 0)) {}
	
	XYZ xyz;
	IMU imu;
	
	inline explicit operator Eigen::Isometry3d() const;
	
	// convenience method
	inline Eigen::Matrix4d tf() const {
		return Eigen::Isometry3d(*this).matrix();
	}
	// convenience method
	inline Eigen::Isometry3d iso() const { return Eigen::Isometry3d(*this); }
	
	friend std::ostream& operator<<(std::ostream& os, const POSE& pos);
};

inline POSE::operator Eigen::Isometry3d() const {
	Eigen::Isometry3d iso(Eigen::Quaterniond(this->imu));
	iso.translation() = Eigen::Vector3d(this->xyz.x, this->xyz.y, this->xyz.z);
	return iso;
}

template <typename Archive>
inline void serialize(Archive& ar, POSE& pose, const unsigned int version) {
	ar& pose.xyz;
	ar& pose.imu;
}

inline POSE toPose(const Eigen::Matrix4d& tf) {
	Eigen::Vector3d ypr = tf.topLeftCorner<3, 3>().eulerAngles(2, 1, 0);
	Eigen::Vector3d xyz = tf.topRightCorner<3, 1>();
	return {xyz, ypr};
}

inline POSE toPose(const Eigen::Isometry3d& iso) {
	Eigen::Vector3d ypr = iso.rotation().eulerAngles(2, 1, 0);
	Eigen::Vector3d xyz = iso.translation();
	return {xyz, ypr};
}

struct BlhPOSE {
	BLH gps;
	IMU imu;
	
	friend std::ostream& operator<<(std::ostream& os, const BlhPOSE& pos);
};

struct UtcPOS : BlhPOSE {
	UtcPOS() = default;
	UtcPOS(const BlhPOSE& pos) : BlhPOSE(pos) {}
	
	// TODO change to long double
	double utc_time = -1.0;
	
	friend std::ostream& operator<<(std::ostream& os, const UtcPOS& pos);
};

template <typename Archive>
inline void serialize(Archive& ar, UtcPOS& pos, const unsigned int version) {
	ar& pos.utc_time;
	
	ar& pos.gps.lat;
	ar& pos.gps.lng;
	ar& pos.gps.height;
	
	ar& pos.imu.yaw;
	ar& pos.imu.pitch;
	ar& pos.imu.roll;
}

struct FullPpkPOS : UtcPOS {
	FullPpkPOS() = default;
	FullPpkPOS(const UtcPOS& pos) : UtcPOS(pos) {}
	
	uint16_t gps_week;
	
	// within the week
	double gps_seconds;
	
	// within the week
	double utc_seconds;
	
	double v_north;  // m/s
	double v_east;   // m/s
	double v_up;     // m/s
	
	double h_msl_hve;
	double master_lat;
	
	double lat_sigma;     // m
	double lon_sigma;     // m
	double height_sigma;  // m
	
	double v_north_sigma;
	double v_east_sigma;
	double v_up_sigma;
	
	double roll_sigma;
	double pitch_sigma;
	double heading_sigma;
	
	double omega;  // deg
	double phi;    // deg
	double kappa;  // deg
	double hmsl;   // m
	
	// 后处理状态， 1代表fixed, 2代表Float,  3其他
	uint8_t ambStatus;
	
	// number of satelites
	uint8_t num_sat;
	
	// 位置点后差分质量，范围1-6
	uint8_t quality_nav;
	
	// 1代表GPS， 2代表其他
	uint8_t i_flag;
	
	// local_data
	
	// local_time
	
	uint8_t quality_sat;
};

template <typename Archive>
inline void serialize(Archive& ar, FullPpkPOS& pos,
                      const unsigned int version) {
	ar& pos.utc_time;
	
	ar& pos.gps.lat;
	ar& pos.gps.lng;
	ar& pos.gps.height;
	
	ar& pos.imu.yaw;
	ar& pos.imu.pitch;
	ar& pos.imu.roll;
	
	ar& pos.gps_week;
	
	ar& pos.gps_seconds;
	
	ar& pos.utc_seconds;
	
	ar& pos.v_north;
	ar& pos.v_east;
	ar& pos.v_up;
	
	ar& pos.h_msl_hve;
	ar& pos.master_lat;
	
	ar& pos.lat_sigma;
	ar& pos.lon_sigma;
	ar& pos.height_sigma;
	
	ar& pos.v_north_sigma;
	ar& pos.v_east_sigma;
	ar& pos.v_up_sigma;
	
	ar& pos.roll_sigma;
	ar& pos.pitch_sigma;
	ar& pos.heading_sigma;
	
	ar& pos.omega;
	ar& pos.phi;
	ar& pos.kappa;
	ar& pos.hmsl;
	ar& pos.ambStatus;
	
	ar& pos.num_sat;
	
	ar& pos.quality_nav;
	
	ar& pos.i_flag;
	
	// local_data
	
	// local_time
	
	ar& pos.quality_sat;
}

inline UtcPOS trackPoint2POS(const KRSTrackPoint& track_point) {
	UtcPOS data;
	data.utc_time = static_cast<long double>(track_point.locTime) / 1000.0;
	data.gps.lat = track_point.y;
	data.gps.lng = track_point.x;
	data.gps.height = track_point.z;
	data.imu.roll = stod(track_point.roll);
	data.imu.pitch = stod(track_point.pitch);
	data.imu.yaw = stod(track_point.azimuth);
	return data;
}

inline BlhPOSE rectifiedTrackPoint2POSDelta(
		const LidarRectifiedTrackPoint& track_point) {
	BlhPOSE data;
	
	data.gps.lat = track_point.yDelta;
	data.gps.lng = track_point.xDelta;
	data.gps.height = track_point.zDelta;
	data.imu.roll = track_point.rollDelta;
	data.imu.pitch = track_point.pitchDelta;
	data.imu.yaw = track_point.azimuthDelta;
	return data;
}

inline UtcPOS trackPoint2POS(const LidarRectifiedTrackPoint& track_point) {
	UtcPOS data;
	data.utc_time = static_cast<long double>(track_point.locTime) / 1000.0;
	data.gps.lat = track_point.y;
	data.gps.lng = track_point.x;
	data.gps.height = track_point.z;
	data.imu.roll = track_point.roll;
	data.imu.pitch = track_point.pitch;
	data.imu.yaw = track_point.azimuth;
	return data;
}

inline FullPpkPOS trackPoint2FullPpkPOS(const KRSTrackPoint& track_point) {
	FullPpkPOS data;
	data.utc_time = static_cast<long double>(track_point.locTime) / 1000.0;
	data.gps.lat = track_point.y;
	data.gps.lng = track_point.x;
	data.gps.height = track_point.z;
	
	if (!track_point.roll.empty()) data.imu.roll = stod(track_point.roll);
	if (!track_point.pitch.empty()) data.imu.pitch = stod(track_point.pitch);
	if (!track_point.azimuth.empty()) data.imu.yaw = stod(track_point.azimuth);
	
	if (!track_point.northVelocity.empty())
		data.v_north = stod(track_point.northVelocity);
	if (!track_point.eastVelocity.empty())
		data.v_east = stod(track_point.eastVelocity);
	if (!track_point.upVelocity.empty()) data.v_up = stod(track_point.upVelocity);
	
	if (!track_point.longitudeSigma.empty())
		data.lon_sigma = stod(track_point.longitudeSigma);
	if (!track_point.latitudeSigma.empty())
		data.lat_sigma = stod(track_point.latitudeSigma);
	if (!track_point.heightSigma.empty())
		data.height_sigma = stod(track_point.heightSigma);
	
	if (!track_point.rollSigma.empty())
		data.roll_sigma = stod(track_point.rollSigma);
	if (!track_point.pitchSigma.empty())
		data.pitch_sigma = stod(track_point.pitchSigma);
	if (!track_point.azimuthSigma.empty())
		data.heading_sigma = stod(track_point.azimuthSigma);
	
	data.quality_nav = track_point.qualityNum;
	data.ambStatus = track_point.ambStatus;
	data.quality_sat = track_point.gnssQuality;
	
	return data;
}

struct LocalPose {
	double utc_time = -1.0;
	Eigen::Matrix3d R;
	Eigen::Vector3d T;
};

}  // namespace jf
