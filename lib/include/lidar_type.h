#pragma once

#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <pcl/point_types.h>

#include "pose.h"

namespace jf {

struct LidarPoint {
	inline LidarPoint() { data[3] = 1.0; }
	
	// WARNING: LidarPoint is not trivially-copyable due to Eigen member.
	// so memcpy and memmove is UB..
	// BUT so is union.., so either keep union and memcpy/memmove, or remove all
	// of them.
	LidarPoint(const LidarPoint& point) { memcpy(this, &point, sizeof(*this)); }
	
	LidarPoint& operator=(const LidarPoint& point) {
		memcpy(this, &point, sizeof(*this));
	}
	
	LidarPoint(LidarPoint&& point) noexcept {
		memmove(this, &point, sizeof(*this));
	}
 
 public:
	union {
		double data[4];
		struct {
			double x;
			double y;
			double z;
		};
		Eigen::Vector4d vector4d;
		Eigen::Vector3d vector3d;
	};
	
	uint32_t secs;
	uint32_t nsecs;
	
	float intensity;
	
	inline long double getTimestamp() const {
		return secs + (long double)(nsecs) / (long double)(1000000000.0);
	}
	
	inline uint64_t getTimestampMili() const {
		return uint64_t(secs) * 1000UL + nsecs / 1000000;
	}
	
	inline double getDistance() const {
		return sqrt(x*x + y*y + z*z);
	}
	
	inline explicit operator pcl::PointXYZ() const;
	
	inline explicit operator pcl::PointXYZI() const;
	
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN64;

/// cast operator to pcl point types
inline LidarPoint::operator pcl::PointXYZ() const {
	pcl::PointXYZ pt;
	pt.x = float(this->x);
	pt.y = float(this->y);
	pt.z = float(this->z);
	return pt;
}

inline LidarPoint::operator pcl::PointXYZI() const {
	pcl::PointXYZI pt;
	pt.x = float(this->x);
	pt.y = float(this->y);
	pt.z = float(this->z);
	pt.intensity = float(this->intensity);
	return pt;
}

/// Lidar Scan
using LidarScan = std::vector<LidarPoint, Eigen::aligned_allocator<LidarPoint>>;
using LidarScanPtr = std::shared_ptr<LidarScan>;
using LidarScanConstPtr = std::shared_ptr<const LidarScan>;

/// Lidar scan with origin

struct LidarScanWithOrigin {
	LidarScanWithOrigin() = default;
	
	explicit LidarScanWithOrigin(long double _origin_timestamp,
	                             LidarScan _lidar_points)
			: origin_timestamp(_origin_timestamp), scan(std::move(_lidar_points)) {}
	// utc timestamp
	long double origin_timestamp = -1.0;
	BlhPOSE pose;
	LidarScan scan;
};
using LidarScanWithOriginPtr = std::shared_ptr<LidarScanWithOrigin>;
using LidarScanWithOriginConstPtr = std::shared_ptr<const LidarScanWithOrigin>;

using LocalLidarScan = LidarScanWithOrigin;
using LocalLidarScanPtr = LidarScanWithOriginPtr;
using LocalLidarScanConstPtr = LidarScanWithOriginConstPtr;

}