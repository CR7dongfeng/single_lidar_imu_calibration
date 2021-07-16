#pragma once

#include <glog/logging.h>

#include "pose.h"

#include <random>

namespace jf {

inline void correctImuRange(IMU& imu) {
	// TODO unit test
	
	if (imu.roll < -90 || imu.roll > 90) {
		imu.yaw += 180;
		imu.pitch = 180 - imu.pitch;
		imu.roll += 180;
	}
	
	while (imu.pitch > 180) {
		imu.pitch -= 360;
	}
	while (imu.roll > 180) {
		imu.roll -= 360;
	}
	while (imu.yaw < 0) {
		imu.yaw += 360;
	}
	while (imu.yaw > 360) {
		imu.yaw -= 360;
	}
}

class PpkFileReader {
 public:
	PpkFileReader() = default;
	
	explicit PpkFileReader(const std::string& ppk_file_path,
	                       int pos_frequency = 1000);
	
	PpkFileReader(const PpkFileReader&) = delete;
	PpkFileReader(PpkFileReader&& other) = default;
	PpkFileReader& operator=(const PpkFileReader&) = delete;
	PpkFileReader& operator=(PpkFileReader&& other) = default;
	
	virtual ~PpkFileReader() = default;
	
	bool load(const std::string& ppk_file_path, int pos_frequency = 1000);
	
	inline void clear() {
		buff_.clear();
		ppk_sep_ = 'n';
	}
	
	/**
	 *
	 * @param [in] timestamp
	 * @param [out] prevPOS
	 * @param [out] nextPOS
	 * @return true if successfully found two POS data, false if unable to locate
	 * such timestamp in POS file
	 */
	bool read(double timestamp, UtcPOS& prevPOS, UtcPOS& nextPOS) const;
	
	virtual UtcPOS read(double timestamp) const;
	
	inline size_t getBufferSize() const { return buff_.size(); }
	
	/**
	 *
	 * @return  1.2 * 2000 / 18 * pos_frequency_
	 */
	inline size_t getMaxBufferSize() const {
		return size_t(1.2 * 2000 / 18 * pos_frequency_);
	}
	
//	std::vector<UtcPOS>& getBuf() { return buff_; }
 
 protected:
	int pos_frequency_ = 1000;
	
	std::vector<UtcPOS> buff_;
	
	bool parseTrimmedPOSFileLine(std::string& line, UtcPOS& data);
	
	void convert(double yaw_input, double pitch_input, double roll_input,
			double& yaw_output, double& pitch_output, double& roll_output);
 
 private:
	char ppk_sep_ = 'n';
};

void interpolatePosData(const UtcPOS& crnt_pos_data,
                        const UtcPOS& next_pos_data, double lidar_utc_time,
                        UtcPOS& lidar_pos_data);
}  // namespace jf