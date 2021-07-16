#include "ppk_file_reader.h"

#include <glog/logging.h>
#include <fstream>
#include <iomanip>
#include <string>

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "scan_input_calib.h"

//#include "coordinate_transformation.h"
//#include "device_utils.hpp"

namespace jf {

using namespace std;



void PpkFileReader::convert(double yaw_input, double pitch_input, double roll_input,
                            double& yaw_output, double& pitch_output, double& roll_output) {
	Eigen::Matrix3d R_raw = yprDegrees2C_n_b(yaw_input, pitch_input, roll_input);
	Eigen::Matrix3d R_delta = (Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY())).toRotationMatrix();
	Eigen::Matrix3d R = R_raw * R_delta;
	
	Eigen::Vector3d ypr = R.eulerAngles(2, 0, 1);
	yaw_output = -ypr[0]*180.0/M_PI;
	pitch_output = ypr[1]*180.0/M_PI;
	roll_output = ypr[2]*180.0/M_PI;
}

vector<string> stringSplit(const string& s, const string& delimiter) {
	size_t pos1 = 0;
	size_t pos2 = 0;
	
	vector<string> result;
	while ((pos2 = s.find(delimiter, pos1)) != string::npos) {
		if (pos1 != pos2) result.emplace_back(s.substr(pos1, pos2 - pos1));
		
		pos1 = pos2 + delimiter.length();
	}
	if (pos1 != s.size()) result.push_back(s.substr(pos1));
	return result;
}

vector<string> stringSplit(const string& s, const char delimiter) {
	vector<string> result;
	stringstream ss(s);
	string item;
	while (getline(ss, item, delimiter))
		if (!item.empty()) result.emplace_back(move(item));
	
	return result;
}

bool PpkFileReader::parseTrimmedPOSFileLine(string& line, UtcPOS& data) {
	if (!isdigit(line[0])) return false;
	
	const int item_num = 22;
	
	while (!line.empty() && !isalnum(line.back())) line.pop_back();
	
	if (ppk_sep_ == 'n') {
		vector<string> space_vector = stringSplit(line, ' ');

		vector<string> tab_vector = stringSplit(line, '\t');
		vector<string> comma_vector = stringSplit(line, ',');
		
//		std::cout << "comma_vector.size()" << comma_vector.size() << std::endl;

		if (space_vector.size() == item_num) {
			ppk_sep_ = ' ';
		} else if (tab_vector.size() == item_num) {
			ppk_sep_ = '\t';
		} else if (comma_vector.size() == item_num) {
			ppk_sep_ = ',';
		} else {
			LOG_EVERY_N(WARNING, 99999)
				<< "Line in ppk file does not contain 7 values,"
				   " skipping and suppressing similar warnings, line : \n"
				<< line;
			return false;
		}
	}
	
	vector<string> str_vector = stringSplit(line, ppk_sep_);
	if (str_vector.size() != item_num) {
		LOG_EVERY_N(WARNING, 99999)
			<< "Line in ppk file does not contain 7 values,"
			   " skipping and suppressing similar warnings, line : \n"
			<< line;
		return false;
	}
	
	try {
		int gps_week = stod(str_vector[0]);
		double gps_sec = stod(str_vector[1]);
		data.utc_time = 315936000 + gps_week*7*24*3600 + gps_sec -18;
		data.gps.lat = stod(str_vector[2]);
		data.gps.lng = stod(str_vector[3]);
		data.gps.height = stod(str_vector[4]);

//		data.imu.roll = stod(str_vector[5]);
//		data.imu.pitch = stod(str_vector[6]);
//		data.imu.yaw = stod(str_vector[7]);
		data.imu.roll = stod(str_vector[5+3]);
		data.imu.pitch = stod(str_vector[6+3]);
		data.imu.yaw = stod(str_vector[7+3]);
		
		if(data.imu.roll < -90)
			data.imu.roll += 180;
		if(data.imu.roll > 90)
			data.imu.roll -= 180;
		
//		double _roll = stod(str_vector[5+3]);
//		double _pitch = stod(str_vector[6+3]);
//		double _yaw = stod(str_vector[7+3]);

//		convert(_yaw, _pitch, _roll, data.imu.yaw, data.imu.pitch, data.imu.roll);

//		correctImuRange(data.imu);

//		cout << "data.utc_time : " << data.utc_time << endl;
//		cout << "data.gps.lat : " << data.gps.lat << endl;
//		cout << "data.gps.lng : " << data.gps.lng << endl;
//		cout << "data.gps.height : " << data.gps.height << endl;
//		cout << "data.imu.roll : " << data.imu.roll << endl;
//		cout << "data.imu.pitch : " << data.imu.pitch << endl;
//		cout << "data.imu.yaw : " << data.imu.yaw << endl;
		
	} catch (const invalid_argument& e) {
		LOG(ERROR) << e.what();
		return false;
	}
	
	return true;
}

PpkFileReader::PpkFileReader(const std::string& ppk_file_path,
                             int pos_frequency)
		: pos_frequency_(pos_frequency) {
	if (!load(ppk_file_path)) {
		string msg = "Unable to load given file: " + ppk_file_path;
		throw runtime_error(msg);
	}
}

bool PpkFileReader::load(const std::string& ppk_file_path, int pos_frequency) {
	pos_frequency_ = pos_frequency;
	clear();
	ifstream ifs(ppk_file_path);
	if (!ifs.is_open()) {
		string msg = "The given ppk file cannot be opened : " + ppk_file_path;
		LOG(ERROR) << msg;
		return false;
	}
	
	string line;
	while (getline(ifs, line) && !line.empty() && !isdigit(line[0]) &&
	       !isdigit(line[1])) {	}
	
	const auto max_queue_size = size_t(1.2 * 2000 * 10 / 18 * pos_frequency_);
	
	buff_.reserve(max_queue_size);
	
	buff_.emplace_back();
	if (!parseTrimmedPOSFileLine(line, buff_.back())) {
		string msg = "The first data line cannot be parsed : " + ppk_file_path;
		LOG(ERROR) << msg;
		return false;
	}
	
	while (buff_.size() < max_queue_size && getline(ifs, line)) {
		buff_.emplace_back();
		if (!parseTrimmedPOSFileLine(line, buff_.back())) {
			buff_.pop_back();
			continue;
		}
	}
	
	if (buff_.size() >= max_queue_size) {
		LOG(ERROR) << "The given ppk file exceeds max buffer size,  aborting.... ";
		return false;
	}
	
	LOG(INFO) << "Finished loading ppk file, number of POS : " << buff_.size()
	          << fixed << setprecision(6)
	          << ", first time : " << buff_.front().utc_time
	          << ", last time : " << buff_.back().utc_time;
	return true;
}

bool PpkFileReader::read(double timestamp, UtcPOS& prevPOS,
                         UtcPOS& nextPOS) const {
	if (buff_.empty()) {
		string msg = "Attempting to use PpkFileReader without initilization";
		LOG(ERROR) << msg;
		throw runtime_error(msg);
	}
	
	if (buff_.front().utc_time > timestamp) {
		LOG_EVERY_N(WARNING, 20000)
			<< fixed << setprecision(6) << "Trying to get timestamp " << timestamp
			<< " earlier than earliest in ppk " << buff_.front().utc_time;
		return false;
	}
	if (timestamp > buff_.back().utc_time) {
		LOG_EVERY_N(WARNING, 20000)
			<< fixed << setprecision(6) << "Trying to get timestamp " << timestamp
			<< " later than latest in ppk " << buff_.back().utc_time;
		return false;
	}
	
	auto prev_idx = size_t((timestamp - buff_.front().utc_time) * pos_frequency_);
	
	if (prev_idx + 1 >= buff_.size()) {
		if (timestamp - buff_.back().utc_time < numeric_limits<double>::epsilon()) {
			prev_idx = buff_.size() - 2;
		} else {
			return false;
		}
	}
	
	while (buff_[prev_idx].utc_time > timestamp && prev_idx > 0) {
		--prev_idx;
	}
	while (buff_[prev_idx + 1].utc_time < timestamp &&
	       prev_idx < buff_.size() - 2) {
		++prev_idx;
	}
	
	prevPOS = buff_[prev_idx];
	nextPOS = buff_[prev_idx + 1];
	
	DCHECK(prevPOS.utc_time <= timestamp);
	DCHECK(nextPOS.utc_time >= timestamp);
	
	return true;
}

UtcPOS PpkFileReader::read(double timestamp) const {
	UtcPOS prevPOS, nextPOS, currPOS;
	
	if (this->read(timestamp, prevPOS, nextPOS))
		interpolatePosData(prevPOS, nextPOS, timestamp, currPOS);
	
	return currPOS;
}



void interpolatePosData(const UtcPOS& crnt_pos_data,
                        const UtcPOS& next_pos_data, double lidar_utc_time,
                        UtcPOS& lidar_pos_data) {
	//差值计算当前时刻ins输出
	double ratio = (lidar_utc_time - crnt_pos_data.utc_time) /
	               (next_pos_data.utc_time - crnt_pos_data.utc_time);
	
	lidar_pos_data = crnt_pos_data;
	lidar_pos_data.utc_time = lidar_utc_time;
	
	lidar_pos_data.gps.lng +=
			ratio * (next_pos_data.gps.lng - crnt_pos_data.gps.lng);
	lidar_pos_data.gps.lat +=
			ratio * (next_pos_data.gps.lat - crnt_pos_data.gps.lat);
	lidar_pos_data.gps.height +=
			ratio * (next_pos_data.gps.height - crnt_pos_data.gps.height);
	lidar_pos_data.imu.pitch +=
			ratio * (next_pos_data.imu.pitch - crnt_pos_data.imu.pitch);
	lidar_pos_data.imu.yaw +=
			ratio * (next_pos_data.imu.yaw - crnt_pos_data.imu.yaw);
	lidar_pos_data.imu.roll +=
			ratio * (next_pos_data.imu.roll - crnt_pos_data.imu.roll);
}

}  // namespace jf