#pragma once

#include <tbb/concurrent_queue.h>
#include <boost/optional.hpp>
#include <thread>

#include "lidar_type.h"

namespace jf {
class motion_compensation {
 protected:
	std::vector<std::thread> work_threads_;
	std::vector<LidarScanWithOrigin> input_scans_;
	std::vector<LidarScanWithOrigin> output_scans_;
	
 public:
	void run(int num_threads, int input_queue_capacity,
	         int output_queue_capacity);
};

}


