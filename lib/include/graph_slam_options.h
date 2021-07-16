#pragma once

#include <string>

namespace jf {

struct GraphSlamOptions {
	int pos_frequency = 1000;
	
	int lidar_frequency = 10;
	
	// minimum radius of lidar point to keep
	double lidar_min_radius = 3.0;
	
	// maximum radius of lidar point to keep
	double lidar_max_radius = 200.0;
	
	// maximum y coordinate in camera frame of lidar point to keep
	double lidar_camdra_max_y = 35.0;
	
	int tile_min_density = 20;
	
	float tile_voxel_filter_leaf_size = 0.01f;
	
	int tile_max_density_thres = 200;
	
	int cpu_threads = 6;
};
}