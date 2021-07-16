cmake_minimum_required(VERSION 3.2)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

project(lidar_imu_calibration_main)

add_executable(${PROJECT_NAME} lidar_imu_calibration_main.cpp)
target_include_directories(${PROJECT_NAME}
		PUBLIC
		${CMAKE_SOURCE_DIR}/lib/include
		)
target_link_libraries(${PROJECT_NAME}
	lidar_imu_calibration_lib)

add_executable(calibration_check_main calibration_check_main.cpp)
target_include_directories(calibration_check_main
		PUBLIC
		${CMAKE_SOURCE_DIR}/lib/include
		)
target_link_libraries(calibration_check_main
		lidar_imu_calibration_lib)

add_executable(gps_utc_transform gps_utc_transform.cpp)
