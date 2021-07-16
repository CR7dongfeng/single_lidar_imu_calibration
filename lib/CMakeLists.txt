cmake_minimum_required(VERSION 3.2)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
project(lidar_imu_calibration_lib)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}/cmake)

find_package(Eigen3)

find_package(PCL REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(GeographicLib REQUIRED)

find_package(TBB REQUIRED)

find_package(Glog REQUIRED)
find_package(GFlags REQUIRED)

file(GLOB_RECURSE ${PROJECT_NAME}_CPP_FILES src/*.cpp src/*.c)
#add_library(${PROJECT_NAME} ${${PROJECT_NAME}_CPP_FILES} include/icp-ceres.h include/eigen_quaternion.h include/nanoflann.hpp)
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_CPP_FILES} )

target_include_directories(${PROJECT_NAME}
		PUBLIC
	${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
	${CERES_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${GeographicLib_INCLUDE_DIRS}
		${OpenCV_INCLUDE_DIRS}
		${TBB_INCLUDE_DIRS}
		${GLOG_INCLUDE_DIRS}
		${GLFAGS_INCLUDE_DIRS}
	)
target_link_directories(${PROJECT_NAME}
		PUBLIC
		${OpenCV_LIBRARY_DIRS}
		)
target_link_libraries(${PROJECT_NAME}
	${PCL_LIBRARIES}
	${CERES_LIBRARIES}
	${GeographicLib_LIBRARIES}
		${OpenCV_LIBS}
		${TBB_LIBRARIES}
		${GLOG_LIBRARIES}
		${GFLAGS_LIBRARIES}
	)




