#include "frame.h"
#include <algorithm>
#include <pcl/common/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>


namespace jf {
Eigen::Isometry3d Frame::extrinsic_;

Frame::~Frame() {
    if(index_computed_) {
        delete index_ptr_;
        index_computed_ = false;
    }
}

bool Frame::loadCloud(const string &filename, int whichCLD) {
	bool ret = true;
	
	if (whichCLD == 1) {
		pcl::io::loadPCDFile(filename, *cloud1_);
	} else if (whichCLD == 2) {
		pcl::io::loadPCDFile(filename, *cloud2_);
	} else {
		ret = false;
		std::cout << "Parameter error!" << std::endl;
	}
	
	return ret;
}

void Frame::loadMoveT(const string &filename) {
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	cv::Mat tmpmat = cv::Mat::eye(4, 4, CV_64FC1);
	fs["deltaIMU matrix T"] >> tmpmat;
	
	cv::cv2eigen(tmpmat, move_t_.matrix());
}

void Frame::calculateNormal() {
	PointCloud::Ptr cloud1_(new PointCloud);
	PointCloud::Ptr cloud2_(new PointCloud);
	
	pcl::NormalEstimation<PointT, PointT> norm_est;
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(10);
	norm_est.setInputCloud(cloud1_);
	norm_est.compute(*cloud1_);
	norm_est.setInputCloud(cloud2_);
	norm_est.compute(*cloud2_);
	
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud1_, *cloud1_, indices);
	pcl::removeNaNFromPointCloud(*cloud2_, *cloud2_, indices);
}

void Frame::filter(bool downsampling, bool outlier_filter, bool removeNaN) {
	if (downsampling) {
		pcl::VoxelGrid<PointT> grid;
		
		grid.setLeafSize(0.1, 0.1, 0.1);
		grid.setInputCloud(cloud1_);
		grid.filter(*cloud1_);
		grid.setInputCloud(cloud2_);
		grid.filter(*cloud2_);
	}
	
	if (outlier_filter) {
		pcl::StatisticalOutlierRemoval<PointT> sor;
		sor.setMeanK(50);
		sor.setStddevMulThresh(1.0);
		sor.setInputCloud(cloud1_);
		sor.filter(*cloud1_);
		sor.setInputCloud(cloud2_);
		sor.filter(*cloud2_);
	}
	
	if (removeNaN) {
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*cloud1_, *cloud1_, indices);
		pcl::removeNaNFromPointCloud(*cloud2_, *cloud2_, indices);
	}
}

void Frame::filterCurvature() {
	pcl::ConditionOr<pcl::PointNormal>::Ptr curva_cond(
			new pcl::ConditionOr<pcl::PointNormal>);
	curva_cond->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr(
			new pcl::FieldComparison<pcl::PointNormal>(
					"curvature", pcl::ComparisonOps::LT, 0.07)));
	pcl::ConditionalRemoval<pcl::PointNormal> condrem;
	condrem.setCondition(curva_cond);
	condrem.setInputCloud(this->cloud1_);
	condrem.filter(*(this->cloud1_));
	condrem.setInputCloud(this->cloud2_);
	condrem.filter(*(this->cloud2_));
	
	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setMeanK(10);
	sor.setStddevMulThresh(1.0);
	sor.setInputCloud(cloud1_);
	sor.filter(*cloud1_);
	sor.setInputCloud(cloud2_);
	sor.filter(*cloud2_);
}

void Frame::copyToEigen() {
	for (auto &point : cloud1_->points) {
		Eigen::Vector3d pt, n;
		pt(0) = point.x;
		pt(1) = point.y;
		pt(2) = point.z;
		n(0) = point.normal_x;
		n(1) = point.normal_y;
		n(2) = point.normal_z;
		
		pts1_.push_back(pt);
		nor1_.push_back(n);
	}
	for (auto &point : cloud2_->points) {
		pts2_.push_back(Eigen::Vector3d{point.x, point.y, point.z});
		nor2_.push_back(
				Eigen::Vector3d{point.normal_x, point.normal_y, point.normal_z});
	}
}

double Frame::getClosestPoint(
		const Eigen::Vector3d &query_pt,
		size_t &ret_index) {  // query_pt must be in dstFrame
	if (!index_computed_) {
		index_ptr_ = new MyKdTreeT(
				3 /*dim*/, *this,
				nanoflann::KDTreeSingleIndexAdaptorParams(1 /* max leaf */));
		index_ptr_->buildIndex();
		//        cout << "flann: build index" << endl;
		index_computed_ = true;
	}
	// do a knn search
	const size_t num_results = 1;
	// size_t ret_index;
	double out_dist_sqr;
	nanoflann::KNNResultSet<double> resultSet(num_results);
	resultSet.init(&ret_index, &out_dist_sqr);
	
	index_ptr_->findNeighbors(resultSet, &query_pt[0],
	                          nanoflann::SearchParams(32, 0, false));
	
	return out_dist_sqr;
}

void Frame::getClosestPoints(double distance_thresh) {
	corr_vec_.clear();
	vector<Eigen::Vector3d> pts_temp1, pts_temp2;
	Eigen::Isometry3d inv_move_t = move_t_.inverse();
	Eigen::Isometry3d inv_extrinsic = extrinsic_.inverse();
	
	for (size_t i = 0; i < pts1_.size(); i++) {
		Eigen::Vector3d pt;
		pt = extrinsic_.linear() * pts1_[i] + extrinsic_.translation();
		pt = inv_move_t.linear() * pt + inv_move_t.translation();
		pt = inv_extrinsic.linear() * pt + inv_extrinsic.translation();
		
		pts_temp1.push_back(pt);
	}
	
	std::vector<double> dists;
	
	for (size_t i = 0; i < pts_temp1.size(); i++) {
		size_t idx_min;
		double point_dist_squared;
		
		point_dist_squared = this->getClosestPoint(pts_temp1[i], idx_min);
		
		double point_dist = 1e10;
		point_dist = sqrt(point_dist_squared);
		
		if (point_dist < distance_thresh) {
			this->corr_vec_.push_back({(int)i, (int)idx_min, point_dist});
			dists.push_back(point_dist);
		}
	}
	
	std::vector<double>::iterator middle = dists.begin() + (dists.size() / 2);
	std::nth_element(dists.begin(), middle, dists.end());
	
	double nthValue;
	if (dists.size() > 0)
		nthValue = *middle;
	else
		nthValue = 0.5;
	
	weight_ = nthValue * 1.5;
}

void Frame::filterPtsByCorrespondences(double thresh) {
	index_computed_ = false;
	
	std::cout << "before filterPtsByCorrespondences : pts1 num is "
	          << pts1_.size() << endl;
	std::cout << "before filterPtsByCorrespondences : pts2 num is "
	          << pts2_.size() << endl;
	getClosestPoints(thresh);
	vector<Eigen::Vector3d> pts1_tmp, nor1_tmp;
	for (size_t i = 0; i < corr_vec_.size(); i++) {
		pts1_tmp.emplace_back(pts1_.at(corr_vec_[i].first_));
		nor1_tmp.emplace_back(nor1_.at(corr_vec_[i].first_));
	}
	swap(pts1_, pts1_tmp);
	swap(nor1_, nor1_tmp);
	
	swap(pts1_, pts2_);
	swap(nor1_, nor2_);
	Eigen::Isometry3d move_t_tmp = move_t_;
	move_t_ = move_t_.inverse();
	
	index_computed_ = false;
	
	getClosestPoints(thresh);
	vector<Eigen::Vector3d> pts2_tmp, nor2_tmp;
	for (size_t i = 0; i < corr_vec_.size(); i++) {
		pts2_tmp.emplace_back(pts1_.at(corr_vec_[i].first_));
		nor2_tmp.emplace_back(nor1_.at(corr_vec_[i].first_));
	}
	
	swap(pts1_, pts2_tmp);
	swap(nor1_, nor2_tmp);
	
	swap(pts1_, pts2_);
	swap(nor1_, nor2_);
	
	move_t_ = move_t_tmp;
	
	index_computed_ = false;
	
	std::cout << "after filterPtsByCorrespondences : pts1 num is " << pts1_.size()
	          << endl;
	std::cout << "after filterPtsByCorrespondences : pts2 num is " << pts2_.size()
	          << endl;
}

void Frame::filterCorrespondences() {
	std::sort(corr_vec_.begin(), corr_vec_.end(), comp);
	
	for (size_t i = 1; i < corr_vec_.size(); i++) {
		if (corr_vec_[i].second_ == corr_vec_[i - 1].second_) {
			if (corr_vec_[i].dist_ > corr_vec_[i - 1].dist_)
				swap(corr_vec_[i], corr_vec_[i - 1]);
		}
	}
	
	std::vector<Correspondence> filtered_corr_vec;
	for (size_t i = 0; i < corr_vec_.size(); i++) {
		if (i == corr_vec_.size() - 1) {
			filtered_corr_vec.push_back(corr_vec_[i]);
		} else if (corr_vec_[i].second_ != corr_vec_[i + 1].second_) {
			filtered_corr_vec.push_back(corr_vec_[i]);
		}
	}
	
	swap(corr_vec_, filtered_corr_vec);
}

void Frame::filterCorrespondencesByNormal() {
	std::vector<Correspondence> filtered_corr_vec;
	for (size_t i = 1; i < corr_vec_.size(); i++) {
		Eigen::Vector3d normal1 = nor1_.at(corr_vec_[i].first_);
		Eigen::Vector3d normal2_ = nor2_.at(corr_vec_[i].second_);
		Eigen::Vector3d normal2 = Frame::extrinsic_.linear().inverse() *
		                          move_t_.linear() * Frame::extrinsic_.linear() *
		                          normal2_;
		double match = normal1.dot(normal2);
		if (fabs(match) > 0.996) filtered_corr_vec.push_back(corr_vec_[i]);
	}
	//    std::cout << "before normal filter, num of corr is :" <<
	//    corr_vec_.size()
	//    << std::endl;
	swap(corr_vec_, filtered_corr_vec);
	//    std::cout << "after normal filter, num of corr is :" << corr_vec_.size()
	//    << std::endl;
}

namespace {
void cloudFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, bool downsampling,
                 bool outlierfilter, bool removeNaN) {
	if (downsampling) {
		pcl::VoxelGrid<pcl::PointXYZI> grid;
		
		grid.setLeafSize(0.1, 0.1, 0.1);
		grid.setInputCloud(cloud);
		grid.filter(*cloud);
	}
	
	if (outlierfilter) {
		pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
		sor.setMeanK(50);
		sor.setStddevMulThresh(1.0);
		sor.setInputCloud(cloud);
		sor.filter(*cloud);
	}
	
	if (removeNaN) {
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
	}
}

PointCloud::Ptr cloudCalculateNormal(
		pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_in) {
	PointCloud::Ptr cloud_out(new PointCloud);
	pcl::copyPointCloud(*cloud_in, *cloud_out);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_out, *cloud_out, indices);
	
	pcl::NormalEstimation<pcl::PointXYZI, PointT> norm_est;
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
			new pcl::search::KdTree<pcl::PointXYZI>());
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(10);
	norm_est.setInputCloud(cloud_in);
	norm_est.compute(*cloud_out);
	
//	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud_out, *cloud_out, indices);
	
	return cloud_out;
}

void cloudFilterCurvature(PointCloud::Ptr &cloud) {
	pcl::ConditionOr<pcl::PointNormal>::Ptr curva_cond(
			new pcl::ConditionOr<pcl::PointNormal>);
	curva_cond->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr(
			new pcl::FieldComparison<pcl::PointNormal>(
					"curvature", pcl::ComparisonOps::LT, 0.07)));
	pcl::ConditionalRemoval<pcl::PointNormal> condrem;
	condrem.setCondition(curva_cond);
	condrem.setInputCloud(cloud);
	condrem.filter(*cloud);
	
	//    pcl::RadiusOutlierRemoval<PointT> outrem;
	//    outrem.setInputCloud(cloud);
	//    outrem.setRadiusSearch(0.8);
	//    outrem.setMinNeighborsInRadius(4);
	//    outrem.filter(*cloud);
	PointCloud::Ptr out(new PointCloud);
	
	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setMeanK(10);
	sor.setStddevMulThresh(1.0);
	sor.setInputCloud(cloud);
	sor.filter(*cloud);
}
}  // namespace

PointCloud::Ptr cloudPreProcess(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,
                                bool downsampling, bool outlierfilter,
                                bool removeNaN) {
	cloudFilter(cloud_in, downsampling, outlierfilter, removeNaN);
	
	PointCloud::Ptr cloud_out = cloudCalculateNormal(cloud_in);
	
	cloudFilterCurvature(cloud_out);
	
	return cloud_out;
}
}