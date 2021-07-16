#pragma once

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ceres/autodiff_cost_function.h>
#include <ceres/autodiff_local_parameterization.h>
#include <ceres/local_parameterization.h>
#include <ceres/types.h>
#include <ceres/rotation.h>

#include "frame.h"
#include "eigen_quaternion.h"

using namespace std;

namespace jf {

namespace ICP_Ceres {
const int CALIB_R_STEP = 1;
const int CALIB_XY_STEP = 2;

void ceresOptimizer(std::vector<std::shared_ptr<Frame>> &frames,
                    bool pointToPlane, bool robust, int step = CALIB_R_STEP);
}

namespace ICPCostFunctions {

struct PointToPlaneErrorGlobal {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	const Eigen::Vector3d p_dst;
	const Eigen::Vector3d p_src;
	const Eigen::Vector3d p_nor;
	const Eigen::Isometry3d move_t_;
	
	PointToPlaneErrorGlobal(const Eigen::Vector3d &dst,
	                        const Eigen::Vector3d &src,
	                        const Eigen::Vector3d &nor,
	                        const Eigen::Isometry3d &move_t_)
			: p_dst(dst), p_src(src), p_nor(nor), move_t_(move_t_) {}
	
	// Factory to hide the construction of the CostFunction object from the client
	// code.
	static ceres::CostFunction *Create(const Eigen::Vector3d &dst,
	                                   const Eigen::Vector3d &src,
	                                   const Eigen::Vector3d &nor,
	                                   const Eigen::Isometry3d &move_t_) {
		return (new ceres::AutoDiffCostFunction<PointToPlaneErrorGlobal, 1, 4, 3>(
				new PointToPlaneErrorGlobal(dst, src, nor, move_t_)));
	}
	
	template <typename T>
	bool operator()(const T *const extrin_rotation,
	                const T *const extrin_translation, T *residuals) const {
		// Make sure the Eigen::Vector world point is using the ceres::Jet type as
		// it's Scalar type
		Eigen::Matrix<T, 3, 1> src;
		src << T(p_src[0]), T(p_src[1]), T(p_src[2]);
		Eigen::Matrix<T, 3, 1> dst;
		dst << T(p_dst[0]), T(p_dst[1]), T(p_dst[2]);
		Eigen::Matrix<T, 3, 1> nor;
		nor << T(p_nor[0]), T(p_nor[1]), T(p_nor[2]);
		
		Eigen::Matrix<T, 3, 3> move_r;
		move_r << T(move_t_(0, 0)), T(move_t_(0, 1)), T(move_t_(0, 2)),
				T(move_t_(1, 0)), T(move_t_(1, 1)), T(move_t_(1, 2)), T(move_t_(2, 0)),
				T(move_t_(2, 1)), T(move_t_(2, 2));
		Eigen::Matrix<T, 3, 1> move_t;
		move_t << T(move_t_(0, 3)), T(move_t_(1, 3)), T(move_t_(2, 3));
		
		Eigen::Quaternion<T> q =
				Eigen::Map<const Eigen::Quaternion<T>>(extrin_rotation);
		Eigen::Matrix<T, 3, 1> t =
				Eigen::Map<const Eigen::Matrix<T, 3, 1>>(extrin_translation);
		
		Eigen::Matrix<T, 3, 1> p1 = q.toRotationMatrix() * src;
		p1 += t;
		
		Eigen::Matrix<T, 3, 1> p2_ = q.toRotationMatrix() * dst;
		p2_ += t;
		Eigen::Matrix<T, 3, 1> n2_ = q.toRotationMatrix() * nor;
		
		Eigen::Matrix<T, 3, 1> p2 = move_r * p2_;
		p2 += move_t;
		Eigen::Matrix<T, 3, 1> n2 = move_r * n2_;
		
		// The error is the difference between the predicted and observed position
		// projected onto normal
		residuals[0] = (p1 - p2).dot(n2);
		
		return true;
	}
};

struct PointToPointErrorGlobal {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	const Eigen::Vector3d p_dst;
	const Eigen::Vector3d p_src;
	const Eigen::Isometry3d move_t_;
	
	PointToPointErrorGlobal(const Eigen::Vector3d &dst,
	                        const Eigen::Vector3d &src,
	                        const Eigen::Isometry3d &move_t_)
			: p_dst(dst), p_src(src), move_t_(move_t_) {}
	
	// Factory to hide the construction of the CostFunction object from the client
	// code.
	static ceres::CostFunction *Create(const Eigen::Vector3d &dst,
	                                   const Eigen::Vector3d &src,
	                                   const Eigen::Isometry3d &move_t_) {
		return (new ceres::AutoDiffCostFunction<PointToPointErrorGlobal, 3, 4, 3>(
				new PointToPointErrorGlobal(dst, src, move_t_)));
	}
	
	template <typename T>
	bool operator()(const T *const extrin_rotation,
	                const T *const extrin_translation, T *residuals) const {
		// Make sure the Eigen::Vector world point is using the ceres::Jet type as
		// it's Scalar type
		Eigen::Matrix<T, 3, 1> src;
		src << T(p_src[0]), T(p_src[1]), T(p_src[2]);
		Eigen::Matrix<T, 3, 1> dst;
		dst << T(p_dst[0]), T(p_dst[1]), T(p_dst[2]);
		
		Eigen::Matrix<T, 3, 3> move_r;
		move_r << T(move_t_(0, 0)), T(move_t_(0, 1)), T(move_t_(0, 2)),
				T(move_t_(1, 0)), T(move_t_(1, 1)), T(move_t_(1, 2)), T(move_t_(2, 0)),
				T(move_t_(2, 1)), T(move_t_(2, 2));
		Eigen::Matrix<T, 3, 1> move_t;
		move_t << T(move_t_(0, 3)), T(move_t_(1, 3)), T(move_t_(2, 3));
		
		Eigen::Quaternion<T> q =
				Eigen::Map<const Eigen::Quaternion<T>>(extrin_rotation);
		Eigen::Matrix<T, 3, 1> t =
				Eigen::Map<const Eigen::Matrix<T, 3, 1>>(extrin_translation);
		
		Eigen::Matrix<T, 3, 1> p1 = q.toRotationMatrix() * src;
		p1 += t;
		
		Eigen::Matrix<T, 3, 1> p2_ = q.toRotationMatrix() * dst;
		p2_ += t;
		
		Eigen::Matrix<T, 3, 1> p2 = move_r * p2_;
		p2 += move_t;
		
		// The error is the difference between the predicted and observed position
		// projected onto normal
		residuals[0] = p1[0] - p2[0];
		residuals[1] = p1[1] - p2[1];
		residuals[2] = p1[2] - p2[2];
		
		return true;
	}
};

struct PointToPlaneErrorGlobal_FixZ {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	const Eigen::Vector3d p_dst;
	const Eigen::Vector3d p_src;
	const Eigen::Vector3d p_nor;
	const Eigen::Isometry3d move_t_;
	
	PointToPlaneErrorGlobal_FixZ(const Eigen::Vector3d &dst,
	                             const Eigen::Vector3d &src,
	                             const Eigen::Vector3d &nor,
	                             const Eigen::Isometry3d &move_t_)
			: p_dst(dst), p_src(src), p_nor(nor), move_t_(move_t_) {}
	
	// Factory to hide the construction of the CostFunction object from the client
	// code.
	static ceres::CostFunction *Create(const Eigen::Vector3d &dst,
	                                   const Eigen::Vector3d &src,
	                                   const Eigen::Vector3d &nor,
	                                   const Eigen::Isometry3d &move_t_) {
		return (new ceres::AutoDiffCostFunction<PointToPlaneErrorGlobal_FixZ, 1, 4,
				1, 1, 1>(
				new PointToPlaneErrorGlobal_FixZ(dst, src, nor, move_t_)));
	}
	
	template <typename T>
	bool operator()(const T *const extrin_rotation, const T *const extrin_x,
	                const T *const extrin_y, const T *const extrin_z,
	                T *residuals) const {
		// Make sure the Eigen::Vector world point is using the ceres::Jet type as
		// it's Scalar type
		Eigen::Matrix<T, 3, 1> src;
		src << T(p_src[0]), T(p_src[1]), T(p_src[2]);
		Eigen::Matrix<T, 3, 1> dst;
		dst << T(p_dst[0]), T(p_dst[1]), T(p_dst[2]);
		Eigen::Matrix<T, 3, 1> nor;
		nor << T(p_nor[0]), T(p_nor[1]), T(p_nor[2]);
		
		Eigen::Matrix<T, 3, 3> move_r;
		move_r << T(move_t_(0, 0)), T(move_t_(0, 1)), T(move_t_(0, 2)),
				T(move_t_(1, 0)), T(move_t_(1, 1)), T(move_t_(1, 2)), T(move_t_(2, 0)),
				T(move_t_(2, 1)), T(move_t_(2, 2));
		Eigen::Matrix<T, 3, 1> move_t;
		move_t << T(move_t_(0, 3)), T(move_t_(1, 3)), T(move_t_(2, 3));
		
		Eigen::Quaternion<T> q =
				Eigen::Map<const Eigen::Quaternion<T>>(extrin_rotation);
		Eigen::Matrix<T, 3, 1> t{T(*extrin_x), T(*extrin_y), T(*extrin_z)};
		
		Eigen::Matrix<T, 3, 1> p1 = q.toRotationMatrix() * src;
		p1 += t;
		
		Eigen::Matrix<T, 3, 1> p2_ = q.toRotationMatrix() * dst;
		p2_ += t;
		Eigen::Matrix<T, 3, 1> n2_ = q.toRotationMatrix() * nor;
		
		Eigen::Matrix<T, 3, 1> p2 = move_r * p2_;
		p2 += move_t;
		Eigen::Matrix<T, 3, 1> n2 = move_r * n2_;
		
		// The error is the difference between the predicted and observed position
		// projected onto normal
		residuals[0] = (p1 - p2).dot(n2);
		
		return true;
	}
};
}
}
