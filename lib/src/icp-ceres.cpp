#include "icp-ceres.h"

#include <math.h>
#include <vector>
#include <unordered_map>

#include <Eigen/Dense>
#include <ceres/autodiff_local_parameterization.h>
#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>
#include <ceres/rotation.h>
#include <ceres/types.h>
#include <ceres/loss_function.h>

#define useLocalParam

namespace jf {

namespace ICP_Ceres {

ceres::Solver::Options getOptions() {
	// Set a few options
	ceres::Solver::Options options;
	// options.use_nonmonotonic_steps = true;
	// options.preconditioner_type = ceres::IDENTITY;
	options.linear_solver_type = ceres::DENSE_QR;
	options.max_num_iterations = 50;
	
	//    options.preconditioner_type = ceres::SCHUR_JACOBI;
	//    options.linear_solver_type = ceres::DENSE_SCHUR;
	//    options.use_explicit_schur_complement=true;
	//    options.max_num_iterations = 100;
	
	std::cout << "Ceres Solver getOptions()" << endl;
	std::cout << "Ceres preconditioner type: " << options.preconditioner_type
	          << endl;
	std::cout << "Ceres linear algebra type: "
	          << options.sparse_linear_algebra_library_type << endl;
	std::cout << "Ceres linear solver type: " << options.linear_solver_type
	          << endl;
	
	return options;
}

ceres::Solver::Options getOptionsMedium() {
	// Set a few options
	ceres::Solver::Options options;

#ifdef _WIN32
	options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.preconditioner_type = ceres::SCHUR_JACOBI;
#else
	// options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
#endif  // _WIN32
	
	// If you are solving small to medium sized problems, consider setting
	// Solver::Options::use_explicit_schur_complement to true, it can result in a
	// substantial performance boost.
	options.use_explicit_schur_complement = true;
	options.max_num_iterations = 50;
	
	std::cout << "Ceres Solver getOptionsMedium()" << endl;
	std::cout << "Ceres preconditioner type: " << options.preconditioner_type
	          << endl;
	std::cout << "Ceres linear algebra type: "
	          << options.sparse_linear_algebra_library_type << endl;
	std::cout << "Ceres linear solver type: " << options.linear_solver_type
	          << endl;
	
	return options;
}

void solve(ceres::Problem &problem, bool smallProblem = false) {
	ceres::Solver::Summary summary;
	ceres::Solve(smallProblem ? getOptions() : getOptionsMedium(), &problem,
	             &summary);
	if (!smallProblem) std::cout << "Final report:\n" << summary.FullReport();
}

Eigen::Isometry3d eigenQuaternionToIso(const Eigen::Quaterniond &q,
                                       const Eigen::Vector3d &t) {
	Eigen::Isometry3d poseFinal = Eigen::Isometry3d::Identity();
	poseFinal.linear() = q.toRotationMatrix();
	poseFinal.translation() = t;
	return poseFinal;  //.cast<float>();
}

void ceresOptimizer(std::vector<std::shared_ptr<Frame>> &frames,
                    bool pointToPlane, bool robust, int step) {
	ceres::Problem problem;
	
	// trans ~Extrinsic to Quaternion (to be optimized)
	Eigen::Quaterniond q;
	Eigen::Vector3d t;
	q = Eigen::Quaterniond(Frame::extrinsic_.linear());
	t = Eigen::Vector3d(Frame::extrinsic_.translation());
	
	cout << "q = " << q.coeffs() << endl;
	cout << "t = " << t << endl;
	
	double x = t[0];
	double y = t[1];
	double z = t[2];
	
	std::cout << "ok ceres" << endl;
	// add edges
	for (size_t frame_id = 0; frame_id < frames.size(); frame_id++) {
		auto &frame = *(frames.at(frame_id));
		auto &correpondances = frame.corr_vec_;
		const Eigen::Isometry3d move_t_ = frame.move_t_;
		
		for (auto corr : correpondances) {
			ceres::CostFunction *cost_function;
			if (step == CALIB_XY_STEP) {
				cost_function = ICPCostFunctions::PointToPlaneErrorGlobal_FixZ::Create(
						frame.pts2_[corr.second_], frame.pts1_[corr.first_],
						frame.nor2_[corr.second_], move_t_);
				ceres::LossFunction *loss = NULL;
				if (robust) loss = new ceres::SoftLOneLoss(frame.weight_);
				
				problem.AddResidualBlock(cost_function, loss, q.coeffs().data(), &x, &y,
				                         &z);
			} else {
				if (pointToPlane) {
					cost_function = ICPCostFunctions::PointToPlaneErrorGlobal::Create(
							frame.pts2_[corr.second_], frame.pts1_[corr.first_],
							frame.nor2_[corr.second_], move_t_);
				} else {
					cost_function = ICPCostFunctions::PointToPointErrorGlobal::Create(
							frame.pts2_[corr.second_], frame.pts1_[corr.first_], move_t_);
				}
				ceres::LossFunction *loss = NULL;
				if (robust) loss = new ceres::SoftLOneLoss(frame.weight_);
				
				problem.AddResidualBlock(cost_function, loss, q.coeffs().data(),
				                         t.data());
			}
		}
	}

#ifdef useLocalParam
	eigen_quaternion::EigenQuaternionParameterization
			*quaternion_parameterization =
			new eigen_quaternion::EigenQuaternionParameterization;
#endif
	
	for (int i = 0; i < frames.size(); ++i) {
#ifdef useLocalParam
		problem.SetParameterization(q.coeffs().data(), quaternion_parameterization);
#endif
	}
	if (step == CALIB_XY_STEP) {
		problem.SetParameterBlockConstant(q.coeffs().data());
		problem.SetParameterBlockConstant(&z);
	} else {
		problem.SetParameterBlockConstant(t.data());
	}
	
	solve(problem, false);
	
	if (step == CALIB_XY_STEP) t << x, y, z;
	
	// update camera poses
	Frame::extrinsic_ = eigenQuaternionToIso(q, t);
	//    Frame::extrinsic_.matrix().block(3,0,1,4) = Eigen::Vector4d(0, 0, 0,
	//    1).transpose();
	
	std::cout << "After once optimize: \n"
	          << Frame::extrinsic_.matrix() << std::endl;
}
}
}
