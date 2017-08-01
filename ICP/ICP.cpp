#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "ICP.h"

class ProjectICPError {
public:
	ProjectICPError(cv::Vec3f& p, cv::Mat depth, cv::Mat normal, double fx, double fy, double cx, double cy)
		: p(p), depth(depth), normal(normal), fx(fx), fy(fy), cx(cx), cy(cy)
	{}
	template <class T>
	bool operator()(const T* const R, const T* const tr, T* residuals) const {
		T p1[3];
		for (int i = 0; i < 3; ++i)
			p1[i] = (T)p.val[i];
		T p2[3];
		ceres::AngleAxisRotatePoint(R, p1, p2);
		for (int i = 0; i < 3; ++i) {
			p1[i] = p2[i] + tr[i];
		}
		T x = p1[0] / p1[2] * fx + cx;
		T y = p1[1] / p1[2] * fy + cy;
		int px = *(double*)&x;
		int py = *(double*)&y;
		T wx = x - (T)px;
		T wy = y - (T)py;
		int rx = px + 1, ry = py + 1;
		if (px < 0 || rx >= depth.cols || py < 0 || ry >= depth.rows) {
			residuals[0] = (T)0;
			return true;
		}
		double d1 = depth.at<unsigned short>(py, px) * 1e-3f;
		double d2 = depth.at<unsigned short>(py, rx) * 1e-3f;
		double d3 = depth.at<unsigned short>(ry, px) * 1e-3f;
		double d4 = depth.at<unsigned short>(ry, rx) * 1e-3f;
		const cv::Vec3f& n1 = normal.at<cv::Vec3f>(py, px);
		const cv::Vec3f& n2 = normal.at<cv::Vec3f>(py, rx);
		const cv::Vec3f& n3 = normal.at<cv::Vec3f>(ry, px);
		const cv::Vec3f& n4 = normal.at<cv::Vec3f>(ry, rx);
		T w1 = ((T)1. - wx) * ((T)1. - wy);
		T w2 = wx * ((T)1. - wy);
		T w3 = ((T)1. - wx) * wy;
		T w4 = wx * wy;
		T d = (T)0, s = (T)0, n = (T)0;
		if (d1)
			d += (T)d1 * w1, s += w1, n += (T)(n1.val[2]) * w1;
		if (d2)
			d += (T)d2 * w2, s += w2, n += (T)(n2.val[2]) * w2;
		if (d3)
			d += (T)d3 * w3, s += w3, n += (T)(n3.val[2]) * w3;
		if (d4)
			d += (T)d4 * w4, s += w4, n += (T)(n4.val[2]) * w4;
		residuals[0] = (d - p1[2]) * n;
		return true;
	}
	static ceres::CostFunction* Create(cv::Vec3f& p, cv::Mat depth, cv::Mat normal, double fx, double fy, double cx, double cy) {
		// first residual dimension, followed with parameters' dimensions
		return (new ceres::AutoDiffCostFunction<ProjectICPError, 1, 3, 3>(
			new ProjectICPError(p, depth, normal, fx, fy, cx, cy)));
	}
private:
	cv::Mat& depth, normal;
	cv::Vec3f p;
	double fx, fy, cx, cy;
};

#define SAMPLE_SIZE 4
#define DIS_THR 0.5

ml::mat4d Ceres2ML(double* param) {
	double rotation_R[9];
	ceres::AngleAxisToRotationMatrix(param, rotation_R);
	ml::mat4d t_extrinsic;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			//Note: ceres-rotationMatrix is the transpose of opencv Matrix, or ml Matrix
			t_extrinsic(i, j) = rotation_R[j * 3 + i];
		}
		t_extrinsic(i, 3) = param[3 + i];
	}
	t_extrinsic(3, 3) = 1;
	return t_extrinsic;
}
void ML2Ceres(ml::mat4d& t_extrinsic, double* param) {
	double rotation_R[9];
	ceres::AngleAxisToRotationMatrix(param, rotation_R);
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			//Note: ceres-rotationMatrix is the transpose of opencv Matrix, or ml Matrix
			rotation_R[j * 3 + i] = t_extrinsic(i, j);
		}
		param[3 + i] = t_extrinsic(i, 3);
	}
	ceres::RotationMatrixToAngleAxis(rotation_R, param);
}

class TerminateCallback : public ceres::IterationCallback{
public:
	explicit TerminateCallback()
	{}
	~TerminateCallback(){}
	ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary){
		if (summary.step_is_valid){
			return ceres::SOLVER_TERMINATE_SUCCESSFULLY;
		}

		return ceres::SOLVER_CONTINUE;
	}
};

ml::mat4d EstimatePose(cv::Mat prev_points, cv::Mat depth, cv::Mat normal, ml::mat4d& intrinsic, ml::mat4d& velocity)
{
	double param[6];
	ML2Ceres(velocity, param);
	double prev_cost = 1e30;
	int trial = 10;
	for (int out_iter = 0; out_iter < 256; ++out_iter) {
		ceres::Problem problem;
		ceres::Solver::Options options;
		options.linear_solver_type = ceres::SPARSE_SCHUR;
		options.minimizer_progress_to_stdout = false;
		options.max_num_iterations = 50;
		options.num_threads = 8;
		TerminateCallback callback;
		options.callbacks.push_back(&callback);
		int count = 0;
		for (int i = 0; i < depth.rows; i += SAMPLE_SIZE) {
			for (int j = 0; j < depth.cols; j += SAMPLE_SIZE) {
				cv::Vec3f& p = prev_points.at<cv::Vec3f>(i, j);
				if (p.val[0] != MINF) {
					auto error = ProjectICPError(p, depth, normal, intrinsic(0, 0), intrinsic(1, 1), intrinsic(0, 2), intrinsic(1, 2));
					double residual;
					if (error(param, param + 3, &residual) && residual < DIS_THR) {
						count += 1;
						problem.AddResidualBlock(
							ProjectICPError::Create(p, depth, normal, intrinsic(0, 0), intrinsic(1, 1), intrinsic(0, 2), intrinsic(1, 2)),
							0, param, param + 3
						);
					}
				}
			}
		}
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);
		double cost = summary.final_cost;
		if (cost >= prev_cost - 1e-4) {
			trial -= 1;
		}
		else {
			trial = 10;
		}
		prev_cost = cost;
		if (trial == 0 || out_iter == 255) 
		{
			std::cout << "cost " << summary.final_cost << "\n";
			break;
		}
	}
	std::cout << param[3] << " " << param[4] << " " << param[5] << "\n";
	return Ceres2ML(param);
}