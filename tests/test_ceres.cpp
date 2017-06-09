#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <opencv2/opencv.hpp>
#include "../mlibutil.h"

// lets start with the optimization!
// first define a optimization helper class
// Always use double!
class MyErrorFunc
{
public:
	MyErrorFunc(double fx, double fy, double cx, double cy, ml::vec2d& feat, ml::vec3d& obj)
		: fx(fx), fy(fy), cx(cx), cy(cy), feat(feat), obj(obj)
	{}

	// Only need to define error (no square): error = K * (R * obj + T) - feat
	template <class T>
	bool operator()(const T* const R, const T* const tr, T* residuals) const {
		T Xw[3], Xc[3];
		Xw[0] = (T)obj.x, Xw[1] = (T)obj.y, Xw[2] = (T)obj.z;
		ceres::AngleAxisRotatePoint(R, Xw, Xc);
		Xc[0] += tr[0];
		Xc[1] += tr[1];
		Xc[2] += tr[2];
		T proj_x = (Xc[0] / Xc[2]) * (T)fx + (T)cx;
		T proj_y = (Xc[1] / Xc[2]) * (T)fy + (T)cy;
		residuals[0] = proj_x - (T)feat.x;
		residuals[1] = proj_y - (T)feat.y;
		return true;
	}
	static ceres::CostFunction* Create(double fx, double fy, double cx, double cy, ml::vec2d& feat, ml::vec3d& obj) {
		// first residual dimension, followed with parameters' dimensions
		return (new ceres::AutoDiffCostFunction<MyErrorFunc, 2, 3, 3>(
			new MyErrorFunc(fx, fy, cx, cy, feat, obj)));
	}
	double fx, fy, cx, cy;
	ml::vec2d feat;
	ml::vec3d obj;
};

double EvaluateError(std::vector<ml::vec2d>& features, std::vector<ml::vec3d>& objects, ml::mat4d& extrinsic, ml::mat3d& intrinsic)
{
	double total = 0;
	for (int i = 0; i < features.size(); ++i) {
		auto feat = intrinsic * (extrinsic * objects[i]);
		double err_x = feat.x / feat.z - features[i].x;
		double err_y = feat.y / feat.z - features[i].y;
		total += err_x * err_x + err_y * err_y;
	}
	return 0.5 * total;
}

ml::mat4d ParamToTransform(double* param) {
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

void test_ceres()
{
	// If we have a given intrinsic matrix K, a set of 2d image points x_i and corresponding 3d points X_i
	// we want to know R and T so that 0.5 * sum_i ||K(RX_i + T) - x_i||^2 is minimal
	
	// generate some data, t_* means ground truth
	float t_fx = 540, t_fy = 540, t_cx = 320, t_cy = 240;
	// parameters: first 3 are angle axis and the last 3 are translation
	double param[] = { 1.0, 0.3, -0.8, 0.2, -0.1, 0.7 };
	double t_param[6];
	memcpy(t_param, param, sizeof(double) * 6);
	// K = intrinsic
	ml::mat3d intrinsic;
	intrinsic.setZero();
	intrinsic(0, 0) = t_fx;
	intrinsic(1, 1) = t_fy;
	intrinsic(0, 2) = t_cx;
	intrinsic(1, 2) = t_cy;
	intrinsic(2, 2) = 1;
	// <R,T> = extrinsic
	auto t_extrinsic = ParamToTransform(param);
	// randomly generate x_i
	int num_samples = 20;
	std::vector<ml::vec2d> features;
	std::vector<ml::vec3d> objects;
	for (int i = 0; i < num_samples; ++i) {
		auto pt = ml::vec2d((double)rand() / RAND_MAX * 640.0, (double)rand() / RAND_MAX * 480.0);
		float depth = ((double)rand() / RAND_MAX) * 3.0 + 1.0;
		auto pt3d = ml::vec3d(depth * (pt.x - t_cx) / t_fx, depth * (pt.y - t_cy) / t_fy, depth);
		pt3d = t_extrinsic.getInverse() * pt3d;
		features.push_back(pt);
		objects.push_back(pt3d);
	}
	printf("error from ground truth parameters: %lf\n", EvaluateError(features, objects, t_extrinsic, intrinsic));

	// manually add some noises to the data
	for (int i = 0; i < 6; ++i) {
		param[i] += ((double)rand() / RAND_MAX - 0.5) / 50.0;
	}
	auto extrinsic = ParamToTransform(param);
	printf("forward error from initial parameters: %lf\n", EvaluateError(features, objects, extrinsic, intrinsic));
	printf("backward relative error:\n");
	for (int i = 0; i < 6; ++i) {
		printf("%lf ", abs((param[i] - t_param[i]) / t_param[i]));
	}
	printf("\n");
	printf("\n\n============================ Optimization ============================\n");
	// lets start the optimization!!!!!!
	// lets create residual terms
	ceres::Problem problem;
	for (int i = 0; i < num_samples; ++i) {
		auto* costFunc = MyErrorFunc::Create(t_fx, t_fy, t_cx, t_cy, features[i], objects[i]);
		problem.AddResidualBlock(costFunc, 0, param, param + 3);
	}
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::SPARSE_SCHUR;
	options.minimizer_progress_to_stdout = true;
	options.max_num_iterations = 50;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	std::cout << summary.FullReport() << "\n";
	printf("============================ Optimization Finished ============================\n\n\n");

	printf("For debug, you can verify your self-computed initial error is consistent the ceres-solver's initial value:\n");
	printf("%lf = %lf\n", summary.initial_cost, EvaluateError(features, objects, extrinsic, intrinsic));
	// compare the estimated value with the ground truth
	printf("backward relative error:\n");
	for (int i = 0; i < 6; ++i) {
		printf("%lf ", abs((param[i] - t_param[i]) / t_param[i]));
	}
	printf("\n");
}