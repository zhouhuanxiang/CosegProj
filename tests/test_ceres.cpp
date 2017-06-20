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
	double param[4][6];
	double t_param[4][6];
	for (int i = 0; i < 4; i++){
		for (int j = 0; j < 3; j++){
			double angle = (double)rand() / RAND_MAX * ml::PI - ml::PI / 2;
			double tr = (double)rand() / RAND_MAX * 2 - 1;
			t_param[i][j] = param[i][j] = angle;
			t_param[i][j + 3] = param[i][j + 3] = tr;
		}
	}
	//t_param[0][0] = param[0][0] = 0;
	//t_param[0][1] = param[0][1] = 0;
	//t_param[0][2] = param[0][2] = 0;
	// K = intrinsic
	ml::mat3d intrinsic;
	intrinsic.setZero();
	intrinsic(0, 0) = t_fx;
	intrinsic(1, 1) = t_fy;
	intrinsic(0, 2) = t_cx;
	intrinsic(1, 2) = t_cy;
	intrinsic(2, 2) = 1;
	// <R,T> = extrinsic
	ml::mat4d t_extrinsic[4];
	for (int i = 0; i < 4; i++){
		t_extrinsic[i] = ParamToTransform(param[i]);
	}
	// randomly generate x_i
	int num_samples = 40;
	std::vector<ml::vec2d> features[4];
	std::vector<ml::vec3d> objects[4];
	int lower_bound[4] = { 0, 10, 0, 30 };
	int upper_bound[4] = { 20, 40, 40, 40 };
	for (int sample_i = 0; sample_i < num_samples; ++sample_i) {
		auto pt3d = ml::vec3d((double)rand() / RAND_MAX * 100 + 10, (double)rand() / RAND_MAX * 100 + 10, (double)rand() / RAND_MAX * 100 + 10);
		for (int i = 0; i < 4; i++){
			if (sample_i >= lower_bound[i] && sample_i <= upper_bound[i]){
				auto tmp = intrinsic * (t_extrinsic[i] * pt3d);
				ml::vec2d pt(tmp.x / tmp.z, tmp.y / tmp.z);
				//if (pt[0] > 0.0 && pt[0] < 640.0 && pt[1] > 0.0 && pt[1] < 480.0){
				features[i].push_back(pt);
				objects[i].push_back(pt3d);
				//}
			}
		}
	}
	// manually add some noises to the extrinsic matrices
	ml::mat4d extrinsic[4];
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 6; j++){
			param[i][j] += ((double)rand() / RAND_MAX - 0.5) / 50.0;
		}
		extrinsic[i] = ParamToTransform(param[i]);
	}
	// manually add some noises to the intrinsic matrix
	intrinsic(0, 0) += ((double)rand() / RAND_MAX - 0.5) / 10000.0;
	intrinsic(1, 1) += ((double)rand() / RAND_MAX - 0.5) / 10000.0;
	intrinsic(0, 2) += ((double)rand() / RAND_MAX - 0.5) / 10000.0;
	intrinsic(1, 2) += ((double)rand() / RAND_MAX - 0.5) / 10000.0;
	// manually add some noises to the objects(3D points)
	for (int i = 0; i < 4; i++){
		for (int j = 0; j < objects[i].size(); j++){
			objects[i][j].x += ((double)rand() / RAND_MAX - 0.5) / 1000.0;
			objects[i][j].y += ((double)rand() / RAND_MAX - 0.5) / 1000.0;
			objects[i][j].z += ((double)rand() / RAND_MAX - 0.5) / 1000.0;
		}
	}

	for (int i = 0; i < 4; i++){
		printf("error from ground truth parameters after noise added %d: %lf\n", i, EvaluateError(features[i], objects[i], extrinsic[i], intrinsic));
	}
	printf("\nbackward relative error befrore optimization:\n");
	for (int i = 0; i < 4; i++){
		for (int j = 0; j < 6; j++) {
			printf("%lf ", abs((param[i][j] - t_param[i][j]) / t_param[i][j]));
		}
		printf("\n");
	}

	//printf("\n\n============================ Optimization ============================\n");
	// lets start the optimization!!!!!!
	// lets create residual terms
	ceres::Problem problem;
	for (int i = 0; i < 4; i++){
		for (int j = 0; j < objects[i].size(); ++j) {
			auto* costFunc = MyErrorFunc::Create(t_fx, t_fy, t_cx, t_cy, features[i][j], objects[i][j]);
			problem.AddResidualBlock(costFunc, 0, param[i], param[i] + 3);
		}
	}
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::SPARSE_SCHUR;
	//options.minimizer_progress_to_stdout = true;
	options.max_num_iterations = 50;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	std::cout << summary.FullReport() << "\n";
	//printf("============================ Optimization Finished ============================\n\n\n");

	printf("\nbackward relative error after optimization:\n");
	for (int i = 0; i < 4; i++){
		for (int j = 0; j < 6; j++) {
			printf("%lf ", abs((param[i][j] - t_param[i][j]) / t_param[i][j]));
		}
		printf("\n");
	}

	system("pause");
}