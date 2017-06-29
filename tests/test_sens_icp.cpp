#include <queue>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "mlibutil.h"

#define DISTANCE_THRESHOLD 50
#define NORMAL_THRESHOLD   0.9

int get_integer_part(double x){
	return static_cast<int>(x);
}

void print_infinitesimal_part(double x){
	//std::cout << "#inf#" << "\n";
}

void print_scalar_part(double x){
	std::cout << "#sca#" << x << "\n";
}

template<typename SCALAR, int N>
int get_integer_part(const ceres::Jet<SCALAR, N>& x){
	return static_cast<int>(x.a);
}

template<typename SCALAR, int N>
void print_infinitesimal_part(const ceres::Jet<SCALAR, N>& x){
	std::cout << "#inf#" << x.v << "\n";
}

template<typename SCALAR, int N>
void print_scalar_part(const ceres::Jet<SCALAR, N>& x){
	std::cout << "#sca#" << x.a << "\n";
}

template <class T>
void get_world_coordinate_from_xy(T* pt2w, unsigned short* depth_data_second, int x, int y, int depthWidth, const T* const R, const T* const tr, double fx, double fy, double cx, double cy){
	T pt2c[3];
	pt2c[2] = (T)*(depth_data_second + x + y * depthWidth);
	pt2c[0] = pt2c[2] * ((T)x - (T)cx) / (T)fx;
	pt2c[1] = pt2c[2] * ((T)y - (T)cy) / (T)fy;
	ceres::AngleAxisRotatePoint(R, pt2c, pt2w);
	pt2w[0] += tr[0];
	pt2w[1] += tr[1];
	pt2w[2] += tr[2];
}

template <class T>
bool get_world_coordinate_from_xy_bilateral_sampling(T* pt1w, unsigned short* depth_data_first, T x1, T y1, int depthWidth, int depthHeight, const T* const R, const T* const tr, double fx, double fy, double cx, double cy){
	// cast double or Jet
	int px = get_integer_part(x1);
	int py = get_integer_part(y1);
	if (px < 0 || px >= depthWidth - 2 || py < 0 || py >= depthHeight - 2){
		return false;
	}
	double ll = (double)(unsigned int)*(depth_data_first + px + py * depthWidth);
	double lr = (double)(unsigned int)*(depth_data_first + (px + 1) + py * depthWidth);
	double rl = (double)(unsigned int)*(depth_data_first + px + (py + 1) * depthWidth);
	double rr = (double)(unsigned int)*(depth_data_first + (px + 1) + (py + 1) * depthWidth);
	// notice: reserve the infinitesimal part in wx, wy  
	T tone = (T)1;
	T wx = x1 - (T)px;
	T wy = y1 - (T)py;
	pt1w[2] = (ll * (tone - wx) + lr * wx) * (tone - wy)
		+ (rl * (tone - wx) + rr * wx) * wy;
	pt1w[0] = pt1w[2] * (x1 - (T)cx) / (T)fx;
	pt1w[1] = pt1w[2] * (y1 - (T)cy) / (T)fy;

	if (abs(ll - lr) > DISTANCE_THRESHOLD / 2 || abs(ll - rl) > DISTANCE_THRESHOLD / 2 || abs(ll - rr) > DISTANCE_THRESHOLD / 2){
		return false;
	}
	else{
		return true;
	}
}

template <class T>
void get_normal_vector(T* pt, T* pt_n1, T* pt_n2, T* n){
	T diff1[3], diff2[3];
	diff1[0] = pt_n1[0] - pt[0]; diff1[1] = pt_n1[1] - pt[1]; diff1[2] = pt_n1[2] - pt[2];
	diff2[0] = pt_n2[0] - pt[0]; diff2[1] = pt_n2[1] - pt[1]; diff2[2] = pt_n2[2] - pt[2];

	n[0] = diff1[1] * diff2[2] - diff1[2] * diff2[1];
	n[1] = diff1[2] * diff2[0] - diff1[0] * diff2[2];
	n[2] = diff1[0] * diff2[1] - diff1[1] * diff2[0];

	T len = ceres::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
	n[0] = n[0] / len;
	n[1] = n[1] / len;
	n[2] = n[2] / len;
}

template <class T>
T icp_cost(const T* const R, const T* const tr, unsigned short* depth_data_first, unsigned short* depth_data_second, int depthWidth, int depthHeight, int x, int y, double fx, double fy, double cx, double cy, T* residuals = NULL){
	// pt2w:    3D point of (x, y) in world coord(a.k.a. first frame coord)
	// (x, y):  2D point in 2nd frame image
	T pt2w[3];
	get_world_coordinate_from_xy(pt2w, depth_data_second, x, y, depthWidth, R, tr, fx, fy, cx, cy);
	if (get_integer_part(pt2w[2]) < 1){
		return T(DISTANCE_THRESHOLD + 1);
	}
	// (x1, y1): 2D point in 1st frame image
	T x1 = (pt2w[0] / pt2w[2]) * (T)fx + (T)cx;
	T y1 = (pt2w[1] / pt2w[2]) * (T)fy + (T)cy;
	// pt1w: corresponding point of (x1, y1)
	T pt1w[3];
	if (!get_world_coordinate_from_xy_bilateral_sampling(pt1w, depth_data_first, x1, y1, depthWidth, depthHeight, R, tr, fx, fy, cx, cy)){
		return T(DISTANCE_THRESHOLD + 1);
	}

	// distance error
	T distance_error = ceres::sqrt((pt1w[0] - pt2w[0]) * (pt1w[0] - pt2w[0]) 
		+ (pt1w[1] - pt2w[1]) * (pt1w[1] - pt2w[1]) 
		+ (pt1w[2] - pt2w[2]) * (pt1w[2] - pt2w[2]));
	if (get_integer_part(distance_error) > DISTANCE_THRESHOLD){
		return T(DISTANCE_THRESHOLD + 1);
	}
	// neighboring points of pt1w, pt2w
	T pt1w_n1[3], pt1w_n2[3], pt2w_n1[3], pt2w_n2[3];
	get_world_coordinate_from_xy(pt2w_n1, depth_data_second, x + 1, y, depthWidth, R, tr, fx, fy, cx, cy);
	get_world_coordinate_from_xy(pt2w_n2, depth_data_second, x, y + 1, depthWidth, R, tr, fx, fy, cx, cy);
	get_world_coordinate_from_xy_bilateral_sampling(pt1w_n1, depth_data_first, x1 + T(1.0), y1, depthWidth, depthHeight, R, tr, fx, fy, cx, cy);
	get_world_coordinate_from_xy_bilateral_sampling(pt1w_n2, depth_data_first, x1, y1 + T(1.0), depthWidth, depthHeight, R, tr, fx, fy, cx, cy);
	// n1: normal vector of pt1w
	// n2					pt2w
	T n1[3], n2[3];
	get_normal_vector(pt1w, pt1w_n1, pt1w_n2, n1);
	get_normal_vector(pt2w, pt2w_n1, pt2w_n2, n2);
	// normal error
	T normal_error = n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2];
	if (normal_error < NORMAL_THRESHOLD){
		return T(DISTANCE_THRESHOLD + 1);
	}

	T residual = distance_error;
	if (residuals){
		//residuals[0] = (pt1w[0] - pt2w[0]);
		//residuals[1] = (pt1w[1] - pt2w[1]);
		//residuals[2] = (pt1w[2] - pt2w[2]);
		residuals[0] = (pt2w[0] - pt1w[0]) * n1[0] + (pt2w[1] - pt1w[1]) * n1[1] + (pt2w[2] - pt1w[2]) * n1[2];
	}
	return residual;
}

class IcpErrorFunc{
public:
	IcpErrorFunc(unsigned short* d1, unsigned short* d2, int dw, int dh, int x, int y, double fx, double fy, double cx, double cy)
		:depth_data_first(d1), depth_data_second(d2), depthWidth(dw), depthHeight(dh), x(x), y(y), fx(fx), fy(fy), cx(cx), cy(cy)
	{}
	template <class T>
	bool operator()(const T* const R, const T* const tr, T* residuals) const {
		T residual = icp_cost(R, tr, depth_data_first, depth_data_second, depthWidth, depthHeight, x, y, fx, fy, cx, cy, residuals);
		//print_scalar_part(residuals[0]);
		if (abs(get_integer_part(residual)) > DISTANCE_THRESHOLD)
			//residuals[0] = residuals[1] = residuals[2] = T(0);
			residuals[0] = T(0);
		return true;
	}
	static ceres::CostFunction* Create(unsigned short* d1, unsigned short* d2, int dw, int dh, int x, int y, double fx, double fy, double cx, double cy) {
		// first residual dimension, followed with parameters' dimensions
		return (new ceres::AutoDiffCostFunction<IcpErrorFunc, 1, 3, 3>(
			new IcpErrorFunc(d1, d2, dw, dh, x, y, fx, fy, cx, cy)));
	}
private:
	unsigned short* depth_data_first;
	unsigned short* depth_data_second;
	int depthWidth, depthHeight;
	int x, y;
	double fx, fy, cx, cy;
};

class TerminateWhenSuccessCallback : public ceres::IterationCallback{
public:
	explicit TerminateWhenSuccessCallback()
	{}
	~TerminateWhenSuccessCallback(){}
	ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary){
		if (summary.step_is_valid){
			return ceres::SOLVER_TERMINATE_SUCCESSFULLY;
		}

		return ceres::SOLVER_CONTINUE;
	}
};

ml::mat4d ParamToTransform1(double* param) {
	double rotation_R[9];
	ceres::AngleAxisToRotationMatrix(param, rotation_R);
	ml::mat4d t_extrinsic;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			//Note: ceres-rotationMatrix is the transpose of opencv Matrix, or ml Matrix
			t_extrinsic(i, j) = rotation_R[j * 3 + i];
		}
		t_extrinsic(i, 3) = param[3 + i];
		t_extrinsic(3, i) = 0;
	}
	t_extrinsic(3, 3) = 1;
	return t_extrinsic;
}

void int2str1(const int &int_temp, std::string &string_temp)
{
	std::stringstream stream;
	stream << int_temp;
	string_temp = stream.str();
}

void print_ith_frame_2_obj1(ml::SensorData* input, int ith,
	unsigned short* depth_data, ml::vec3uc* color_data,
	ml::mat4d& pose, ml::mat4d& intrinsic){
	std::ofstream ofs;
	std::string filename;
	int2str1(ith, filename);
	filename.append("th.obj");
	ofs.open(filename);

	int step = 8;
	for (int y = 0; y < input->m_colorHeight;){
		for (int x = 0; x < input->m_colorWidth;){
			ml::vec3uc rgb = (ml::vec3uc) *(color_data + x + y*input->m_colorWidth);
			double	      Z = (double)*(depth_data + x + y*input->m_colorWidth);
			double        X = Z * (x - intrinsic(0, 2)) / intrinsic(0, 0);
			double        Y = Z * (y - intrinsic(1, 2)) / intrinsic(1, 1);
			ml::vec4d coord_C(X, Y, Z, 1);
			ml::vec4d coord_W = pose * coord_C;
			ofs << "v " << coord_W[0] << " " << coord_W[1] << " " << coord_W[2] << " "
				<< rgb[0] / 256.0 << " " << rgb[1] / 256.0 << " " << rgb[2] / 256.0 << "\n";
			x += step;
		}
		y += step;
	}

	ofs.close();
}

void test_sens_icp()
{
	// Load RGB-D Data
	ml::SensorData* input = new ml::SensorData("../../data/test.sens");

	// accumulate extrinsic matrix
	ml::mat4d obsolute_extrinsic;
	obsolute_extrinsic.setIdentity();
	obsolute_extrinsic(3, 3) = 1;
	// intrinsic matrix
	ml::mat4d intrinsic;
	intrinsic = input->m_calibrationColor.m_intrinsic;

	int second_frame = 30;
	unsigned short* depth_data_second = input->decompressDepthAlloc(second_frame);

	for (int outer_iter = 0; outer_iter < 50; outer_iter++){
		int first_frame = second_frame;
		++second_frame;
		unsigned short* depth_data_first = depth_data_second;
		depth_data_second = input->decompressDepthAlloc(second_frame);
		int depthHeight = input->m_depthHeight;
		int depthWidth = input->m_depthWidth;
		// parameters: first 3 are angle axis and the last 3 are translation
		double param[] = { 0, 0, 0, 0, 0, 0 };

		//double rotation_R[9];
		//ml::mat4d extrinsic1 = input->m_frames[first_frame].getCameraToWorld();
		//ml::mat4d extrinsic2 = input->m_frames[second_frame].getCameraToWorld();
		//ml::mat4d extrinsic21 = extrinsic1.getInverse() * extrinsic2;
		//std::cout << extrinsic21;
		//for (int i = 0; i < 3; ++i) {
		//	for (int j = 0; j < 3; ++j) {
		//		//Note: ceres-rotationMatrix is the transpose of opencv Matrix, or ml Matrix
		//		//t_extrinsic(i, j) = rotation_R[j * 3 + i];
		//		rotation_R[j * 3 + i] = extrinsic21(i, j);
		//		}
		//	//t_extrinsic(i, 3) = param[3 + i];
		//	param[3 + i] = extrinsic21(i, 3);
		//}
		//ceres::RotationMatrixToAngleAxis(rotation_R, param);
		//for (int i = 0; i < 6; i++){
		//	std::cout << "##" << param[i] << "\n";
		//}

		ceres::Solver::Options options;
		options.num_threads = 8;
		options.linear_solver_type = ceres::SPARSE_SCHUR;
		options.minimizer_progress_to_stdout = true;
		options.max_num_iterations = 50;
		options.update_state_every_iteration = true;
		TerminateWhenSuccessCallback callback;
		options.callbacks.push_back(&callback);

		double min_final_cost = 100000000000;
		for (int max_iteration = 0; max_iteration < 20; max_iteration++){
			std::cout << "====== new round ======\n";
			int sample_step = 5;
			ceres::Problem problem;
			for (int y = 0; y < depthHeight - 2;){
				for (int x = 0; x < depthWidth - 2;){
					if ((unsigned short)*(depth_data_second + x + y * depthWidth) < 1) {
						x += sample_step;
						continue;
					}
					// fx, fy, cx, cy
					double pt_cost = icp_cost<double>(param, param + 3, depth_data_first, depth_data_second, depthWidth, depthHeight,
						x, y, intrinsic(0, 0), intrinsic(1, 1), intrinsic(0, 2), intrinsic(1, 2));
					if (abs(pt_cost) < DISTANCE_THRESHOLD){
						auto* costFunc = IcpErrorFunc::Create(depth_data_first, depth_data_second, depthWidth, depthHeight,
							x, y, intrinsic(0, 0), intrinsic(1, 1), intrinsic(0, 2), intrinsic(1, 2));				
						problem.AddResidualBlock(costFunc, 0, param, param + 3);
					}
					x += sample_step;
				}
				y += sample_step;
			}

			ceres::Solver::Summary summary;
			ceres::Solve(options, &problem, &summary);
			std::cout << "problem.NumResiduals: " << problem.NumResiduals() << "\n";
			std::cout << "summary.finalcost:    " << summary.final_cost << "\n\n";
			for (int i = 0; i < 6; i++)
				std::cout << "##" << param[i] << "\n";
			if (abs(min_final_cost - summary.final_cost) < 0.001 * min_final_cost)
				break;
			min_final_cost = summary.final_cost;
		};

		// test code:
		for (int i = 0; i < 6; i++){
			std::cout << "##" << param[i] << "\n";
		}
		ml::vec3uc* color_data_second = input->decompressColorAlloc(second_frame);
		obsolute_extrinsic = obsolute_extrinsic * ParamToTransform1(param);
		std::cout << obsolute_extrinsic;
		//if (outer_iter % 5 == 0){
			print_ith_frame_2_obj1(input, second_frame, depth_data_second, color_data_second, obsolute_extrinsic, intrinsic);
		//}
		::free(color_data_second);

		// free memory
		::free(depth_data_first);
	}
	//free memory
	::free(depth_data_second);
	system("pause");
}
