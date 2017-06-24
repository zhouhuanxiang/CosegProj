#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "mlibutil.h"

#define THRESHOLD 50

int get_integer_part(double x){
	return static_cast<int>(x);
}

void print_infinitesimal_part(double x){
}

void print_scalar_part(double x){
	std::cout << "##" << x << "\n";
}

template<typename SCALAR, int N>
int get_integer_part(const ceres::Jet<SCALAR, N>& x){
	return static_cast<int>(x.a);
}

template<typename SCALAR, int N>
void print_infinitesimal_part(const ceres::Jet<SCALAR, N>& x){
	std::cout << "##" << x.v << "\n";
}

template<typename SCALAR, int N>
void print_scalar_part(const ceres::Jet<SCALAR, N>& x){
	std::cout << "##" << x.a << "\n";
}

template <class T>
T icp_cost(const T* const R, const T* const tr, unsigned short* depth_data_first, unsigned short* depth_data_second, int depthWidth, int depthHeight, 
	int x, int y, double fx, double fy, double cx, double cy){
	// pt1_3d: 3D point in 1st camera coordf
	// pt2_3d:             2nd 
	T pt1_3d[3], pt2_3d[3];
	pt2_3d[2] = (T)*(depth_data_second + x + y * depthWidth);
	pt2_3d[0] = pt2_3d[2] * ((T)x - (T)cx) / (T)fx;
	pt2_3d[1] = pt2_3d[2] * ((T)y - (T)cy) / (T)fy;
	ceres::AngleAxisRotatePoint(R, pt2_3d, pt1_3d);
	pt1_3d[0] += tr[0];
	pt1_3d[1] += tr[1];
	pt1_3d[2] += tr[2];
	// (proj_x, proj_y): 2D point in 1st image
	T proj_x = (pt1_3d[0] / pt1_3d[2]) * (T)fx + (T)cx;
	T proj_y = (pt1_3d[1] / pt1_3d[2]) * (T)fy + (T)cy;
	// cast double or Jet
	int px = get_integer_part(proj_x);
	int py = get_integer_part(proj_y);
	if (px < 0 || px > depthWidth - 2 || py < 0 || py > depthHeight - 2){
		return T(THRESHOLD + 1);
	}
	// notice: reserve the infinitesimal part in wx, wy  
	T wx = proj_x - (double)px;
	T wy = proj_y - (double)py;
	double ll = (double)(unsigned int)*(depth_data_first + px + py * depthWidth);
	double lr = (double)(unsigned int)*(depth_data_first + (px + 1) + py * depthWidth);
	double rl = (double)(unsigned int)*(depth_data_first + px + (py + 1) * depthWidth);
	double rr = (double)(unsigned int)*(depth_data_first + (px + 1) + (py + 1) * depthWidth);
	if (abs(ll - lr) > THRESHOLD / 2 || abs(ll - rl) > THRESHOLD / 2 || abs(ll - rr) > THRESHOLD / 2){
		return T(THRESHOLD + 1);
	}

	T tone = (T)1;
	T depth = (ll * (tone - wx) + lr * wx) * (tone - wy)
		+ (rl * (tone - wx) + rr * wx) * wy;
	T residual = depth - pt1_3d[2];
	//print_scalar_part(residual);
	return residual;
}

class IcpErrorFunc{
public:
	IcpErrorFunc(unsigned short* d1, unsigned short* d2, int dw, int dh, int x, int y, double fx, double fy, double cx, double cy)
		:depth_data_first(d1), depth_data_second(d2), depthWidth(dw), depthHeight(dh), x(x), y(y), fx(fx), fy(fy), cx(cx), cy(cy)
	{}
	template <class T>
	bool operator()(const T* const R, const T* const tr, T* residuals) const {
		residuals[0] = icp_cost(R, tr, depth_data_first, depth_data_second, depthWidth, depthHeight, x, y, fx, fy, cx, cy);
		//print_scalar_part(residuals[0]);
		if (abs(get_integer_part(residuals[0])) > THRESHOLD)
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

//ml::mat4d ParamToTransform1(double* param) {
//	double rotation_R[9];
//	ceres::AngleAxisToRotationMatrix(param, rotation_R);
//	ml::mat4d t_extrinsic;
//	for (int i = 0; i < 3; ++i) {
//		for (int j = 0; j < 3; ++j) {
//			//Note: ceres-rotationMatrix is the transpose of opencv Matrix, or ml Matrix
//			t_extrinsic(i, j) = rotation_R[j * 3 + i];
//		}
//		t_extrinsic(i, 3) = param[3 + i];
//	}
//	t_extrinsic(3, 3) = 1;
//	return t_extrinsic;
//}
//
//void int2str1(const int &int_temp, std::string &string_temp)
//{
//	std::stringstream stream;
//	stream << int_temp;
//	string_temp = stream.str();
//}
//
//void print_ith_frame_2_obj1(ml::SensorData* input, int ith,
//	unsigned short* depth_data, ml::vec3uc* color_data,
//	ml::mat4d& pose, ml::mat4d& intrinsic){
//	std::ofstream ofs;
//	std::string filename;
//	int2str1(ith, filename);
//	filename.append("th.obj");
//	ofs.open(filename);
//
//	for (int y = 0; y < input->m_colorHeight; y++){
//		for (int x = 0; x < input->m_colorWidth; x++){
//			ml::vec3uc rgb = (ml::vec3uc) *(color_data + x + y*input->m_colorWidth);
//			double	      Z = (double)*(depth_data + x + y*input->m_colorWidth);
//			double        X = Z * (x - intrinsic(0, 2)) / intrinsic(0, 0);
//			double        Y = Z * (y - intrinsic(1, 2)) / intrinsic(1, 1);
//			ml::vec4d coord_C(X, Y, Z, 1);
//			ml::vec4d coord_W = pose * coord_C;
//			ofs << "v " << coord_W[0] << " " << coord_W[1] << " " << coord_W[2] << " "
//				<< rgb[0] / 256.0 << " " << rgb[1] / 256.0 << " " << rgb[2] / 256.0 << "\n";
//		}
//	}
//
//	ofs.close();
//}

void test_sens_icp()
{
	// Load RGB-D Data
	ml::SensorData* input = new ml::SensorData("../../data/test.sens");

	int first_frame = 2000;
	int second_frame = first_frame + 20;
	// decompress depth and color data of two frame
	unsigned short* depth_data_first = input->decompressDepthAlloc(first_frame);
	unsigned short* depth_data_second = input->decompressDepthAlloc(second_frame);
	int depthHeight = input->m_depthHeight;
	int depthWidth = input->m_depthWidth;
	// intrinsic matrix
	ml::mat4d intrinsic = input->m_calibrationColor.m_intrinsic;
	// parameters: first 3 are angle axis and the last 3 are translation
	double param[] = { 0, 0, 0, 0, 0, 0 };
	double min_final_cost = 100000000000;

	ceres::Solver::Options options;
	options.num_threads = 8;
	options.linear_solver_type = ceres::SPARSE_SCHUR;
	options.minimizer_progress_to_stdout = true;
	options.max_num_iterations = 50;
	options.update_state_every_iteration = true;
	TerminateWhenSuccessCallback callback;
	options.callbacks.push_back(&callback);

	do{
		int residual_count = 50000;
		int sample_step = 5;
		ceres::Problem problem;
		for (int y = 10; y < depthHeight - 10;){
			for (int x = 10; x < depthWidth - 10;){
				if ((unsigned short)*(depth_data_second + x + y * depthWidth) < 1){
					x += sample_step;
					continue;
				}
				// fx, fy, cx, cy
				double pt_cost = icp_cost<double>(param, param + 3, depth_data_first, depth_data_second, depthWidth, depthHeight, 
					x, y, intrinsic(0, 0), intrinsic(1, 1), intrinsic(0, 2), intrinsic(1, 2));
				if (pt_cost < THRESHOLD){
					auto* costFunc = IcpErrorFunc::Create(depth_data_first, depth_data_second, depthWidth, depthHeight,
						x, y, intrinsic(0, 0), intrinsic(1, 1), intrinsic(0, 2), intrinsic(1, 2));
					problem.AddResidualBlock(costFunc, 0, param, param + 3);
					--residual_count;
				}
				x += sample_step;
				if (residual_count < 0)
					break;
			}
			y += sample_step;
			if (residual_count < 0)
				break;
		}
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);
		std::cout << "problem.NumResiduals: " << problem.NumResiduals() << "\n";
		std::cout << "summary.finalcost:    " << summary.final_cost     << "\n\n";
		if (abs(min_final_cost - summary.final_cost) / min_final_cost < 0.01){
			break;
		}
		min_final_cost = summary.final_cost;
	} while (true);

	// test code:
	for (int i = 0; i < 6; i++){
		std::cout << "##" << param[i] << "\n";
	}
	/*ml::vec3<double> v0 = { 1, 0, 0 };
	ml::vec3<double> v1 = { 0, 1, 0 };
	ml::vec3<double> v2 = { 0, 0, 1 };
	ml::mat4d m4(v0, v1, v2);
	ml::vec3uc* color_data_first = input->decompressColorAlloc(first_frame);
	ml::vec3uc* color_data_second = input->decompressColorAlloc(second_frame);
	print_ith_frame_2_obj1(input, first_frame, depth_data_first, color_data_first, m4, intrinsic);
	print_ith_frame_2_obj1(input, second_frame, depth_data_second, color_data_second, ParamToTransform1(param), intrinsic);
	::free(color_data_first);
	::free(color_data_second);*/

	// free memory
	::free(depth_data_first);
	::free(depth_data_second);
	system("pause");
}
