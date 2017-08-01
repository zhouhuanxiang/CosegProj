#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <omp.h>

#include "ColorICP.h"
#include "ICP/ICP.h"

class ColorICPError {
public:
  ColorICPError(cv::Mat& color_img, ml::vec4f& color, ml::vec3f& point, double fx, double fy, double cx, double cy)
	: color_img(color_img), color(color), point(point), fx(fx), fy(fy), cx(cx), cy(cy)
  {}

  template <class T>
  bool operator()(const T* const R, const T* const tr, T* residuals) const {
	T pt_w[3];
	for (int i = 0; i < 3; ++i){
	  pt_w[i] = (T)point[i];
	}
	T pt_c[3];
	ceres::AngleAxisRotatePoint(R, pt_w, pt_c);
	for (int i = 0; i < 3; ++i){
	  pt_c[i] = pt_w[i] + tr[i];
	}
	T x = pt_c[0] / pt_c[2] * fx + cx;
	T y = pt_c[1] / pt_c[2] * fy + cy;
	int px = *(double*)&x;
	int py = *(double*)&y;
	T wx = x - (T)px;
	T wy = y - (T)py;
	int rx = px + 1;
	int ry = py + 1;
	cv::Vec3b c1 = color_img.at<cv::Vec3b>(py, px);
	cv::Vec3b c2 = color_img.at<cv::Vec3b>(py, rx);
	cv::Vec3b c3 = color_img.at<cv::Vec3b>(ry, px);
	cv::Vec3b c4 = color_img.at<cv::Vec3b>(ry, rx);
	T w1 = ((T)1. - wx) * ((T)1. - wy);
	T w2 = wx * ((T)1. - wy);
	T w3 = ((T)1. - wx) * wy;
	T w4 = wx * wy;
	double c10 = c1[0], c20 = c2[0], c30 = c3[0], c40 = c4[0];
	double c11 = c1[1], c21 = c2[1], c31 = c3[1], c41 = c4[1];
	double c12 = c1[2], c22 = c2[2], c32 = c3[2], c42 = c4[2];
	T average_color[3];
	average_color[0] = c10 * w1 + c20 * w2 + c30 * w3 + c40 * w4;
	average_color[1] = c11 * w1 + c21 * w2 + c31 * w3 + c41 * w4;
	average_color[2] = c12 * w1 + c22 * w2 + c32 * w3 + c42 * w4;
	for (int i = 0; i < 3; i++){
	  if (abs(*(double*)&average_color[0] - color[2]) < 10
		&& abs(*(double*)&average_color[1] - color[1]) < 10
		&& abs(*(double*)&average_color[2] - color[0]) < 10){
		residuals[i] = average_color[i] - (double)color[2 - i];
	  }
	  else
		residuals[i] = (T)0;
	}
	/*T b = (double)c1[0] * w1 + (double)c2[0] * w2 + (double)c3[0] * w3 + (double)c4[0] * w4;
	T g = (double)c1[1] * w1 + (double)c2[1] * w2 + (double)c3[1] * w3 + (double)c4[1] * w4;
	T r = (double)c1[2] * w1 + (double)c2[2] * w2 + (double)c3[2] * w3 + (double)c4[2] * w4;
	residuals[0] = b - (double)color[2];
	residuals[1] = g - (double)color[1];
	residuals[2] = r - (double)color[0];*/
	return true;
  }

  static ceres::CostFunction* Create(cv::Mat& color_img, ml::vec4f& color, ml::vec3f& point, double fx, double fy, double cx, double cy){
	// first residual dimension, followed with parameters' dimensions
	return (new ceres::AutoDiffCostFunction<ColorICPError, 3, 3, 3>(
	  new ColorICPError(color_img, color, point, fx, fy, cx, cy)));
  }
private:
  cv::Mat& color_img;
  ml::vec4f color; // (gbr [0,255],[0,255],[0,255])
  ml::vec3f point;
  int colorWidth;
  double fx, fy, cx, cy;
};

void OptimizePose(ml::SensorData* input, ml::MeshDataf& mesh, KeyFrame& kf)
{
  // intrinsic maxtrix
  ml::mat4d intrinsic = input->m_calibrationColor.m_intrinsic;

  for (int f = 0; f < kf.frames_.size(); f++)
  {
	std::cout << "== frame " << f << "==\n";
	//	world2camera pose
	ml::mat4d pose = input->m_frames[kf.frames_[f]].getCameraToWorld().getInverse();
	double param[6];
	ML2Ceres(pose, param);
	// basic problem parameters
	ceres::Problem problem;
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::SPARSE_SCHUR;
	options.minimizer_progress_to_stdout = true;
	options.max_num_iterations = 50;
	options.num_threads = 8;
	// color data
	ml::vec3uc* color_data = input->decompressColorAlloc(kf.frames_[f]);
	cv::Mat color_img(input->m_colorHeight, input->m_colorWidth, CV_8UC3, color_data);
	// free color data
	::free(color_data);

	std::vector<int>& vts = kf.vertices_[f];
	for (int v = 0; v < vts.size(); v++)
	{
	  problem.AddResidualBlock(
		ColorICPError::Create(color_img, mesh.m_Colors[vts[v]], mesh.m_Vertices[vts[v]], intrinsic(0, 0), intrinsic(1, 1), intrinsic(0, 2), intrinsic(1, 2)),
		0, param, param + 3
		);
	}
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	ml::mat4d new_pose = Ceres2ML(param);
	input->m_frames[kf.frames_[f]].setCameraToWorld(new_pose);
  }
}
