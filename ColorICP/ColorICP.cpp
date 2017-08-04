#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <omp.h>

#include "ColorICP.h"
#include "ICP/ICP.h"

#define USE_DEPTH

class ColorICPError {
public:
  ColorICPError(cv::Mat& grey_img, cv::Mat& depth_img, double pt_color, ml::vec3f& point, double fx, double fy, double cx, double cy, int width, int height)
	: grey_img(grey_img), depth_img(depth_img), pt_color(pt_color), point(point), fx(fx), fy(fy), cx(cx), cy(cy), width(width), height(height)
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
	  pt_c[i] += tr[i];
	}
	T x = pt_c[0] / pt_c[2] * fx + cx;
	T y = pt_c[1] / pt_c[2] * fy + cy;
	int px = *(double*)&x;
	int py = *(double*)&y;
	// test boundary
	if (px < 9 || px >= width - 10 || py < 9 || py >= height - 10){
	  residuals[0] = (T)0;
	  return true;
	}

	T wx = x - (T)px;
	T wy = y - (T)py;
	T w[4];
	w[0] = ((T)1. - wx) * ((T)1. - wy);
	w[1] = wx * ((T)1. - wy);
	w[2] = ((T)1. - wx) * wy;
	w[3] = wx * wy;
	w[0] = w[0] * w[0];
	w[1] = w[1] * w[1];
	w[2] = w[2] * w[2];
	w[3] = w[3] * w[3];
	T sum = w[0] + w[1] + w[2] + w[3];
	w[0] /= sum; w[1] /= sum; w[2] /= sum; w[3] /= sum;
	int rx = px + 1;
	int ry = py + 1;
	double c[4];
	c[0] = (double)grey_img.at<uchar>(py, px);
	c[1] = (double)grey_img.at<uchar>(py, rx);
	c[2] = (double)grey_img.at<uchar>(ry, px);
	c[3] = (double)grey_img.at<uchar>(ry, rx);
	T color = c[0] * w[0] + c[1] * w[1] + c[2] * w[2] + c[3] * w[3];
#ifdef USE_DEPTH
	double d[4];
	d[0] = (double)depth_img.at<ushort>(py, px);
	d[1] = (double)depth_img.at<ushort>(py, rx);
	d[2] = (double)depth_img.at<ushort>(ry, px);
	d[3] = (double)depth_img.at<ushort>(ry, rx);
	T depth = (d[0] * w[0] + d[1] * w[1] + d[2] * w[2] + d[3] * w[3]) * 0.001;
	T depth_error = depth / pt_c[2] - 1.0;
	// test depth consistency
	if (abs(*(double*)&(depth_error)) < 0.01)
	  residuals[0] = (color - pt_color) /* * (depth - pt_c[2] * 1000.0) */;
	//std::cout << residuals[0] << "\n";
	//std::cout << *(double*)&color << " " << pt_color << " " << *(double*)residuals << "\n";
#else
	residuals[0] = color - pt_color;
#endif
	if (abs(*(double*)residuals) > 20)
	  residuals[0] = (T)0;
	return true;
  }

  static ceres::CostFunction* Create(cv::Mat& grey_img, cv::Mat& depth_img, double pt_color, ml::vec3f& point, double fx, double fy, double cx, double cy, int width, int height){
	// first residual dimension, followed with parameters' dimensions
	return (new ceres::AutoDiffCostFunction<ColorICPError, 1, 3, 3>(
	  new ColorICPError(grey_img, depth_img, pt_color, point, fx, fy, cx, cy, width, height)));
  }
private:
  cv::Mat& grey_img;
  cv::Mat& depth_img;
  double pt_color; // (grey color)
  ml::vec3f point;
  int colorWidth;
  double fx, fy, cx, cy;
  int width, height;
};

void OptimizePose(ml::SensorData* input, ml::MeshDataf& mesh, KeyFrame& kf)
{
  // intrinsic maxtrix
  ml::mat4d intrinsic = input->m_calibrationColor.m_intrinsic;

#pragma omp parallel for
  for (int f = 0; f < kf.frames_.size(); f++)
  {
	//	world2camera pose
	ml::mat4d pose = input->m_frames[kf.frames_[f]].getCameraToWorld().getInverse();
	double param[6];
	ML2Ceres(pose, param);
	// basic problem parameters
	ceres::Problem problem;
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::SPARSE_SCHUR;
	options.minimizer_progress_to_stdout = false;
	options.max_num_iterations = 30;
	options.num_threads = 8;

	std::vector<int>& vts = kf.vertices_[f];
	for (int v = 0; v < vts.size();)
	{
	  //double color = 0.299 * mesh.m_Colors[vts[v]][0] + 0.587 * mesh.m_Colors[vts[v]][1] + 0.144 * mesh.m_Colors[vts[v]][2];
	  cv::Mat m(1, 1, CV_8UC3, cv::Scalar(mesh.m_Colors[vts[v]][2] * 256.0, mesh.m_Colors[vts[v]][1] * 256.0, mesh.m_Colors[vts[v]][0] * 256.0));
	  cv::Mat gm;
	  cv::cvtColor(m, gm, CV_BGR2GRAY);

	  problem.AddResidualBlock(
		ColorICPError::Create(kf.grey_images_[f], kf.depth_images_[f], (double)gm.at<unsigned char>(0, 0), mesh.m_Vertices[vts[v]], intrinsic(0, 0), intrinsic(1, 1), intrinsic(0, 2), intrinsic(1, 2), kf.grey_images_[f].cols, kf.grey_images_[f].rows),
		0, param, param + 3
		);

	  v += 10;
	}
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	ml::mat4d new_pose = Ceres2ML(param);
	input->m_frames[kf.frames_[f]].setCameraToWorld(new_pose.getInverse());

	if (f == 0){
	  std::cout << f << "th frame, final cost = " << summary.final_cost << "\n";
	  std::cout << "	       blocks = " << problem.NumResidualBlocks() << "\n";
	}
  }
}
