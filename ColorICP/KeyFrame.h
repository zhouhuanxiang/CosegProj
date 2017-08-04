#ifndef CICP_KEYFRAME
#define CICP_KEYFRAME

#include "mlibutil.h"

#include <vector>
#include <algorithm>
#include <omp.h>

#include "../ICP/ICP.h"

class KeyFrame
{
public:
  KeyFrame() {};
  ~KeyFrame() {}

  void Init(ml::SensorData* input, ml::MeshDataf& mesh)
  {
	height = input->m_colorHeight;
	width = input->m_colorWidth;
	// select 40 keyframes
	int kf_size = 40;
	frames_.resize(kf_size);
	int step = input->m_frames.size() / kf_size;
	for (int i = 0; i < kf_size; i++){
	  frames_[i] = step * i;
	}
	//
	vertices_.resize(kf_size);
	for (int i = 0; i < kf_size; i++)
	  vertices_[i].reserve(mesh.m_Vertices.size());
	// store images of key frame, so we don't need to read from SensorData everytime
	color_images_.resize(kf_size);
	depth_images_.resize(kf_size);
	grey_images_.resize(kf_size);
	for (int f = 0; f < kf_size; f++){
	  ml::vec3uc* color_data = input->decompressColorAlloc(frames_[f]);
	  unsigned short* depth_data = input->decompressDepthAlloc(frames_[f]);
	  cv::Mat color_img(input->m_colorHeight, input->m_colorWidth, CV_8UC3, color_data);
	  cv::Mat gauss_img;
	  // gauss filter
	  cv::GaussianBlur(color_img, gauss_img, cv::Size(5, 5), 0, 0);
	  cv::Mat depth_img(input->m_depthHeight, input->m_depthWidth, CV_16UC1, depth_data);
	  cv::Mat grey_img;
	  // simplify to grey image when calculate residuals
	  cv::cvtColor(gauss_img, grey_img, CV_BGR2GRAY);
	  gauss_img.copyTo(color_images_[f]);
	  depth_img.copyTo(depth_images_[f]);
	  grey_img.copyTo(grey_images_[f]);
	  // free data
	  ::free(color_data);
	  ::free(depth_data);
	}
  }

  void VisibleTest(ml::SensorData* input, ml::MeshDataf& mesh)
  {
	for (int f = 0; f < frames_.size(); f++)
	{
	  // world2camera pose
	  ml::mat4d pose = input->m_frames[frames_[f]].getCameraToWorld().getInverse();
	  // inntrinsic matrix
	  ml::mat4d intrinsic = input->m_calibrationColor.m_intrinsic;

//#pragma omp parallel for
	  for (int v = 0; v < mesh.m_Vertices.size(); v++)
	  {
		ml::vec3<double> point = pose * mesh.m_Vertices[v];
		double x = (point.x / point.z) * intrinsic(0, 0) + intrinsic(0, 2);
		double y = (point.y / point.z) * intrinsic(1, 1) + intrinsic(1, 2);
		if (x >= 9 && x < width - 10 && y >= 9 && y < height - 10)

		{
		  int px = (int)static_cast<int>(x);
		  int py = (int)static_cast<int>(y);
		  int rx = px + 1;
		  int ry = py + 1;
		  double d1 = depth_images_[f].at<unsigned short>(py, px) * 1e-3f;
		  double d2 = depth_images_[f].at<unsigned short>(py, rx) * 1e-3f;
		  double d3 = depth_images_[f].at<unsigned short>(ry, px) * 1e-3f;
		  double d4 = depth_images_[f].at<unsigned short>(ry, rx) * 1e-3f;
		  double wx = x - px;
		  double wy = y - py;
		  double w1 = (1.0 - wx) * (1.0 - wy);
		  double w2 = wx * (1.0 - wy);
		  double w3 = (1.0 - wx) * wy;
		  double w4 = wx * wy;
		  // test continuity
		  if (d1 && abs(d1 - d2) / d1 < 0.01 && abs(d1 - d3) / d1 < 0.01 && abs(d1 - d3) / d1 < 0.01)
		  {
			// bilinear interpolation
			double d = w1 * d1 + w2 * d2 + w3 * d3 + w4 * d4;
			// select vertices with depth error, error threshold is 0.01
			if (abs(point.z / d - 1) < 0.01){
			  vertices_[f].push_back(v);
			}
		  }
		}
	  }
	}
  }

  /* 
  with_weight 
	when set false, (color in different frame) weight = 1
	when set true, weight = Border * cos(Angle) / Distance^2
  */
  void ResetColor(ml::SensorData* input, ml::MeshDataf& mesh, bool with_weight)
  {
	// orignal model doesn't has vertex color
	mesh.m_Colors.clear();
	mesh.m_Colors.resize(mesh.m_Vertices.size(), ml::vec4f(0, 0, 0, 0));

	std::vector<float> weight(mesh.m_Vertices.size(), 0);
	for (int f = 0; f < frames_.size(); f++)
	{
	  // world2camera pose
	  ml::mat4f pose = input->m_frames[frames_[f]].getCameraToWorld().getInverse();
	  // inntrinsic matrix
	  ml::mat4f intrinsic = input->m_calibrationColor.m_intrinsic;
	  // Border mask: consider borders and discontinuities
	  cv::Mat border_mask;
	  if (with_weight){
		cv::Mat tmp;
		// Sobel filter find discontinuities
		Sobel(depth_images_[f], tmp, CV_32F, 1, 0);
		cv::convertScaleAbs(tmp, border_mask);
		for (int y = 0; y < height; y++){
		  for (int x = 0; x < width; x++){
			// if continuous
			if (border_mask.at<uchar>(y, x) < 240){
			  // border mask equals to pixel-to-pixel distance from the borders
			  float wx = width / 2.0 - abs(x - width / 2.0);
			  float wy = height / 2.0 - abs(y - height / 2.0);
			  border_mask.at<uchar>(y, x) = std::min(wx, wy);
			}
			else
			  // discontinuities set to 0
			  border_mask.at<uchar>(y, x) = 0;
		  }
		}
	  }

	  std::vector<int>& vts = vertices_[f];
#pragma omp parallel for
	  for (int i = 0; i < vts.size(); i++)
	  {
		ml::vec3f point = pose * mesh.m_Vertices[vts[i]];
		double x = (point.x / point.z) * intrinsic(0, 0) + intrinsic(0, 2);
		double y = (point.y / point.z) * intrinsic(1, 1) + intrinsic(1, 2);
		int px = static_cast<int>(x);
		int py = static_cast<int>(y);
		// in case of boundary vertices
		if (px < 9 || px >= width - 10 || py < 9 || py >= height - 10){
		  continue;
		}

		int rx = px + 1;
		int ry = py + 1;
		cv::Vec3b c[4];
		c[0] = color_images_[f].at<cv::Vec3b>(py, px);
		c[1] = color_images_[f].at<cv::Vec3b>(py, rx);
		c[2] = color_images_[f].at<cv::Vec3b>(ry, px);
		c[3] = color_images_[f].at<cv::Vec3b>(ry, rx);
		double wx = x - px;
		double wy = y - py;
		double w[4];
		w[0] = (1.0 - wx) * (1.0 - wy); 
		w[1] = wx * (1.0 - wy);
		w[2] = (1.0 - wx) * wy;
		w[3] = wx * wy;
		w[0] = w[0] * w[0];
		w[1] = w[1] * w[1];
		w[2] = w[2] * w[2];
		w[3] = w[3] * w[3];
		double sum = w[0] + w[1] + w[2] + w[3];
		w[0] /= sum; w[1] /= sum; w[2] /= sum; w[3] /= sum;
		ml::vec4f color(0, 0, 0, 1);
		for (int i = 0; i < 3; i++){
		  // bilinear interpolation
		  color[i] = w[0] * c[0][2 - i] + w[1] * c[1][2 - i] + w[2] * c[2][2 - i] + w[3] * c[3][2 - i];
		}
		if (with_weight){
		  // distance between camera and vertex
		  float distance = point.length();
		  float tmp = border_mask.at<uchar>(py, px)	// Border mask
			* abs(ml::vec3f::dot(mesh.m_Normals[vts[i]], point))  // Angle mask
			/ (mesh.m_Normals[vts[i]].length() * distance * distance * distance); // Depth mask
		  color *= tmp;
		  weight[vts[i]] += tmp;
		}
		else{
		  weight[vts[i]] += 1;
		}
		mesh.m_Colors[vts[i]] += color / 256.0;
	  }
	}

	for (int i = 0; i < mesh.m_Vertices.size(); i++){
	  if (weight[i]){
		mesh.m_Colors[i] /= weight[i];
	  }
	}
  }

  std::vector<int> frames_;
  std::vector<std::vector<int> > vertices_;
  std::vector<cv::Mat> color_images_;
  std::vector<cv::Mat> depth_images_;
  std::vector<cv::Mat> grey_images_;

  int height, width;
};

#endif //CICP_KEYFRAME