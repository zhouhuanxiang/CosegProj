#ifndef CICP_KEYFRAME
#define CICP_KEYFRAME

#include "mlibutil.h"

#include <vector>
#include <omp.h>

class KeyFrame
{
public:
  KeyFrame() {}
  ~KeyFrame() {}

  void Init(ml::SensorData* input, ml::MeshDataf& mesh)
  {
	int kf_size = 40;

	frames_.resize(kf_size);
	int step = input->m_frames.size() / kf_size;
	for (int i = 0; i < kf_size; i++){
	  frames_[i] = step * i;
	}

	vertices_.resize(kf_size);
	for (int i = 0; i < kf_size; i++)
	  vertices_[i].reserve(mesh.m_Vertices.size());
  }

  void AddPoseNoise(ml::SensorData* input)
  {
#pragma omp parallel for
	for (int f = 0; f < frames_.size(); f++)
	{
	  ml::mat4d pose = input->m_frames[frames_[f]].getCameraToWorld();
	  ml::mat4d new_pose = pose;
	  // add 0.001 noise to 3x4 submatrix 
	  for (int r = 0; r < 3; r++){
		for (int c = 0; c < 4; c++){
		  new_pose(r, c) *= 1 + (double)(rand() % 2000 - 1000) / 1000 / 100;
		}
	  }
	  input->m_frames[frames_[f]].setCameraToWorld(new_pose);
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
	  // depth data
	  unsigned short* depth_data = input->decompressDepthAlloc(frames_[f]);
	  cv::Mat depth_img(input->m_depthHeight, input->m_depthWidth, CV_16UC1, depth_data);

#pragma omp parallel for
	  for (int v = 0; v < mesh.m_Vertices.size(); v++)
	  {
		ml::vec3<double> point = pose * mesh.m_Vertices[v];
		double x = (point.x / point.z) * intrinsic(0, 0) + intrinsic(0, 2);
		double y = (point.y / point.z) * intrinsic(1, 1) + intrinsic(1, 2);
		if (x >= 9 && x < depth_img.cols - 10 && y >= 9 && y < depth_img.rows - 10)
		{
		  int px = (int)static_cast<int>(x);
		  int py = (int)static_cast<int>(y);
		  int rx = px + 1;
		  int ry = py + 1;
		  double d1 = depth_img.at<unsigned short>(py, px) * 1e-3f;
		  double d2 = depth_img.at<unsigned short>(py, rx) * 1e-3f;
		  double d3 = depth_img.at<unsigned short>(ry, px) * 1e-3f;
		  double d4 = depth_img.at<unsigned short>(ry, rx) * 1e-3f;
		  double wx = x - px;
		  double wy = y - py;
		  double w1 = (1.0 - wx) * (1.0 - wy);
		  double w2 = wx * (1.0 - wy);
		  double w3 = (1.0 - wx) * wy;
		  double w4 = wx * wy;
		  if (d1 && abs(d1 - d2) < 0.05 && abs(d1 - d3) < 0.05 && abs(d1 - d3) < 0.05)
		  {
			double d = w1 * d1 + w2 * d2 + w3 * d3 + w4 * d4;
			// threshold is 0.005
			if (abs(point.z / d - 1) < 0.005){
			  vertices_[f].push_back(v);
			}
		  }
		}
	  }
	  // free depth data
	  ::free(depth_data);
	}
  }

  void InitColor(ml::SensorData* input, ml::MeshDataf& mesh)
  {
	// orignal model doesn't has vertex color
	mesh.m_Colors.clear();
	mesh.m_Colors.resize(mesh.m_Vertices.size(), ml::vec4f(0, 0, 0, 0));

	std::vector<int> count(mesh.m_Vertices.size(), 0);
	for (int f = 0; f < frames_.size(); f++)
	{
	  // world2camera pose
	  ml::mat4d pose = input->m_frames[frames_[f]].getCameraToWorld().getInverse();
	  // inntrinsic matrix
	  ml::mat4d intrinsic = input->m_calibrationColor.m_intrinsic;
	  // color data
	  ml::vec3uc* color_data = input->decompressColorAlloc(frames_[f]);
	  cv::Mat color_img(input->m_colorHeight, input->m_colorWidth, CV_8UC3, color_data);

	  std::vector<int>& vts = vertices_[f];
#pragma omp parallel for
	  for (int i = 0; i < vts.size(); i++)
	  {
		ml::vec3<double> point = pose * mesh.m_Vertices[vts[i]];
		double x = (point.x / point.z) * intrinsic(0, 0) + intrinsic(0, 2);
		double y = (point.y / point.z) * intrinsic(1, 1) + intrinsic(1, 2);
		int px = (int)static_cast<int>(x);
		int py = (int)static_cast<int>(y);
		int rx = px + 1;
		int ry = py + 1;
		cv::Vec3b c1 = color_img.at<cv::Vec3b>(py, px);
		cv::Vec3b c2 = color_img.at<cv::Vec3b>(py, rx);
		cv::Vec3b c3 = color_img.at<cv::Vec3b>(ry, px);
		cv::Vec3b c4 = color_img.at<cv::Vec3b>(ry, rx);
		double wx = x - px;
		double wy = y - py;
		double w1 = (1.0 - wx) * (1.0 - wy);
		double w2 = wx * (1.0 - wy);
		double w3 = (1.0 - wx) * wy;
		double w4 = wx * wy;
		float b = (w1 * c1[0] + w2 * c2[0] + w3 * c3[0] + w4 * c4[0]) / 256.0;
		float g = (w1 * c1[1] + w2 * c2[1] + w3 * c3[1] + w4 * c4[1]) / 256.0;
		float r = (w1 * c1[2] + w2 * c2[2] + w3 * c3[2] + w4 * c4[2]) / 256.0;
		mesh.m_Colors[vts[i]] += ml::vec4f(r, g, b, 1);
		count[vts[i]]++;
	  }
	  // free color data
	  ::free(color_data);
	}

#pragma omp parallel for
	for (int i = 0; i < mesh.m_Vertices.size(); i++)
	{
	  if (count[i])
		mesh.m_Colors[i] /= count[i];
	}
  }

  void ResetColor(ml::SensorData* input, ml::MeshDataf& mesh)
  {
	// same as init procedure
	// the only difference is pose updated
	InitColor(input, mesh);
  }

  std::vector<int> frames_;
  std::vector<std::vector<int> > vertices_;
};

#endif //CICP_KEYFRAME