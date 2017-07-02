#include "mlibutil.h"

#include <Eigen/Dense>  

#include <fstream>
#include <string>
#include <sstream>
#include <random>
#include "imgproc\filter.h"
#include "..\ICP\ICP.h"

void test_icp()
{
	// Load RGB-D Data
	ml::SensorData* input = new ml::SensorData("../../data/test.sens");
	// Basic Information
	std::cout << "Color Frame Size (w, h) = (" << input->m_colorWidth << ", " << input->m_colorHeight << ")\n";
	std::cout << "Depth Frame Size (w, h) = (" << input->m_depthWidth << ", " << input->m_depthHeight << ")\n";
	std::cout << "Color Intrinsic Matrix:\n";
	std::cout << input->m_calibrationColor.m_intrinsic << "\n";
	std::cout << "Depth Intrinsic Matrix:\n";
	std::cout << input->m_calibrationDepth.m_intrinsic << "\n";
	std::cout << "Frame number:\n";
	std::cout << input->m_frames.size() << "\n";
	// Frame Information¡®
	cv::Mat prev_points, prev_normal;
	std::vector<ml::mat4d> poses;
	int start_frame = 80;
	for (int i = start_frame; i < input->m_frames.size(); ++i) {
		std::cout << i << "\n";
		// decompress depth and color data
		// depth (mm)
		unsigned short* depth_data = input->decompressDepthAlloc(i);
		// color (rgb [0,255],[0,255],[0,255])
		ml::vec3uc* color_data = input->decompressColorAlloc(i);
		// extrinsic (camera2world)
		ml::mat4d pose = input->m_frames[i].getCameraToWorld();
		ml::mat4d intrinsic = input->m_calibrationColor.m_intrinsic;
		ml::mat4d rigid_pose;
		//get_homography_martix(input, i, depth_data, color_data, homography_pose);
		//print_ith_frame_2_obj(input, i, depth_data, color_data, pose, intrinsic);


		// Visualize color and depth
		cv::Mat colorimg(input->m_colorHeight, input->m_colorWidth, CV_8UC3, color_data);
		cv::Mat depthimg(input->m_depthHeight, input->m_depthWidth, CV_16UC1, depth_data);
		cv::Mat colorVis, depthVis;
		cv::cvtColor(colorimg, colorVis, CV_RGB2BGR);
		depthimg.convertTo(depthVis, CV_8U, 0.064f);
		cv::Mat filterDepth = FilterDepth(depthimg);
		cv::Mat points = ComputePoints(filterDepth, intrinsic(0, 0), intrinsic(1, 1), intrinsic(0, 2), intrinsic(1, 2));
		cv::Mat normal = ComputeNormal(points);
		points = ComputePoints(depthimg, intrinsic(0, 0), intrinsic(1, 1), intrinsic(0, 2), intrinsic(1, 2));
		if (i == start_frame) {
			poses.push_back(ml::mat4d::identity());
		}
		else {
			ml::mat4d velocity = ml::mat4d::identity();
			if (i > start_frame + 1) {
				//				velocity = poses[poses.size() - 1] * poses[poses.size() - 2].getInverse();
			}
			ml::mat4d relaPose = EstimatePose(prev_points, depthimg, normal, intrinsic, velocity);
			poses.push_back(poses.back() * relaPose.getInverse());
		}
		std::cout << poses.back() << "\n";
		std::vector<cv::Vec3b> colors;
		std::vector<cv::Vec3f> pts;
		for (int y = 0; y < depthimg.rows; y += 8) {
			for (int x = 0; x < depthimg.cols; x += 8) {
				cv::Vec3f& p = points.at<cv::Vec3f>(y, x);
				if (p.val[0] != MINF) {
					ml::vec3f pt(p.val[0], p.val[1], p.val[2]);
					pt = poses.back() * pt;
					pts.push_back(cv::Vec3f(pt.x, pt.y, pt.z));
					colors.push_back(colorimg.at<cv::Vec3b>(y, x));
				}
			}
		}
		char buffer[200];
		sprintf(buffer, "ply\\%d.ply", i);
		std::ofstream os(buffer);
		os << "ply\nformat ascii 1.0\nelement vertex " << colors.size() << "\nproperty float x\nproperty float y\nproperty float z\n";
		os << "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";
		for (int j = 0; j < colors.size(); ++j) {
			os << pts[j].val[0] << " " << pts[j].val[1] << " " << pts[j].val[2] << " "
				<< (int)colors[j].val[0] << " " << (int)colors[j].val[1] << " " << (int)colors[j].val[2] << "\n";
		}
		prev_normal = normal.clone();
		prev_points = points.clone();
		::free(color_data);
		::free(depth_data);
	}

	system("pause");
}