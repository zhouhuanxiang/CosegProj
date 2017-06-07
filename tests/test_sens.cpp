#include "mlibutil.h"

void test_sens()
{
	// Load RGB-D Data
	ml::SensorData* input = new ml::SensorData("../data/test.sens");
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

	for (int i = 0; i < input->m_frames.size(); ++i) {
		// decompress depth and color data
		// depth (mm)
		unsigned short* depth_data = input->decompressDepthAlloc(i);
		// color (rgb [0,255],[0,255],[0,255])
		ml::vec3uc* color_data = input->decompressColorAlloc(i);
		// extrinsic (camera2world)
		ml::mat4f pose = input->m_frames[i].getCameraToWorld();
		// Visualize color and depth
		cv::Mat colorimg(input->m_colorHeight, input->m_colorWidth, CV_8UC3, color_data);
		cv::Mat depthimg(input->m_depthHeight, input->m_depthWidth, CV_16UC1, depth_data);
		cv::Mat colorVis, depthVis;
		cv::cvtColor(colorimg, colorVis, CV_RGB2BGR);
		depthimg.convertTo(depthVis, CV_8U, 0.064f);
		cv::imshow("color", colorVis);
		cv::imshow("depth", depthVis);
		cv::waitKey((i == 0) ? 0 : 1);
		// remember to free the data
		::free(color_data);
		::free(depth_data);
	}
}