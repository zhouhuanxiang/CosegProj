#include "../mlibutil.h"
#include "../PlaneExtraction/Plane.h"

void test_plane()
{
	ml::SensorData* input = new ml::SensorData("../data/test.sens");
	for (int i = 0; i < input->m_frames.size(); ++i) {
		unsigned short* depth_data = input->decompressDepthAlloc(i);
		ml::vec3uc* color_data = input->decompressColorAlloc(i);
		cv::Mat colorimg(input->m_colorHeight, input->m_colorWidth, CV_8UC3, color_data);
		cv::Mat depthimg(input->m_depthHeight, input->m_depthWidth, CV_16UC1, depth_data);
		cv::Mat colorVis, depthVis;
		cv::cvtColor(colorimg, colorVis, CV_RGB2BGR);
		depthimg.convertTo(depthVis, CV_8U, 0.064f);

		// planeInfo <labelsMat, Params>,
		// Params[i] = plane_i.<param, disMat> 
		// param: (A,B,C,D)=>A^2+B^2+C^2=1, Ax+By+Cz=0 in camera space
		// disMat: \sum_p \frac{1}{N} \cdot outerproduct(p, p)
		auto planeInfo = ExtractPlane(depthimg, input->m_calibrationDepth.m_intrinsic(0, 0), input->m_calibrationDepth.m_intrinsic(1, 1),
			input->m_calibrationDepth.m_intrinsic(0, 2), input->m_calibrationDepth.m_intrinsic(1, 2));

		cv::Mat colorLabels = colorMember(planeInfo.first);
		colorVis = colorVis / 2 + colorLabels / 2;
		cv::imshow("color", colorVis);
		cv::imshow("depth", depthVis);
		cv::waitKey((i == 0) ? 0 : 1);
		// remember to free the data
		::free(color_data);
		::free(depth_data);
	}
}