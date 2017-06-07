#ifndef PLANE_H_
#define PLANE_H_
#ifndef PLANE_LIB
#define EXPORT_H
#else
#define EXPORT_H __declspec(dllimport)
#endif

#include <opencv2/opencv.hpp>

class PlaneHelper
{
public:
	double params[4];
	double disMat[16];
};

EXPORT_H std::pair<cv::Mat, std::vector<PlaneHelper> > ExtractPlane(cv::Mat depth_input, double fx, double fy, double cx, double cy, double threshold = 1e30);

inline cv::Mat colorMember(cv::Mat membership)
{
	cv::Mat color(membership.size(), CV_8UC3);
	for (int i = 0; i < membership.rows; ++i) {
		for (int j = 0; j < membership.cols; ++j) {
			int type;
			if (membership.type() == CV_8UC3)
			{
				type = membership.at<cv::Point3_<unsigned char> >(i, j).x;
			}
			else {
				type = membership.at<unsigned char>(i, j);
			}
			if (type == 255)
				color.at<cv::Point3_<unsigned char> >(i, j) = cv::Point3_<unsigned char>(0, 0, 0);
			else {
				int r = 1 - type % 2;
				int g = 1 - type / 2 % 2;
				int b = 1 - type / 4 % 2;
				color.at<cv::Point3_<unsigned char> >(i, j) = cv::Point3_<unsigned char>(r * 255, g * 255, b * 255);
			}
		}
	}
	return color;
}
#endif