#include "../imgproc/filter.h"

#include <opencv2/opencv.hpp>
#include <numeric>

inline float gaussR(float sigma, float dist)
{
	return exp(-(dist*dist) / (2.0*sigma*sigma));
}
inline float gaussD(float sigma, int x, int y)
{
	return exp(-((x*x + y*y) / (2.0f*sigma*sigma)));
}

inline float gaussD(float sigma, int x)
{
	return exp(-((x*x) / (2.0f*sigma*sigma)));
}

cv::Mat FilterDepth(cv::Mat depth) {
	float sigmaD = 3.0f; float sigmaR = 100;

	const int kernelRadius = (int)ceil(2.0*sigmaD);
	cv::Mat filteredDepth = cv::Mat::zeros(depth.rows, depth.cols, CV_16UC1);

	for (int i = 0; i < depth.rows; ++i) {
		for (int j = 0; j < depth.cols; ++j) {
			float sum = 0, sumWeight = 0;
			float depthCenter = depth.at<unsigned short>(i, j);
			for (int m = i - kernelRadius; m <= i + kernelRadius; ++m) {
				for (int n = j - kernelRadius; n <= j + kernelRadius; ++n) {
					if (m >= 0 && n >= 0 && m < depth.rows && n < depth.cols) {
						float d = depth.at<unsigned short>(m, n);
						if (d != 0 && abs(d - depthCenter) < sigmaR) {
							float weight = gaussD(sigmaD, m - i, n - j);
							sumWeight += weight;
							sum += weight * d;
						}
					}
				}
			}
			if (sumWeight)
				filteredDepth.at<unsigned short>(i, j) = sum / sumWeight;
		}
	}
	return filteredDepth;
}

cv::Mat ComputePoints(cv::Mat depth, float fx, float fy, float cx, float cy) {
	cv::Mat points = cv::Mat::zeros(depth.rows, depth.cols, CV_32FC3);
	for (int y = 0; y < depth.rows; ++y) {
		for (int x = 0; x < depth.cols; ++x) {
			float z = depth.at<unsigned short>(y, x) * 1e-3;
			if (z == 0)
				points.at<cv::Vec3f>(y, x) = cv::Vec3f(MINF, MINF, MINF);
			else {
				float px = (x - cx) / fx * z;
				float py = (y - cy) / fy * z;
				points.at<cv::Vec3f>(y, x) = cv::Vec3f(px, py, z);
			}
		}
	}
	return points;
}

cv::Mat ComputeNormal(cv::Mat points) {
	cv::Mat normal = cv::Mat::zeros(points.rows, points.cols, CV_32FC3);
	for (int y = 0; y < points.rows; ++y) {
		for (int x = 0; x < points.cols; ++x) {
			if (x > 0 && x < points.cols - 1 && y > 0 && y < points.rows - 1)
			{
				cv::Vec3f& CC = points.at<cv::Vec3f>(y, x);
				cv::Vec3f& PC = points.at<cv::Vec3f>(y + 1, x);
				cv::Vec3f& CP = points.at<cv::Vec3f>(y, x + 1);
				cv::Vec3f& MC = points.at<cv::Vec3f>(y - 1, x);
				cv::Vec3f& CM = points.at<cv::Vec3f>(y, x - 1);

				if (CC.val[0] != MINF && PC.val[0] != MINF && CP.val[0] != MINF && MC.val[0] != MINF && CM.val[0] != MINF)
				{
					cv::Vec3f n = (PC - MC).cross(CP - CM);
					float l = n.dot(n);
					
					if (l > 0.0f)
					{
						auto tn = n / sqrt(l);
						for (int i = 0; i < 3; ++i) {
							if (tn.val[i] < 0)
								tn.val[i] = -tn.val[i];
						}
						normal.at<cv::Vec3f>(y, x) = tn;
					}
				}
				else {
					normal.at<cv::Vec3f>(y, x) = cv::Vec3f(0, 0, 0);
				}
			}
		}
	}
	return normal;
}