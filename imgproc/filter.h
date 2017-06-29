#ifndef FILTER_H_
#define FILTER_H_

#include "../mlibutil.h"

#define MINF FLT_MIN

cv::Mat FilterDepth(cv::Mat depth);
cv::Mat ComputePoints(cv::Mat depth, float fx, float fy, float cx, float cy);
cv::Mat ComputeNormal(cv::Mat points);

#endif