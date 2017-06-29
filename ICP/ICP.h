#ifndef USE_ICP_H_
#define USE_ICP_H_
#include "../imgproc/filter.h"

ml::mat4d EstimatePose(cv::Mat prev_points, cv::Mat depth, cv::Mat normal, ml::mat4d& intrinsic, ml::mat4d& velocity);

#endif