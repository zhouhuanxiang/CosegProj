#ifndef USE_ICP_H_
#define USE_ICP_H_
#include "../imgproc/filter.h"

ml::mat4d Ceres2ML(double* param);

void ML2Ceres(ml::mat4d& t_extrinsic, double* param);

ml::mat4d EstimatePose(cv::Mat prev_points, cv::Mat depth, cv::Mat normal, ml::mat4d& intrinsic, ml::mat4d& velocity);

#endif