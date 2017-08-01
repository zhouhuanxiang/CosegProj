#ifndef CICP_COLOR_ICP
#define CICP_COLOR_ICP

#include "KeyFrame.h"

void OptimizePose(ml::SensorData* input, ml::MeshDataf& mesh, KeyFrame& kf);

#endif //CICP_COLOR_ICP