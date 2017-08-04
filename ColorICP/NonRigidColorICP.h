#ifndef CICP_NONRIGIDCOLOR_ICP
#define CICP_NONRIGIDCOLOR_ICP

#include "KeyFrame.h"

void NonRigidOptimizePose(ml::SensorData* input, ml::MeshDataf& mesh, KeyFrame& kf);

#endif //CICP_COLOR_ICP