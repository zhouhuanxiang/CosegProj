#include "mlibutil.h"

#include <vector>

#include "ColorICP/KeyFrame.h"
#include "ColorICP/ColorICP.h"

//#define NEED_SUBDIVISION

void test_color_icp()
{
  // Load RGB-D Data
  ml::SensorData* input = new ml::SensorData("../../data/cup.sens");
  std::cout << "RGB_D data loaded\n";

  // KinectFusion mesh
  ml::MeshDataf mesh;
#ifdef NEED_SUBDIVISION
  ml::MeshIOf::loadFromPLY("../../data/cup.ply", mesh);
  // mesh subdivision by edge length or vertices size
  float edgeThresh = 0.005;
  float maxEdgeLen;
  do{
	maxEdgeLen = mesh.subdivideFacesLoop(/*edgeThresh =*/ edgeThresh);
	std::cout << "vertices size: " << mesh.m_Vertices.size() << "\n";
	std::cout << "maxEdgeLen: " << maxEdgeLen << "\n";
  } while (maxEdgeLen > edgeThresh && mesh.m_Vertices.size() < 100000);
  ml::MeshIOf::saveToPLY("../../data/cup_subdivided.ply", mesh);
#else
  ml::MeshIOf::loadFromPLY("../../data/cup_subdivided.ply", mesh);
#endif
  std::cout << "mesh loaded\n";

  KeyFrame kf;
  // select key frames
  kf.Init(input, mesh);
  // add noise to pose matrix
  kf.AddPoseNoise(input);
  // if abs( point#z - depthImage#z) < threshold, then add to frame 
  kf.VisibleTest(input, mesh);
  // key frame point color initialize
  kf.InitColor(input, mesh);

  // now begin optimization!
  int max_iteration = 1;
  for (int outer_iteraton = 0; outer_iteraton < max_iteration; outer_iteraton++)
  {
	// fix color, optimize pose
	OptimizePose(input, mesh, kf);

	// fix pose, optimize color
	kf.ResetColor(input, mesh);
  }

  ml::MeshIOf::saveToPLY("../../data/cup_color.ply", mesh);

  system("pause");
}