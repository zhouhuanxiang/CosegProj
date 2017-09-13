#include "mlibutil.h"

#include <vector>

#include "ColorICP/KeyFrame.h"
#include "ColorICP/ColorICP.h"
#include "ColorICP/NonRigidColorICP.h"

#include "../UVAtlas/mUVAtlas.h"

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
	ml::MeshIOf::loadFromPLY("../../data/cup_sim1.ply", mesh);
#endif
	std::cout << "mesh loaded\n";
	mesh.computeVertexNormals();
	std::cout << "mesh vertex normal computed\n";

	KeyFrame kf;
	// select key frames
	kf.Init(input, mesh);
	// if abs( point#z - depthImage#z) / depthImage#z < threshold, then add to frame 
	kf.VisibleTest(input, mesh);

	// now begin optimization!
	int max_iteration = 0;
	// use non-rigid correction or not
	bool non_rigid = true;
	for (int outer_iteraton = 0; outer_iteraton < max_iteration; outer_iteraton++){
		std::cout << outer_iteraton << "th iteration\n";
		// fix pose, optimize color
		kf.ResetColor(input, mesh, false, non_rigid);
		// fix color, optimize pos
		if (non_rigid)
			NonRigidOptimizePose(input, mesh, kf);
		else
			OptimizePose(input, mesh, kf);

		if (outer_iteraton % 10 == 1){
			char filename[50];
			sprintf(filename, "../../data/cup_color%d.ply", outer_iteraton);
			ml::MeshIOf::saveToPLY(filename, mesh);
		}
		std::cout << "end\n";
	}

	//test
	//ml::MeshIOf::loadFromOBJ("C:\\Users\\zhx\\Desktop\\data\\cat\\1_tri.obj", mesh);

	mUVAtlas uvatlas(mesh);

	// generate multi texture image(#texture = #keyframe)
	uvatlas.GenTexture(input, mesh, kf);

	system("pause");
}