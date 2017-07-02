#include "tests/tests.h"
#include "fileutil/sensio.h"
#include "sdf/Fuser.h"
int main()
{
//	Image2Sens("C:\\Users\\Jingwei Huang\\Desktop\\Recon3D\\data", "lf.sens");
	Fuser fuser;
	fuser.fuse("a.ply", "lf.sens", true);
	//	MeshDataf mesh = MarchingCubes::doMC(*grid);
//	test_icp();
	//test_plane();
//	test_ceres();
//	test_sens_icp();
	return 0;
}