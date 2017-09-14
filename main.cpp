#include "tests/tests.h"
#include "fileutil/sensio.h"
#include "sdf/Fuser.h"
#include "GLRender/RenderPlatform.h"
#include "UVAtlas/mUVAtlas.h"

int main(int argc, char **argv)
{
	RenderPlatform platform(texture_width, texture_height);
	platform.InitDevice(&argc, argv);

	//Image2Sens("../render/tmp/", "../../data/cup.sens", false);

	// depth noise
	//Sens2SensNoise("../../data/depth/cup_noi015.sens", "../../data/perfect/cup.sens");

	Fuser fuser;
	//fuser.fuse("../../data/depth/cup_noi015.ply", "../../data/depth/cup_noi015.sens", true);

	test_color_icp();

	return 0;
}