#include "tests/tests.h"
#include "fileutil/sensio.h"
#include "sdf/Fuser.h"
int main()
{
  //Image2Sens("../render/tmp/", "../../data/oldman.sens");
  Fuser fuser;
  fuser.fuse("../../data/oldman.ply", "../../data/oldman.sens", true);

  //test_color_icp();

  return 0;
}