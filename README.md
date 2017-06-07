# CosegProj: RGB-D Model Cosegmentation

## File Distribution
1. home/
2. home/data/
3. home/data/test.sens (Download from: https://drive.google.com/open?id=0BzZxe82vq9VMWkotSEUwZklnMG8)
4. home/project/
5. home/project/CosegProj/
6. home/thirdparty/ (Download from: https://drive.google.com/open?id=0BzZxe82vq9VMSHpXQ2dtUW5zMk0)

## Other Dependency
1. Visual Studio 2013 (64 bit!)
2. Cuda 8.0

## Libraries Include
1. glm
2. Eigen
3. OpenCV
4. MLIB
5. pangolin
6. Canon SDK
7. Ceres-Solver
8. glut
9. glew
10. glfw
11. SuiteSparse
12. uvatlas (for textures)

## Way to add Library
1. If you need additional library, put it in thirdparty.
2. Please make sure that you add the include and lib path in "All Configurations" with a relative path $(SolutionDir)../../thirdparty/XXX
3. In Linker->input, add debug and release lib name for debug and release mode respectively
4. In Build Events->Post Build Events, add copy "*.dll" "$(OutDir)" for debug and release mode respectively

## New component (code)
1. For each component, create a new folder in home/project/CosegProj/, and put all relavant files in that folder.
2. Add a filter (same name) in VSProj, include all related files

## features included
1. test_sens, load and use sens file
2. test_plane, a simple demo of plane detection

## RGB-D Calibration
1. pixel = intrinsic * camera2world.inv() * Pos3D_world
2. RGB and depth are well registered (same camera2world).
3. Frame sizes of RGB and Depth are not guaranteed to be the same (therefore different intrinsic).
