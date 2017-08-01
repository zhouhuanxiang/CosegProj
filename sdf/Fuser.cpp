
#include "Fuser.h"
#include "MarchingCubes.h"
#define VOXEL_SIZE 0.01
#define DEPTH_MIN 0.4
#define DEPTH_MAX 4

Fuser::Fuser()
{
	
}

Fuser::~Fuser()
{

}


bool valid(ml::mat4f& c) {
	bool flag = false;
	for (int j = 0; j < 4; ++j) {
		for (int k = 0; k < 4; ++k) {
			if (c(j, k) != 0)
				flag = true;
		}
	}
	return flag;
}
void Fuser::fuse(const std::string& outputFile, const std::string& name, bool debugOut /*= false*/)
{
	ml::SensorData* input = new ml::SensorData(name.c_str());
	const float voxelSize = VOXEL_SIZE;
	const float depthMin = DEPTH_MIN;
	const float depthMax = DEPTH_MAX;

	const unsigned int imageWidth = input->m_depthWidth;
	const unsigned int imageHeight = input->m_depthHeight;

	//PointCloudf pc;
	BoundingBox3f bounds;
	for (int iter = 0; iter < input->m_frames.size(); iter += 1) {
		printf("test on frame %d\n", iter);
		auto c = input->m_frames[iter].getCameraToWorld();
		unsigned short* depth_data = input->decompressDepthAlloc(iter);
		for (int i = 0; i < input->m_depthHeight; ++i) {
			for (int j = 0; j < input->m_depthWidth; ++j) {
				float depth_value = depth_data[i * input->m_depthWidth + j] * 1e-3;
				if (depth_value == 0)
					continue;
				ml::vec3f camera_pos((j - input->m_calibrationDepth.m_intrinsic(0, 2)) / input->m_calibrationDepth.m_intrinsic(0, 0) * depth_value,
					(i - input->m_calibrationDepth.m_intrinsic(1, 2)) / input->m_calibrationDepth.m_intrinsic(1, 1) * depth_value,
					depth_value);
				if (depth_value >= depthMin && depth_value <= depthMax) {
					vec3f posWorld = c * camera_pos;
					bounds.include(posWorld);
				}
			}
		}
		::free(depth_data);
	}

	bounds.scale(1.0f + voxelSize*5.0f);
	vec3ul voxelDim = math::ceil(bounds.getExtent() / voxelSize);
	mat4f worldToGrid = mat4f::translation(-bounds.getMin());

	size_t estimatedMemory = voxelDim.x * voxelDim.y * voxelDim.z * sizeof(Voxel);

	VoxelGrid* grid = new VoxelGrid(voxelDim, worldToGrid, voxelSize, depthMin, depthMax);

	for (int iter = 0; iter < input->m_frames.size(); iter += 1) {
		printf("test on frame %d\n", iter);
		auto c = input->m_frames[iter].getCameraToWorld();
		if (!valid(c))
			continue;
		unsigned short* depth_data = input->decompressDepthAlloc(iter);

		for (int i = 0; i < input->m_depthHeight; ++i) {
			for (int j = 0; j < input->m_depthWidth; ++j) {
				float depth_value = depth_data[i * input->m_depthWidth + j] * 1e-3;
				if (depth_value == 0)
					continue;
				ml::vec3f camera_pos((j - input->m_calibrationDepth.m_intrinsic(0, 2)) / input->m_calibrationDepth.m_intrinsic(0, 0) * depth_value,
					(i - input->m_calibrationDepth.m_intrinsic(1, 2)) / input->m_calibrationDepth.m_intrinsic(1, 1) * depth_value,
					depth_value);
				if (depth_value >= depthMin && depth_value <= depthMax) {
					vec3f posWorld = c * camera_pos;
					bounds.include(posWorld);
				}
			}
		}

		for (int i = 0; i < input->m_depthHeight; ++i) {
			for (int j = 0; j < input->m_depthWidth; ++j) {
				float depth_value = depth_data[i * input->m_depthWidth + j] * 1e-3;
				if (depth_value < DEPTH_MIN || depth_value > DEPTH_MAX)
					depth_data[i * input->m_depthWidth + j] = 0;
			}
		}
		grid->integrate(input->m_calibrationDepth.m_intrinsic, c, depth_data, input->m_depthWidth, input->m_depthHeight);
	}

	MeshDataf mesh = MarchingCubes::doMC(*grid);
	printf("export mesh!\n");
	MeshIOf::saveToPLY(outputFile, mesh);
	printf("finsih...\n");
//	printf("%s\n", outputFile.c_str());
//	grid->saveToFile(outputFile);
	SAFE_DELETE(grid);
}



