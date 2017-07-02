#pragma once

#include "VoxelGrid.h"

class Fuser
{
public:
	Fuser();

	~Fuser();
	
	void fuse(const std::string& outputFile, const std::string& name, bool marchingCubes = false);
private:
	
	const int INF = std::numeric_limits<int>::max();

//	void render(Scene& scene, const Cameraf& camera, ColorImageR32G32B32A32& color, DepthImage32& depth);
//	void renderDepth(Scene& scene, const Cameraf& camera, DepthImage32& depth);

//	void includeInBoundingBox(BoundingBox3f& bb, const DepthImage16& depthImage, const Cameraf& camera);
//	void boundDepthMap(float minDepth, float maxDepth, DepthImage16& depthImage);

//	ml::ApplicationData& m_app;
//	D3D11RenderTarget m_renderTarget;
};

