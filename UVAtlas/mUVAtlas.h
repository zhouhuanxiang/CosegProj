#ifndef M_UVATLAS_H
#define M_UVATLAS_H

#include "../ColorICP/KeyFrame.h"
#include "../GLRender/RenderGLColor.h"

#include <DirectXMesh/DirectXMesh.h>
#include <uvatlas.h>

static const int texture_width = 512 * 4;	
static const int texture_height = 512 * 4;

class mUVAtlas{
public:
	mUVAtlas(ml::MeshDataf& mesh);
	~mUVAtlas(){};

	void ExportAtlas(ml::MeshDataf& mesh);
	void ExportModel(ml::MeshDataf& mesh);
	void SetKthFrameUV(ml::SensorData* input, ml::MeshDataf& mesh, KeyFrame& kf, ml::TriMeshAcceleratorBVHf& tri_mesh_bvh, int k);
	void mUVAtlas::RenderKthFrameAtlas(ml::MeshDataf& mesh, int k);
	void GenTexture(ml::SensorData* input, 
		ml::MeshDataf& mesh, 
		KeyFrame& kf);
private:
	RenderGLColor render;

	int nFaces;
	std::vector<DirectX::UVAtlasVertex> vb;
	std::unique_ptr<uint32_t[]> Indices;
	std::vector<uint32_t> vertexRemapArray;
};

#endif