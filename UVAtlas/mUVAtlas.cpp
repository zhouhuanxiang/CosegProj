#include "mUVAtlas.h"



HRESULT __cdecl UVAtlasCallback(float fPercentDone)
{
	static ULONGLONG s_lastTick = 0;

	ULONGLONG tick = GetTickCount64();

	if ((tick - s_lastTick) > 1000)
	{
		wprintf(L"%.2f%%   \r", fPercentDone * 100);
		s_lastTick = tick;
	}

	if (_kbhit())
	{
		if (_getch() == 27)
		{
			wprintf(L"*** ABORT ***");
			return E_ABORT;
		}
	}

	return S_OK;
}

// for DEBUG
// output: 
//		atlas 3D model(.obj)
void mUVAtlas::ExportAtlas(ml::MeshDataf& mesh) {
	std::ofstream outFile;

	outFile.open("C:/Users/zhx/Desktop/o_atlas.obj");

	outFile << "mtllib atlas.mtl\no Plane\n";
	for (size_t j = 0; j < vb.size(); ++j)
	{
		outFile << "v " << vb[j].uv.x << " " << vb[j].uv.y << " " << 0 << "\n";
	}

	for (size_t j = 0; j < vb.size(); ++j)
	{
		outFile << "vt " << mesh.m_TextureCoords[vertexRemapArray[j]].x << " " << mesh.m_TextureCoords[vertexRemapArray[j]].y << "\n";
	}

	outFile << "usemtl map\ns 1\n";
	uint32_t* indices = Indices.get();
	for (size_t f = 0; f < nFaces; f++)
	{
		outFile << "f " << *(indices) + 1 << "/" << *(indices) + 1 << " " <<
			*(indices + 1) + 1 << "/" << *(indices + 1) + 1 << " " <<
			*(indices + 2) + 1 << "/" << *(indices + 2) + 1 << "\n";
		indices += 3;
	}
	outFile.close();
}

// output: 
//		texture-remaped model(.obj)
void mUVAtlas::ExportModel(ml::MeshDataf& mesh) {
	std::ofstream outFile;

	outFile.open("C:/Users/zhx/Desktop/o_model.obj");

	outFile << "mtllib model.mtl\no Plane\n";
	for (size_t j = 0; j < vb.size(); ++j)
	{
		outFile << "v " << mesh.m_Vertices[vertexRemapArray[j]].x << " " << mesh.m_Vertices[vertexRemapArray[j]].y << " " << mesh.m_Vertices[vertexRemapArray[j]].z << "\n";
		//outFile << "v " << vb[j].pos.x << " " << vb[j].pos.y << " " << vb[j].pos.z << "\n";
	}

	for (size_t j = 0; j < vb.size(); ++j)
	{
		outFile << "vt " << vb[j].uv.x << " " << vb[j].uv.y << "\n";
	}

	outFile << "usemtl map\ns 1\n";
	uint32_t* indices = Indices.get();
	for (size_t f = 0; f < nFaces; f++)
	{
		outFile << "f " << *(indices)+1 << "/" << *(indices)+1 << " " <<
			*(indices + 1) + 1 << "/" << *(indices + 1) + 1 << " " <<
			*(indices + 2) + 1 << "/" << *(indices + 2) + 1 << "\n";
		indices += 3;
	}
	outFile.close();
}

// uses Microsoft/UVAtlas to generate atlas (only shape, still needs to be rendered with color) (for all frames)
// output: 
//		vb					-- new vertex (position and uv)
//		Indices				-- updated indices
//		vertexRemapArray	-- map new vertex to old vertex
mUVAtlas::mUVAtlas(ml::MeshDataf& mesh)
{
	int nVerts = mesh.m_Vertices.size();
	nFaces = mesh.m_FaceIndicesVertices.size();
	// Positions
	std::unique_ptr<DirectX::XMFLOAT3[]> Positions;
	Positions.reset(new (std::nothrow) DirectX::XMFLOAT3[nVerts]);
	DirectX::XMFLOAT3* vptr = Positions.get();
	for (auto iter : mesh.m_Vertices){
		vptr->x = iter.x;
		vptr->y = iter.y;
		vptr->z = iter.z;
		vptr++;
	}
	// Indices
	Indices.reset(new (std::nothrow) uint32_t[nFaces * 3]);
	uint32_t* fptr = Indices.get();
	for (ml::MeshDataf::Indices::iterator iter = mesh.m_FaceIndicesVertices.begin(); iter != mesh.m_FaceIndicesVertices.end(); iter++){
		for (int i = 0; i < 3; i++){
			(*fptr) = (*iter)[i];
			fptr++;
		}
	}
	// Adjacency
	std::unique_ptr<uint32_t[]> Adjacency;
	Adjacency.reset(new (std::nothrow) uint32_t[nFaces * 3]);
	DirectX::GenerateAdjacencyAndPointReps(Indices.get(), nFaces, Positions.get(), nVerts, 0.f, nullptr, Adjacency.get());
	//
	float outStretch = 0.f;
	size_t outCharts = 0;
	std::vector<uint8_t> ib;
	std::vector<uint32_t> facePartitioning;
	//return;
	HRESULT hr = DirectX::UVAtlasCreate(Positions.get(), nVerts,
		Indices.get(), DXGI_FORMAT_R32_UINT, nFaces,
		0, 0.16667f, /*width=*/texture_width, /*height=*/texture_height, 2.f,
		Adjacency.get(), nullptr,
		nullptr,
		UVAtlasCallback, DirectX::UVATLAS_DEFAULT_CALLBACK_FREQUENCY,
		DirectX::UVATLAS_DEFAULT, vb, ib,
		&facePartitioning,
		&vertexRemapArray,
		&outStretch, &outCharts);
	if (FAILED(hr))
	{}

	assert((ib.size() / sizeof(uint32_t)) == (nFaces * 3));
	assert(facePartitioning.size() == nFaces);
	assert(vertexRemapArray.size() == vb.size());

	uint32_t* idx = reinterpret_cast<uint32_t*>(ib.data());
	memcpy(Indices.get(), idx, sizeof(uint32_t) * 3 * nFaces);

	ExportModel(mesh);
}

void mUVAtlas::SetKthFrameUV(ml::SensorData* input, ml::MeshDataf& mesh, KeyFrame& kf, ml::TriMeshAcceleratorBVHf& tri_mesh_bvh, int k)
{
	ml::mat4d intrinsic = input->m_calibrationColor.m_intrinsic;
	ml::mat4d pose_c2w = input->m_frames[kf.frames_[k]].getCameraToWorld();
	ml::mat4d pose_w2c = pose_c2w.getInverse();

	cv::Mat kth_image = kf.color_images_[k];
	int rows = kth_image.rows;
	int cols = kth_image.cols;

	int nVerts = mesh.m_Vertices.size();
	mesh.m_TextureCoords.clear();
	mesh.m_TextureCoords.resize(nVerts, ml::vec2f(0, 0));
	for (int y = 5; y < rows - 5; y++){
		for (int x = 5; x < cols - 5; x++){
			// ray
			float direction_x = (x - intrinsic(0, 2)) / intrinsic(0, 0);
			float direction_y = (y - intrinsic(1, 2)) / intrinsic(1, 1);
			ml::Rayf ray(ml::vec3f(0, 0, 0), ml::vec3f(direction_x, direction_y, 1));
			// accelerators
			std::vector<std::pair<const ml::TriMeshAcceleratorBVHf*, ml::mat4f> > accelerators;
			accelerators.push_back(std::pair<const ml::TriMeshAcceleratorBVHf*, ml::mat4f>(&tri_mesh_bvh, pose_c2w));//
			// intersection test
			UINT object_index;
			ml::TriMeshRayAcceleratorf::Intersection intersection;
			intersection = ml::TriMeshRayAcceleratorf::getFirstIntersectionTransform(ray, accelerators, object_index);
			// ray and mesh intersects
			if (intersection.triangle){
				int index = intersection.triangle->getIndex();
				for (int i = 0; i < 3; i++){
					// normal vector should point to camera
					ml::vec3f vnor = intersection.triangle->getSurfaceNormal(i % 2, i / 2);
					ml::vec4f vnor_c(vnor, 0);
					vnor_c = pose_w2c * vnor_c;
					vnor = vnor_c.getVec3();
					vnor.normalize();
					if (vnor.z * -1 > 0.5){
						ml::vec3f vpos = intersection.triangle->getSurfacePosition(i % 2, i / 2);
						ml::vec4f vpos_c(vpos, 1);
						vpos_c = pose_w2c * vpos_c;
						float ix = vpos_c.x / vpos_c.z * intrinsic(0, 0) + intrinsic(0, 2);
						float iy = vpos_c.y / vpos_c.z * intrinsic(1, 1) + intrinsic(1, 2);
						if (abs(ix - x) < 1 && abs(iy - y) < 1){
							float u = float(ix) / cols;
							float v = 1.0f - float(iy) / rows;
							//mesh.m_Colors[mesh.m_FaceIndicesVertices[index][i]] = ml::vec4f(u, v, 1.0, 1.0);
							mesh.m_TextureCoords[mesh.m_FaceIndicesVertices[index][i]] = ml::vec2f(u, v);
						}
					}
				}
			}
		}
	}

	// UVs in mesh.m_TextureCoords and material in kf.color_images_
	// UVAtlas
	for (int i = rows - 5; i < rows; i++){
		cv::Vec3b* kth_image_i = kth_image.ptr<cv::Vec3b>(i);
		for (int j = 0; j < 5; j++){
			kth_image_i[j] = cv::Vec3b(0, 0, 0);
		}
		for (int j = cols - 5; j < cols; j++){
			kth_image_i[j] = cv::Vec3b(0, 0, 0);
		}
	}
	char texture_file[100];
	sprintf(texture_file, "C:/Users/zhx/Desktop/tex/%d.jpg", k);
	cv::imwrite(texture_file, kth_image);
}

void mUVAtlas::RenderKthFrameAtlas(ml::MeshDataf& mesh, int k)
{
	// init
	char texture_file[100];
	sprintf(texture_file, "C:/Users/zhx/Desktop/tex/%d.jpg", k);
	std::vector<std::string> textureFiles = { texture_file };
	render.init(texture_width, texture_height, textureFiles);
	//load mesh
	std::vector<glm::vec3> positions(nFaces * 3);
	std::vector<int> indices(nFaces * 3);
	std::vector<glm::vec2> uvs(nFaces * 3);
	std::vector<float> materials(nFaces * 3);
	std::vector<glm::vec3> normals(nFaces * 3);
	uint32_t* tmp = Indices.get();
	for (int f = 0; f < nFaces; f++){
		for (int v = 0; v < 3; v++){
			int idx = *(tmp + v);
			positions[3 * f + v] = glm::vec3(vb[idx].uv.x - 0.5, -vb[idx].uv.y + 0.5, 1.5);
			indices[3 * f + v] = 3 * f + v;
			uvs[3 * f + v] = glm::vec2(mesh.m_TextureCoords[vertexRemapArray[idx]].x, mesh.m_TextureCoords[vertexRemapArray[idx]].y);
			materials[3 * f + v] = 0;
			normals[3 * f + v] = glm::vec3(0, 0, -1);
		}
		if (uvs[3 * f].x < 0.01 && uvs[3 * f].y < 0.01 ||
			uvs[3 * f + 1].x < 0.01 && uvs[3 * f + 1].y < 0.01 ||
			uvs[3 * f + 2].x < 0.01 && uvs[3 * f + 2].y < 0.01){
			uvs[3 * f].x = uvs[3 * f].y = uvs[3 * f + 1].x = uvs[3 * f + 1].y = uvs[3 * f + 2].x = uvs[3 * f + 2].y = 0;
		}
		tmp += 3;
	}
	render.LoadMesh(positions, indices, uvs, materials, normals);
	//render
	cv::Mat raw_image(texture_width, texture_height, CV_8UC3);
	render.RenderColorImg(raw_image);

	sprintf(texture_file, "C:/Users/zhx/Desktop/tex/%d_remap.jpg", k);
	cv::imwrite(texture_file, raw_image);
}

// main function
void mUVAtlas::GenTexture(ml::SensorData* input, ml::MeshDataf& mesh, KeyFrame& kf)
{
	ml::TriMeshf tri_mesh(mesh);
	ml::TriMeshAcceleratorBVHf tri_mesh_bvh(tri_mesh);

	for (int f = 0; f < kf.frames_.size(); f++){
		SetKthFrameUV(input, mesh, kf, tri_mesh_bvh, f);
		RenderKthFrameAtlas(mesh, f);
	}

	// notice: #keyfame < 256
	// otherwise, adjust the size of image
	char texture_file[256];
	cv::Mat color_image(texture_width, texture_height, CV_16UC3);
	cv::Mat count_image(texture_width, texture_height, CV_8U);
	color_image.setTo(cv::Vec3w(0, 0, 0));
	count_image.setTo(0);
	int alpha = 5;
	for (int f = 0; f < kf.frames_.size(); f++){
		cv::Mat tmp(texture_width, texture_height, CV_8UC3);
		sprintf(texture_file, "C:/Users/zhx/Desktop/tex/%d_remap.jpg", f);
		tmp = cv::imread(texture_file);
		for (int i = 0; i < texture_height; i++){
			cv::Vec3b* tmp_i = tmp.ptr<cv::Vec3b>(i);
			uchar* count_image_i = count_image.ptr<uchar>(i);
			cv::Vec3w* color_image_i = color_image.ptr<cv::Vec3w>(i);
			for (int j = 0; j < texture_width; j++){
				if (tmp_i[j][0] > 0 && tmp_i[j][1] > 0 && tmp_i[j][2] > 0){
					if (count_image_i[j] == 0){
						count_image_i[j] = 1;
						color_image_i[j] = tmp_i[j];
					}
					else{
						cv::Vec3w cur = color_image_i[j] / count_image_i[j];
						if (cur[0] > alpha * tmp_i[j][0] && cur[1] > alpha * tmp_i[j][1] && cur[2] > alpha * tmp_i[j][2]){
							continue;
						}
						else if (alpha * cur[0] < tmp_i[j][0] && alpha * cur[1] < tmp_i[j][1] && alpha * cur[2] < tmp_i[j][2]){
							count_image_i[j] = 1;
							color_image_i[j] = tmp_i[j];
						}
						else{
							count_image_i[j]++;
							color_image_i[j] += tmp_i[j];
						}
					}
				}
			}
		}
	}
	for (int i = 0; i < texture_height; i++){
		uchar* count_image_i = count_image.ptr<uchar>(i);
		cv::Vec3w* color_image_i = color_image.ptr<cv::Vec3w>(i);
		for (int j = 0; j < texture_width; j++){
			if (count_image_i[j] != 0){
				color_image_i[j] /= count_image_i[j];
			}
		}
	}
	cv::Mat final_image(texture_width, texture_height, CV_8UC3);
	color_image.convertTo(final_image, CV_8UC3);
	sprintf(texture_file, "C:/Users/zhx/Desktop/tex/final.jpg");
	cv::imwrite(texture_file, final_image);
}
