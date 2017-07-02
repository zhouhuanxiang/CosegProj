#pragma once
#include "../mlibutil.h"
#include <opencv2/opencv.hpp>
using namespace ml;
struct Voxel {
	Voxel() {
		freeCtr = 0;
		sdf = 0;
		weight = 0;
		color = vec3us(0, 0, 0);
	}

	unsigned int	freeCtr;
	float			sdf;
	uchar			weight;
	vec3us			color;
};

class VoxelGrid : public Grid3 < Voxel >
{
public:
	VoxelGrid(){}
	VoxelGrid(const vec3l& dim, const mat4f& worldToGrid, float voxelSize, float depthMin, float depthMax) : Grid3(dim.x, dim.y, dim.z) {
		m_voxelSize = voxelSize;
		m_depthMin = depthMin;
		m_depthMax = depthMax;
		m_worldToGrid = worldToGrid;
		m_gridToWorld = m_worldToGrid.getInverse();


		m_trunaction = m_voxelSize * 5.0f;
		//m_trunaction = m_voxelSize * 1.5f;
		//m_truncationScale = m_trunaction * 0.5f;
		m_truncationScale = 0.0f;
		m_weightUpdate = 1;
	}

	~VoxelGrid() {

	}

	void reset() {
#pragma omp parallel for
		for (int i = 0; i < (int)getNumElements(); i++) {
			getData()[i] = Voxel();
		}
	}

	float fetchDepth(unsigned short* depthImage, int width, int height, float x, float y) {
		int lx = x, ly = y, rx = lx + 1, ry = ly + 1;
		float wx = x - lx, wy = y - ly;
		unsigned short p1 = 0, p2 = 0, p3 = 0, p4 = 0;
		int index = ly * width + lx;
		float w1 = 0, w2 = 0, w3 = 0, w4 = 0;
		if (lx >= 0 && ly >= 0 && lx < width && ly < height)
			p1 = depthImage[index], w1 = (1 - wx) * (1 - wy);
		if (rx >= 0 && ly >= 0 && rx < width && ly < height)
			p2 = depthImage[index + 1], w2 = wx * (1 - wy);
		if (lx >= 0 && ry >= 0 && lx < width && ry < height)
			p3 = depthImage[index + width], w3 = (1 - wx) * wy;
		if (rx >= 0 && ry >= 0 && rx < width && ry < height)
			p4 = depthImage[index + 1 + width], w4 = wx * wy;
		if (w1 + w2 + w3 + w4 >= 0)
			return (p1 * w1 + p2 * w2 + p3 * w3 + p4 * w4) / (w1 + w2 + w3 + w4) * 1e-3f;
		else
			return 0;
	}

	void integrate(const mat4f& intrinsic, const mat4f& cameraToWorld, unsigned short* depthImage, int width, int height) {

		const mat4f worldToCamera = cameraToWorld.getInverse();
		BoundingBox3<int> voxelBounds = computeFrustumBounds(intrinsic, cameraToWorld, width, height);

#pragma omp parallel for
		for (int k = voxelBounds.getMinZ(); k <= voxelBounds.getMaxZ(); k++) {
			for (int j = voxelBounds.getMinY(); j <= voxelBounds.getMaxY(); j++) {
				for (int i = voxelBounds.getMinX(); i <= voxelBounds.getMaxX(); i++) {

					//transform to current frame
					vec3f p = worldToCamera * voxelToWorld(vec3i(i, j, k));

					//project into depth image
					p = skeletonToDepth(intrinsic, p);

					vec3i pi = math::round(p);
					if (pi.x >= 0 && pi.y >= 0 && pi.x < width && pi.y < height) {
						float d = fetchDepth(depthImage, width, height, pi.x, pi.y);

						//check for a valid depth range
						if (d >= m_depthMin && d <= m_depthMax) {

							//update free space counter if voxel is in front of observation
							if (p.z < d) {
								(*this)(i, j, k).freeCtr++;
							}

							//compute signed distance; positive in front of the observation
							float sdf = d - p.z;
							float truncation = getTruncation(d);

							//if (std::abs(sdf) < truncation) {
							if (sdf > -truncation) {
								Voxel& v = (*this)(i, j, k);
								v.sdf = (v.sdf * (float)v.weight + sdf * (float)m_weightUpdate) / (float)(v.weight + m_weightUpdate);
								v.weight = (uchar)std::min((int)v.weight + (int)m_weightUpdate, (int)std::numeric_limits<unsigned char>::max());
							}
						}
					}

				}
			}
		}
	}

	//! returns all the voxels on the isosurface
	std::vector<Voxel> getSurfaceVoxels(unsigned int weightThresh, float sdfThresh) const {

		std::vector<Voxel> res;
		for (size_t k = 0; k < getDimZ(); k++) {
			for (size_t j = 0; j < getDimY(); j++) {
				for (size_t i = 0; i < getDimX(); i++) {

					const Voxel& v = (*this)(i, j, k);
					if (v.weight >= weightThresh && std::abs(v.sdf) < sdfThresh) {
						res.push_back(v);
					}
				}
			}
		}

		return res;
	}

	BinaryGrid3 toBinaryGridFree(unsigned int freeThresh) const {
		BinaryGrid3 res(getDimX(), getDimY(), getDimZ());
		for (size_t k = 0; k < getDimZ(); k++) {
			for (size_t j = 0; j < getDimY(); j++) {
				for (size_t i = 0; i < getDimX(); i++) {

					if ((*this)(i, j, k).freeCtr >= freeThresh) {
						res.setVoxel(i, j, k);
					}
				}
			}
		}
		return res;
	}

	BinaryGrid3 toBinaryGridOccupied(unsigned int weightThresh, float sdfThresh) const {

		BinaryGrid3 res(getDimX(), getDimY(), getDimZ());
		for (size_t k = 0; k < getDimZ(); k++) {
			for (size_t j = 0; j < getDimY(); j++) {
				for (size_t i = 0; i < getDimX(); i++) {

					if ((*this)(i, j, k).weight >= weightThresh && std::abs((*this)(i, j, k).sdf) < sdfThresh) {
						res.setVoxel(i, j, k);
					}
				}
			}
		}
		return res;
	}


	void saveToFile(const std::string& filename) const {

		std::ofstream outFile(filename, std::ios::binary);

		outFile << "dimensions\t" << (UINT64)getDimX() << " " << (UINT64)getDimY() << " " << (UINT64)getDimZ() << std::endl;
		outFile << "worldToGrid\t";
		for (unsigned int i = 0; i < 15; i++) {
			outFile << m_worldToGrid.getData()[i] / m_voxelSize << " ";
		}
		outFile << m_worldToGrid.getData()[15] << std::endl;
		outFile << "depthMin\t" << m_depthMin << std::endl;
		outFile << "depthMax\t" << m_depthMax << std::endl;
		outFile << "voxelSize\t" << m_voxelSize << std::endl;
		outFile.write((char*)getData(), sizeof(Voxel) * getNumElements());

		outFile.close();
	}

	//void saveToTsdfFile(const std::string& filename) const {

	//	/*std::ofstream outFile(filename, std::ios::binary);

	//	outFile << "dimensions\t" << (UINT64)getDimX() << " " << (UINT64)getDimY() << " " << (UINT64)getDimZ() << std::endl;
	//	outFile << "worldToGrid\t";
	//	for (unsigned int i = 0; i < 15; i++) {
	//		outFile << m_worldToGrid.getPointer()[i] / m_voxelSize << " ";
	//	}
	//	outFile << m_worldToGrid.getPointer()[15] << std::endl;
	//	outFile << "depthMin\t" << m_depthMin << std::endl;
	//	outFile << "depthMax\t" << m_depthMax << std::endl;
	//	outFile << "voxelSize\t" << m_voxelSize << std::endl;
	//	outFile.write((char*)getPointer(), sizeof(Voxel) * getNumElements());

	//	outFile.close();
	//	*/

	//	/*
	//	HEADER FORMAT:
	//	2 BYTES (1 HALF-UINT): grid's X dimension 
	//	2 BYTES (1 HALF-UINT): grid's Y dimension 
	//	2 BYTES (1 HALF-UINT): grid's Z dimension 
	//	2 BYTES (1 HALF-FLOAT): voxel size
	//	30 BYTES (15 HALF-FLOAT): world to grid matrix
	//	
	//	*/


	//	string binStr = "";
	//	char byteArray[8];
	//	byteArray[0] = (getDimX() >> 8) & 0xff;
	//	byteArray[1] = getDimX() & 0xff;
	//	cout << getDimX() << endl;
	//	byteArray[2] = (getDimY() >> 8) & 0xff;
	//	byteArray[3] = getDimY() & 0xff;
	//	cout << getDimY() << endl;
	//	byteArray[4] = (getDimZ() >> 8) & 0xff;
	//	byteArray[5] = getDimZ() & 0xff;
	//	cout << getDimZ() << endl;
	//	cout << m_voxelSize << endl;
	//	cout << m_worldToGrid << endl;
	//	for (unsigned int i = 0; i < 15; i++) {
	//		cout << m_worldToGrid.getPointer()[i] << " " << m_worldToGrid.getPointer()[i] / m_voxelSize << endl;
	//	}

	//	//binaryString2Bytes()
	//	std::ofstream outFile(filename, std::ios::binary | std::ios::out);
	//	outFile.write(byteArray, sizeof(byteArray));
	//	outFile.close();
	//}

	void loadFromFile(const std::string& filename) {

		std::ifstream inFile(filename, std::ios::binary);
		if (!inFile.is_open())	throw MLIB_EXCEPTION("file not found " + filename);

		std::string symbol;
		UINT64 dimX, dimY, dimZ;
		mat4f worldToGrid;
		float depthMin, depthMax, voxelSize;
		for (unsigned int i = 0; i < 5; i++) {
			std::string line;
			std::getline(inFile, line);

			std::cout << line << std::endl;
			std::stringstream ss(line);
			ss >> symbol;
			if (symbol == "dimensions") {
				ss >> dimX >> dimY >> dimZ;
			}
			else if (symbol == "worldToGrid") {
				for (unsigned int i = 0; i < 16; i++) ss >> worldToGrid.getData()[i];
			}
			else if (symbol == "depthMin") {
				ss >> depthMin;
			}
			else if (symbol == "depthMax") {
				ss >> depthMax;
			}
			else if (symbol == "voxelSize") {
				ss >> voxelSize;
			}
		}

		allocate(dimX, dimY, dimZ);
		m_worldToGrid = worldToGrid;
		// Correction below necessary since FreeSpace logic expects matrix to be already multiplied by voxelSize
		m_worldToGrid = m_worldToGrid * voxelSize;  m_worldToGrid[15] = 1.f;
		m_gridToWorld = m_worldToGrid.getInverse();
		m_voxelSize = voxelSize;
		m_depthMin = depthMin;
		m_depthMax = depthMax;

		inFile.read((char*)getData(), sizeof(Voxel)*getNumElements());
		inFile.close();
	}



	mat4f getGridToWorld() const {
		return m_gridToWorld;
	}

	mat4f getWorldToGrid() const {
		return m_worldToGrid;
	}

	vec3i worldToVoxel(const vec3f& p) const {
		return math::round((m_worldToGrid * p) / m_voxelSize);
	}

	vec3f worldToVoxelFloat(const vec3f& p) const {
		return (m_worldToGrid * p) / m_voxelSize;
	}

	vec3f voxelToWorld(const vec3i& v) const {
		return m_gridToWorld * (vec3f(v) * m_voxelSize);
	}

	float getVoxelSize() const {
		return m_voxelSize;
	}


	bool trilinearInterpolationSimpleFastFast(const vec3f& pos, float& dist, vec3uc& color) const {
		const float oSet = m_voxelSize;
		const vec3f posDual = pos - vec3f(oSet / 2.0f, oSet / 2.0f, oSet / 2.0f);
		vec3f weight = frac(worldToVoxelFloat(pos));

		dist = 0.0f;
		vec3f colorFloat = vec3f(0.0f, 0.0f, 0.0f);

		Voxel v; vec3f vColor;
		v = getVoxel(posDual + vec3f(0.0f, 0.0f, 0.0f)); if (v.weight == 0) return false;		   vColor = vec3f(v.color.x, v.color.y, v.color.z); dist += (1.0f - weight.x)*(1.0f - weight.y)*(1.0f - weight.z)*v.sdf; colorFloat += (1.0f - weight.x)*(1.0f - weight.y)*(1.0f - weight.z)*vColor;
		v = getVoxel(posDual + vec3f(oSet, 0.0f, 0.0f)); if (v.weight == 0) return false;		   vColor = vec3f(v.color.x, v.color.y, v.color.z); dist += weight.x *(1.0f - weight.y)*(1.0f - weight.z)*v.sdf; colorFloat += weight.x *(1.0f - weight.y)*(1.0f - weight.z)*vColor;
		v = getVoxel(posDual + vec3f(0.0f, oSet, 0.0f)); if (v.weight == 0) return false;		   vColor = vec3f(v.color.x, v.color.y, v.color.z); dist += (1.0f - weight.x)*	   weight.y *(1.0f - weight.z)*v.sdf; colorFloat += (1.0f - weight.x)*	   weight.y *(1.0f - weight.z)*vColor;
		v = getVoxel(posDual + vec3f(0.0f, 0.0f, oSet)); if (v.weight == 0) return false;		   vColor = vec3f(v.color.x, v.color.y, v.color.z); dist += (1.0f - weight.x)*(1.0f - weight.y)*	   weight.z *v.sdf; colorFloat += (1.0f - weight.x)*(1.0f - weight.y)*	   weight.z *vColor;
		v = getVoxel(posDual + vec3f(oSet, oSet, 0.0f)); if (v.weight == 0) return false;		   vColor = vec3f(v.color.x, v.color.y, v.color.z); dist += weight.x *	   weight.y *(1.0f - weight.z)*v.sdf; colorFloat += weight.x *	   weight.y *(1.0f - weight.z)*vColor;
		v = getVoxel(posDual + vec3f(0.0f, oSet, oSet)); if (v.weight == 0) return false;		   vColor = vec3f(v.color.x, v.color.y, v.color.z); dist += (1.0f - weight.x)*	   weight.y *	   weight.z *v.sdf; colorFloat += (1.0f - weight.x)*	   weight.y *	   weight.z *vColor;
		v = getVoxel(posDual + vec3f(oSet, 0.0f, oSet)); if (v.weight == 0) return false;		   vColor = vec3f(v.color.x, v.color.y, v.color.z); dist += weight.x *(1.0f - weight.y)*	   weight.z *v.sdf; colorFloat += weight.x *(1.0f - weight.y)*	   weight.z *vColor;
		v = getVoxel(posDual + vec3f(oSet, oSet, oSet)); if (v.weight == 0) return false;		   vColor = vec3f(v.color.x, v.color.y, v.color.z); dist += weight.x *	   weight.y *	   weight.z *v.sdf; colorFloat += weight.x *	   weight.y *	   weight.z *vColor;

		color = vec3uc(math::round(colorFloat.x), math::round(colorFloat.y), math::round(colorFloat.z));//v.color;

		return true;
	}

	vec3f getSurfaceNormal(size_t x, size_t y, size_t z) const {
		float SDFx = (*this)(x + 1, y, z).sdf - (*this)(x - 1, y, z).sdf;
		float SDFy = (*this)(x, y + 1, z).sdf - (*this)(x, y - 1, z).sdf;
		float SDFz = (*this)(x, y, z + 1).sdf - (*this)(x, y, z - 1).sdf;
		if (SDFx == 0 && SDFy == 0 && SDFz == 0) {// Don't divide by zero!
			return vec3f(SDFx, SDFy, SDFz);
		}
		else {
			return vec3f(SDFx, SDFy, SDFz).getNormalized();
		}
	}

	bool isSurfaceNormalValid(size_t x, size_t y, size_t z, unsigned int weightThreshold, float sdfThreshold) const {

		const Voxel* voxels[6];
		voxels[0] = &(*this)(x + 1, y, z);
		voxels[1] = &(*this)(x, y + 1, z);
		voxels[2] = &(*this)(x, y, z + 1);

		voxels[3] = &(*this)(x - 1, y, z);
		voxels[4] = &(*this)(x, y - 1, z);
		voxels[5] = &(*this)(x, y, z - 1);

		for (unsigned int i = 0; i < 6; i++) {
			if (voxels[i]->weight < weightThreshold && std::abs(voxels[i]->sdf) >= sdfThreshold) {
				return false;
			}
		}

		return true;
	}

	mat3f getNormalCovariance(int x, int y, int z, int radius, float weightThreshold, float sdfThreshold) const {
		// Compute neighboring surface normals
		std::vector<vec3f> normals;
		for (int k = -radius; k <= radius; k++)
			for (int j = -radius; j <= radius; j++)
				for (int i = -radius; i <= radius; i++)
					if ((*this)(x + i, y + j, z + k).weight >= weightThreshold && std::abs((*this)(x + i, y + j, z + k).sdf) < sdfThreshold)
						normals.push_back(getSurfaceNormal(x + i, y + j, z + k));

		// Find covariance matrix
		float Ixx = 0; float Ixy = 0; float Ixz = 0;
		float Iyy = 0; float Iyz = 0; float Izz = 0;
		for (int i = 0; i < normals.size(); i++) {
			Ixx = Ixx+normals[i].x*normals[i].x;
			Ixy = Ixy+normals[i].x*normals[i].y;
			Ixz = Ixz+normals[i].x*normals[i].z;
			Iyy = Iyy+normals[i].y*normals[i].y;
			Iyz = Iyz+normals[i].y*normals[i].z;
			Izz = Izz+normals[i].z*normals[i].z;
		}
		float scale = 10.0f / ((float)normals.size()); // Normalize and upscale
		return mat3f(Ixx, Ixy, Ixz, Ixy, Iyy, Iyz, Ixz, Iyz, Izz)*scale;
	}

	Voxel getVoxel(const vec3f& worldPos) const {
		vec3i voxelPos = worldToVoxel(worldPos);

		if (isValidCoordinate(voxelPos.x, voxelPos.y, voxelPos.z)) {
			return (*this)(voxelPos.x, voxelPos.y, voxelPos.z);
		}
		else {
			return Voxel();
		}
	}

	float getDepthMin() const {
		return m_depthMin;
	}

	float getDepthMax() const {
		return m_depthMax;
	}
private:



	float frac(float val) const {
		return (val - floorf(val));
	}

	vec3f frac(const vec3f& val) const {
		return vec3f(frac(val.x), frac(val.y), frac(val.z));
	}

	BoundingBox3<int> computeFrustumBounds(const mat4f& intrinsic, const mat4f& rigidTransform, unsigned int width, unsigned int height) const {

		std::vector<vec3f> cornerPoints(8);

		cornerPoints[0] = depthToSkeleton(intrinsic, 0, 0, m_depthMin);
		cornerPoints[1] = depthToSkeleton(intrinsic, width - 1, 0, m_depthMin);
		cornerPoints[2] = depthToSkeleton(intrinsic, width - 1, height - 1, m_depthMin);
		cornerPoints[3] = depthToSkeleton(intrinsic, 0, height - 1, m_depthMin);

		cornerPoints[4] = depthToSkeleton(intrinsic, 0, 0, m_depthMax);
		cornerPoints[5] = depthToSkeleton(intrinsic, width - 1, 0, m_depthMax);
		cornerPoints[6] = depthToSkeleton(intrinsic, width - 1, height - 1, m_depthMax);
		cornerPoints[7] = depthToSkeleton(intrinsic, 0, height - 1, m_depthMax);

		BoundingBox3<int> box;
		for (unsigned int i = 0; i < 8; i++) {

			vec3f pl = math::floor(rigidTransform * cornerPoints[i]);
			vec3f pu = math::ceil(rigidTransform * cornerPoints[i]);
			box.include(worldToVoxel(pl));
			box.include(worldToVoxel(pu));
		}

		box.setMin(math::max(box.getMin(), 0));
		box.setMax(math::min(box.getMax(), vec3i((int)getDimX() - 1, (int)getDimY() - 1, (int)getDimZ() - 1)));

		return box;
	}

	static vec3f depthToSkeleton(const mat4f& intrinsic, unsigned int ux, unsigned int uy, float depth) {
		if (depth == 0.0f || depth == -std::numeric_limits<float>::infinity()) return vec3f(-std::numeric_limits<float>::infinity());

		float x = ((float)ux - intrinsic(0, 2)) / intrinsic(0, 0);
		float y = ((float)uy - intrinsic(1, 2)) / intrinsic(1, 1);

		return vec3f(depth*x, depth*y, depth);
	}

	static vec3f skeletonToDepth(const mat4f& intrinsics, const vec3f& p) {

		float x = (p.x * intrinsics(0, 0)) / p.z + intrinsics(0, 2);
		float y = (p.y * intrinsics(1, 1)) / p.z + intrinsics(1, 2);

		return vec3f(x, y, p.z);
	}


	float getTruncation(float d) const {
		return m_trunaction + d * m_truncationScale;
	}
	
	float getMaxTruncation() const {
		return getTruncation(m_depthMax);
	}

	float m_voxelSize;
	mat4f m_worldToGrid;
	mat4f m_gridToWorld; //inverse of worldToGrid
	float m_depthMin;
	float m_depthMax;


	float			m_trunaction;
	float			m_truncationScale;
	unsigned int	m_weightUpdate;
};