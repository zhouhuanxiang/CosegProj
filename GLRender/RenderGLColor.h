#ifndef RENDER_GL_COLOR
#define RENDER_GL_COLOR

#include <conio.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <glm/glm.hpp>
#include "RenderGLHelper.h"

class RenderGLColor{
public:
	void init(int w, int h, std::vector<std::string>& textureFiles)
	{
		/*width = cam_info.width;
		height = cam_info.height;*/
		width = w;
		height = h;
		glColorHelper_.InitDevice(width, height, 3, RenderGLHelper::UNSIGNED_CHAR, textureFiles);
		glColorHelper_.SetIntrinsic(580, 580, w / 2.0, h / 2.0);
	}

	void LoadMesh(std::vector<glm::vec3>& positions,
		std::vector<int>& indices,
		std::vector<glm::vec2>& uvs,
		std::vector<float>& materials,
		std::vector<glm::vec3>& normals){
		// already indexed
		glColorHelper_.LoadMesh(positions, indices, uvs, materials, normals);
	}

	void RenderColorImg(cv::Mat& color){
		glColorHelper_.Render();
		glColorHelper_.GrabCPU(color.data);
		glColorHelper_.Release();
	}

private:
	RenderGLHelper glColorHelper_;
	int width, height;
};

#endif