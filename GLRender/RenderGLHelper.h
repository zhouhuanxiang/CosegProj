#ifndef RENDER_GL_HELPER_H_
#define RENDER_GL_HELPER_H_

#include <map>
#include <vector>
#include <iostream>

#include <glm/glm.hpp>

#include "RenderHelper.h"
#include "shader_src.h"
class RenderGLHelper : public RenderHelper
{
public:
	enum RenderType { RENDER_INFO = 0, RENDER_DEPTH = 1, RENDER_COLOR = 2 };
	enum OutputType { UNSIGNED_CHAR = 0, FLOAT = 1 };
	RenderGLHelper(bool offscreen = true, RenderType type = RENDER_COLOR);
	void InitDevice(int imgw, int imgh, int channels, OutputType format, std::vector<std::string>& textureFiles);
	void loadTexture(std::vector<std::string>& textureFiles);
	void LoadMesh(std::vector<glm::vec3>& positions, std::vector<int>& indices, std::vector<glm::vec2>& uvs, std::vector<float>& materials, 
	  std::vector<glm::vec3>& normals, std::vector<glm::vec3>& colors = std::vector<glm::vec3>());
	void GetFormat(int channels, OutputType);
	void Render();
	void GrabCPU(void* data);
	void RetrieveGLVariable(std::string variable);
	void SetGLVariable(std::string variable, int val);
	void SetGLVariable(std::string variable, float val);
	void SetGLVariable(std::string variable, glm::vec3& val);
	void SetGLVariable(std::string variable, glm::mat4& val);
	void SetGLVariable(std::string variable, std::vector<glm::mat4>& val);
	void SetGLVariable(std::string variable, std::vector<glm::ivec3>& val);

	void SetIntrinsic(float fx, float fy, float cx, float cy);
	void Release();
	static GLuint CompileGLSLprogram(const char *vertex_shader_src, const char* geometry_shader_src, const char *fragment_shader_src);

	std::map<std::string, GLuint> variableMap;
	RenderType render_type;
	bool render_offscreen;
	int output_channels;
	int output_type;

	GLuint InputFormat, OutputFormat, dataType;
	GLuint FramebufferName;
	GLuint renderedTexture;
	GLuint depthrenderbuffer;
	GLuint VertexArrayID;
	GLuint vertexbuffer;
	GLuint normalbuffer;
	GLuint colorbuffer;
	GLuint uvbuffer;
	GLuint materialbuffer;
	GLuint elementbuffer;
	GLuint programID;
	int width, height, faceNum;
	glm::mat4 world2cam;
	glm::vec3 lightPos;
	glm::mat4 projection;
	std::vector<double> textureWidthRatio;
	std::vector<double> textureHeightRatio;

};
#endif