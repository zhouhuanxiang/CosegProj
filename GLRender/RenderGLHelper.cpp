#include "RenderGLHelper.h"

#include <iostream>

#include <FreeImage.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

RenderGLHelper::RenderGLHelper(bool offscreen, RenderType type)
{
	render_offscreen = offscreen;
	render_type = type;
	world2cam = glm::mat4(1.0);
	projection = glm::ortho(-0.5f, 0.5f, -0.5f, 0.5f, -10.0f, 10.0f);
	lightPos = glm::vec3(-100, -100, -100);
	colorbuffer = -1;
	vertexbuffer = -1;
	normalbuffer = -1;
	uvbuffer = -1;
	materialbuffer = -1;
	elementbuffer = -1;
}

void RenderGLHelper::loadTexture(std::vector<std::string>& textureFiles)
{
  std::vector<FIBITMAP*> bitmaps;
  std::vector<FIBITMAP*> bitmaps32;
  std::vector<int> widths;
  std::vector<int> heights;
  int max_width = INT_MIN;
  int max_height = INT_MIN;

  for (int i = 0; i < textureFiles.size(); i++){
	const char* filename = textureFiles[i].c_str();
	FREE_IMAGE_FORMAT format = FreeImage_GetFileType(filename, 0);
	if (format == -1 && format == FIF_UNKNOWN){
	  std::cout << "Could not find image: " << textureFiles[i] << " - Aborting." << std::endl;
	  exit(-1);
	}
	FIBITMAP* bitmap = FreeImage_Load(format, filename);
	int bitsPerPixel = FreeImage_GetBPP(bitmap);
	FIBITMAP* bitmap32;
	if (bitsPerPixel == 32)
	  bitmap32 = bitmap;
	else
	  bitmap32 = FreeImage_ConvertTo32Bits(bitmap);

	bitmaps.push_back(bitmap);
	bitmaps32.push_back(bitmap32);
	int imageWidth = FreeImage_GetWidth(bitmap32);
	int imageHeight = FreeImage_GetHeight(bitmap32);
	widths.push_back(imageWidth);
	heights.push_back(imageHeight);
	max_width = max(max_width, imageWidth);
	max_height = max(max_height, imageHeight);
  }

  for (int i = 0; i < textureFiles.size(); i++){
	textureWidthRatio.push_back(widths[i] / static_cast<double>(max_width));
	textureHeightRatio.push_back(heights[i] / static_cast<double>(max_height));
  }

  glActiveTexture(GL_TEXTURE0);//?
  GLuint my_texture;
  glGenTextures(1, &my_texture);
  glBindTexture(GL_TEXTURE_2D_ARRAY, my_texture);
  glTexStorage3D(GL_TEXTURE_2D_ARRAY, 1, GL_RGBA8, max_width, max_height, textureFiles.size());
  for (int i = 0; i < textureFiles.size(); i++){
	glTexSubImage3D(GL_TEXTURE_2D_ARRAY, 0, 0, 0, i, widths[i], heights[i], 1, GL_RGBA, GL_UNSIGNED_BYTE, FreeImage_GetBits(bitmaps32[i]));
  }
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);

  for (int i = 0; i < textureFiles.size(); i++){
	int bitsPerPixel = FreeImage_GetBPP(bitmaps[i]);
	FreeImage_Unload(bitmaps32[i]);
	if (bitsPerPixel != 32)
	  FreeImage_Unload(bitmaps[i]);
  }
}

void RenderGLHelper::InitDevice(int imgw, int imgh, int channels, OutputType format, std::vector<std::string>& textureFiles)
{
  if (render_offscreen) {
	glGenFramebuffers(1, &FramebufferName);
	glBindFramebuffer(GL_FRAMEBUFFER, FramebufferName);

	// load texture array
	// textureFiles: file path of texture
	loadTexture(textureFiles);
	// The texture we're going to render to
	glGenTextures(1, &renderedTexture);

	// "Bind" the newly created texture : all future texture functions will modify this texture
	glBindTexture(GL_TEXTURE_2D, renderedTexture);

	GetFormat(channels, format);
	// Give an empty image to OpenGL ( the last "0" )
	glTexImage2D(GL_TEXTURE_2D, 0, InputFormat, imgw, imgh, 0, OutputFormat, dataType, 0);
	// Poor filtering. Needed !
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

	// The depth buffer
	glGenRenderbuffers(1, &depthrenderbuffer);
	glBindRenderbuffer(GL_RENDERBUFFER, depthrenderbuffer);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, imgw, imgh);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthrenderbuffer);

	// Set "renderedTexture" as our colour attachement #0
	glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, renderedTexture, 0);

	// Set the list of draw buffers.
	GLenum DrawBuffers[1] = { GL_COLOR_ATTACHMENT0 };
	glDrawBuffers(1, DrawBuffers); // "1" is the size of DrawBuffers

	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
	{
	  printf("configuration error!|n");
	  exit(0);
	}
  }
  else {
	FramebufferName = 0;
  }

  width = imgw;
  height = imgh;

  if (render_type == RENDER_INFO) {
	programID = CompileGLSLprogram(vertShader, NULL, Info_fragShader);
  }
  else if (render_type == RENDER_DEPTH) {
	programID = CompileGLSLprogram(vertShader, NULL, Depth_fragShader);
  }
  else if (render_type == RENDER_COLOR) {
	programID = CompileGLSLprogram(vertShader, NULL, Color_fragShader);
  }
  SetGLVariable("W", (float)imgw);
  SetGLVariable("H", (float)imgh);
  SetGLVariable("nearZ", 0.001f);
  SetGLVariable("farZ", 10.0F);
  SetGLVariable("textureArray", 0);//?
}

void RenderGLHelper::Render()
{
	glBindFramebuffer(GL_FRAMEBUFFER, FramebufferName);

	glClearColor(0, 0, 0, 1);
	glDisable(GL_CULL_FACE);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glViewport(0, 0, width, height);

	// if the texture is a 8 bits UI, scale the fetch with a GLSL shader
	glUseProgram(programID);
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	SetGLVariable("world2cam", world2cam);
	SetGLVariable("projection", projection);
	SetGLVariable("light_modelspace", lightPos);
	// 1rst attribute buffer : vertices
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glVertexAttribPointer(
		0,                  // attribute
		3,                  // size
		GL_FLOAT,           // type
		GL_FALSE,           // normalized?
		0,                  // stride
		(void*)0            // array buffer offset
		);
	glEnableVertexAttribArray(1);
	glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
	glVertexAttribPointer(
	  1,                  // attribute
	  2,                  // size
	  GL_FLOAT,           // type
	  GL_FALSE,           // normalized?
	  0,                  // stride
	  (void*)0            // array buffer offset
	  );
	glEnableVertexAttribArray(2);
	glBindBuffer(GL_ARRAY_BUFFER, materialbuffer);
	glVertexAttribPointer(
	  2,                  // attribute
	  1,                  // size
	  GL_FLOAT,           // type
	  GL_FALSE,           // normalized?
	  0,                  // stride
	  (void*)0            // array buffer offset
	  );
	glEnableVertexAttribArray(3);
	glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
	glVertexAttribPointer(
	  3,                  // attribute
	  3,                  // size
	  GL_FLOAT,           // type
	  GL_FALSE,           // normalized?
	  0,                  // stride
	  (void*)0            // array buffer offset
	  );
	if (colorbuffer != (GLuint)-1) {
		glEnableVertexAttribArray(4);
		glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
		glVertexAttribPointer(
			4,                  // attribute
			3,                  // size
			GL_FLOAT,           // type
			GL_FALSE,           // normalized?
			0,                  // stride
			(void*)0            // array buffer offset
			);
	}
	// Index buffer
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbuffer);

	// Draw the triangles !
	glDrawElements(
		GL_TRIANGLES,      // mode
		faceNum * 3,    // count
		GL_UNSIGNED_INT, // type
		(void*)0           // element array buffer offset
		);
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();


	glBindFramebuffer(GL_FRAMEBUFFER, 0);

}

void RenderGLHelper::LoadMesh(std::vector<glm::vec3>& positions, 
  std::vector<int>& indices, 
  std::vector<glm::vec2>& uvs,
  std::vector<float>& materials,
  std::vector<glm::vec3>& normals,
  std::vector<glm::vec3>& colors) {

  //textureWidthRatio
  //textureHeightRatio
  for (int i = 0; i < uvs.size(); i++){
	int ix = static_cast<int>(uvs[i][0]);
	int iy = static_cast<int>(uvs[i][1]);
	uvs[i][0] = (uvs[i][0] - ix) * textureWidthRatio[materials[i]];
	uvs[i][1] = (uvs[i][1] - iy) * textureWidthRatio[materials[i]];
  }

	glGenVertexArrays(1, &VertexArrayID);
	glBindVertexArray(VertexArrayID);

	glGenBuffers(1, &vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, positions.size() * sizeof(glm::vec3), &positions[0], GL_STATIC_DRAW);

	glGenBuffers(1, &uvbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
	glBufferData(GL_ARRAY_BUFFER, uvs.size() * sizeof(glm::vec2), &uvs[0], GL_STATIC_DRAW);

	glGenBuffers(1, &materialbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, materialbuffer);
	glBufferData(GL_ARRAY_BUFFER, materials.size() * sizeof(float), &materials[0], GL_STATIC_DRAW);

	glGenBuffers(1, &normalbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
	glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(glm::vec3), &normals[0], GL_STATIC_DRAW);

	if (colors.size()) {
		glGenBuffers(1, &colorbuffer);
		glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
		glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(glm::vec3), &colors[0], GL_STATIC_DRAW);
	}

	// Generate a buffer for the indices as well
	glGenBuffers(1, &elementbuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(int), &indices[0], GL_STATIC_DRAW);
	faceNum = positions.size() / 3;
}

void RenderGLHelper::GrabCPU(void* data) {
	glBindFramebuffer(GL_FRAMEBUFFER, FramebufferName);
	glPixelStorei(GL_PACK_ALIGNMENT, 1);
	glReadPixels(0, 0, width, height,
		OutputFormat, dataType, data);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

GLuint RenderGLHelper::CompileGLSLprogram(const char *vertex_shader_src, const char* geometry_shader_src, const char *fragment_shader_src)
{
	GLuint v, g, f, p = 0;

	p = glCreateProgram();

	if (vertex_shader_src)
	{
		v = glCreateShader(GL_VERTEX_SHADER);
		glShaderSource(v, 1, &vertex_shader_src, NULL);
		glCompileShader(v);

		// check if shader compiled
		GLint compiled = 0;
		glGetShaderiv(v, GL_COMPILE_STATUS, &compiled);

		if (!compiled)
		{
			//#ifdef NV_REPORT_COMPILE_ERRORS
			char temp[256] = "";
			glGetShaderInfoLog(v, 256, NULL, temp);
			printf("Vtx Compile failed:\n%s\n", temp);
			//#endif
			glDeleteShader(v);
			return 0;
		}
		else
		{
			glAttachShader(p, v);
		}
	}

	if (geometry_shader_src)
	{
		g = glCreateShader(GL_GEOMETRY_SHADER);
		glShaderSource(g, 1, &geometry_shader_src, NULL);
		glCompileShader(g);

		// check if shader compiled
		GLint compiled = 0;
		glGetShaderiv(v, GL_COMPILE_STATUS, &compiled);

		if (!compiled)
		{
			//#ifdef NV_REPORT_COMPILE_ERRORS
			char temp[256] = "";
			glGetShaderInfoLog(g, 256, NULL, temp);
			printf("Vtx Compile failed:\n%s\n", temp);
			//#endif
			glDeleteShader(g);
			return 0;
		}
		else
		{
			glAttachShader(p, g);
		}
	}
	
	if (fragment_shader_src)
	{
		f = glCreateShader(GL_FRAGMENT_SHADER);
		glShaderSource(f, 1, &fragment_shader_src, NULL);
		glCompileShader(f);

		// check if shader compiled
		GLint compiled = 0;
		glGetShaderiv(f, GL_COMPILE_STATUS, &compiled);

		if (!compiled)
		{
			//#ifdef NV_REPORT_COMPILE_ERRORS
			char temp[256] = "";
			glGetShaderInfoLog(f, 256, NULL, temp);
			printf("frag Compile failed:\n%s\n", temp);
			//#endif
			glDeleteShader(f);
			return 0;
		}
		else
		{
			glAttachShader(p, f);
		}
	}

	glLinkProgram(p);

	int infologLength = 0;
	int charsWritten = 0;

	glGetProgramiv(p, GL_INFO_LOG_LENGTH, (GLint *)&infologLength);

	if (infologLength > 0)
	{
		char *infoLog = (char *)malloc(infologLength);
		glGetProgramInfoLog(p, infologLength, (GLsizei *)&charsWritten, infoLog);
		//		printf("Shader compilation error: %s\n", infoLog);
		free(infoLog);
	}

	return p;
}

void RenderGLHelper::GetFormat(int channels, OutputType format) {
	if (channels == 1) {
		if (format == FLOAT)
			InputFormat = GL_R32F, dataType = GL_FLOAT;
		else
			InputFormat = GL_R, dataType = GL_UNSIGNED_BYTE;
		OutputFormat = GL_R;
	}
	else if (channels == 2) {
		if (format == FLOAT)
			InputFormat = GL_RG32F, dataType = GL_FLOAT;
		else
			InputFormat = GL_RG, dataType = GL_UNSIGNED_BYTE;
		OutputFormat = GL_RG;
	}
	else if (channels == 3) {
		if (format == FLOAT)
			InputFormat = GL_RGB32F, dataType = GL_FLOAT;
		else
			InputFormat = GL_RGB, dataType = GL_UNSIGNED_BYTE;
		OutputFormat = GL_RGB;
	}
	else if (channels == 4) {
		if (format == FLOAT)
			InputFormat = GL_RGBA32F, dataType = GL_FLOAT;
		else
			InputFormat = GL_RGBA, dataType = GL_UNSIGNED_BYTE;
		OutputFormat = GL_RGBA;
	}
}

void RenderGLHelper::RetrieveGLVariable(std::string variable)
{
	if (variableMap.count(variable) == 0) {
		GLuint id = glGetUniformLocation(programID, variable.c_str());
		if (id == (GLuint)-1) {
			printf("variable %s not found...\n", variable.c_str());
		}
		variableMap[variable] = id;
	}
}

void RenderGLHelper::SetGLVariable(std::string variable, int s) {
	if (variableMap.count(variable) == 0) {
		RetrieveGLVariable(variable);
	}
	glUseProgram(programID);
	glUniform1i(variableMap[std::string(variable)], s);
}

void RenderGLHelper::SetGLVariable(std::string variable, float s) {
	if (variableMap.count(variable) == 0) {
		RetrieveGLVariable(variable);
	}
	glUseProgram(programID);
	glUniform1f(variableMap[std::string(variable)], s);
}

void RenderGLHelper::SetGLVariable(std::string variable, glm::vec3& s) {
	if (variableMap.count(variable) == 0) {
		RetrieveGLVariable(variable);
	}
	glUseProgram(programID);
	glUniform3f(variableMap[std::string(variable)], s[0], s[1], s[2]);
}

void RenderGLHelper::SetGLVariable(std::string variable, glm::mat4& s) {
	if (variableMap.count(variable) == 0) {
		RetrieveGLVariable(variable);
	}
	glUseProgram(programID);
	glUniformMatrix4fv(variableMap[std::string(variable)], 1, GL_FALSE, &s[0][0]);
}

void RenderGLHelper::SetGLVariable(std::string variable, std::vector<glm::ivec3>& val) {
	if (variableMap.count(variable) == 0) {
		RetrieveGLVariable(variable);
	}
	glUseProgram(programID);
	glUniform3iv(variableMap[std::string(variable)], val.size(), (int*)val.data());
}

void RenderGLHelper::SetGLVariable(std::string variable, std::vector<glm::mat4>& s) {
	if (variableMap.count(variable) == 0) {
		RetrieveGLVariable(variable);
	}
	glUseProgram(programID);
	glUniformMatrix4fv(variableMap[std::string(variable)], s.size(), GL_FALSE, (float*)s.data());
}

void RenderGLHelper::SetIntrinsic(float fx, float fy, float cx, float cy) {
	SetGLVariable("fx", fx);
	SetGLVariable("fy", fy);
	SetGLVariable("cx", cx);
	SetGLVariable("cy", cy);
}

void RenderGLHelper::Release()
{
	if (vertexbuffer != (GLuint)-1)
		glDeleteBuffers(1, &vertexbuffer);
	if (uvbuffer != (GLuint)-1)
	  glDeleteBuffers(1, &uvbuffer);
	if (materialbuffer != (GLuint)-1)
	  glDeleteBuffers(1, &materialbuffer);
	if (colorbuffer != (GLuint)-1)
		glDeleteBuffers(1, &colorbuffer);
	if (elementbuffer != (GLuint)-1)
		glDeleteBuffers(1, &elementbuffer);
	if (normalbuffer != (GLuint)-1)
	  glDeleteBuffers(1, &normalbuffer);
	glDeleteProgram(programID);

	glDeleteFramebuffers(1, &FramebufferName);
	glDeleteTextures(1, &renderedTexture);
	glDeleteRenderbuffers(1, &depthrenderbuffer);
	glDeleteVertexArrays(1, &VertexArrayID);

	textureWidthRatio.clear();
	textureHeightRatio.clear();
}