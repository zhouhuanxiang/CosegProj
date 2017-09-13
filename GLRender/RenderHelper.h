#ifndef RENDER_HELPER_H_
#define RENDER_HELPER_H_

#include <GL/glew.h>
#include <GL/GL.h>
#include <GL/GLU.h>
class RenderHelper
{
public:
	RenderHelper(){}
	virtual void Render() = 0;
	virtual void Release() = 0;
	virtual void motion(float dx, float dy, int mouse) {}
	virtual void zoom(int dir) {}
};

#endif