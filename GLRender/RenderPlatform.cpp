#include "RenderPlatform.h"
#include <GL/glew.h>
#include <iostream>
#include <GL/freeglut.h>

#define REFRESH_DELAY     10 //ms

RenderPlatform* RenderPlatform::instance = 0;
RenderPlatform::RenderPlatform(int w, int h)
{
	width = w;
	height = h;
	fpsCount = 0;
	fpsLimit = 1;
	renderHelper = 0;
	mouse_state = 0;
}

void RenderPlatform::InitDevice(int *argc, char **argv)
{
	glutInit(argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_ALPHA | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(width, height);
	iGLUTWindowHandle = glutCreateWindow("CUDA OpenGL post-processing");

	// initialize necessary OpenGL extensions
	glewInit();

	if (!glewIsSupported(
		"GL_VERSION_2_0 "
		"GL_ARB_pixel_buffer_object "
		"GL_EXT_framebuffer_object "
		))
	{
		printf("ERROR: Support for necessary OpenGL extensions missing.");
		fflush(stderr);
		exit(0);
	}
	
	if (!instance) {
		instance = this;
	}
	else {
		printf("already initialized!\n");
	}
	glutDisplayFunc(displayCallback);
	glutKeyboardFunc(keyboardCallback);
	glutMouseFunc(mouseCallback);
	glutMotionFunc(motionCallback);
	glutReshapeFunc(reshapeCallback);
	glutTimerFunc(REFRESH_DELAY, timerEventCallback, 0);
	glutMouseWheelFunc(mouseWheelCallback);
}

void RenderPlatform::ReigsterRenderHelper(RenderHelper* helper)
{
	renderHelper = helper;
}

void RenderPlatform::StartMainloop()
{
	glutMainLoop();
}

void RenderPlatform::mouseWheel(int button, int dir, int x, int y)
{
	renderHelper->zoom(dir);

	return;
}

void RenderPlatform::keyboard(unsigned char key, int /*x*/, int /*y*/)
{
	switch (key)
	{
	case (27) :
		renderHelper->Release();
		if (iGLUTWindowHandle)
		{
			glutDestroyWindow(iGLUTWindowHandle);
		}
		exit(0);
		break;
	}
}

void RenderPlatform::display()
{
	if (renderHelper) {
		renderHelper->Render();
	}

	// flip backbuffer
	glutSwapBuffers();

	if (++fpsCount == fpsLimit)
	{
//		glutSetWindowTitle("demo");
		//printf("%s\n", cTitle);
	}
}

void RenderPlatform::mouse(int button, int state, int x, int y)
{
	switch (button)
	{
	case GLUT_LEFT_BUTTON:
		if (state == GLUT_DOWN) {
			mouse_state = 1;
		}
		else if (state == GLUT_UP) {
			mouse_state = 0;
		}
		break;
	case GLUT_RIGHT_BUTTON:
		if (state == GLUT_DOWN) {
			mouse_state = 3;
		}
		else if (state == GLUT_UP) {
			mouse_state = 4;
		}
	}
}

void RenderPlatform::motion(int x, int y) {
	if (mouse_state % 2 == 1) {
		mouse_state += 1;
		mouse_x = x;
		mouse_y = y;
		return;
	}
	if (mouse_state == 2 || mouse_state == 4) {
		float dx = (x - mouse_x) / (float)width;
		float dy = (y - mouse_y) / (float)height;
		mouse_x = x;
		mouse_y = y;
		if (renderHelper)
			renderHelper->motion(dx, dy, mouse_state / 2 - 1);
	}
}

void RenderPlatform::reshape(int w, int h)
{
	width = w;
	height = h;
}

void RenderPlatform::timerEvent(int value)
{
	glutPostRedisplay();
	glutTimerFunc(REFRESH_DELAY, timerEventCallback, 0);
}

