#ifndef RENDER_PLATFORM_H_
#define RENDER_PLATFORM_H_

#include "RenderHelper.h"

class RenderPlatform
{
public:
	RenderPlatform(int width, int height);
	void InitDevice(int *argc, char **argv);
	void ReigsterRenderHelper(RenderHelper* helper);
	void StartMainloop();
	
	void display();
	void keyboard(unsigned char, int, int);
	void mouse(int button, int state, int x, int y);
	void motion(int x, int y);
	void reshape(int w, int h);
	void timerEvent(int t);
	void mouseWheel(int button, int dir, int x, int y);
	static RenderPlatform* instance;
	static inline void displayCallback() {
		instance->display();
	}
	static inline void keyboardCallback(unsigned char a, int b, int c) {
		instance->keyboard(a, b, c);
	}
	static inline void mouseCallback(int button, int state, int x, int y) {
		instance->mouse(button, state, x, y);
	}
	static inline void motionCallback(int x, int y) {
		instance->motion(x, y);
	}
	static inline void reshapeCallback(int w, int h) {
		instance->reshape(w, h);
	}
	static inline void timerEventCallback(int t) {
		instance->timerEvent(t);
	}
	static inline void mouseWheelCallback(int button, int dir, int x, int y) {
		instance->mouseWheel(button, dir, x, y);
	}

	RenderHelper* renderHelper;
	int width, height;
	int iGLUTWindowHandle;
	int fpsCount;
	int fpsLimit;
	int mouse_state, mouse_x, mouse_y;
};
#endif