#ifndef MAC_OPENGL_WINDOW_OBJC_H
#define MAC_OPENGL_WINDOW_OBJC_H

struct MacOpenGLWindowInternalData;

#include "../CommonInterfaces/CommonCallbacks.h"

struct MacWindowConstructionInfo
{
		int m_width;
		int m_height;
		int m_fullscreen;
		int m_colorBitsPerPixel;
		void* m_windowHandle;
		const char* m_title;
		int m_openglVersion;
		int m_allowRetina;
};


enum
{
	MY_MAC_ALTKEY=1,
	MY_MAC_SHIFTKEY=2,
	MY_MAC_CONTROL_KEY=4
};

#ifdef __cplusplus
extern "C" {
#endif

struct MacOpenGLWindowInternalData* Mac_createData();
void Mac_destroyData(struct MacOpenGLWindowInternalData* data);

int Mac_createWindow(struct MacOpenGLWindowInternalData* m_internalData,struct MacWindowConstructionInfo* ci);

void Mac_setWindowTitle(struct MacOpenGLWindowInternalData* data, const char* windowTitle);
int Mac_updateWindow(struct MacOpenGLWindowInternalData* m_internalData);
void Mac_swapBuffer(struct MacOpenGLWindowInternalData* m_internalData);
int Mac_requestedExit(struct MacOpenGLWindowInternalData* m_internalData);
void Mac_setRequestExit(struct MacOpenGLWindowInternalData* m_internalData);
float Mac_getRetinaScale(struct MacOpenGLWindowInternalData* m_internalData);
void	Mac_setAllowRetina(struct MacOpenGLWindowInternalData* m_internalData, int allow);	

int   Mac_getWidth(struct MacOpenGLWindowInternalData* m_internalData);
int   Mac_getHeight(struct MacOpenGLWindowInternalData* m_internalData);

int Mac_fileOpenDialog(char* filename, int maxNameLength);

void Mac_setKeyboardCallback( struct MacOpenGLWindowInternalData* m_internalData, b3KeyboardCallback	keyboardCallback);
b3KeyboardCallback Mac_getKeyboardCallback(struct MacOpenGLWindowInternalData* m_internalData);
int Mac_isModifierKeyPressed(struct MacOpenGLWindowInternalData* m_internalData, int key);

void Mac_setMouseButtonCallback(struct MacOpenGLWindowInternalData* m_internalData, b3MouseButtonCallback	mouseCallback);
b3MouseButtonCallback Mac_getMouseButtonCallback(struct MacOpenGLWindowInternalData* m_internalData);
void Mac_getMouseCoordinates(struct MacOpenGLWindowInternalData* m_internalData, int* xPtr, int* yPtr);
void Mac_setMouseMoveCallback(struct MacOpenGLWindowInternalData* m_internalData, b3MouseMoveCallback	mouseCallback);
b3MouseMoveCallback Mac_getMouseMoveCallback(struct MacOpenGLWindowInternalData* m_internalData);

void Mac_setWheelCallback(struct MacOpenGLWindowInternalData* m_internalData, b3WheelCallback wheelCallback);
b3WheelCallback Mac_getWheelCallback(struct MacOpenGLWindowInternalData* m_internalData);

void Mac_setResizeCallback(struct MacOpenGLWindowInternalData* m_internalData, b3ResizeCallback resizeCallback);
b3ResizeCallback Mac_getResizeCallback(struct MacOpenGLWindowInternalData* m_internalData);


//void Mac_setRenderCallback(struct MacOpenGLWindowInternalData* m_internalData, b3RenderCallback renderCallback);
	
#ifdef __cplusplus
}
#endif



#endif //MAC_OPENGL_WINDOW_OBJC_H


