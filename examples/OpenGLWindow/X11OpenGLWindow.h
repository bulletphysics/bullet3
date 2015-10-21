#ifndef X11_OPENGL_WINDOW_H
#define X11_OPENGL_WINDOW_H

#define b3gDefaultOpenGLWindow X11OpenGLWindow

#include "../CommonInterfaces/CommonWindowInterface.h"

class X11OpenGLWindow : public CommonWindowInterface
{

	struct InternalData2*   m_data;
        bool m_OpenGLInitialized;
	bool m_requestedExit;

protected:

        void enableOpenGL();

        void disableOpenGL();

        void pumpMessage();

        int getAsciiCodeFromVirtualKeycode(int orgCode);

public:

        X11OpenGLWindow();

        virtual ~X11OpenGLWindow();

        virtual void    createWindow(const b3gWindowConstructionInfo& ci);

        virtual void    closeWindow();

        virtual void    startRendering();

        virtual void    renderAllObjects();

        virtual void    endRendering();

        virtual float getRetinaScale() const {return 1.f;}
	virtual void setAllowRetina(bool /*allowRetina*/) {};

	virtual void    runMainLoop();
        virtual float   getTimeInSeconds();

        virtual bool    requestedExit() const;
        virtual void    setRequestExit() ;

	virtual bool isModifierKeyPressed(int key);

        virtual void setMouseMoveCallback(b3MouseMoveCallback   mouseCallback);
        virtual void setMouseButtonCallback(b3MouseButtonCallback       mouseCallback);
        virtual void setResizeCallback(b3ResizeCallback resizeCallback);
        virtual void setWheelCallback(b3WheelCallback wheelCallback);
        virtual void setKeyboardCallback( b3KeyboardCallback    keyboardCallback);
        
		virtual b3MouseMoveCallback getMouseMoveCallback();
		virtual b3MouseButtonCallback getMouseButtonCallback();
		virtual b3ResizeCallback getResizeCallback();
		virtual b3WheelCallback getWheelCallback();
		virtual b3KeyboardCallback      getKeyboardCallback();

        virtual void setRenderCallback( b3RenderCallback renderCallback);

        virtual void setWindowTitle(const char* title);

	int fileOpenDialog(char* filename, int maxNameLength);
};


#endif

