#ifndef X11_OPENGL_WINDOW_H
#define X11_OPENGL_WINDOW_H

#define btgDefaultOpenGLWindow X11OpenGLWindow

#include "btgWindowInterface.h"

class X11OpenGLWindow : public btgWindowInterface
{

	struct InternalData2*   m_data;
        bool m_OpenGLInitialized;

protected:

        void enableOpenGL();

        void disableOpenGL();

        void pumpMessage();

        int getAsciiCodeFromVirtualKeycode(int orgCode);

public:

        X11OpenGLWindow();

        virtual ~X11OpenGLWindow();

        virtual void    createWindow(const btgWindowConstructionInfo& ci);

        virtual void    closeWindow();

        virtual void    startRendering();

        virtual void    renderAllObjects();

        virtual void    endRendering();

        virtual float getRetinaScale() const {return 1.f;}


	virtual void    runMainLoop();
        virtual float   getTimeInSeconds();

        virtual bool    requestedExit() const;
        virtual void    setRequestExit() ;


        virtual void setMouseMoveCallback(btMouseMoveCallback   mouseCallback);
        virtual void setMouseButtonCallback(btMouseButtonCallback       mouseCallback);
        virtual void setResizeCallback(btResizeCallback resizeCallback);
        virtual void setWheelCallback(btWheelCallback wheelCallback);
        virtual void setKeyboardCallback( btKeyboardCallback    keyboardCallback);

        virtual void setRenderCallback( btRenderCallback renderCallback);

        virtual void setWindowTitle(const char* title);

};


#endif

