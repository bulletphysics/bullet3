#ifndef MAC_OPENGL_WINDOW_H
#define MAC_OPENGL_WINDOW_H

#include "btgWindowInterface.h"

#define btgDefaultOpenGLWindow MacOpenGLWindow

class MacOpenGLWindow : public btgWindowInterface
{
    struct MacOpenGLWindowInternalData* m_internalData;
    float m_mouseX;
    float m_mouseY;

   
    btMouseButtonCallback m_mouseButtonCallback;
    btMouseMoveCallback m_mouseMoveCallback;
    btWheelCallback m_wheelCallback;
    btKeyboardCallback m_keyboardCallback;
	btRenderCallback m_renderCallback;
	
    float m_retinaScaleFactor;
public:
    
    MacOpenGLWindow();
    virtual ~MacOpenGLWindow();
    
    void init(int width, int height, const char* windowTitle);

    void closeWindow();
    
    void startRendering();
    
    void endRendering();//swap buffers
    
  	virtual bool	requestedExit() const;

	virtual	void	setRequestExit();
    
    void getMouseCoordinates(int& x, int& y);
    
    void runMainLoop();
    
    void setMouseButtonCallback(btMouseButtonCallback	mouseCallback)
    {
        m_mouseButtonCallback = mouseCallback;
    }

    void setMouseMoveCallback(btMouseMoveCallback	mouseCallback)
    {
        m_mouseMoveCallback = mouseCallback;
    }
    
    void setResizeCallback(btResizeCallback resizeCallback);
    
	void setKeyboardCallback( btKeyboardCallback	keyboardCallback)
    {
        m_keyboardCallback = keyboardCallback;
    }
    
    void setWheelCallback (btWheelCallback wheelCallback)
    {
        m_wheelCallback = wheelCallback;
    }

    float getRetinaScale() const
    {
        return m_retinaScaleFactor;
    }
	
	virtual	void	createWindow(const btgWindowConstructionInfo& ci);
	
	virtual	float	getTimeInSeconds();
	

	
	virtual void setRenderCallback( btRenderCallback renderCallback);
	
	virtual void setWindowTitle(const char* title);

	

};


#endif

