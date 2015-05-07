#ifndef MAC_OPENGL_WINDOW_H
#define MAC_OPENGL_WINDOW_H

#include "../CommonInterfaces/CommonWindowInterface.h"

#define b3gDefaultOpenGLWindow MacOpenGLWindow

class MacOpenGLWindow : public CommonWindowInterface
{
    struct MacOpenGLWindowInternalData* m_internalData;
    float m_mouseX;
    float m_mouseY;
   int m_modifierFlags;
   
    b3MouseButtonCallback m_mouseButtonCallback;
    b3MouseMoveCallback m_mouseMoveCallback;
    b3WheelCallback m_wheelCallback;
    b3KeyboardCallback m_keyboardCallback;
	b3RenderCallback m_renderCallback;
	
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

     virtual bool    isModifierKeyPressed(int key);
    
    void setMouseButtonCallback(b3MouseButtonCallback	mouseCallback)
    {
        m_mouseButtonCallback = mouseCallback;
    }

    void setMouseMoveCallback(b3MouseMoveCallback	mouseCallback)
    {
        m_mouseMoveCallback = mouseCallback;
    }
    
    void setResizeCallback(b3ResizeCallback resizeCallback);
   
 
	void setKeyboardCallback( b3KeyboardCallback	keyboardCallback)
    {
        m_keyboardCallback = keyboardCallback;
    }

	virtual b3MouseMoveCallback getMouseMoveCallback()
	{
		return m_mouseMoveCallback;
	}
	virtual b3MouseButtonCallback getMouseButtonCallback()
	{
		return m_mouseButtonCallback;
	}
	virtual b3ResizeCallback getResizeCallback();

	virtual b3WheelCallback getWheelCallback()
	{
		return m_wheelCallback;
	}

    b3KeyboardCallback getKeyboardCallback()
	{
		return m_keyboardCallback;
	}
    
	void setWheelCallback (b3WheelCallback wheelCallback)
    {
        m_wheelCallback = wheelCallback;
    }

    float getRetinaScale() const
    {
        return m_retinaScaleFactor;
    }
	
	virtual	void	createWindow(const b3gWindowConstructionInfo& ci);
	
	virtual	float	getTimeInSeconds();
	

	
	virtual void setRenderCallback( b3RenderCallback renderCallback);
	
	virtual void setWindowTitle(const char* title);

    int fileOpenDialog(char* filename, int maxNameLength);

};


#endif

