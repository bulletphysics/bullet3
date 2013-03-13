#ifndef BTG_WINDOW_INTERFACE_H
#define BTG_WINDOW_INTERFACE_H


typedef void (*btWheelCallback)(float deltax, float deltay);
typedef void (*btResizeCallback)( float width, float height);
typedef void (*btMouseMoveCallback)( float x, float y);
typedef void (*btMouseButtonCallback)(int button, int state, float x, float y);
typedef void (*btKeyboardCallback)(int keycode, int state);
typedef void (*btRenderCallback) ();

enum {
	BTG_ESCAPE = 27,
	BTG_F1 = 0xff00,
	BTG_F2,
	BTG_F3,
	BTG_F4,
	BTG_F5,
	BTG_F6,
	BTG_F7,
	BTG_F8,
	BTG_F9,
	BTG_F10,
	BTG_F11,
	BTG_F12,
	BTG_F13,
	BTG_F14,
	BTG_F15,
	BTG_LEFT_ARROW,
	BTG_RIGHT_ARROW,
	BTG_UP_ARROW,
	BTG_DOWN_ARROW,
	BTG_PAGE_UP,
	BTG_PAGE_DOWN,
	BTG_END,
	BTG_HOME,
	BTG_INSERT,
	BTG_DELETE
};

struct btgWindowConstructionInfo
{
		int m_width;
		int m_height;
		bool m_fullscreen;
		int m_colorBitsPerPixel;
		void* m_windowHandle;
		const char* m_title;
		int m_openglVersion;

		btgWindowConstructionInfo(int width=1024, int height=768)
		:m_width(width),
			m_height(height),
			m_fullscreen(false),
			m_colorBitsPerPixel(32),
			m_windowHandle(0),
			m_title("title"),
			m_openglVersion(3)
			{
			}
};


class btgWindowInterface
{
	public:
		
		virtual ~btgWindowInterface()
		{
		}

		virtual void	createDefaultWindow(int width, int height, const char* title)
		{
			btgWindowConstructionInfo ci(width,height);
			ci.m_title = title;
			createWindow(ci);
		}

		virtual	void	createWindow(const btgWindowConstructionInfo& ci)=0;
		
		virtual void	closeWindow()=0;

		virtual void	runMainLoop()=0;
		virtual	float	getTimeInSeconds()=0;

		virtual bool	requestedExit() const = 0;
		virtual	void	setRequestExit() = 0;

		virtual	void	startRendering()=0;

		virtual	void	endRendering()=0;

		
		virtual void setMouseMoveCallback(btMouseMoveCallback	mouseCallback)=0;
		virtual void setMouseButtonCallback(btMouseButtonCallback	mouseCallback)=0;
		virtual void setResizeCallback(btResizeCallback	resizeCallback)=0;
		virtual void setWheelCallback(btWheelCallback wheelCallback)=0;
		virtual void setKeyboardCallback( btKeyboardCallback	keyboardCallback)=0;

		virtual void setRenderCallback( btRenderCallback renderCallback) = 0;
	
		virtual void setWindowTitle(const char* title)=0;

		virtual	float	getRetinaScale() const =0;

};

#endif //BTG_WINDOW_INTERFACE_H