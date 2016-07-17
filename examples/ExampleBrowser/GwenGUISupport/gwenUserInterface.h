#ifndef _GWEN_USER_INTERFACE_H
#define _GWEN_USER_INTERFACE_H

struct GwenInternalData;

typedef void (*b3ComboBoxCallback) (int combobox, const char* item);
typedef void (*b3ToggleButtonCallback)(int button, int state);
typedef void (*b3FileOpenCallback)();
typedef void (*b3QuitCallback)();

namespace Gwen
{
	namespace Renderer
	{
		class Base;
	};
};
class GwenUserInterface
{
	GwenInternalData*	m_data;

	public:
		
		GwenUserInterface();
		
		virtual ~GwenUserInterface();
		
		void	init(int width, int height,Gwen::Renderer::Base* gwenRenderer,float retinaScale);
		void	exit();
		void	setFocus();
		void	forceUpdateScrollBars();
		
		void	draw(int width, int height);

		void	resize(int width, int height);
				
		bool	mouseMoveCallback( float x, float y);
		bool	mouseButtonCallback(int button, int state, float x, float y);
		bool	keyboardCallback(int key, int state);


		void	setToggleButtonCallback(b3ToggleButtonCallback callback);
		b3ToggleButtonCallback getToggleButtonCallback();

		void	registerToggleButton2(int buttonId, const char* name);

		void	setComboBoxCallback(b3ComboBoxCallback callback);
		b3ComboBoxCallback getComboBoxCallback();
		void	registerComboBox2(int buttonId, int numItems, const char** items, int startItem = 0);
		
		void	setStatusBarMessage(const char* message, bool isLeft=true);

		void	textOutput(const char* msg);
		void	setExampleDescription(const char* msg);
		

        void    registerFileOpenCallback(b3FileOpenCallback callback);
        void    registerQuitCallback(b3QuitCallback callback);
    
		GwenInternalData* getInternalData()
		{
			return m_data;
		}

};



#endif //_GWEN_USER_INTERFACE_H

