/*
	GWEN
	Copyright (c) 2011 Facepunch Studios
	See license in Gwen.h
*/
#ifndef GWEN_INPUT_SDL13_H
#define GWEN_INPUT_SDL13_H

#include "Gwen/InputHandler.h"
#include "Gwen/Gwen.h"
#include "Gwen/Controls/Canvas.h"

#ifdef _WIN32
#define UCS_STRING "UCS-2"
#else
#define UCS_STRING "UCS-4"
#endif

namespace Gwen
{
namespace Input
{
class SDL13
{
public:
	SDL13()
	{
		m_Canvas = NULL;
	}

	void Initialize(Gwen::Controls::Canvas* c)
	{
		m_Canvas = c;
	}

	bool ProcessEvent(SDL_Event* Event)
	{
		if (!m_Canvas) return false;

		switch (Event->type)
		{
			case SDL_KEYUP:
			case SDL_KEYDOWN:
			{
				SDL_KeyboardEvent* E = &Event->key;
				int iKey = -1;
				SDL_scancode scancode = E->keysym.scancode;
				switch (scancode)
				{
					case SDL_SCANCODE_RETURN:
						iKey = Gwen::Key::Return;
						break;
					case SDL_SCANCODE_BACKSPACE:
						iKey = Gwen::Key::Backspace;
						break;
					case SDL_SCANCODE_DELETE:
						iKey = Gwen::Key::Delete;
						break;
					case SDL_SCANCODE_LEFT:
						iKey = Gwen::Key::Left;
						break;
					case SDL_SCANCODE_RIGHT:
						iKey = Gwen::Key::Right;
						break;
					case SDL_SCANCODE_LSHIFT:
						iKey = Gwen::Key::Shift;
						break;
					case SDL_SCANCODE_RSHIFT:
						iKey = Gwen::Key::Shift;
						break;
					case SDL_SCANCODE_TAB:
						iKey = Gwen::Key::Tab;
						break;
					case SDL_SCANCODE_SPACE:
						iKey = Gwen::Key::Space;
						break;
					case SDL_SCANCODE_HOME:
						iKey = Gwen::Key::Home;
						break;
					case SDL_SCANCODE_END:
						iKey = Gwen::Key::End;
						break;
					case SDL_SCANCODE_LCTRL:
						iKey = Gwen::Key::Control;
						break;
					case SDL_SCANCODE_RCTRL:
						iKey = Gwen::Key::Control;
						break;
					case SDL_SCANCODE_UP:
						iKey = Gwen::Key::Up;
						break;
					case SDL_SCANCODE_DOWN:
						iKey = Gwen::Key::Down;
						break;
					case SDL_SCANCODE_ESCAPE:
						iKey = Gwen::Key::Escape;
						break;
					case SDL_SCANCODE_LALT:
						iKey = Gwen::Key::Alt;
						break;
					case SDL_SCANCODE_RALT:
						iKey = Gwen::Key::Alt;
						break;
					default:
						return false;
				}
				return m_Canvas->InputKey(iKey, E->state);
			}
			case SDL_TEXTINPUT:
			{
				SDL_TextInputEvent* E = &Event->text;
				wchar_t* widechar = (wchar_t*)SDL_iconv_string(UCS_STRING, "UTF-8", E->text, SDL_strlen(E->text) + 1);
				bool ret = m_Canvas->InputCharacter(*widechar);
				SDL_free(widechar);
				return ret;
			}
			case SDL_MOUSEMOTION:
			{
				SDL_MouseMotionEvent* E = &Event->motion;
				return m_Canvas->InputMouseMoved(E->x, E->y, E->xrel, E->yrel);
			}
			case SDL_MOUSEBUTTONDOWN:
			case SDL_MOUSEBUTTONUP:
			{
				SDL_MouseButtonEvent* E = &Event->button;
				int Button = -1;
				switch (E->button)
				{
					case SDL_BUTTON_LEFT:
						Button = 0;
						break;
					case SDL_BUTTON_MIDDLE:
						Button = 2;
						break;
					case SDL_BUTTON_RIGHT:
						Button = 1;
						break;
					default:
						return false;
				}
				return m_Canvas->InputMouseButton(Button, E->state);
			}
			case SDL_MOUSEWHEEL:
			{
				SDL_MouseWheelEvent* E = &Event->wheel;
				return m_Canvas->InputMouseWheel(E->y);
			}
			default:
			{
				return false;
			}
		}
	}

protected:
	Gwen::Controls::Canvas* m_Canvas;
};
}  // namespace Input
}  // namespace Gwen
#endif
