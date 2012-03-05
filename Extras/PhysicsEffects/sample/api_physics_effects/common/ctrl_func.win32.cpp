/*
Physics Effects Copyright(C) 2010 Sony Computer Entertainment Inc.
All rights reserved.

Physics Effects is open software; you can redistribute it and/or
modify it under the terms of the BSD License.

Physics Effects is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the BSD License for more details.

A copy of the BSD License is distributed with
Physics Effects under the filename: physics_effects_license.txt
*/

#include "common.h"
#include "ctrl_func.h"

int keyState[2][BTN_NUM] = {0};
int keySw = 0;

void ctrl_init()
{
	keySw = 0;
}

void ctrl_release()
{
}

void ctrl_update()
{
	keyState[keySw][BTN_SCENE_RESET] = GetAsyncKeyState(VK_F1);
	keyState[keySw][BTN_SCENE_NEXT]  = GetAsyncKeyState(VK_F2);
	keyState[keySw][BTN_SIMULATION]  = GetAsyncKeyState(VK_F3);
	keyState[keySw][BTN_STEP]        = GetAsyncKeyState(VK_F4);
	keyState[keySw][BTN_UP]          = GetAsyncKeyState(VK_UP);
	keyState[keySw][BTN_DOWN]        = GetAsyncKeyState(VK_DOWN);
	keyState[keySw][BTN_LEFT]        = GetAsyncKeyState(VK_LEFT);
	keyState[keySw][BTN_RIGHT]       = GetAsyncKeyState(VK_RIGHT);
	keyState[keySw][BTN_ZOOM_IN]     = GetAsyncKeyState(VK_INSERT);
	keyState[keySw][BTN_ZOOM_OUT]    = GetAsyncKeyState(VK_DELETE);
	keyState[keySw][BTN_PICK]        = GetAsyncKeyState(VK_LBUTTON);
	keySw = 1-keySw;
}

ButtonStatus ctrl_button_pressed(ButtonID btnId)
{
	if(keyState[1-keySw][btnId] && !keyState[keySw][btnId]) {
		return BTN_STAT_DOWN;
	}
	else if(keyState[1-keySw][btnId] && keyState[keySw][btnId]) {
		return BTN_STAT_KEEP;
	}
	else if(!keyState[1-keySw][btnId] && keyState[keySw][btnId]) {
		return BTN_STAT_UP;
	}
	
	return BTN_STAT_NONE;
}

void ctrl_set_screen_size(int w,int h)
{
}

void ctrl_get_cursor_position(int &cursorX,int &cursorY)
{
	HWND hWnd = ::GetActiveWindow();

	POINT pnt;
	RECT rect;
	::GetCursorPos(&pnt);
	::ScreenToClient(hWnd,&pnt);
	::GetClientRect(hWnd,&rect);
	cursorX = pnt.x - (rect.right - rect.left) / 2;
	cursorY = (rect.bottom-rect.top) / 2 - pnt.y;
}
