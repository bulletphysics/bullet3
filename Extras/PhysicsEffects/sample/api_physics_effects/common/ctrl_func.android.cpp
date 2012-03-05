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
}
