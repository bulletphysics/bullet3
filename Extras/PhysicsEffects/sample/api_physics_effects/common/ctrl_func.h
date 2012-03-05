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

#ifndef __CTRL_FUNC_H__
#define __CTRL_FUNC_H__

enum ButtonID {
	BTN_SCENE_RESET=0,
	BTN_SCENE_NEXT,
	BTN_SIMULATION,
	BTN_STEP,
	BTN_UP,
	BTN_DOWN,
	BTN_LEFT,
	BTN_RIGHT,
	BTN_ZOOM_IN,
	BTN_ZOOM_OUT,
	BTN_PICK,
	BTN_NUM
};

enum ButtonStatus {
	BTN_STAT_NONE = 0,
	BTN_STAT_DOWN,
	BTN_STAT_UP,
	BTN_STAT_KEEP
};

void ctrl_init();
void ctrl_release();
void ctrl_update();

void ctrl_set_screen_size(int w,int h);
void ctrl_get_cursor_position(int &cursorX,int &cursorY);

ButtonStatus ctrl_button_pressed(ButtonID btnId);

#endif /* __CTRL_FUNC_H__ */
