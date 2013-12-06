/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef __BT_KEY_CODE_H__
#define __BT_KEY_CODE_H__

#include <GLUT/glut.h>

#if defined(__cplusplus)
extern "C" {
#endif

/**
	@brief Determine if a key is a letter.
	@return true if @a keycode is a letter, and not a control
	or modifier key. E.g., the letter 'b' is, where the Esc key ( BTKey_Escape ) is not.
*/
extern BOOL BTKeyIsAlpha( int );

/**
	@brief Determine if a key is a GLUT special key. ( arrow, F-Keys, etc )
*/
extern BOOL BTKeyIsSpecial( int keycode );

/**
	@brief Convert an OS X keycode to GLUT Special key representation
*/
extern int BTKeyTranslateKeyCodeToSpecial( int );

#if defined(__cplusplus)
}
#endif

#endif
