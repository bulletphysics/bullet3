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

#import "BTGLUTKeyAdapter.h"
#import <Cocoa/Cocoa.h>

BOOL BTKeyIsAlpha( int key )
{
	return !BTKeyIsSpecial( key );
}

BOOL BTKeyIsSpecial( int keycode )
{
	BOOL ret = NO;
	switch( keycode )
	{
		case NSUpArrowFunctionKey:
		case NSDownArrowFunctionKey:
		case NSLeftArrowFunctionKey:
		case NSRightArrowFunctionKey:
		case NSF1FunctionKey:
		case NSF2FunctionKey:
		case NSF3FunctionKey:
		case NSF4FunctionKey:
		case NSF5FunctionKey:
		case NSF6FunctionKey:
		case NSF7FunctionKey:
		case NSF8FunctionKey:
		case NSF9FunctionKey:
		case NSF10FunctionKey:
		case NSF11FunctionKey:
		case NSF12FunctionKey:
			ret = YES;
			break;
						
		default: break;	
	}
	
	return ret;
}


int BTKeyTranslateKeyCodeToSpecial( int kc )
{
	int ret = kc;
	switch( kc )
	{
		case NSUpArrowFunctionKey:
			ret = GLUT_KEY_UP;
			break;
	
		case NSDownArrowFunctionKey:
			ret = GLUT_KEY_DOWN;
			break;
	
		case NSLeftArrowFunctionKey:
			ret = GLUT_KEY_LEFT;
			break;
	
		case NSRightArrowFunctionKey:
			ret = GLUT_KEY_RIGHT;
			break;

		case NSF1FunctionKey:
			ret = GLUT_KEY_F1;
			break;

		case NSF2FunctionKey:
			ret = GLUT_KEY_F2;
			break;

		case NSF3FunctionKey:
			ret = GLUT_KEY_F3;
			break;

		case NSF4FunctionKey:
			ret = GLUT_KEY_F4;
			break;

		case NSF5FunctionKey:
			ret = GLUT_KEY_F5;
			break;

		case NSF6FunctionKey:
			ret = GLUT_KEY_F6;
			break;

		case NSF7FunctionKey:
			ret = GLUT_KEY_F7;
			break;

		case NSF8FunctionKey:
			ret = GLUT_KEY_F8;
			break;

		case NSF9FunctionKey:
			ret = GLUT_KEY_F9;
			break;

		case NSF10FunctionKey:
			ret = GLUT_KEY_F10;
			break;

		case NSF11FunctionKey:
			ret = GLUT_KEY_F11;
			break;

		case NSF12FunctionKey:
			ret = GLUT_KEY_F12;
			break;
						
		default: break;	
	}
	
	return ret;
}
