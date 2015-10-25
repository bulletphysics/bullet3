/*

GWEN modified and adapted for the Bullet Physics Library
Using the Zlib license with permission from Garry Newman

Copyright (c) 2010 Facepunch Studios
This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:
1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef GWEN_GWEN_H
#define GWEN_GWEN_H

#include "Gwen/Macros.h"
#include "Gwen/Config.h"
#include "Gwen/Exports.h"
#include "Gwen/Structures.h"
#include "Gwen/Skin.h"
#include "Gwen/Controls/Base.h"
#include "Gwen/Controls/Canvas.h"
#include "Gwen/Align.h"
#include "Gwen/TextObject.h"

// Enable the hook system (se Hook.h)
#define GWEN_HOOKSYSTEM

namespace Gwen
{
	namespace Controls
	{
		class Base;
		class Canvas;
	}

	namespace Renderer
	{
		class Base;
	}

	namespace Debug 
	{
		void GWEN_EXPORT Msg( const wchar_t* str, ... );
		void GWEN_EXPORT Msg( const char* str, ... );
		void GWEN_EXPORT AssertCheck( bool b, const char* strMsg );
	}


	namespace Colors
	{
		static const Color Black	( 0, 0, 0, 255 );
		static const Color Red		( 255, 0, 0, 255 );
		static const Color Yellow	( 255, 255, 0, 255 );
		static const Color White	( 255, 255, 255, 255 );
		static const Color Blue		( 0, 0, 255, 255 );
		static const Color Green	( 0, 255, 0, 255 );
		static const Color Grey		( 200, 200, 200, 255 );
		static const Color GreyLight( 230, 230, 230, 255 );
		static const Color GwenPink	( 255, 65, 199, 255 );
		
		
	};

	extern GWEN_EXPORT Controls::Base*	HoveredControl;
	extern GWEN_EXPORT Controls::Base*	KeyboardFocus;
	extern GWEN_EXPORT Controls::Base*	MouseFocus;

} //namespace Gwen

#endif
