/*
	GWEN

	Copyright (c) 2010 Facepunch Studios

	MIT License

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/

#pragma once
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
