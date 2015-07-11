/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifdef _MSC_VER
#pragma warning( disable : 4244 )
#pragma warning( disable : 4251 )
#endif
#ifndef GWEN_STRUCTURES_H
#define GWEN_STRUCTURES_H

#include "Gwen/Exports.h"
#include <string>

namespace Gwen
{
	namespace Controls
	{
		class Base;
		class Canvas;
	}

	namespace CursorType
	{
		static const unsigned char Normal	= 0;
		static const unsigned char Beam		= 1;
		static const unsigned char SizeNS	= 2;
		static const unsigned char SizeWE	= 3;
		static const unsigned char SizeNWSE	= 4;
		static const unsigned char SizeNESW	= 5;
		static const unsigned char SizeAll	= 6;
		static const unsigned char No		= 7;
		static const unsigned char Wait		= 8;
		static const unsigned char Finger	= 9;

		static const unsigned char Count	= 10;
	}

	typedef std::wstring UnicodeString;
	typedef std::string String;
	typedef wchar_t UnicodeChar; // Portability??

	struct GWEN_EXPORT Margin
	{
		Margin( int left = 0, int top = 0, int right = 0, int bottom = 0 )
		{
			this->top = top;
			this->bottom = bottom;
			this->left = left;
			this->right = right;
		}

		int top, bottom, left, right;
	};


	typedef Margin Padding;


	struct GWEN_EXPORT Rect 
	{
		Rect( int x = 0, int y = 0, int w = 0, int h = 0 )
		{
			this->x = x;
			this->y = y;
			this->w = w;
			this->h = h;
		}

		int x, y, w, h;
	};


	struct GWEN_EXPORT Point
	{
		Point(int x = 0, int y = 0) 
		{ 
			this->x = x; 
			this->y = y;
		}

		int x, y;
	};

	struct GWEN_EXPORT HSV 
	{
		float h;
		float s;
		float v;
	};


	struct GWEN_EXPORT Color
	{
		Color( unsigned char r = 255, unsigned char g = 255, unsigned char b = 255, unsigned char a = 255 )
		{
			this->r = r;
			this->g = g;
			this->b = b;
			this->a = a;
		}

		void operator = ( Color c )
		{
			this->r = c.r;
			this->g = c.g;
			this->b = c.b;
			this->a = c.a;
		}

		void operator += ( Color c )
		{
			this->r += c.r;
			this->g += c.g;
			this->b += c.b;
			this->a += c.a;
		}

		void operator -= ( Color c )
		{
			this->r -= c.r;
			this->g -= c.g;
			this->b -= c.b;
			this->a -= c.a;
		}

		void operator *= ( float f )
		{
			this->r *= f;
			this->g *= f;
			this->b *= f;
			this->a *= f;
		}

		Color operator *( float f )
		{
			return Color( 
				(float)this->r*f, 
				(float)this->g*f, 
				(float)this->b*f, 
				(float)this->a*f 
				);
		}

		Color operator - ( Color c )
		{
			return Color( 
				this->r - c.r, 
				this->g - c.g, 
				this->b - c.b, 
				this->a - c.a 
				);
		}

		Color operator + ( Color c )
		{
			return Color( 
				this->r + c.r, 
				this->g + c.g, 
				this->b + c.b, 
				this->a + c.a 
				);
		}

		bool operator ==( const Color& c ) const
		{
			return c.r==r && c.g==g && c.b==b && c.a==a;
		}
		

		unsigned char r, g, b, a;
	};


	namespace DragAndDrop
	{
		struct GWEN_EXPORT Package
		{
			Package()
			{
				userdata = NULL;
				draggable = false;
				drawcontrol = NULL;
				holdoffset = Gwen::Point( 0, 0 );
			}

			String	name;
			void*	userdata;
			bool	draggable;

			Gwen::Controls::Base*	drawcontrol;
			Gwen::Point	holdoffset;
		};
	}

}
#endif
