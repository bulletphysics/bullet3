/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifdef _MSC_VER
#pragma warning(disable : 4244)
#pragma warning(disable : 4251)
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
}  // namespace Controls

namespace CursorType
{
static const unsigned char Normal = 0;
static const unsigned char Beam = 1;
static const unsigned char SizeNS = 2;
static const unsigned char SizeWE = 3;
static const unsigned char SizeNWSE = 4;
static const unsigned char SizeNESW = 5;
static const unsigned char SizeAll = 6;
static const unsigned char No = 7;
static const unsigned char Wait = 8;
static const unsigned char Finger = 9;

static const unsigned char Count = 10;
}  // namespace CursorType

typedef std::wstring UnicodeString;
typedef std::string String;
typedef wchar_t UnicodeChar;  // Portability??

struct GWEN_EXPORT Margin
{
	Margin(int left_ = 0, int top_ = 0, int right_ = 0, int bottom_ = 0)
	{
		this->top = top_;
		this->bottom = bottom_;
		this->left = left_;
		this->right = right_;
	}

	int top, bottom, left, right;
};

typedef Margin Padding;

struct GWEN_EXPORT Rect
{
	Rect(int x_ = 0, int y_ = 0, int w_ = 0, int h_ = 0)
	{
		this->x = x_;
		this->y = y_;
		this->w = w_;
		this->h = h_;
	}

	int x, y, w, h;
};

struct GWEN_EXPORT Point
{
	Point(int x_ = 0, int y_ = 0)
	{
		this->x = x_;
		this->y = y_;
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
	Color(unsigned char r_ = 255, unsigned char g_ = 255, unsigned char b_ = 255, unsigned char a_ = 255)
	{
		this->r = r_;
		this->g = g_;
		this->b = b_;
		this->a = a_;
	}

	void operator=(Color c)
	{
		this->r = c.r;
		this->g = c.g;
		this->b = c.b;
		this->a = c.a;
	}

	void operator+=(Color c)
	{
		this->r += c.r;
		this->g += c.g;
		this->b += c.b;
		this->a += c.a;
	}

	void operator-=(Color c)
	{
		this->r -= c.r;
		this->g -= c.g;
		this->b -= c.b;
		this->a -= c.a;
	}

	void operator*=(float f)
	{
		this->r *= (unsigned char)f;
		this->g *= (unsigned char)f;
		this->b *= (unsigned char)f;
		this->a *= (unsigned char)f;
	}

	Color operator*(float f)
	{
		return Color(
			(unsigned char)((float)this->r * f),
			(unsigned char)((float)this->g * f),
			(unsigned char)((float)this->b * f),
			(unsigned char)((float)this->a * f));
	}

	Color operator-(Color c)
	{
		return Color(
			(unsigned char)(this->r - c.r),
			(unsigned char)(this->g - c.g),
			(unsigned char)(this->b - c.b),
			(unsigned char)(this->a - c.a));
	}

	Color operator+(Color c)
	{
		return Color(
			(unsigned char)(this->r + c.r),
			(unsigned char)(this->g + c.g),
			(unsigned char)(this->b + c.b),
			(unsigned char)(this->a + c.a));
	}

	bool operator==(const Color& c) const
	{
		return c.r == r && c.g == g && c.b == b && c.a == a;
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
		holdoffset = Gwen::Point(0, 0);
	}

	String name;
	void* userdata;
	bool draggable;

	Gwen::Controls::Base* drawcontrol;
	Gwen::Point holdoffset;
};
}  // namespace DragAndDrop

}  // namespace Gwen
#endif
