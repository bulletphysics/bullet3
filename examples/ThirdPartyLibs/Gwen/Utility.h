/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_UTILITY_H
#define GWEN_UTILITY_H
#include <sstream>
#include <vector>
#include "Gwen/Structures.h"

namespace Gwen
{
namespace Utility
{
template <typename T>
const T& Max(const T& x, const T& y)
{
	if (y < x) return x;
	return y;
}

template <typename T>
const T& Min(const T& x, const T& y)
{
	if (y > x) return x;
	return y;
}

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4996)
#endif

inline String UnicodeToString(const UnicodeString& strIn)
{
	if (!strIn.length()) return "";

	String temp(strIn.length(), (char)0);

	std::use_facet<std::ctype<wchar_t> >(std::locale()).narrow(&strIn[0], &strIn[0] + strIn.length(), ' ', &temp[0]);

	return temp;
}

inline UnicodeString StringToUnicode(const String& strIn)
{
	if (!strIn.length()) return L"";

	UnicodeString temp(strIn.length(), (wchar_t)0);

	std::use_facet<std::ctype<wchar_t> >(std::locale()).widen(&strIn[0], &strIn[0] + strIn.length(), &temp[0]);

	return temp;
}

#ifdef _MSC_VER
#pragma warning(pop)
#endif

template <class T>
String ToString(const T& object)
{
	std::ostringstream os;
	os << object;
	return os.str();
}

inline Gwen::Rect ClampRectToRect(Gwen::Rect inside, Gwen::Rect outside, bool clampSize = false)
{
	if (inside.x < outside.x)
		inside.x = outside.x;

	if (inside.y < outside.y)
		inside.y = outside.y;

	if (inside.x + inside.w > outside.x + outside.w)
	{
		if (clampSize)
			inside.w = outside.w;
		else
			inside.x = outside.x + outside.w - inside.w;
	}
	if (inside.y + inside.h > outside.y + outside.h)
	{
		if (clampSize)
			inside.h = outside.h;
		else
			inside.y = outside.w + outside.h - inside.h;
	}

	return inside;
}

GWEN_EXPORT UnicodeString Format(const wchar_t* fmt, ...);

namespace Strings
{
typedef std::vector<Gwen::String> List;
typedef std::vector<Gwen::UnicodeString> UnicodeList;

GWEN_EXPORT void Split(const Gwen::String& str, const Gwen::String& seperator, Strings::List& outbits, bool bLeaveSeperators = false);
GWEN_EXPORT void Split(const Gwen::UnicodeString& str, const Gwen::UnicodeString& seperator, Strings::UnicodeList& outbits, bool bLeaveSeperators = false);

template <typename T>
T TrimLeft(const T& str, const T& strChars)
{
	T outstr = str;
	outstr.erase(0, outstr.find_first_not_of(strChars));
	return outstr;
}

namespace To
{
GWEN_EXPORT bool Bool(const Gwen::String& str);
GWEN_EXPORT int Int(const Gwen::String& str);
GWEN_EXPORT float Float(const Gwen::String& str);
GWEN_EXPORT bool Floats(const Gwen::String& str, float* f, size_t iCount);
}  // namespace To
}  // namespace Strings
}  // namespace Utility

}  // namespace Gwen
#endif
