/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/
/*
	Based on the FS Import classes:
	Copyright (C) 2005-2006 Feeling Software Inc
	Copyright (C) 2005-2006 Autodesk Media Entertainment
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include <limits>

#ifdef WIN32
#include <float.h>
#endif

#ifndef SAFE_DELETE_ARRAY
#define SAFE_DELETE_ARRAY(ptr) if (ptr != NULL) { delete [] ptr; ptr = NULL; }
#endif

template <class Char, class SPrintF>
FUStringBuilderT<Char,SPrintF>::FUStringBuilderT(const String& sz)
{
	this->buffer = NULL;
	this->size = 0;
	this->reserved = 0;
	
	reserve(sz.size() + 32);
	append(sz.c_str());
}

template <class Char, class SPrintF>
FUStringBuilderT<Char,SPrintF>::FUStringBuilderT(const Char* sz)
{
	this->buffer = NULL;
	this->size = 0;
	this->reserved = 0;

	SPrintF s;
	reserve(s.StrLen(sz) + 32);
	append(sz);
}

template <class Char, class SPrintF>
FUStringBuilderT<Char,SPrintF>::FUStringBuilderT(size_t reservation)
{
	this->buffer = NULL;
	this->size = 0;
	this->reserved = 0;

	reserve(reservation);
}

template <class Char, class SPrintF>
FUStringBuilderT<Char,SPrintF>::FUStringBuilderT()
{
	this->buffer = NULL;
	this->size = 0;
	this->reserved = 0;

#ifndef _DEBUG
	reserve(32);
#endif
}

template <class Char, class SPrintF>
FUStringBuilderT<Char,SPrintF>::~FUStringBuilderT()
{
	reserve(0);
}

template <class Char, class SPrintF>
void FUStringBuilderT<Char,SPrintF>::enlarge(size_t minimum)
{
	reserve(max(reserved + minimum + 32, 2 * reserved + 32));
}

template <class Char, class SPrintF>
void FUStringBuilderT<Char,SPrintF>::clear()
{
	size = 0;
}

template <class Char, class SPrintF>
void FUStringBuilderT<Char,SPrintF>::reserve(size_t _length)
{
	FUAssert(size <= reserved, );
	if (_length > reserved)
	{
		Char* b = new Char[_length];
		memcpy(b, buffer, size * sizeof(Char));
		SAFE_DELETE_ARRAY(buffer);
		buffer = b;
		reserved = _length;
	}
	else if (_length == 0)
	{
		SAFE_DELETE_ARRAY(buffer);
		size = reserved = 0;
	}
	else if (_length < reserved)
	{
		size_t realSize = min(size, _length);
		Char* b = new Char[_length];
		memcpy(b, buffer, realSize * sizeof(Char));
		SAFE_DELETE_ARRAY(buffer);
		buffer = b;
		reserved = _length;
		size = realSize;
	}
}

template <class Char, class SPrintF>
void FUStringBuilderT<Char,SPrintF>::append(Char c)
{
	if (size + 1 >= reserved) enlarge(2);

	buffer[size++] = c;
}

template <class Char, class SPrintF>
void FUStringBuilderT<Char,SPrintF>::append(const String& sz) { append(sz.c_str()); }
template <class Char, class SPrintF>
void FUStringBuilderT<Char,SPrintF>::append(const Char* sz)
{
	// This is optimized for SMALL strings.
	for (; *sz != 0; ++sz)
	{
		if (size >= reserved) enlarge(64);
		buffer[size++] = *sz;
	}
}
template <class Char, class SPrintF>
void FUStringBuilderT<Char,SPrintF>::append(const FUStringBuilderT& b)
{
	if (size + b.size >= reserved) enlarge(64 + size + b.size - reserved);
	memcpy(buffer + size, b.buffer, b.size * sizeof(Char));
	size += b.size;
}

template <class Char, class SPrintF>
void FUStringBuilderT<Char,SPrintF>::append(uint32 i)
{
	Char sz[128];
	SPrintF writer; writer.PrintUInt32(sz, 128, i);
	append(sz);
}

template <class Char, class SPrintF>
void FUStringBuilderT<Char,SPrintF>::append(uint64 i)
{
	Char sz[128];
	SPrintF writer; writer.PrintUInt64(sz, 128, i);
	append(sz);
}

template <class Char, class SPrintF>
void FUStringBuilderT<Char,SPrintF>::append(int32 i)
{
	Char sz[128];
	SPrintF writer; writer.PrintInt32(sz, 128, i);
	append(sz);
}

template <class Char, class SPrintF>
void FUStringBuilderT<Char,SPrintF>::append(float f)
{
	if (f != std::numeric_limits<float>::infinity() && f != -std::numeric_limits<float>::infinity() && f != std::numeric_limits<float>::quiet_NaN() && f != std::numeric_limits<float>::signaling_NaN())
	{
		if (IsEquivalent(f, 0.0f, std::numeric_limits<float>::epsilon())) append('0');
		else
		{
			Char sz[128];
			SPrintF writer; writer.PrintFloat(sz, 128, f);
			append(sz);
		}
	}
	else if (f == std::numeric_limits<float>::infinity())
	{ append('I'); append('N'); append('F'); }
	else if (f == -std::numeric_limits<float>::infinity())
	{ append('-'); append('I'); append('N'); append('F'); }
	else
	{ append('N'); append('a'); append('N'); }
}

template <class Char, class SPrintF>
void FUStringBuilderT<Char,SPrintF>::append(double f)
{
	if (f != std::numeric_limits<double>::infinity() && f != -std::numeric_limits<double>::infinity() && f != std::numeric_limits<double>::quiet_NaN() && f != std::numeric_limits<double>::signaling_NaN())
	{
		if (IsEquivalent(f, 0.0, std::numeric_limits<double>::epsilon())) append('0');
		else
		{
			Char sz[128];
			SPrintF writer; writer.PrintFloat(sz, 128, f);
			append(sz);
		}
	}
	else if (f == std::numeric_limits<double>::infinity())
	{ append('I'); append('N'); append('F'); }
	else if (f == -std::numeric_limits<double>::infinity())
	{ append('-'); append('I'); append('N'); append('F'); }
	else
	{ append('N'); append('a'); append('N'); }
}

template <class Char, class SPrintF>
void FUStringBuilderT<Char,SPrintF>::appendLine(const Char* sz)
{
	append(sz);
	append("\n");
}

template <class Char, class SPrintF>
void FUStringBuilderT<Char,SPrintF>::remove(int32 start)
{
	if ((int32)size > start && start >= 0) size = start;
}

template <class Char, class SPrintF>
void FUStringBuilderT<Char,SPrintF>::remove(int32 start, int32 end)
{
	int32 diff = end - start;
	if ((int32)size >= end && start >= 0 && diff > 0)
	{
		const Char* stop = buffer + size - diff;
		for (Char* p = buffer + start; p != stop; ++p)
		{
			*p = *(p + diff);
		}
		size -= diff;
	}
}

template <class Char, class SPrintF> 
typename FUStringBuilderT<Char,SPrintF>::String FUStringBuilderT<Char,SPrintF>::ToString()
{
	return String((const Char*)*this);
}

template <class Char, class SPrintF> 
const Char* FUStringBuilderT<Char,SPrintF>::ToCharPtr()
{
	if (size + 1 > reserved) enlarge(1);
	buffer[size] = 0;
	return buffer;
}

template <class Char, class SPrintF>
int32 FUStringBuilderT<Char,SPrintF>::index(Char c)
{
	if (buffer != NULL && size > 0)
	{
		const Char* end = buffer + size + 1;
		for (const Char* p = buffer; p != end; ++p)
		{
			if (*p == c) return (int32)(p - buffer);
		}
	}
	return -1;
}

template <class Char, class SPrintF>
int32 FUStringBuilderT<Char,SPrintF>::rindex(Char c)
{
	if (buffer != NULL && size > 0)
	{
		for (const Char* p = buffer + size - 1; p != buffer; --p)
		{
			if (*p == c) return (int32)(p - buffer);
		}
	}
	return -1;
}
