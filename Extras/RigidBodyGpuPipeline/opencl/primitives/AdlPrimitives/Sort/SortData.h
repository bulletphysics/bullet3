/*
		2011 Takahiro Harada
*/

#pragma once

#include <AdlPrimitives/Math/Math.h>

namespace adl
{

struct SortData
{
	SortData(){}
	SortData( u32 key, u32 value ) : m_key(key), m_value(value) {}

	union
	{
		u32 m_key;
		struct { u16 m_key16[2]; };
	};
	u32 m_value;

	friend bool operator <(const SortData& a, const SortData& b)
	{
		return a.m_key < b.m_key;
	}
};


};
