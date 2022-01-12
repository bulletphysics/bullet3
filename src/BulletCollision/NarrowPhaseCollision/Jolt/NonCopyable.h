// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT
#ifndef __JOLT_NON_COPYABLE_H
#define __JOLT_NON_COPYABLE_H
#pragma once

namespace BTJPH {
  
typedef unsigned int uint;
typedef unsigned char jUint8;
typedef unsigned short jUint16;
#ifdef __GNUC__
#include <stdint.h>
typedef int32_t jInt32a_t;
typedef int64_t jInt64a_t;
typedef uint32_t jUint32a_t;
typedef uint64_t jUint64a_t;
#elif defined(_MSC_VER)
typedef __int32 jInt32a_t;
typedef __int64 jInt64a_t;
typedef unsigned __int32 jUint32a_t;
typedef unsigned __int64 jUint64a_t;
#else
typedef int jInt32a_t;
typedef long long int jInt64a_t;
typedef unsigned int jUint32a_t;
typedef unsigned long long int jUint64a_t;
#endif

/// Class that makes another class non-copyable. Usage: Inherit from NonCopyable.
class NonCopyable
{
public:
			NonCopyable() = default;
			NonCopyable(const NonCopyable &) = delete;
	void	operator = (const NonCopyable &) = delete;
};

};

#endif //__JOLT_NON_COPYABLE_H
