// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
//
// Memory.h
//
// Copyright (c) 2006 Simon Hobbs
//
// This software is provided 'as-is', without any express or implied warranty. In no event will the authors be held liable for any damages arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose, including commercial applications, and to alter it and redistribute it freely, subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
//
// 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
//
// 3. This notice may not be removed or altered from any source distribution.

#ifndef BULLET_MEMORY2_H
#define BULLET_MEMORY2_H

#ifdef WIN32

//added __cdecl, thanks Jack

// default new and delete overrides that guarantee 16 byte alignment and zero allocated memory
void* __cdecl operator new(size_t sz) throw();
void* __cdecl operator new[](size_t sz) throw();
void __cdecl operator delete(void* m) throw();
void __cdecl operator delete[](void* m) throw();

#include <malloc.h>

#define BULLET_ALIGNED_NEW_AND_DELETE \
\
inline void* operator new(size_t sz) throw()	\
{												\
	void* mem = _aligned_malloc(sz + 64, 16);	\
	return mem;									\
}												\
												\
inline void* operator new[](size_t sz) throw()	\
{												\
	void* mem = _aligned_malloc(sz + 64, 16);	\
	return mem;									\
}												\
												\
inline void operator delete(void* m) throw()	\
{												\
	if (m == 0)									\
		return;									\
	_aligned_free(m);							\
}												\
												\
inline void operator delete[](void* m) throw()	\
{												\
	_aligned_free(m);							\
}												\



#endif //WIN32	

#endif //MEMORY2_H
