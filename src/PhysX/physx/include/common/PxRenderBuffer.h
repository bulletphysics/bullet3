//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2019 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  


#ifndef PX_FOUNDATION_PXRENDERBUFFER_H
#define PX_FOUNDATION_PXRENDERBUFFER_H

/** \addtogroup common
@{
*/

#include "common/PxPhysXCommonConfig.h"
#include "foundation/PxVec3.h"
#include "foundation/PxMat33.h"
#include "foundation/PxBounds3.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief Default color values used for debug rendering.
*/
struct PxDebugColor
{
	enum Enum
	{
		eARGB_BLACK		= 0xff000000,
		eARGB_RED		= 0xffff0000,
		eARGB_GREEN		= 0xff00ff00,
		eARGB_BLUE		= 0xff0000ff,
		eARGB_YELLOW	= 0xffffff00,
		eARGB_MAGENTA	= 0xffff00ff,
		eARGB_CYAN		= 0xff00ffff,
		eARGB_WHITE		= 0xffffffff,
		eARGB_GREY		= 0xff808080,
		eARGB_DARKRED	= 0x88880000,
		eARGB_DARKGREEN	= 0x88008800,
		eARGB_DARKBLUE	= 0x88000088
	};
};

/**
\brief Used to store a single point and colour for debug rendering.
*/
struct PxDebugPoint
{
	PxDebugPoint(const PxVec3& p, const PxU32& c)
		: pos(p), color(c) {}

	PxVec3	pos;
	PxU32	color;
};

/**
\brief Used to store a single line and colour for debug rendering.
*/
struct PxDebugLine
{
	PxDebugLine(const PxVec3& p0, const PxVec3& p1, const PxU32& c)
		: pos0(p0), color0(c), pos1(p1), color1(c) {}

	PxVec3	pos0;
	PxU32	color0;
	PxVec3	pos1;
	PxU32	color1;
};

/**
\brief Used to store a single triangle and colour for debug rendering.
*/
struct PxDebugTriangle
{
	PxDebugTriangle(const PxVec3& p0, const PxVec3& p1, const PxVec3& p2, const PxU32& c)
		: pos0(p0), color0(c), pos1(p1), color1(c), pos2(p2), color2(c) {}

	PxVec3	pos0;
	PxU32	color0;
	PxVec3	pos1;
	PxU32	color1;
	PxVec3	pos2;
	PxU32	color2;
};

/**
\brief Used to store a text for debug rendering. Doesn't own 'string' array.
*/
struct PxDebugText
{
	PxDebugText() : string(0) {}

	PxDebugText(const PxVec3& p, const PxReal& s, const PxU32& c, const char* str)
		: position(p), size(s), color(c), string(str) {}

	PxVec3		position;
	PxReal		size;
	PxU32		color;
	const char*	string;
};

/**
\brief Interface for points, lines, triangles, and text buffer.
*/
class PxRenderBuffer
{
public:
	virtual ~PxRenderBuffer() {}

	virtual PxU32 getNbPoints() const = 0;
	virtual const PxDebugPoint* getPoints() const = 0;

	virtual PxU32 getNbLines() const = 0;
	virtual const PxDebugLine* getLines() const = 0;

	virtual PxU32 getNbTriangles() const = 0;
	virtual const PxDebugTriangle* getTriangles() const = 0;

	virtual PxU32 getNbTexts() const = 0;
	virtual const PxDebugText* getTexts() const = 0;

	virtual void append(const PxRenderBuffer& other) = 0;
	virtual void clear() = 0;
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
