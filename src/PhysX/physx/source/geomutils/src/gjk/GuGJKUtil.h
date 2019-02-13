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
// Copyright (c) 2008-2018 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef GU_GJKUTIL_H
#define GU_GJKUTIL_H

#include "PsVecMath.h"
#include "CmPhysXCommon.h"

/*
	This file is used to avoid the inner loop cross DLL calls
*/
namespace physx
{
namespace Gu
{

enum GjkStatus
{
	GJK_NON_INTERSECT,	// two shapes doesn't intersect
	GJK_CLOSE,			// two shapes doesn't intersect and gjk algorithm will return closest point information
	GJK_CONTACT,		// two shapes overlap within margin 
	GJK_UNDEFINED,		// undefined status
	GJK_DEGENERATE,		// gjk can't converage

	EPA_CONTACT,		// two shapes intersect
	EPA_DEGENERATE,		// epa can't converage
	EPA_FAIL			// epa fail to construct an initial polygon to work with 
};

struct GjkOutput
{
public:
	GjkOutput()
	{
		using namespace Ps::aos;
		closestA = closestB = normal = V3Zero();
		penDep = FZero();
	}
	Ps::aos::Vec3V closestA;
	Ps::aos::Vec3V closestB;
	Ps::aos::Vec3V normal;
	Ps::aos::Vec3V searchDir;
	Ps::aos::FloatV penDep;
};

}//Gu
}//physx

#endif
