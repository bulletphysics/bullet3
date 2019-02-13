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


#ifndef DY_SOLVERCONSTRAINTTYPES_H
#define DY_SOLVERCONSTRAINTTYPES_H

#include "foundation/PxSimpleTypes.h"
#include "PxvConfig.h"

namespace physx
{

enum SolverConstraintType
{
	DY_SC_TYPE_NONE = 0,
	DY_SC_TYPE_RB_CONTACT,		// RB-only contact
	DY_SC_TYPE_RB_1D,			// RB-only 1D constraint
	DY_SC_TYPE_EXT_CONTACT,	// contact involving articulations
	DY_SC_TYPE_EXT_1D,			// 1D constraint involving articulations
	DY_SC_TYPE_STATIC_CONTACT,	// RB-only contact where body b is static
	DY_SC_TYPE_NOFRICTION_RB_CONTACT, //RB-only contact with no friction patch
	DY_SC_TYPE_BLOCK_RB_CONTACT,
	DY_SC_TYPE_BLOCK_STATIC_RB_CONTACT,
	DY_SC_TYPE_BLOCK_1D,
	DY_SC_TYPE_FRICTION,
	DY_SC_TYPE_STATIC_FRICTION,
	DY_SC_TYPE_EXT_FRICTION,
	DY_SC_TYPE_BLOCK_FRICTION,
	DY_SC_TYPE_BLOCK_STATIC_FRICTION,
	DY_SC_CONSTRAINT_TYPE_COUNT //Count of the number of different constraint types in the solver
};

enum SolverConstraintFlags
{
	DY_SC_FLAG_OUTPUT_FORCE		= (1<<1),
	DY_SC_FLAG_KEEP_BIAS		= (1<<2),
	DY_SC_FLAG_ROT_EQ			= (1<<3),
	DY_SC_FLAG_ORTHO_TARGET		= (1<<4),
	DY_SC_FLAG_SPRING			= (1<<5),
	DY_SC_FLAG_INEQUALITY		= (1<<6)
};

}

#endif //DY_SOLVERCONSTRAINTTYPES_H
