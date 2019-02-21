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


#ifndef PX_COLLISION_NXHEIGHTFIELDFLAG
#define PX_COLLISION_NXHEIGHTFIELDFLAG
/** \addtogroup geomutils
@{
*/

#include "foundation/PxFlags.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief Describes the format of height field samples.
@see PxHeightFieldDesc.format PxHeightFieldDesc.samples
*/
struct PxHeightFieldFormat
{
	enum Enum
	{
		/**
		\brief Height field height data is 16 bit signed integers, followed by triangle materials. 
		
		Each sample is 32 bits wide arranged as follows:
		
		\image html heightFieldFormat_S16_TM.png

		1) First there is a 16 bit height value.
		2) Next, two one byte material indices, with the high bit of each byte reserved for special use.
		(so the material index is only 7 bits).
		The high bit of material0 is the tess-flag.
		The high bit of material1 is reserved for future use.
		
		There are zero or more unused bytes before the next sample depending on PxHeightFieldDesc.sampleStride, 
		where the application may eventually keep its own data.

		This is the only format supported at the moment.

		@see PxHeightFieldDesc.format PxHeightFieldDesc.samples
		*/
		eS16_TM = (1 << 0)
	};
};

/** 
\brief Determines the tessellation of height field cells.
@see PxHeightFieldDesc.format PxHeightFieldDesc.samples
*/
struct PxHeightFieldTessFlag
{
	enum Enum
	{
		/**
		\brief This flag determines which way each quad cell is subdivided.

		The flag lowered indicates subdivision like this: (the 0th vertex is referenced by only one triangle)
		
		\image html heightfieldTriMat2.PNG

		<pre>
		+--+--+--+---> column
		| /| /| /|
		|/ |/ |/ |
		+--+--+--+
		| /| /| /|
		|/ |/ |/ |
		+--+--+--+
		|
		|
		V row
		</pre>
		
		The flag raised indicates subdivision like this: (the 0th vertex is shared by two triangles)
		
		\image html heightfieldTriMat1.PNG

		<pre>
		+--+--+--+---> column
		|\ |\ |\ |
		| \| \| \|
		+--+--+--+
		|\ |\ |\ |
		| \| \| \|
		+--+--+--+
		|
		|
		V row
		</pre>
		
		@see PxHeightFieldDesc.format PxHeightFieldDesc.samples
		*/
		e0TH_VERTEX_SHARED = (1 << 0)
	};
};


/**
\brief Enum with flag values to be used in PxHeightFieldDesc.flags.
*/
struct PxHeightFieldFlag
{
	enum Enum
	{
		/**
		\brief Disable collisions with height field with boundary edges.
		
		Raise this flag if several terrain patches are going to be placed adjacent to each other, 
		to avoid a bump when sliding across.

		This flag is ignored in contact generation with sphere and capsule shapes.

		@see PxHeightFieldDesc.flags
		*/
		eNO_BOUNDARY_EDGES = (1 << 0)
	};
};

/**
\brief collection of set bits defined in PxHeightFieldFlag.

@see PxHeightFieldFlag
*/
typedef PxFlags<PxHeightFieldFlag::Enum,PxU16> PxHeightFieldFlags;
PX_FLAGS_OPERATORS(PxHeightFieldFlag::Enum,PxU16)

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
