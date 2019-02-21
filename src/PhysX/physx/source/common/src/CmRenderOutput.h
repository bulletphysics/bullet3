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


#ifndef PX_FOUNDATION_PSRENDEROUTPUT_H
#define PX_FOUNDATION_PSRENDEROUTPUT_H

#include "foundation/PxMat44.h"
#include "CmRenderBuffer.h"
#include "CmUtils.h"

namespace physx
{
namespace Cm
{
	struct DebugText;

#if PX_VC 
    #pragma warning(push)
	#pragma warning( disable : 4251 ) // class needs to have dll-interface to be used by clients of class
#endif

	/**
	Output stream to fill RenderBuffer
	*/
	class PX_PHYSX_COMMON_API RenderOutput
	{
	public:

		enum Primitive {
			POINTS,
			LINES,
			LINESTRIP,
			TRIANGLES,
			TRIANGLESTRIP,
			TEXT
		};

		RenderOutput(RenderBuffer& buffer) 
			: mPrim(POINTS), mColor(0), mVertex0(0.0f), mVertex1(0.0f)
			, mVertexCount(0), mTransform(PxIdentity), mBuffer(buffer) 
		{}

		RenderOutput& operator<<(Primitive prim);
		RenderOutput& operator<<(PxU32 color); // 0xbbggrr
		RenderOutput& operator<<(const PxMat44& transform);
		RenderOutput& operator<<(const PxTransform&);

		RenderOutput& operator<<(PxVec3 vertex);
		RenderOutput& operator<<(const DebugText& text);

		PX_FORCE_INLINE PxDebugLine* reserveSegments(PxU32 nbSegments)
		{
			return reserveContainerMemory(mBuffer.mLines, nbSegments);
		}

		// PT: using the operators is just too slow.
		PX_FORCE_INLINE	void outputSegment(const PxVec3& v0, const PxVec3& v1)
		{
			PxDebugLine* segment = reserveSegments(1);
			segment->pos0 = v0;
			segment->pos1 = v1;
			segment->color0 = segment->color1 = mColor;
		}

		RenderOutput&	outputCapsule(PxF32 radius, PxF32 halfHeight, const PxMat44& absPose);

	private:

		RenderOutput& operator=(const RenderOutput&);

		Primitive		mPrim;
		PxU32			mColor;
		PxVec3			mVertex0, mVertex1;
		PxU32			mVertexCount;
		PxMat44			mTransform;
		RenderBuffer&	mBuffer;
	};

	/** debug render helper types */
	struct PX_PHYSX_COMMON_API DebugText 
	{
		DebugText(const PxVec3& position, PxReal size, const char* string, ...); 
		static const int sBufferSize = 1008; // sizeof(DebugText)==1kB
		char buffer[sBufferSize]; 
		PxVec3 position;
		PxReal size;
	};

	struct DebugBox 
	{
		explicit DebugBox(const PxVec3& extents, bool wireframe_ = true) 
		: minimum(-extents), maximum(extents), wireframe(wireframe_) {}

		explicit DebugBox(const PxVec3& pos, const PxVec3& extents, bool wireframe_ = true) 
		: minimum(pos-extents), maximum(pos+extents), wireframe(wireframe_) {}

		explicit DebugBox(const PxBounds3& bounds, bool wireframe_ = true)
		: minimum(bounds.minimum), maximum(bounds.maximum), wireframe(wireframe_) {}

		PxVec3 minimum, maximum;
		bool wireframe;
	};
	PX_PHYSX_COMMON_API RenderOutput& operator<<(RenderOutput& out, const DebugBox& box);

	struct DebugArrow 
	{
		DebugArrow(const PxVec3& pos, const PxVec3& vec) 
		: base(pos), tip(pos+vec), headLength(vec.magnitude()*0.15f) {}

		DebugArrow(const PxVec3& pos, const PxVec3& vec, PxReal headLength_) 
		: base(pos), tip(pos+vec), headLength(headLength_) {}

		PxVec3 base, tip;
		PxReal headLength;
	};
	PX_PHYSX_COMMON_API RenderOutput& operator<<(RenderOutput& out, const DebugArrow& arrow);

	struct DebugBasis
	{
		DebugBasis(const PxVec3& ext, PxU32 cX = PxDebugColor::eARGB_RED, 
			PxU32 cY = PxDebugColor::eARGB_GREEN, PxU32 cZ = PxDebugColor::eARGB_BLUE) 
		: extends(ext), colorX(cX), colorY(cY), colorZ(cZ) {}
		PxVec3 extends;
		PxU32 colorX, colorY, colorZ;
	};
	PX_PHYSX_COMMON_API RenderOutput& operator<<(RenderOutput& out, const DebugBasis& basis);

#if PX_VC 
     #pragma warning(pop) 
#endif

	struct DebugCircle
	{
		DebugCircle(PxU32 s, PxReal r) 
		: nSegments(s), radius(r) {}
		PxU32 nSegments;
		PxReal radius;
	};
	PX_PHYSX_COMMON_API RenderOutput& operator<<(RenderOutput& out, const DebugCircle& circle);

	struct DebugArc
	{
		DebugArc(PxU32 s, PxReal r, PxReal minAng, PxReal maxAng) 
		: nSegments(s), radius(r), minAngle(minAng), maxAngle(maxAng) {}
		PxU32 nSegments;
		PxReal radius;
		PxReal minAngle, maxAngle;
	};
	PX_PHYSX_COMMON_API RenderOutput& operator<<(RenderOutput& out, const DebugArc& arc);

} // namespace Cm

}

#endif
