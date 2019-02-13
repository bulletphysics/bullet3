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

#ifndef GU_CONVEX_SUPPORT_TABLE_H
#define GU_CONVEX_SUPPORT_TABLE_H

#include "GuVecConvex.h"
#include "PsVecTransform.h"
#include "PxPhysXCommonConfig.h"

namespace physx
{
namespace Gu  
{

	class TriangleV; 
	class CapsuleV;
	class BoxV;
	class ConvexHullV;
	class ConvexHullNoScaleV;

#if PX_VC 
    #pragma warning(push)   
	#pragma warning( disable : 4324 ) // Padding was added at the end of a structure because of a __declspec(align) value.
#endif
	class SupportLocal
	{
	public:		
		Ps::aos::Vec3V shapeSpaceCenterOfMass;
		const Ps::aos::PsTransformV& transform;
		const Ps::aos::Mat33V& vertex2Shape;
		const Ps::aos::Mat33V& shape2Vertex;
		const bool isIdentityScale;	

		SupportLocal(const Ps::aos::PsTransformV& _transform, const Ps::aos::Mat33V& _vertex2Shape, const Ps::aos::Mat33V& _shape2Vertex, const bool _isIdentityScale = true): transform(_transform), 
			vertex2Shape(_vertex2Shape), shape2Vertex(_shape2Vertex), isIdentityScale(_isIdentityScale)
		{
		}

		PX_FORCE_INLINE void setShapeSpaceCenterofMass(const Ps::aos::Vec3VArg _shapeSpaceCenterOfMass)
		{
			shapeSpaceCenterOfMass = _shapeSpaceCenterOfMass;
		}
        virtual ~SupportLocal() {}
		virtual Ps::aos::Vec3V doSupport(const Ps::aos::Vec3VArg dir) const = 0;
		virtual void doSupport(const Ps::aos::Vec3VArg dir, Ps::aos::FloatV& min, Ps::aos::FloatV& max) const = 0;
		virtual void populateVerts(const PxU8* inds, PxU32 numInds, const PxVec3* originalVerts, Ps::aos::Vec3V* verts)const = 0;
	
	protected:
		SupportLocal& operator=(const SupportLocal&);
	};
#if PX_VC 
     #pragma warning(pop) 
#endif

	template <typename Convex>
	class SupportLocalImpl : public SupportLocal
	{
		
	public:
		const Convex& conv;
		SupportLocalImpl(const Convex& _conv, const Ps::aos::PsTransformV& _transform, const Ps::aos::Mat33V& _vertex2Shape, const Ps::aos::Mat33V& _shape2Vertex, const bool _isIdentityScale = true) : 
		SupportLocal(_transform, _vertex2Shape, _shape2Vertex, _isIdentityScale), conv(_conv)
		{
		}

		Ps::aos::Vec3V doSupport(const Ps::aos::Vec3VArg dir) const
		{
			//return conv.supportVertsLocal(dir);
			return conv.supportLocal(dir);
		}

		void doSupport(const Ps::aos::Vec3VArg dir, Ps::aos::FloatV& min, Ps::aos::FloatV& max) const
		{
			return conv.supportLocal(dir, min, max);
		}

		void populateVerts(const PxU8* inds, PxU32 numInds, const PxVec3* originalVerts, Ps::aos::Vec3V* verts) const 
		{
			conv.populateVerts(inds, numInds, originalVerts, verts);
		}

	protected:
		SupportLocalImpl& operator=(const SupportLocalImpl&);

	};
}

}

#endif
