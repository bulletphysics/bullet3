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


#ifndef PX_PHYSICS_NP_SHAPE
#define PX_PHYSICS_NP_SHAPE

#include "PxShape.h"
#include "buffering/ScbShape.h"
#include "PxMetaData.h"

namespace physx
{

struct NpInternalShapeFlag
{
	enum Enum
	{
		eEXCLUSIVE				= (1<<0)
	};
};

/**
\brief collection of set bits defined in PxShapeFlag.

@see PxShapeFlag
*/
typedef PxFlags<NpInternalShapeFlag::Enum,PxU8> NpInternalShapeFlags;
PX_FLAGS_OPERATORS(NpInternalShapeFlag::Enum,PxU8)


class NpScene;
class NpShapeManager;

namespace Scb
{
	class Scene;
	class RigidObject;
}

namespace Sc
{
	class MaterialCore;
}

class NpShape : public PxShape, public Ps::UserAllocated, public Cm::RefCountable
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================
public:
// PX_SERIALIZATION
												NpShape(PxBaseFlags baseFlags);
	virtual			void						exportExtraData(PxSerializationContext& stream);
					void						importExtraData(PxDeserializationContext& context);
	virtual			void						requiresObjects(PxProcessPxBaseCallback& c);
					void						resolveReferences(PxDeserializationContext& context);
	static			NpShape*					createObject(PxU8*& address, PxDeserializationContext& context);
	static			void						getBinaryMetaData(PxOutputStream& stream);
//~PX_SERIALIZATION
												NpShape(const PxGeometry& geometry,
													PxShapeFlags shapeFlags,
													const PxU16* materialIndices,
													PxU16 materialCount, 
													bool isExclusive);

	virtual										~NpShape();
	
	//---------------------------------------------------------------------------------
	// PxShape implementation
	//---------------------------------------------------------------------------------
	virtual			void						release(); //!< call to release from actor
	virtual			PxU32						getReferenceCount() const;
	virtual			void						acquireReference();

	virtual			PxGeometryType::Enum		getGeometryType() const;

	virtual			void						setGeometry(const PxGeometry&);
	virtual			PxGeometryHolder			getGeometry() const;
	virtual			bool						getBoxGeometry(PxBoxGeometry&) const;
	virtual			bool						getSphereGeometry(PxSphereGeometry&) const;
	virtual			bool						getCapsuleGeometry(PxCapsuleGeometry&) const;
	virtual			bool						getPlaneGeometry(PxPlaneGeometry&) const;
	virtual			bool						getConvexMeshGeometry(PxConvexMeshGeometry& g) const;
	virtual			bool						getTriangleMeshGeometry(PxTriangleMeshGeometry& g) const;
	virtual			bool						getHeightFieldGeometry(PxHeightFieldGeometry& g) const;

	virtual			PxRigidActor*				getActor() const;

	virtual			void						setLocalPose(const PxTransform& pose);
	virtual			PxTransform					getLocalPose() const;

	virtual			void						setSimulationFilterData(const PxFilterData& data);
	virtual			PxFilterData				getSimulationFilterData() const;
	virtual			void						setQueryFilterData(const PxFilterData& data);
	virtual			PxFilterData				getQueryFilterData() const;

	virtual			void						setMaterials(PxMaterial*const* materials, PxU16 materialCount);
	virtual			PxU16						getNbMaterials()															const;
	virtual			PxU32						getMaterials(PxMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex=0)	const;
	virtual			PxMaterial*					getMaterialFromInternalFaceIndex(PxU32 faceIndex)							const;

	virtual			void						setContactOffset(PxReal);
	virtual			PxReal						getContactOffset() const;

	virtual			void						setRestOffset(PxReal);
	virtual			PxReal						getRestOffset() const;

	virtual			void						setTorsionalPatchRadius(PxReal);
	virtual			PxReal						getTorsionalPatchRadius() const;

	virtual			void						setMinTorsionalPatchRadius(PxReal);
	virtual			PxReal						getMinTorsionalPatchRadius() const;

	virtual			void						setFlag(PxShapeFlag::Enum flag, bool value);
	virtual			void						setFlags( PxShapeFlags inFlags );
	virtual			PxShapeFlags				getFlags() const;

	virtual			bool						isExclusive() const;

	virtual			void					    setName(const char* debugName);
	virtual			const char*					getName() const;

	//---------------------------------------------------------------------------------
	// RefCountable implementation
	//---------------------------------------------------------------------------------

	// Ref counting for shapes works like this: 
	// * for exclusive shapes the actor has a counted reference
	// * for shared shapes, each actor has a counted reference, and the user has a counted reference
	// * for either kind, each instance of the shape in a scene (i.e. each shapeSim) causes the reference count to be incremented by 1.
	// Because these semantics aren't clear to users, this reference count should not be exposed in the API

	virtual			void						onRefCountZero();

	//---------------------------------------------------------------------------------
	// Miscellaneous
	//---------------------------------------------------------------------------------

					void						setFlagsInternal( PxShapeFlags inFlags );

	PX_FORCE_INLINE	PxShapeFlags				getFlagsFast()			const	{ return mShape.getFlags();									}
	PX_FORCE_INLINE	PxShapeFlags				getFlagsUnbuffered()	const	{ return mShape.getScShape().getFlags();					}
	PX_FORCE_INLINE	PxGeometryType::Enum		getGeometryTypeFast()	const	{ return mShape.getGeometryType();							}
	PX_FORCE_INLINE	const Gu::GeometryUnion&	getGeometryFast()		const	{ return mShape.getGeometryUnion();							}
	PX_FORCE_INLINE const PxTransform&			getLocalPoseFast()		const	{ return mShape.getShape2Actor();							}
	PX_FORCE_INLINE PxU32						getActorCount()			const	{ return PxU32(mExclusiveAndActorCount & ACTOR_COUNT_MASK);	}
	PX_FORCE_INLINE PxI32						isExclusiveFast()		const	{ return mExclusiveAndActorCount & EXCLUSIVE_MASK;			}

	PX_FORCE_INLINE	const PxFilterData&			getQueryFilterDataFast() const
												{
													return mShape.getScShape().getQueryFilterData();	// PT: this one doesn't need double-buffering
												}

	PX_FORCE_INLINE	const Scb::Shape&			getScbShape()			const	{ return mShape;	}
	PX_FORCE_INLINE	Scb::Shape&					getScbShape()					{ return mShape;	}

	PX_INLINE		PxMaterial*					getMaterial(PxU32 index) const { return mShape.getMaterial(index); }
	static			bool						checkMaterialSetup(const PxGeometry& geom, const char* errorMsgPrefix, PxMaterial*const* materials, PxU16 materialCount);

					void						onActorAttach(PxRigidActor& actor);
					void						onActorDetach();

					// These methods are used only for sync'ing, and may only be called on exclusive shapes since only exclusive shapes have buffering
					Sc::RigidCore&				getScRigidObjectExclusive() const;
					void						releaseInternal();

					NpScene*					getOwnerScene()				const;	// same distinctions as for NpActor
private:
					NpScene*					getAPIScene()				const;

					void						incMeshRefCount();
					void						decMeshRefCount();
					Cm::RefCountable*			getMeshRefCountable();
					bool						isWritable();
					void						updateSQ(const char* errorMessage);

					PxRigidActor*				mActor;							// Auto-resolving refs breaks DLL loading for some reason
					Scb::Shape					mShape;
					const char*					mName;

					static const PxI32 EXCLUSIVE_MASK = 0x80000000;
					static const PxI32 ACTOR_COUNT_MASK = 0x7fffffff;
					
					volatile PxI32				mExclusiveAndActorCount;
};

}

#endif
