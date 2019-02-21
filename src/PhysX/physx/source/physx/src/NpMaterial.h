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


#ifndef PX_PHYSICS_NP_MATERIAL
#define PX_PHYSICS_NP_MATERIAL

#include "PxMaterial.h"
#include "ScMaterialCore.h"
#include "PsUserAllocated.h"
#include "CmRefCountable.h"
#include "PsUtilities.h"

// PX_SERIALIZATION
#include "PxSerialFramework.h"
//~PX_SERIALIZATION

namespace physx
{

// Compared to other objects, materials are special since they belong to the SDK and not to scenes
// (similar to meshes). That's why the NpMaterial does have direct access to the core material instead
// of having a buffered interface for it. Scenes will have copies of the SDK material table and there
// the materials will be buffered.


class NpMaterial : public PxMaterial,  public Ps::UserAllocated, public Cm::RefCountable
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================
public:
// PX_SERIALIZATION            
									NpMaterial(PxBaseFlags baseFlags) : PxMaterial(baseFlags), Cm::RefCountable(PxEmpty), mMaterial(PxEmpty) {}								
	virtual		void				onRefCountZero();
	virtual		void				resolveReferences(PxDeserializationContext& context);
	static		NpMaterial*			createObject(PxU8*& address, PxDeserializationContext& context);
	static		void				getBinaryMetaData(PxOutputStream& stream);
				void				exportExtraData(PxSerializationContext&)	{}
				void				importExtraData(PxDeserializationContext&) {}
	virtual		void				requiresObjects(PxProcessPxBaseCallback&){}
//~PX_SERIALIZATION
									NpMaterial(const Sc::MaterialCore& desc);
									~NpMaterial();

	virtual		void				release();

	virtual		void				acquireReference();
	virtual		PxU32				getReferenceCount() const;

	virtual		void				setDynamicFriction(PxReal);
	virtual		PxReal				getDynamicFriction() const;
	virtual		void				setStaticFriction(PxReal);
	virtual		PxReal				getStaticFriction() const;
	virtual		void				setRestitution(PxReal);
	virtual		PxReal				getRestitution() const;  
	virtual		void				setFlag(PxMaterialFlag::Enum flag, bool value);
	virtual		void				setFlags(PxMaterialFlags inFlags);
	virtual		PxMaterialFlags		getFlags() const;
	virtual		void				setFrictionCombineMode(PxCombineMode::Enum);
	virtual		PxCombineMode::Enum	getFrictionCombineMode() const;
	virtual		void				setRestitutionCombineMode(PxCombineMode::Enum);
	virtual		PxCombineMode::Enum	getRestitutionCombineMode() const;

	PX_INLINE	const Sc::MaterialCore&	getScMaterial()	const	{ return mMaterial;			}
	PX_INLINE	Sc::MaterialCore&	getScMaterial()				{ return mMaterial;			}
	PX_INLINE	PxU32				getHandle()			const	{ return mMaterial.getMaterialIndex();}
	PX_INLINE	void				setHandle(PxU32 handle)		{ return mMaterial.setMaterialIndex(handle);}

	PX_FORCE_INLINE static void		getMaterialIndices(PxMaterial*const* materials, PxU16* materialIndices, PxU32 materialCount);

private:
	PX_INLINE	void				updateMaterial();

// PX_SERIALIZATION
public:
//~PX_SERIALIZATION
		Sc::MaterialCore			mMaterial;
};


PX_FORCE_INLINE void NpMaterial::getMaterialIndices(PxMaterial*const* materials, PxU16* materialIndices, PxU32 materialCount)
{
	for(PxU32 i=0; i < materialCount; i++)
	{
		materialIndices[i] = Ps::to16((static_cast<NpMaterial*>(materials[i]))->getHandle());
	}
}


}

#endif
