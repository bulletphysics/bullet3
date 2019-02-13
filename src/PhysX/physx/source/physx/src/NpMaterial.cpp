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


#include "NpMaterial.h"
#include "NpPhysics.h"
#include "CmUtils.h"

using namespace physx;

NpMaterial::NpMaterial(const Sc::MaterialCore& desc)
: PxMaterial(PxConcreteType::eMATERIAL, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE)
, mMaterial(desc)
{
	mMaterial.setNxMaterial(this);  // back-reference	
}

NpMaterial::~NpMaterial()
{
	NpPhysics::getInstance().removeMaterialFromTable(*this);
}

// PX_SERIALIZATION
void NpMaterial::resolveReferences(PxDeserializationContext&)
{
	// ### this one could be automated if NpMaterial would inherit from MaterialCore
	// ### well actually in that case the pointer would not even be needed....
	mMaterial.setNxMaterial(this);	// Resolve MaterialCore::mNxMaterial

	// Maybe not the best place to do it but it has to be done before the shapes resolve material indices
	// since the material index translation table is needed there. This requires that the materials have
	// been added to the table already.
	NpPhysics::getInstance().addMaterial(this);
}

void NpMaterial::onRefCountZero()
{
	void* ud = userData;	

	if(getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
		NpFactory::getInstance().releaseMaterialToPool(*this);
	else
		this->~NpMaterial();

	NpPhysics::getInstance().notifyDeletionListenersMemRelease(this, ud);
}

NpMaterial* NpMaterial::createObject(PxU8*& address, PxDeserializationContext& context)
{
	NpMaterial* obj = new (address) NpMaterial(PxBaseFlag::eIS_RELEASABLE);
	address += sizeof(NpMaterial);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}
//~PX_SERIALIZATION

void NpMaterial::release()
{
	decRefCount();
}

void NpMaterial::acquireReference()
{
	incRefCount();
}

PxU32 NpMaterial::getReferenceCount() const
{
	return getRefCount();
}

PX_INLINE void NpMaterial::updateMaterial()
{
	NpPhysics::getInstance().updateMaterial(*this);
}

///////////////////////////////////////////////////////////////////////////////

void NpMaterial::setDynamicFriction(PxReal x)
{
	PX_CHECK_AND_RETURN(PxIsFinite(x), "PxMaterial::setDynamicFriction: invalid float");
	mMaterial.dynamicFriction = x;

	updateMaterial();
}

PxReal NpMaterial::getDynamicFriction() const
{
	return mMaterial.dynamicFriction;
}

///////////////////////////////////////////////////////////////////////////////

void NpMaterial::setStaticFriction(PxReal x)
{
	PX_CHECK_AND_RETURN(PxIsFinite(x), "PxMaterial::setStaticFriction: invalid float");
	mMaterial.staticFriction = x;

	updateMaterial();
}

PxReal NpMaterial::getStaticFriction() const
{
	return mMaterial.staticFriction;
}

///////////////////////////////////////////////////////////////////////////////

void NpMaterial::setRestitution(PxReal x)
{
	PX_CHECK_AND_RETURN(PxIsFinite(x), "PxMaterial::setRestitution: invalid float");
	PX_CHECK_MSG(((x >= 0.0f) && (x <= 1.0f)), "PxMaterial::setRestitution: Restitution value has to be in [0,1]!");
	if ((x < 0.0f) || (x > 1.0f))
	{
		PxClamp(x, 0.0f, 1.0f);
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "PxMaterial::setRestitution: Invalid value %f was clamped to [0,1]!", PxF64(x));
	}
	mMaterial.restitution = x;

	updateMaterial();
}

PxReal NpMaterial::getRestitution() const
{
	return mMaterial.restitution;
}

/////////////////////////////////////////////////////////////////////////////////

void NpMaterial::setFlag(PxMaterialFlag::Enum flag, bool value)
{
	if (value)
		mMaterial.flags |= flag;
	else
		mMaterial.flags &= ~PxMaterialFlags(flag);

	updateMaterial();
}

void NpMaterial::setFlags(PxMaterialFlags inFlags)
{
	mMaterial.flags = inFlags;
	updateMaterial();
}

PxMaterialFlags NpMaterial::getFlags() const
{
	return mMaterial.flags;
}

///////////////////////////////////////////////////////////////////////////////

void NpMaterial::setFrictionCombineMode(PxCombineMode::Enum x)
{
	mMaterial.setFrictionCombineMode(x);

	updateMaterial();
}

PxCombineMode::Enum NpMaterial::getFrictionCombineMode() const
{
	return mMaterial.getFrictionCombineMode();
}

///////////////////////////////////////////////////////////////////////////////

void NpMaterial::setRestitutionCombineMode(PxCombineMode::Enum x)
{
	mMaterial.setRestitutionCombineMode(x);
	updateMaterial();
}

PxCombineMode::Enum NpMaterial::getRestitutionCombineMode() const
{
	return mMaterial.getRestitutionCombineMode();
}

///////////////////////////////////////////////////////////////////////////////
