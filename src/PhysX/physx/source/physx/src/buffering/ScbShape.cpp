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

#include "ScbShape.h"

using namespace physx;

bool Scb::Shape::setMaterialsHelper(PxMaterial* const* materials, PxU16 materialCount)
{
	PX_ASSERT(!isBuffering());

	if(materialCount == 1)
	{
		const PxU16 materialIndex = Ps::to16((static_cast<NpMaterial*>(materials[0]))->getHandle());

		mShape.setMaterialIndices(&materialIndex, 1);
	}
	else
	{
		PX_ASSERT(materialCount > 1);

		PX_ALLOCA(materialIndices, PxU16, materialCount);

		if(materialIndices)
		{
			NpMaterial::getMaterialIndices(materials, materialIndices, materialCount);
			mShape.setMaterialIndices(materialIndices, materialCount);
		}
		else
		{
			Ps::getFoundation().error(PxErrorCode::eOUT_OF_MEMORY, __FILE__, __LINE__, 
				"PxShape::setMaterials() failed. Out of memory. Call will be ignored.");
			return false;
		}
	}

	Scb::Scene* sc = getScbScene();
	if(sc)
		sc->getScScene().notifyNphaseOnUpdateShapeMaterial(mShape);

	return true;
}

void Scb::Shape::syncState()
{
	const PxU32 flags = getBufferFlags();
	if(flags)
	{
		const PxShapeFlags oldShapeFlags = mShape.getFlags();

		const Scb::ShapeBuffer&	buffer = *getBufferedData();

		Scb::Scene* scbScene = getScbScene();	// PT: can be NULL. See e.g. RbShapeTest.ReleaseShapeWithPendingUpdate UT.

		if(flags & Buf::BF_Geometry)
		{
			if(scbScene)
				scbScene->getScScene().unregisterShapeFromNphase(mShape);

			mShape.setGeometry(buffer.geometry.getGeometry());

			if(scbScene)
				scbScene->getScScene().registerShapeInNphase(mShape);

#if PX_SUPPORT_PVD
			if(getControlState() == ControlState::eIN_SCENE)
			{
				PX_ASSERT(scbScene);
				scbScene->getScenePvdClient().releaseAndRecreateGeometry(this);
			}
#endif
		}

		if(flags & Buf::BF_Material)
		{
			// PT: not sure if this is correct. Added the check for PX-800 but "getMaterialBuffer" doesn't always need the scene pointer...
			if(scbScene)
			{
				const PxU16* materialIndices = getMaterialBuffer(*scbScene, buffer);
				mShape.setMaterialIndices(materialIndices, buffer.materialCount);
				scbScene->getScScene().notifyNphaseOnUpdateShapeMaterial(mShape);
			}
			UPDATE_PVD_MATERIALS()
			// TODO: So far we did not bother to fail gracefully in the case of running out of memory. If that should change then this
			// method is somewhat problematic. The material ref counters have been adjusted at the time when the public API was called.
			// Could be that one of the old materials was deleted afterwards. The problem now is what to do if this method fails?
			// We can't adjust the material ref counts any longer since some of the old materials might have been deleted.
			// One solution could be that this class allocates an array of material pointers when the buffered method is called.
			// This array is then passed into the core object and is used by the core object, i.e., the core object does not allocate the
			// buffer itself.
		}

#ifdef USE_NEW_SYSTEM
		syncShape2Actor();
		syncSimulationFilterData();
		syncContactOffset();
		syncRestOffset();
		syncFlags();
		syncTorsionalPatchRadius();
		syncMinTorsionalPatchRadius();
#else
		flush<Buf::BF_Shape2Actor>(buffer);
		flush<Buf::BF_SimulationFilterData>(buffer);
		flush<Buf::BF_ContactOffset>(buffer);
		flush<Buf::BF_RestOffset>(buffer);
		flush<Buf::BF_Flags>(buffer);
		flush<Buf::BF_TorsionalPatchRadius>(buffer);
		flush<Buf::BF_MinTorsionalPatchRadius>(buffer);
#endif

		Sc::RigidCore* scRigidCore = NpShapeGetScRigidObjectFromScbSLOW(*this);

		if(scRigidCore) // may be NULL for exclusive shapes because of pending shape updates after buffered release of actor.
			scRigidCore->onShapeChange(mShape, Sc::ShapeChangeNotifyFlags(flags), oldShapeFlags, true);
	}

	postSyncState();
}
