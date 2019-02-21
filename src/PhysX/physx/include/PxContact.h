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


#ifndef PX_CONTACT_H
#define PX_CONTACT_H

#include "foundation/PxVec3.h"
#include "foundation/PxAssert.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

#if PX_VC
#pragma warning(push)
#pragma warning(disable: 4324)	// Padding was added at the end of a structure because of a __declspec(align) value.
#endif

#define PXC_CONTACT_NO_FACE_INDEX 0xffffffff

PX_ALIGN_PREFIX(16)
struct PxMassModificationProps
{
	PxReal mInvMassScale0;
	PxReal mInvInertiaScale0;
	PxReal mInvMassScale1;
	PxReal mInvInertiaScale1;
}
PX_ALIGN_SUFFIX(16);

/**
\brief Header for contact patch where all points share same material and normal
*/

PX_ALIGN_PREFIX(16)
struct PxContactPatch
{
	enum PxContactPatchFlags
	{
		eHAS_FACE_INDICES = 1,				//!< Indicates this contact stream has face indices.
		eMODIFIABLE = 2,					//!< Indicates this contact stream is modifiable.
		eFORCE_NO_RESPONSE = 4,				//!< Indicates this contact stream is notify-only (no contact response).
		eHAS_MODIFIED_MASS_RATIOS = 8,		//!< Indicates this contact stream has modified mass ratios
		eHAS_TARGET_VELOCITY = 16,			//!< Indicates this contact stream has target velocities set
		eHAS_MAX_IMPULSE = 32,				//!< Indicates this contact stream has max impulses set
		eREGENERATE_PATCHES = 64,		//!< Indicates this contact stream needs patches re-generated. 
											//!< This is required if the application modified either the contact normal or the material properties
		eCOMPRESSED_MODIFIED_CONTACT = 128
	};

	PX_ALIGN(16, PxMassModificationProps mMassModification);			//16
	/**
	\brief Contact normal
	*/
	PX_ALIGN(16, PxVec3	normal);									//28
	/**
	\brief Restitution coefficient
	*/
	PxReal	restitution;											//32

	PxReal	dynamicFriction;										//36
	PxReal	staticFriction;											//40
	PxU8	startContactIndex;										//41
	PxU8	nbContacts;												//42  //Can be a U8

	PxU8	materialFlags;											//43  //Can be a U16
	PxU8	internalFlags;											//44  //Can be a U16
	PxU16	materialIndex0;											//46  //Can be a U16
	PxU16	materialIndex1;											//48  //Can be a U16
	
	
}
PX_ALIGN_SUFFIX(16);

/**
\brief Contact point data including face (feature) indices
*/

PX_ALIGN_PREFIX(16)
struct PxContact
{
	/**
	\brief Contact point in world space
	*/
	PxVec3	contact;							//12
	/**
	\brief Separation value (negative implies penetration).
	*/
	PxReal	separation;							//16
}
PX_ALIGN_SUFFIX(16);

PX_ALIGN_PREFIX(16)
struct PxExtendedContact : public PxContact
{
	/**
	\brief Target velocity
	*/
	PX_ALIGN(16, PxVec3 targetVelocity);		//28
	/**
	\brief Maximum impulse
	*/
	PxReal	maxImpulse;							//32
}
PX_ALIGN_SUFFIX(16);

/**
\brief A modifiable contact point. This has additional fields per-contact to permit modification by user.
\note Not all fields are currently exposed to the user.
*/
PX_ALIGN_PREFIX(16)
struct PxModifiableContact : public PxExtendedContact
{
	/**
	\brief Contact normal
	*/
	PX_ALIGN(16, PxVec3	normal);					//44
	/**
	\brief Restitution coefficient
	*/
	PxReal	restitution;							//48
	
	/**
	\brief Material Flags
	*/
	PxU32	materialFlags;							//52
	
	/**
	\brief Shape A's material index
	*/
	PxU16	materialIndex0;							//54
	/**
	\brief Shape B's material index
	*/
	PxU16	materialIndex1;							//56
	/**
	\brief static friction coefficient
	*/
	PxReal	staticFriction;							//60
	/**
	\brief dynamic friction coefficient
	*/	
	PxReal dynamicFriction;							//64
}
PX_ALIGN_SUFFIX(16);

/**
\brief A class to iterate over a compressed contact stream. This supports read-only access to the various contact formats.
*/
struct PxContactStreamIterator
{
	enum StreamFormat
	{
		eSIMPLE_STREAM,
		eMODIFIABLE_STREAM,
		eCOMPRESSED_MODIFIABLE_STREAM
	};
	/**
	\brief Utility zero vector to optimize functions returning zero vectors when a certain flag isn't set.
	\note This allows us to return by reference instead of having to return by value. Returning by value will go via memory (registers -> stack -> registers), which can 
	cause performance issues on certain platforms.
	*/
	PxVec3 zero;
	/**
	\brief The patch headers.
	*/
	const PxContactPatch* patch;

	/**
	\brief The contacts
	*/
	const PxContact* contact;

	/**
	\brief The contact triangle face index
	*/
	const PxU32* faceIndice;


	/**
	\brief The total number of patches in this contact stream
	*/
	PxU32 totalPatches;
	
	/**
	\brief The total number of contact points in this stream
	*/
	PxU32 totalContacts;

	/**
	\brief The current contact index
	*/
	PxU32 nextContactIndex;
	
	/**
	\brief The current patch Index
	*/
	PxU32 nextPatchIndex;

	/*
	\brief Size of contact patch header 
	\note This varies whether the patch is modifiable or not.
	*/
	PxU32 contactPatchHeaderSize;
	/**
	\brief Contact point size
	\note This varies whether the patch has feature indices or is modifiable.
	*/
	PxU32 contactPointSize;
	/**
	\brief The stream format
	*/
	StreamFormat mStreamFormat;
	/**
	\brief Indicates whether this stream is notify-only or not.
	*/
	PxU32 forceNoResponse;

	bool pointStepped;

	PxU32 hasFaceIndices;

	/**
	\brief Constructor
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxContactStreamIterator(const PxU8* contactPatches, const PxU8* contactPoints, const PxU32* contactFaceIndices, PxU32 nbPatches, PxU32 nbContacts) 
		: zero(0.f)
	{		
		bool modify = false;
		bool compressedModify = false;
		bool response = false;
		bool indices = false; 
		
		PxU32 pointSize = 0;
		PxU32 patchHeaderSize = sizeof(PxContactPatch);

		const PxContactPatch* patches = reinterpret_cast<const PxContactPatch*>(contactPatches);

		if(patches)
		{
			modify = (patches->internalFlags & PxContactPatch::eMODIFIABLE) != 0;
			compressedModify = (patches->internalFlags & PxContactPatch::eCOMPRESSED_MODIFIED_CONTACT) != 0;
			indices = (patches->internalFlags & PxContactPatch::eHAS_FACE_INDICES) != 0;

			patch = patches;

			contact = reinterpret_cast<const PxContact*>(contactPoints);

			faceIndice = contactFaceIndices;

			pointSize = compressedModify ?  sizeof(PxExtendedContact) : modify ? sizeof(PxModifiableContact) : sizeof(PxContact);

			response = (patch->internalFlags & PxContactPatch::eFORCE_NO_RESPONSE) == 0;
		}


		mStreamFormat = compressedModify ? eCOMPRESSED_MODIFIABLE_STREAM : modify ? eMODIFIABLE_STREAM : eSIMPLE_STREAM;
		hasFaceIndices = PxU32(indices);
		forceNoResponse = PxU32(!response);

		contactPatchHeaderSize = patchHeaderSize;
		contactPointSize = pointSize;
		nextPatchIndex = 0;
		nextContactIndex = 0;
		totalContacts = nbContacts;
		totalPatches = nbPatches;
		
		pointStepped = false;
	}

	/**
	\brief Returns whether there are more patches in this stream.
	\return Whether there are more patches in this stream.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE bool hasNextPatch() const
	{
		return nextPatchIndex < totalPatches;
	}

	/**
	\brief Returns the total contact count.
	\return Total contact count.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 getTotalContactCount() const
	{
		return totalContacts;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 getTotalPatchCount() const
	{
		return totalPatches;
	}

	/**
	\brief Advances iterator to next contact patch.
	*/
	PX_CUDA_CALLABLE PX_INLINE void nextPatch()
	{
		PX_ASSERT(nextPatchIndex < totalPatches);
		if(nextPatchIndex)
		{
			if(nextContactIndex < patch->nbContacts)
			{
				PxU32 nbToStep = patch->nbContacts - this->nextContactIndex;
				contact = reinterpret_cast<const PxContact*>(reinterpret_cast<const PxU8*>(contact) + contactPointSize * nbToStep);
			}
			patch = reinterpret_cast<const PxContactPatch*>(reinterpret_cast<const PxU8*>(patch) + contactPatchHeaderSize);
		}
		nextPatchIndex++;
		nextContactIndex = 0;
	}

	/**
	\brief Returns if the current patch has more contacts.
	\return If there are more contacts in the current patch.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE bool hasNextContact() const
	{
		return nextContactIndex < (patch->nbContacts);
	}

	/**
	\brief Advances to the next contact in the patch.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE void nextContact()
	{
		PX_ASSERT(nextContactIndex < patch->nbContacts);
		if(pointStepped)
		{
			contact = reinterpret_cast<const PxContact*>(reinterpret_cast<const PxU8*>(contact) + contactPointSize);
			faceIndice++;
		}
		nextContactIndex++;
		pointStepped = true;
	}


	/**
	\brief Gets the current contact's normal
	\return The current contact's normal.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE const PxVec3& getContactNormal() const
	{
		return getContactPatch().normal;
	}

	/**
	\brief Gets the inverse mass scale for body 0.
	\return The inverse mass scale for body 0.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal getInvMassScale0() const
	{
		return patch->mMassModification.mInvMassScale0;
	}

	/**
	\brief Gets the inverse mass scale for body 1.
	\return The inverse mass scale for body 1.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal getInvMassScale1() const
	{
		return patch->mMassModification.mInvMassScale1;
	}

	/**
	\brief Gets the inverse inertia scale for body 0.
	\return The inverse inertia scale for body 0.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal getInvInertiaScale0() const
	{
		return patch->mMassModification.mInvInertiaScale0;
	}

	/**
	\brief Gets the inverse inertia scale for body 1.
	\return The inverse inertia scale for body 1.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal getInvInertiaScale1() const
	{
		return patch->mMassModification.mInvInertiaScale1;
	}

	/**
	\brief Gets the contact's max impulse.
	\return The contact's max impulse.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal getMaxImpulse() const
	{
		return mStreamFormat != eSIMPLE_STREAM ? getExtendedContact().maxImpulse : PX_MAX_REAL;
	}

	/**
	\brief Gets the contact's target velocity.
	\return The contact's target velocity.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE const PxVec3& getTargetVel() const
	{
		return mStreamFormat != eSIMPLE_STREAM ? getExtendedContact().targetVelocity : zero;
	}

	/**
	\brief Gets the contact's contact point.
	\return The contact's contact point.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE const PxVec3& getContactPoint() const
	{
		return contact->contact;
	}

	/**
	\brief Gets the contact's separation.
	\return The contact's separation.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal getSeparation() const
	{
		return contact->separation;
	}

	/**
	\brief Gets the contact's face index for shape 0.
	\return The contact's face index for shape 0.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 getFaceIndex0() const
	{
		return PXC_CONTACT_NO_FACE_INDEX;
	}

	/**
	\brief Gets the contact's face index for shape 1.
	\return The contact's face index for shape 1.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 getFaceIndex1() const
	{
		return hasFaceIndices ? *faceIndice : PXC_CONTACT_NO_FACE_INDEX;
	}

	/**
	\brief Gets the contact's static friction coefficient.
	\return The contact's static friction coefficient.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal getStaticFriction() const
	{
		return getContactPatch().staticFriction;
	}

	/**
	\brief Gets the contact's static dynamic coefficient.
	\return The contact's static dynamic coefficient.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal getDynamicFriction() const
	{
		return getContactPatch().dynamicFriction;
	}

	/**
	\brief Gets the contact's restitution coefficient.
	\return The contact's restitution coefficient.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal getRestitution() const
	{
		return getContactPatch().restitution;
	}

	/**
	\brief Gets the contact's material flags.
	\return The contact's material flags.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 getMaterialFlags() const
	{
		return getContactPatch().materialFlags;
	}

	/**
	\brief Gets the contact's material index for shape 0.
	\return The contact's material index for shape 0.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxU16 getMaterialIndex0() const
	{
		return PxU16(getContactPatch().materialIndex0);
	}

	/**
	\brief Gets the contact's material index for shape 1.
	\return The contact's material index for shape 1.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxU16 getMaterialIndex1() const
	{
		return PxU16(getContactPatch().materialIndex1);
	}

	/**
	\brief Advances the contact stream iterator to a specific contact index.
	*/
	bool advanceToIndex(const PxU32 initialIndex)
	{
		PX_ASSERT(this->nextPatchIndex == 0 && this->nextContactIndex == 0);
	
		PxU32 numToAdvance = initialIndex;

		if(numToAdvance == 0)
		{
			PX_ASSERT(hasNextPatch());
			nextPatch();
			return true;
		}
		
		while(numToAdvance)
		{
			while(hasNextPatch())
			{
				nextPatch();
				PxU32 patchSize = patch->nbContacts;
				if(numToAdvance <= patchSize)
				{
					contact = reinterpret_cast<const PxContact*>(reinterpret_cast<const PxU8*>(contact) + contactPointSize * numToAdvance);
					nextContactIndex += numToAdvance;
					return true;
				}
				else
				{
					numToAdvance -= patchSize;
				}
			}
		}
		return false;
	}

private:

	/**
	\brief Internal helper
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE const PxContactPatch& getContactPatch() const
	{
		return *static_cast<const PxContactPatch*>(patch);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE const PxExtendedContact& getExtendedContact() const
	{
		PX_ASSERT(mStreamFormat == eMODIFIABLE_STREAM || mStreamFormat == eCOMPRESSED_MODIFIABLE_STREAM);
		return *static_cast<const PxExtendedContact*>(contact);
	}

};


#if PX_VC
#pragma warning(pop)
#endif

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif
