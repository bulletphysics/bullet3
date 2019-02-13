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


#ifndef PX_PHYSICS_PX_BASE
#define PX_PHYSICS_PX_BASE

/** \addtogroup common
@{
*/

#include "PxSerialFramework.h"
#include "PxCollection.h"
#include "common/PxTypeInfo.h"
#include "foundation/PxFlags.h"
#include <string.h>	// For strcmp

#if !PX_DOXYGEN
namespace physx
{
#endif

typedef PxU16 PxType;

/**
\brief Flags for PxBase.
*/
struct PxBaseFlag
{	
	enum Enum
	{
		eOWNS_MEMORY			= (1<<0),
		eIS_RELEASABLE			= (1<<1)
	};
};

typedef PxFlags<PxBaseFlag::Enum, PxU16> PxBaseFlags;
PX_FLAGS_OPERATORS(PxBaseFlag::Enum, PxU16)

/**
\brief Base class for objects that can be members of a PxCollection.

All PxBase sub-classes can be serialized.

@see PxCollection 
*/
class PxBase
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================
public:
	/**
	\brief Releases the PxBase instance, please check documentation of release in derived class.
	*/
	virtual     void                        release()										= 0;	

	/**
	\brief Returns string name of dynamic type.
	\return	Class name of most derived type of this object.
	*/
	virtual		const char*					getConcreteTypeName() const						= 0;

	/* brief Implements dynamic cast functionality. 

	Example use:
	
	if(actor->is<PxRigidDynamic>()) {...}

	\return A pointer to the specified type if object matches, otherwise NULL
	*/
	template<class T> T*					is()											{ return typeMatch<T>() ? static_cast<T*>(this) : NULL; }

	/* brief Implements dynamic cast functionality for const objects. 

	Example use:
	
	if(actor->is<PxRigidDynamic>()) {...}

	\return A pointer to the specified type if object matches, otherwise NULL
	*/
	template<class T> const T*				is() const										{ return typeMatch<T>() ? static_cast<const T*>(this) : NULL; }

	/**
	\brief	Returns concrete type of object.
	\return	PxConcreteType::Enum of serialized object

	@see PxConcreteType
	*/
	PX_FORCE_INLINE	PxType					getConcreteType() const							{ return mConcreteType;	}
				
	/**
	\brief Set PxBaseFlag	

	\param[in] flag The flag to be set
	\param[in] value The flags new value
	*/
	PX_FORCE_INLINE	void					setBaseFlag(PxBaseFlag::Enum flag, bool value)	{ mBaseFlags = value ? mBaseFlags|flag : mBaseFlags&~flag; }
	
	/**
	\brief Set PxBaseFlags	

	\param[in] inFlags The flags to be set

	@see PxBaseFlags
	*/
	PX_FORCE_INLINE	void					setBaseFlags(PxBaseFlags inFlags)				{ mBaseFlags = inFlags; }
	
	/**
	\brief Returns PxBaseFlags 

	\return	PxBaseFlags

	@see PxBaseFlags
	*/
	PX_FORCE_INLINE	PxBaseFlags				getBaseFlags() const							{ return mBaseFlags; }

	/**
	\brief Whether the object is subordinate.
	
	A class is subordinate, if it can only be instantiated in the context of another class.

	\return	Whether the class is subordinate
	
	@see PxSerialization::isSerializable
	*/
	virtual		bool						isReleasable() const							{ return mBaseFlags & PxBaseFlag::eIS_RELEASABLE; }

protected:
	/**
	\brief Constructor setting concrete type and base flags.
	*/
	PX_INLINE								PxBase(PxType concreteType, PxBaseFlags baseFlags)
												: mConcreteType(concreteType), mBaseFlags(baseFlags) {}

	/**
	\brief Deserialization constructor setting base flags.
	*/
	PX_INLINE								PxBase(PxBaseFlags baseFlags) : mBaseFlags(baseFlags) {}

	/**
	\brief Destructor.
	*/
	virtual									~PxBase()										{}

	/**
	\brief Returns whether a given type name matches with the type of this instance
	*/	
	virtual				bool				isKindOf(const char* superClass) const { return !::strcmp(superClass, "PxBase"); }

	template<class T>	bool				typeMatch() const
											{
												return PxU32(PxTypeInfo<T>::eFastTypeId)!=PxU32(PxConcreteType::eUNDEFINED) ? 
													PxU32(getConcreteType()) == PxU32(PxTypeInfo<T>::eFastTypeId) : isKindOf(PxTypeInfo<T>::name());
											}


private:
	friend				void				getBinaryMetaData_PxBase(PxOutputStream& stream);

protected:
	PxType									mConcreteType;			// concrete type identifier - see PxConcreteType.
	PxBaseFlags								mBaseFlags;				// internal flags

};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
