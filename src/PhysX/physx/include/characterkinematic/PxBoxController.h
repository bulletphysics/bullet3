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


#ifndef PX_PHYSICS_CCT_BOX_CONTROLLER
#define PX_PHYSICS_CCT_BOX_CONTROLLER
/** \addtogroup character
  @{
*/

#include "characterkinematic/PxCharacter.h"
#include "characterkinematic/PxController.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief Descriptor for a box character controller.

@see PxBoxController PxControllerDesc
*/
class PxBoxControllerDesc : public PxControllerDesc
{
public:
	/**
	\brief constructor sets to default.
	*/
	PX_INLINE								PxBoxControllerDesc();
	PX_INLINE virtual						~PxBoxControllerDesc() {}

	/**
	\brief copy constructor.
	*/
	PX_INLINE								PxBoxControllerDesc(const PxBoxControllerDesc&);

	/**
	\brief assignment operator.
	*/
	PX_INLINE PxBoxControllerDesc&			operator=(const PxBoxControllerDesc&);

	/**
	\brief (re)sets the structure to the default.
	*/
	PX_INLINE virtual	void				setToDefault();

	/**
	\brief returns true if the current settings are valid

	\return True if the descriptor is valid.
	*/
	PX_INLINE virtual	bool				isValid()		const;

	/**
	\brief Half height

	<b>Default:</b> 1.0
	*/
	PxF32				halfHeight;			// Half-height in the "up" direction

	/**
	\brief Half side extent

	<b>Default:</b> 0.5
	*/
	PxF32				halfSideExtent;		// Half-extent in the "side" direction

	/**
	\brief Half forward extent

	<b>Default:</b> 0.5
	*/
	PxF32				halfForwardExtent;	// Half-extent in the "forward" direction

protected:
	PX_INLINE void copy(const PxBoxControllerDesc&);
};

PX_INLINE PxBoxControllerDesc::PxBoxControllerDesc() :
	PxControllerDesc	(PxControllerShapeType::eBOX),
	halfHeight			(1.0f),
	halfSideExtent		(0.5f),
	halfForwardExtent	(0.5f)
{
}

PX_INLINE PxBoxControllerDesc::PxBoxControllerDesc(const PxBoxControllerDesc& other) : PxControllerDesc(other)
{
	copy(other);
}

PX_INLINE PxBoxControllerDesc& PxBoxControllerDesc::operator=(const PxBoxControllerDesc& other)
{
	PxControllerDesc::operator=(other);
	copy(other);
	return *this;
}

PX_INLINE void PxBoxControllerDesc::copy(const PxBoxControllerDesc& other)
{
	halfHeight			= other.halfHeight;
	halfSideExtent		= other.halfSideExtent;
	halfForwardExtent	= other.halfForwardExtent;
}

PX_INLINE void PxBoxControllerDesc::setToDefault()
{
	*this = PxBoxControllerDesc();
}

PX_INLINE bool PxBoxControllerDesc::isValid() const
{
	if(!PxControllerDesc::isValid())	return false;
	if(halfHeight<=0.0f)				return false;
	if(halfSideExtent<=0.0f)			return false;
	if(halfForwardExtent<=0.0f)			return false;
	if(stepOffset>2.0f*halfHeight)		return false;	// Prevents obvious mistakes
	return true;
}

/**
\brief Box character controller.

@see PxBoxControllerDesc PxController
*/
class PxBoxController : public PxController
{
public:

	/**
	\brief Gets controller's half height.

	\return The half height of the controller.

	@see PxBoxControllerDesc.halfHeight setHalfHeight()
	*/
	virtual		PxF32			getHalfHeight()			const	= 0;

	/**
	\brief Gets controller's half side extent.

	\return The half side extent of the controller.

	@see PxBoxControllerDesc.halfSideExtent setHalfSideExtent()
	*/
	virtual		PxF32			getHalfSideExtent()		const	= 0;

	/**
	\brief Gets controller's half forward extent.

	\return The half forward extent of the controller.

	@see PxBoxControllerDesc.halfForwardExtent setHalfForwardExtent()
	*/
	virtual		PxF32			getHalfForwardExtent()	const	= 0;

	/**
	\brief Sets controller's half height.

	\warning this doesn't check for collisions.

	\param[in] halfHeight The new half height for the controller.
	\return Currently always true.

	@see PxBoxControllerDesc.halfHeight getHalfHeight()
	*/
	virtual		bool			setHalfHeight(PxF32 halfHeight)					= 0;

	/**
	\brief Sets controller's half side extent.

	\warning this doesn't check for collisions.

	\param[in] halfSideExtent The new half side extent for the controller.
	\return Currently always true.

	@see PxBoxControllerDesc.halfSideExtent getHalfSideExtent()
	*/
	virtual		bool			setHalfSideExtent(PxF32 halfSideExtent)			= 0;

	/**
	\brief Sets controller's half forward extent.

	\warning this doesn't check for collisions.

	\param[in] halfForwardExtent The new half forward extent for the controller.
	\return Currently always true.

	@see PxBoxControllerDesc.halfForwardExtent getHalfForwardExtent()
	*/
	virtual		bool			setHalfForwardExtent(PxF32 halfForwardExtent)	= 0;

protected:
	PX_INLINE					PxBoxController()	{}
	virtual						~PxBoxController()	{}
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
