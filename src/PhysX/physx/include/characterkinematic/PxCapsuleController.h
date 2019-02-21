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


#ifndef PX_PHYSICS_CCT_CAPSULE_CONTROLLER
#define PX_PHYSICS_CCT_CAPSULE_CONTROLLER
/** \addtogroup character
  @{
*/

#include "characterkinematic/PxController.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

struct PxCapsuleClimbingMode
{
	enum Enum
	{
		eEASY,			//!< Standard mode, let the capsule climb over surfaces according to impact normal
		eCONSTRAINED,	//!< Constrained mode, try to limit climbing according to the step offset

		eLAST
	};
};

/**
\brief A descriptor for a capsule character controller.

@see PxCapsuleController PxControllerDesc
*/
class PxCapsuleControllerDesc : public PxControllerDesc
{
public:
	/**
	\brief constructor sets to default.
	*/
	PX_INLINE									PxCapsuleControllerDesc ();
	PX_INLINE virtual							~PxCapsuleControllerDesc () {}

	/**
	\brief copy constructor.
	*/
	PX_INLINE									PxCapsuleControllerDesc(const PxCapsuleControllerDesc&);

	/**
	\brief assignment operator.
	*/
	PX_INLINE PxCapsuleControllerDesc&			operator=(const PxCapsuleControllerDesc&);

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
	\brief The radius of the capsule

	<b>Default:</b> 0.0

	@see PxCapsuleController
	*/
	PxF32				radius;

	/**
	\brief The height of the controller

	<b>Default:</b> 0.0

	@see PxCapsuleController
	*/
	PxF32				height;

	/**
	\brief The climbing mode

	<b>Default:</b> PxCapsuleClimbingMode::eEASY

	@see PxCapsuleController
	*/
	PxCapsuleClimbingMode::Enum		climbingMode;
	
protected:
	PX_INLINE void								copy(const PxCapsuleControllerDesc&);
};

PX_INLINE PxCapsuleControllerDesc::PxCapsuleControllerDesc () : PxControllerDesc(PxControllerShapeType::eCAPSULE)
{
	radius = height = 0.0f;
	climbingMode = PxCapsuleClimbingMode::eEASY;
}

PX_INLINE PxCapsuleControllerDesc::PxCapsuleControllerDesc(const PxCapsuleControllerDesc& other) : PxControllerDesc(other)
{
	copy(other);
}

PX_INLINE PxCapsuleControllerDesc& PxCapsuleControllerDesc::operator=(const PxCapsuleControllerDesc& other)
{
	PxControllerDesc::operator=(other);
	copy(other);
	return *this;
}

PX_INLINE void PxCapsuleControllerDesc::copy(const PxCapsuleControllerDesc& other)
{
	radius			= other.radius;
	height			= other.height;
	climbingMode	= other.climbingMode;
}

PX_INLINE void PxCapsuleControllerDesc::setToDefault()
{
	*this = PxCapsuleControllerDesc();
}

PX_INLINE bool PxCapsuleControllerDesc::isValid() const
{
	if(!PxControllerDesc::isValid())	return false;
	if(radius<=0.0f)					return false;
	if(height<=0.0f)					return false;
	if(stepOffset>height+radius*2.0f)	return false;	// Prevents obvious mistakes
	return true;
}
/**
\brief A capsule character controller.

The capsule is defined as a position, a vertical height, and a radius.
The height is the distance between the two sphere centers at the end of the capsule.
In other words:

p = pos (returned by controller)<br>
h = height<br>
r = radius<br>

p = center of capsule<br>
top sphere center = p.y + h*0.5<br>
bottom sphere center = p.y - h*0.5<br>
top capsule point = p.y + h*0.5 + r<br>
bottom capsule point = p.y - h*0.5 - r<br>
*/
class PxCapsuleController : public PxController
{
public:

	/**
	\brief Gets controller's radius.

	\return The radius of the controller.

	@see PxCapsuleControllerDesc.radius setRadius()
	*/
	virtual		PxF32			getRadius() const = 0;

	/**
	\brief Sets controller's radius.

	\warning this doesn't check for collisions.

	\param[in] radius The new radius for the controller.
	\return Currently always true.

	@see PxCapsuleControllerDesc.radius getRadius()
	*/
	virtual		bool			setRadius(PxF32 radius) = 0;

	/**
	\brief Gets controller's height.

	\return The height of the capsule controller.

	@see PxCapsuleControllerDesc.height setHeight()
	*/
	virtual		PxF32			getHeight() const = 0;

	/**
	\brief Resets controller's height.

	\warning this doesn't check for collisions.

	\param[in] height The new height for the controller.
	\return Currently always true.

	@see PxCapsuleControllerDesc.height getHeight()
	*/
	virtual		bool			setHeight(PxF32 height) = 0;

	/**
	\brief Gets controller's climbing mode.

	\return The capsule controller's climbing mode.

	@see PxCapsuleControllerDesc.climbingMode setClimbingMode()
	*/
	virtual		PxCapsuleClimbingMode::Enum		getClimbingMode()	const	= 0;

	/**
	\brief Sets controller's climbing mode.

	\param[in] mode The capsule controller's climbing mode.

	@see PxCapsuleControllerDesc.climbingMode getClimbingMode()
	*/
	virtual		bool			setClimbingMode(PxCapsuleClimbingMode::Enum mode)	= 0;
	
protected:
	PX_INLINE					PxCapsuleController()	{}
	virtual						~PxCapsuleController()	{}
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
