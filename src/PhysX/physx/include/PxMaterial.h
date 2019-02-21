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


#ifndef PX_PHYSICS_NXMATERIAL
#define PX_PHYSICS_NXMATERIAL
/** \addtogroup physics
@{
*/

#include "PxPhysXConfig.h"
#include "common/PxBase.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxScene;

/**
\brief Flags which control the behavior of a material.

@see PxMaterial 
*/
struct PxMaterialFlag
{
	enum Enum
	{

		/**
		If this flag is set, friction computations are always skipped between shapes with this material and any other shape.
		*/
		eDISABLE_FRICTION = 1 << 0,

		/**
		The difference between "normal" and "strong" friction is that the strong friction feature
		remembers the "friction error" between simulation steps. The friction is a force trying to
		hold objects in place (or slow them down) and this is handled in the solver. But since the
		solver is only an approximation, the result of the friction calculation can include a small
		"error" - e.g. a box resting on a slope should not move at all if the static friction is in
		action, but could slowly glide down the slope because of a small friction error in each 
		simulation step. The strong friction counter-acts this by remembering the small error and
		taking it to account during the next simulation step.

		However, in some cases the strong friction could cause problems, and this is why it is
		possible to disable the strong friction feature by setting this flag. One example is
		raycast vehicles, that are sliding fast across the surface, but still need a precise
		steering behavior. It may be a good idea to reenable the strong friction when objects
		are coming to a rest, to prevent them from slowly creeping down inclines.

		Note: This flag only has an effect if the PxMaterialFlag::eDISABLE_FRICTION bit is 0.
		*/
		eDISABLE_STRONG_FRICTION = 1 << 1,

		/**
		This flag only has an effect if PxFrictionType::ePATCH friction model is used.

		When using the patch friction model, up to 2 friction anchors are generated per patch. As the number of friction anchors
		can be smaller than the number of contacts, the normal force is accumulated over all contacts and used to compute friction
		for all anchors. Where there are more than 2 anchors, this can produce frictional behavior that is too strong (approximately 2x as strong
		as analytical models suggest). 

		This flag causes the normal force to be distributed between the friction anchors such that the total amount of friction applied does not 
		exceed the analyical results.
		*/
		eIMPROVED_PATCH_FRICTION = 1 << 2
	};
};

/**
\brief collection of set bits defined in PxMaterialFlag.

@see PxMaterialFlag
*/
typedef PxFlags<PxMaterialFlag::Enum,PxU16> PxMaterialFlags;
PX_FLAGS_OPERATORS(PxMaterialFlag::Enum,PxU16)


/**
\brief enumeration that determines the way in which two material properties will be combined to yield a friction or restitution coefficient for a collision.

When two actors come in contact with each other, they each have materials with various coefficients, but we only need a single set of coefficients for the pair.

Physics doesn't have any inherent combinations because the coefficients are determined empirically on a case by case
basis. However, simulating this with a pairwise lookup table is often impractical.

For this reason the following combine behaviors are available:

eAVERAGE
eMIN
eMULTIPLY
eMAX

The effective combine mode for the pair is maximum(material0.combineMode, material1.combineMode).

@see PxMaterial.setFrictionCombineMode() PxMaterial.getFrictionCombineMode() PxMaterial.setRestitutionCombineMode() PxMaterial.getFrictionCombineMode()
*/
struct PxCombineMode
{
	enum Enum
	{
		eAVERAGE	= 0,		//!< Average: (a + b)/2
		eMIN		= 1,		//!< Minimum: minimum(a,b)
		eMULTIPLY	= 2,		//!< Multiply: a*b
		eMAX		= 3,		//!< Maximum: maximum(a,b)
		eN_VALUES	= 4,		//!< This is not a valid combine mode, it is a sentinel to denote the number of possible values. We assert that the variable's value is smaller than this.
		ePAD_32		= 0x7fffffff //!< This is not a valid combine mode, it is to assure that the size of the enum type is big enough.
	};
};

/**
\brief Material class to represent a set of surface properties. 

@see PxPhysics.createMaterial
*/
class PxMaterial : public PxBase
{
public:

	/**
	\brief Decrements the reference count of a material and releases it if the new reference count is zero.		
	
	@see PxPhysics.createMaterial()
	*/
	virtual		void			release() = 0;

	/**
	\brief Returns the reference count of the material.

	At creation, the reference count of the material is 1. Every shape referencing this material increments the
	count by 1.	When the reference count reaches 0, and only then, the material gets destroyed automatically.

	\return the current reference count.
	*/
	virtual PxU32				getReferenceCount() const = 0;

	/**
	\brief Acquires a counted reference to a material.

	This method increases the reference count of the material by 1. Decrement the reference count by calling release()
	*/
	virtual void				acquireReference() = 0;

	/**
	\brief Sets the coefficient of dynamic friction.
	
	The coefficient of dynamic friction should be in [0, PX_MAX_F32). If set to greater than staticFriction, the effective value of staticFriction will be increased to match.

	<b>Sleeping:</b> Does <b>NOT</b> wake any actors which may be affected.

	\param[in] coef Coefficient of dynamic friction. <b>Range:</b> [0, PX_MAX_F32)

	@see getDynamicFriction()
	*/
	virtual		void			setDynamicFriction(PxReal coef) = 0;

	/**
	\brief Retrieves the DynamicFriction value.

	\return The coefficient of dynamic friction.

	@see setDynamicFriction
	*/
	virtual		PxReal			getDynamicFriction() const = 0;

	/**
	\brief Sets the coefficient of static friction
	
	The coefficient of static friction should be in the range [0, PX_MAX_F32)

	<b>Sleeping:</b> Does <b>NOT</b> wake any actors which may be affected.

	\param[in] coef Coefficient of static friction. <b>Range:</b> [0, PX_MAX_F32)

	@see getStaticFriction() 
	*/
	virtual		void			setStaticFriction(PxReal coef) = 0;

	/**
	\brief Retrieves the coefficient of static friction.
	\return The coefficient of static friction.

	@see setStaticFriction 
	*/
	virtual		PxReal			getStaticFriction() const = 0;

	/**
	\brief Sets the coefficient of restitution 
	
	A coefficient of 0 makes the object bounce as little as possible, higher values up to 1.0 result in more bounce.

	<b>Sleeping:</b> Does <b>NOT</b> wake any actors which may be affected.

	\param[in] rest Coefficient of restitution. <b>Range:</b> [0,1]

	@see getRestitution() 
	*/
	virtual		void			setRestitution(PxReal rest) = 0;

	/**
	\brief Retrieves the coefficient of restitution.     

	See #setRestitution.

	\return The coefficient of restitution.

	@see setRestitution() 
	*/
	virtual		PxReal			getRestitution() const = 0;

	/**
	\brief Raises or clears a particular material flag.
	
	See the list of flags #PxMaterialFlag

	<b>Sleeping:</b> Does <b>NOT</b> wake any actors which may be affected.

	\param[in] flag The PxMaterial flag to raise(set) or clear.

	@see getFlags() PxMaterialFlag
	*/
	virtual		void			setFlag(PxMaterialFlag::Enum flag, bool) = 0;


	/**
	\brief sets all the material flags.
	
	See the list of flags #PxMaterialFlag

	<b>Sleeping:</b> Does <b>NOT</b> wake any actors which may be affected.

	*/
	virtual		void 			setFlags( PxMaterialFlags inFlags ) = 0;

	/**
	\brief Retrieves the flags. See #PxMaterialFlag.

	\return The material flags.

	@see PxMaterialFlag setFlags()
	*/
	virtual		PxMaterialFlags	getFlags() const = 0;

	/**
	\brief Sets the friction combine mode.
	
	See the enum ::PxCombineMode .

	<b>Sleeping:</b> Does <b>NOT</b> wake any actors which may be affected.

	\param[in] combMode Friction combine mode to set for this material. See #PxCombineMode.

	@see PxCombineMode getFrictionCombineMode setStaticFriction() setDynamicFriction()
	*/
	virtual		void			setFrictionCombineMode(PxCombineMode::Enum combMode) = 0;

	/**
	\brief Retrieves the friction combine mode.
	
	See #setFrictionCombineMode.

	\return The friction combine mode for this material.

	@see PxCombineMode setFrictionCombineMode() 
	*/
	virtual		PxCombineMode::Enum	getFrictionCombineMode() const = 0;

	/**
	\brief Sets the restitution combine mode.
	
	See the enum ::PxCombineMode .

	<b>Sleeping:</b> Does <b>NOT</b> wake any actors which may be affected.

	\param[in] combMode Restitution combine mode for this material. See #PxCombineMode.

	@see PxCombineMode getRestitutionCombineMode() setRestitution()
	*/
	virtual		void			setRestitutionCombineMode(PxCombineMode::Enum combMode) = 0;

	/**
	\brief Retrieves the restitution combine mode.
	
	See #setRestitutionCombineMode.

	\return The coefficient of restitution combine mode for this material.

	@see PxCombineMode setRestitutionCombineMode getRestitution()
	*/
	virtual		PxCombineMode::Enum	getRestitutionCombineMode() const = 0;

	//public variables:
				void*			userData;	//!< user can assign this to whatever, usually to create a 1:1 relationship with a user object.

	virtual		const char*		getConcreteTypeName() const { return "PxMaterial"; }

protected:
	PX_INLINE					PxMaterial(PxType concreteType, PxBaseFlags baseFlags) : PxBase(concreteType, baseFlags), userData(NULL) {}
	PX_INLINE					PxMaterial(PxBaseFlags baseFlags) : PxBase(baseFlags) {}
	virtual						~PxMaterial() {}
	virtual		bool			isKindOf(const char* name) const { return !::strcmp("PxMaterial", name) || PxBase::isKindOf(name); }

};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
