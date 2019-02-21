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

#ifndef PX_CONTACTJOINT_H
#define PX_CONTACTJOINT_H

#include "extensions/PxJoint.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

	class PxContactJoint;

	/**
	\brief Create a distance Joint.

	\param[in] physics		The physics SDK
	\param[in] actor0		An actor to which the joint is attached. NULL may be used to attach the joint to a specific point in the world frame
	\param[in] localFrame0	The position and orientation of the joint relative to actor0
	\param[in] actor1		An actor to which the joint is attached. NULL may be used to attach the joint to a specific point in the world frame
	\param[in] localFrame1	The position and orientation of the joint relative to actor1

	@see PxContactJoint
	*/
	PxContactJoint*	PxContactJointCreate(PxPhysics& physics, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1);

	/**
	\brief a joint that maintains an upper or lower bound (or both) on the distance between two points on different objects

	@see PxContactJointCreate PxJoint
	*/

	struct PxJacobianRow
	{
		PxVec3 linear0;
		PxVec3 linear1;
		PxVec3 angular0;
		PxVec3 angular1;
		
		PxJacobianRow(){}
		
		PxJacobianRow(const PxVec3& lin0, const PxVec3& lin1, const PxVec3& ang0, const PxVec3& ang1) : 
		linear0(lin0), linear1(lin1), angular0(ang0), angular1(ang1)
		{
			
		}

		void operator *= (const PxReal scale)
		{
			linear0 *= scale;
			linear1 *= scale;
			angular0 *= scale;
			angular1 *= scale;
		}

		PxJacobianRow operator * (const PxReal scale) const
		{
			return PxJacobianRow(linear0*scale, linear1*scale, angular0*scale, angular1*scale);
		}
	};

	/**
	\brief a joint that maintains an upper or lower bound (or both) on the distance between two points on different objects

	@see PxContactJointCreate PxJoint
	*/
	class PxContactJoint : public PxJoint
	{
	public:

		/**
		\brief Set the current contact of the joint
		*/
		virtual void					setContact(const PxVec3& contact)	= 0;

		/**
		\brief Set the current contact normal of the joint
		*/
		virtual	void					setContactNormal(const PxVec3& contactNormal) = 0;

		/**
		\brief Set the current penetration of the joint
		*/
		virtual	void					setPenetration(const PxReal penetration) = 0;
		
		/**
		\brief Return the current contact of the joint
		*/
		virtual PxVec3					getContact()	const = 0;

		/**
		\brief Return the current contact normal of the joint
		*/
		virtual PxVec3					getContactNormal() const = 0;

		/**
		\brief Return the current penetration value of the joint
		*/
		virtual PxReal					getPenetration() const = 0;

		virtual	PxReal					getResititution()	const = 0;
		virtual	void					setResititution(const PxReal resititution) = 0;
		virtual PxReal					getBounceThreshold() const = 0;
		virtual void					setBounceThreshold(const PxReal bounceThreshold) = 0;

		/**
		\brief Returns string name of PxContactJoint, used for serialization
		*/
		virtual	const char*				getConcreteTypeName() const { return "PxContactJoint"; }
	
		virtual void computeJacobians(PxJacobianRow* jacobian) const = 0;
		virtual PxU32 getNbJacobianRows() const = 0;

	protected:
		//serialization

		/**
		\brief Constructor
		*/
		PX_INLINE						PxContactJoint(PxType concreteType, PxBaseFlags baseFlags) : PxJoint(concreteType, baseFlags) {}

		/**
		\brief Deserialization constructor
		*/
		PX_INLINE						PxContactJoint(PxBaseFlags baseFlags) : PxJoint(baseFlags) {}

		/**
		\brief Returns whether a given type name matches with the type of this instance
		*/
		virtual	bool					isKindOf(const char* name)	const { return !::strcmp("PxContactJoint", name) || PxJoint::isKindOf(name); }

		//~serialization
	};

#if !PX_DOXYGEN
}
#endif


#endif
