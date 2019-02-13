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


#ifndef PX_PHYSICS_NX_ARTICULATION_LINK
#define PX_PHYSICS_NX_ARTICULATION_LINK
/** \addtogroup physics 
@{ */

#include "PxPhysXConfig.h"
#include "PxArticulationJoint.h"
#include "PxRigidBody.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

	class PxArticulationBase;

/**
\brief a component of an articulation that represents a rigid body

A limited subset of the properties of PxRigidDynamic are supported. In particular, sleep properties
are attributes of the articulation rather than each individual body, damping and velocity limits
are not supported, and links may not be kinematic.

@see PxArticulation PxArticulation.createLink PxArticulationJoint PxRigidBody
*/

class PxArticulationLink : public PxRigidBody
{
public:
	/**
	\brief Deletes the articulation link.
	
	\note Only a leaf articulation link can be released
	
	Do not keep a reference to the deleted instance.

	@see PxArticulation::createLink()
	*/
	virtual		void			release() = 0;


	/**
	\brief get the articulation to which this articulation link belongs. This returns the base class. The application should
	establish which articulation implementation this actually is and upcast to that type to access non-common functionality

	\return the articulation to which this link belongs
	*/
	virtual		PxArticulationBase&		getArticulation() const = 0;


	/**
	\brief Get the joint which connects this link to its parent.
	
	\return The joint connecting the link to the parent. NULL for the root link.

	@see PxArticulationJoint
	*/
	virtual		PxArticulationJointBase*	getInboundJoint() const = 0;

	/**
	\brief Get the degree of freedom of the joint which connects this link to its parent.

	\return The degree of freeedom of the joint connecting the link to the parent. 0xffffffff for the root link.

	@see PxArticulationJoint
	*/
	virtual		PxU32					getInboundJointDof() const = 0;

	/**
	\brief Get number of child links.

	\return the number of child links

	@see getChildren()
	*/
	virtual		PxU32			getNbChildren() const = 0;


	/**
	\brief Get low-level link index 

	\return low-level index
	*/
	virtual		PxU32			getLinkIndex() const = 0;

	/**
	\brief Retrieve all the child links.

	\param[out] userBuffer The buffer to receive articulation link pointers.
	\param[in] bufferSize Size of provided user buffer.
	\return Number of articulation links written to the buffer.
	\param[in] startIndex Index of first child pointer to be retrieved

	@see getNbChildren()
	*/
	virtual		PxU32			getChildren(PxArticulationLink** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const = 0;

	virtual		const char*		getConcreteTypeName() const					{	return "PxArticulationLink"; }

protected:
	PX_INLINE					PxArticulationLink(PxType concreteType, PxBaseFlags baseFlags) : PxRigidBody(concreteType, baseFlags) {}
	PX_INLINE					PxArticulationLink(PxBaseFlags baseFlags) : PxRigidBody(baseFlags)	{}
	virtual						~PxArticulationLink()	{}
	virtual		bool			isKindOf(const char* name)	const		{	return !::strcmp("PxArticulationLink", name) || PxRigidBody::isKindOf(name);		}
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
