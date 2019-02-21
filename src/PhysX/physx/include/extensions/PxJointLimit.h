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

#ifndef PX_EXTENSIONS_JOINT_LIMIT
#define PX_EXTENSIONS_JOINT_LIMIT
/** \addtogroup extensions
  @{
*/

#include "foundation/PxMath.h"
#include "PxPhysXConfig.h"
#include "common/PxTolerancesScale.h"
#include "PxJoint.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief Describes the parameters for a joint limit. 

Limits are enabled or disabled by setting flags or other configuration parameters joints, see the
documentation for specific joint types for details.
*/
class PxJointLimitParameters
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================
public:
	/**
	\brief Controls the amount of bounce when the joint hits a limit.

	A restitution value of 1.0 causes the joint to bounce back with the velocity which it hit the limit.
	A value of zero causes the joint to stop dead.

	In situations where the joint has many locked DOFs (e.g. 5) the restitution may not be applied 
	correctly. This is due to a limitation in the solver which causes the restitution velocity to become zero 
	as the solver enforces constraints on the other DOFs.

	This limitation applies to both angular and linear limits, however it is generally most apparent with limited
	angular DOFs. Disabling joint projection and increasing the solver iteration count may improve this behavior 
	to some extent.

	Also, combining soft joint limits with joint drives driving against those limits may affect stability.

	<b>Range:</b> [0,1]<br>
	<b>Default:</b> 0.0
	*/
	PxReal restitution;

	/**
	determines the minimum impact velocity which will cause the joint to bounce
	*/
	PxReal bounceThreshold;

	/**
	\brief if greater than zero, the limit is soft, i.e. a spring pulls the joint back to the limit

	<b>Range:</b> [0, PX_MAX_F32)<br>
	<b>Default:</b> 0.0
	*/
	PxReal stiffness;

	/**
	\brief if spring is greater than zero, this is the damping of the limit spring

	<b>Range:</b> [0, PX_MAX_F32)<br>
	<b>Default:</b> 0.0
	*/
	PxReal damping;

	/**
	\brief the distance inside the limit value at which the limit will be considered to be active by the
	solver.  As this value is made larger, the limit becomes active more quickly. It thus becomes less 
	likely to violate the extents of the limit, but more expensive.
	
	The contact distance should be less than the limit angle or distance, and in the case of a pair limit,
	less than half the distance between the upper and lower bounds. Exceeding this value will result in
	the limit being active all the time.

	Making this value too small can result in jitter around the limit.

	<b>Default:</b> depends on the joint

	@see PxPhysics::getTolerancesScale()
	*/
	PxReal contactDistance;

	PxJointLimitParameters() :
		restitution		(0.0f),
		bounceThreshold	(0.0f),
		stiffness		(0.0f),
		damping			(0.0f),
		contactDistance	(0.0f)
	{
	}
	
	PxJointLimitParameters(const PxJointLimitParameters& p) :
		restitution		(p.restitution),
		bounceThreshold	(p.bounceThreshold),
		stiffness		(p.stiffness),
		damping			(p.damping),
		contactDistance	(p.contactDistance)
	{
	}	

	/**
	\brief Returns true if the current settings are valid.

	\return true if the current settings are valid
	*/
	PX_INLINE bool isValid() const
	{
		return	PxIsFinite(restitution) && restitution >= 0 && restitution <= 1 && 
			    PxIsFinite(stiffness) && stiffness >= 0 && 
			    PxIsFinite(damping) && damping >= 0 &&
				PxIsFinite(bounceThreshold) && bounceThreshold >= 0 &&
				PxIsFinite(contactDistance) && contactDistance >= 0;
	}

	PX_INLINE bool isSoft() const
	{
		return damping>0 || stiffness>0;
	}

protected:
	~PxJointLimitParameters() {}
};


/**
\brief Describes a one-sided linear limit.
*/
class PxJointLinearLimit : public PxJointLimitParameters
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================
public:
	/**
	\brief the extent of the limit. 

	<b>Range:</b> (0, PX_MAX_F32) <br>
	<b>Default:</b> PX_MAX_F32
	*/
	PxReal value;

	/**
	\brief construct a linear hard limit

	\param[in] scale		A PxTolerancesScale struct. Should be the same as used when creating the PxPhysics object.
	\param[in] extent		The extent of the limit
	\param[in] contactDist	The distance from the limit at which it becomes active. Default is 0.01f scaled by the tolerance length scale

	@see PxJointLimitParameters PxTolerancesScale
	*/
	PxJointLinearLimit(const PxTolerancesScale& scale, PxReal extent, PxReal contactDist = -1.0f)
	: value(extent)
	{
		PxJointLimitParameters::contactDistance = contactDist == -1.0f ? 0.01f*scale.length : contactDist; 
	}

	/**
	\brief construct a linear soft limit 

	\param[in] extent the extent of the limit
	\param[in] spring the stiffness and damping parameters for the limit spring

	@see PxJointLimitParameters PxTolerancesScale
	*/
	PxJointLinearLimit(PxReal extent, const PxSpring& spring) : value(extent)
	{
		stiffness = spring.stiffness;
		damping = spring.damping;
	}

	/**
	\brief Returns true if the limit is valid

	\return true if the current settings are valid
	*/
	PX_INLINE bool isValid() const
	{
		return PxJointLimitParameters::isValid() &&
			   PxIsFinite(value) && 
			   value > 0.0f;
	}
};


/**
\brief Describes a two-sided limit.
*/
class PxJointLinearLimitPair : public PxJointLimitParameters
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================
public:
	/**
	\brief the range of the limit. The upper limit must be no lower than the
	lower limit, and if they are equal the limited degree of freedom will be treated as locked.

	<b>Range:</b> See the joint on which the limit is used for details<br>
	<b>Default:</b> lower = -PX_MAX_F32/3, upper = PX_MAX_F32/3
	*/
	PxReal upper, lower;

	/**
	\brief Construct a linear hard limit pair. The lower distance value must be less than the upper distance value. 

	\param[in] scale		A PxTolerancesScale struct. Should be the same as used when creating the PxPhysics object.
	\param[in] lowerLimit	The lower distance of the limit
	\param[in] upperLimit	The upper distance of the limit
	\param[in] contactDist	The distance from the limit at which it becomes active. Default is the lesser of 0.01f scaled by the tolerance length scale, and 0.49 * (upperLimit - lowerLimit)

	@see PxJointLimitParameters PxTolerancesScale
	*/
	PxJointLinearLimitPair(const PxTolerancesScale& scale, PxReal lowerLimit = -PX_MAX_F32/3.0f, PxReal upperLimit = PX_MAX_F32/3.0f, PxReal contactDist = -1.0f) :
		upper(upperLimit),
		lower(lowerLimit)
	{
		PxJointLimitParameters::contactDistance = contactDist == -1.0f ? PxMin(scale.length * 0.01f, (upperLimit*0.49f-lowerLimit*0.49f)) : contactDist; 
		bounceThreshold = 2.0f*scale.length;
	}

	/**
	\brief construct a linear soft limit pair

	\param[in] lowerLimit	The lower distance of the limit
	\param[in] upperLimit	The upper distance of the limit
	\param[in] spring		The stiffness and damping parameters of the limit spring

	@see PxJointLimitParameters PxTolerancesScale
	*/
	PxJointLinearLimitPair(PxReal lowerLimit, PxReal upperLimit, const PxSpring& spring) :
		upper(upperLimit),
		lower(lowerLimit)
	{
		stiffness = spring.stiffness;
		damping = spring.damping;
	}

	/**
	\brief Returns true if the limit is valid.

	\return true if the current settings are valid
	*/
	PX_INLINE bool isValid() const
	{
		return PxJointLimitParameters::isValid() &&
			   PxIsFinite(upper) && PxIsFinite(lower) && upper >= lower &&
			   PxIsFinite(upper - lower);
	}
};


class PxJointAngularLimitPair : public PxJointLimitParameters
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================
public:
	/**
	\brief the range of the limit. The upper limit must be no lower than the lower limit.

	<b>Unit:</b> Angular: Radians
	<b>Range:</b> See the joint on which the limit is used for details<br>
	<b>Default:</b> lower = -PI/2, upper = PI/2
	*/
	PxReal upper, lower;

	/**
	\brief construct an angular hard limit pair. 
	
	The lower value must be less than the upper value. 

	\param[in] lowerLimit	The lower angle of the limit
	\param[in] upperLimit	The upper angle of the limit
	\param[in] contactDist	The distance from the limit at which it becomes active. Default is the lesser of 0.1 radians, and 0.49 * (upperLimit - lowerLimit)

	@see PxJointLimitParameters
	*/
	PxJointAngularLimitPair(PxReal lowerLimit, PxReal upperLimit, PxReal contactDist = -1.0f) :
		upper(upperLimit),
		lower(lowerLimit)
	{
		PxJointLimitParameters::contactDistance = contactDist ==-1.0f ? PxMin(0.1f, 0.49f*(upperLimit-lowerLimit)) : contactDist;
		bounceThreshold = 0.5f;
	}

	/**
	\brief construct an angular soft limit pair. 
	
	The lower value must be less than the upper value. 

	\param[in] lowerLimit	The lower angle of the limit
	\param[in] upperLimit	The upper angle of the limit
	\param[in] spring		The stiffness and damping of the limit spring

	@see PxJointLimitParameters
	*/
	PxJointAngularLimitPair(PxReal lowerLimit, PxReal upperLimit, const PxSpring& spring) :
		upper(upperLimit),
		lower(lowerLimit)
	{
		stiffness = spring.stiffness;
		damping = spring.damping;
	}

	/**
	\brief Returns true if the limit is valid.

	\return true if the current settings are valid
	*/
	PX_INLINE bool isValid() const
	{
		return PxJointLimitParameters::isValid() &&
			   PxIsFinite(upper) && PxIsFinite(lower) && upper >= lower;
	}
};

/**
\brief Describes an elliptical conical joint limit. Note that very small or highly elliptical limit cones may 
result in jitter.

@see PxD6Joint PxSphericalJoint
*/
class PxJointLimitCone : public PxJointLimitParameters
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================
public:
	/**
	\brief the maximum angle from the Y axis of the constraint frame.

	<b>Unit:</b> Angular: Radians
	<b>Range:</b> Angular: (0,PI)<br>
	<b>Default:</b> PI/2
	*/
	PxReal yAngle;

	/**
	\brief the maximum angle from the Z-axis of the constraint frame.

	<b>Unit:</b> Angular: Radians
	<b>Range:</b> Angular: (0,PI)<br>
	<b>Default:</b> PI/2
	*/
	PxReal zAngle;

	/**
	\brief Construct a cone hard limit. 

	\param[in] yLimitAngle	The limit angle from the Y-axis of the constraint frame
	\param[in] zLimitAngle	The limit angle from the Z-axis of the constraint frame
	\param[in] contactDist	The distance from the limit at which it becomes active. Default is the lesser of 0.1 radians, and 0.49 * the lower of the limit angles

	@see PxJointLimitParameters
	*/
	PxJointLimitCone(PxReal yLimitAngle, PxReal zLimitAngle, PxReal contactDist = -1.0f) :
		yAngle(yLimitAngle),
		zAngle(zLimitAngle)
	{
		PxJointLimitParameters::contactDistance = contactDist == -1.0f ? PxMin(0.1f, PxMin(yLimitAngle, zLimitAngle)*0.49f) : contactDist;
		bounceThreshold = 0.5f;
	}

	/**
	\brief Construct a cone soft limit. 

	\param[in] yLimitAngle	The limit angle from the Y-axis of the constraint frame
	\param[in] zLimitAngle	The limit angle from the Z-axis of the constraint frame
	\param[in] spring		The stiffness and damping of the limit spring

	@see PxJointLimitParameters
	*/
	PxJointLimitCone(PxReal yLimitAngle, PxReal zLimitAngle, const PxSpring& spring) :
		yAngle(yLimitAngle),
		zAngle(zLimitAngle)
	{
		stiffness = spring.stiffness;
		damping = spring.damping;
	}

	/**
	\brief Returns true if the limit is valid.

	\return true if the current settings are valid
	*/
	PX_INLINE bool isValid() const
	{
		return PxJointLimitParameters::isValid() &&
			   PxIsFinite(yAngle) && yAngle>0 && yAngle<PxPi && 
			   PxIsFinite(zAngle) && zAngle>0 && zAngle<PxPi;
	}
};

/**
\brief Describes a pyramidal joint limit.

@see PxD6Joint
*/
class PxJointLimitPyramid : public PxJointLimitParameters
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================
public:
	/**
	\brief the minimum angle from the Y axis of the constraint frame.

	<b>Unit:</b> Angular: Radians
	<b>Range:</b> Angular: (-PI,PI)<br>
	<b>Default:</b> -PI/2
	*/
	PxReal yAngleMin;

	/**
	\brief the maximum angle from the Y axis of the constraint frame.

	<b>Unit:</b> Angular: Radians
	<b>Range:</b> Angular: (-PI,PI)<br>
	<b>Default:</b> PI/2
	*/
	PxReal yAngleMax;

	/**
	\brief the minimum angle from the Z-axis of the constraint frame.

	<b>Unit:</b> Angular: Radians
	<b>Range:</b> Angular: (-PI,PI)<br>
	<b>Default:</b> -PI/2
	*/
	PxReal zAngleMin;

	/**
	\brief the maximum angle from the Z-axis of the constraint frame.

	<b>Unit:</b> Angular: Radians
	<b>Range:</b> Angular: (-PI,PI)<br>
	<b>Default:</b> PI/2
	*/
	PxReal zAngleMax;

	/**
	\brief Construct a pyramid hard limit. 

	\param[in] yLimitAngleMin	The minimum limit angle from the Y-axis of the constraint frame
	\param[in] yLimitAngleMax	The maximum limit angle from the Y-axis of the constraint frame
	\param[in] zLimitAngleMin	The minimum limit angle from the Z-axis of the constraint frame
	\param[in] zLimitAngleMax	The maximum limit angle from the Z-axis of the constraint frame
	\param[in] contactDist		The distance from the limit at which it becomes active. Default is the lesser of 0.1 radians, and 0.49 * the lower of the limit angles

	@see PxJointLimitParameters
	*/
	PxJointLimitPyramid(PxReal yLimitAngleMin, PxReal yLimitAngleMax, PxReal zLimitAngleMin, PxReal zLimitAngleMax, PxReal contactDist = -1.0f) :
		yAngleMin(yLimitAngleMin),
		yAngleMax(yLimitAngleMax),
		zAngleMin(zLimitAngleMin),
		zAngleMax(zLimitAngleMax)
	{
		if(contactDist == -1.0f)
		{
			const PxReal contactDistY = PxMin(0.1f, 0.49f*(yLimitAngleMax - yLimitAngleMin));
			const PxReal contactDistZ = PxMin(0.1f, 0.49f*(zLimitAngleMax - zLimitAngleMin));
			PxJointLimitParameters::contactDistance = contactDist == PxMin(contactDistY, contactDistZ);
		}
		else
		{
			PxJointLimitParameters::contactDistance = contactDist;
		}

		bounceThreshold = 0.5f;
	}

	/**
	\brief Construct a pyramid soft limit. 

	\param[in] yLimitAngleMin	The minimum limit angle from the Y-axis of the constraint frame
	\param[in] yLimitAngleMax	The maximum limit angle from the Y-axis of the constraint frame
	\param[in] zLimitAngleMin	The minimum limit angle from the Z-axis of the constraint frame
	\param[in] zLimitAngleMax	The maximum limit angle from the Z-axis of the constraint frame
	\param[in] spring			The stiffness and damping of the limit spring

	@see PxJointLimitParameters
	*/
	PxJointLimitPyramid(PxReal yLimitAngleMin, PxReal yLimitAngleMax, PxReal zLimitAngleMin, PxReal zLimitAngleMax, const PxSpring& spring) :
		yAngleMin(yLimitAngleMin),
		yAngleMax(yLimitAngleMax),
		zAngleMin(zLimitAngleMin),
		zAngleMax(zLimitAngleMax)
	{
		stiffness = spring.stiffness;
		damping = spring.damping;
	}

	/**
	\brief Returns true if the limit is valid.

	\return true if the current settings are valid
	*/
	PX_INLINE bool isValid() const
	{
		return PxJointLimitParameters::isValid() &&
				PxIsFinite(yAngleMin) && yAngleMin>-PxPi && yAngleMin<PxPi && 
				PxIsFinite(yAngleMax) && yAngleMax>-PxPi && yAngleMax<PxPi && 
				PxIsFinite(zAngleMin) && zAngleMin>-PxPi && zAngleMin<PxPi && 
				PxIsFinite(zAngleMax) && zAngleMax>-PxPi && zAngleMax<PxPi && 
				yAngleMax>=yAngleMin && zAngleMax>=zAngleMin;
	}
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
