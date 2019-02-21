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

#ifndef PX_PHYSICS_NX_CONSTRAINTDESC
#define PX_PHYSICS_NX_CONSTRAINTDESC

/** \addtogroup physics
@{
*/

#include "PxPhysXConfig.h"
#include "foundation/PxFlags.h"
#include "foundation/PxVec3.h"
#include "common/PxBase.h"

#if !PX_DOXYGEN
namespace physx { namespace pvdsdk {
#endif
	class PvdDataStream;
#if !PX_DOXYGEN
}}
#endif

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxConstraintConnector;
class PxRigidActor;
class PxScene;
class PxConstraintConnector;
class PxRenderBuffer;
class PxDeletionListener;

/**
 \brief constraint row flags

 These flags configure the post-processing of constraint rows and the behavior of the solver while solving constraints
*/
struct Px1DConstraintFlag
{
	PX_CUDA_CALLABLE Px1DConstraintFlag(){}

	enum Type
	{
		eSPRING					= 1<<0,	//!< whether the constraint is a spring. Mutually exclusive with eRESTITUTION. If set, eKEEPBIAS is ignored.
		eACCELERATION_SPRING	= 1<<1,	//!< whether the constraint is a force or acceleration spring. Only valid if eSPRING is set.
		eRESTITUTION			= 1<<2,	//!< whether the restitution model should be applied to generate the target velocity. Mutually exclusive with eSPRING. If restitution causes a bounces, eKEEPBIAS is ignored
		eKEEPBIAS				= 1<<3,	//!< whether to keep the error term when solving for velocity. Ignored if restitution generates bounce, or eSPRING is set.
		eOUTPUT_FORCE			= 1<<4,	//!< whether to accumulate the force value from this constraint in the force total that is reported for the constraint and tested for breakage
		eHAS_DRIVE_LIMIT		= 1<<5,	//!< whether the constraint has a drive force limit (which will be scaled by dt unless PxConstraintFlag::eLIMITS_ARE_FORCES is set)
		eANGULAR_CONSTRAINT		= 1 << 6,//!< Whether this is an angular or linear constraint
		eDRIVE_ROW				= 1 << 7
	};
};

typedef PxFlags<Px1DConstraintFlag::Type, PxU16> Px1DConstraintFlags;
PX_FLAGS_OPERATORS(Px1DConstraintFlag::Type, PxU16)

/**
\brief constraint type hints which the solver uses to optimize constraint handling
*/
struct PxConstraintSolveHint
{
	enum Enum
	{
		eNONE					= 0,		//!< no special properties
		eACCELERATION1			= 256,		//!< a group of acceleration drive constraints with the same stiffness and drive parameters
		eSLERP_SPRING			= 258,		//!< temporary special value to identify SLERP drive rows
		eACCELERATION2			= 512,		//!< a group of acceleration drive constraints with the same stiffness and drive parameters
		eACCELERATION3			= 768,		//!< a group of acceleration drive constraints with the same stiffness and drive parameters
		eROTATIONAL_EQUALITY	= 1024,		//!< rotational equality constraints with no force limit and no velocity target
		eROTATIONAL_INEQUALITY	= 1025,		//!< rotational inequality constraints with (0, PX_MAX_FLT) force limits	
		eEQUALITY				= 2048,		//!< equality constraints with no force limit and no velocity target
		eINEQUALITY				= 2049		//!< inequality constraints with (0, PX_MAX_FLT) force limits	
	};
};

/**
\brief A constraint

A constraint is expressed as a set of 1-dimensional constraint rows which define the required constraint
on the objects' velocities. 

Each constraint is either a hard constraint or a spring. We define the velocity at the constraint to be
the quantity 

 v = body0vel.dot(lin0,ang0) - body1vel.dot(lin1, ang1)

For a hard constraint, the solver attempts to generate 

1. a set of velocities for the objects which, when integrated, respect the constraint errors:

  v + (geometricError / timestep) = velocityTarget

2. a set of velocities for the objects which respect the constraints:

  v = velocityTarget

Hard constraints support restitution: if the impact velocity exceeds the bounce threshold, then the target velocity
of the constraint will be set to restitution * -v

Alternatively, the solver can attempt to resolve the velocity constraint as an implicit spring:

  F = stiffness * -geometricError + damping * (velocityTarget - v)

where F is the constraint force or acceleration. Springs are fully implicit: that is, the force or acceleration 
is a function of the position and velocity after the solve.

All constraints support limits on the minimum or maximum impulse applied.
*/

PX_ALIGN_PREFIX(16)
struct Px1DConstraint
{
	PxVec3				linear0;				//!< linear component of velocity jacobian in world space
	PxReal				geometricError;			//!< geometric error of the constraint along this axis
	PxVec3				angular0;				//!< angular component of velocity jacobian in world space
	PxReal				velocityTarget;			//!< velocity target for the constraint along this axis

	PxVec3				linear1;				//!< linear component of velocity jacobian in world space
	PxReal				minImpulse;				//!< minimum impulse the solver may apply to enforce this constraint
	PxVec3				angular1;				//!< angular component of velocity jacobian in world space
	PxReal				maxImpulse;				//!< maximum impulse the solver may apply to enforce this constraint

	union
	{
		struct SpringModifiers
		{
			PxReal		stiffness;				//!< spring parameter, for spring constraints
			PxReal		damping;				//!< damping parameter, for spring constraints
		} spring;
		struct RestitutionModifiers
		{
			PxReal		restitution;			//!< restitution parameter for determining additional "bounce"
			PxReal		velocityThreshold;		//!< minimum impact velocity for bounce
		} bounce;
	} mods;

	PxReal				forInternalUse;			//!< for internal use only
	PxU16				flags;					//!< a set of Px1DConstraintFlags
	PxU16				solveHint;				//!< constraint optimization hint, should be an element of PxConstraintSolveHint
} 
PX_ALIGN_SUFFIX(16);


/** 
\brief Flags for determining which components of the constraint should be visualized.

@see PxConstraintVisualize
*/
struct PxConstraintVisualizationFlag
{
	enum Enum
	{
		eLOCAL_FRAMES	= 1,	//!< visualize constraint frames
		eLIMITS			= 2		//!< visualize constraint limits
	};
};

PX_ALIGN_PREFIX(16)
struct PxConstraintInvMassScale
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================

	PxReal linear0;		//!< multiplier for inverse mass of body0
	PxReal angular0;	//!< multiplier for inverse MoI of body0
	PxReal linear1;		//!< multiplier for inverse mass of body1
	PxReal angular1;	//!< multiplier for inverse MoI of body1

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxConstraintInvMassScale(){}
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxConstraintInvMassScale(PxReal lin0, PxReal ang0, PxReal lin1, PxReal ang1) : linear0(lin0), angular0(ang0), linear1(lin1), angular1(ang1){}
}
PX_ALIGN_SUFFIX(16);

/** solver constraint generation shader

This function is called by the constraint solver framework. The function must be reentrant, since it may be called simultaneously
from multiple threads, and should access only the arguments passed into it.

Developers writing custom constraints are encouraged to read the documentation in the user guide and the implementation code in PhysXExtensions.

\param[out] constraints			An array of solver constraint rows to be filled in
\param[out] bodyAWorldOffset	The origin point (offset from the position vector of bodyA's center of mass) at which the constraint is resolved. This value does not affect how constraints are solved, only the constraint force reported. 
\param[in] maxConstraints		The size of the constraint buffer. At most this many constraints rows may be written
\param[out] invMassScale		The inverse mass and inertia scales for the constraint
\param[in] constantBlock		The constant data block
\param[in] bodyAToWorld			The center of mass frame of the first constrained body (the identity transform if the first actor is static, or if a NULL actor pointer was provided for it)
\param[in] bodyBToWorld			The center of mass frame of the second constrained body (the identity transform if the second actor is static, or if a NULL actor pointer was provided for it)
\param[in] useExtendedLimits	Enables limit ranges outside of (-PI, PI)
\param[out] cAtW				The world space location of body A's joint frame (position only)
\param[out] cBtW				The world space location of body B's joint frame (position only)

\return the number of constraint rows written.
*/
typedef PxU32 (*PxConstraintSolverPrep)(Px1DConstraint* constraints,
										PxVec3& bodyAWorldOffset,
										PxU32 maxConstraints,
										PxConstraintInvMassScale& invMassScale,
										const void* constantBlock,
										const PxTransform& bodyAToWorld,
										const PxTransform& bodyBToWorld,
										bool useExtendedLimits,
										PxVec3& cAtW,
										PxVec3& cBtW);

/** solver constraint projection shader

This function is called by the constraint post-solver framework. The function must be reentrant, since it may be called simultaneously
from multiple threads and should access only the arguments passed into it.

\param[in] constantBlock	The constant data block
\param[out] bodyAToWorld	The center of mass frame of the first constrained body (the identity if the actor is static or a NULL pointer was provided for it)
\param[out] bodyBToWorld	The center of mass frame of the second constrained body (the identity if the actor is static or a NULL pointer was provided for it)
\param[in] projectToA		True if the constraint should be projected by moving the second body towards the first, false if the converse
*/
typedef void (*PxConstraintProject)(const void* constantBlock,
									PxTransform& bodyAToWorld,
									PxTransform& bodyBToWorld,
									bool projectToA);

/**
	API used to visualize details about a constraint.
*/
class PxConstraintVisualizer
{
protected:
	virtual ~PxConstraintVisualizer(){}
public:
	/** Visualize joint frames

	\param[in] parent	Parent transformation
	\param[in] child	Child transformation
	*/
	virtual void visualizeJointFrames(const PxTransform& parent, const PxTransform& child) = 0;

	/** Visualize joint linear limit

	\param[in] t0	Base transformation
	\param[in] t1	End transformation
	\param[in] value	Distance
	\param[in] active	State of the joint - active/inactive
	*/
	virtual void visualizeLinearLimit(const PxTransform& t0, const PxTransform& t1, PxReal value, bool active) = 0;

	/** Visualize joint angular limit

	\param[in] t0	Transformation for the visualization
	\param[in] lower Lower limit angle
	\param[in] upper Upper limit angle
	\param[in] active	State of the joint - active/inactive
	*/
	virtual void visualizeAngularLimit(const PxTransform& t0, PxReal lower, PxReal upper, bool active) = 0;

	/** Visualize limit cone

	\param[in] t	Transformation for the visualization
	\param[in] tanQSwingY	Tangent of the quarter Y angle 
	\param[in] tanQSwingZ	Tangent of the quarter Z angle 
	\param[in] active	State of the joint - active/inactive
	*/
	virtual void visualizeLimitCone(const PxTransform& t, PxReal tanQSwingY, PxReal tanQSwingZ, bool active) = 0;

	/** Visualize joint double cone

	\param[in] t	Transformation for the visualization
	\param[in] angle Limit angle
	\param[in] active	State of the joint - active/inactive
	*/
	virtual void visualizeDoubleCone(const PxTransform& t, PxReal angle, bool active) = 0;

	/** Visualize line

	\param[in] p0	Start position
	\param[in] p1	End postion
	\param[in] color Color
	*/
	virtual void visualizeLine(const PxVec3& p0, const PxVec3& p1, PxU32 color) = 0;
};

/** solver constraint visualization function

This function is called by the constraint post-solver framework to visualize the constraint

\param[out] visualizer		The render buffer to render to
\param[in] constantBlock	The constant data block
\param[in] body0Transform	The center of mass frame of the first constrained body (the identity if the actor is static, or a NULL pointer was provided for it)
\param[in] body1Transform	The center of mass frame of the second constrained body (the identity if the actor is static, or a NULL pointer was provided for it)
\param[in] flags			The visualization flags (PxConstraintVisualizationFlag)

@see PxRenderBuffer 
*/
typedef void (*PxConstraintVisualize)(PxConstraintVisualizer& visualizer,
									  const void* constantBlock,
									  const PxTransform& body0Transform,
									  const PxTransform& body1Transform,
									  PxU32 flags);


struct PxPvdUpdateType
{
	enum Enum
	{
		CREATE_INSTANCE,
		RELEASE_INSTANCE,
		UPDATE_ALL_PROPERTIES,
		UPDATE_SIM_PROPERTIES
	};
};

/** 

\brief This class connects a custom constraint to the SDK

This class connects a custom constraint to the SDK, and functions are called by the SDK
to query the custom implementation for specific information to pass on to the application
or inform the constraint when the application makes calls into the SDK which will update
the custom constraint's internal implementation
*/
class PxConstraintConnector
{
public:
	/** 
	when the constraint is marked dirty, this function is called at the start of the simulation
	step for the SDK to copy the constraint data block.
	*/
	virtual void*					prepareData()											= 0;

	/** 
	this function is called by the SDK to update PVD's view of it
	*/
	virtual bool					updatePvdProperties(physx::pvdsdk::PvdDataStream& pvdConnection,
												const PxConstraint* c,
												PxPvdUpdateType::Enum updateType) const		= 0;

	/** 
	When the SDK deletes a PxConstraint object this function is called by the SDK. In general
	custom constraints should not be deleted directly by applications: rather, the constraint
	should respond to a release() request by calling PxConstraint::release(), then wait for
	this call to release its own resources, so that even if the release() call occurs during
	a simulation step, the deletion of the constraint is buffered until that step completes.
	
	This function is also called when a PxConstraint object is deleted on cleanup due to 
	destruction of the PxPhysics object.
	*/
	virtual void					onConstraintRelease()									= 0;

	/** 
	This function is called by the SDK when the CoM of one of the actors is moved. Since the
	API specifies constraint positions relative to actors, and the constraint shader functions
	are supplied with coordinates relative to bodies, some synchronization is usually required
	when the application moves an object's center of mass.
	*/
	virtual void					onComShift(PxU32 actor)									= 0;

	/** 
	This function is called by the SDK when the scene origin gets shifted and allows to adjust
	custom data which contains world space transforms.

	\note If the adjustments affect constraint shader data, it is necessary to call PxConstraint::markDirty()
	to make sure that the data gets synced at the beginning of the next simulation step.

	\param[in] shift Translation vector the origin is shifted by.

	@see PxScene.shiftOrigin()
	*/
	virtual void					onOriginShift(const PxVec3& shift)						= 0;

	/**
	\brief Fetches external data for a constraint.
	
	This function is used by the SDK to acquire a reference to the owner of a constraint and a unique
	owner type ID. This information will be passed on when a breakable constraint breaks or when
	#PxConstraint::getExternalReference() is called.

	\param[out] typeID Unique type identifier of the external object. The value 0xffffffff is reserved and should not be used. Furthermore, if the PhysX extensions library is used, some other IDs are reserved already (see PxConstraintExtIDs)
	\return Reference to the external object which owns the constraint.

	@see PxConstraintInfo PxSimulationEventCallback.onConstraintBreak()
	*/
	virtual void*					getExternalReference(PxU32& typeID)						= 0;

	/**
	\brief Obtain a reference to a PxBase interface if the constraint has one.

	If the constraint does not implement the PxBase interface, it should return NULL. 
	*/
	virtual PxBase*					getSerializable()										= 0;

	/**
	\brief Obtain the shader function pointer used to prep rows for this constraint
	*/
	virtual PxConstraintSolverPrep getPrep()	const										= 0;

	/**
	\brief Obtain the pointer to the constraint's constant data
	*/
	virtual const void*				getConstantBlock()	const								= 0;

	/**
	\brief virtual destructor
	*/
	virtual							~PxConstraintConnector() {}
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
