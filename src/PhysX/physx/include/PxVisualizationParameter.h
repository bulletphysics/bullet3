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


#ifndef PX_PHYSICS_NX_DEBUG_VISUALIZATION_PARAMETER
#define PX_PHYSICS_NX_DEBUG_VISUALIZATION_PARAMETER

#include "foundation/PxPreprocessor.h"

/** \addtogroup physics
@{
*/

#if !PX_DOXYGEN
namespace physx
{
#endif

/*
NOTE: Parameters should NOT be conditionally compiled out. Even if a particular feature is not available.
Otherwise the parameter values get shifted about and the numeric values change per platform. This causes problems
when trying to serialize parameters.

New parameters should also be added to the end of the list for this reason. Also make sure to update 
eNUM_VALUES, which should be one higher than the maximum value in the enum.
*/

/**
\brief Debug visualization parameters.

#PxVisualizationParameter::eSCALE is the master switch for enabling visualization, please read the corresponding documentation
for further details.

@see PxScene.setVisualizationParameter() PxScene.getVisualizationParameter() PxScene.getRenderBuffer()
*/
struct PxVisualizationParameter
{
	enum Enum
	{
	/* RigidBody-related parameters  */

		/**
		\brief This overall visualization scale gets multiplied with the individual scales. Setting to zero ignores all visualizations. Default is 0.

		The below settings permit the debug visualization of various simulation properties. 
		The setting is either zero, in which case the property is not drawn. Otherwise it is a scaling factor
		that determines the size of the visualization widgets.

		Only objects for which visualization is turned on using setFlag(eVISUALIZATION) are visualized (see #PxActorFlag::eVISUALIZATION, #PxShapeFlag::eVISUALIZATION, ...).
		Contacts are visualized if they involve a body which is being visualized.
		Default is 0.

		Notes:
		- to see any visualization, you have to set PxVisualizationParameter::eSCALE to nonzero first.
		- the scale factor has been introduced because it's difficult (if not impossible) to come up with a
		good scale for 3D vectors. Normals are normalized and their length is always 1. But it doesn't mean
		we should render a line of length 1. Depending on your objects/scene, this might be completely invisible
		or extremely huge. That's why the scale factor is here, to let you tune the length until it's ok in
		your scene.
		- however, things like collision shapes aren't ambiguous. They are clearly defined for example by the
		triangles & polygons themselves, and there's no point in scaling that. So the visualization widgets
		are only scaled when it makes sense.

		<b>Range:</b> [0, PX_MAX_F32)<br>
		<b>Default:</b> 0
		*/
		eSCALE,

		
		/**
		\brief Visualize the world axes.
		*/
		eWORLD_AXES,
		
	/* Body visualizations */

		/**
		\brief Visualize a bodies axes.

		@see PxActor.globalPose PxActor
		*/
		eBODY_AXES,
		
		/**
		\brief Visualize a body's mass axes.

		This visualization is also useful for visualizing the sleep state of bodies. Sleeping bodies are drawn in
		black, while awake bodies are drawn in white. If the body is sleeping and part of a sleeping group, it is
		drawn in red.

		@see PxBodyDesc.massLocalPose PxActor
		*/
		eBODY_MASS_AXES,
		
		/**
		\brief Visualize the bodies linear velocity.

		@see PxBodyDesc.linearVelocity PxActor
		*/
		eBODY_LIN_VELOCITY,
		
		/**
		\brief Visualize the bodies angular velocity.

		@see PxBodyDesc.angularVelocity PxActor
		*/
		eBODY_ANG_VELOCITY,


	/* Contact visualisations */

		/**
		\brief  Visualize contact points. Will enable contact information.
		*/
		eCONTACT_POINT,
		
		/**
		\brief Visualize contact normals. Will enable contact information.
		*/
		eCONTACT_NORMAL,
		
		/**
		\brief  Visualize contact errors. Will enable contact information.
		*/
		eCONTACT_ERROR,
		
		/**
		\brief Visualize Contact forces. Will enable contact information.
		*/
		eCONTACT_FORCE,

		
		/**
		\brief Visualize actor axes.

		@see PxRigidStatic PxRigidDynamic PxArticulationLink
		*/
		eACTOR_AXES,

		
		/**
		\brief Visualize bounds (AABBs in world space)
		*/
		eCOLLISION_AABBS,
		
		/**
		\brief Shape visualization

		@see PxShape
		*/
		eCOLLISION_SHAPES,
		
		/**
		\brief Shape axis visualization

		@see PxShape
		*/
		eCOLLISION_AXES,

		/**
		\brief Compound visualization (compound AABBs in world space)
		*/
		eCOLLISION_COMPOUNDS,

		/**
		\brief Mesh & convex face normals

		@see PxTriangleMesh PxConvexMesh
		*/
		eCOLLISION_FNORMALS,
		
		/**
		\brief Active edges for meshes

		@see PxTriangleMesh
		*/
		eCOLLISION_EDGES,

		/**
		\brief Static pruning structures
		*/
		eCOLLISION_STATIC,

		/**
		\brief Dynamic pruning structures
		*/
		eCOLLISION_DYNAMIC,

		/**
		\brief Visualizes pairwise state.
		*/
		eDEPRECATED_COLLISION_PAIRS,

		/**
		\brief Joint local axes
		*/
		eJOINT_LOCAL_FRAMES,

		/** 
		\brief Joint limits
		*/
		eJOINT_LIMITS,
	
		/**
		\brief Visualize culling box
		*/
		eCULL_BOX,

		/**
		\brief MBP regions
		*/
		eMBP_REGIONS,

		/**
		\brief This is not a parameter, it just records the current number of parameters (as maximum(PxVisualizationParameter)+1) for use in loops.
		*/
		eNUM_VALUES,

		eFORCE_DWORD = 0x7fffffff
	};
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
