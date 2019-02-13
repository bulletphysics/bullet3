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

#include "PxsSimulationController.h"

#ifndef SC_SIMULATION_CONTROLLER_H
#define	SC_SIMULATION_CONTROLLER_H


namespace physx
{

class PxsHeapMemoryAllocator;

namespace Sc
{

	class SimulationController : public PxsSimulationController
	{
		PX_NOCOPY(SimulationController)
	public:
		SimulationController(PxsSimulationControllerCallback* callback): PxsSimulationController(callback)
		{
		}
	
		virtual ~SimulationController(){}
		virtual void addJoint(const PxU32 /*edgeIndex*/, Dy::Constraint* /*constraint*/, IG::IslandSim& /*islandSim*/, Ps::Array<PxU32, Ps::VirtualAllocator>& /*jointIndices*/,
		Ps::Array<PxgSolverConstraintManagerConstants, Ps::VirtualAllocator>& /*managerIter*/, PxU32 /*uniqueId*/){}
		virtual void removeJoint(const PxU32 /*edgeIndex*/, Dy::Constraint* /*constraint*/, Ps::Array<PxU32, Ps::VirtualAllocator>& /*jointIndices*/, IG::IslandSim& /*islandSim*/){}
		virtual void addShape(PxsShapeSim* /*shapeSim*/, const PxU32 /*index*/){}
		virtual void removeShape(const PxU32 /*index*/){}
		virtual void addDynamic(PxsRigidBody* /*rigidBody*/, const IG::NodeIndex& /*nodeIndex*/){}
		virtual void addDynamics(PxsRigidBody** /*rigidBody*/, const PxU32* /*nodeIndex*/, PxU32 /*nbBodies*/) {}
		virtual void addArticulation(Dy::ArticulationV* /*articulation*/, const IG::NodeIndex& /*nodeIndex*/){}
		virtual void releaseArticulation(Dy::ArticulationV* /*articulation*/) {}
		virtual void releaseDeferredArticulationIds(){}
		virtual void updateDynamic(const bool /*isArticulationLink*/, const IG::NodeIndex& /*nodeIndex*/) {}
		virtual void updateJoint(const PxU32 /*edgeIndex*/, Dy::Constraint* /*constraint*/){}
		virtual void updateBodies(PxsRigidBody** /*rigidBodies*/,  PxU32* /*nodeIndices*/, const PxU32 /*nbBodies*/) {}
		virtual void updateBody(PxsRigidBody* /*rigidBody*/, const PxU32 /*nodeIndex*/) {}
		virtual void updateBodiesAndShapes(PxBaseTask* /*continuation*/){}
		virtual void update(const PxU32 /*bitMapWordCounts*/){}
		virtual void gpuDmabackData(PxsTransformCache& /*cache*/, Bp::BoundsArray& /*boundArray*/, Cm::BitMapPinned&  /*changedAABBMgrHandles*/){}
		virtual void udpateScBodyAndShapeSim(PxsTransformCache& cache, Bp::BoundsArray& boundArray, PxBaseTask* continuation);
		virtual PxU32* getActiveBodies() { return NULL; }
		virtual PxU32* getDeactiveBodies() { return NULL; }
		virtual void** getRigidBodies() { return NULL; }
		virtual PxU32	getNbBodies() { return 0; }
		virtual PxU32	getNbFrozenShapes() { return 0; }
		virtual PxU32	getNbUnfrozenShapes() { return 0; }

		virtual PxU32*	getUnfrozenShapes() { return NULL; }
		virtual PxU32*	getFrozenShapes() { return NULL; }
		virtual PxsShapeSim** getShapeSims() { return NULL; }
		virtual PxU32	getNbShapes()	{ return 0; }

		virtual void	clear() { }
		virtual void	setBounds(Bp::BoundsArray* /*boundArray*/){}
		virtual void	reserve(const PxU32 /*nbBodies*/) {}
		
	};
}

}

#endif
