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


#ifndef PXS_SIMULATION_CONTROLLER_H
#define PXS_SIMULATION_CONTROLLER_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxPreprocessor.h"
#include "foundation/PxTransform.h"
#include "CmBitMap.h"
#include "PsArray.h"


namespace physx
{
	namespace Dy
	{
		class Context;
		struct Constraint;
		class ArticulationV;
	}

	namespace Cm
	{
		class EventProfiler;
	}

	namespace Bp
	{
		class BoundsArray;
		class BroadPhase;
	}

	namespace IG
	{
		class SimpleIslandManager;
		class IslandSim;
		class NodeIndex;
	}

	class PxGpuDispatcher;
	class PxsTransformCache;
	class PxvNphaseImplementationContext;
	class PxBaseTask;

	struct PxsBodySim;
	struct PxsShapeSim;
	class PxsRigidBody;
	class PxsKernelWranglerManager;
	class PxsHeapMemoryAllocatorManager;

	template<typename T> class PxgIterator;
	struct PxgSolverConstraintManagerConstants;
	

	class PxsSimulationControllerCallback
	{
	public:
		virtual void updateScBodyAndShapeSim(PxBaseTask* continuation) = 0;
		virtual PxU32	getNbCcdBodies() = 0;

		virtual ~PxsSimulationControllerCallback() {}
	};


	class PxsSimulationController
	{
	public:
		PxsSimulationController(PxsSimulationControllerCallback* callback): mCallback(callback){}
		virtual ~PxsSimulationController(){}

		virtual void addJoint(const PxU32 edgeIndex, Dy::Constraint* constraint, IG::IslandSim& islandSim, Ps::Array<PxU32, Ps::VirtualAllocator>& jointIndices, 
			Ps::Array<PxgSolverConstraintManagerConstants, Ps::VirtualAllocator>& managerIter, PxU32 uniqueId) = 0;
		virtual void removeJoint(const PxU32 edgeIndex, Dy::Constraint* constraint, Ps::Array<PxU32, Ps::VirtualAllocator>& jointIndices, IG::IslandSim& islandSim) = 0;
		virtual void addShape(PxsShapeSim* shapeSim, const PxU32 index) = 0;
		virtual void removeShape(const PxU32 index) = 0;
		virtual void addDynamic(PxsRigidBody* rigidBody, const IG::NodeIndex& nodeIndex) = 0;
		virtual void addDynamics(PxsRigidBody** rigidBody, const PxU32* nodeIndex, PxU32 nbToProcess) = 0;
		virtual void addArticulation(Dy::ArticulationV* articulation, const IG::NodeIndex& nodeIndex) = 0;
		virtual void releaseArticulation(Dy::ArticulationV* articulation) = 0;
		virtual void releaseDeferredArticulationIds() = 0;
		virtual void updateDynamic(const bool isArticulationLink, const IG::NodeIndex&) = 0;
		virtual void updateJoint(const PxU32 edgeIndex, Dy::Constraint* constraint) = 0;
		virtual void updateBodies(PxsRigidBody** rigidBodies, PxU32* nodeIndices, const PxU32 nbBodies) = 0;
		virtual void updateBodiesAndShapes(PxBaseTask* continuation) = 0;
		virtual void update(const PxU32 bitMapWordCounts) = 0;
		virtual void gpuDmabackData(PxsTransformCache& cache, Bp::BoundsArray& boundArray, Cm::BitMapPinned&  changedAABBMgrHandles) = 0;
		virtual void udpateScBodyAndShapeSim(PxsTransformCache& cache, Bp::BoundsArray& boundArray, PxBaseTask* continuation) = 0;
		virtual PxU32* getActiveBodies() = 0;
		virtual PxU32* getDeactiveBodies() = 0;
		virtual void** getRigidBodies() = 0;
		virtual PxU32	getNbBodies() = 0;

		virtual PxU32*	getUnfrozenShapes() = 0;
		virtual PxU32*	getFrozenShapes() = 0;
		virtual PxsShapeSim** getShapeSims() = 0;
		virtual PxU32	getNbFrozenShapes() = 0;
		virtual PxU32	getNbUnfrozenShapes() = 0;

		virtual void	clear() = 0;
		virtual void	setBounds(Bp::BoundsArray* boundArray) = 0;
		virtual void	reserve(const PxU32 nbBodies) = 0;

	protected:
		PxsSimulationControllerCallback* mCallback;

	};

	PxsSimulationController* createSimulationController(PxsSimulationControllerCallback* callback);
}

#endif
