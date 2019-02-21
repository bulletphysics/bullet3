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

#include "common/PxProfileZone.h"
#include "PxsSimpleIslandManager.h"
#include "PsSort.h"
#include "PxsContactManager.h"
#include "CmTask.h"
#include "DyVArticulation.h"


#define IG_SANITY_CHECKS 0

namespace physx
{
namespace IG
{
	ThirdPassTask::ThirdPassTask(PxU64 contextID, SimpleIslandManager& islandManager, IslandSim& islandSim) : Cm::Task(contextID), mIslandManager(islandManager), mIslandSim(islandSim)
	{
	}

	PostThirdPassTask::PostThirdPassTask(PxU64 contextID, SimpleIslandManager& islandManager) : Cm::Task(contextID), mIslandManager(islandManager)
	{
	}

	SimpleIslandManager::SimpleIslandManager(bool useEnhancedDeterminism, PxU64 contextID) : 
		mDestroyedNodes				(PX_DEBUG_EXP("mDestroyedNodes")), 
		mDestroyedEdges				(PX_DEBUG_EXP("mDestroyedEdges")), 
		mFirstPartitionEdges		(PX_DEBUG_EXP("mFirstPartitionEdges")), 
		mDestroyedPartitionEdges	(PX_DEBUG_EXP("IslandSim::mDestroyedPartitionEdges")), 
		mIslandManager				(&mFirstPartitionEdges, mEdgeNodeIndices, &mDestroyedPartitionEdges, contextID),
		mSpeculativeIslandManager	(NULL, mEdgeNodeIndices, NULL, contextID),
		mSpeculativeThirdPassTask	(contextID, *this, mSpeculativeIslandManager),
		mAccurateThirdPassTask		(contextID, *this, mIslandManager),
		mPostThirdPassTask			(contextID, *this),
		mContextID					(contextID)
{
	mFirstPartitionEdges.resize(1024);
	mMaxDirtyNodesPerFrame = useEnhancedDeterminism ? 0xFFFFFFFF : 1000u;
}

SimpleIslandManager::~SimpleIslandManager()
{
}

NodeIndex SimpleIslandManager::addRigidBody(PxsRigidBody* body, bool isKinematic, bool isActive)
{
	PxU32 handle = mNodeHandles.getHandle();
	NodeIndex nodeIndex(handle);
	mIslandManager.addRigidBody(body, isKinematic, isActive, nodeIndex);
	mSpeculativeIslandManager.addRigidBody(body, isKinematic, isActive, nodeIndex);
	return nodeIndex;
}

void SimpleIslandManager::removeNode(const NodeIndex index)
{
	PX_ASSERT(mNodeHandles.isValidHandle(index.index()));
	mDestroyedNodes.pushBack(index);
}

NodeIndex SimpleIslandManager::addArticulation(Sc::ArticulationSim* articulation, Dy::ArticulationV* llArtic, bool isActive)
{
	PxU32 handle = mNodeHandles.getHandle();
	NodeIndex nodeIndex(handle);
	mIslandManager.addArticulation(articulation, llArtic, isActive, nodeIndex);
	mSpeculativeIslandManager.addArticulation(articulation, llArtic, isActive, nodeIndex);
	return nodeIndex;
}

EdgeIndex SimpleIslandManager::addContactManager(PxsContactManager* manager, NodeIndex nodeHandle1, NodeIndex nodeHandle2, Sc::Interaction* interaction)
{
	EdgeIndex handle = mEdgeHandles.getHandle();

	PxU32 nodeIds = 2 * handle;
	if (mEdgeNodeIndices.size() == nodeIds)
	{
		PX_PROFILE_ZONE("ReserveEdges", getContextId());
		const PxU32 newSize = nodeIds + 2048;
		mEdgeNodeIndices.resize(newSize);
		mConstraintOrCm.resize(newSize);
		mInteractions.resize(newSize);
	}

	mEdgeNodeIndices[nodeIds] = nodeHandle1;
	mEdgeNodeIndices[nodeIds+1] = nodeHandle2;
	mConstraintOrCm[handle] = manager;
	mInteractions[handle] = interaction;

	mSpeculativeIslandManager.addContactManager(manager, nodeHandle1, nodeHandle2, handle);

	if (manager)
		manager->getWorkUnit().mEdgeIndex = handle;

	if(mConnectedMap.size() == handle)
	{
		mConnectedMap.resize(2 * (handle + 1));
	}
	if (mFirstPartitionEdges.capacity() == handle)
	{
		mFirstPartitionEdges.resize(2 * (handle + 1));
	}
	mConnectedMap.reset(handle);
	return handle;
}

EdgeIndex SimpleIslandManager::addConstraint(Dy::Constraint* constraint, NodeIndex nodeHandle1, NodeIndex nodeHandle2, Sc::Interaction* interaction)
{
	EdgeIndex handle = mEdgeHandles.getHandle();

	PxU32 nodeIds = 2 * handle;
	if (mEdgeNodeIndices.size() == nodeIds)
	{
		mEdgeNodeIndices.resize(2 * (mEdgeNodeIndices.size() + 2));
		mConstraintOrCm.resize(2 * (handle + 1));
		mInteractions.resize(2 * (handle + 1));
	}

	mEdgeNodeIndices[nodeIds] = nodeHandle1;
	mEdgeNodeIndices[nodeIds + 1] = nodeHandle2;

	mConstraintOrCm[handle] = constraint;

	mInteractions[handle] = interaction;

	mIslandManager.addConstraint(constraint, nodeHandle1, nodeHandle2, handle);
	mSpeculativeIslandManager.addConstraint(constraint, nodeHandle1, nodeHandle2, handle);
	if(mConnectedMap.size() == handle)
	{
		mConnectedMap.resize(2*(mConnectedMap.size()+1));
	}

	if (mFirstPartitionEdges.capacity() == handle)
	{
		mFirstPartitionEdges.resize(2 * (mFirstPartitionEdges.capacity() + 1));
	}
	mConnectedMap.set(handle);
	return handle;
}

void SimpleIslandManager::activateNode(NodeIndex index)
{
	mIslandManager.activateNode(index);
	mSpeculativeIslandManager.activateNode(index);
}

void SimpleIslandManager::deactivateNode(NodeIndex index)
{
	mIslandManager.deactivateNode(index);
	mSpeculativeIslandManager.deactivateNode(index);
}

void SimpleIslandManager::putNodeToSleep(NodeIndex index)
{
	mIslandManager.putNodeToSleep(index);
	mSpeculativeIslandManager.putNodeToSleep(index);
}

void SimpleIslandManager::removeConnection(EdgeIndex edgeIndex)
{
	if(edgeIndex == IG_INVALID_EDGE)
		return;
	mDestroyedEdges.pushBack(edgeIndex);
	mSpeculativeIslandManager.removeConnection(edgeIndex);
	if(mConnectedMap.test(edgeIndex))
	{
		mIslandManager.removeConnection(edgeIndex);
		mConnectedMap.reset(edgeIndex);
	}

	mConstraintOrCm[edgeIndex] = NULL;
	mInteractions[edgeIndex] = NULL;
}

void SimpleIslandManager::firstPassIslandGen()
{
	PX_PROFILE_ZONE("Basic.firstPassIslandGen", getContextId());
	mSpeculativeIslandManager.clearDeactivations();
	mSpeculativeIslandManager.wakeIslands();
	mSpeculativeIslandManager.processNewEdges();
	mSpeculativeIslandManager.removeDestroyedEdges();
	mSpeculativeIslandManager.processLostEdges(mDestroyedNodes, false, false, mMaxDirtyNodesPerFrame);
}

void SimpleIslandManager::additionalSpeculativeActivation()
{
	mSpeculativeIslandManager.wakeIslands2();
}

void SimpleIslandManager::secondPassIslandGen()
{
	PX_PROFILE_ZONE("Basic.secondPassIslandGen", getContextId());
	
	mIslandManager.wakeIslands();
	mIslandManager.processNewEdges();

	mIslandManager.removeDestroyedEdges();
	mIslandManager.processLostEdges(mDestroyedNodes, false, false, mMaxDirtyNodesPerFrame);

	for(PxU32 a = 0; a < mDestroyedNodes.size(); ++a)
	{
		mNodeHandles.freeHandle(mDestroyedNodes[a].index());
	}
	mDestroyedNodes.clear();
	//mDestroyedEdges.clear();
}

bool SimpleIslandManager::validateDeactivations() const
{
	//This method sanity checks the deactivations produced by third-pass island gen. Specifically, it ensures that any bodies that 
	//the speculative IG wants to deactivate are also candidates for deactivation in the accurate island gen. In practice, both should be the case. If this fails, something went wrong...

	const NodeIndex* const nodeIndices = mSpeculativeIslandManager.getNodesToDeactivate(Node::eRIGID_BODY_TYPE);
	const PxU32 nbNodesToDeactivate = mSpeculativeIslandManager.getNbNodesToDeactivate(Node::eRIGID_BODY_TYPE);

	for(PxU32 i = 0; i < nbNodesToDeactivate; ++i)
	{
		//Node is active in accurate sim => mismatch between accurate and inaccurate sim!
		const Node& node = mIslandManager.getNode(nodeIndices[i]);
		const Node& speculativeNode = mSpeculativeIslandManager.getNode(nodeIndices[i]);
		//KS - we need to verify that the bodies in the "deactivating" list are still candidates for deactivation. There are cases where they may not no longer be candidates, e.g. if the application
		//put bodies to sleep and activated them
		if(node.isActive() && !speculativeNode.isActive())
			return false;
	}
	return true;
}

void ThirdPassTask::runInternal()
{
	PX_PROFILE_ZONE("Basic.thirdPassIslandGen", mIslandSim.getContextId());
	mIslandSim.removeDestroyedEdges();
	mIslandSim.processLostEdges(mIslandManager.mDestroyedNodes, true, true, mIslandManager.mMaxDirtyNodesPerFrame);
}

void PostThirdPassTask::runInternal()
{
	for (PxU32 a = 0; a < mIslandManager.mDestroyedNodes.size(); ++a)
	{
		mIslandManager.mNodeHandles.freeHandle(mIslandManager.mDestroyedNodes[a].index());
	}
	mIslandManager.mDestroyedNodes.clear();

	for (PxU32 a = 0; a < mIslandManager.mDestroyedEdges.size(); ++a)
	{
		mIslandManager.mEdgeHandles.freeHandle(mIslandManager.mDestroyedEdges[a]);
	}
	mIslandManager.mDestroyedEdges.clear();

	PX_ASSERT(mIslandManager.validateDeactivations());
}

void SimpleIslandManager::thirdPassIslandGen(PxBaseTask* continuation)
{

	mIslandManager.clearDeactivations();

	mPostThirdPassTask.setContinuation(continuation);
	
	mSpeculativeThirdPassTask.setContinuation(&mPostThirdPassTask);
	mAccurateThirdPassTask.setContinuation(&mPostThirdPassTask);

	mSpeculativeThirdPassTask.removeReference();
	mAccurateThirdPassTask.removeReference();

	mPostThirdPassTask.removeReference();

	//PX_PROFILE_ZONE("Basic.thirdPassIslandGen", getContextId());
	//mSpeculativeIslandManager.removeDestroyedEdges();
	//mSpeculativeIslandManager.processLostEdges(mDestroyedNodes, true, true);

	//mIslandManager.removeDestroyedEdges();
	//mIslandManager.processLostEdges(mDestroyedNodes, true, true);

	
}

bool SimpleIslandManager::checkInternalConsistency()
{
	return mIslandManager.checkInternalConsistency() && mSpeculativeIslandManager.checkInternalConsistency();
}

void SimpleIslandManager::clearDestroyedEdges()
{
	mDestroyedPartitionEdges.forceSize_Unsafe(0);
}

void SimpleIslandManager::setEdgeConnected(EdgeIndex edgeIndex)
{
	if(!mConnectedMap.test(edgeIndex))
	{
		mIslandManager.addContactManager(reinterpret_cast<PxsContactManager*>(mConstraintOrCm[edgeIndex]), mEdgeNodeIndices[edgeIndex * 2], mEdgeNodeIndices[edgeIndex * 2 + 1], edgeIndex);
		mConnectedMap.set(edgeIndex);
	}
}

bool SimpleIslandManager::getIsEdgeConnected(EdgeIndex edgeIndex)
{
	return !!mConnectedMap.test(edgeIndex);
}

void SimpleIslandManager::deactivateEdge(const EdgeIndex edgeIndex)
{
	if (mFirstPartitionEdges[edgeIndex])
	{
		mDestroyedPartitionEdges.pushBack(mFirstPartitionEdges[edgeIndex]);
		mFirstPartitionEdges[edgeIndex] = NULL;
	}
}

void SimpleIslandManager::setEdgeDisconnected(EdgeIndex edgeIndex)
{
	if(mConnectedMap.test(edgeIndex))
	{
		//PX_ASSERT(!mIslandManager.getEdge(edgeIndex).isInDirtyList());
		mIslandManager.removeConnection(edgeIndex);
		mConnectedMap.reset(edgeIndex);
	}
}

void SimpleIslandManager::setEdgeRigidCM(const EdgeIndex edgeIndex, PxsContactManager* cm)
{
	mConstraintOrCm[edgeIndex] = cm;
	cm->getWorkUnit().mEdgeIndex = edgeIndex;
}

void SimpleIslandManager::clearEdgeRigidCM(const EdgeIndex edgeIndex)
{
	mConstraintOrCm[edgeIndex] = NULL;
	if (mFirstPartitionEdges[edgeIndex])
	{
		//this is the partition edges created/updated by the gpu solver
		mDestroyedPartitionEdges.pushBack(mFirstPartitionEdges[edgeIndex]);
		mFirstPartitionEdges[edgeIndex] = NULL;
	}
}

void SimpleIslandManager::setKinematic(IG::NodeIndex nodeIndex) 
{ 
	mIslandManager.setKinematic(nodeIndex); 
	mSpeculativeIslandManager.setKinematic(nodeIndex);
}

void SimpleIslandManager::setDynamic(IG::NodeIndex nodeIndex) 
{ 
	mIslandManager.setDynamic(nodeIndex); 
	mSpeculativeIslandManager.setDynamic(nodeIndex);
}

}
}

