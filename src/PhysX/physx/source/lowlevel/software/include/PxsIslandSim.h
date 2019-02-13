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

#ifndef PXS_ISLAND_SIM_H
#define PXS_ISLAND_SIM_H

#include "CmPhysXCommon.h"
#include "foundation/PxAssert.h"
#include "PsArray.h"
#include "CmBitMap.h"
#include "CmPriorityQueue.h"
#include "CmBlockArray.h"
#include "PxsIslandNodeIndex.h"

namespace physx
{

namespace Dy
{
	struct Constraint;
	class ArticulationV;
}

namespace Sc
{
	class ArticulationSim;
}

class PxsContactManager;
class PxsRigidBody;

struct PartitionEdge;

namespace IG
{

//This index is 
#define IG_INVALID_ISLAND 0xFFFFFFFFu
#define IG_INVALID_EDGE 0xFFFFFFFFu
#define IG_INVALID_LINK 0xFFu

typedef PxU32 IslandId;
typedef PxU32 EdgeIndex;
typedef PxU32 EdgeInstanceIndex;

class IslandSim;

struct Edge
{
	//Edge instances can be implicitly calculated based on this edge index, which is an offset into the array of edges.
	//From that, the child edge index is simply the 
	//The constraint or contact referenced by this edge

	enum EdgeType
	{
		eCONTACT_MANAGER,
		eCONSTRAINT,
		eEDGE_TYPE_COUNT
	};
	

	enum EdgeState
	{
		eINSERTED			=1<<0,
		ePENDING_DESTROYED	=1<<1,
		eACTIVE				=1<<2,
		eIN_DIRTY_LIST		=1<<3,
		eDESTROYED			=1<<4,
		eREPORT_ONLY_DESTROY=1<<5,
		eACTIVATING			=1<<6
	};


	//NodeIndex mNode1, mNode2;
	EdgeType mEdgeType;
	PxU16 mEdgeState;
	
	EdgeIndex mNextIslandEdge, mPrevIslandEdge;

	

	PX_FORCE_INLINE void setInserted() { mEdgeState |= (eINSERTED); }
	
	PX_FORCE_INLINE void clearInserted() { mEdgeState &= (~eINSERTED); }
	
	PX_FORCE_INLINE void clearDestroyed() { mEdgeState &=(~eDESTROYED);}
	PX_FORCE_INLINE void setPendingDestroyed() { mEdgeState |= ePENDING_DESTROYED; }
	PX_FORCE_INLINE void clearPendingDestroyed() { mEdgeState &= (~ePENDING_DESTROYED); }
	PX_FORCE_INLINE void activateEdge() { mEdgeState |= eACTIVE; }
	PX_FORCE_INLINE void deactivateEdge() { mEdgeState &= (~eACTIVE); }
	PX_FORCE_INLINE void markInDirtyList() { mEdgeState |= (eIN_DIRTY_LIST); }
	PX_FORCE_INLINE void clearInDirtyList() { mEdgeState &= (~eIN_DIRTY_LIST); }
	PX_FORCE_INLINE void setReportOnlyDestroy() { mEdgeState |= (eREPORT_ONLY_DESTROY); }
	PX_FORCE_INLINE void clearReportOnlyDestroy() { mEdgeState &= (~eREPORT_ONLY_DESTROY); }
public:
	Edge() : mEdgeType(Edge::eCONTACT_MANAGER), mEdgeState(eDESTROYED),
		mNextIslandEdge(IG_INVALID_EDGE), mPrevIslandEdge(IG_INVALID_EDGE)
	{
	}
	PX_FORCE_INLINE bool isInserted() const { return !!(mEdgeState & eINSERTED);}
	PX_FORCE_INLINE bool isDestroyed() const { return !!(mEdgeState & eDESTROYED); }
	PX_FORCE_INLINE bool isPendingDestroyed() const { return !!(mEdgeState & ePENDING_DESTROYED); }
	PX_FORCE_INLINE bool isActive() const	{ return !!(mEdgeState & eACTIVE); }
	PX_FORCE_INLINE bool isInDirtyList() const { return !!(mEdgeState & eIN_DIRTY_LIST); }
	PX_FORCE_INLINE EdgeType getEdgeType() const { return mEdgeType; }
	//PX_FORCE_INLINE const NodeIndex getIndex1() const { return mNode1; }
	//PX_FORCE_INLINE const NodeIndex getIndex2() const { return mNode2; }
	PX_FORCE_INLINE bool isReportOnlyDestroy() { return !!(mEdgeState & eREPORT_ONLY_DESTROY); }
};

struct EdgeInstance
{
	EdgeInstanceIndex mNextEdge, mPrevEdge; //The next edge instance in this node's list of edge instances

	EdgeInstance() : mNextEdge(IG_INVALID_EDGE), mPrevEdge(IG_INVALID_EDGE)
	{
	}
};

template<typename Handle>
class HandleManager
{
	Ps::Array<Handle> mFreeHandles;
	Handle mCurrentHandle;

public:

	HandleManager() : mFreeHandles(PX_DEBUG_EXP("FreeHandles")), mCurrentHandle(0)
	{
	}

	~HandleManager(){}

	Handle getHandle()
	{
		if(mFreeHandles.size())
		{
			Handle handle = mFreeHandles.popBack();
			PX_ASSERT(isValidHandle(handle));
			return handle;
		}
		return mCurrentHandle++;				
	}

	bool isNotFreeHandle(Handle handle)
	{
		for(PxU32 a = 0; a < mFreeHandles.size(); ++a)
		{
			if(mFreeHandles[a] == handle)
				return false;
		}
		return true;
	}

	void freeHandle(Handle handle)
	{
		PX_ASSERT(isValidHandle(handle));
		PX_ASSERT(isNotFreeHandle(handle));
		if(handle == mCurrentHandle)
			mCurrentHandle--;
		else
			mFreeHandles.pushBack(handle);
	}

	bool isValidHandle(Handle handle)
	{
		return handle < mCurrentHandle;
	}

	PX_FORCE_INLINE PxU32 getTotalHandles() const { return mCurrentHandle; }
};

class Node
{

public:
	enum NodeType
	{
		eRIGID_BODY_TYPE,
		eARTICULATION_TYPE,
		eTYPE_COUNT
	};
	enum State
	{
		eREADY_FOR_SLEEPING = 1u << 0,				//! Ready to go to sleep
		eACTIVE				= 1u << 1,				//! Active
		eKINEMATIC			= 1u << 2,				//! Kinematic
		eDELETED			= 1u << 3,				//! Is pending deletion
		eDIRTY				= 1u << 4,				//! Is dirty (i.e. lost a connection)
		eACTIVATING			= 1u << 5,				//! Is in the activating list
		eDEACTIVATING		= 1u << 6				//! It is being forced to deactivate this frame
	};
	EdgeInstanceIndex mFirstEdgeIndex;

	PxU8 mFlags;
	PxU8 mType;
	PxU16 mStaticTouchCount;
	//PxU32 mActiveNodeIndex; //! Look-up for this node in the active nodes list, activating list or deactivating list...

	NodeIndex mNextNode, mPrevNode;

	//A counter for the number of active references to this body. Whenever an edge is activated, this is incremented. 
	//Whenver an edge is deactivated, this is decremented. This is used for kinematic bodies to determine if they need
	//to be in the active kinematics list
	PxU32 mActiveRefCount;
	

	//A node can correspond with either a rigid body or an articulation or softBody 
	union
	{
		PxsRigidBody*				mRigidBody;
		Dy::ArticulationV*			mLLArticulation;
	};



	PX_FORCE_INLINE Node() : mFirstEdgeIndex(IG_INVALID_EDGE), mFlags(eDELETED), mType(eRIGID_BODY_TYPE),
	 mStaticTouchCount(0), mActiveRefCount(0), mRigidBody(NULL)
	{
	}

	PX_FORCE_INLINE ~Node() {}

	PX_FORCE_INLINE void reset()
	{
		mFirstEdgeIndex = IG_INVALID_EDGE;
		mFlags = eDELETED;
		mRigidBody = NULL;
		mActiveRefCount = 0;
		mStaticTouchCount = 0;
	}

	PX_FORCE_INLINE void setRigidBody(PxsRigidBody* body) { mRigidBody = body; }

	PX_FORCE_INLINE PxsRigidBody* getRigidBody() const { return mRigidBody; }

	PX_FORCE_INLINE Dy::ArticulationV* getArticulation() const { return mLLArticulation; }


	PX_FORCE_INLINE void setActive() { mFlags |= eACTIVE; }
	PX_FORCE_INLINE void clearActive() { mFlags &= ~eACTIVE; }

	PX_FORCE_INLINE void setActivating() { mFlags |= eACTIVATING; }
	PX_FORCE_INLINE void clearActivating() { mFlags &= ~eACTIVATING; }

	PX_FORCE_INLINE void setDeactivating() { mFlags |= eDEACTIVATING; }
	PX_FORCE_INLINE void clearDeactivating() { mFlags &= (~eDEACTIVATING); }


	//Activates a body/node.
	PX_FORCE_INLINE void setIsReadyForSleeping() { mFlags |= eREADY_FOR_SLEEPING; }

	PX_FORCE_INLINE void clearIsReadyForSleeping(){	mFlags &= (~eREADY_FOR_SLEEPING);}

	PX_FORCE_INLINE void setIsDeleted(){mFlags |= eDELETED;	}

	PX_FORCE_INLINE void setKinematicFlag()	{PX_ASSERT(!isKinematic());	mFlags |= eKINEMATIC;}

	PX_FORCE_INLINE void clearKinematicFlag(){ PX_ASSERT(isKinematic()); mFlags &= (~eKINEMATIC);}

	PX_FORCE_INLINE void markDirty(){mFlags |= eDIRTY;}

	PX_FORCE_INLINE void clearDirty(){mFlags &= (~eDIRTY);}

public:

	PX_FORCE_INLINE bool isActive() const { return !!(mFlags & eACTIVE); }

	PX_FORCE_INLINE bool isActiveOrActivating() const { return !!(mFlags & (eACTIVE | eACTIVATING)); }

	PX_FORCE_INLINE bool isActivating() const { return !!(mFlags & eACTIVATING); }

	PX_FORCE_INLINE bool isDeactivating() const { return !!(mFlags & eDEACTIVATING); }

	PX_FORCE_INLINE bool isKinematic() const { return !!(mFlags & eKINEMATIC); }

	PX_FORCE_INLINE bool isDeleted() const { return !!(mFlags & eDELETED); }

	PX_FORCE_INLINE bool isDirty() const { return !!(mFlags & eDIRTY); }

	PX_FORCE_INLINE bool isReadyForSleeping() const { return !!(mFlags & eREADY_FOR_SLEEPING); }

	PX_FORCE_INLINE NodeType getNodeType() const { return NodeType(mType); }

	friend class SimpleIslandManager;

};

struct Island
{
	NodeIndex mRootNode;
	NodeIndex mLastNode;
	PxU32 mSize[Node::eTYPE_COUNT];
	PxU32 mActiveIndex;

	EdgeIndex mFirstEdge[Edge::eEDGE_TYPE_COUNT], mLastEdge[Edge::eEDGE_TYPE_COUNT];
	PxU32 mEdgeCount[Edge::eEDGE_TYPE_COUNT];

	Island() : mActiveIndex(IG_INVALID_ISLAND)
	{
		for(PxU32 a = 0; a < Edge::eEDGE_TYPE_COUNT; ++a)
		{
			mFirstEdge[a] = IG_INVALID_EDGE;
			mLastEdge[a] = IG_INVALID_EDGE;
			mEdgeCount[a] = 0;
		}

		for(PxU32 a = 0; a < Node::eTYPE_COUNT; ++a)
		{
			mSize[a] = 0;
		}
	}
};

struct TraversalState
{
	NodeIndex mNodeIndex;
	PxU32 mCurrentIndex;
	PxU32 mPrevIndex;
	PxU32 mDepth;

	TraversalState()
	{
	}

	TraversalState(NodeIndex nodeIndex, PxU32 currentIndex, PxU32 prevIndex, PxU32 depth) : 
						mNodeIndex(nodeIndex), mCurrentIndex(currentIndex), mPrevIndex(prevIndex), mDepth(depth)
	{
	}
};

struct QueueElement
{
	TraversalState* mState;
	PxU32 mHopCount;

	QueueElement()
	{
	}

	QueueElement(TraversalState* state, PxU32 hopCount) : mState(state), mHopCount(hopCount)
	{
	}
};

struct NodeComparator
{
	NodeComparator()
	{
	}

	bool operator() (const QueueElement& node0, const QueueElement& node1) const
	{
		return node0.mHopCount < node1.mHopCount;
	}
private:
	NodeComparator& operator = (const NodeComparator&);
};


class IslandSim
{
	HandleManager<IslandId> mIslandHandles;					//! Handle manager for islands

	Ps::Array<Node> mNodes;									//! The nodes used in the constraint graph
	Ps::Array<PxU32> mActiveNodeIndex;						//! The active node index for each node
	Cm::BlockArray<Edge> mEdges;
	Cm::BlockArray<EdgeInstance> mEdgeInstances;			//! Edges used to connect nodes in the constraint graph
	Ps::Array<Island> mIslands;								//! The array of islands
	Ps::Array<PxU32> mIslandStaticTouchCount;				//! Array of static touch counts per-island


	Ps::Array<NodeIndex> mActiveNodes[Node::eTYPE_COUNT];			//! An array of active nodes
	Ps::Array<NodeIndex> mActiveKinematicNodes;						//! An array of active or referenced kinematic nodes
	Ps::Array<EdgeIndex> mActivatedEdges[Edge::eEDGE_TYPE_COUNT];	//! An array of active edges

	PxU32 mActiveEdgeCount[Edge::eEDGE_TYPE_COUNT];
	
	Ps::Array<PxU32> mHopCounts;							//! The observed number of "hops" from a given node to its root node. May be inaccurate but used to accelerate searches.
	Ps::Array<NodeIndex> mFastRoute;						//! The observed last route from a given node to the root node. We try the fast route (unless its broken) before trying others.

	Ps::Array<IslandId> mIslandIds;							//! The array of per-node island ids
	
	Cm::BitMap mIslandAwake;								//! Indicates whether an island is awake or not

	Cm::BitMap mActiveContactEdges;

	//An array of active islands
	Ps::Array<IslandId> mActiveIslands;

	PxU32 mInitialActiveNodeCount[Edge::eEDGE_TYPE_COUNT];

	Ps::Array<NodeIndex> mNodesToPutToSleep[Node::eTYPE_COUNT];

	//Input to this frame's island management (changed nodes/edges)

	//Input list of changes observed this frame. If there no changes, no work to be done.
	Ps::Array<EdgeIndex> mDirtyEdges[Edge::eEDGE_TYPE_COUNT];
	//Dirty nodes. These nodes lost at least one connection so we need to recompute islands from these nodes
	//Ps::Array<NodeIndex> mDirtyNodes;
	Cm::BitMap mDirtyMap;
	PxU32 mLastMapIndex;

	//An array of nodes to activate
	Ps::Array<NodeIndex> mActivatingNodes;
	Ps::Array<EdgeIndex> mDestroyedEdges;
	Ps::Array<IslandId> mTempIslandIds;

	
	//Temporary, transient data used for traversals. TODO - move to PxsSimpleIslandManager. Or if we keep it here, we can 
	//process multiple island simulations in parallel
	Cm::PriorityQueue<QueueElement, NodeComparator> 
		mPriorityQueue;										//! Priority queue used for graph traversal
	Ps::Array<TraversalState> mVisitedNodes;				//! The list of nodes visited in the current traversal
	Cm::BitMap mVisitedState;								//! Indicates whether a node has been visited
	Ps::Array<EdgeIndex> mIslandSplitEdges[Edge::eEDGE_TYPE_COUNT];

	Ps::Array<EdgeIndex> mDeactivatingEdges[Edge::eEDGE_TYPE_COUNT];

	Ps::Array<PartitionEdge*>* mFirstPartitionEdges;
	Cm::BlockArray<NodeIndex>& mEdgeNodeIndices;
	Ps::Array<physx::PartitionEdge*>* mDestroyedPartitionEdges;

	PxU32* mNpIndexPtr;
	
	PxU64	mContextId;

public:

	IslandSim(Ps::Array<PartitionEdge*>* firstPartitionEdges, Cm::BlockArray<NodeIndex>& edgeNodeIndices, Ps::Array<PartitionEdge*>* destroyedPartitionEdges, PxU64 contextID);
	~IslandSim() {}

	void resize(const PxU32 nbNodes, const PxU32 nbContactManagers, const PxU32 nbConstraints);

	void addRigidBody(PxsRigidBody* body, bool isKinematic, bool isActive, NodeIndex nodeIndex);

	void addArticulation(Sc::ArticulationSim* articulation, Dy::ArticulationV* llArtic, bool isActive, NodeIndex nodeIndex);

	void addContactManager(PxsContactManager* manager, NodeIndex nodeHandle1, NodeIndex nodeHandle2, EdgeIndex handle);

	void addConstraint(Dy::Constraint* constraint, NodeIndex nodeHandle1, NodeIndex nodeHandle2, EdgeIndex handle);

	void activateNode(NodeIndex index);
	void deactivateNode(NodeIndex index);
	void putNodeToSleep(NodeIndex index);

	void removeConnection(EdgeIndex edgeIndex);

	PX_FORCE_INLINE PxU32 getNbNodes() const { return mNodes.size(); }

	PX_FORCE_INLINE PxU32 getNbActiveNodes(Node::NodeType type) const { return mActiveNodes[type].size(); }

	PX_FORCE_INLINE const NodeIndex* getActiveNodes(Node::NodeType type) const { return mActiveNodes[type].begin(); }

	PX_FORCE_INLINE PxU32 getNbActiveKinematics() const { return mActiveKinematicNodes.size(); }

	PX_FORCE_INLINE const NodeIndex* getActiveKinematics() const { return mActiveKinematicNodes.begin(); }

	PX_FORCE_INLINE PxU32 getNbNodesToActivate(Node::NodeType type) const { return mActiveNodes[type].size() - mInitialActiveNodeCount[type]; }

	PX_FORCE_INLINE const NodeIndex* getNodesToActivate(Node::NodeType type) const { return mActiveNodes[type].begin() + mInitialActiveNodeCount[type]; }

	PX_FORCE_INLINE PxU32 getNbNodesToDeactivate(Node::NodeType type) const { return mNodesToPutToSleep[type].size(); }

	PX_FORCE_INLINE const NodeIndex* getNodesToDeactivate(Node::NodeType type) const { return mNodesToPutToSleep[type].begin(); }

	PX_FORCE_INLINE PxU32 getNbActivatedEdges(Edge::EdgeType type) const { return mActivatedEdges[type].size(); } 

	PX_FORCE_INLINE const EdgeIndex* getActivatedEdges(Edge::EdgeType type) const { return mActivatedEdges[type].begin(); }

	PX_FORCE_INLINE PxU32 getNbActiveEdges(Edge::EdgeType type) const { return mActiveEdgeCount[type]; }

	PX_FORCE_INLINE PartitionEdge* getFirstPartitionEdge(IG::EdgeIndex edgeIndex) const { return (*mFirstPartitionEdges)[edgeIndex]; }
	PX_FORCE_INLINE void setFirstPartitionEdge(IG::EdgeIndex edgeIndex, PartitionEdge* partitionEdge)  { (*mFirstPartitionEdges)[edgeIndex] = partitionEdge; }

	//PX_FORCE_INLINE const EdgeIndex* getActiveEdges(Edge::EdgeType type) const { return mActiveEdges[type].begin(); }

	PX_FORCE_INLINE PxsRigidBody* getRigidBody(NodeIndex nodeIndex) const
	{
		const Node& node = mNodes[nodeIndex.index()];
		PX_ASSERT(node.mType == Node::eRIGID_BODY_TYPE);
		return node.mRigidBody;
	}

	PX_FORCE_INLINE Dy::ArticulationV* getLLArticulation(NodeIndex nodeIndex) const
	{
		const Node& node = mNodes[nodeIndex.index()];
		PX_ASSERT(node.mType == Node::eARTICULATION_TYPE);
		return node.mLLArticulation;
	}

	PX_FORCE_INLINE void clearDeactivations()
	{
		mNodesToPutToSleep[0].forceSize_Unsafe(0);
		mNodesToPutToSleep[1].forceSize_Unsafe(0);

		mDeactivatingEdges[0].forceSize_Unsafe(0);
		mDeactivatingEdges[1].forceSize_Unsafe(0);
	}

	PX_FORCE_INLINE const Island& getIsland(IG::IslandId islandIndex) const { return mIslands[islandIndex]; }

	PX_FORCE_INLINE PxU32 getNbActiveIslands() const { return mActiveIslands.size(); }
	PX_FORCE_INLINE const IslandId* getActiveIslands() const { return mActiveIslands.begin(); }

	PX_FORCE_INLINE PxU32 getNbDeactivatingEdges(const IG::Edge::EdgeType edgeType) const { return mDeactivatingEdges[edgeType].size(); }
	PX_FORCE_INLINE const EdgeIndex* getDeactivatingEdges(const IG::Edge::EdgeType edgeType) const { return mDeactivatingEdges[edgeType].begin(); }

	PX_FORCE_INLINE PxU32 getNbDestroyedEdges() const { return mDestroyedEdges.size(); }
	PX_FORCE_INLINE const EdgeIndex* getDestroyedEdges() const { return mDestroyedEdges.begin(); }

	PX_FORCE_INLINE PxU32 getNbDestroyedPartitionEdges() const { return mDestroyedPartitionEdges->size(); }
	PX_FORCE_INLINE const PartitionEdge*const * getDestroyedPartitionEdges() const { return mDestroyedPartitionEdges->begin(); }
	PX_FORCE_INLINE PartitionEdge** getDestroyedPartitionEdges() { return mDestroyedPartitionEdges->begin(); }

	PX_FORCE_INLINE PxU32 getNbDirtyEdges(IG::Edge::EdgeType type) const { return mDirtyEdges[type].size(); }
	PX_FORCE_INLINE const EdgeIndex* getDirtyEdges(IG::Edge::EdgeType type) const { return mDirtyEdges[type].begin(); }

	PX_FORCE_INLINE const Edge& getEdge(const EdgeIndex edgeIndex) const { return mEdges[edgeIndex]; }

	PX_FORCE_INLINE Edge& getEdge(const EdgeIndex edgeIndex) { return mEdges[edgeIndex]; }

	PX_FORCE_INLINE const Node& getNode(const NodeIndex& nodeIndex) const { return mNodes[nodeIndex.index()]; }

	PX_FORCE_INLINE const Island& getIsland(const NodeIndex& nodeIndex) const { PX_ASSERT(mIslandIds[nodeIndex.index()] != IG_INVALID_ISLAND); return mIslands[mIslandIds[nodeIndex.index()]]; }

	PX_FORCE_INLINE PxU32 getIslandStaticTouchCount(const NodeIndex& nodeIndex) const { PX_ASSERT(mIslandIds[nodeIndex.index()] != IG_INVALID_ISLAND); return mIslandStaticTouchCount[mIslandIds[nodeIndex.index()]]; }

	PX_FORCE_INLINE const Cm::BitMap& getActiveContactManagerBitmap() const { return mActiveContactEdges; }

	PX_FORCE_INLINE PxU32 getActiveNodeIndex(const NodeIndex& nodeIndex) const { PxU32 activeNodeIndex = mActiveNodeIndex[nodeIndex.index()]; return activeNodeIndex;}

	PX_FORCE_INLINE const PxU32* getActiveNodeIndex() const { return mActiveNodeIndex.begin(); }

	PX_FORCE_INLINE PxU32 getNbActiveNodeIndex() const { return mActiveNodeIndex.size(); }

	void setKinematic(IG::NodeIndex nodeIndex);

	void setDynamic(IG::NodeIndex nodeIndex);

	PX_FORCE_INLINE void setEdgeNodeIndexPtr(PxU32* ptr) { mNpIndexPtr = ptr; }

	PX_FORCE_INLINE NodeIndex getNodeIndex1(IG::EdgeIndex index) const { return mEdgeNodeIndices[2 * index]; }
	PX_FORCE_INLINE NodeIndex getNodeIndex2(IG::EdgeIndex index) const { return mEdgeNodeIndices[2 * index + 1]; }

	PX_FORCE_INLINE PxU32*	getEdgeNodeIndexPtr()	const { return mNpIndexPtr;	}
	PX_FORCE_INLINE	PxU64	getContextId()			const { return mContextId;	}

	PxU32 getNbIslands() const { return mIslandStaticTouchCount.size(); }

	const PxU32* getIslandStaticTouchCount() const { return mIslandStaticTouchCount.begin(); }

	const PxU32* getIslandIds() const { return mIslandIds.begin(); }

	bool checkInternalConsistency();

private:

	void insertNewEdges();
	void removeDestroyedEdges();
	void wakeIslands();
	void wakeIslands2();
	void processNewEdges();
	void processLostEdges(Ps::Array<NodeIndex>& destroyedNodes, bool allowDeactivation, bool permitKinematicDeactivation, PxU32 dirtyNodeLimit);

	void removeConnectionInternal(EdgeIndex edgeIndex);

	void addConnection(NodeIndex nodeHandle1, NodeIndex nodeHandle2, Edge::EdgeType edgeType, EdgeIndex handle);

	void addConnectionToGraph(EdgeIndex index);
	void removeConnectionFromGraph(EdgeIndex edgeIndex);
	void connectEdge(EdgeInstance& instance, EdgeInstanceIndex edgeIndex, Node& source, NodeIndex destination);
	void disconnectEdge(EdgeInstance& instance, EdgeInstanceIndex edgeIndex, Node& node);

	//Merges 2 islands together. The returned id is the id of the merged island
	IslandId mergeIslands(IslandId island0, IslandId island1, NodeIndex node0, NodeIndex node1);

	void mergeIslandsInternal(Island& island0, Island& island1, IslandId islandId0, IslandId islandId1, NodeIndex node0, NodeIndex node1);
	

	IslandSim& operator = (const IslandSim&);
	IslandSim(const IslandSim&);

	void unwindRoute(PxU32 traversalIndex, NodeIndex lastNode, PxU32 hopCount, IslandId id);

	void activateIsland(IslandId island);

	void deactivateIsland(IslandId island);

	bool canFindRoot(NodeIndex startNode, NodeIndex targetNode, Ps::Array<NodeIndex>* visitedNodes);

	bool tryFastPath(NodeIndex startNode, NodeIndex targetNode, IslandId islandId);

	bool findRoute(NodeIndex startNode, NodeIndex targetNode, IslandId islandId);

	bool isPathTo(NodeIndex startNode, NodeIndex targetNode);

	void addNode(bool isActive, bool isKinematic, Node::NodeType type, NodeIndex nodeIndex);

	void activateNodeInternal(NodeIndex index);
	void deactivateNodeInternal(NodeIndex index);

	PX_FORCE_INLINE  void notifyReadyForSleeping(const NodeIndex nodeIndex)
	{
		Node& node = mNodes[nodeIndex.index()];
		//PX_ASSERT(node.isActive());
		node.setIsReadyForSleeping();
	}

	PX_FORCE_INLINE  void notifyNotReadyForSleeping(const NodeIndex nodeIndex)
	{
		Node& node = mNodes[nodeIndex.index()];
		PX_ASSERT(node.isActive() || node.isActivating());
		node.clearIsReadyForSleeping();
	}

	PX_FORCE_INLINE void markIslandActive(IslandId islandId)
	{
		Island& island = mIslands[islandId];
		PX_ASSERT(!mIslandAwake.test(islandId));
		PX_ASSERT(island.mActiveIndex == IG_INVALID_ISLAND);

		mIslandAwake.set(islandId);
		island.mActiveIndex = mActiveIslands.size();
		mActiveIslands.pushBack(islandId);
	}

	PX_FORCE_INLINE void markIslandInactive(IslandId islandId)
	{
		Island& island = mIslands[islandId];
		PX_ASSERT(mIslandAwake.test(islandId));
		PX_ASSERT(island.mActiveIndex != IG_INVALID_ISLAND);
		PX_ASSERT(mActiveIslands[island.mActiveIndex] == islandId);
		IslandId replaceId = mActiveIslands[mActiveIslands.size()-1];
		PX_ASSERT(mIslandAwake.test(replaceId));
		Island& replaceIsland = mIslands[replaceId];
		replaceIsland.mActiveIndex = island.mActiveIndex;
		mActiveIslands[island.mActiveIndex] = replaceId;
		mActiveIslands.forceSize_Unsafe(mActiveIslands.size()-1);
		island.mActiveIndex = IG_INVALID_ISLAND;
		mIslandAwake.reset(islandId);
	}

	PX_FORCE_INLINE void markKinematicActive(NodeIndex index)
	{
		Node& node = mNodes[index.index()];
		PX_ASSERT(node.isKinematic());
		if(node.mActiveRefCount == 0 && mActiveNodeIndex[index.index()] == IG_INVALID_NODE)
		{
			//PX_ASSERT(mActiveNodeIndex[index.index()] == IG_INVALID_NODE);
			//node.mActiveNodeIndex = mActiveKinematicNodes.size();
			mActiveNodeIndex[index.index()] = mActiveKinematicNodes.size();
			mActiveKinematicNodes.pushBack(index);
		}
	}

	PX_FORCE_INLINE void markKinematicInactive(NodeIndex index)
	{
		Node& node = mNodes[index.index()];
		PX_ASSERT(node.isKinematic());
		PX_ASSERT(mActiveNodeIndex[index.index()] != IG_INVALID_NODE);
		PX_ASSERT(mActiveKinematicNodes[mActiveNodeIndex[index.index()]].index() == index.index());

		if(node.mActiveRefCount == 0)
		{
			//Only remove from active kinematic list if it has no active contacts referencing it *and* it is asleep
			if(mActiveNodeIndex[index.index()] != IG_INVALID_NODE)
			{
				//Need to verify active node index because there is an edge case where a node could be woken, then put to 
				//sleep in the same frame. This would mean that it would not have an active index at this stage.
				NodeIndex replaceIndex = mActiveKinematicNodes.back();
				PX_ASSERT(mActiveNodeIndex[replaceIndex.index()] == mActiveKinematicNodes.size()-1);
				mActiveNodeIndex[replaceIndex.index()] = mActiveNodeIndex[index.index()];
				mActiveKinematicNodes[mActiveNodeIndex[index.index()]] = replaceIndex;
				mActiveKinematicNodes.forceSize_Unsafe(mActiveKinematicNodes.size()-1);
				mActiveNodeIndex[index.index()] = IG_INVALID_NODE;
			}
		}
	}

	PX_FORCE_INLINE void markActive(NodeIndex index)
	{
		Node& node = mNodes[index.index()];
		PX_ASSERT(!node.isKinematic());
		PX_ASSERT(mActiveNodeIndex[index.index()] == IG_INVALID_NODE);
		mActiveNodeIndex[index.index()] = mActiveNodes[node.mType].size();
		mActiveNodes[node.mType].pushBack(index);
	}

	PX_FORCE_INLINE void markInactive(NodeIndex index)
	{
		Node& node = mNodes[index.index()];

		PX_ASSERT(!node.isKinematic());
		PX_ASSERT(mActiveNodeIndex[index.index()] != IG_INVALID_NODE);

		Ps::Array<NodeIndex>& activeNodes = mActiveNodes[node.mType];

		PX_ASSERT(activeNodes[mActiveNodeIndex[index.index()]].index() == index.index());
		const PxU32 initialActiveNodeCount = mInitialActiveNodeCount[node.mType];

		if(mActiveNodeIndex[index.index()] < initialActiveNodeCount)
		{
			//It's in the initial active node set. We retain a list of active nodes, where the existing active nodes
			//are at the beginning of the array and the newly activated nodes are at the end of the array...
			//The solution is to move the node to the end of the initial active node list in this case
			PxU32 activeNodeIndex = mActiveNodeIndex[index.index()];
			NodeIndex replaceIndex = activeNodes[initialActiveNodeCount-1];
			PX_ASSERT(mActiveNodeIndex[replaceIndex.index()] == initialActiveNodeCount-1);
			mActiveNodeIndex[index.index()] = mActiveNodeIndex[replaceIndex.index()];
			mActiveNodeIndex[replaceIndex.index()] = activeNodeIndex;
			activeNodes[activeNodeIndex] = replaceIndex;
			activeNodes[mActiveNodeIndex[index.index()]] = index;
			mInitialActiveNodeCount[node.mType]--;	
		}

		PX_ASSERT(!node.isKinematic());
		PX_ASSERT(mActiveNodeIndex[index.index()] != IG_INVALID_NODE);
		PX_ASSERT(activeNodes[mActiveNodeIndex[index.index()]].index() == index.index());

		NodeIndex replaceIndex = activeNodes.back();
		PX_ASSERT(mActiveNodeIndex[replaceIndex.index()] == activeNodes.size()-1);
		mActiveNodeIndex[replaceIndex.index()] = mActiveNodeIndex[index.index()];
		activeNodes[mActiveNodeIndex[index.index()]] = replaceIndex;
		activeNodes.forceSize_Unsafe(activeNodes.size()-1);
		mActiveNodeIndex[index.index()] = IG_INVALID_NODE;			
	}

	PX_FORCE_INLINE void markEdgeActive(EdgeIndex index)
	{
		Edge& edge = mEdges[index];

		PX_ASSERT((edge.mEdgeState & Edge::eACTIVATING) == 0);

		edge.mEdgeState |= Edge::eACTIVATING;

		mActivatedEdges[edge.mEdgeType].pushBack(index);

		mActiveEdgeCount[edge.mEdgeType]++;
		
		//Set the active bit...
		if(edge.mEdgeType == Edge::eCONTACT_MANAGER)
			mActiveContactEdges.set(index);

		NodeIndex nodeIndex1 = mEdgeNodeIndices[2 * index];
		NodeIndex nodeIndex2 = mEdgeNodeIndices[2 * index + 1];

		if (nodeIndex1.index() != IG_INVALID_NODE && nodeIndex2.index() != IG_INVALID_NODE)
		{
			PX_ASSERT((!mNodes[nodeIndex1.index()].isKinematic()) || (!mNodes[nodeIndex2.index()].isKinematic()) || edge.getEdgeType() == IG::Edge::eCONTACT_MANAGER);
			{
				Node& node = mNodes[nodeIndex1.index()];

				if(node.mActiveRefCount == 0 && node.isKinematic() && !(node.isActive() || node.isActivating()))
				{
					//Add to active kinematic list
					markKinematicActive(nodeIndex1);
				}
				node.mActiveRefCount++;
			}

			{
				Node& node = mNodes[nodeIndex2.index()];
				if(node.mActiveRefCount == 0 && node.isKinematic() && !(node.isActive() || node.isActivating()))
				{
					//Add to active kinematic list
					markKinematicActive(nodeIndex2);
				}
				node.mActiveRefCount++;
			}
		}

	}

	void removeEdgeFromActivatingList(EdgeIndex index);

	PX_FORCE_INLINE void removeEdgeFromIsland(Island& island, EdgeIndex edgeIndex)
	{
		Edge& edge = mEdges[edgeIndex];
		if(edge.mNextIslandEdge != IG_INVALID_EDGE)
		{
			PX_ASSERT(mEdges[edge.mNextIslandEdge].mPrevIslandEdge == edgeIndex);
			mEdges[edge.mNextIslandEdge].mPrevIslandEdge = edge.mPrevIslandEdge;
		}
		else
		{
			PX_ASSERT(island.mLastEdge[edge.mEdgeType] == edgeIndex);
			island.mLastEdge[edge.mEdgeType] = edge.mPrevIslandEdge;
		}

		if(edge.mPrevIslandEdge != IG_INVALID_EDGE)
		{
			PX_ASSERT(mEdges[edge.mPrevIslandEdge].mNextIslandEdge == edgeIndex);
			mEdges[edge.mPrevIslandEdge].mNextIslandEdge = edge.mNextIslandEdge;
		}
		else
		{
			PX_ASSERT(island.mFirstEdge[edge.mEdgeType] == edgeIndex);
			island.mFirstEdge[edge.mEdgeType] = edge.mNextIslandEdge;
		}

		island.mEdgeCount[edge.mEdgeType]--;
		edge.mNextIslandEdge = edge.mPrevIslandEdge = IG_INVALID_EDGE;
	}

	PX_FORCE_INLINE void addEdgeToIsland(Island& island, EdgeIndex edgeIndex)
	{
		Edge& edge = mEdges[edgeIndex];
		PX_ASSERT(edge.mNextIslandEdge == IG_INVALID_EDGE && edge.mPrevIslandEdge == IG_INVALID_EDGE);

		if(island.mLastEdge[edge.mEdgeType] != IG_INVALID_EDGE)
		{
			PX_ASSERT(mEdges[island.mLastEdge[edge.mEdgeType]].mNextIslandEdge == IG_INVALID_EDGE);
			mEdges[island.mLastEdge[edge.mEdgeType]].mNextIslandEdge = edgeIndex;
		}
		else
		{
			PX_ASSERT(island.mFirstEdge[edge.mEdgeType] == IG_INVALID_EDGE);
			island.mFirstEdge[edge.mEdgeType] = edgeIndex;
		}

		edge.mPrevIslandEdge = island.mLastEdge[edge.mEdgeType];
		island.mLastEdge[edge.mEdgeType] = edgeIndex;
		island.mEdgeCount[edge.mEdgeType]++;
	}

	PX_FORCE_INLINE void removeNodeFromIsland(Island& island, NodeIndex nodeIndex)
	{
		Node& node = mNodes[nodeIndex.index()];
		if(node.mNextNode.isValid())
		{
			PX_ASSERT(mNodes[node.mNextNode.index()].mPrevNode.index() == nodeIndex.index());
			mNodes[node.mNextNode.index()].mPrevNode = node.mPrevNode;
		}
		else
		{
			PX_ASSERT(island.mLastNode.index() == nodeIndex.index());
			island.mLastNode = node.mPrevNode;
		}

		if(node.mPrevNode.isValid())
		{
			PX_ASSERT(mNodes[node.mPrevNode.index()].mNextNode.index() == nodeIndex.index());
			mNodes[node.mPrevNode.index()].mNextNode = node.mNextNode;
		}
		else
		{
			PX_ASSERT(island.mRootNode.index() == nodeIndex.index());
			island.mRootNode = node.mNextNode;
		}

		island.mSize[node.mType]--;

		node.mNextNode = NodeIndex(); node.mPrevNode = NodeIndex();
	}

	//void setEdgeConnectedInternal(EdgeIndex edgeIndex);

	//void setEdgeDisconnectedInternal(EdgeIndex edgeIndex);

	friend class SimpleIslandManager;
	friend class ThirdPassTask;

};


}


struct PartitionIndexData
{
	PxU16 mPartitionIndex;				//! The current partition this edge is in. Used to find the edge efficiently. PxU8 is probably too small (256 partitions max) but PxU16 should be more than enough
	PxU8 mPatchIndex;					//! The patch index for this partition edge. There may be multiple entries for a given edge if there are multiple patches.
	PxU8 mCType;			//! The type of constraint this is
	PxU32 mPartitionEntryIndex;			//! index of partition edges for this partition
};

struct PartitionNodeData
{
	IG::NodeIndex mNodeIndex0;
	IG::NodeIndex mNodeIndex1;
	PxU32 mNextIndex0;
	PxU32 mNextIndex1;
};


#define INVALID_PARTITION_INDEX 0xFFFF

struct PartitionEdge
{
	IG::EdgeIndex mEdgeIndex;			//! The edge index into the island manager. Used to identify the contact manager/constraint
	IG::NodeIndex mNode0;				//! The node index for node 0. Can be obtained from the edge index alternatively
	IG::NodeIndex mNode1;				//! The node idnex for node 1. Can be obtained from the edge index alternatively
	bool mInfiniteMass0;					//! Whether body 0 is kinematic
	bool mInfiniteMass1;					//! Whether body 1 is kinematic

	PartitionEdge* mNextPatch;	 //! for the contact manager has more than 1 patch, we have next patch's edge and previous patch's edge to connect to this edge

	PxU32 mUniqueIndex;					//! a unique ID for this edge

	PartitionEdge() : mEdgeIndex(IG_INVALID_EDGE), mInfiniteMass0(false), 
		mInfiniteMass1(false), mNextPatch(NULL)
	{
	}
};

}

#endif
