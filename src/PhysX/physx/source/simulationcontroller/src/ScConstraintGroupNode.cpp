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


#include "ScConstraintGroupNode.h"
#include "ScConstraintProjectionManager.h"
#include "PsFoundation.h"
#include "ScBodySim.h"
#include "ScConstraintSim.h"
#include "ScConstraintInteraction.h"

using namespace physx;

Sc::ConstraintGroupNode::ConstraintGroupNode(BodySim& b) : 
	body(&b),
	parent(this),
	tail(this),
	rank(0),
	next(NULL),

	projectionFirstRoot(NULL),
	projectionNextRoot(NULL),
	projectionParent(NULL),
	projectionFirstChild(NULL),
	projectionNextSibling(NULL),
	projectionConstraint(NULL),

	flags(0)
{
}


//
// Implementation of FIND of 
// UNION-FIND algo.
//
Sc::ConstraintGroupNode& Sc::ConstraintGroupNode::getRoot()
{
	PX_ASSERT(parent);

	ConstraintGroupNode* root = parent;

	if (root->parent == root)
		return *root;
	else
	{
		PxU32 nbHops = 1;
		root = root->parent;

		while(root != root->parent)
		{
			root = root->parent;
			nbHops++;
		}

		// Write root to all nodes on the path
		ConstraintGroupNode* curr = this;
		while(nbHops)
		{
			ConstraintGroupNode* n = curr->parent;
			curr->parent = root;
			curr = n;
			nbHops--;
		}

		return *root;
	}
}


void Sc::ConstraintGroupNode::markForProjectionTreeRebuild(ConstraintProjectionManager& cpManager)
{
	ConstraintGroupNode& root = getRoot();
	if (!root.readFlag(ConstraintGroupNode::ePENDING_TREE_UPDATE))
	{
		cpManager.addToPendingTreeUpdates(root);
	}
}


void Sc::ConstraintGroupNode::initProjectionData(ConstraintGroupNode* parent_, ConstraintSim* c)
{
	projectionConstraint = c;

	//add us to parent's child list:
	if (parent_)
	{
		projectionNextSibling = parent_->projectionFirstChild;
		parent_->projectionFirstChild = this;

		projectionParent = parent_;
	}
}


void Sc::ConstraintGroupNode::clearProjectionData()
{
	projectionFirstRoot = NULL;
	projectionNextRoot = NULL;
	projectionParent = NULL;
	projectionFirstChild = NULL;
	projectionNextSibling = NULL;
	projectionConstraint = NULL;
}


void Sc::ConstraintGroupNode::projectPose(ConstraintGroupNode& node, Ps::Array<BodySim*>& projectedBodies)
{
	PX_ASSERT(node.hasProjectionTreeRoot());

	Sc::ConstraintProjectionTree::projectPose(node, projectedBodies);
}
