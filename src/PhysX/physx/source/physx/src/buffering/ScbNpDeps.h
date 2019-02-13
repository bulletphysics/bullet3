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


#ifndef PX_PHYSICS_SCB_NPDEPS
#define PX_PHYSICS_SCB_NPDEPS

namespace physx
{

// The Scb layer needs to delete the owning Np objects, but we don't want to include the Np headers
// necessary to find their addresses. So we use link-level dependencies instead.

namespace Scb
{
	class Base;
	class Shape;
	class RigidObject;
	class Constraint;
	class Scene;
	class ArticulationJoint;
	class Articulation;
	class RigidStatic;
	class Body;
}

namespace Sc
{
	class RigidCore;
}

class PxScene;

extern void NpDestroy(Scb::Base&);

// we want to get the pointer to the rigid object that owns a shape, and the two actor pointers for a constraint, so that we don't
// duplicate the scene graph in Scb

extern PxU32 NpRigidStaticGetShapes(Scb::RigidStatic& rigid, void* const *&shapes);
extern PxU32 NpRigidDynamicGetShapes(Scb::Body& body, void* const *&shapes, bool* isCompound = NULL);
extern size_t NpShapeGetScPtrOffset();
extern void NpShapeIncRefCount(Scb::Shape& shape);
extern void NpShapeDecRefCount(Scb::Shape& shape);

extern Sc::RigidCore* NpShapeGetScRigidObjectFromScbSLOW(const Scb::Shape &);
extern void NpConstraintGetRigidObjectsFromScb(const Scb::Constraint&, Scb::RigidObject*&, Scb::RigidObject*&);
extern void NpArticulationJointGetBodiesFromScb(Scb::ArticulationJoint&, Scb::Body*&, Scb::Body*&);
extern Scb::Body* NpArticulationGetRootFromScb(Scb::Articulation&);
}

#endif
