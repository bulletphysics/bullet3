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

#ifndef PX_PHYSICS_NP_RIGIDACTOR_TEMPLATE_INTERNAL
#define PX_PHYSICS_NP_RIGIDACTOR_TEMPLATE_INTERNAL

namespace physx
{

template<class T, class T2>
static PX_FORCE_INLINE void releaseActorT(NpRigidActorTemplate<T>* actor, T2& scbActor)
{
	NP_WRITE_CHECK(NpActor::getOwnerScene(*actor));

	NpPhysics::getInstance().notifyDeletionListenersUserRelease(actor, actor->userData);

	Scb::Scene* s = scbActor.getScbSceneForAPI();

	const bool noSim = scbActor.isSimDisabledInternally();
	// important to check the non-buffered flag because it tells what the current internal state of the object is
	// (someone might switch to non-simulation and release all while the sim is running). Reading is fine even if 
	// the sim is running because actor flags are read-only internally.
	if(s && noSim)
	{
		// need to do it here because the Np-shape buffer will not be valid anymore after the release below
		// and unlike simulation objects, there is no shape buffer in the simulation controller
		actor->getShapeManager().clearShapesOnRelease(*s, *actor);
	}

	actor->NpRigidActorTemplate<T>::release();

	if(s)
	{
		s->removeActor(scbActor, true, noSim);
		static_cast<NpScene*>(s->getPxScene())->removeFromRigidActorList(actor->getRigidActorArrayIndex());
	}

	scbActor.destroy();
}

}

#endif
