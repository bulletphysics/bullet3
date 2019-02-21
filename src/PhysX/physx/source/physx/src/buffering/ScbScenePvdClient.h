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

#ifndef SCB_SCENE_PVD_CLIENT_H
#define SCB_SCENE_PVD_CLIENT_H

#include "PxPhysXConfig.h"

#if PX_SUPPORT_PVD

#include "foundation/PxStrideIterator.h"
#include "pvd/PxPvdTransport.h"

#include "PxPvdSceneClient.h"
#include "PvdMetaDataPvdBinding.h"

#include "CmBitMap.h"

#include "PxPvdClient.h"
#include "PxPvdUserRenderer.h"
#include "PsPvd.h"

namespace physx
{
class PxScene;
class PxActor;
class PxShape;
class PxGeometryHolder;
class PxArticulationLink;
class PxRenderBuffer;

namespace Scb
{
class Scene;
class Actor;
class Body;
class RigidStatic;
class RigidObject;
class Shape;
class Constraint;
class Articulation;
class ArticulationJoint;
class Aggregate;
class SoftBody;
}

namespace Sc
{
class MaterialCore;
class ConstraintCore;
}

namespace Vd
{
class ScbScenePvdClient : public PxPvdSceneClient, public PvdClient, public PvdVisualizer
{
	PX_NOCOPY(ScbScenePvdClient)
  public:
							ScbScenePvdClient(Scb::Scene& scene);
	virtual					~ScbScenePvdClient();

	// PxPvdSceneClient
	virtual	void			setScenePvdFlag(PxPvdSceneFlag::Enum flag, bool value);
	virtual	void			setScenePvdFlags(PxPvdSceneFlags flags)				{ mFlags = flags;	}
	virtual	PxPvdSceneFlags	getScenePvdFlags()							const	{ return mFlags;	}
	virtual	void			updateCamera(const char* name, const PxVec3& origin, const PxVec3& up, const PxVec3& target);
	virtual	void			drawPoints(const PvdDebugPoint* points, PxU32 count);
	virtual	void			drawLines(const PvdDebugLine* lines, PxU32 count);
	virtual	void			drawTriangles(const PvdDebugTriangle* triangles, PxU32 count);
	virtual	void			drawText(const PvdDebugText& text);
	virtual	PvdClient*		getClientInternal()									{ return this;		}
	//~PxPvdSceneClient
	
	// pvdClient	
	virtual	PvdDataStream*		getDataStream()			{ return mPvdDataStream;	}
	virtual	PvdMetaDataBinding*	getMetaDataBinding()	{ return &mMetaDataBinding;	}
	virtual	PvdUserRenderer*	getUserRender()			{ return mUserRender;		}
	virtual bool                isConnected()	const	{ return mIsConnected;		}
	virtual void                onPvdConnected();
	virtual void                onPvdDisconnected();
	virtual void                flush()					{}
	//~pvdClient

	PX_FORCE_INLINE bool checkPvdDebugFlag()	const
	{
		return mIsConnected && (mPvd->getInstrumentationFlags() & PxPvdInstrumentationFlag::eDEBUG);
	}

	PX_FORCE_INLINE	PxPvdSceneFlags	getScenePvdFlagsFast() const	{ return mFlags;	}
	PX_FORCE_INLINE	void             setPsPvd(PsPvd* pvd)			{ mPvd = pvd;		}

	void frameStart(PxReal simulateElapsedTime);
	void frameEnd();

	void updatePvdProperties();
	void releasePvdInstance();

	void createPvdInstance	(const PxActor* actor);          // temporary for deformables and particle systems - sschirm: deformables and particles are gone...
	void updatePvdProperties(const PxActor* actor);
	void releasePvdInstance	(const PxActor* actor); // temporary for deformables and particle systems - sschirm: deformables and particles are gone...

	void createPvdInstance	(const Scb::Actor* actor); // temporary for deformables and particle systems - sschirm: deformables and particles are gone...
	void updatePvdProperties(const Scb::Actor* actor);
	void releasePvdInstance	(const Scb::Actor* actor); // temporary for deformables and particle systems - sschirm: deformables and particles are gone...

	void createPvdInstance		(const Scb::Body* body);
	void updatePvdProperties	(const Scb::Body* body);
	void updateKinematicTarget	(const Scb::Body* body, const PxTransform& p);

	void createPvdInstance		(const Scb::RigidStatic* rigidStatic);
	void updatePvdProperties	(const Scb::RigidStatic* rigidStatic);

	void releasePvdInstance		(const Scb::RigidObject* rigidObject);

	void createPvdInstance	(const Scb::Constraint* constraint);
	void updatePvdProperties(const Scb::Constraint* constraint);
	void releasePvdInstance	(const Scb::Constraint* constraint);

	void createPvdInstance	(const Scb::Articulation* articulation);
	void updatePvdProperties(const Scb::Articulation* articulation);
	void releasePvdInstance	(const Scb::Articulation* articulation);

	void createPvdInstance	(const Scb::ArticulationJoint* articulationJoint);
	void updatePvdProperties(const Scb::ArticulationJoint* articulationJoint);
	void releasePvdInstance	(const Scb::ArticulationJoint* articulationJoint);

	void createPvdInstance	(const Sc::MaterialCore* materialCore);
	void updatePvdProperties(const Sc::MaterialCore* materialCore);
	void releasePvdInstance	(const Sc::MaterialCore* materialCore);

	void createPvdInstance			(const Scb::Shape* shape, PxActor& owner);
	void updateMaterials			(const Scb::Shape* shape);
	void updatePvdProperties		(const Scb::Shape* shape);
	void releaseAndRecreateGeometry	(const Scb::Shape* shape);
	void releasePvdInstance			(const Scb::Shape* shape, PxActor& owner);
	void addBodyAndShapesToPvd		(Scb::Body& b);
	void addStaticAndShapesToPvd	(Scb::RigidStatic& s);

	void createPvdInstance		(const Scb::Aggregate* aggregate);
	void updatePvdProperties	(const Scb::Aggregate* aggregate);
	void attachAggregateActor	(const Scb::Aggregate* aggregate, Scb::Actor* actor);
	void detachAggregateActor	(const Scb::Aggregate* aggregate, Scb::Actor* actor);
	void releasePvdInstance		(const Scb::Aggregate* aggregate);

	void originShift(PxVec3 shift);
	void updateJoints();
	void updateContacts();
	void updateSceneQueries();

	// PvdVisualizer
	void visualize(PxArticulationLink& link);
	void visualize(const PxRenderBuffer& debugRenderable);

  private:

	void				sendEntireScene();
	void				updateConstraint(const Sc::ConstraintCore& scConstraint, PxU32 updateType);
	void				setCreateContactReports(bool b);

	PxPvdSceneFlags			mFlags;
	PsPvd*					mPvd;
	Scb::Scene&				mScbScene;
	
	PvdDataStream*			mPvdDataStream;
	PvdMetaDataBinding		mMetaDataBinding;
	PvdUserRenderer*		mUserRender;
	RendererEventClient*	mRenderClient;
	bool					mIsConnected;
};

} // pvd

} // physx
#endif // PX_SUPPORT_PVD

#endif // SCB_SCENE_PVD_CLIENT_H
