/*
Physics Effects Copyright(C) 2010 Sony Computer Entertainment Inc.
All rights reserved.

Physics Effects is open software; you can redistribute it and/or
modify it under the terms of the BSD License.

Physics Effects is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the BSD License for more details.

A copy of the BSD License is distributed with
Physics Effects under the filename: physics_effects_license.txt
*/

#include "../../../include/physics_effects/base_level/collision/pfx_shape.h"
#include "../../base_level/collision/pfx_contact_box_box.h"
#include "../../base_level/collision/pfx_contact_box_capsule.h"
#include "../../base_level/collision/pfx_contact_box_sphere.h"
#include "../../base_level/collision/pfx_contact_capsule_capsule.h"
#include "../../base_level/collision/pfx_contact_capsule_sphere.h"
#include "../../base_level/collision/pfx_contact_sphere_sphere.h"
#include "../../base_level/collision/pfx_gjk_solver.h"
#include "../../base_level/collision/pfx_contact_large_tri_mesh.h"
#include "../../base_level/collision/pfx_gjk_support_func.h"
#include "pfx_detect_collision_func.h"


namespace sce {
namespace PhysicsEffects {


///////////////////////////////////////////////////////////////////////////////
// Collision Detection Function

void detectCollisionDummy(
				PfxContactCache &contacts,
				const PfxShape &shapeA,const PfxTransform3 &offsetTransformA,const PfxTransform3 &worldTransformA,int shapeIdA,
				const PfxShape &shapeB,const PfxTransform3 &offsetTransformB,const PfxTransform3 &worldTransformB,int shapeIdB,
				float contactThreshold)
{
	(void)contacts;
	(void)shapeA,(void)offsetTransformA,(void)worldTransformA,(void)shapeIdA;
	(void)shapeB,(void)offsetTransformB,(void)worldTransformB,(void)shapeIdB;
	(void)contactThreshold;
}

void detectCollisionBoxBox(
				PfxContactCache &contacts,
				const PfxShape &shapeA,const PfxTransform3 &offsetTransformA,const PfxTransform3 &worldTransformA,int shapeIdA,
				const PfxShape &shapeB,const PfxTransform3 &offsetTransformB,const PfxTransform3 &worldTransformB,int shapeIdB,
				float contactThreshold)
{
	(void)shapeIdA,(void)shapeIdB;
	PfxBox boxA = shapeA.getBox();
	PfxBox boxB = shapeB.getBox();
	
	PfxVector3 nml;
	PfxPoint3 pA,pB;
	
	PfxFloat d = pfxContactBoxBox(nml,pA,pB,&boxA,worldTransformA,&boxB,worldTransformB);
	
	if(d < contactThreshold) {
		contacts.addContactPoint(d,-nml,offsetTransformA*pA,offsetTransformB*pB,PfxSubData());
	}
}

void detectCollisionBoxCapsule(
				PfxContactCache &contacts,
				const PfxShape &shapeA,const PfxTransform3 &offsetTransformA,const PfxTransform3 &worldTransformA,int shapeIdA,
				const PfxShape &shapeB,const PfxTransform3 &offsetTransformB,const PfxTransform3 &worldTransformB,int shapeIdB,
				float contactThreshold)
{
	(void)shapeIdA,(void)shapeIdB;
	PfxBox boxA = shapeA.getBox();
	PfxCapsule capsuleB = shapeB.getCapsule();
	
	PfxVector3 nml;
	PfxPoint3 pA,pB;
	
	PfxFloat d = pfxContactBoxCapsule(nml,pA,pB,&boxA,worldTransformA,&capsuleB,worldTransformB);
	
	if(d < contactThreshold) {
		contacts.addContactPoint(d,-nml,offsetTransformA*pA,offsetTransformB*pB,PfxSubData());
	}
}

void detectCollisionBoxSphere(
				PfxContactCache &contacts,
				const PfxShape &shapeA,const PfxTransform3 &offsetTransformA,const PfxTransform3 &worldTransformA,int shapeIdA,
				const PfxShape &shapeB,const PfxTransform3 &offsetTransformB,const PfxTransform3 &worldTransformB,int shapeIdB,
				float contactThreshold)
{
	(void)shapeIdA,(void)shapeIdB;
	PfxBox boxA = shapeA.getBox();
	PfxSphere sphereB = shapeB.getSphere();
	
	PfxVector3 nml;
	PfxPoint3 pA,pB;
	
	PfxFloat d = pfxContactBoxSphere(nml,pA,pB,&boxA,worldTransformA,&sphereB,worldTransformB);
	
	if(d < contactThreshold) {
		contacts.addContactPoint(d,-nml,offsetTransformA*pA,offsetTransformB*pB,PfxSubData());
	}
}

void detectCollisionCapsuleBox(
				PfxContactCache &contacts,
				const PfxShape &shapeA,const PfxTransform3 &offsetTransformA,const PfxTransform3 &worldTransformA,int shapeIdA,
				const PfxShape &shapeB,const PfxTransform3 &offsetTransformB,const PfxTransform3 &worldTransformB,int shapeIdB,
				float contactThreshold)
{
	(void)shapeIdA,(void)shapeIdB;
	PfxCapsule capsuleA = shapeA.getCapsule();
	PfxBox boxB = shapeB.getBox();
	
	PfxVector3 nml;
	PfxPoint3 pA,pB;
	
	PfxFloat d = pfxContactBoxCapsule(nml,pB,pA,&boxB,worldTransformB,&capsuleA,worldTransformA);
	
	if(d < contactThreshold) {
		contacts.addContactPoint(d,nml,offsetTransformA*pA,offsetTransformB*pB,PfxSubData());
	}
}

void detectCollisionCapsuleCapsule(
				PfxContactCache &contacts,
				const PfxShape &shapeA,const PfxTransform3 &offsetTransformA,const PfxTransform3 &worldTransformA,int shapeIdA,
				const PfxShape &shapeB,const PfxTransform3 &offsetTransformB,const PfxTransform3 &worldTransformB,int shapeIdB,
				float contactThreshold)
{
	(void)shapeIdA,(void)shapeIdB;
	PfxCapsule capsuleA = shapeA.getCapsule();
	PfxCapsule capsuleB = shapeB.getCapsule();
	
	PfxVector3 nml;
	PfxPoint3 pA,pB;
	
	PfxFloat d = pfxContactCapsuleCapsule(nml,pA,pB,&capsuleA,worldTransformA,&capsuleB,worldTransformB);
	
	if(d < contactThreshold) {
		contacts.addContactPoint(d,-nml,offsetTransformA*pA,offsetTransformB*pB,PfxSubData());
	}
}

void detectCollisionCapsuleSphere(
				PfxContactCache &contacts,
				const PfxShape &shapeA,const PfxTransform3 &offsetTransformA,const PfxTransform3 &worldTransformA,int shapeIdA,
				const PfxShape &shapeB,const PfxTransform3 &offsetTransformB,const PfxTransform3 &worldTransformB,int shapeIdB,
				float contactThreshold)
{
	(void)shapeIdA,(void)shapeIdB;
	PfxCapsule capsuleA = shapeA.getCapsule();
	PfxSphere sphereB = shapeB.getSphere();
	
	PfxVector3 nml;
	PfxPoint3 pA,pB;
	
	PfxFloat d = pfxContactCapsuleSphere(nml,pA,pB,&capsuleA,worldTransformA,&sphereB,worldTransformB);
	
	if(d < contactThreshold) {
		contacts.addContactPoint(d,-nml,offsetTransformA*pA,offsetTransformB*pB,PfxSubData());
	}
}

void detectCollisionSphereBox(
				PfxContactCache &contacts,
				const PfxShape &shapeA,const PfxTransform3 &offsetTransformA,const PfxTransform3 &worldTransformA,int shapeIdA,
				const PfxShape &shapeB,const PfxTransform3 &offsetTransformB,const PfxTransform3 &worldTransformB,int shapeIdB,
				float contactThreshold)
{
	(void)shapeIdA,(void)shapeIdB;
	PfxSphere sphereA = shapeA.getSphere();
	PfxBox boxB = shapeB.getBox();
	
	PfxVector3 nml;
	PfxPoint3 pA,pB;
	
	PfxFloat d = pfxContactBoxSphere(nml,pB,pA,&boxB,worldTransformB,&sphereA,worldTransformA);
	
	if(d < contactThreshold) {
		contacts.addContactPoint(d,nml,offsetTransformA*pA,offsetTransformB*pB,PfxSubData());
	}
}

void detectCollisionSphereCapsule(
				PfxContactCache &contacts,
				const PfxShape &shapeA,const PfxTransform3 &offsetTransformA,const PfxTransform3 &worldTransformA,int shapeIdA,
				const PfxShape &shapeB,const PfxTransform3 &offsetTransformB,const PfxTransform3 &worldTransformB,int shapeIdB,
				float contactThreshold)
{
	(void)shapeIdA,(void)shapeIdB;
	PfxSphere sphereA = shapeA.getSphere();
	PfxCapsule capsuleB = shapeB.getCapsule();
	
	PfxVector3 nml;
	PfxPoint3 pA,pB;
	
	PfxFloat d = pfxContactCapsuleSphere(nml,pB,pA,&capsuleB,worldTransformB,&sphereA,worldTransformA);
	
	if(d < contactThreshold) {
		contacts.addContactPoint(d,nml,offsetTransformA*pA,offsetTransformB*pB,PfxSubData());
	}
}

void detectCollisionSphereSphere(
				PfxContactCache &contacts,
				const PfxShape &shapeA,const PfxTransform3 &offsetTransformA,const PfxTransform3 &worldTransformA,int shapeIdA,
				const PfxShape &shapeB,const PfxTransform3 &offsetTransformB,const PfxTransform3 &worldTransformB,int shapeIdB,
				float contactThreshold)
{
	(void)shapeIdA,(void)shapeIdB;
	PfxSphere sphereA = shapeA.getSphere();
	PfxSphere sphereB = shapeB.getSphere();
	
	PfxVector3 nml;
	PfxPoint3 pA,pB;
	
	PfxFloat d = pfxContactSphereSphere(nml,pA,pB,&sphereA,worldTransformA,&sphereB,worldTransformB);
	
	if(d < contactThreshold) {
		contacts.addContactPoint(d,-nml,offsetTransformA*pA,offsetTransformB*pB,PfxSubData());
	}
}

void detectCollisionGjk(
				PfxContactCache &contacts,
				const PfxShape &shapeA,const PfxTransform3 &offsetTransformA,const PfxTransform3 &worldTransformA,int shapeIdA,
				const PfxShape &shapeB,const PfxTransform3 &offsetTransformB,const PfxTransform3 &worldTransformB,int shapeIdB,
				float contactThreshold)
{
	(void)shapeIdA,(void)shapeIdB;
	PfxFloat d = SCE_PFX_FLT_MAX;
	PfxVector3 nml;
	PfxPoint3 pA,pB;
	PfxGjkSolver gjk;

	if(shapeA.getType() == kPfxShapeCylinder) {
		PfxCylinder cylinderA = shapeA.getCylinder();
		
		if(shapeB.getType() == kPfxShapeSphere) {
			PfxSphere sphereB = shapeB.getSphere();
			gjk.setup((void*)&cylinderA,(void*)&sphereB,pfxGetSupportVertexCylinder,pfxGetSupportVertexSphere);
			d = gjk.collide(nml,pA,pB,worldTransformA,worldTransformB,SCE_PFX_FLT_MAX);
		}
		else if(shapeB.getType() == kPfxShapeBox) {
			PfxBox boxB = shapeB.getBox();
			gjk.setup((void*)&cylinderA,(void*)&boxB,pfxGetSupportVertexCylinder,pfxGetSupportVertexBox);
			d = gjk.collide(nml,pA,pB,worldTransformA,worldTransformB,SCE_PFX_FLT_MAX);
		}
		else if(shapeB.getType() == kPfxShapeCapsule) {
			PfxCapsule capsuleB = shapeB.getCapsule();
			gjk.setup((void*)&cylinderA,(void*)&capsuleB,pfxGetSupportVertexCylinder,pfxGetSupportVertexCapsule);
			d = gjk.collide(nml,pA,pB,worldTransformA,worldTransformB,SCE_PFX_FLT_MAX);
		}
		else if(shapeB.getType() == kPfxShapeCylinder) {
			PfxCylinder cylinderB = shapeB.getCylinder();
			gjk.setup((void*)&cylinderA,(void*)&cylinderB,pfxGetSupportVertexCylinder,pfxGetSupportVertexCylinder);
			d = gjk.collide(nml,pA,pB,worldTransformA,worldTransformB,SCE_PFX_FLT_MAX);
		}
		else if(shapeB.getType() == kPfxShapeConvexMesh) {
		const PfxConvexMesh *convexB = shapeB.getConvexMesh();

			gjk.setup((void*)&cylinderA,(void*)convexB,pfxGetSupportVertexCylinder,pfxGetSupportVertexConvex);
			d = gjk.collide(nml,pA,pB,worldTransformA,worldTransformB,SCE_PFX_FLT_MAX);

		}
	}
	else if(shapeB.getType() == kPfxShapeCylinder) {
		PfxCylinder cylinderB = shapeB.getCylinder();
		
		if(shapeA.getType() == kPfxShapeSphere) {
			PfxSphere sphereA = shapeA.getSphere();
			gjk.setup((void*)&sphereA,(void*)&cylinderB,pfxGetSupportVertexSphere,pfxGetSupportVertexCylinder);
			d = gjk.collide(nml,pA,pB,worldTransformA,worldTransformB,SCE_PFX_FLT_MAX);
		}
		else if(shapeA.getType() == kPfxShapeBox) {
			PfxBox boxA = shapeA.getBox();
			gjk.setup((void*)&boxA,(void*)&cylinderB,pfxGetSupportVertexBox,pfxGetSupportVertexCylinder);
			d = gjk.collide(nml,pA,pB,worldTransformA,worldTransformB,SCE_PFX_FLT_MAX);
		}
		else if(shapeA.getType() == kPfxShapeCapsule) {
			PfxCapsule capsuleA = shapeA.getCapsule();
			gjk.setup((void*)&capsuleA,(void*)&cylinderB,pfxGetSupportVertexCapsule,pfxGetSupportVertexCylinder);
			d = gjk.collide(nml,pA,pB,worldTransformA,worldTransformB,SCE_PFX_FLT_MAX);
		}
		else if(shapeA.getType() == kPfxShapeConvexMesh) {
		const PfxConvexMesh *convexA = shapeA.getConvexMesh();

			gjk.setup((void*)convexA,(void*)&cylinderB,pfxGetSupportVertexConvex,pfxGetSupportVertexCylinder);
			d = gjk.collide(nml,pA,pB,worldTransformA,worldTransformB,SCE_PFX_FLT_MAX);

		}
	}
	else if(shapeA.getType() == kPfxShapeConvexMesh) {
	const PfxConvexMesh *convexA = shapeA.getConvexMesh();
		
		if(shapeB.getType() == kPfxShapeSphere) {
			PfxSphere sphereB = shapeB.getSphere();
			gjk.setup((void*)convexA,(void*)&sphereB,pfxGetSupportVertexConvex,pfxGetSupportVertexSphere);
			d = gjk.collide(nml,pA,pB,worldTransformA,worldTransformB,SCE_PFX_FLT_MAX);
		}
		else if(shapeB.getType() == kPfxShapeBox) {
			PfxBox boxB = shapeB.getBox();
			gjk.setup((void*)convexA,(void*)&boxB,pfxGetSupportVertexConvex,pfxGetSupportVertexBox);
			d = gjk.collide(nml,pA,pB,worldTransformA,worldTransformB,SCE_PFX_FLT_MAX);
		}
		else if(shapeB.getType() == kPfxShapeCapsule) {
			PfxCapsule capsuleB = shapeB.getCapsule();
			gjk.setup((void*)convexA,(void*)&capsuleB,pfxGetSupportVertexConvex,pfxGetSupportVertexCapsule);
			d = gjk.collide(nml,pA,pB,worldTransformA,worldTransformB,SCE_PFX_FLT_MAX);
		}
		else if(shapeB.getType() == kPfxShapeCylinder) {
			PfxCylinder cylinderB = shapeB.getCylinder();
			gjk.setup((void*)convexA,(void*)&cylinderB,pfxGetSupportVertexConvex,pfxGetSupportVertexCylinder);
			d = gjk.collide(nml,pA,pB,worldTransformA,worldTransformB,SCE_PFX_FLT_MAX);
		}
		else if(shapeB.getType() == kPfxShapeConvexMesh) {
		const PfxConvexMesh *convexB = shapeB.getConvexMesh();

			gjk.setup((void*)convexA,(void*)convexB,pfxGetSupportVertexConvex,pfxGetSupportVertexConvex);
			d = gjk.collide(nml,pA,pB,worldTransformA,worldTransformB,SCE_PFX_FLT_MAX);

		}
		
	}
	else if(shapeB.getType() == kPfxShapeConvexMesh) {
	const PfxConvexMesh *convexB = shapeB.getConvexMesh();
		
		if(shapeA.getType() == kPfxShapeSphere) {
			PfxSphere sphereA = shapeA.getSphere();
			gjk.setup((void*)&sphereA,(void*)convexB,pfxGetSupportVertexSphere,pfxGetSupportVertexConvex);
			d = gjk.collide(nml,pA,pB,worldTransformA,worldTransformB,SCE_PFX_FLT_MAX);
		}
		else if(shapeA.getType() == kPfxShapeBox) {
			PfxBox boxA = shapeA.getBox();
			gjk.setup((void*)&boxA,(void*)convexB,pfxGetSupportVertexBox,pfxGetSupportVertexConvex);
			d = gjk.collide(nml,pA,pB,worldTransformA,worldTransformB,SCE_PFX_FLT_MAX);
		}
		else if(shapeA.getType() == kPfxShapeCapsule) {
			PfxCapsule capsuleA = shapeA.getCapsule();
			gjk.setup((void*)&capsuleA,(void*)convexB,pfxGetSupportVertexCapsule,pfxGetSupportVertexConvex);
			d = gjk.collide(nml,pA,pB,worldTransformA,worldTransformB,SCE_PFX_FLT_MAX);
		}
		else if(shapeA.getType() == kPfxShapeConvexMesh) {
		const PfxConvexMesh *convexA = shapeA.getConvexMesh();
			
			gjk.setup((void*)convexA,(void*)convexB,pfxGetSupportVertexConvex,pfxGetSupportVertexConvex);
			d = gjk.collide(nml,pA,pB,worldTransformA,worldTransformB,SCE_PFX_FLT_MAX);

		}

	}

	if(d < contactThreshold) {
		contacts.addContactPoint(d,nml,offsetTransformA*pA,offsetTransformB*pB,PfxSubData());
	}
}

void detectCollisionLargeTriMesh(
				PfxContactCache &contacts,
				const PfxShape &shapeA,const PfxTransform3 &offsetTransformA,const PfxTransform3 &worldTransformA,int shapeIdA,
				const PfxShape &shapeB,const PfxTransform3 &offsetTransformB,const PfxTransform3 &worldTransformB,int shapeIdB,
				float contactThreshold)
{
	(void)shapeIdA,(void)shapeIdB;
	if(shapeA.getType() == kPfxShapeLargeTriMesh) {
	const PfxLargeTriMesh *lmeshA = shapeA.getLargeTriMesh();
		
		PfxContactCache localContacts;
		pfxContactLargeTriMesh(localContacts,
			lmeshA,worldTransformA,
			shapeB,worldTransformB,
			contactThreshold);
		
		
		for(int i=0;i<localContacts.getNumContacts();i++) {
			contacts.addContactPoint(
				localContacts.getDistance(i),
				localContacts.getNormal(i),
				offsetTransformA * localContacts.getLocalPointA(i),
				offsetTransformB * localContacts.getLocalPointB(i),
				localContacts.getSubData(i));
		}
	}
	else if(shapeB.getType() == kPfxShapeLargeTriMesh) {
	const PfxLargeTriMesh *lmeshB = shapeB.getLargeTriMesh();

		PfxContactCache localContacts;
		pfxContactLargeTriMesh(localContacts,
			lmeshB,worldTransformB,
			shapeA,worldTransformA,
			contactThreshold);
		
		
		for(int i=0;i<localContacts.getNumContacts();i++) {
			contacts.addContactPoint(
				localContacts.getDistance(i),
				-localContacts.getNormal(i),
				offsetTransformA * localContacts.getLocalPointB(i),
				offsetTransformB * localContacts.getLocalPointA(i),
				localContacts.getSubData(i));
		}
	}
}
///////////////////////////////////////////////////////////////////////////////
// Collision Detection Function Table

pfx_detect_collision_func funcTbl_detectCollision[kPfxShapeCount][kPfxShapeCount] = {
	{detectCollisionSphereSphere	,detectCollisionSphereBox	,detectCollisionSphereCapsule	,detectCollisionGjk			,detectCollisionGjk				,detectCollisionLargeTriMesh,	detectCollisionDummy,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy,	detectCollisionDummy},
	{detectCollisionBoxSphere		,detectCollisionBoxBox		,detectCollisionBoxCapsule		,detectCollisionGjk			,detectCollisionGjk				,detectCollisionLargeTriMesh,	detectCollisionDummy,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy,	detectCollisionDummy},
	{detectCollisionCapsuleSphere	,detectCollisionCapsuleBox	,detectCollisionCapsuleCapsule	,detectCollisionGjk			,detectCollisionGjk				,detectCollisionLargeTriMesh,	detectCollisionDummy,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy,	detectCollisionDummy},
	{detectCollisionGjk				,detectCollisionGjk			,detectCollisionGjk				,detectCollisionGjk			,detectCollisionGjk				,detectCollisionLargeTriMesh,	detectCollisionDummy,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy,	detectCollisionDummy},
	{detectCollisionGjk				,detectCollisionGjk			,detectCollisionGjk				,detectCollisionGjk			,detectCollisionGjk				,detectCollisionLargeTriMesh,	detectCollisionDummy,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy,	detectCollisionDummy},
	{detectCollisionLargeTriMesh	,detectCollisionLargeTriMesh,detectCollisionLargeTriMesh	,detectCollisionLargeTriMesh,detectCollisionLargeTriMesh	,detectCollisionDummy,	detectCollisionDummy,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy,	detectCollisionDummy},
	{detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy,	detectCollisionDummy,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy,	detectCollisionDummy},
	{detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy,	detectCollisionDummy,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy,	detectCollisionDummy},
	{detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy,	detectCollisionDummy,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy,	detectCollisionDummy},
	{detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy,	detectCollisionDummy,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy,	detectCollisionDummy},
	{detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy,	detectCollisionDummy,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy,	detectCollisionDummy},
	{detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy,	detectCollisionDummy,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy	,detectCollisionDummy,	detectCollisionDummy},
};

///////////////////////////////////////////////////////////////////////////////
// Collision Detection Function Table Interface

pfx_detect_collision_func pfxGetDetectCollisionFunc(PfxUInt8 shapeTypeA,PfxUInt8 shapeTypeB)
{
	SCE_PFX_ASSERT(shapeTypeA<kPfxShapeCount);
	SCE_PFX_ASSERT(shapeTypeB<kPfxShapeCount);
	return funcTbl_detectCollision[shapeTypeA][shapeTypeB];
}

int pfxSetDetectCollisionFunc(PfxUInt8 shapeTypeA,PfxUInt8 shapeTypeB,pfx_detect_collision_func func)
{
	if(shapeTypeA >= kPfxShapeCount || shapeTypeB >= kPfxShapeCount) {
		return SCE_PFX_ERR_OUT_OF_RANGE;
	}
	
	funcTbl_detectCollision[shapeTypeA][shapeTypeB] = func;
	
	return SCE_PFX_OK;
}

} //namespace PhysicsEffects
} //namespace sce
