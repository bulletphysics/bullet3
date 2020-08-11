#include "btBulletDynamicsCommon.h"

//for inverse dynamics, DeepMimic implementation
#include "RBDModel.h"
#include "RBDUtil.h"
#include "KinTree.h"

//for BulletInverseDynamics
//#include "BulletInverseDynamics/IDConfig.hpp"
//#include "../Extras/InverseDynamics/btMultiBodyTreeCreator.hpp"

//#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
//#include "BulletDynamics/MLCPSolvers/btLemkeSolver.h"
//#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"

#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyMLCPConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyLink.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
#include "BulletDynamics/Featherstone/btMultiBodyPoint2Point.h"
#include "BulletDynamics/Featherstone/btMultiBodyFixedConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBodySliderConstraint.h"


struct TempLink
{
	int m_parentIndex;
	const btCollisionObject* m_collider;
	double m_mass;
	int m_jointType;
	int m_dofOffset;
	int m_dofCount;
	btVector3 m_dVector;
	btVector3 m_eVector;
	btQuaternion m_zeroRotParentToThis;
	btQuaternion m_this_to_body1;
};

bool btExtractJointBodyFromTempLinks(btAlignedObjectArray<TempLink>& links, Eigen::MatrixXd& bodyDefs, Eigen::MatrixXd& jointMat)
{

	bool result = true;

	int num_joints = links.size();

	btAlignedObjectArray<btVector3> bodyToLinkPositions;
	btAlignedObjectArray<btQuaternion> bodyToLinkRotations;
	btAlignedObjectArray<btQuaternion> dVectorRot;
	bodyToLinkRotations.resize(num_joints);
	bodyToLinkPositions.resize(num_joints);
	dVectorRot.resize(num_joints);

	jointMat.resize(num_joints, 19);
	bodyDefs.resize(num_joints, 17);
	for (int i = 0; i < num_joints * 19; i++)
	{
		jointMat(i) = SIMD_INFINITY;
	}
	for (int i = 0; i < num_joints * 17; i++)
	{
		bodyDefs(i) = SIMD_INFINITY;
	}

	for (int i = 0; i < num_joints * 17; i++)
	{
		bodyDefs(i) = SIMD_INFINITY;
	}

	btScalar unk = -12345;

	int totalDofs = 0;

	for (int j = 0; j < num_joints; ++j)
	{

		int i = j;
		int parentIndex = links[j].m_parentIndex;

		cShape::eShape shapeType = cShape::eShapeNull;
		double param0 = 0, param1 = 0, param2 = 0;
		if (links[j].m_collider)
		{
			const btCollisionShape* collisionShape = links[j].m_collider->getCollisionShape();
			if (collisionShape->isCompound())
			{
				const btCompoundShape* compound = (const btCompoundShape*)collisionShape;
				if (compound->getNumChildShapes() > 0)
				{
					collisionShape = compound->getChildShape(0);
				}
			}
			
			switch (collisionShape->getShapeType())
			{
			case BOX_SHAPE_PROXYTYPE:
			{
				shapeType = cShape::eShapeBox;
				btBoxShape* box = (btBoxShape*)collisionShape;
				param0 = box->getHalfExtentsWithMargin()[0] * 2;
				param1 = box->getHalfExtentsWithMargin()[1] * 2;
				param2 = box->getHalfExtentsWithMargin()[2] * 2;
				
				break;
			}
			case SPHERE_SHAPE_PROXYTYPE:
			{
				btSphereShape* sphere = (btSphereShape*)collisionShape;
				param0 = sphere->getRadius() * 2;
				param1 = sphere->getRadius() * 2;
				param2 = sphere->getRadius() * 2;
				shapeType = cShape::eShapeSphere;
				break;
			}
			case CAPSULE_SHAPE_PROXYTYPE:
			{
				btCapsuleShape* caps = (btCapsuleShape*)collisionShape;
				param0 = caps->getRadius() * 2;
				param1 = caps->getHalfHeight() * 2;
				param2 = caps->getRadius() * 2;
				shapeType = cShape::eShapeCapsule;
				break;
			}
			default:
			{
				//approximate by its box
				btTransform identity;
				identity.setIdentity();
				btVector3 aabbMin, aabbMax;
				collisionShape->getAabb(identity, aabbMin, aabbMax);
				btVector3 halfExtents = (aabbMax - aabbMin) * btScalar(0.5);
				btScalar margin = collisionShape->getMargin();
				btScalar lx = btScalar(2.) * (halfExtents.x() + margin);
				btScalar ly = btScalar(2.) * (halfExtents.y() + margin);
				btScalar lz = btScalar(2.) * (halfExtents.z() + margin);
				param0 = lx;
				param1 = ly;
				param2 = lz;
				shapeType = cShape::eShapeBox;
			}
			}
		}


		btVector3 body_attach_pt1 = links[j].m_dVector;

		//tQuaternion body_to_parent_body = parent_to_parent_body * this_to_parent * body_to_this;
		//tQuaternion parent_to_parent_body = parent_body_to_parent.inverse();


		bodyDefs(i, cKinTree::eBodyParam0) = param0;
		bodyDefs(i, cKinTree::eBodyParam1) = param1;
		bodyDefs(i, cKinTree::eBodyParam2) = param2;

		bodyDefs(i, cKinTree::eBodyParamShape) = shapeType;
		bodyDefs(i, cKinTree::eBodyParamMass) = links[j].m_mass;


		bodyDefs(i, cKinTree::eBodyParamColGroup) = unk;
		bodyDefs(i, cKinTree::eBodyParamEnableFallContact) = unk;

		bodyDefs(i, cKinTree::eBodyColorR) = unk;
		bodyDefs(i, cKinTree::eBodyColorG) = unk;
		bodyDefs(i, cKinTree::eBodyColorB) = unk;
		bodyDefs(i, cKinTree::eBodyColorA) = unk;

		dVectorRot[j] = links[j].m_this_to_body1;

		btVector3 body_attach_pt2 = quatRotate(links[j].m_this_to_body1.inverse(), body_attach_pt1);
		bodyToLinkPositions[i] = body_attach_pt2;
		bodyDefs(i, cKinTree::eBodyParamAttachX) = body_attach_pt2[0];
		bodyDefs(i, cKinTree::eBodyParamAttachY) = body_attach_pt2[1];
		bodyDefs(i, cKinTree::eBodyParamAttachZ) = body_attach_pt2[2];
		btScalar bodyAttachThetaX = 0;
		btScalar bodyAttachThetaY = 0;
		btScalar bodyAttachThetaZ = 0;
		btQuaternion body_to_this1 = links[j].m_this_to_body1.inverse();



		body_to_this1.getEulerZYX(bodyAttachThetaZ, bodyAttachThetaY, bodyAttachThetaX);
		bodyDefs(i, cKinTree::eBodyParamAttachThetaX) = bodyAttachThetaX;
		bodyDefs(i, cKinTree::eBodyParamAttachThetaY) = bodyAttachThetaY;
		bodyDefs(i, cKinTree::eBodyParamAttachThetaZ) = bodyAttachThetaZ;

		jointMat(i, cKinTree::eJointDescType) = links[j].m_jointType;
		jointMat(i, cKinTree::eJointDescParent) = parentIndex;


		btVector3 jointAttachPointMy = links[j].m_eVector;
		btVector3 jointAttachPointMyv0 = jointAttachPointMy;
		btVector3 parentBodyAttachPtMy(0, 0, 0);
		btQuaternion parentBodyToLink;
		parentBodyToLink = btQuaternion::getIdentity();
		btQuaternion linkToParentBody = btQuaternion::getIdentity();
		int parent_joint = links[j].m_parentIndex;

		if (parent_joint != gInvalidIdx)
		{
			parentBodyAttachPtMy = bodyToLinkPositions[parent_joint];
			parentBodyToLink = bodyToLinkRotations[parent_joint];
			linkToParentBody = parentBodyToLink.inverse();
		}
		parentBodyAttachPtMy = quatRotate(linkToParentBody, parentBodyAttachPtMy);
		//bodyToLinkRotations
		jointAttachPointMy += parentBodyAttachPtMy;
		jointAttachPointMy = quatRotate(linkToParentBody.inverse(), jointAttachPointMy);

		btVector3 parent_body_attach_pt1(0, 0, 0);
		if (parentIndex >= 0)
		{
			parent_body_attach_pt1 = links[parentIndex].m_dVector;
		}
		btQuaternion myparent_body_to_body(0, 0, 0, 1);
		btQuaternion mybody_to_parent_body(0, 0, 0, 1);
		btQuaternion parent_body_to_body1 = links[i].m_zeroRotParentToThis;
		btQuaternion body_to_parent_body1 = parent_body_to_body1.inverse();

		bodyToLinkRotations[i] = body_to_this1;

		jointMat(i, cKinTree::eJointDescAttachX) = jointAttachPointMy[0];
		jointMat(i, cKinTree::eJointDescAttachY) = jointAttachPointMy[1];
		jointMat(i, cKinTree::eJointDescAttachZ) = jointAttachPointMy[2];


		btQuaternion parent2parent_body2(0, 0, 0, 1);

		if (parent_joint >= 0)
		{
			//parent2parent_body2 = bulletMB->getLink(parent_joint).m_dVectorRot;
			parent2parent_body2 = dVectorRot[parent_joint];
		}
		///btQuaternion this2bodyA = bulletMB->getLink(j).m_dVectorRot;
		btQuaternion this2bodyA = dVectorRot[j];

		btQuaternion parent_body_2_body = links[j].m_zeroRotParentToThis;
		btQuaternion combined2 = parent_body_2_body.inverse();
		btQuaternion recoverthis2parent = parent2parent_body2.inverse()*combined2*this2bodyA;// body2this.inverse();
		btScalar eulZ, eulY, eulX;
		recoverthis2parent.getEulerZYX(eulZ, eulY, eulX);


		jointMat(i, cKinTree::eJointDescAttachThetaX) = eulX;
		jointMat(i, cKinTree::eJointDescAttachThetaY) = eulY;
		jointMat(i, cKinTree::eJointDescAttachThetaZ) = eulZ;


		jointMat(i, cKinTree::eJointDescLimLow0) = unk;
		jointMat(i, cKinTree::eJointDescLimLow1) = unk;
		jointMat(i, cKinTree::eJointDescLimLow2) = unk;
		jointMat(i, cKinTree::eJointDescLimHigh0) = unk;
		jointMat(i, cKinTree::eJointDescLimHigh1) = unk;
		jointMat(i, cKinTree::eJointDescLimHigh2) = unk;
		jointMat(i, cKinTree::eJointDescTorqueLim) = unk;

		jointMat(i, cKinTree::eJointDescForceLim) = unk;
		jointMat(i, cKinTree::eJointDescIsEndEffector) = unk;
		jointMat(i, cKinTree::eJointDescDiffWeight) = unk;
		jointMat(i, cKinTree::eJointDescParamOffset) = totalDofs;
		totalDofs += links[j].m_dofCount;

	}
	return result;
}


void btExtractJointBodyFromBullet(const btMultiBody* bulletMB, Eigen::MatrixXd& bodyDefs, Eigen::MatrixXd& jointMat)
{
	btAlignedObjectArray<TempLink> links;

	int numBaseShapes = 0;
	if (bulletMB->getBaseCollider())
	{
		switch (bulletMB->getBaseCollider()->getCollisionShape()->getShapeType())
		{
			case CAPSULE_SHAPE_PROXYTYPE:
			case SPHERE_SHAPE_PROXYTYPE:
			case BOX_SHAPE_PROXYTYPE:
			{
				numBaseShapes++;
				break;
			}
			case COMPOUND_SHAPE_PROXYTYPE:
			{
				btCompoundShape* compound = (btCompoundShape*)bulletMB->getBaseCollider()->getCollisionShape();
				numBaseShapes += compound->getNumChildShapes();
				break;
			}
			default:
			{
			}
		}

	}

	//links include the 'base' and its childlinks
	int baseLink = numBaseShapes? 1 : 0;
	links.resize(bulletMB->getNumLinks() + baseLink);
	for (int i = 0; i < links.size(); i++)
	{
		memset(&links[i], 0xffffffff, sizeof(TempLink));
	}

	int totalDofs = 0;
	if (numBaseShapes)
	{
		//links[0] is the root/base
		links[0].m_parentIndex = -1;
		links[0].m_collider = bulletMB->getBaseCollider();
		links[0].m_mass = bulletMB->getBaseMass();
		links[0].m_jointType = (bulletMB->hasFixedBase()) ? cKinTree::eJointTypeFixed : cKinTree::eJointTypeNone;
		links[0].m_dofOffset = 0;
		links[0].m_dofCount = 7;
		links[0].m_dVector.setValue(0, 0, 0);
		links[0].m_eVector.setValue(0, 0, 0);
		links[0].m_zeroRotParentToThis = btQuaternion(0, 0, 0, 1);
		links[0].m_this_to_body1 = btQuaternion(0, 0, 0, 1);
		totalDofs = 7;
	}
	

	for (int j = 0; j < bulletMB->getNumLinks(); ++j)
	{
		int parentIndex = bulletMB->getLink(j).m_parent;
		links[j + baseLink].m_parentIndex = parentIndex + baseLink;
		links[j + baseLink].m_collider = bulletMB->getLinkCollider(j);
		links[j + baseLink].m_mass = bulletMB->getLink(j).m_mass;

		int jointType = 0;
		btQuaternion this_to_body1(0, 0, 0, 1);
		int dofCount = 0;

		if ((baseLink)==0 &&j == 0)//for 'root' either use fixed or none
		{
			dofCount = 7;
			links[j].m_parentIndex = -1;
			links[j].m_jointType = (bulletMB->hasFixedBase()) ? cKinTree::eJointTypeFixed : cKinTree::eJointTypeNone;
			links[j].m_dofOffset = 0;
			links[j].m_dofCount = dofCount;
			
			
			links[j].m_zeroRotParentToThis = btQuaternion(0, 0, 0, 1);
			//links[j].m_dVector.setValue(0, 0, 0);
			links[j].m_dVector = bulletMB->getLink(j).m_dVector;
			links[j].m_eVector.setValue(0, 0, 0);
			//links[j].m_eVector = bulletMB->getLink(j).m_eVector;
			links[j].m_zeroRotParentToThis = bulletMB->getLink(j).m_zeroRotParentToThis;

			links[j].m_this_to_body1 = btQuaternion(0, 0, 0, 1);
			totalDofs = 7;
		}
		else
		{

			switch (bulletMB->getLink(j).m_jointType)
			{
			case btMultibodyLink::eFixed:
			{
				jointType = cKinTree::eJointTypeFixed;
				break;
			}
			case btMultibodyLink::ePrismatic:
			{
				dofCount = 1;
				btVector3 refAxis(1, 0, 0);
				btVector3 axis = bulletMB->getLink(j).getAxisTop(0);
				this_to_body1 = shortestArcQuat(refAxis, btVector3(axis[0], axis[1], axis[2]));
				jointType = cKinTree::eJointTypePrismatic;
				break;
			}
			case btMultibodyLink::eSpherical:
			{
				dofCount = 4;//??
				jointType = cKinTree::eJointTypeSpherical;
				break;
			}
			case btMultibodyLink::eRevolute:
			{
				dofCount = 1;
				btVector3 refAxis(0, 0, 1);
				btVector3 axis = bulletMB->getLink(j).getAxisTop(0);
				this_to_body1 = shortestArcQuat(refAxis, btVector3(axis[0], axis[1], axis[2]));
				jointType = cKinTree::eJointTypeRevolute;
				break;
			}
			default:
			{
			}
			}
			links[j + baseLink].m_jointType = jointType;
			links[j + baseLink].m_dofOffset = totalDofs;
			links[j + baseLink].m_dofCount = dofCount;
			links[j + baseLink].m_dVector = bulletMB->getLink(j).m_dVector;
			links[j + baseLink].m_eVector = bulletMB->getLink(j).m_eVector;
			links[j + baseLink].m_zeroRotParentToThis = bulletMB->getLink(j).m_zeroRotParentToThis;
			links[j + baseLink].m_this_to_body1 = this_to_body1;
		}

		totalDofs += dofCount;
	}

	btExtractJointBodyFromTempLinks(links, bodyDefs, jointMat);
}
