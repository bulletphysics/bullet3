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

#ifndef _SCE_PFX_CONSTRAINT_ROW_SOLVER_H
#define _SCE_PFX_CONSTRAINT_ROW_SOLVER_H

#include "../../../include/physics_effects/base_level/base/pfx_vec_utils.h"
#include "../../../include/physics_effects/base_level/solver/pfx_constraint_row.h"
#include "../../../include/physics_effects/base_level/solver/pfx_joint_constraint.h"

namespace sce {
namespace PhysicsEffects {

///////////////////////////////////////////////////////////////////////////////
// Constraint Row Solver

// ARA begin insert new code
#ifdef __ARM_NEON__
// prototype for inline NEON assembly version
void pfxSolveLinearConstraintRowNEON(PfxConstraintRow &constraint,
	PfxVector3 &deltaLinearVelocityA,PfxVector3 &deltaAngularVelocityA,
	PfxFloat &massInvA,const PfxMatrix3 &inertiaInvA,const PfxVector3 &rA,
	PfxVector3 &deltaLinearVelocityB,PfxVector3 &deltaAngularVelocityB,
	PfxFloat &massInvB,const PfxMatrix3 &inertiaInvB,const PfxVector3 &rB);
#endif // __ARM_NEON__
// ARA end

static SCE_PFX_FORCE_INLINE
void pfxSolveLinearConstraintRow(PfxConstraintRow &constraint,
	PfxVector3 &deltaLinearVelocityA,PfxVector3 &deltaAngularVelocityA,
	PfxFloat massInvA,const PfxMatrix3 &inertiaInvA,const PfxVector3 &rA,
	PfxVector3 &deltaLinearVelocityB,PfxVector3 &deltaAngularVelocityB,
	PfxFloat massInvB,const PfxMatrix3 &inertiaInvB,const PfxVector3 &rB)
{
const PfxVector3 normal(pfxReadVector3(constraint.m_normal));
PfxFloat deltaImpulse = constraint.m_rhs;
PfxVector3 dVA = deltaLinearVelocityA + cross(deltaAngularVelocityA,rA);
PfxVector3 dVB = deltaLinearVelocityB + cross(deltaAngularVelocityB,rB);
deltaImpulse -= constraint.m_jacDiagInv * dot(normal,dVA-dVB);
PfxFloat oldImpulse = constraint.m_accumImpulse;
constraint.m_accumImpulse = SCE_PFX_CLAMP(oldImpulse + deltaImpulse,constraint.m_lowerLimit,constraint.m_upperLimit);
deltaImpulse = constraint.m_accumImpulse - oldImpulse;
deltaLinearVelocityA += deltaImpulse * massInvA * normal;
deltaAngularVelocityA += deltaImpulse * inertiaInvA * cross(rA,normal);
deltaLinearVelocityB -= deltaImpulse * massInvB * normal;
deltaAngularVelocityB -= deltaImpulse * inertiaInvB * cross(rB,normal);
}

static SCE_PFX_FORCE_INLINE
void pfxSolveAngularConstraintRow(PfxConstraintRow &constraint,
	PfxVector3 &deltaAngularVelocityA,
	const PfxMatrix3 &inertiaInvA,const PfxVector3 &rA,
	PfxVector3 &deltaAngularVelocityB,
	const PfxMatrix3 &inertiaInvB,const PfxVector3 &rB)
{
	(void)rA,(void)rB;
const PfxVector3 normal(pfxReadVector3(constraint.m_normal));
PfxFloat deltaImpulse = constraint.m_rhs;
deltaImpulse -= constraint.m_jacDiagInv * dot(normal,deltaAngularVelocityA-deltaAngularVelocityB);
PfxFloat oldImpulse = constraint.m_accumImpulse;
constraint.m_accumImpulse = SCE_PFX_CLAMP(oldImpulse + deltaImpulse,constraint.m_lowerLimit,constraint.m_upperLimit);
deltaImpulse = constraint.m_accumImpulse - oldImpulse;
deltaAngularVelocityA += deltaImpulse * inertiaInvA * normal;
deltaAngularVelocityB -= deltaImpulse * inertiaInvB * normal;
}


///////////////////////////////////////////////////////////////////////////////
// Calc Joint Angle

static SCE_PFX_FORCE_INLINE
void pfxCalcJointAngleSwingTwist(PfxMatrix3 &worldFrameA,PfxMatrix3 &worldFrameB,PfxFloat *angle,PfxVector3 *axis)
{
	// フレームA座標系への変換マトリクス
	PfxMatrix3 frameBA = transpose(worldFrameA) * worldFrameB;

	// クォータニオン回転をtwistとswingに分離
	PfxQuat swing,twist,qBA(frameBA);
	swing = PfxQuat::rotation(PfxVector3(1.0f,0.0f,0.0f),frameBA.getCol0());
	twist = qBA * conj(swing);

	if(dot(twist,PfxQuat::rotationX(0.0f)) < 0.0f) {
		twist = -twist;
	}

	// それぞれの回転軸と回転角度を算出
	pfxGetRotationAngleAndAxis(normalize(twist),angle[0],axis[0]);
	pfxGetRotationAngleAndAxis(normalize(swing),angle[1],axis[1]);

	if(angle[1] < 0.00001f) {
		axis[1] = PfxVector3(0.0f,1.0f,0.0f);
	}

	// twistの軸方向のチェック
	if(dot(axis[0],frameBA.getCol0()) < 0.0f) {
		angle[0] = -angle[0];
	}

	axis[0] = worldFrameB.getCol0();
	axis[1] = worldFrameA * axis[1];
	axis[2] = cross(axis[0],axis[1]);
	angle[2] = 0.0f;
}

static SCE_PFX_FORCE_INLINE
void pfxCalcJointAngleSwing1Swing2Twist(PfxMatrix3 &worldFrameA,PfxMatrix3 &worldFrameB,PfxFloat *angle,PfxVector3 *axis)
{
	// フレームA座標系への変換マトリクス
	PfxMatrix3 frameBA = transpose(worldFrameA) * worldFrameB;

	// クォータニオン回転をtwistとswingに分離
	PfxQuat swing,twist,qBA(frameBA);
	swing = PfxQuat::rotation(PfxVector3(1.0f,0.0f,0.0f),frameBA.getCol0());
	twist = qBA * conj(swing);

	if(dot(twist,PfxQuat::rotationX(0.0f)) < 0.0f) {
		twist = -twist;
	}

	PfxQuat swing1,swing2;

	PfxVector3 pXY = frameBA.getCol0();pXY[2] = 0.0f;
	PfxVector3 pXZ = frameBA.getCol0();pXZ[1] = 0.0f;

	if(fabsf(frameBA.getCol0()[1]) < fabsf(frameBA.getCol0()[2])) {
		swing1 = PfxQuat::rotation(PfxVector3(1.0f,0.0f,0.0f),pXZ);
		swing2 = swing * conj(swing1);
	}
	else {
		swing2 = PfxQuat::rotation(PfxVector3(1.0f,0.0f,0.0f),pXY);
		swing1 = conj(swing2) * swing;
	}

	// それぞれの回転軸と回転角度を算出
	pfxGetRotationAngleAndAxis(normalize(twist),angle[0],axis[0]);
	pfxGetRotationAngleAndAxis(normalize(swing2),angle[1],axis[1]);
	pfxGetRotationAngleAndAxis(normalize(swing1),angle[2],axis[2]);

	if(angle[1] < 0.00001f) {
		angle[1] = 0.0f;
		axis[1] = PfxVector3(0.0f,1.0f,0.0f);
	}

	if(angle[2] < 0.00001f) {
		angle[2] = 0.0f;
		axis[2] = PfxVector3(0.0f,0.0f,1.0f);
	}

	// twistの軸方向のチェック
	if(dot(axis[0],frameBA.getCol0()) < 0.0f) {
		angle[0] = -angle[0];
	}

	axis[0] = worldFrameB.getCol0();
	axis[1] = worldFrameA * axis[1];
	axis[2] = worldFrameA * axis[2];
}

static SCE_PFX_FORCE_INLINE
void pfxCalcJointAngleUniversal(PfxMatrix3 &worldFrameA,PfxMatrix3 &worldFrameB,PfxFloat *angle,PfxVector3 *axis)
{
	// フレームA座標系への変換マトリクス
	PfxMatrix3 frameBA = transpose(worldFrameA) * worldFrameB;

	// クォータニオン回転をtwistとswingに分離
	PfxQuat swing,swing1,swing2,twist,qBA(frameBA);
	PfxVector3 Pxy(frameBA.getCol0());
	Pxy[2] = 0.0f;
	swing1 = PfxQuat::rotation(PfxVector3(1.0f,0.0f,0.0f),normalize(Pxy));
	swing = PfxQuat::rotation(PfxVector3(1.0f,0.0f,0.0f),frameBA.getCol0());
	swing2 = swing * conj(swing1);
	twist = conj(swing) * qBA;

	if(dot(twist,PfxQuat::rotationX(0.0f)) < 0.0f) {
		twist = -twist;
	}

	pfxGetRotationAngleAndAxis(normalize(twist),angle[0],axis[0]);
	pfxGetRotationAngleAndAxis(normalize(swing1),angle[1],axis[1]);
	pfxGetRotationAngleAndAxis(normalize(swing2),angle[2],axis[2]);

	if(axis[1].getZ() < 0.0f) {
		axis[1] = -axis[1];
		angle[1] = -angle[1];
	}

	PfxVector3 chkY = cross(PfxVector3(0.0f,0.0f,1.0f),frameBA.getCol0());
	if(dot(chkY,axis[2]) < 0.0f) {
		axis[2] = -axis[2];
		angle[2] = -angle[2];
	}

	// twistの軸方向のチェック
	if(dot(axis[0],frameBA.getCol0()) < 0.0f) {
		angle[0] = -angle[0];
	}

	axis[0] = worldFrameB.getCol0();
	axis[1] = worldFrameA * axis[1];
	axis[2] = worldFrameA * axis[2];
}

///////////////////////////////////////////////////////////////////////////////
// Calc Joint Limit

static SCE_PFX_FORCE_INLINE
void pfxCalcLinearLimit(
	const PfxJointConstraint &jointConstraint,
	PfxFloat &posErr,PfxFloat &velocityAmp,PfxFloat &lowerLimit,PfxFloat &upperLimit)
{
	switch(jointConstraint.m_lock) {
		case SCE_PFX_JOINT_LOCK_FREE:
		posErr = 0.0f;
		velocityAmp *= jointConstraint.m_damping;
		break;
		
		case SCE_PFX_JOINT_LOCK_LIMIT:
		if(posErr >= jointConstraint.m_movableLowerLimit && posErr <= jointConstraint.m_movableUpperLimit) {
			posErr = 0.0f;
			velocityAmp *= jointConstraint.m_damping;
		}
		else {
			if(posErr < jointConstraint.m_movableLowerLimit) {
				posErr = posErr - jointConstraint.m_movableLowerLimit;
				posErr = SCE_PFX_MIN(0.0f,posErr+SCE_PFX_JOINT_LINEAR_SLOP);
				upperLimit = SCE_PFX_MIN(0.0f,upperLimit);
				velocityAmp = 1.0f;
			}
			else { // posErr > movableUpperLimit
				posErr = posErr - jointConstraint.m_movableUpperLimit;
				posErr = SCE_PFX_MAX(0.0f,posErr-SCE_PFX_JOINT_LINEAR_SLOP);
				lowerLimit = SCE_PFX_MAX(0.0f,lowerLimit);
				velocityAmp = 1.0f;
			}
		}
		break;
		
		default: // SCE_PFX_JOINT_LOCK_FIX
		break;
	}
}

static SCE_PFX_FORCE_INLINE
void pfxCalcAngularLimit(
	const PfxJointConstraint &jointConstraint,
	PfxFloat &posErr,PfxFloat &velocityAmp,PfxFloat &lowerLimit,PfxFloat &upperLimit)
{
	switch(jointConstraint.m_lock) {
		case SCE_PFX_JOINT_LOCK_FREE:
		posErr = 0.0f;
		velocityAmp *= jointConstraint.m_damping;
		break;
		
		case SCE_PFX_JOINT_LOCK_LIMIT:
		if(posErr >= jointConstraint.m_movableLowerLimit && posErr <= jointConstraint.m_movableUpperLimit) {
			posErr = 0.0f;
			velocityAmp *= jointConstraint.m_damping;
		}
		else {
			if(posErr < jointConstraint.m_movableLowerLimit) {
				posErr = posErr - jointConstraint.m_movableLowerLimit;
				posErr = SCE_PFX_MIN(0.0f,posErr+SCE_PFX_JOINT_ANGULAR_SLOP);
				upperLimit = SCE_PFX_MIN(0.0f,upperLimit);
				velocityAmp = 1.0f;
			}
			else { // posErr > movableUpperLimit
				posErr = posErr - jointConstraint.m_movableUpperLimit;
				posErr = SCE_PFX_MAX(0.0f,posErr-SCE_PFX_JOINT_ANGULAR_SLOP);
				lowerLimit = SCE_PFX_MAX(0.0f,lowerLimit);
				velocityAmp = 1.0f;
			}
		}
		break;
		
		default: // SCE_PFX_JOINT_LOCK_FIX
		break;
	}
}

} //namespace PhysicsEffects
} //namespace sce

#endif // _SCE_PFX_CONSTRAINT_ROW_SOLVER_H
