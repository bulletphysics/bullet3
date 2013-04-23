/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_POINT2POINTCONSTRAINT_H
#define BT_POINT2POINTCONSTRAINT_H

#include "Bullet3Common/b3Vector3.h"
//#include "b3JacobianEntry.h"
#include "b3TypedConstraint.h"

class btRigidBody;


#ifdef BT_USE_DOUBLE_PRECISION
#define btPoint2PointConstraintData	btPoint2PointConstraintDoubleData
#define btPoint2PointConstraintDataName	"btPoint2PointConstraintDoubleData"
#else
#define btPoint2PointConstraintData	btPoint2PointConstraintFloatData
#define btPoint2PointConstraintDataName	"btPoint2PointConstraintFloatData"
#endif //BT_USE_DOUBLE_PRECISION

struct	btConstraintSetting
{
	btConstraintSetting()	:
		m_tau(b3Scalar(0.3)),
		m_damping(b3Scalar(1.)),
		m_impulseClamp(b3Scalar(0.))
	{
	}
	b3Scalar		m_tau;
	b3Scalar		m_damping;
	b3Scalar		m_impulseClamp;
};

enum btPoint2PointFlags
{
	BT_P2P_FLAGS_ERP = 1,
	BT_P2P_FLAGS_CFM = 2
};

/// point to point constraint between two rigidbodies each with a pivotpoint that descibes the 'ballsocket' location in local space
ATTRIBUTE_ALIGNED16(class) b3Point2PointConstraint : public b3TypedConstraint
{
#ifdef IN_PARALLELL_SOLVER
public:
#endif
	
	b3Vector3	m_pivotInA;
	b3Vector3	m_pivotInB;
	
	int			m_flags;
	b3Scalar	m_erp;
	b3Scalar	m_cfm;
	
public:

	BT_DECLARE_ALIGNED_ALLOCATOR();

	///for backwards compatibility during the transition to 'getInfo/getInfo2'
	bool		m_useSolveConstraintObsolete;

	btConstraintSetting	m_setting;

	b3Point2PointConstraint(int  rbA,int rbB, const b3Vector3& pivotInA,const b3Vector3& pivotInB);

	b3Point2PointConstraint(int  rbA,const b3Vector3& pivotInA);


	virtual void	buildJacobian();

	virtual void getInfo1 (btConstraintInfo1* info);

	void getInfo1NonVirtual (btConstraintInfo1* info);

	virtual void getInfo2 (btConstraintInfo2* info, const b3RigidBodyCL* bodies);

	void getInfo2NonVirtual (btConstraintInfo2* info, const b3Transform& body0_trans, const b3Transform& body1_trans);

	void	updateRHS(b3Scalar	timeStep);

	void	setPivotA(const b3Vector3& pivotA)
	{
		m_pivotInA = pivotA;
	}

	void	setPivotB(const b3Vector3& pivotB)
	{
		m_pivotInB = pivotB;
	}

	const b3Vector3& getPivotInA() const
	{
		return m_pivotInA;
	}

	const b3Vector3& getPivotInB() const
	{
		return m_pivotInB;
	}

	///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
	///If no axis is provided, it uses the default axis for this constraint.
	virtual	void	setParam(int num, b3Scalar value, int axis = -1);
	///return the local value of parameter
	virtual	b3Scalar getParam(int num, int axis = -1) const;

//	virtual	int	calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
//	virtual	const char*	serialize(void* dataBuffer, btSerializer* serializer) const;


};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	btPoint2PointConstraintFloatData
{
	btTypedConstraintData	m_typeConstraintData;
	btVector3FloatData	m_pivotInA;
	btVector3FloatData	m_pivotInB;
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	btPoint2PointConstraintDoubleData
{
	btTypedConstraintData	m_typeConstraintData;
	btVector3DoubleData	m_pivotInA;
	btVector3DoubleData	m_pivotInB;
};

/*
SIMD_FORCE_INLINE	int	b3Point2PointConstraint::calculateSerializeBufferSize() const
{
	return sizeof(btPoint2PointConstraintData);

}

	///fills the dataBuffer and returns the struct name (and 0 on failure)
SIMD_FORCE_INLINE	const char*	b3Point2PointConstraint::serialize(void* dataBuffer, btSerializer* serializer) const
{
	btPoint2PointConstraintData* p2pData = (btPoint2PointConstraintData*)dataBuffer;

	b3TypedConstraint::serialize(&p2pData->m_typeConstraintData,serializer);
	m_pivotInA.serialize(p2pData->m_pivotInA);
	m_pivotInB.serialize(p2pData->m_pivotInB);

	return btPoint2PointConstraintDataName;
}
*/

#endif //BT_POINT2POINTCONSTRAINT_H
