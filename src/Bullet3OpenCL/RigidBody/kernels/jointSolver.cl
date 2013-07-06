

typedef float4 Quaternion;

typedef struct
{
	float4 m_row[3];
}Matrix3x3;



typedef struct
{
	Matrix3x3 m_invInertia;
	Matrix3x3 m_initInvInertia;
} Shape;

typedef struct
{
	Matrix3x3 m_basis;//orientation
	float4	m_origin;//transform
}b3Transform;

typedef struct
{
	b3Transform		m_worldTransform;
	float4		m_deltaLinearVelocity;
	float4		m_deltaAngularVelocity;
	float4		m_angularFactor;
	float4		m_linearFactor;
	float4		m_invMass;
	float4		m_pushVelocity;
	float4		m_turnVelocity;
	float4		m_linearVelocity;
	float4		m_angularVelocity;

	union 
	{
		void*	m_originalBody;
		int		m_originalBodyIndex;
	};
	int padding[3];

} b3SolverBody;



typedef struct
{

	float4		m_relpos1CrossNormal;
	float4		m_contactNormal;

	float4		m_relpos2CrossNormal;
	//float4		m_contactNormal2;//usually m_contactNormal2 == -m_contactNormal

	float4		m_angularComponentA;
	float4		m_angularComponentB;
	
	float4	m_appliedPushImpulse;
	float4	m_appliedImpulse;

	float	m_friction;
	float	m_jacDiagABInv;
	float		m_rhs;
	float		m_cfm;
	
    float		m_lowerLimit;
	float		m_upperLimit;
	float		m_rhsPenetration;

    union
	{
		void*		m_originalContactPoint;
		float m_unusedPadding4;
	};

	int	m_overrideNumSolverIterations;
    int			m_frictionIndex;
	int m_solverBodyIdA;
	int m_solverBodyIdB;

} b3SolverConstraint;

typedef struct
{
	int m_bodyAPtrAndSignBit;
	int m_bodyBPtrAndSignBit;
	int	m_constraintRowOffset;
	short int	m_numConstraintRows;
	short int m_batchId;

} b3BatchConstraint;

#define mymake_float4 (float4)


__inline float dot3F4(float4 a, float4 b)
{
	float4 a1 = mymake_float4(a.xyz,0.f);
	float4 b1 = mymake_float4(b.xyz,0.f);
	return dot(a1, b1);
}

__inline void internalApplyImpulse(__global b3SolverBody* body,  float4 linearComponent, float4 angularComponent,float impulseMagnitude)
{
	body->m_deltaLinearVelocity += linearComponent*impulseMagnitude*body->m_linearFactor;
	body->m_deltaAngularVelocity += angularComponent*(impulseMagnitude*body->m_angularFactor);
}


void resolveSingleConstraintRowGeneric(__global b3SolverBody* body1, __global b3SolverBody* body2, __global b3SolverConstraint* c)
{
	float deltaImpulse = c->m_rhs-c->m_appliedImpulse.x*c->m_cfm;
	float deltaVel1Dotn	=	dot3F4(c->m_contactNormal,body1->m_deltaLinearVelocity) 	+ dot3F4(c->m_relpos1CrossNormal,body1->m_deltaAngularVelocity);
	float deltaVel2Dotn	=	-dot3F4(c->m_contactNormal,body2->m_deltaLinearVelocity) + dot3F4(c->m_relpos2CrossNormal,body2->m_deltaAngularVelocity);

	deltaImpulse	-=	deltaVel1Dotn*c->m_jacDiagABInv;
	deltaImpulse	-=	deltaVel2Dotn*c->m_jacDiagABInv;

	float sum = c->m_appliedImpulse.x + deltaImpulse;
	if (sum < c->m_lowerLimit)
	{
		deltaImpulse = c->m_lowerLimit-c->m_appliedImpulse.x;
		c->m_appliedImpulse.x = c->m_lowerLimit;
	}
	else if (sum > c->m_upperLimit) 
	{
		deltaImpulse = c->m_upperLimit-c->m_appliedImpulse.x;
		c->m_appliedImpulse.x = c->m_upperLimit;
	}
	else
	{
		c->m_appliedImpulse.x = sum;
	}

	internalApplyImpulse(body1,c->m_contactNormal*body1->m_invMass,c->m_angularComponentA,deltaImpulse);
	internalApplyImpulse(body2,-c->m_contactNormal*body2->m_invMass,c->m_angularComponentB,deltaImpulse);

}

__kernel
void solveJointConstraintRows(__global b3SolverBody* solverBodies,
					  __global b3BatchConstraint* batchConstraints,
					  	__global b3SolverConstraint* rows,
						int batchOffset,
						int constraintOffset,
						int numConstraintsInBatch
                      )
{
	int b = get_global_id(0);
	if (b>=numConstraintsInBatch)
		return;

	__global b3BatchConstraint* c = &batchConstraints[b+batchOffset];
	for (int jj=0;jj<c->m_numConstraintRows;jj++)
	{
		__global b3SolverConstraint* constraint = &rows[c->m_constraintRowOffset+jj];
		resolveSingleConstraintRowGeneric(&solverBodies[constraint->m_solverBodyIdA],&solverBodies[constraint->m_solverBodyIdB],constraint);
	}
};