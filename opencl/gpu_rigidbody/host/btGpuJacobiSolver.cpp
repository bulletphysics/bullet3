
#include "btGpuJacobiSolver.h"
#include "BulletCommon/btAlignedObjectArray.h"
#include "parallel_primitives/host/btPrefixScanCL.h"
#include "btGpuConstraint4.h"
#include "BulletCommon/btQuickprof.h"

struct btGpuJacobiSolverInternalData
{
		//btRadixSort32CL*	m_sort32;
		//btBoundSearchCL*	m_search;
		btPrefixScanCL*	m_scan;
};

btGpuJacobiSolver::btGpuJacobiSolver(cl_context ctx, cl_device_id device, cl_command_queue queue, int pairCapacity)
	:m_context(ctx),
	m_device(device),
	m_queue(queue)
{
	m_data = new btGpuJacobiSolverInternalData;
	m_data->m_scan = new btPrefixScanCL(m_context,m_device,m_queue);
}

btGpuJacobiSolver::~btGpuJacobiSolver()
{
	delete m_data->m_scan;
	delete m_data;
}


btVector3 make_float4(float v)
{
	return btVector3 (v,v,v);
}

btVector4 make_float4(float x,float y, float z, float w)
{
	return btVector4 (x,y,z,w);
}


	static
	inline
	float calcRelVel(const btVector3& l0, const btVector3& l1, const btVector3& a0, const btVector3& a1, 
					 const btVector3& linVel0, const btVector3& angVel0, const btVector3& linVel1, const btVector3& angVel1)
	{
		return btDot(l0, linVel0) + btDot(a0, angVel0) + btDot(l1, linVel1) + btDot(a1, angVel1);
	}


	static
	inline
	void setLinearAndAngular(const btVector3& n, const btVector3& r0, const btVector3& r1,
							 btVector3& linear, btVector3& angular0, btVector3& angular1)
	{
		linear = -n;
		angular0 = -btCross(r0, n);
		angular1 = btCross(r1, n);
	}


template<bool JACOBI>
static
__inline
void solveContact(btGpuConstraint4& cs, 
	const btVector3& posA, btVector3& linVelA, btVector3& angVelA, float invMassA, const btMatrix3x3& invInertiaA,
	const btVector3& posB, btVector3& linVelB, btVector3& angVelB, float invMassB, const btMatrix3x3& invInertiaB, 
	float maxRambdaDt[4], float minRambdaDt[4])
{

	btVector3 dLinVelA; dLinVelA.setZero();
	btVector3 dAngVelA; dAngVelA.setZero();
	btVector3 dLinVelB; dLinVelB.setZero();
	btVector3 dAngVelB; dAngVelB.setZero();

	for(int ic=0; ic<4; ic++)
	{
		//	dont necessary because this makes change to 0
		if( cs.m_jacCoeffInv[ic] == 0.f ) continue;

		{
			btVector3 angular0, angular1, linear;
			btVector3 r0 = cs.m_worldPos[ic] - (btVector3&)posA;
			btVector3 r1 = cs.m_worldPos[ic] - (btVector3&)posB;
			setLinearAndAngular( (const btVector3 &)-cs.m_linear, (const btVector3 &)r0, (const btVector3 &)r1, linear, angular0, angular1 );

			float rambdaDt = calcRelVel((const btVector3 &)cs.m_linear,(const btVector3 &) -cs.m_linear, angular0, angular1,
				linVelA, angVelA, linVelB, angVelB ) + cs.m_b[ic];
			rambdaDt *= cs.m_jacCoeffInv[ic];

			{
				float prevSum = cs.m_appliedRambdaDt[ic];
				float updated = prevSum;
				updated += rambdaDt;
				updated = btMax( updated, minRambdaDt[ic] );
				updated = btMin( updated, maxRambdaDt[ic] );
				rambdaDt = updated - prevSum;
				cs.m_appliedRambdaDt[ic] = updated;
			}

			btVector3 linImp0 = invMassA*linear*rambdaDt;
			btVector3 linImp1 = invMassB*(-linear)*rambdaDt;
			btVector3 angImp0 = (invInertiaA* angular0)*rambdaDt;
			btVector3 angImp1 = (invInertiaB* angular1)*rambdaDt;
#ifdef _WIN32
            btAssert(_finite(linImp0.x()));
			btAssert(_finite(linImp1.x()));
#endif
			if( JACOBI )
			{
				dLinVelA += linImp0;
				dAngVelA += angImp0;
				dLinVelB += linImp1;
				dAngVelB += angImp1;
			}
			else
			{
				linVelA += linImp0;
				angVelA += angImp0;
				linVelB += linImp1;
				angVelB += angImp1;
			}
		}
	}

	if( JACOBI )
	{
		linVelA += dLinVelA;
		angVelA += dAngVelA;
		linVelB += dLinVelB;
		angVelB += dAngVelB;
	}

}





static inline void solveFriction(btGpuConstraint4& cs, 
	const btVector3& posA, btVector3& linVelA, btVector3& angVelA, float invMassA, const btMatrix3x3& invInertiaA,
	const btVector3& posB, btVector3& linVelB, btVector3& angVelB, float invMassB, const btMatrix3x3& invInertiaB, 
	float maxRambdaDt[4], float minRambdaDt[4])
{

	if( cs.m_fJacCoeffInv[0] == 0 && cs.m_fJacCoeffInv[0] == 0 ) return;
	const btVector3& center = (const btVector3&)cs.m_center;

	btVector3 n = -(const btVector3&)cs.m_linear;

	btVector3 tangent[2];
#if 1		
	btPlaneSpace1 (n, tangent[0],tangent[1]);
#else
	btVector3 r = cs.m_worldPos[0]-center;
	tangent[0] = cross3( n, r );
	tangent[1] = cross3( tangent[0], n );
	tangent[0] = normalize3( tangent[0] );
	tangent[1] = normalize3( tangent[1] );
#endif

	btVector3 angular0, angular1, linear;
	btVector3 r0 = center - posA;
	btVector3 r1 = center - posB;
	for(int i=0; i<2; i++)
	{
		setLinearAndAngular( tangent[i], r0, r1, linear, angular0, angular1 );
		float rambdaDt = calcRelVel(linear, -linear, angular0, angular1,
			linVelA, angVelA, linVelB, angVelB );
		rambdaDt *= cs.m_fJacCoeffInv[i];

			{
				float prevSum = cs.m_fAppliedRambdaDt[i];
				float updated = prevSum;
				updated += rambdaDt;
				updated = btMax( updated, minRambdaDt[i] );
				updated = btMin( updated, maxRambdaDt[i] );
				rambdaDt = updated - prevSum;
				cs.m_fAppliedRambdaDt[i] = updated;
			}

		btVector3 linImp0 = invMassA*linear*rambdaDt;
		btVector3 linImp1 = invMassB*(-linear)*rambdaDt;
		btVector3 angImp0 = (invInertiaA* angular0)*rambdaDt;
		btVector3 angImp1 = (invInertiaB* angular1)*rambdaDt;
#ifdef _WIN32
		btAssert(_finite(linImp0.x()));
		btAssert(_finite(linImp1.x()));
#endif
		linVelA += linImp0;
		angVelA += angImp0;
		linVelB += linImp1;
		angVelB += angImp1;
	}

	{	//	angular damping for point constraint
		btVector3 ab = ( posB - posA ).normalized();
		btVector3 ac = ( center - posA ).normalized();
		if( btDot( ab, ac ) > 0.95f || (invMassA == 0.f || invMassB == 0.f))
		{
			float angNA = btDot( n, angVelA );
			float angNB = btDot( n, angVelB );

			angVelA -= (angNA*0.1f)*n;
			angVelB -= (angNB*0.1f)*n;
		}
	}

}


btVector3 mtMul3(const btVector3& a, const btMatrix3x3& b)
{
	btVector3 colx = make_float4(b.getRow(0)[0], b.getRow(1)[0], b.getRow(2)[0], 0);
	btVector3 coly = make_float4(b.getRow(0)[1], b.getRow(1)[1], b.getRow(2)[1], 0);
	btVector3 colz = make_float4(b.getRow(0)[2], b.getRow(1)[2], b.getRow(2)[2], 0);

	btVector3 ans;
	ans[0] = btDot( a, colx );
	ans[1] = btDot( a, coly );
	ans[2] = btDot( a, colz );
	return ans;
}


float calcJacCoeff(const btVector3& linear0, const btVector3& linear1, const btVector3& angular0, const btVector3& angular1,
					float invMass0, const btMatrix3x3* invInertia0, float invMass1, const btMatrix3x3* invInertia1)
{
	//	linear0,1 are normlized
	float jmj0 = invMass0;//dot3F4(linear0, linear0)*invMass0;
	
	float jmj1 = btDot(mtMul3(angular0,*invInertia0), angular0);
	float jmj2 = invMass1;//dot3F4(linear1, linear1)*invMass1;
	float jmj3 = btDot(mtMul3(angular1,*invInertia1), angular1);
	return -1.f/(jmj0+jmj1+jmj2+jmj3);
}


void setConstraint4( const btVector3& posA, const btVector3& linVelA, const btVector3& angVelA, float invMassA, const btMatrix3x3& invInertiaA,
	const btVector3& posB, const btVector3& linVelB, const btVector3& angVelB, float invMassB, const btMatrix3x3& invInertiaB, 
	 btContact4* src, float dt, float positionDrift, float positionConstraintCoeff,
	btGpuConstraint4* dstC )
{
	dstC->m_bodyA = abs(src->m_bodyAPtrAndSignBit);
	dstC->m_bodyB = abs(src->m_bodyBPtrAndSignBit);

	float dtInv = 1.f/dt;
	for(int ic=0; ic<4; ic++)
	{
		dstC->m_appliedRambdaDt[ic] = 0.f;
	}
	dstC->m_fJacCoeffInv[0] = dstC->m_fJacCoeffInv[1] = 0.f;


	dstC->m_linear = -src->m_worldNormal;
	dstC->m_linear[3] = 0.7f ;//src->getFrictionCoeff() );
	for(int ic=0; ic<4; ic++)
	{
		btVector3 r0 = src->m_worldPos[ic] - posA;
		btVector3 r1 = src->m_worldPos[ic] - posB;

		if( ic >= src->m_worldNormal[3] )//npoints
		{
			dstC->m_jacCoeffInv[ic] = 0.f;
			continue;
		}

		float relVelN;
		{
			btVector3 linear, angular0, angular1;
			setLinearAndAngular(src->m_worldNormal, r0, r1, linear, angular0, angular1);

			dstC->m_jacCoeffInv[ic] = calcJacCoeff(linear, -linear, angular0, angular1,
				invMassA, &invInertiaA, invMassB, &invInertiaB );

			relVelN = calcRelVel(linear, -linear, angular0, angular1,
				linVelA, angVelA, linVelB, angVelB);

			float e = 0.f;//src->getRestituitionCoeff();
			if( relVelN*relVelN < 0.004f ) e = 0.f;

			dstC->m_b[ic] = e*relVelN;
			//float penetration = src->m_worldPos[ic].w;
			dstC->m_b[ic] += (src->m_worldPos[ic][3] + positionDrift)*positionConstraintCoeff*dtInv;
			dstC->m_appliedRambdaDt[ic] = 0.f;
		}
	}

	if( src->m_worldNormal[3] > 0 )//npoints
	{	//	prepare friction
		btVector3 center = make_float4(0.f);
		for(int i=0; i<src->m_worldNormal[3]; i++) 
			center += src->m_worldPos[i];
		center /= (float)src->m_worldNormal[3];

		btVector3 tangent[2];
		btPlaneSpace1(src->m_worldNormal,tangent[0],tangent[1]);
		
		btVector3 r[2];
		r[0] = center - posA;
		r[1] = center - posB;

		for(int i=0; i<2; i++)
		{
			btVector3 linear, angular0, angular1;
			setLinearAndAngular(tangent[i], r[0], r[1], linear, angular0, angular1);

			dstC->m_fJacCoeffInv[i] = calcJacCoeff(linear, -linear, angular0, angular1,
				invMassA, &invInertiaA, invMassB, &invInertiaB );
			dstC->m_fAppliedRambdaDt[i] = 0.f;
		}
		dstC->m_center = center;
	}

	for(int i=0; i<4; i++)
	{
		if( i<src->m_worldNormal[3] )
		{
			dstC->m_worldPos[i] = src->m_worldPos[i];
		}
		else
		{
			dstC->m_worldPos[i] = make_float4(0.f);
		}
	}
}



void ContactToConstraintKernel(btContact4* gContact, btRigidBodyCL* gBodies, btInertiaCL* gShapes, btGpuConstraint4* gConstraintOut, int nContacts,
float dt,
float positionDrift,
float positionConstraintCoeff, int gIdx
)
{
	//int gIdx = 0;//GET_GLOBAL_IDX;
	
	if( gIdx < nContacts )
	{
		int aIdx = abs(gContact[gIdx].m_bodyAPtrAndSignBit);
		int bIdx = abs(gContact[gIdx].m_bodyBPtrAndSignBit);

		btVector3 posA = gBodies[aIdx].m_pos;
		btVector3 linVelA = gBodies[aIdx].m_linVel;
		btVector3 angVelA = gBodies[aIdx].m_angVel;
		float invMassA = gBodies[aIdx].m_invMass;
		btMatrix3x3 invInertiaA = gShapes[aIdx].m_invInertiaWorld;//.m_invInertia;

		btVector3 posB = gBodies[bIdx].m_pos;
		btVector3 linVelB = gBodies[bIdx].m_linVel;
		btVector3 angVelB = gBodies[bIdx].m_angVel;
		float invMassB = gBodies[bIdx].m_invMass;
		btMatrix3x3 invInertiaB = gShapes[bIdx].m_invInertiaWorld;//m_invInertia;

		btGpuConstraint4 cs;

    	setConstraint4( posA, linVelA, angVelA, invMassA, invInertiaA, posB, linVelB, angVelB, invMassB, invInertiaB,
			&gContact[gIdx], dt, positionDrift, positionConstraintCoeff,
			&cs );
		

		
		cs.m_batchIdx = gContact[gIdx].m_batchIdx;

		gConstraintOut[gIdx] = cs;
	}
}


void btGpuJacobiSolver::solveGroup(btRigidBodyCL* bodies,btInertiaCL* inertias,int numBodies,btContact4* manifoldPtr, int numManifolds,btTypedConstraint** constraints,int numConstraints,const btJacobiSolverInfo& solverInfo)
{
	BT_PROFILE("btGpuJacobiSolver::solveGroup");
	/*
	btAlignedObjectArray<unsigned int> bodyCount;
	bodyCount.resize(numBodies);
	for (int i=0;i<numBodies;i++)
		bodyCount[i] = 0;
	
	for (int i=0;i<numManifolds;i++)
	{
		int pa = manifoldPtr[i].m_bodyAPtrAndSignBit;
		int pb = manifoldPtr[i].m_bodyBPtrAndSignBit;

		bool isFixedA = (pa <0) || (pa == solverInfo.m_fixedBodyIndex);
		bool isFixedB = (pb <0) || (pb == solverInfo.m_fixedBodyIndex);

		int bodyIndexA = manifoldPtr[i].getBodyA();
		int bodyIndexB = manifoldPtr[i].getBodyB();

		if (!isFixedA)
		{
			bodyCount[bodyIndexA]++;
		}
		if (!isFixedB)
		{
			bodyCount[bodyIndexB]++;
		} 
	}

	btAlignedObjectArray<unsigned int> offsetSplitBodies;
	offsetSplitBodies.resize(numBodies);
	unsigned int totalNumSplitBodies;
	m_data->m_scan->executeHost(bodyCount,offsetSplitBodies,numBodies,&totalNumSplitBodies);

	btAlignedObjectArray<btRigidBodyCL> splitBodies;
	//splitBodies.resize();

	*/


	btAlignedObjectArray<btGpuConstraint4> contactConstraints;
	contactConstraints.resize(numManifolds);

	for (int i=0;i<numManifolds;i++)
	{
		ContactToConstraintKernel(&manifoldPtr[0],bodies,inertias,&contactConstraints[0],numManifolds,
			solverInfo.m_deltaTime,
			solverInfo.m_positionDrift,
			solverInfo.m_positionConstraintCoeff,i);
	}
	int maxIter = 4;

	for (int iter = 0;iter<maxIter;iter++)
	{
		for(int i=0; i<numManifolds; i++)
		{

			float frictionCoeff = contactConstraints[i].getFrictionCoeff();
			int aIdx = (int)contactConstraints[i].m_bodyA;
			int bIdx = (int)contactConstraints[i].m_bodyB;
			btRigidBodyCL& bodyA = bodies[aIdx];
			btRigidBodyCL& bodyB = bodies[bIdx];

			{
				float maxRambdaDt[4] = {FLT_MAX,FLT_MAX,FLT_MAX,FLT_MAX};
				float minRambdaDt[4] = {0.f,0.f,0.f,0.f};

				solveContact<false>( contactConstraints[i], (btVector3&)bodyA.m_pos, (btVector3&)bodyA.m_linVel, (btVector3&)bodyA.m_angVel, bodyA.m_invMass, inertias[aIdx].m_invInertiaWorld, 
					 (btVector3&)bodyB.m_pos, (btVector3&)bodyB.m_linVel, (btVector3&)bodyB.m_angVel, bodyB.m_invMass, inertias[bIdx].m_invInertiaWorld,
					maxRambdaDt, minRambdaDt );
			}
		}

		//solve friction
		for(int i=0; i<numManifolds; i++)
		{
			float maxRambdaDt[4] = {FLT_MAX,FLT_MAX,FLT_MAX,FLT_MAX};
			float minRambdaDt[4] = {0.f,0.f,0.f,0.f};

			float sum = 0;
			for(int j=0; j<4; j++)
			{
				sum +=contactConstraints[i].m_appliedRambdaDt[j];
			}
			float frictionCoeff = contactConstraints[i].getFrictionCoeff();
			int aIdx = (int)contactConstraints[i].m_bodyA;
			int bIdx = (int)contactConstraints[i].m_bodyB;
			btRigidBodyCL& bodyA = bodies[aIdx];
			btRigidBodyCL& bodyB = bodies[bIdx];

			for(int j=0; j<4; j++)
			{
				maxRambdaDt[j] = frictionCoeff*sum;
				minRambdaDt[j] = -maxRambdaDt[j];
			}

			solveFriction( contactConstraints[i], (btVector3&)bodyA.m_pos, (btVector3&)bodyA.m_linVel, (btVector3&)bodyA.m_angVel, bodyA.m_invMass,inertias[aIdx].m_invInertiaWorld, 
					(btVector3&)bodyB.m_pos, (btVector3&)bodyB.m_linVel, (btVector3&)bodyB.m_angVel, bodyB.m_invMass, inertias[bIdx].m_invInertiaWorld,
					maxRambdaDt, minRambdaDt );
			

		}
	}

}

