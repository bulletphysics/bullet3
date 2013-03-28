
#include "btGpuJacobiSolver.h"
#include "BulletCommon/btAlignedObjectArray.h"
#include "parallel_primitives/host/btPrefixScanCL.h"
#include "btGpuConstraint4.h"
#include "BulletCommon/btQuickprof.h"
#include "../../parallel_primitives/host/btInt2.h"
#include "../../parallel_primitives/host/btFillCL.h"



#include "../../parallel_primitives/host/btLauncherCL.h"


#include "../kernels/solverUtils.h"

#define SOLVER_UTILS_KERNEL_PATH "opencl/gpu_rigidbody/kernels/solverUtils.cl"

struct btGpuJacobiSolverInternalData
{
		//btRadixSort32CL*	m_sort32;
		//btBoundSearchCL*	m_search;
		btPrefixScanCL*	m_scan;

		btOpenCLArray<unsigned int>* m_bodyCount;
		btOpenCLArray<btInt2>*		m_contactConstraintOffsets;
		btOpenCLArray<unsigned int>* m_offsetSplitBodies;

		btOpenCLArray<btVector3>*	m_deltaLinearVelocities;
		btOpenCLArray<btVector3>*	m_deltaAngularVelocities;


		btOpenCLArray<btGpuConstraint4>* m_contactConstraints;

		btFillCL*	m_filler;
		

		cl_kernel	m_countBodiesKernel;
		cl_kernel	m_contactToConstraintSplitKernel;
		cl_kernel	m_clearVelocitiesKernel;
		cl_kernel	m_averageVelocitiesKernel;
		cl_kernel	m_solveContactKernel;
		cl_kernel	m_solveFrictionKernel;



};

btGpuJacobiSolver::btGpuJacobiSolver(cl_context ctx, cl_device_id device, cl_command_queue queue, int pairCapacity)
	:m_context(ctx),
	m_device(device),
	m_queue(queue)
{
	m_data = new btGpuJacobiSolverInternalData;
	m_data->m_scan = new btPrefixScanCL(m_context,m_device,m_queue);
	m_data->m_bodyCount = new btOpenCLArray<unsigned int>(m_context,m_queue);
	m_data->m_filler = new btFillCL(m_context,m_device,m_queue);
	m_data->m_contactConstraintOffsets = new btOpenCLArray<btInt2>(m_context,m_queue);
	m_data->m_offsetSplitBodies = new btOpenCLArray<unsigned int>(m_context,m_queue);
	m_data->m_contactConstraints = new btOpenCLArray<btGpuConstraint4>(m_context,m_queue);
	m_data->m_deltaLinearVelocities = new btOpenCLArray<btVector3>(m_context,m_queue);
	m_data->m_deltaAngularVelocities = new btOpenCLArray<btVector3>(m_context,m_queue);

	cl_int pErrNum;
	const char* additionalMacros="";
	const char* solverUtilsSource = solverUtilsCL;
	{
		cl_program solverUtilsProg= btOpenCLUtils::compileCLProgramFromString( ctx, device, solverUtilsSource, &pErrNum,additionalMacros, SOLVER_UTILS_KERNEL_PATH);
		btAssert(solverUtilsProg);
		m_data->m_countBodiesKernel =  btOpenCLUtils::compileCLKernelFromString( ctx, device, solverUtilsSource, "CountBodiesKernel", &pErrNum, solverUtilsProg,additionalMacros );
		btAssert(m_data->m_countBodiesKernel);

		m_data->m_contactToConstraintSplitKernel  = btOpenCLUtils::compileCLKernelFromString( ctx, device, solverUtilsSource, "ContactToConstraintSplitKernel", &pErrNum, solverUtilsProg,additionalMacros );
		btAssert(m_data->m_contactToConstraintSplitKernel);
		m_data->m_clearVelocitiesKernel  = btOpenCLUtils::compileCLKernelFromString( ctx, device, solverUtilsSource, "ClearVelocitiesKernel", &pErrNum, solverUtilsProg,additionalMacros );
		btAssert(m_data->m_clearVelocitiesKernel);

		m_data->m_averageVelocitiesKernel  = btOpenCLUtils::compileCLKernelFromString( ctx, device, solverUtilsSource, "AverageVelocitiesKernel", &pErrNum, solverUtilsProg,additionalMacros );
		btAssert(m_data->m_averageVelocitiesKernel);

		

		
		m_data->m_solveContactKernel = btOpenCLUtils::compileCLKernelFromString( ctx, device, solverUtilsSource, "SolveContactJacobiKernel", &pErrNum, solverUtilsProg,additionalMacros );
		btAssert(m_data->m_solveContactKernel );

		m_data->m_solveFrictionKernel = btOpenCLUtils::compileCLKernelFromString( ctx, device, solverUtilsSource, "SolveFrictionJacobiKernel", &pErrNum, solverUtilsProg,additionalMacros );
		btAssert(m_data->m_solveFrictionKernel);
	}

}

btGpuJacobiSolver::~btGpuJacobiSolver()
{
	clReleaseKernel(m_data->m_solveContactKernel);
	clReleaseKernel(m_data->m_solveFrictionKernel);
	clReleaseKernel(m_data->m_countBodiesKernel);
	clReleaseKernel(m_data->m_contactToConstraintSplitKernel);
	clReleaseKernel(m_data->m_averageVelocitiesKernel);
	clReleaseKernel(m_data->m_clearVelocitiesKernel );

	delete m_data->m_deltaLinearVelocities;
	delete m_data->m_deltaAngularVelocities;
	delete m_data->m_contactConstraints;
	delete m_data->m_offsetSplitBodies;
	delete m_data->m_contactConstraintOffsets;
	delete m_data->m_bodyCount;
	delete m_data->m_filler;
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


static __inline void solveContact(btGpuConstraint4& cs, 
	const btVector3& posA, const btVector3& linVelARO, const btVector3& angVelARO, float invMassA, const btMatrix3x3& invInertiaA,
	const btVector3& posB, const btVector3& linVelBRO, const btVector3& angVelBRO, float invMassB, const btMatrix3x3& invInertiaB, 
	float maxRambdaDt[4], float minRambdaDt[4], btVector3& dLinVelA, btVector3& dAngVelA, btVector3& dLinVelB, btVector3& dAngVelB)
{


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
				linVelARO+dLinVelA, angVelARO+dAngVelA, linVelBRO+dLinVelB, angVelBRO+dAngVelB ) + cs.m_b[ic];
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
			
			if (invMassA)
			{
				dLinVelA += linImp0;
				dAngVelA += angImp0;
			}
			if (invMassB)
			{
				dLinVelB += linImp1;
				dAngVelB += angImp1;
			}
		}
	}
}



void solveContact3(btGpuConstraint4* cs,
			btVector3* posAPtr, btVector3* linVelA, btVector3* angVelA, float invMassA, const btMatrix3x3& invInertiaA,
			btVector3* posBPtr, btVector3* linVelB, btVector3* angVelB, float invMassB, const btMatrix3x3& invInertiaB,
			btVector3* dLinVelA, btVector3* dAngVelA, btVector3* dLinVelB, btVector3* dAngVelB)
{
	float minRambdaDt = 0;
	float maxRambdaDt = FLT_MAX;

	for(int ic=0; ic<4; ic++)
	{
		if( cs->m_jacCoeffInv[ic] == 0.f ) continue;

		btVector3 angular0, angular1, linear;
		btVector3 r0 = cs->m_worldPos[ic] - *posAPtr;
		btVector3 r1 = cs->m_worldPos[ic] - *posBPtr;
		setLinearAndAngular( -cs->m_linear, r0, r1, linear, angular0, angular1 );

		float rambdaDt = calcRelVel( cs->m_linear, -cs->m_linear, angular0, angular1, 
			*linVelA+*dLinVelA, *angVelA+*dAngVelA, *linVelB+*dLinVelB, *angVelB+*dAngVelB ) + cs->m_b[ic];
		rambdaDt *= cs->m_jacCoeffInv[ic];

		{
			float prevSum = cs->m_appliedRambdaDt[ic];
			float updated = prevSum;
			updated += rambdaDt;
			updated = btMax( updated, minRambdaDt );
			updated = btMin( updated, maxRambdaDt );
			rambdaDt = updated - prevSum;
			cs->m_appliedRambdaDt[ic] = updated;
		}

		btVector3 linImp0 = invMassA*linear*rambdaDt;
		btVector3 linImp1 = invMassB*(-linear)*rambdaDt;
		btVector3 angImp0 = (invInertiaA* angular0)*rambdaDt;
		btVector3 angImp1 = (invInertiaB* angular1)*rambdaDt;

		if (invMassA)
		{
			*dLinVelA += linImp0;
			*dAngVelA += angImp0;
		}
		if (invMassB)
		{
			*dLinVelB += linImp1;
			*dAngVelB += angImp1;
		}
	}
}


static inline void solveFriction(btGpuConstraint4& cs, 
	const btVector3& posA, const btVector3& linVelARO, const btVector3& angVelARO, float invMassA, const btMatrix3x3& invInertiaA,
	const btVector3& posB, const btVector3& linVelBRO, const btVector3& angVelBRO, float invMassB, const btMatrix3x3& invInertiaB, 
	float maxRambdaDt[4], float minRambdaDt[4], btVector3& dLinVelA, btVector3& dAngVelA, btVector3& dLinVelB, btVector3& dAngVelB)
{

	btVector3 linVelA = linVelARO+dLinVelA;
	btVector3 linVelB = linVelBRO+dLinVelB;
	btVector3 angVelA = angVelARO+dAngVelA;
	btVector3 angVelB = angVelBRO+dAngVelB;

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
		if (invMassA)
		{
			dLinVelA += linImp0;
			dAngVelA += angImp0;
		}
		if (invMassB)
		{
			dLinVelB += linImp1;
			dAngVelB += angImp1;
		}
	}

	{	//	angular damping for point constraint
		btVector3 ab = ( posB - posA ).normalized();
		btVector3 ac = ( center - posA ).normalized();
		if( btDot( ab, ac ) > 0.95f || (invMassA == 0.f || invMassB == 0.f))
		{
			float angNA = btDot( n, angVelA );
			float angNB = btDot( n, angVelB );

			if (invMassA)
				dAngVelA -= (angNA*0.1f)*n;
			if (invMassB)
				dAngVelB -= (angNB*0.1f)*n;
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
					float invMass0, const btMatrix3x3* invInertia0, float invMass1, const btMatrix3x3* invInertia1, float countA, float countB)
{
	//	linear0,1 are normlized
	float jmj0 = invMass0;//dot3F4(linear0, linear0)*invMass0;
	
	float jmj1 = btDot(mtMul3(angular0,*invInertia0), angular0);
	float jmj2 = invMass1;//dot3F4(linear1, linear1)*invMass1;
	float jmj3 = btDot(mtMul3(angular1,*invInertia1), angular1);
	return -1.f/((jmj0+jmj1)*countA+(jmj2+jmj3)*countB);
//	return -1.f/((jmj0+jmj1)+(jmj2+jmj3));

}


void setConstraint4( const btVector3& posA, const btVector3& linVelA, const btVector3& angVelA, float invMassA, const btMatrix3x3& invInertiaA,
	const btVector3& posB, const btVector3& linVelB, const btVector3& angVelB, float invMassB, const btMatrix3x3& invInertiaB, 
	 btContact4* src, float dt, float positionDrift, float positionConstraintCoeff, float countA, float countB,
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
				invMassA, &invInertiaA, invMassB, &invInertiaB ,countA,countB);

			relVelN = calcRelVel(linear, -linear, angular0, angular1,
				linVelA, angVelA, linVelB, angVelB);

			float e = 0.f;//src->getRestituitionCoeff();
			if( relVelN*relVelN < 0.004f ) 
			{
				e = 0.f;
			}

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
				invMassA, &invInertiaA, invMassB, &invInertiaB ,countA,countB);
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
float positionConstraintCoeff, int gIdx, btAlignedObjectArray<unsigned int>& bodyCount
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
		float countA = invMassA ? (float)(bodyCount[aIdx]) : 1;
		float countB = invMassB ? (float)(bodyCount[bIdx]) : 1;
    	setConstraint4( posA, linVelA, angVelA, invMassA, invInertiaA, posB, linVelB, angVelB, invMassB, invInertiaB,
			&gContact[gIdx], dt, positionDrift, positionConstraintCoeff,countA,countB,
			&cs );
		

		
		cs.m_batchIdx = gContact[gIdx].m_batchIdx;

		gConstraintOut[gIdx] = cs;
	}
}


void btGpuJacobiSolver::solveGroupHost(btRigidBodyCL* bodies,btInertiaCL* inertias,int numBodies,btContact4* manifoldPtr, int numManifolds,btTypedConstraint** constraints,int numConstraints,const btJacobiSolverInfo& solverInfo)
{
	BT_PROFILE("btGpuJacobiSolver::solveGroup");

	btAlignedObjectArray<unsigned int> bodyCount;
	bodyCount.resize(numBodies);
	for (int i=0;i<numBodies;i++)
		bodyCount[i] = 0;

	btAlignedObjectArray<btInt2> contactConstraintOffsets;
	contactConstraintOffsets.resize(numManifolds);


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
			contactConstraintOffsets[i].x = bodyCount[bodyIndexA];
			bodyCount[bodyIndexA]++;
		}
		if (!isFixedB)
		{
			contactConstraintOffsets[i].y = bodyCount[bodyIndexB];
			bodyCount[bodyIndexB]++;
		} 
	}

	btAlignedObjectArray<unsigned int> offsetSplitBodies;
	offsetSplitBodies.resize(numBodies);
	unsigned int totalNumSplitBodies;
	m_data->m_scan->executeHost(bodyCount,offsetSplitBodies,numBodies,&totalNumSplitBodies);
	int numlastBody = bodyCount[numBodies-1];
	totalNumSplitBodies += numlastBody;

	



	btAlignedObjectArray<btGpuConstraint4> contactConstraints;
	contactConstraints.resize(numManifolds);

	for (int i=0;i<numManifolds;i++)
	{
		ContactToConstraintKernel(&manifoldPtr[0],bodies,inertias,&contactConstraints[0],numManifolds,
			solverInfo.m_deltaTime,
			solverInfo.m_positionDrift,
			solverInfo.m_positionConstraintCoeff,
			i, bodyCount);
	}
	int maxIter = 1;


	btAlignedObjectArray<btVector3> deltaLinearVelocities;
	btAlignedObjectArray<btVector3> deltaAngularVelocities;
	deltaLinearVelocities.resize(totalNumSplitBodies);
	deltaAngularVelocities.resize(totalNumSplitBodies);
	for (int i=0;i<totalNumSplitBodies;i++)
	{
		deltaLinearVelocities[i].setZero();
		deltaAngularVelocities[i].setZero();
	}



	for (int iter = 0;iter<maxIter;iter++)
	{
		int i=0;
		for( i=0; i<numManifolds; i++)
		{

			float frictionCoeff = contactConstraints[i].getFrictionCoeff();
			int aIdx = (int)contactConstraints[i].m_bodyA;
			int bIdx = (int)contactConstraints[i].m_bodyB;
			btRigidBodyCL& bodyA = bodies[aIdx];
			btRigidBodyCL& bodyB = bodies[bIdx];

			btVector3 zero(0,0,0);
			
			btVector3* dlvAPtr=&zero;
			btVector3* davAPtr=&zero;
			btVector3* dlvBPtr=&zero;
			btVector3* davBPtr=&zero;
			
			if (bodyA.getInvMass())
			{
				int bodyOffsetA = offsetSplitBodies[aIdx];
				int constraintOffsetA = contactConstraintOffsets[i].x;
				int splitIndexA = bodyOffsetA+constraintOffsetA;
				dlvAPtr = &deltaLinearVelocities[splitIndexA];
				davAPtr = &deltaAngularVelocities[splitIndexA];
			}

			if (bodyB.getInvMass())
			{
				int bodyOffsetB = offsetSplitBodies[bIdx];
				int constraintOffsetB = contactConstraintOffsets[i].y;
				int splitIndexB= bodyOffsetB+constraintOffsetB;
				dlvBPtr =&deltaLinearVelocities[splitIndexB];
				davBPtr = &deltaAngularVelocities[splitIndexB];
			}



			{
				float maxRambdaDt[4] = {FLT_MAX,FLT_MAX,FLT_MAX,FLT_MAX};
				float minRambdaDt[4] = {0.f,0.f,0.f,0.f};

				solveContact( contactConstraints[i], (btVector3&)bodyA.m_pos, (btVector3&)bodyA.m_linVel, (btVector3&)bodyA.m_angVel, bodyA.m_invMass, inertias[aIdx].m_invInertiaWorld, 
					(btVector3&)bodyB.m_pos, (btVector3&)bodyB.m_linVel, (btVector3&)bodyB.m_angVel, bodyB.m_invMass, inertias[bIdx].m_invInertiaWorld,
					maxRambdaDt, minRambdaDt , *dlvAPtr,*davAPtr,*dlvBPtr,*davBPtr		);


			}
		}

		
		//easy
		for (int i=0;i<numBodies;i++)
		{
			if (bodies[i].getInvMass())
			{
				int bodyOffset = offsetSplitBodies[i];
				int count = bodyCount[i];
				float factor = 1.f/float(count);
				btVector3 averageLinVel;
				averageLinVel.setZero();
				btVector3 averageAngVel;
				averageAngVel.setZero();
				for (int j=0;j<count;j++)
				{
					averageLinVel += deltaLinearVelocities[bodyOffset+j]*factor;
					averageAngVel += deltaAngularVelocities[bodyOffset+j]*factor;
				}
				for (int j=0;j<count;j++)
				{
					deltaLinearVelocities[bodyOffset+j] = averageLinVel;
					deltaAngularVelocities[bodyOffset+j] = averageAngVel;
				}
			}
		}

		
#if 0

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

			btVector3 zero(0,0,0);
			
			btVector3* dlvAPtr=&zero;
			btVector3* davAPtr=&zero;
			btVector3* dlvBPtr=&zero;
			btVector3* davBPtr=&zero;
			
			if (bodyA.getInvMass())
			{
				int bodyOffsetA = offsetSplitBodies[aIdx];
				int constraintOffsetA = contactConstraintOffsets[i].x;
				int splitIndexA = bodyOffsetA+constraintOffsetA;
				dlvAPtr = &deltaLinearVelocities[splitIndexA];
				davAPtr = &deltaAngularVelocities[splitIndexA];
			}

			if (bodyB.getInvMass())
			{
				int bodyOffsetB = offsetSplitBodies[bIdx];
				int constraintOffsetB = contactConstraintOffsets[i].y;
				int splitIndexB= bodyOffsetB+constraintOffsetB;
				dlvBPtr =&deltaLinearVelocities[splitIndexB];
				davBPtr = &deltaAngularVelocities[splitIndexB];
			}

			for(int j=0; j<4; j++)
			{
				maxRambdaDt[j] = frictionCoeff*sum;
				minRambdaDt[j] = -maxRambdaDt[j];
			}

			solveFriction( contactConstraints[i], (btVector3&)bodyA.m_pos, (btVector3&)bodyA.m_linVel, (btVector3&)bodyA.m_angVel, bodyA.m_invMass,inertias[aIdx].m_invInertiaWorld, 
				(btVector3&)bodyB.m_pos, (btVector3&)bodyB.m_linVel, (btVector3&)bodyB.m_angVel, bodyB.m_invMass, inertias[bIdx].m_invInertiaWorld,
				maxRambdaDt, minRambdaDt , *dlvAPtr,*davAPtr,*dlvBPtr,*davBPtr);

		}

		//easy
		for (int i=0;i<numBodies;i++)
		{
			if (bodies[i].getInvMass())
			{
				int bodyOffset = offsetSplitBodies[i];
				int count = bodyCount[i];
				float factor = 1.f/float(count);
				btVector3 averageLinVel;
				averageLinVel.setZero();
				btVector3 averageAngVel;
				averageAngVel.setZero();
				for (int j=0;j<count;j++)
				{
					averageLinVel += deltaLinearVelocities[bodyOffset+j]*factor;
					averageAngVel += deltaAngularVelocities[bodyOffset+j]*factor;
				}
				for (int j=0;j<count;j++)
				{
					deltaLinearVelocities[bodyOffset+j] = averageLinVel;
					deltaAngularVelocities[bodyOffset+j] = averageAngVel;
				}
			}
		}

#endif


	}


	//easy
	for (int i=0;i<numBodies;i++)
	{
		if (bodies[i].getInvMass())
		{
			int bodyOffset = offsetSplitBodies[i];
			int count = bodyCount[i];
			if (count)
			{
				bodies[i].m_linVel += deltaLinearVelocities[bodyOffset];
				bodies[i].m_angVel += deltaAngularVelocities[bodyOffset];
			}
		}
	}
}


void btGpuJacobiSolver::solveGroupMixedHost(btRigidBodyCL* bodiesCPU,btInertiaCL* inertiasCPU,int numBodies,btContact4* manifoldPtrCpu, int numManifolds,btTypedConstraint** constraints,int numConstraints,const btJacobiSolverInfo& solverInfo)
{
	BT_PROFILE("btGpuJacobiSolver::solveGroup");

	btAlignedObjectArray<unsigned int> bodyCount;



	bool useHost = true;
	btAlignedObjectArray<btInt2> contactConstraintOffsets;
	btOpenCLArray<btContact4> manifoldGPU(m_context,m_queue);
	manifoldGPU.resize(numManifolds);
	manifoldGPU.copyFromHostPointer(manifoldPtrCpu,numManifolds);

	btAlignedObjectArray<unsigned int> offsetSplitBodies;
	unsigned int totalNumSplitBodies;

	btAlignedObjectArray<btGpuConstraint4> contactConstraints;
	contactConstraints.resize(numManifolds);


	btOpenCLArray<btRigidBodyCL> bodiesGPU(m_context,m_queue);
	bodiesGPU.resize(numBodies);

	btOpenCLArray<btInertiaCL> inertiasGPU(m_context,m_queue);
	inertiasGPU.resize(numBodies);


	if (useHost)
	{
		bodyCount.resize(numBodies);
		for (int i=0;i<numBodies;i++)
			bodyCount[i] = 0;

		contactConstraintOffsets.resize(numManifolds);


		for (int i=0;i<numManifolds;i++)
		{
			int pa = manifoldPtrCpu[i].m_bodyAPtrAndSignBit;
			int pb = manifoldPtrCpu[i].m_bodyBPtrAndSignBit;

			bool isFixedA = (pa <0) || (pa == solverInfo.m_fixedBodyIndex);
			bool isFixedB = (pb <0) || (pb == solverInfo.m_fixedBodyIndex);

			int bodyIndexA = manifoldPtrCpu[i].getBodyA();
			int bodyIndexB = manifoldPtrCpu[i].getBodyB();

			if (!isFixedA)
			{
				contactConstraintOffsets[i].x = bodyCount[bodyIndexA];
				bodyCount[bodyIndexA]++;
			}
			if (!isFixedB)
			{
				contactConstraintOffsets[i].y = bodyCount[bodyIndexB];
				bodyCount[bodyIndexB]++;
			} 
		}
		offsetSplitBodies.resize(numBodies);
		m_data->m_scan->executeHost(bodyCount,offsetSplitBodies,numBodies,&totalNumSplitBodies);
		int numlastBody = bodyCount[numBodies-1];
		totalNumSplitBodies += numlastBody;
		for (int i=0;i<numManifolds;i++)
		{
			ContactToConstraintKernel(&manifoldPtrCpu[0],bodiesCPU,inertiasCPU,&contactConstraints[0],numManifolds,
				solverInfo.m_deltaTime,
				solverInfo.m_positionDrift,
				solverInfo.m_positionConstraintCoeff,
				i, bodyCount);
		}

	} else
	{
//		int numBodies = bodies->size();
	//	int numManifolds = manifoldPtr->size();

		m_data->m_bodyCount->resize(numBodies);
	
		unsigned int val=0;
		btInt2 val2;
		val2.x=0;
		val2.y=0;

		 {
			BT_PROFILE("m_filler");
			m_data->m_contactConstraintOffsets->resize(numManifolds);
			m_data->m_filler->execute(*m_data->m_bodyCount,val,numBodies);
		
	
			m_data->m_filler->execute(*m_data->m_contactConstraintOffsets,val2,numManifolds);
		}

		{
			BT_PROFILE("m_countBodiesKernel");
			btLauncherCL launcher(this->m_queue,m_data->m_countBodiesKernel);
			launcher.setBuffer(manifoldGPU.getBufferCL());
			launcher.setBuffer(m_data->m_bodyCount->getBufferCL());
			launcher.setBuffer(m_data->m_contactConstraintOffsets->getBufferCL());
			launcher.setConst(numManifolds);
			launcher.setConst(solverInfo.m_fixedBodyIndex);
			launcher.launch1D(numManifolds);
		}
		m_data->m_contactConstraintOffsets->copyToHost(contactConstraintOffsets);
		m_data->m_bodyCount->copyToHost(bodyCount);

//		unsigned int totalNumSplitBodies=0;
		m_data->m_offsetSplitBodies->resize(numBodies);
		m_data->m_scan->execute(*m_data->m_bodyCount,*m_data->m_offsetSplitBodies,numBodies,&totalNumSplitBodies);
		totalNumSplitBodies+=m_data->m_bodyCount->at(numBodies-1);
		m_data->m_offsetSplitBodies->copyToHost(offsetSplitBodies);

		int numContacts = manifoldGPU.size();
		m_data->m_contactConstraints->resize(numContacts);

		bodiesGPU.copyFromHostPointer(bodiesCPU,numBodies);
		inertiasGPU.copyFromHostPointer(inertiasCPU,numBodies);
		{
			BT_PROFILE("contactToConstraintSplitKernel");
			btLauncherCL launcher( m_queue, m_data->m_contactToConstraintSplitKernel);
			launcher.setBuffer(manifoldGPU.getBufferCL());
			launcher.setBuffer(bodiesGPU.getBufferCL());
			launcher.setBuffer(inertiasGPU.getBufferCL());
			launcher.setBuffer(m_data->m_contactConstraints->getBufferCL());
			launcher.setBuffer(m_data->m_bodyCount->getBufferCL());
			launcher.setConst(numContacts);
			launcher.setConst(solverInfo.m_deltaTime);
			launcher.setConst(solverInfo.m_positionDrift);
			launcher.setConst(solverInfo.m_positionConstraintCoeff);
			launcher.launch1D( numContacts, 64 );
			clFinish(m_queue);
		}

		m_data->m_contactConstraints->copyToHost(contactConstraints);

	}



	




	int maxIter = 1;


	btAlignedObjectArray<btVector3> deltaLinearVelocities;
	btAlignedObjectArray<btVector3> deltaAngularVelocities;
	deltaLinearVelocities.resize(totalNumSplitBodies);
	deltaAngularVelocities.resize(totalNumSplitBodies);
	for (int i=0;i<totalNumSplitBodies;i++)
	{
		deltaLinearVelocities[i].setZero();
		deltaAngularVelocities[i].setZero();
	}

	m_data->m_deltaLinearVelocities->copyFromHost(deltaLinearVelocities);
	m_data->m_deltaAngularVelocities->copyFromHost(deltaAngularVelocities);


	for (int iter = 0;iter<maxIter;iter++)
	{

		bool solveHost=true;
		if (solveHost)
		{
			int i=0;
			for( i=0; i<numManifolds; i++)
			{

				float frictionCoeff = contactConstraints[i].getFrictionCoeff();
				int aIdx = (int)contactConstraints[i].m_bodyA;
				int bIdx = (int)contactConstraints[i].m_bodyB;
				btRigidBodyCL& bodyA = bodiesCPU[aIdx];
				btRigidBodyCL& bodyB = bodiesCPU[bIdx];

				btVector3 zero(0,0,0);
			
				btVector3* dlvAPtr=&zero;
				btVector3* davAPtr=&zero;
				btVector3* dlvBPtr=&zero;
				btVector3* davBPtr=&zero;
			
				if (bodyA.getInvMass())
				{
					int bodyOffsetA = offsetSplitBodies[aIdx];
					int constraintOffsetA = contactConstraintOffsets[i].x;
					int splitIndexA = bodyOffsetA+constraintOffsetA;
					dlvAPtr = &deltaLinearVelocities[splitIndexA];
					davAPtr = &deltaAngularVelocities[splitIndexA];
				}

				if (bodyB.getInvMass())
				{
					int bodyOffsetB = offsetSplitBodies[bIdx];
					int constraintOffsetB = contactConstraintOffsets[i].y;
					int splitIndexB= bodyOffsetB+constraintOffsetB;
					dlvBPtr =&deltaLinearVelocities[splitIndexB];
					davBPtr = &deltaAngularVelocities[splitIndexB];
				}



				{

					bool test=false;
					if (test)
					{
						float maxRambdaDt[4] = {FLT_MAX,FLT_MAX,FLT_MAX,FLT_MAX};
						float minRambdaDt[4] = {0.f,0.f,0.f,0.f};

						solveContact( contactConstraints[i], (btVector3&)bodyA.m_pos, (btVector3&)bodyA.m_linVel, (btVector3&)bodyA.m_angVel, bodyA.m_invMass, inertiasCPU[aIdx].m_invInertiaWorld, 
						(btVector3&)bodyB.m_pos, (btVector3&)bodyB.m_linVel, (btVector3&)bodyB.m_angVel, bodyB.m_invMass, inertiasCPU[bIdx].m_invInertiaWorld,
						maxRambdaDt, minRambdaDt , *dlvAPtr,*davAPtr,*dlvBPtr,*davBPtr		);
					} else
					{
						solveContact3(&contactConstraints[i], &bodyA.m_pos, &bodyA.m_linVel, &bodyA.m_angVel, bodyA.m_invMass, inertiasCPU[aIdx].m_invInertiaWorld, 
						&bodyB.m_pos, &bodyB.m_linVel, &bodyB.m_angVel, bodyB.m_invMass, inertiasCPU[bIdx].m_invInertiaWorld,
						  dlvAPtr,davAPtr,dlvBPtr,davBPtr		);

					}
				printf("!");

				}
			}
		}else
		{
			{

				//__kernel void SolveContactJacobiKernel(__global Constraint4* gConstraints, __global Body* gBodies, __global Shape* gShapes ,
				//__global int2* contactConstraintOffsets,__global int* offsetSplitBodies,__global float4* deltaLinearVelocities, __global float4* deltaAngularVelocities,
				//float deltaTime, float positionDrift, float positionConstraintCoeff, int fixedBodyIndex, int numManifolds


				BT_PROFILE("m_solveContactKernel");
				btLauncherCL launcher( m_queue, m_data->m_solveContactKernel );
				launcher.setBuffer(m_data->m_contactConstraints->getBufferCL());
				launcher.setBuffer(bodiesGPU.getBufferCL());
				launcher.setBuffer(inertiasGPU.getBufferCL());
				launcher.setBuffer(m_data->m_contactConstraintOffsets->getBufferCL());
				launcher.setBuffer(m_data->m_offsetSplitBodies->getBufferCL());
				launcher.setBuffer(m_data->m_deltaLinearVelocities->getBufferCL());
				launcher.setBuffer(m_data->m_deltaAngularVelocities->getBufferCL());
				launcher.setConst(solverInfo.m_deltaTime);
				launcher.setConst(solverInfo.m_positionDrift);
				launcher.setConst(solverInfo.m_positionConstraintCoeff);
				launcher.setConst(solverInfo.m_fixedBodyIndex);
				launcher.setConst(numManifolds);

				launcher.launch1D(numManifolds);
				clFinish(m_queue);
			}

			m_data->m_deltaLinearVelocities->copyToHost(deltaLinearVelocities);
			m_data->m_deltaAngularVelocities->copyToHost(deltaAngularVelocities);


		}

		bool useHostAverage=false;
		if (useHostAverage)
		{
			//easy
			for (int i=0;i<numBodies;i++)
			{
				if (bodiesCPU[i].getInvMass())
				{
					int bodyOffset = offsetSplitBodies[i];
					int count = bodyCount[i];
					float factor = 1.f/float(count);
					btVector3 averageLinVel;
					averageLinVel.setZero();
					btVector3 averageAngVel;
					averageAngVel.setZero();
					for (int j=0;j<count;j++)
					{
						averageLinVel += deltaLinearVelocities[bodyOffset+j]*factor;
						averageAngVel += deltaAngularVelocities[bodyOffset+j]*factor;
					}
					for (int j=0;j<count;j++)
					{
						deltaLinearVelocities[bodyOffset+j] = averageLinVel;
						deltaAngularVelocities[bodyOffset+j] = averageAngVel;
					}
				}
			}
		} else
		{

	//		bodiesGPU.copyFromHostPointer(bodiesCPU,numBodies);
			m_data->m_deltaLinearVelocities->copyFromHost(deltaLinearVelocities);
			m_data->m_deltaAngularVelocities->copyFromHost(deltaAngularVelocities);

			BT_PROFILE("average velocities");
			//__kernel void AverageVelocitiesKernel(__global Body* gBodies,__global int* offsetSplitBodies,__global const unsigned int* bodyCount,
			//__global float4* deltaLinearVelocities, __global float4* deltaAngularVelocities, int numBodies)
			btLauncherCL launcher( m_queue, m_data->m_averageVelocitiesKernel);
			launcher.setBuffer(bodiesGPU.getBufferCL());
			launcher.setBuffer(m_data->m_offsetSplitBodies->getBufferCL());
			launcher.setBuffer(m_data->m_bodyCount->getBufferCL());
			launcher.setBuffer(m_data->m_deltaLinearVelocities->getBufferCL());
			launcher.setBuffer(m_data->m_deltaAngularVelocities->getBufferCL());
			launcher.setConst(numBodies);
			launcher.launch1D(numBodies);
			clFinish(m_queue);
		
			m_data->m_deltaLinearVelocities->copyToHost(deltaLinearVelocities);
			m_data->m_deltaAngularVelocities->copyToHost(deltaAngularVelocities);

		}
		
#if 0

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

			btVector3 zero(0,0,0);
			
			btVector3* dlvAPtr=&zero;
			btVector3* davAPtr=&zero;
			btVector3* dlvBPtr=&zero;
			btVector3* davBPtr=&zero;
			
			if (bodyA.getInvMass())
			{
				int bodyOffsetA = offsetSplitBodies[aIdx];
				int constraintOffsetA = contactConstraintOffsets[i].x;
				int splitIndexA = bodyOffsetA+constraintOffsetA;
				dlvAPtr = &deltaLinearVelocities[splitIndexA];
				davAPtr = &deltaAngularVelocities[splitIndexA];
			}

			if (bodyB.getInvMass())
			{
				int bodyOffsetB = offsetSplitBodies[bIdx];
				int constraintOffsetB = contactConstraintOffsets[i].y;
				int splitIndexB= bodyOffsetB+constraintOffsetB;
				dlvBPtr =&deltaLinearVelocities[splitIndexB];
				davBPtr = &deltaAngularVelocities[splitIndexB];
			}

			for(int j=0; j<4; j++)
			{
				maxRambdaDt[j] = frictionCoeff*sum;
				minRambdaDt[j] = -maxRambdaDt[j];
			}

			solveFriction( contactConstraints[i], (btVector3&)bodyA.m_pos, (btVector3&)bodyA.m_linVel, (btVector3&)bodyA.m_angVel, bodyA.m_invMass,inertias[aIdx].m_invInertiaWorld, 
				(btVector3&)bodyB.m_pos, (btVector3&)bodyB.m_linVel, (btVector3&)bodyB.m_angVel, bodyB.m_invMass, inertias[bIdx].m_invInertiaWorld,
				maxRambdaDt, minRambdaDt , *dlvAPtr,*davAPtr,*dlvBPtr,*davBPtr);

		}

		//easy
		for (int i=0;i<numBodies;i++)
		{
			if (bodies[i].getInvMass())
			{
				int bodyOffset = offsetSplitBodies[i];
				int count = bodyCount[i];
				float factor = 1.f/float(count);
				btVector3 averageLinVel;
				averageLinVel.setZero();
				btVector3 averageAngVel;
				averageAngVel.setZero();
				for (int j=0;j<count;j++)
				{
					averageLinVel += deltaLinearVelocities[bodyOffset+j]*factor;
					averageAngVel += deltaAngularVelocities[bodyOffset+j]*factor;
				}
				for (int j=0;j<count;j++)
				{
					deltaLinearVelocities[bodyOffset+j] = averageLinVel;
					deltaAngularVelocities[bodyOffset+j] = averageAngVel;
				}
			}
		}

#endif


	}


	//easy
	for (int i=0;i<numBodies;i++)
	{
		if (bodiesCPU[i].getInvMass())
		{
			int bodyOffset = offsetSplitBodies[i];
			int count = bodyCount[i];
			if (count)
			{
				bodiesCPU[i].m_linVel += deltaLinearVelocities[bodyOffset];
				bodiesCPU[i].m_angVel += deltaAngularVelocities[bodyOffset];
			}
		}
	}
}


void  btGpuJacobiSolver::solveGroup(btOpenCLArray<btRigidBodyCL>* bodies,btOpenCLArray<btInertiaCL>* inertias,btOpenCLArray<btContact4>* manifoldPtr,const btJacobiSolverInfo& solverInfo)
{

	BT_PROFILE("btGpuJacobiSolver::solveGroup");

	int numBodies = bodies->size();
	int numManifolds = manifoldPtr->size();

	m_data->m_bodyCount->resize(numBodies);
	
	unsigned int val=0;
	btInt2 val2;
	val2.x=0;
	val2.y=0;

	 {
		BT_PROFILE("m_filler");
		m_data->m_contactConstraintOffsets->resize(numManifolds);
		m_data->m_filler->execute(*m_data->m_bodyCount,val,numBodies);
		
	
		m_data->m_filler->execute(*m_data->m_contactConstraintOffsets,val2,numManifolds);
	}

	{
		BT_PROFILE("m_countBodiesKernel");
		btLauncherCL launcher(this->m_queue,m_data->m_countBodiesKernel);
		launcher.setBuffer(manifoldPtr->getBufferCL());
		launcher.setBuffer(m_data->m_bodyCount->getBufferCL());
		launcher.setBuffer(m_data->m_contactConstraintOffsets->getBufferCL());
		launcher.setConst(numManifolds);
		launcher.setConst(solverInfo.m_fixedBodyIndex);
		launcher.launch1D(numManifolds);
	}

	unsigned int totalNumSplitBodies=0;
	m_data->m_offsetSplitBodies->resize(numBodies);
	m_data->m_scan->execute(*m_data->m_bodyCount,*m_data->m_offsetSplitBodies,numBodies,&totalNumSplitBodies);
	totalNumSplitBodies+=m_data->m_bodyCount->at(numBodies-1);


	int numContacts = manifoldPtr->size();
	m_data->m_contactConstraints->resize(numContacts);

	
	{
		BT_PROFILE("contactToConstraintSplitKernel");
		btLauncherCL launcher( m_queue, m_data->m_contactToConstraintSplitKernel);
		launcher.setBuffer(manifoldPtr->getBufferCL());
		launcher.setBuffer(bodies->getBufferCL());
		launcher.setBuffer(inertias->getBufferCL());
		launcher.setBuffer(m_data->m_contactConstraints->getBufferCL());
		launcher.setBuffer(m_data->m_bodyCount->getBufferCL());
        launcher.setConst(numContacts);
		launcher.setConst(solverInfo.m_deltaTime);
		launcher.setConst(solverInfo.m_positionDrift);
		launcher.setConst(solverInfo.m_positionConstraintCoeff);
		launcher.launch1D( numContacts, 64 );
		clFinish(m_queue);
	}

	
	m_data->m_deltaLinearVelocities->resize(totalNumSplitBodies);
	m_data->m_deltaAngularVelocities->resize(totalNumSplitBodies);


	
	{
		BT_PROFILE("m_clearVelocitiesKernel");
		btLauncherCL launch(m_queue,m_data->m_clearVelocitiesKernel);
		launch.setBuffer(m_data->m_deltaAngularVelocities->getBufferCL());
		launch.setBuffer(m_data->m_deltaLinearVelocities->getBufferCL());
		launch.setConst(totalNumSplitBodies);
		launch.launch1D(totalNumSplitBodies);
	}
	
	int maxIter = 14;

	for (int iter = 0;iter<maxIter;iter++)
	{
		{

			//__kernel void SolveContactJacobiKernel(__global Constraint4* gConstraints, __global Body* gBodies, __global Shape* gShapes ,
			//__global int2* contactConstraintOffsets,__global int* offsetSplitBodies,__global float4* deltaLinearVelocities, __global float4* deltaAngularVelocities,
			//float deltaTime, float positionDrift, float positionConstraintCoeff, int fixedBodyIndex, int numManifolds


			BT_PROFILE("m_solveContactKernel");
			btLauncherCL launcher( m_queue, m_data->m_solveContactKernel );
			launcher.setBuffer(m_data->m_contactConstraints->getBufferCL());
			launcher.setBuffer(bodies->getBufferCL());
			launcher.setBuffer(inertias->getBufferCL());
			launcher.setBuffer(m_data->m_contactConstraintOffsets->getBufferCL());
			launcher.setBuffer(m_data->m_offsetSplitBodies->getBufferCL());
			launcher.setBuffer(m_data->m_deltaLinearVelocities->getBufferCL());
			launcher.setBuffer(m_data->m_deltaAngularVelocities->getBufferCL());
			launcher.setConst(solverInfo.m_deltaTime);
			launcher.setConst(solverInfo.m_positionDrift);
			launcher.setConst(solverInfo.m_positionConstraintCoeff);
			launcher.setConst(solverInfo.m_fixedBodyIndex);
			launcher.setConst(numManifolds);

			launcher.launch1D(numManifolds);
			clFinish(m_queue);
		}

		{
			BT_PROFILE("average velocities");
			//__kernel void AverageVelocitiesKernel(__global Body* gBodies,__global int* offsetSplitBodies,__global const unsigned int* bodyCount,
			//__global float4* deltaLinearVelocities, __global float4* deltaAngularVelocities, int numBodies)
			btLauncherCL launcher( m_queue, m_data->m_averageVelocitiesKernel);
			launcher.setBuffer(bodies->getBufferCL());
			launcher.setBuffer(m_data->m_offsetSplitBodies->getBufferCL());
			launcher.setBuffer(m_data->m_bodyCount->getBufferCL());
			launcher.setBuffer(m_data->m_deltaLinearVelocities->getBufferCL());
			launcher.setBuffer(m_data->m_deltaAngularVelocities->getBufferCL());
			launcher.setConst(numBodies);
			launcher.launch1D(numBodies);
			clFinish(m_queue);
		}
		/*
		for(int i=0; i<numManifolds; i++)
		{

			float frictionCoeff = contactConstraints[i].getFrictionCoeff();
			int aIdx = (int)contactConstraints[i].m_bodyA;
			int bIdx = (int)contactConstraints[i].m_bodyB;
			btRigidBodyCL& bodyA = bodies[aIdx];
			btRigidBodyCL& bodyB = bodies[bIdx];

			btVector3 zero(0,0,0);
			
			btVector3* dlvAPtr=&zero;
			btVector3* davAPtr=&zero;
			btVector3* dlvBPtr=&zero;
			btVector3* davBPtr=&zero;
			
			if (bodyA.getInvMass())
			{
				int bodyOffsetA = offsetSplitBodies[aIdx];
				int constraintOffsetA = contactConstraintOffsets[i].x;
				int splitIndexA = bodyOffsetA+constraintOffsetA;
				dlvAPtr = &deltaLinearVelocities[splitIndexA];
				davAPtr = &deltaAngularVelocities[splitIndexA];
			}

			if (bodyB.getInvMass())
			{
				int bodyOffsetB = offsetSplitBodies[bIdx];
				int constraintOffsetB = contactConstraintOffsets[i].y;
				int splitIndexB= bodyOffsetB+constraintOffsetB;
				dlvBPtr =&deltaLinearVelocities[splitIndexB];
				davBPtr = &deltaAngularVelocities[splitIndexB];
			}



			{
				float maxRambdaDt[4] = {FLT_MAX,FLT_MAX,FLT_MAX,FLT_MAX};
				float minRambdaDt[4] = {0.f,0.f,0.f,0.f};

				solveContact( contactConstraints[i], (btVector3&)bodyA.m_pos, (btVector3&)bodyA.m_linVel, (btVector3&)bodyA.m_angVel, bodyA.m_invMass, inertias[aIdx].m_invInertiaWorld, 
					(btVector3&)bodyB.m_pos, (btVector3&)bodyB.m_linVel, (btVector3&)bodyB.m_angVel, bodyB.m_invMass, inertias[bIdx].m_invInertiaWorld,
					maxRambdaDt, minRambdaDt , *dlvAPtr,*davAPtr,*dlvBPtr,*davBPtr		);


			}
		}

		
		//easy
		for (int i=0;i<numBodies;i++)
		{
			if (bodies[i].getInvMass())
			{
				int bodyOffset = offsetSplitBodies[i];
				int count = bodyCount[i];
				float factor = 1.f/float(count);
				btVector3 averageLinVel;
				averageLinVel.setZero();
				btVector3 averageAngVel;
				averageAngVel.setZero();
				for (int j=0;j<count;j++)
				{
					averageLinVel += deltaLinearVelocities[bodyOffset+j]*factor;
					averageAngVel += deltaAngularVelocities[bodyOffset+j]*factor;
				}
				for (int j=0;j<count;j++)
				{
					deltaLinearVelocities[bodyOffset+j] = averageLinVel;
					deltaAngularVelocities[bodyOffset+j] = averageAngVel;
				}
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

			btVector3 zero(0,0,0);
			
			btVector3* dlvAPtr=&zero;
			btVector3* davAPtr=&zero;
			btVector3* dlvBPtr=&zero;
			btVector3* davBPtr=&zero;
			
			if (bodyA.getInvMass())
			{
				int bodyOffsetA = offsetSplitBodies[aIdx];
				int constraintOffsetA = contactConstraintOffsets[i].x;
				int splitIndexA = bodyOffsetA+constraintOffsetA;
				dlvAPtr = &deltaLinearVelocities[splitIndexA];
				davAPtr = &deltaAngularVelocities[splitIndexA];
			}

			if (bodyB.getInvMass())
			{
				int bodyOffsetB = offsetSplitBodies[bIdx];
				int constraintOffsetB = contactConstraintOffsets[i].y;
				int splitIndexB= bodyOffsetB+constraintOffsetB;
				dlvBPtr =&deltaLinearVelocities[splitIndexB];
				davBPtr = &deltaAngularVelocities[splitIndexB];
			}

			for(int j=0; j<4; j++)
			{
				maxRambdaDt[j] = frictionCoeff*sum;
				minRambdaDt[j] = -maxRambdaDt[j];
			}

			solveFriction( contactConstraints[i], (btVector3&)bodyA.m_pos, (btVector3&)bodyA.m_linVel, (btVector3&)bodyA.m_angVel, bodyA.m_invMass,inertias[aIdx].m_invInertiaWorld, 
				(btVector3&)bodyB.m_pos, (btVector3&)bodyB.m_linVel, (btVector3&)bodyB.m_angVel, bodyB.m_invMass, inertias[bIdx].m_invInertiaWorld,
				maxRambdaDt, minRambdaDt , *dlvAPtr,*davAPtr,*dlvBPtr,*davBPtr);

		}

		//easy
		for (int i=0;i<numBodies;i++)
		{
			if (bodies[i].getInvMass())
			{
				int bodyOffset = offsetSplitBodies[i];
				int count = bodyCount[i];
				float factor = 1.f/float(count);
				btVector3 averageLinVel;
				averageLinVel.setZero();
				btVector3 averageAngVel;
				averageAngVel.setZero();
				for (int j=0;j<count;j++)
				{
					averageLinVel += deltaLinearVelocities[bodyOffset+j]*factor;
					averageAngVel += deltaAngularVelocities[bodyOffset+j]*factor;
				}
				for (int j=0;j<count;j++)
				{
					deltaLinearVelocities[bodyOffset+j] = averageLinVel;
					deltaAngularVelocities[bodyOffset+j] = averageAngVel;
				}
			}
		}

		
		*/

	}

	btAlignedObjectArray<btVector3> dLinVel;
	btAlignedObjectArray<btVector3> dAngVel;
	m_data->m_deltaLinearVelocities->copyToHost(dLinVel);
	m_data->m_deltaAngularVelocities->copyToHost(dAngVel);

	btAlignedObjectArray<btRigidBodyCL> bodiesCPU;
	bodies->copyToHost(bodiesCPU);
	btAlignedObjectArray<unsigned int> bodyCountCPU;
	m_data->m_bodyCount->copyToHost(bodyCountCPU);
	btAlignedObjectArray<unsigned int> offsetSplitBodiesCPU;
	m_data->m_offsetSplitBodies->copyToHost(offsetSplitBodiesCPU);

	for (int i=0;i<numBodies;i++)
	{
		if (bodiesCPU[i].getInvMass())
		{
			int bodyOffset = offsetSplitBodiesCPU[i];
			int count = bodyCountCPU[i];
			if (count)
			{
				bodiesCPU[i].m_linVel += dLinVel[bodyOffset];
				bodiesCPU[i].m_angVel += dAngVel[bodyOffset];
			}
		}
	}
	bodies->copyFromHost(bodiesCPU);

	printf(".");

	/*
	//easy
	for (int i=0;i<numBodies;i++)
	{
		if (bodies[i].getInvMass())
		{
			int bodyOffset = offsetSplitBodies[i];
			int count = bodyCount[i];
			if (count)
			{
				bodies[i].m_linVel += deltaLinearVelocities[bodyOffset];
				bodies[i].m_angVel += deltaAngularVelocities[bodyOffset];
			}
		}
	}
	*/

}