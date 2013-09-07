
#include "PositionBasedDynamics.h"
#include "CpuSoftClothDemoInternalData.h"
#include "ExplicitEuler.h"

void PositionBasedDynamics::solveLinks(struct CpuSoftClothDemoInternalData* clothData, char* vertexPositions, int vertexStride,float dt)
{
	float kst = 1.f;
	float kLST = 1.f;

	
	for(int i=0; i<clothData->m_springs.size();i++)
	{			

		ClothSpring&	link=clothData->m_springs[i];

		//if(l.m_c0>0)
		{

			float invMassA = clothData->m_particleMasses[link.m_particleIndexA]? 1.f/clothData->m_particleMasses[link.m_particleIndexA] : 0.f;
			float invMassB = clothData->m_particleMasses[link.m_particleIndexB]? 1.f/clothData->m_particleMasses[link.m_particleIndexB] : 0.f;

			float			m_c0  = (invMassA+invMassB)*kLST;
			float			m_c1  = link.m_restLength*link.m_restLength;
						
			b3Vector3& posA = (b3Vector3&)vertexPositions[link.m_particleIndexA*vertexStride];
			b3Vector3& posB = (b3Vector3&)vertexPositions[link.m_particleIndexB*vertexStride];

			const b3Vector3	del=posB-posA;
			const float len=del.length2();
			if (m_c1+len > B3_EPSILON)
			{
				const float k=((m_c1-len)/(m_c0*(m_c1+len)))*kst;
				posA-=del*(k*invMassA);
				posB+=del*(k*invMassB);
			}
		}
	}

}

void PositionBasedDynamics::solveConstraints(struct CpuSoftClothDemoInternalData* clothData, char* vtx, int vertexStride,float dt)
{
	B3_PROFILE("computeGravityForces");
	ExplicitEuler::computeGravityForces(clothData,vtx,vertexStride,dt);
	ExplicitEuler::integrateExplicitEuler(clothData,vtx,vertexStride,dt);

	int numIter=10;
	for (int i=0;i<numIter;i++)
	{
		solveLinks(clothData,vtx,vertexStride,dt);
	}

};
