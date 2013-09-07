#include "ExplicitEuler.h"
	
#include "CpuSoftClothDemoInternalData.h"





void ExplicitEuler::computeGravityForces(struct CpuSoftClothDemoInternalData* clothData, char* vertexPositions, int vertexStride, float dt)
{
	
	B3_PROFILE("computeForces");
	int numPoints = clothData->m_particleMasses.size();

	b3Vector3 gravityAcceleration = b3MakeVector3(0,-9.8,0);
	//f=m*a
	for (int i=0;i<numPoints;i++)
	{
		{
			float	particleMass = clothData->m_particleMasses[i];

			b3Vector3 particleMassVec = b3MakeVector3(particleMass,particleMass,particleMass,0);
			clothData->m_forces[i] = gravityAcceleration*particleMass;
		}
	}

}
	
void ExplicitEuler::computeSpringForces(struct CpuSoftClothDemoInternalData* clothData, char* vertexPositions, int vertexStride, float dt)
{
	
	//add spring forces
	for(int i=0;i<clothData->m_springs.size();i++) 
	{

		int indexA = clothData->m_springs[i].m_particleIndexA;
		int indexB = clothData->m_springs[i].m_particleIndexB;
		float restLength = clothData->m_springs[i].m_restLength;
		const ClothMaterial& mat = clothData->m_materials[clothData->m_springs[i].m_material];

		const b3Vector3& posA = (const b3Vector3&)vertexPositions[indexA*vertexStride];
		const b3Vector3& posB = (const b3Vector3&)vertexPositions[indexB*vertexStride];
		const b3Vector3& velA = clothData->m_velocities[indexA];
		const b3Vector3& velB = clothData->m_velocities[indexB];

		b3Vector3 deltaP = posA-posB;
		b3Vector3 deltaV = velA-velB;
		float dist = deltaP.length();
		b3Vector3 deltaPNormalized = deltaP/dist;

		float spring = -mat.m_stiffness * (dist-restLength)*100000;
		float damper = mat.m_damping * b3Dot(deltaV,deltaPNormalized)*100;

		b3Vector3 springForce = (spring+damper)*deltaPNormalized;
		float particleMassA = clothData->m_particleMasses[indexA];
		float particleMassB = clothData->m_particleMasses[indexB];

		//if (springForce.length())
		{
			if (particleMassA)
			{
				clothData->m_forces[indexA] += springForce*particleMassA;
			}

			if (particleMassB)
			{
				clothData->m_forces[indexB] -= springForce*particleMassB;
			}
		}
	}
}
void ExplicitEuler::integrateExplicitEuler(struct CpuSoftClothDemoInternalData* clothData, char* vertexPositions, int vertexStride,float deltaTime)
{
	B3_PROFILE("integrateEuler");
	b3Vector3 deltaTimeVec = b3MakeVector3(deltaTime,deltaTime,deltaTime,0);

	int numPoints = clothData->m_particleMasses.size();
	
	for (int i=0;i<numPoints;i++)
	{
		float mass = clothData->m_particleMasses[i];
		if (mass)
		{
			


			b3Vector3 dv = (clothData->m_forces[i]/mass)*deltaTimeVec;
			clothData->m_velocities[i]+= dv;
			clothData->m_velocities[i]*=0.999;

			b3Vector3& pos = (b3Vector3&)vertexPositions[i*vertexStride];
			
			pos += clothData->m_velocities[i]*deltaTimeVec;
		}
	}
	
}

void ExplicitEuler::solveConstraints(struct CpuSoftClothDemoInternalData* data, char* vertexPositions, int vertexStride,float deltaTime)
{
	computeGravityForces(data,vertexPositions,vertexStride,deltaTime);
	computeSpringForces(data,vertexPositions,vertexStride,deltaTime);

	integrateExplicitEuler(data,vertexPositions,vertexStride,deltaTime);
}