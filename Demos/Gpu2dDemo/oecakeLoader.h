
#ifndef OE_CAKE_LOADER_H
#define OE_CAKE_LOADER_H

#include "btBulletDynamicsCommon.h"


class BasicOECakeReader
{
	int	m_materialType;
	int	m_particleObjectIndex;
	int	m_particleColor;
	btAlignedObjectArray<btVector3>	m_particlePositions;
	btAlignedObjectArray<btScalar>	m_particleRadii;

	void addParticle(int materialType, int pIndex, int pColor, float pPosX, float pPosY, float radius=1);

	virtual	void	addNewCollisionShape(int numParticles, btVector3* particlePositions, btScalar* radii, int materialType, int objectIndex,int color );

	int processLine(char * buffer, int size);

	void	convertParticleGroup();

public:

	BasicOECakeReader()
	{
	}

	bool processFile(char * fileName);

	virtual void createBodyForCompoundShape(btCompoundShape* compound,bool addConstraint,const btTransform& worldTransform, btScalar mass) = 0;	
	
};
#endif //OE_CAKE_LOADER_H
