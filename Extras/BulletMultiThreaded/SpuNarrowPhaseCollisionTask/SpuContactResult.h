#ifndef SPU_CONTACT_RESULT2_H
#define SPU_CONTACT_RESULT2_H


#ifndef WIN32
#include <stdint.h>
#endif



#ifdef WIN32
#include "SpuDoubleBuffer.h"
#else
#include "SPU_Common/SpuDefines.h"
#include "SPU_Common/SpuDoubleBuffer.h"
#include <spu_printf.h>
#endif //WIN32


#include "LinearMath/btTransform.h"
#include "LinearMath/btPoint3.h"


#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"


struct SpuCollisionPairInput
{
	uint64_t m_collisionShapes[2];
	void*	m_spuCollisionShapes[2];

	uint64_t m_persistentManifoldPtr;
	btVector3	m_primitiveDimensions0;
	btVector3	m_primitiveDimensions1;
	int		m_shapeType0;
	int		m_shapeType1;	
	float	m_collisionMargin0;
	float	m_collisionMargin1;

	btTransform	m_worldTransform0;
	btTransform m_worldTransform1;
	
	bool	m_isSwapped;


	
};


struct SpuClosestPointInput
{
    SpuClosestPointInput()
        :m_maximumDistanceSquared(float(1e30)),
        m_stackAlloc(0)
    {
    }

    btTransform m_transformA;
    btTransform m_transformB;
    float	m_maximumDistanceSquared;
    class	btStackAlloc* m_stackAlloc;
	struct SpuConvexPolyhedronVertexData* m_convexVertexData;
};

///SpuContactResult exports the contact points using double-buffered DMA transfers, only when needed
///So when an existing contact point is duplicated, no transfer/refresh is performed.
class SpuContactResult
{
    btTransform		m_rootWorldTransform0;
	btTransform		m_rootWorldTransform1;
	uint64_t	m_manifoldAddress;

    btPersistentManifold* m_spuManifold;
	bool m_RequiresWriteBack;

	DoubleBuffer<btPersistentManifold, 1> g_manifoldDmaExport;

	public:
		SpuContactResult();
		virtual ~SpuContactResult();

		btPersistentManifold*	GetSpuManifold() const
		{
			return m_spuManifold;
		}

		virtual void setShapeIdentifiers(int partId0,int index0,	int partId1,int index1);

        void	setContactInfo(btPersistentManifold* spuManifold, uint64_t	manifoldAddress,const btTransform& worldTrans0,const btTransform& worldTrans1);

        void writeDoubleBufferedManifold(btPersistentManifold* lsManifold, btPersistentManifold* mmManifold);

        virtual void addContactPoint(const btVector3& normalOnBInWorld,const btPoint3& pointInWorld,float depth);

		void flush();
};



#endif //SPU_CONTACT_RESULT2_H

