#include "btMultiBodyWorldImporter.h"

#include "LinearMath/btSerializer.h"
#include "../BulletFileLoader/btBulletFile.h"
#include "btBulletWorldImporter.h"
#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"

struct btMultiBodyWorldImporterInternalData
{
	btMultiBodyDynamicsWorld* m_mbDynamicsWorld;
	btHashMap<btHashPtr, btMultiBody*> m_mbMap;
};

btMultiBodyWorldImporter::btMultiBodyWorldImporter(btMultiBodyDynamicsWorld* world)
	:btBulletWorldImporter(world)
{
	m_data = new btMultiBodyWorldImporterInternalData;
	m_data->m_mbDynamicsWorld = world;
}
	

btMultiBodyWorldImporter::~btMultiBodyWorldImporter()
{
	delete m_data;
}

void btMultiBodyWorldImporter::deleteAllData()
{
	btBulletWorldImporter::deleteAllData();
}



bool btMultiBodyWorldImporter::convertAllObjects(  bParse::btBulletFile* bulletFile2)
{
	bool result = btBulletWorldImporter::convertAllObjects(bulletFile2);
	
	
	//convert all multibodies
	for (int i=0;i<bulletFile2->m_multiBodies.size();i++)
	{
		
		if (bulletFile2->getFlags() & bParse::FD_DOUBLE_PRECISION)
		{
			btMultiBodyDoubleData* mbd = (btMultiBodyDoubleData*) bulletFile2->m_multiBodies[i];
			bool isFixedBase = mbd->m_baseMass==0;
			bool canSleep = false;
			btVector3 baseInertia;
			baseInertia.deSerializeDouble(mbd->m_baseInertia);
			btMultiBody* mb = new btMultiBody(mbd->m_numLinks,mbd->m_baseMass,baseInertia,isFixedBase,canSleep);
			mb->setHasSelfCollision(false);

			m_data->m_mbMap.insert(mbd,mb);
			for (int i=0;i<mbd->m_numLinks;i++)
			{
				btVector3 localInertiaDiagonal;
				localInertiaDiagonal.deSerializeDouble(mbd->m_links[i].m_linkInertia);
				btQuaternion parentRotToThis;
				parentRotToThis.deSerializeDouble(mbd->m_links[i].m_zeroRotParentToThis);
				btVector3 parentComToThisPivotOffset;
				parentComToThisPivotOffset.deSerializeDouble(mbd->m_links[i].m_parentComToThisComOffset);
				btVector3 thisPivotToThisComOffset;
				thisPivotToThisComOffset.deSerializeDouble(mbd->m_links[i].m_thisPivotToThisComOffset);

				switch (mbd->m_links[i].m_jointType)
				{
					case btMultibodyLink::eFixed:
					{
						
						
						mb->setupFixed(i, mbd->m_links[i].m_linkMass, localInertiaDiagonal, mbd->m_links[i].m_parentIndex,
                                                            parentRotToThis, parentComToThisPivotOffset,thisPivotToThisComOffset);
						//search for the collider
						//mbd->m_links[i].m_linkCollider
						break;
					}
					case btMultibodyLink::ePrismatic:
					{
						btVector3 jointAxis;
						jointAxis.deSerializeDouble(mbd->m_links[i].m_jointAxisBottom[0]);
						bool disableParentCollision = true;//todo
						mb->setupPrismatic(i,mbd->m_links[i].m_linkMass,localInertiaDiagonal,mbd->m_links[i].m_parentIndex,
							parentRotToThis,jointAxis, parentComToThisPivotOffset,thisPivotToThisComOffset,disableParentCollision);
						break;
					}
					case btMultibodyLink::eRevolute:
					{
						btVector3 jointAxis;
						jointAxis.deSerializeDouble(mbd->m_links[i].m_jointAxisTop[0]);
						bool disableParentCollision = true;//todo
						mb->setupRevolute(i,mbd->m_links[i].m_linkMass,localInertiaDiagonal,mbd->m_links[i].m_parentIndex,
							parentRotToThis, jointAxis,parentComToThisPivotOffset,thisPivotToThisComOffset,disableParentCollision);
						break;
					}
					case btMultibodyLink::eSpherical:
					{
						btAssert(0);
						bool disableParentCollision = true;//todo
						mb->setupSpherical(i,mbd->m_links[i].m_linkMass,localInertiaDiagonal,mbd->m_links[i].m_parentIndex,
							parentRotToThis,parentComToThisPivotOffset,thisPivotToThisComOffset,disableParentCollision);
						break;
					}
					case btMultibodyLink::ePlanar:
					{
						btAssert(0);
						break;
					}
					default:
					{
						btAssert(0);
					}
				}
			}
		}
	}
	
	//convert all multibody link colliders
	for (int i=0;i<bulletFile2->m_multiBodyLinkColliders.size();i++)
	{
		if (bulletFile2->getFlags() & bParse::FD_DOUBLE_PRECISION)
		{
			btMultiBodyLinkColliderDoubleData* mblcd = (btMultiBodyLinkColliderDoubleData*) bulletFile2->m_multiBodyLinkColliders[i];

			btMultiBody** ptr = m_data->m_mbMap[mblcd->m_multiBody];
			if (ptr)
			{
				btMultiBody* multiBody = *ptr;
				

				btCollisionShape** shapePtr = m_shapeMap.find(mblcd->m_colObjData.m_collisionShape);
				if (shapePtr && *shapePtr)
				{
					btTransform startTransform;
					mblcd->m_colObjData.m_worldTransform.m_origin.m_floats[3] = 0.f;
					startTransform.deSerializeDouble(mblcd->m_colObjData.m_worldTransform);
				
					btCollisionShape* shape = (btCollisionShape*)*shapePtr;
					if (shape)
					{
						btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(multiBody, mblcd->m_link);
						col->setCollisionShape( shape );

						//btCollisionObject* body = createCollisionObject(startTransform,shape,mblcd->m_colObjData.m_name);
						col->setFriction(btScalar(mblcd->m_colObjData.m_friction));
						col->setRestitution(btScalar(mblcd->m_colObjData.m_restitution));
						//m_bodyMap.insert(colObjData,body);
						if (mblcd->m_link==-1)
						{
							multiBody->setBaseCollider(col);
						} else
						{
							multiBody->getLink(mblcd->m_link).m_collider = col;
						}
						int mbLinkIndex = mblcd->m_link;

						bool isDynamic = (mbLinkIndex<0 && multiBody->hasFixedBase())? false : true;
						int collisionFilterGroup = isDynamic? int(btBroadphaseProxy::DefaultFilter) : int(btBroadphaseProxy::StaticFilter);
						int collisionFilterMask = isDynamic? 	int(btBroadphaseProxy::AllFilter) : 	int(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);

	#if 0
						int colGroup=0, colMask=0;
						int collisionFlags = mblcd->m_colObjData.m_collisionFlags;
						if (collisionFlags & URDF_HAS_COLLISION_GROUP)
						{
							collisionFilterGroup = colGroup;
						}
						if (collisionFlags & URDF_HAS_COLLISION_MASK)
						{
							collisionFilterMask = colMask;
						}
	#endif
						m_data->m_mbDynamicsWorld->addCollisionObject(col,collisionFilterGroup,collisionFilterMask);
					}
					
				} else
				{
					printf("error: no shape found\n");
				}
#if 0
				//base and fixed? -> static, otherwise flag as dynamic
				
				world1->addCollisionObject(col,collisionFilterGroup,collisionFilterMask);
#endif
			}

		}
	}

	for (int i=0;i<m_data->m_mbMap.size();i++)
	{
		btMultiBody**ptr = m_data->m_mbMap.getAtIndex(i);
		if (ptr)
		{
			btMultiBody* mb = *ptr;
			mb->finalizeMultiDof();
			
			m_data->m_mbDynamicsWorld->addMultiBody(mb);
		}
	}
	return result;
}