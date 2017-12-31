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
	bool result = false;
	btAlignedObjectArray<btQuaternion> scratchQ;
	btAlignedObjectArray<btVector3> scratchM;

	if (m_importerFlags&eRESTORE_EXISTING_OBJECTS)
	{
		//check if the snapshot is valid for the existing world
		//equal number of objects, # links etc
		if (bulletFile2->m_multiBodies.size() != m_data->m_mbDynamicsWorld->getNumMultibodies())
		{
			result = false;
			return result;
		}
		result = true;

		//convert all multibodies
		if (bulletFile2->getFlags() & bParse::FD_DOUBLE_PRECISION)
		{

			//for (int i = 0; i < bulletFile2->m_multiBodies.size(); i++)
			for (int i = bulletFile2->m_multiBodies.size()-1; i >=0; i--)
			{
				btMultiBodyDoubleData* mbd = (btMultiBodyDoubleData*)bulletFile2->m_multiBodies[i];
				bool isFixedBase = mbd->m_baseMass == 0;
				bool canSleep = false;
				btVector3 baseInertia;
				baseInertia.deSerializeDouble(mbd->m_baseInertia);

				btMultiBody* mb = m_data->m_mbDynamicsWorld->getMultiBody(i);
				btVector3 baseWorldPos;
				baseWorldPos.deSerializeDouble(mbd->m_baseWorldPosition);
				mb->setBasePos(baseWorldPos);
				btQuaternion baseWorldRot;
				baseWorldRot.deSerializeDouble(mbd->m_baseWorldOrientation);
				mb->setWorldToBaseRot(baseWorldRot.inverse());
				btVector3 baseLinVal;
				baseLinVal.deSerializeDouble(mbd->m_baseLinearVelocity);
				btVector3 baseAngVel;
				baseAngVel.deSerializeDouble(mbd->m_baseAngularVelocity);
				mb->setBaseVel(baseLinVal);
				mb->setBaseOmega(baseAngVel);

				for (int i = 0; i < mbd->m_numLinks; i++)
				{
					switch (mbd->m_links[i].m_jointType)
					{
					case btMultibodyLink::eFixed:
					{
						break;
					}
					case btMultibodyLink::ePrismatic:
					{
						mb->setJointPos(i, mbd->m_links[i].m_jointPos[0]);
						mb->setJointVel(i, mbd->m_links[i].m_jointVel[0]);
						break;
					}
					case btMultibodyLink::eRevolute:
					{
						mb->setJointPos(i, mbd->m_links[i].m_jointPos[0]);
						mb->setJointVel(i, mbd->m_links[i].m_jointVel[0]);
						break;
					}
					case btMultibodyLink::eSpherical:
					{
						btScalar jointPos[3] = { mbd->m_links[i].m_jointPos[0], mbd->m_links[i].m_jointPos[1], mbd->m_links[i].m_jointPos[2] };
						btScalar jointVel[3] = { mbd->m_links[i].m_jointVel[0], mbd->m_links[i].m_jointVel[1], mbd->m_links[i].m_jointVel[2] };
						mb->setJointPosMultiDof(i, jointPos);
						mb->setJointVelMultiDof(i, jointVel);

						break;
					}
					case btMultibodyLink::ePlanar:
					{
						break;
					}
					default:
					{
					}
					}
				}
				mb->forwardKinematics(scratchQ, scratchM);
				mb->updateCollisionObjectWorldTransforms(scratchQ, scratchM);
			}

			//todo: check why body1 pointer is not properly deserialized
			for (int i = 0; i < bulletFile2->m_contactManifolds.size(); i++)
			{
				btPersistentManifoldDoubleData* manifoldData = (btPersistentManifoldDoubleData*)bulletFile2->m_contactManifolds[i];
				void* ptr = bulletFile2->findLibPointer(manifoldData->m_body1);
				if (ptr)
				{
					manifoldData->m_body1 = ptr;
					btCollisionObjectDoubleData* cdd = (btCollisionObjectDoubleData*)ptr;
				}
			}

			btDispatcherInfo& dispatchInfo = m_data->m_mbDynamicsWorld->getDispatchInfo();

			{
				m_data->m_mbDynamicsWorld->updateAabbs();
				m_data->m_mbDynamicsWorld->computeOverlappingPairs();
				btDispatcher* dispatcher = m_data->m_mbDynamicsWorld->getDispatcher();
				
				if (dispatcher)
				{
					btOverlappingPairCache* pairCache = m_data->m_mbDynamicsWorld->getBroadphase()->getOverlappingPairCache();
					if (dispatcher)
					{
						dispatcher->dispatchAllCollisionPairs(pairCache, dispatchInfo, dispatcher);
					}
					int numExistingManifolds = m_data->m_mbDynamicsWorld->getDispatcher()->getNumManifolds();
					btManifoldArray manifoldArray;
					for (int i = 0; i < pairCache->getNumOverlappingPairs(); i++)
					{
						btBroadphasePair& pair = pairCache->getOverlappingPairArray()[i];
						if (pair.m_algorithm)
						{
							pair.m_algorithm->getAllContactManifolds(manifoldArray);
							//for each existing manifold, search a matching manifoldData and reconstruct
							for (int m = 0; m < manifoldArray.size(); m++)
							{
								btPersistentManifold* existingManifold = manifoldArray[m];
								int uid0 = existingManifold->getBody0()->getBroadphaseHandle()->m_uniqueId;
								int uid1 = existingManifold->getBody1()->getBroadphaseHandle()->m_uniqueId;
								int matchingManifoldIndex = -1;
								for (int q= 0; q < bulletFile2->m_contactManifolds.size(); q++)
								{
									btPersistentManifoldDoubleData* manifoldData = (btPersistentManifoldDoubleData*)bulletFile2->m_contactManifolds[q];
									btCollisionObjectDoubleData* cdd0 = (btCollisionObjectDoubleData*)manifoldData->m_body0;
									btCollisionObjectDoubleData* cdd1 = (btCollisionObjectDoubleData*)manifoldData->m_body1;
									if (uid0 == cdd0->m_uniqueId && uid1 == cdd1->m_uniqueId)
									{
										matchingManifoldIndex = q;
									}
								}
								if (matchingManifoldIndex >= 0)
								{
									btPersistentManifoldDoubleData* manifoldData = (btPersistentManifoldDoubleData*)bulletFile2->m_contactManifolds[matchingManifoldIndex];
									existingManifold->deSerializeDouble(manifoldData);
								}
								else
								{
									existingManifold->setNumContacts(0);
									//printf("Issue: cannot find maching contact manifold (%d, %d), may cause issues in determinism.\n", uid0, uid1);
								}

								manifoldArray.clear();
							}
						}
					}
				}
			}
		}
	}
	else
	{
		result = btBulletWorldImporter::convertAllObjects(bulletFile2);


		//convert all multibodies
		for (int i = 0; i < bulletFile2->m_multiBodies.size(); i++)
		{

			if (bulletFile2->getFlags() & bParse::FD_DOUBLE_PRECISION)
			{
				btMultiBodyDoubleData* mbd = (btMultiBodyDoubleData*)bulletFile2->m_multiBodies[i];
				bool isFixedBase = mbd->m_baseMass == 0;
				bool canSleep = false;
				btVector3 baseInertia;
				baseInertia.deSerializeDouble(mbd->m_baseInertia);
				btMultiBody* mb = new btMultiBody(mbd->m_numLinks, mbd->m_baseMass, baseInertia, isFixedBase, canSleep);
				mb->setHasSelfCollision(false);
				
				btVector3 baseWorldPos;
				baseWorldPos.deSerializeDouble(mbd->m_baseWorldPosition);

				btQuaternion baseWorldOrn;
				baseWorldOrn.deSerializeDouble(mbd->m_baseWorldOrientation);
				mb->setBasePos(baseWorldPos);
				mb->setWorldToBaseRot(baseWorldOrn.inverse());

				m_data->m_mbMap.insert(mbd, mb);
				for (int i = 0; i < mbd->m_numLinks; i++)
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
							parentRotToThis, parentComToThisPivotOffset, thisPivotToThisComOffset);
						//search for the collider
						//mbd->m_links[i].m_linkCollider
						break;
					}
					case btMultibodyLink::ePrismatic:
					{
						btVector3 jointAxis;
						jointAxis.deSerializeDouble(mbd->m_links[i].m_jointAxisBottom[0]);
						bool disableParentCollision = true;//todo
						mb->setupPrismatic(i, mbd->m_links[i].m_linkMass, localInertiaDiagonal, mbd->m_links[i].m_parentIndex,
							parentRotToThis, jointAxis, parentComToThisPivotOffset, thisPivotToThisComOffset, disableParentCollision);
						mb->setJointPos(i, mbd->m_links[i].m_jointPos[0]);
						mb->setJointVel(i, mbd->m_links[i].m_jointVel[0]);
						break;
					}
					case btMultibodyLink::eRevolute:
					{
						btVector3 jointAxis;
						jointAxis.deSerializeDouble(mbd->m_links[i].m_jointAxisTop[0]);
						bool disableParentCollision = true;//todo
						mb->setupRevolute(i, mbd->m_links[i].m_linkMass, localInertiaDiagonal, mbd->m_links[i].m_parentIndex,
							parentRotToThis, jointAxis, parentComToThisPivotOffset, thisPivotToThisComOffset, disableParentCollision);
						mb->setJointPos(i, mbd->m_links[i].m_jointPos[0]);
						mb->setJointVel(i, mbd->m_links[i].m_jointVel[0]);
						break;
					}
					case btMultibodyLink::eSpherical:
					{
						btAssert(0);
						bool disableParentCollision = true;//todo
						mb->setupSpherical(i, mbd->m_links[i].m_linkMass, localInertiaDiagonal, mbd->m_links[i].m_parentIndex,
							parentRotToThis, parentComToThisPivotOffset, thisPivotToThisComOffset, disableParentCollision);
						btScalar jointPos[3] = { mbd->m_links[i].m_jointPos[0], mbd->m_links[i].m_jointPos[1], mbd->m_links[i].m_jointPos[2] };
						btScalar jointVel[3] = { mbd->m_links[i].m_jointVel[0], mbd->m_links[i].m_jointVel[1], mbd->m_links[i].m_jointVel[2] };
						mb->setJointPosMultiDof(i, jointPos);
						mb->setJointVelMultiDof(i, jointVel);

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

		//forward kinematics, so that the link world transforms are valid, for collision detection
		for (int i = 0; i < m_data->m_mbMap.size(); i++)
		{
			btMultiBody**ptr = m_data->m_mbMap.getAtIndex(i);
			if (ptr)
			{
				btMultiBody* mb = *ptr;
				mb->finalizeMultiDof();
				btVector3 linvel = mb->getBaseVel();
				btVector3 angvel = mb->getBaseOmega();
				mb->forwardKinematics(scratchQ, scratchM);
			}
		}

		//convert all multibody link colliders
		for (int i = 0; i < bulletFile2->m_multiBodyLinkColliders.size(); i++)
		{
			if (bulletFile2->getFlags() & bParse::FD_DOUBLE_PRECISION)
			{
				btMultiBodyLinkColliderDoubleData* mblcd = (btMultiBodyLinkColliderDoubleData*)bulletFile2->m_multiBodyLinkColliders[i];

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
							col->setCollisionShape(shape);
							//btCollisionObject* body = createCollisionObject(startTransform,shape,mblcd->m_colObjData.m_name);
							col->setFriction(btScalar(mblcd->m_colObjData.m_friction));
							col->setRestitution(btScalar(mblcd->m_colObjData.m_restitution));
							//m_bodyMap.insert(colObjData,body);
							if (mblcd->m_link == -1)
							{
								col->setWorldTransform(multiBody->getBaseWorldTransform());
								multiBody->setBaseCollider(col);
							}
							else
							{
								col->setWorldTransform(multiBody->getLink(mblcd->m_link).m_cachedWorldTransform);
								multiBody->getLink(mblcd->m_link).m_collider = col;
							}
							int mbLinkIndex = mblcd->m_link;

							bool isDynamic = (mbLinkIndex < 0 && multiBody->hasFixedBase()) ? false : true;
							int collisionFilterGroup = isDynamic ? int(btBroadphaseProxy::DefaultFilter) : int(btBroadphaseProxy::StaticFilter);
							int collisionFilterMask = isDynamic ? int(btBroadphaseProxy::AllFilter) : int(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);

#if 0
							int colGroup = 0, colMask = 0;
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
							m_data->m_mbDynamicsWorld->addCollisionObject(col, collisionFilterGroup, collisionFilterMask);
						}

					}
					else
					{
						printf("error: no shape found\n");
					}
#if 0
					//base and fixed? -> static, otherwise flag as dynamic

					world1->addCollisionObject(col, collisionFilterGroup, collisionFilterMask);
#endif
				}

			}
		}

		for (int i = 0; i < m_data->m_mbMap.size(); i++)
		{
			btMultiBody**ptr = m_data->m_mbMap.getAtIndex(i);
			if (ptr)
			{
				btMultiBody* mb = *ptr;
				mb->finalizeMultiDof();

				m_data->m_mbDynamicsWorld->addMultiBody(mb);
			}
		}
	}
	return result;
}