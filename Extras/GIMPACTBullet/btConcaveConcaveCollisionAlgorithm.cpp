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

/*
Author: Francisco León Nájera
Concave-Concave Collision

*/

#include "BulletCollision/CollisionDispatch/btManifoldResult.h"
#include "LinearMath/btIDebugDraw.h"
#include "btConcaveConcaveCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "btGIMPACTMeshShape.h"
#include "../Extras/GIMPACT/include/GIMPACT/gimpact.h"


btConcaveConcaveCollisionAlgorithm::btConcaveConcaveCollisionAlgorithm( const btCollisionAlgorithmConstructionInfo& ci, btCollisionObject* body0,btCollisionObject* body1)
: btCollisionAlgorithm(ci)
{
}

btConcaveConcaveCollisionAlgorithm::~btConcaveConcaveCollisionAlgorithm()
{
    clearCache();
}

void btConcaveConcaveCollisionAlgorithm::clearCache()
{
    btPersistentManifold* mainfold;
    for (size_t i=0;i<this->m_mainfoldsPtr.size() ; i++)
    {
    	mainfold = m_mainfoldsPtr[i];
    	m_dispatcher->releaseManifold(mainfold);
    }
	m_mainfoldsPtr.clear();
}

btPersistentManifold* btConcaveConcaveCollisionAlgorithm::newContactMainfold(btCollisionObject* body0,btCollisionObject* body1)
{
    btPersistentManifold* newmainfold;
    newmainfold = m_dispatcher->getNewManifold(body0,body1);
    m_mainfoldsPtr.push_back(newmainfold);
    return newmainfold;
}

void process_gimpact_contacts(GIM_CONTACT * pcontacts,
					  int contact_count,
					  btConcaveConcaveCollisionAlgorithm * algorithm,
					  btCollisionObject* body0,
					  btCollisionObject* body1,
					  btManifoldResult* resultOut)
{
		int i, ci = MANIFOLD_CACHE_SIZE;//Max point size
		btPersistentManifold * current_mainfold = 0;

		for(i=0;i<contact_count;i++)
		{
			if(ci>=MANIFOLD_CACHE_SIZE)
			{
				current_mainfold = algorithm->newContactMainfold(body0,body1);
				resultOut->setPersistentManifold(current_mainfold);
				ci=0;
			}
			
			btVector3 cpoint(pcontacts->m_point[0],pcontacts->m_point[1],pcontacts->m_point[2]);			
			//Normal points to body0
			btVector3 cnormal(pcontacts->m_normal[0],pcontacts->m_normal[1],pcontacts->m_normal[2]);			
			
			resultOut->addContactPoint(cnormal,cpoint,-pcontacts->m_depth);

			pcontacts++;
			ci++;
		}

}

void btConcaveConcaveCollisionAlgorithm::processCollision (btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)
{
    clearCache();
	if (body0->getCollisionShape()->getShapeType()==GIMPACT_SHAPE_PROXYTYPE && body1->getCollisionShape()->getShapeType()==GIMPACT_SHAPE_PROXYTYPE )
	{
	    btGIMPACTMeshShape*	tri0b = static_cast<btGIMPACTMeshShape*>( body0->getCollisionShape());
	    btGIMPACTMeshShape*	tri1b = static_cast<btGIMPACTMeshShape*>( body1->getCollisionShape());

		tri0b->prepareMeshes(body0->getWorldTransform());
		tri1b->prepareMeshes(body1->getWorldTransform());

		size_t i,j;
		size_t parts0 = tri0b->m_gim_trimesh_parts.size();
		size_t parts1 = tri1b->m_gim_trimesh_parts.size();

		GIM_TRIMESH * trimesh0;
		GIM_TRIMESH * trimesh1;
		GDYNAMIC_ARRAY contacts;
		GIM_CONTACT * pcontacts;
		for(i=0;i<parts0;i++)
		{
			for(j=0;j<parts1;j++)
			{
				trimesh0 = (GIM_TRIMESH * )tri0b->m_gim_trimesh_parts[i];
				trimesh1 = (GIM_TRIMESH * )tri1b->m_gim_trimesh_parts[j];
				
				GIM_CREATE_CONTACT_LIST(contacts);

				gim_trimesh_trimesh_collision(trimesh0,trimesh1,&contacts);

				if(contacts.m_size>0)
				{
					pcontacts = GIM_DYNARRAY_POINTER(GIM_CONTACT,contacts);
					process_gimpact_contacts(pcontacts,contacts.m_size,this,body0,body1,resultOut);
				}
				GIM_DYNARRAY_DESTROY(contacts);		
			}
		}
	}
}


float btConcaveConcaveCollisionAlgorithm::calculateTimeOfImpact(btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)
{
	return 1.f;

}
