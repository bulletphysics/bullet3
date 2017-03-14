/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2017 Erik Ogenvik <erik@ogenvik.org>
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include <array>
#include <limits>
#include "TerrainExample.h"

#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"


struct TerrainExample : public CommonRigidBodyBase
{
	TerrainExample(struct GUIHelperInterface* helper)
		:CommonRigidBodyBase(helper)
	{
	}
	virtual ~TerrainExample(){}
	virtual void initPhysics();
	void resetCamera()
	{
		float dist = 15;
		float pitch = 52;
		float yaw = 35;
		float targetPos[3]={0,0,0};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}
};

constexpr int pointsPerSide = 65;
std::array<float, pointsPerSide*pointsPerSide> points;

void TerrainExample::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawContactPoints);



	float height = 0.0f;
    float min = std::numeric_limits<float>::max();
    float max = std::numeric_limits<float>::min();


    //Set this to non-zero to add waves to the terrain.
    float waveHeight = .0f;

	for (size_t i = 0; i < pointsPerSide; ++i) {
		for (size_t j = 0; j < pointsPerSide; ++j) {
            float pointHeight = height + (std::sin((float)i * 0.75f) * waveHeight);
            max = std::max(pointHeight, max);
            min = std::min(pointHeight, min);
			points[(i * pointsPerSide) + j] = pointHeight;
 		}
	}

	btHeightfieldTerrainShape* groundShape = new btHeightfieldTerrainShape(pointsPerSide, pointsPerSide, points.data(), 1.0f, min, max, 1, PHY_FLOAT, false);
	//groundShape->setMargin(0.5);

	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,0,0));

	createRigidBody(.0,groundTransform,groundShape, btVector4(0,0,1,1));

    for (int boxHeight = 0; boxHeight < 5; boxHeight++) {

        btBoxShape* colShape = createBoxShape(btVector3(.1,.1f + (boxHeight * .2f),.1));

        m_collisionShapes.push_back(colShape);

        /// Create Dynamic Objects
        btTransform startTransform;
        startTransform.setIdentity();

        btScalar mass(1.f);

        for (int i = 0; i < 20; i++) {
            startTransform.setOrigin(btVector3(
                    btScalar(boxHeight - 4),
                    btScalar(1+i),
                    btScalar(10 - i)));
            btRigidBody* box = createRigidBody(mass,startTransform,colShape);
            box->setCcdMotionThreshold(0.1);
            box->setCcdSweptSphereRadius(0.1);
            box->setAngularFactor(btVector3(0,0,0));
        }
    }
	
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
	
}






CommonExampleInterface*    TerrainExampleCreateFunc(CommonExampleOptions& options)
{
	return new TerrainExample(options.m_guiHelper);

}


B3_STANDALONE_EXAMPLE(TerrainExampleCreateFunc)



