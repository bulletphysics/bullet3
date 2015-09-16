#include "VHACDDemo.h"

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btIDebugDraw.h"
#include "../../Extras/VHACD/public/VHACD.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "../SoftDemo/BunnyMesh.h"
#include "../SoftDemo/TorusMesh.h"

using namespace VHACD;

///Show the use of Volumetric Hierarchical Approximate Convex Decomposition.
class VHACDDemo : public CommonRigidBodyBase
{
public:
	VHACDDemo(struct GUIHelperInterface* helper);

	virtual void initPhysics();
	void doConvexDecomposition(const btVector3& offset,
		const btScalar* points, unsigned int stridePoints, unsigned int countPoints,
		const int* triangles, unsigned int strideTriangles, unsigned int countTriangles);

	virtual void resetCamera()
	{
		float dist = 10;
		float pitch = 135;
		float yaw = 30;
		float targetPos[3]={0,0,0};
		m_guiHelper->resetCamera(dist,pitch,yaw,
			targetPos[0],targetPos[1],targetPos[2]);
	}
};

void VHACDDemo::doConvexDecomposition(const btVector3& offset,
	const btScalar* points, unsigned int stridePoints, unsigned int countPoints,
	const int* triangles, unsigned int strideTriangles, unsigned int countTriangles)
{
	IVHACD* hacd = CreateVHACD();
	IVHACD::Parameters params = IVHACD::Parameters();
	params.m_oclAcceleration = false;
	bool res = hacd->Compute(points, stridePoints, countPoints,
		triangles, strideTriangles, countTriangles, params);

	if (res)
	{
		// Add the resulting convex shapes to a compound shape
		btCompoundShape* compoundShape = new btCompoundShape();

		unsigned int nConvexHulls = hacd->GetNConvexHulls();
		IVHACD::ConvexHull convexHull;
		for (unsigned int ch = 0; ch < nConvexHulls; ch++)
		{
			hacd->GetConvexHull(ch, convexHull);
			unsigned int nDoubles = convexHull.m_nPoints * 3;
			unsigned int i;

			// Calculate centroid (center of mass)
			btVector3 centroid(0,0,0);
			for (i = 0; i < nDoubles; i += 3)
			{
				centroid += btVector3(
					(btScalar)convexHull.m_points[i],
					(btScalar)convexHull.m_points[i + 1],
					(btScalar)convexHull.m_points[i + 2]);
			}
			centroid /= (btScalar)convexHull.m_nTriangles;


			// Create convex shape
			// Adjust points such that the centroid is at (0,0,0)
			btConvexHullShape* convexShape = convexShape = new btConvexHullShape();
			for (i = 0; i < nDoubles; i += 3)
			{
				btVector3 point(
					(btScalar)convexHull.m_points[i],
					(btScalar)convexHull.m_points[i + 1],
					(btScalar)convexHull.m_points[i + 2]);
				point -= centroid;
				convexShape->addPoint(point, false);
			}
			convexShape->recalcLocalAabb();
			m_collisionShapes.push_back(convexShape);


			// Append to the compound shape
			btTransform transform;
			transform.setIdentity();
			transform.setOrigin(centroid);
			compoundShape->addChildShape(transform, convexShape);


			// Also create a separate body for each convex shape
			transform.setOrigin(offset + btVector3(3,0,0) + centroid);
			createRigidBody(0.1f, transform, convexShape);
		}

		m_collisionShapes.push_back(compoundShape);
		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(offset + btVector3(-3,0,0));
		createRigidBody(1.0f, transform, compoundShape);
	}

	hacd->Clean();
	hacd->Release();
}

void VHACDDemo::initPhysics()
{
	// World setup
	m_guiHelper->setUpAxis(1);
	createEmptyDynamicsWorld();
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);


	// Ground
	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0), 0);
	m_collisionShapes.push_back(groundShape);
	btTransform groundTransform;
	groundTransform.setIdentity();
	btRigidBody* groundBody = createRigidBody(0, groundTransform, groundShape);


	// Convex decomposition on the Stanford bunny and the torus mesh
	doConvexDecomposition(btVector3(0,0.5f,2),
		&gVerticesBunny[0], 3, BUNNY_NUM_VERTICES,
		gIndicesBunny[0], 3, BUNNY_NUM_TRIANGLES);
	/*
	doConvexDecomposition(btVector3(0,1,-3),
		&gVertices[0], 3, NUM_VERTICES,
		gIndices[0], 3, NUM_TRIANGLES);
	*/

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

VHACDDemo::VHACDDemo(struct GUIHelperInterface* helper)
:CommonRigidBodyBase(helper)
{
}

class CommonExampleInterface* VHACDCreateFunc(struct CommonExampleOptions& options)
{
	return new VHACDDemo(options.m_guiHelper);
}
