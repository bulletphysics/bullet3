/*
* Copyright (c) 2005 Erwin Coumans <www.erwincoumans.com>
*
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Erwin Coumans makes no representations about the suitability 
* of this software for any purpose.  
* It is provided "as is" without express or implied warranty.
*/



#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"

/*
Raytracer uses the Convex rayCast to visualize the Collision Shapes/Minkowski Sum.
Very basic raytracer, rendering into a texture.
*/

///Low level demo, doesn't include btBulletCollisionCommon.h

#include "LinearMath/btQuaternion.h"
#include "LinearMath/btTransform.h"
#include "GL_ShapeDrawer.h"
#include "GLDebugDrawer.h"

#include "Raytracer.h"
#include "GlutStuff.h"


#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.h"

#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btMultiSphereShape.h"

#include "BulletCollision/CollisionShapes/btConvexHullShape.h"
#include "LinearMath/btAabbUtil2.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"


#include "BulletCollision/CollisionShapes/btTetrahedronShape.h"
#include "BulletCollision/CollisionShapes/btConeShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"
#include "BulletCollision/CollisionShapes/btMinkowskiSumShape.h"

#include "BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h"
#include "BulletCollision/BroadphaseCollision/btAxisSweep3.h"

#include "RenderTexture.h"



static btVoronoiSimplexSolver	simplexSolver;

static float yaw=0.f,pitch=0.f,roll=0.f;
static const int maxNumObjects = 4;
static const int numObjects = 3;


static btConvexShape*	shapePtr[maxNumObjects];
static btTransform transforms[maxNumObjects];

renderTexture*	raytracePicture = 0;

//this applies to the raytracer virtual screen/image buffer
static int screenWidth = 128;//256;
//float aspectRatio = (3.f/4.f);
static int screenHeight = 64;//256;//screenWidth * aspectRatio;
GLuint glTextureId;

btConeShape myCone(1,1);
btSphereShape mysphere(1);
btBoxShape mybox(btVector3(1,1,1));

btCollisionWorld* m_collisionWorld = 0;



///
///
///

void	Raytracer::initPhysics()
{
	m_ele = 0;

	raytracePicture = new renderTexture(screenWidth,screenHeight);
	myCone.setMargin(0.2f);

	//choose shape
	shapePtr[0] = &myCone;
	shapePtr[1] = &mysphere;
	shapePtr[2] = &mybox;

	for (int i=0;i<numObjects;i++)
	{
		transforms[i].setIdentity();
		btVector3	pos(0.f,0.f,-(2.5* numObjects * 0.5)+i*2.5f);
		transforms[i].setIdentity();
		transforms[i].setOrigin( pos );
		btQuaternion orn;
		if (i < 2)
		{
			orn.setEuler(yaw,pitch,roll);
			transforms[i].setRotation(orn);
		}
	}


	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	btVector3 worldMin(-1000,-1000,-1000);
	btVector3 worldMax(1000,1000,1000);
	m_overlappingPairCache = new btAxisSweep3(worldMin,worldMax);

	m_collisionWorld = new btCollisionWorld(m_dispatcher,m_overlappingPairCache,m_collisionConfiguration);
	
	for (int s=0;s<numObjects;s++)
	{
		btCollisionObject* obj = new btCollisionObject();
		obj->setCollisionShape(shapePtr[s]);
		obj->setWorldTransform(transforms[s]);
		m_collisionWorld->addCollisionObject(obj);
	}


}

Raytracer::~Raytracer()
{

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_collisionWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_collisionWorld->getCollisionObjectArray()[i];
		m_collisionWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision world
	delete m_collisionWorld;

	//delete broadphase
	delete m_overlappingPairCache;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;

	delete raytracePicture;
	raytracePicture=0;
}

//to be implemented by the demo

void Raytracer::clientMoveAndDisplay()
{
	displayCallback();
}






bool	Raytracer::worldRaytest(const btVector3& rayFrom,const btVector3& rayTo,btVector3& worldNormal,btVector3& worldHitPoint)
{

	struct	AllRayResultCallback : public btCollisionWorld::RayResultCallback
	{
		AllRayResultCallback(const btVector3&	rayFromWorld,const btVector3&	rayToWorld)
		:m_rayFromWorld(rayFromWorld),
		m_rayToWorld(rayToWorld)
		{
		}

		btVector3	m_rayFromWorld;//used to calculate hitPointWorld from hitFraction
		btVector3	m_rayToWorld;

		btVector3	m_hitNormalWorld;
		btVector3	m_hitPointWorld;
			
		virtual	btScalar	addSingleResult(btCollisionWorld::LocalRayResult& rayResult,bool normalInWorldSpace)
		{

//caller already does the filter on the m_closestHitFraction
			btAssert(rayResult.m_hitFraction <= m_closestHitFraction);
			
			m_closestHitFraction = rayResult.m_hitFraction;

			m_collisionObject = rayResult.m_collisionObject;
			if (normalInWorldSpace)
			{
				m_hitNormalWorld = rayResult.m_hitNormalLocal;
			} else
			{
				///need to transform normal into worldspace
				m_hitNormalWorld = m_collisionObject->getWorldTransform().getBasis()*rayResult.m_hitNormalLocal;
			}
			m_hitPointWorld.setInterpolate3(m_rayFromWorld,m_rayToWorld,rayResult.m_hitFraction);
			return 1.f;
		}
	};


	AllRayResultCallback	resultCallback(rayFrom,rayTo);
//	btCollisionWorld::ClosestRayResultCallback resultCallback(rayFrom,rayTo);
	m_collisionWorld->rayTest(rayFrom,rayTo,resultCallback);
	if (resultCallback.hasHit())
	{
		worldNormal = resultCallback.m_hitNormalWorld;
		return true;
	}
	return false;
}


bool	Raytracer::singleObjectRaytest(const btVector3& rayFrom,const btVector3& rayTo,btVector3& worldNormal,btVector3& worldHitPoint)
{

//	btScalar closestHitResults = 1.f;

	btCollisionWorld::ClosestRayResultCallback resultCallback(rayFrom,rayTo);

	bool hasHit = false;
	btConvexCast::CastResult rayResult;
	btSphereShape pointShape(0.0f);
	btTransform rayFromTrans;
	btTransform rayToTrans;

	rayFromTrans.setIdentity();
	rayFromTrans.setOrigin(rayFrom);
	rayToTrans.setIdentity();
	rayToTrans.setOrigin(rayTo);

	for (int s=0;s<numObjects;s++)
	{
		//comment-out next line to get all hits, instead of just the closest hit
		//resultCallback.m_closestHitFraction = 1.f;

		//do some culling, ray versus aabb
		btVector3 aabbMin,aabbMax;
		shapePtr[s]->getAabb(transforms[s],aabbMin,aabbMax);
		btScalar hitLambda = 1.f;
		btVector3 hitNormal;
		btCollisionObject	tmpObj;
		tmpObj.setWorldTransform(transforms[s]);


		if (btRayAabb(rayFrom,rayTo,aabbMin,aabbMax,hitLambda,hitNormal))
		{
			//reset previous result

			btCollisionWorld::rayTestSingle(rayFromTrans,rayToTrans, &tmpObj, shapePtr[s], transforms[s], resultCallback);
			if (resultCallback.hasHit())
			{
				//float fog = 1.f - 0.1f * rayResult.m_fraction;
				resultCallback.m_hitNormalWorld.normalize();//.m_normal.normalize();
				worldNormal = resultCallback.m_hitNormalWorld;
				//worldNormal = transforms[s].getBasis() *rayResult.m_normal;
				worldNormal.normalize();
				hasHit = true;
			}
		}
	}

	return hasHit;
}


bool	Raytracer::lowlevelRaytest(const btVector3& rayFrom,const btVector3& rayTo,btVector3& worldNormal,btVector3& worldHitPoint)
{

	btScalar closestHitResults = 1.f;

	bool hasHit = false;
	btConvexCast::CastResult rayResult;
	btSphereShape pointShape(0.0f);
	btTransform rayFromTrans;
	btTransform rayToTrans;

	rayFromTrans.setIdentity();
	rayFromTrans.setOrigin(rayFrom);
	rayToTrans.setIdentity();
	rayToTrans.setOrigin(rayTo);

	for (int s=0;s<numObjects;s++)
	{
		
		//do some culling, ray versus aabb
		btVector3 aabbMin,aabbMax;
		shapePtr[s]->getAabb(transforms[s],aabbMin,aabbMax);
		btScalar hitLambda = 1.f;
		btVector3 hitNormal;
		btCollisionObject	tmpObj;
		tmpObj.setWorldTransform(transforms[s]);


		if (btRayAabb(rayFrom,rayTo,aabbMin,aabbMax,hitLambda,hitNormal))
		{
			//reset previous result

			//choose the continuous collision detection method
			btSubsimplexConvexCast convexCaster(&pointShape,shapePtr[s],&simplexSolver);
			//btGjkConvexCast convexCaster(&pointShape,shapePtr[s],&simplexSolver);
			//btContinuousConvexCollision convexCaster(&pointShape,shapePtr[s],&simplexSolver,0);

			if (convexCaster.calcTimeOfImpact(rayFromTrans,rayToTrans,transforms[s],transforms[s],rayResult))
			{
				if (rayResult.m_fraction < closestHitResults)
				{
					closestHitResults = rayResult.m_fraction;

					worldNormal = transforms[s].getBasis() *rayResult.m_normal;
					worldNormal.normalize();
					hasHit = true;
				}
			}
		}
	}

	return hasHit;

}


void Raytracer::displayCallback() 
{

	updateCamera();

	for (int i=0;i<numObjects;i++)
	{
		transforms[i].setIdentity();
		btVector3	pos(0.f,0.f,-(2.5* numObjects * 0.5)+i*2.5f);
		transforms[i].setOrigin( pos );
		btQuaternion orn;
		if (i < 2)
		{
			orn.setEuler(yaw,pitch,roll);
			transforms[i].setRotation(orn);
		}
	}

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	glDisable(GL_LIGHTING);
	if (!m_initialized)
	{
		m_initialized = true;
		glGenTextures(1, &glTextureId);
	}
	
	glBindTexture(GL_TEXTURE_2D,glTextureId );
	
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_BLEND);


	btVector4 rgba(1.f,0.f,0.f,0.5f);

	float top = 1.f;
	float bottom = -1.f;
	float nearPlane = 1.f;

	float tanFov = (top-bottom)*0.5f / nearPlane;

	float fov = 2.0 * atanf (tanFov);


	btVector3	rayFrom = getCameraPosition();
	btVector3 rayForward = getCameraTargetPosition()-getCameraPosition();
	rayForward.normalize();
	float farPlane = 600.f;
	rayForward*= farPlane;

	btVector3 rightOffset;
	btVector3 vertical(0.f,1.f,0.f);
	btVector3 hor;
	hor = rayForward.cross(vertical);
	hor.normalize();
	vertical = hor.cross(rayForward);
	vertical.normalize();

	float tanfov = tanf(0.5f*fov);

	hor *= 2.f * farPlane * tanfov;
	vertical *= 2.f * farPlane * tanfov;

	btVector3 rayToCenter = rayFrom + rayForward;

	btVector3 dHor = hor * 1.f/float(screenWidth);
	btVector3 dVert = vertical * 1.f/float(screenHeight);

	btTransform rayFromTrans;
	rayFromTrans.setIdentity();
	rayFromTrans.setOrigin(rayFrom);

	btTransform rayFromLocal;
	btTransform	rayToLocal;




	int x;

	///clear texture
	for (x=0;x<screenWidth;x++)
	{
		for (int y=0;y<screenHeight;y++)
		{
			btVector4 rgba(0.2f,0.2f,0.2f,1.f);
			raytracePicture->setPixel(x,y,rgba);
		}
	}

#if 1
	btVector3 rayTo;
	btTransform colObjWorldTransform;
	colObjWorldTransform.setIdentity();

	int	mode = 0;

	for (x=0;x<screenWidth;x++)
	{
		for (int y=0;y<screenHeight;y++)
		{

			rayTo = rayToCenter - 0.5f * hor + 0.5f * vertical;
			rayTo += x * dHor;
			rayTo -= y * dVert;
			btVector3	worldNormal(0,0,0);
			btVector3	worldPoint(0,0,0);



			bool hasHit = false;
			int mode = 0;
			switch (mode)
			{
			case 0:
				hasHit = lowlevelRaytest(rayFrom,rayTo,worldNormal,worldPoint);
				break;
			case 1:
				hasHit = singleObjectRaytest(rayFrom,rayTo,worldNormal,worldPoint);
				break;
			case 2:
				hasHit = worldRaytest(rayFrom,rayTo,worldNormal,worldPoint);
				break;
			default:
				{
				}
			}

			if (hasHit)
			{
				float lightVec0 = worldNormal.dot(btVector3(0,-1,-1));//0.4f,-1.f,-0.4f));
				float lightVec1= worldNormal.dot(btVector3(-1,0,-1));//-0.4f,-1.f,-0.4f));


				rgba = btVector4(lightVec0,lightVec1,0,1.f);
				rgba.setMin(btVector3(1,1,1));
				rgba.setMax(btVector3(0.2,0.2,0.2));
				rgba[3] = 1.f;
				raytracePicture->setPixel(x,y,rgba);
			} else
            {
			//	btVector4 rgba = raytracePicture->getPixel(x,y);
            }
			if (!rgba.length2())
			{
				raytracePicture->setPixel(x,y,btVector4(1,1,1,1));
			}
		}
	}
#endif

extern unsigned char sFontData[];
	if (0)
	{

		const char* text="ABC abc 123 !@#";
			int x=0;
		for (int cc = 0;cc<strlen(text);cc++)
		{
		
			char testChar = text[cc];//'b';
			char ch = testChar-32;
			int startx=ch%16;
			int starty=ch/16;


			//for (int i=0;i<256;i++)
			for (int i=startx*16;i<(startx*16+16);i++)
			{
				int y=0;
				//for (int j=0;j<256;j++)
				//for (int j=0;j<256;j++)
				for (int j=starty*16;j<(starty*16+16);j++)
				{
					
					btVector4 rgba(0,0,0,1);
					rgba[0] = (sFontData[i*3+255*256*3-(256*j)*3])/255.f;
					//rgba[0] += (sFontData[(i+1)*3+255*256*3-(256*j)*3])/255.*0.25f;
					//rgba[0] += (sFontData[(i)*3+255*256*3-(256*j+1)*3])/255.*0.25f;
					//rgba[0] += (sFontData[(i+1)*3+255*256*3-(256*j+1)*3])/255.*0.25;

					//if (rgba[0]!=0.f)
					{
						rgba[1]=rgba[0];
						rgba[2]=rgba[0];
						rgba[3]=1.f;

						//raytracePicture->setPixel(x,y,rgba);
						raytracePicture->addPixel(x,y,rgba);
					}
					y++;
				}
				x++;
			}
		}
	}


	//raytracePicture->grapicalPrintf("CCD RAYTRACER",sFontData);
	char buffer[256];
	sprintf(buffer,"%d rays",screenWidth*screenHeight*numObjects);
	//sprintf(buffer,"Toggle",screenWidth*screenHeight*numObjects);
	//sprintf(buffer,"TEST",screenWidth*screenHeight*numObjects);
	//raytracePicture->grapicalPrintf(buffer,sFontData,0,10);//&BMF_font_helv10,0,10);
	raytracePicture->grapicalPrintf(buffer,sFontData,0,0);//&BMF_font_helv10,0,10);


	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glFrustum(-1.0,1.0,-1.0,1.0,3,2020.0);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();									// reset The Modelview Matrix
	glTranslatef(0.0f,0.0f,-3.1f);						// Move Into The Screen 5 Units



	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D,glTextureId );

	const unsigned char *ptr = raytracePicture->getBuffer();
	glTexImage2D(GL_TEXTURE_2D, 
		0, 
		GL_RGBA, 
		raytracePicture->getWidth(),raytracePicture->getHeight(), 
		0, 
		GL_RGBA, 
		GL_UNSIGNED_BYTE, 
		ptr);


	glEnable (GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glColor4f (1,1,1,1); // alpha=0.5=half visible

	glBegin(GL_QUADS);
	glTexCoord2f(0.0f, 0.0f);
	glVertex2f(-1,1);
	glTexCoord2f(1.0f, 0.0f);
	glVertex2f(1,1);
	glTexCoord2f(1.0f, 1.0f);
	glVertex2f(1,-1);
	glTexCoord2f(0.0f, 1.0f);
	glVertex2f(-1,-1);
	glEnd();



	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);


	glDisable(GL_TEXTURE_2D);
	glDisable(GL_DEPTH_TEST);

	GL_ShapeDrawer::drawCoordSystem();



	{
		for (int i=0;i<numObjects;i++)
		{
			btVector3 aabbMin,aabbMax;
			shapePtr[i]->getAabb(transforms[i],aabbMin,aabbMax);
		}
	}

	glPushMatrix();




	glPopMatrix();

	pitch += 0.005f;
	yaw += 0.01f;
	m_azi += 1.f;

	glFlush();
	glutSwapBuffers();
}

