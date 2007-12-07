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



#include "RenderTexture.h"



static btVoronoiSimplexSolver	simplexSolver;

static float yaw=0.f,pitch=0.f,roll=0.f;
static const int maxNumObjects = 4;
static const int numObjects = 1;


static btConvexShape*	shapePtr[maxNumObjects];
static btTransform transforms[maxNumObjects];

renderTexture*	raytracePicture = 0;

//this applies to the raytracer virtual screen/image buffer
static int screenWidth = 128;
//float aspectRatio = (3.f/4.f);
static int screenHeight = 128;//screenWidth * aspectRatio;
GLuint glTextureId;

btConeShape myCone(1,1);




///
///
///

void	Raytracer::initPhysics()
{
	raytracePicture = new renderTexture(screenWidth,screenHeight);

	myCone.setMargin(0.2f);

	
	/// convex hull of 5 spheres
#define NUM_SPHERES 5
	btVector3 inertiaHalfExtents(10.f,10.f,10.f);
	btVector3 positions[NUM_SPHERES] = {
		btVector3(-1.2f,	-0.3f,	0.f),
		btVector3(0.8f,	-0.3f,	0.f),
		btVector3(0.5f,	0.6f,	0.f),
		btVector3(-0.5f,	0.6f,	0.f),
		btVector3(0.f,	0.f,	0.f)
	};


	btVector3 sphereOffset1(0,0,0);
	btScalar sphereRadius = 2.f;
	btVector3 nonUniformScaling(0.5,2,0.5);
	//choose shape
	shapePtr[0] = &myCone;
	


}

Raytracer::~Raytracer()
{

	delete raytracePicture;
	raytracePicture=0;
}

//to be implemented by the demo

void Raytracer::clientMoveAndDisplay()
{
	displayCallback();
}

int once = 1;






void Raytracer::displayCallback() 
{

	updateCamera();

	for (int i=0;i<numObjects;i++)
	{
		transforms[i].setIdentity();
		btVector3	pos(-(2.5* numObjects * 0.5)+i*2.5f,0.f,0.f);
		transforms[i].setOrigin( pos );
		btQuaternion orn;
		if (i < 2)
		{
			orn.setEuler(yaw,pitch,roll);
			//transforms[i].setRotation(orn);
		}
	}

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	glDisable(GL_LIGHTING);
	if (once)
	{
		glGenTextures(1, &glTextureId);
		glBindTexture(GL_TEXTURE_2D,glTextureId );
		once = 0;
		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	}



	glDisable(GL_TEXTURE_2D);
	glDisable(GL_BLEND);

#define RAYTRACER
#ifdef RAYTRACER






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


	btSphereShape pointShape(0.0f);

	int x;

	///clear texture
	for (x=0;x<screenWidth;x++)
	{
		for (int y=0;y<screenHeight;y++)
		{
			btVector4 rgba(0.f,0.f,0.f,0.f);
			raytracePicture->setPixel(x,y,rgba);
		}
	}
	

#define USE_WORLD_RAYCAST 1
#ifndef USE_WORLD_RAYCAST
	btConvexCast::CastResult rayResult;
#endif

	btTransform rayToTrans;
	rayToTrans.setIdentity();
	btVector3 rayTo;
	btTransform colObjWorldTransform;
	colObjWorldTransform.setIdentity();

	

	for (x=0;x<screenWidth;x++)
	{
		for (int y=0;y<screenHeight;y++)
		{
			rayTo = rayToCenter - 0.5f * hor + 0.5f * vertical;
			rayTo += x * dHor;
			rayTo -= y * dVert;
			rayToTrans.setOrigin(rayTo);
			for (int s=0;s<numObjects;s++)
			{
				//do some culling, ray versus aabb
				btVector3 aabbMin,aabbMax;
				shapePtr[s]->getAabb(transforms[s],aabbMin,aabbMax);
				btScalar hitLambda = 1.f;
				btVector3 hitNormal;

				btCollisionWorld::ClosestRayResultCallback resultCallback(rayFrom,rayTo);
				btCollisionObject	tmpObj;
				tmpObj.setWorldTransform(transforms[s]);
		

				if (btRayAabb(rayFrom,rayTo,aabbMin,aabbMax,hitLambda,hitNormal))
				{

#ifdef USE_WORLD_RAYCAST
					btCollisionWorld::rayTestSingle(rayFromTrans,rayToTrans,
					  &tmpObj,
					  shapePtr[s],
					  transforms[s],
					  resultCallback);
					  if (resultCallback.HasHit())
					  {
							//float fog = 1.f - 0.1f * rayResult.m_fraction;
							resultCallback.m_hitNormalWorld.normalize();//.m_normal.normalize();
							btVector3 worldNormal = resultCallback.m_hitNormalWorld;
							
#else //use USE_WORLD_RAYCAST
					//reset previous result
					rayResult.m_fraction = 1.f;

					//choose the continuous collision detection method

					btSubsimplexConvexCast convexCaster(&pointShape,shapePtr[s],&simplexSolver);
					//btGjkConvexCast convexCaster(&pointShape,shapePtr[0],&simplexSolver);
					//btContinuousConvexCollision convexCaster(&pointShape,shapePtr[0],&simplexSolver,0);
					
					if (convexCaster.calcTimeOfImpact(rayFromTrans,rayToTrans,transforms[s],transforms[s],rayResult))
						{
							btVector3 worldNormal;
							worldNormal = transforms[s].getBasis() *rayResult.m_normal;
							worldNormal.normalize();
#endif //	USE_WORLD_RAYCAST
					
//					
					

						

						float lightVec0 = worldNormal.dot(btVector3(0,-1,-1));//0.4f,-1.f,-0.4f));
						float lightVec1= worldNormal.dot(btVector3(-1,0,-1));//-0.4f,-1.f,-0.4f));


						rgba = btVector4(lightVec0,lightVec1,0,1.f);
						rgba.setMin(btVector3(1,1,1));
						rgba.setMax(btVector3(0.2,0.2,0.2));
						rgba[3] = 1.f;
						raytracePicture->setPixel(x,y,rgba);
					} else
					{
						//clear is already done
						//rgba = btVector4(0.f,0.f,0.f,0.f);
						//raytracePicture->setPixel(x,y,rgba);
					}
				} else
				{
					btVector4 rgba = raytracePicture->getPixel(x,y);
					if (!rgba.length2())
					{
						raytracePicture->setPixel(x,y,btVector4(1,1,1,1));
					}
				}
				
			}
		}
	}

#define TEST_PRINTF
#ifdef TEST_PRINTF

	
	extern BMF_FontData BMF_font_helv10;
	
	raytracePicture->grapicalPrintf("CCD RAYTRACER",&BMF_font_helv10);
	char buffer[256];
	sprintf(buffer,"%d RAYS / Frame",screenWidth*screenHeight*numObjects);
	raytracePicture->grapicalPrintf(buffer,&BMF_font_helv10,0,10);
	

#endif //TEST_PRINTF

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

#endif //RAYRACER

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

	glFlush();
	glutSwapBuffers();
}

