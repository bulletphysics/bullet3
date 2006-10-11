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



/*
Raytracer uses the Convex rayCast to visualize the Collision Shapes/Minkowski Sum.
Very basic raytracer, rendering into a texture.
*/

///Low level demo, doesn't include btBulletCollisionCommon.h

#include "GL_Simplex1to4.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btTransform.h"
#include "GL_ShapeDrawer.h"

#include "Raytracer.h"
#include "GlutStuff.h"

#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.h"
#ifdef USE_ALGEBRAIC_CCD
#include "NarrowPhaseCollision/BU_CollisionPair.h"
#endif //USE_ALGEBRAIC_CCD


#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btMultiSphereShape.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btTetrahedronShape.h"
#include "BulletCollision/CollisionShapes/btConeShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"
#include "BulletCollision/CollisionShapes/btMinkowskiSumShape.h"



#include "RenderTexture.h"

btVoronoiSimplexSolver	simplexSolver;

float yaw=0.f,pitch=0.f,roll=0.f;
const int maxNumObjects = 4;
const int numObjects = 4;

/// simplex contains the vertices, and some extra code to draw and debug
GL_Simplex1to4	simplex;

btConvexShape*	shapePtr[maxNumObjects];
btTransform transforms[maxNumObjects];

renderTexture*	raytracePicture = 0;

int screenWidth = 128;
int screenHeight = 128;
GLuint glTextureId;

btSphereShape	mySphere(1);
btBoxShape myBox(btVector3(0.4f,0.4f,0.4f));
btCylinderShape myCylinder(btVector3(0.3f,0.3f,0.3f));
btConeShape myCone(1,1);

btMinkowskiSumShape myMink(&myCylinder,&myBox);


///
///
///
int main(int argc,char** argv)
{
	Raytracer* raytraceDemo = new Raytracer();

	raytraceDemo->initPhysics();
	
	raytraceDemo->setCameraDistance(6.f);

	return glutmain(argc, argv,screenWidth,screenHeight,"Minkowski-Sum Raytracer Demo",raytraceDemo);
}

void	Raytracer::initPhysics()
{
	raytracePicture = new renderTexture(screenWidth,screenHeight);

	myBox.setMargin(0.02f);
	myCone.setMargin(0.2f);

	simplex.setSimplexSolver(&simplexSolver);
	simplex.addVertex(btPoint3(-1,0,-1));
	simplex.addVertex(btPoint3(1,0,-1));
	simplex.addVertex(btPoint3(0,0,1));
	simplex.addVertex(btPoint3(0,1,0));
	
	
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

	// btMultiSphereShape* multiSphereShape = new btMultiSphereShape(inertiaHalfExtents,positions,radi,NUM_SPHERES);
	btConvexHullShape* convexHullShape = new btConvexHullShape(&positions[0].getX(),3);


	//choose shape
	shapePtr[0] = &myCone;
	shapePtr[1] =&simplex;
	shapePtr[2] =convexHullShape;
	shapePtr[3] =&myMink;//myBox;//multiSphereShape

	simplex.setMargin(0.3f);


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
		btVector3	pos(-3.5f+i*2.5f,0.f,0.f);
		transforms[i].setOrigin( pos );
		btQuaternion orn;
		if (i < 2)
		{
			orn.setEuler(yaw,pitch,roll);
			transforms[i].setRotation(orn);
		}
	}
	myMink.setTransformA(btTransform(transforms[0].getRotation()));

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
	

	btConvexCast::CastResult rayResult;
	btTransform rayToTrans;
	rayToTrans.setIdentity();
	btVector3 rayTo;
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
			//	rayFromLocal = transforms[s].inverse()* rayFromTrans;
			//	rayToLocal = transforms[s].inverse()* rayToTrans;

				//choose the continuous collision detection method
				btSubsimplexConvexCast convexCaster(&pointShape,shapePtr[s],&simplexSolver);
				//GjkConvexCast convexCaster(&pointShape,shapePtr[0],&simplexSolver);
				//ContinuousConvexCollision convexCaster(&pointShape,shapePtr[0],&simplexSolver,0);
				
				//	btBU_Simplex1to4	ptShape(btVector3(0,0,0));//algebraic needs features, doesnt use 'supporting vertex'
				//	BU_CollisionPair convexCaster(&ptShape,shapePtr[0]);


				//reset previous result
				rayResult.m_fraction = 1.f;


				if (convexCaster.calcTimeOfImpact(rayFromTrans,rayToTrans,transforms[s],transforms[s],rayResult))
				{
					//float fog = 1.f - 0.1f * rayResult.m_fraction;
					rayResult.m_normal.normalize();

					btVector3 worldNormal;
					worldNormal = transforms[s].getBasis() *rayResult.m_normal;

					float light = worldNormal.dot(btVector3(0.4f,-1.f,-0.4f));
					if (light < 0.2f)
						light = 0.2f;
					if (light > 1.f)
						light = 1.f;

					rgba = btVector4(light,light,light,1.f);
					raytracePicture->setPixel(x,y,rgba);
				} else
				{
					//clear is already done
					//rgba = btVector4(0.f,0.f,0.f,0.f);
					//raytracePicture->setPixel(x,y,rgba);

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
	glTranslatef(0.0f,0.0f,-3.0f);						// Move Into The Screen 5 Units



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

	glPushMatrix();



	
	/*
	/// normal opengl rendering
	float m[16];
	int i;

	for (i=0;i<numObjects;i++)
	{


		transA.getOpenGLMatrix( m );
		/// draw the simplex
		GL_ShapeDrawer::drawOpenGL(m,shapePtr[i],btVector3(1,1,1));
		/// calculate closest point from simplex to the origin, and draw this vector
		simplex.calcClosest(m);

	}
	*/

	glPopMatrix();

	pitch += 0.005f;
	yaw += 0.01f;

	glFlush();
	glutSwapBuffers();
}

