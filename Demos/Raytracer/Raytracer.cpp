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



#define IN_TO_M_CONSTANT   (0.0254f)
#define M_TO_IN_CONSTANT   (39.3700787f)
#define CM_TO_IN_CONSTANT  (0.393700787f)
#define LBS_TO_KG_CONSTANT (0.45359237f)

#define FEET_TO_IN(x) (12.0f * (float)(x))
#define IN_TO_FT(x)   ((float)(x)/12.0f)
#define IN_TO_M(x)    ((float)(x) * IN_TO_M_CONSTANT)
#define M_TO_IN(x)    ((float)(x) * M_TO_IN_CONSTANT)
#define CM_TO_IN(x)   ((float)(x) * CM_TO_IN_CONSTANT)
#define FT_TO_M(x)    (IN_TO_M(FEET_TO_IN(x)))
#define LBS_TO_KG(x)  ((float)(x) * LBS_TO_KG_CONSTANT)

#define PIN_HEIGHT     IN_TO_M(15.0f)
#define PIN_DIAMETER   IN_TO_M(4.76f)
#define PIN_MASS       LBS_TO_KG(3.5f)
#define PIN_FRICTION   (BALL_FRICTION) // a guess
//#define PIN_COR        (0.67f) // was 0.67
#define PIN_COR        (0.2f)

#define BALL_DIAMETER     IN_TO_M(8.55f)
#define BALL_MASS         LBS_TO_KG(16.0f)
#define BALL_FRICTION     (0.3f) // max is 0.32
#define BALL_COR          (0.7)
#define BALL_MAX_FRICTION (0.32f)
#define BALL_MAX_MASS     (16.0f) // lbs
#define BALL_MIN_MASS     (8.0f)  // lbs

#define LANE_DECK_FUDGE (IN_TO_M(6.0f))

#define LANE_WIDTH        IN_TO_M(42.0f)
#define LANE_TOTAL_WIDTH  IN_TO_M(62.88f)
#define LANE_LENGTH       FT_TO_M(60.0f-LANE_DECK_FUDGE)

#define GRAVITY_VECTOR (btVector3(0.0f,-9.81f,0.0f))

#define PIN_Z_DIST IN_TO_M(10.3923048f)
#define PIN_Y_DIST IN_TO_M(12.0f)

#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"


/*
Raytracer uses the Convex rayCast to visualize the Collision Shapes/Minkowski Sum.
Very basic raytracer, rendering into a texture.
*/

///Low level demo, doesn't include btBulletCollisionCommon.h

#include "GL_Simplex1to4.h"
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
#ifdef USE_ALGEBRAIC_CCD
#include "NarrowPhaseCollision/BU_CollisionPair.h"
#endif //USE_ALGEBRAIC_CCD


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

/// simplex contains the vertices, and some extra code to draw and debug
static GL_Simplex1to4	simplex;

static btCollisionShape*	shapePtr[maxNumObjects];
static btTransform transforms[maxNumObjects];

renderTexture*	raytracePicture = 0;

//this applies to the raytracer virtual screen/image buffer
static int screenWidth = 128;
//float aspectRatio = (3.f/4.f);
static int screenHeight = 128;//screenWidth * aspectRatio;
GLuint glTextureId;

btSphereShape	mySphere(1);
btBoxShape myBox(btVector3(0.4f,0.4f,0.4f));
btCylinderShape myCylinder(btVector3(0.3f,0.3f,0.3f));
btConeShape myCone(1,1);
btCompoundShape compound;


btMinkowskiSumShape myMink(&myCylinder,&myBox);




///
///
///

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

	//btMultiSphereShape* multiSphereShape = new btMultiSphereShape(inertiaHalfExtents,positions,radi,NUM_SPHERES);

	btVector3 sphereOffset1(0,0,0);
	btScalar sphereRadius = 2.f;
	btVector3 nonUniformScaling(0.5,2,0.5);
	btMultiSphereShape* nonuniformScaledSphere = new btMultiSphereShape(inertiaHalfExtents,&sphereOffset1,&sphereRadius,1);
	nonuniformScaledSphere->setLocalScaling(nonUniformScaling);
	nonuniformScaledSphere->setMargin(0.04);
	btConvexHullShape* convexHullShape = new btConvexHullShape(&positions[0].getX(),3);

	//attempt to approximate a bowling pin
	//choose shape
	shapePtr[0] = &myCone;//&compound;//&myCone;//&myBox;//nonuniformScaledSphere;//&myCone;
	//	shapePtr[0] = &myCone;//&myBox;//nonuniformScaledSphere;//&myCone;
	
	shapePtr[1] =&simplex;
	shapePtr[2] =convexHullShape;
	shapePtr[3] =&myMink;//myBox;//multiSphereShape

	btVector3 sphereOffset(0,PIN_HEIGHT/4.0f,0);

	// create pin collision shape
	btCollisionShape* cyl = new btCylinderShape(btVector3(PIN_DIAMETER/4.0f, PIN_HEIGHT/4.0f, PIN_DIAMETER/4.0f));
	cyl->setMargin(IN_TO_M(0.000025f));
	btVector3	spherepositions[3] = {btVector3(0,-PIN_HEIGHT/2.f +(PIN_DIAMETER/2.0f)+IN_TO_M(1.25f),0),
btVector3(0,-PIN_HEIGHT/2.f +(PIN_DIAMETER/2.0f)+IN_TO_M(1.25f),0)+sphereOffset,
btVector3(0,-PIN_HEIGHT/2.f +(PIN_DIAMETER/2.0f)+IN_TO_M(1.25f),0)-sphereOffset};
	btScalar	radii[3] = {(PIN_DIAMETER/2.0f),(PIN_DIAMETER/4.0f),(PIN_DIAMETER/4.0f)};
	btCollisionShape* sph = new btMultiSphereShape(inertiaHalfExtents,spherepositions,radii,3);
	
	btTransform ident;
	ident.setIdentity();
	ident.setOrigin(btVector3(0.f,-PIN_HEIGHT/4.f,0.f));
	compound.addChildShape(ident,cyl);
	ident.setIdentity();
	//ident.setOrigin(btVector3(0.0f, -PIN_HEIGHT/2.0f + PIN_DIAMETER/2.0f + IN_TO_M(3.5f), 0.0f));
	compound.addChildShape(ident,sph);

	btVector3	spherepositions2[2] = {btVector3(0,+PIN_HEIGHT/2.f -(PIN_DIAMETER/4.0f),0),
		btVector3(0,0,0)};
	btScalar	radii2[2] = {(PIN_DIAMETER/4.0f),(PIN_DIAMETER/6.0f)};

	btCollisionShape* sph2 = new btMultiSphereShape(inertiaHalfExtents,spherepositions2,radii2,2);
	compound.addChildShape(ident,sph2);
	
	compound.setMargin(0.001);
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
		btVector3	pos(-(2.5* numObjects * 0.5)+i*2.5f,0.f,0.f);
		transforms[i].setOrigin( pos );
		btQuaternion orn;
		if (i < 2)
		{
			orn.setEuler(yaw,pitch,roll);
			//transforms[i].setRotation(orn);
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
	

//	btConvexCast::CastResult rayResult;
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


					btCollisionWorld::rayTestSingle(rayFromTrans,rayToTrans,
					  &tmpObj,
					  shapePtr[s],
					  transforms[s],
					  resultCallback);
				
					//choose the continuous collision detection method
					//btSubsimplexConvexCast convexCaster(&pointShape,shapePtr[s],&simplexSolver);
					//GjkConvexCast convexCaster(&pointShape,shapePtr[0],&simplexSolver);
					//ContinuousConvexCollision convexCaster(&pointShape,shapePtr[0],&simplexSolver,0);
					
					//reset previous result
					//rayResult.m_fraction = 1.f;
					if (resultCallback.HasHit())
//					if (convexCaster.calcTimeOfImpact(rayFromTrans,rayToTrans,transforms[s],transforms[s],rayResult))
					{
						//float fog = 1.f - 0.1f * rayResult.m_fraction;
						resultCallback.m_hitNormalWorld.normalize();//.m_normal.normalize();

						btVector3 worldNormal = resultCallback.m_hitNormalWorld;
//						worldNormal = transforms[s].getBasis() *rayResult.m_normal;
						

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

