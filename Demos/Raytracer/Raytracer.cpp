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
Raytracer uses the Convex Raycast to visualize the Collision Shapes/Minkowski Sum.
Very basic raytracer, rendering into a texture.
*/

#include "GL_Simplex1to4.h"
#include "SimdQuaternion.h"
#include "SimdTransform.h"
#include "GL_ShapeDrawer.h"

#include "Raytracer.h"
#include "GlutStuff.h"

#include "NarrowPhaseCollision/VoronoiSimplexSolver.h"
#include "NarrowPhaseCollision/SubSimplexConvexCast.h"
#include "NarrowPhaseCollision/GjkConvexCast.h"
#include "NarrowPhaseCollision/ContinuousConvexCollision.h"
#ifdef USE_ALGEBRAIC_CCD
#include "NarrowPhaseCollision/BU_CollisionPair.h"
#endif //USE_ALGEBRAIC_CCD


#include "CollisionShapes/SphereShape.h"
#include "CollisionShapes/MultiSphereShape.h"
#include "CollisionShapes/ConvexHullShape.h"
#include "CollisionShapes/BoxShape.h"
#include "CollisionShapes/Simplex1to4Shape.h"
#include "CollisionShapes/ConeShape.h"
#include "CollisionShapes/CylinderShape.h"
#include "CollisionShapes/MinkowskiSumShape.h"



#include "RenderTexture.h"

VoronoiSimplexSolver	simplexSolver;

float yaw=0.f,pitch=0.f,roll=0.f;
const int maxNumObjects = 4;
const int numObjects = 4;

/// simplex contains the vertices, and some extra code to draw and debug
GL_Simplex1to4	simplex;

ConvexShape*	shapePtr[maxNumObjects];
SimdTransform transforms[maxNumObjects];

RenderTexture*	raytracePicture = 0;

int screenWidth = 128;
int screenHeight = 128;
GLuint glTextureId;

SphereShape	mySphere(1);
BoxShape myBox(SimdVector3(0.4f,0.4f,0.4f));
CylinderShape myCylinder(SimdVector3(0.3f,0.3f,0.3f));
ConeShape myCone(1,1);

MinkowskiSumShape myMink(&myCylinder,&myBox);


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
	raytracePicture = new RenderTexture(screenWidth,screenHeight);

	myBox.SetMargin(0.02f);
	myCone.SetMargin(0.2f);

	simplex.SetSimplexSolver(&simplexSolver);
	simplex.AddVertex(SimdPoint3(-1,0,-1));
	simplex.AddVertex(SimdPoint3(1,0,-1));
	simplex.AddVertex(SimdPoint3(0,0,1));
	simplex.AddVertex(SimdPoint3(0,1,0));
	
	
	/// convex hull of 5 spheres
#define NUM_SPHERES 5
	SimdVector3 inertiaHalfExtents(10.f,10.f,10.f);
	SimdVector3 positions[NUM_SPHERES] = {
		SimdVector3(-1.2f,	-0.3f,	0.f),
		SimdVector3(0.8f,	-0.3f,	0.f),
		SimdVector3(0.5f,	0.6f,	0.f),
		SimdVector3(-0.5f,	0.6f,	0.f),
		SimdVector3(0.f,	0.f,	0.f)
	};

	// MultiSphereShape* multiSphereShape = new MultiSphereShape(inertiaHalfExtents,positions,radi,NUM_SPHERES);
	ConvexHullShape* convexHullShape = new ConvexHullShape(positions,3);


	//choose shape
	shapePtr[0] = &myCone;
	shapePtr[1] =&simplex;
	shapePtr[2] =convexHullShape;
	shapePtr[3] =&myMink;//myBox;//multiSphereShape

	simplex.SetMargin(0.3f);


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
		SimdVector3	pos(-3.5f+i*2.5f,0.f,0.f);
		transforms[i].setOrigin( pos );
		SimdQuaternion orn;
		if (i < 2)
		{
			orn.setEuler(yaw,pitch,roll);
			transforms[i].setRotation(orn);
		}
	}
	myMink.SetTransformA(SimdTransform(transforms[0].getRotation()));

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






	SimdVector4 rgba(1.f,0.f,0.f,0.5f);

	float top = 1.f;
	float bottom = -1.f;
	float nearPlane = 1.f;

	float tanFov = (top-bottom)*0.5f / nearPlane;

	float fov = 2.0 * atanf (tanFov);


	SimdVector3	rayFrom = getCameraPosition();
	SimdVector3 rayForward = getCameraTargetPosition()-getCameraPosition();
	rayForward.normalize();
	float farPlane = 600.f;
	rayForward*= farPlane;

	SimdVector3 rightOffset;
	SimdVector3 vertical(0.f,1.f,0.f);
	SimdVector3 hor;
	hor = rayForward.cross(vertical);
	hor.normalize();
	vertical = hor.cross(rayForward);
	vertical.normalize();

	float tanfov = tanf(0.5f*fov);

	hor *= 2.f * farPlane * tanfov;
	vertical *= 2.f * farPlane * tanfov;

	SimdVector3 rayToCenter = rayFrom + rayForward;

	SimdVector3 dHor = hor * 1.f/float(screenWidth);
	SimdVector3 dVert = vertical * 1.f/float(screenHeight);

	SimdTransform rayFromTrans;
	rayFromTrans.setIdentity();
	rayFromTrans.setOrigin(rayFrom);

	SimdTransform rayFromLocal;
	SimdTransform	rayToLocal;


	SphereShape pointShape(0.0f);


	///clear texture
	for (int x=0;x<screenWidth;x++)
	{
		for (int y=0;y<screenHeight;y++)
		{
			SimdVector4 rgba(0.f,0.f,0.f,0.f);
			raytracePicture->SetPixel(x,y,rgba);
		}
	}
	

	ConvexCast::CastResult rayResult;
	SimdTransform rayToTrans;
	rayToTrans.setIdentity();
	SimdVector3 rayTo;
	for (int x=0;x<screenWidth;x++)
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
				SubsimplexConvexCast convexCaster(&pointShape,shapePtr[s],&simplexSolver);
				//GjkConvexCast convexCaster(&pointShape,shapePtr[0],&simplexSolver);
				//ContinuousConvexCollision convexCaster(&pointShape,shapePtr[0],&simplexSolver,0);
				
				//	BU_Simplex1to4	ptShape(SimdVector3(0,0,0));//algebraic needs features, doesnt use 'supporting vertex'
				//	BU_CollisionPair convexCaster(&ptShape,shapePtr[0]);


				//reset previous result
				rayResult.m_fraction = 1.f;


				if (convexCaster.calcTimeOfImpact(rayFromTrans,rayToTrans,transforms[s],transforms[s],rayResult))
				{
					//float fog = 1.f - 0.1f * rayResult.m_fraction;
					rayResult.m_normal.normalize();

					SimdVector3 worldNormal;
					worldNormal = transforms[s].getBasis() *rayResult.m_normal;

					float light = worldNormal.dot(SimdVector3(0.4f,-1.f,-0.4f));
					if (light < 0.2f)
						light = 0.2f;
					if (light > 1.f)
						light = 1.f;

					rgba = SimdVector4(light,light,light,1.f);
					raytracePicture->SetPixel(x,y,rgba);
				} else
				{
					//clear is already done
					//rgba = SimdVector4(0.f,0.f,0.f,0.f);
					//raytracePicture->SetPixel(x,y,rgba);

				}

				
			}
		}
	}

#define TEST_PRINTF
#ifdef TEST_PRINTF

	
	extern BMF_FontData BMF_font_helv10;
	
	raytracePicture->Printf("CCD RAYTRACER",&BMF_font_helv10);
	char buffer[256];
	sprintf(buffer,"%d RAYS / Frame",screenWidth*screenHeight*numObjects);
	raytracePicture->Printf(buffer,&BMF_font_helv10,0,10);
	

#endif //TEST_PRINTF

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glFrustum(-1.0,1.0,-1.0,1.0,3,2020.0);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();									// Reset The Modelview Matrix
	glTranslatef(0.0f,0.0f,-3.0f);						// Move Into The Screen 5 Units



	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D,glTextureId );

	const unsigned char *ptr = raytracePicture->GetBuffer();
	glTexImage2D(GL_TEXTURE_2D, 
		0, 
		GL_RGBA, 
		raytracePicture->GetWidth(),raytracePicture->GetHeight(), 
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

	GL_ShapeDrawer::DrawCoordSystem();

	glPushMatrix();



	
	/*
	/// normal opengl rendering
	float m[16];
	int i;

	for (i=0;i<numObjects;i++)
	{


		transA.getOpenGLMatrix( m );
		/// draw the simplex
		GL_ShapeDrawer::DrawOpenGL(m,shapePtr[i],SimdVector3(1,1,1));
		/// calculate closest point from simplex to the origin, and draw this vector
		simplex.CalcClosest(m);

	}
	*/

	glPopMatrix();

	pitch += 0.005f;
	yaw += 0.01f;

	glFlush();
	glutSwapBuffers();
}

