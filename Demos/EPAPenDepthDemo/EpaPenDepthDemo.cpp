
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

EPA Copyright (c) Ricardo Padrela 2006 

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


/**
*	Sample that shows the Expanding Polytope Algorithm ( EPA )
*	bterates two convex shapes and calculates the penetration depth
*	between them in case they are penetrating
*/

#include "GL_Simplex1to4.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btTransform.h"
#include "GL_ShapeDrawer.h"
#include <GL/glut.h>
#include "GlutStuff.h"

#include <iostream>
#include <list>
#include <time.h>

#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"

#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"

#include "NarrowPhaseCollision/EpaCommon.h"
#include "NarrowPhaseCollision/EpaVertex.h"
#include "NarrowPhaseCollision/EpaHalfEdge.h"
#include "NarrowPhaseCollision/EpaFace.h"
#include "NarrowPhaseCollision/EpaPolyhedron.h"
#include "NarrowPhaseCollision/Epa.h"
#include "BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h"
#include "NarrowPhaseCollision/EpaPenetrationDepthSolver.h"
EpaPenetrationDepthSolver epaPenDepthSolver;


btSimplexSolverInterface	simplexSolver;


int screenWidth = 640.f;
int screenHeight = 480.f;

// Scene stuff
btPoint3	g_sceneVolumeMin( -1, -1, -1 );
btPoint3	g_sceneVolumeMax( 1, 1, 1 );

bool		g_shapesPenetrate = false;

btVector3	g_wWitnesses[ 2 ];

// Shapes stuff
btConvexShape*	g_pConvexShapes[ 2 ] = { 0 };
btTransform	g_convexShapesTransform[ 2 ];

btScalar		g_animAngle = SIMD_RADS_PER_DEG;
bool			g_pauseAnim = true;

// 0 - Box ; 1 - Sphere
int g_shapesType[ 2 ] = { 0 };

// Box config
btVector3 g_boxExtents( 1, 1, 1 );
// Sphere config
btScalar	g_sphereRadius = 1;

float randomFloat( float rangeMin, float rangeMax )
{
	return ( ( ( float ) ( rand() ) / ( float ) ( RAND_MAX ) ) * ( rangeMax - rangeMin ) + rangeMin );
}

int randomShapeType( int minShapeType, int maxShapeType )
{
	return ( ( ( ( maxShapeType - minShapeType ) + 1 ) * rand() ) / ( ( RAND_MAX + 1 ) + minShapeType ) );
}

btVector3 randomPosition( const btPoint3& minPoint, const btPoint3& maxPoint )
{
	return btVector3( randomFloat( minPoint.getX(), maxPoint.getX() ),
		randomFloat( minPoint.getY(), maxPoint.getY() ),
		randomFloat( minPoint.getZ(), maxPoint.getZ() ) );
}

bool createBoxShape( int shapeIndex )
{
	//#ifdef _DEBUG
	//static bool b = true;
	//
	//if ( b )
	//{
	//	g_pConvexShapes[ shapeIndex ] = new btBoxShape( btVector3( 1, 1, 1 ) );

	//	g_pConvexShapes[ shapeIndex ]->setMargin( 0.05 );

	//	g_convexShapesTransform[ shapeIndex ].setIdentity();

	//	btMatrix3x3 basis(  0.99365157, 0.024418538, -0.10981932,
	//						 -0.025452739, 0.99964380, -0.0080251107,
	//						  0.10958424, 0.010769366, 0.99391919 );

	//	g_convexShapesTransform[ shapeIndex ].setOrigin( btVector3( 4.4916530, -19.059078, -0.22695254 ) );
	//	g_convexShapesTransform[ shapeIndex ].setBasis( basis );

	//	b = false;
	//}
	//else
	//{
	//	g_pConvexShapes[ shapeIndex ] = new btBoxShape( btVector3( 25, 10, 25 ) );

	//	g_pConvexShapes[ shapeIndex ]->setMargin( 0.05 );

	//	//btMatrix3x3 basis( 0.658257, 0.675022, -0.333709,
	//	//					-0.333120, 0.658556, 0.675023,
	//	//					 0.675314, -0.333120, 0.658256 );

	//	g_convexShapesTransform[ shapeIndex ].setIdentity();

	//	g_convexShapesTransform[ shapeIndex ].setOrigin( btVector3( 0, -30, 0/*0.326090, -0.667531, 0.214331*/ ) );
	//	//g_convexShapesTransform[ shapeIndex ].setBasis( basis );
	//}
	//#endif

	g_pConvexShapes[ shapeIndex ] = new btBoxShape( btVector3( 1, 1, 1 ) );

	g_pConvexShapes[ shapeIndex ]->setMargin( 1e-1 );

	g_convexShapesTransform[ shapeIndex ].setIdentity();

	g_convexShapesTransform[ shapeIndex ].setOrigin( randomPosition( g_sceneVolumeMin, g_sceneVolumeMax ) );

	return true;
}

bool createSphereShape( int shapeIndex )
{
	g_pConvexShapes[ shapeIndex ] = new btSphereShape( g_sphereRadius );

	g_pConvexShapes[ shapeIndex ]->setMargin( 1e-1 );

	g_convexShapesTransform[ shapeIndex ].setIdentity();
	g_convexShapesTransform[ shapeIndex ].setOrigin( randomPosition( g_sceneVolumeMin, g_sceneVolumeMax ) );

	//#ifdef _DEBUG
	//static bool b = true;
	//if ( b )
	//{
	//	g_convexShapesTransform[ shapeIndex ].setOrigin( btVector3( 0.001, 0, 0 ) );
	//	b = false;
	//}
	//else
	//{
	//	g_convexShapesTransform[ shapeIndex ].setOrigin( btVector3( 0, 0, 0 ) );
	//}
	//#endif

	return true;
}

void destroyShapes()
{
	if ( g_pConvexShapes[ 0 ] )
	{
		delete g_pConvexShapes[ 0 ];
		g_pConvexShapes[ 0 ] = 0;
	}

	if ( g_pConvexShapes[ 1 ] ) 
	{
		delete g_pConvexShapes[ 1 ];
		g_pConvexShapes[ 1 ] = 0;
	}
}

bool calcPenDepth()
{
	// Ryn Hybrid Pen Depth and EPA if necessary

	btVector3 v( 1, 0, 0 );

	btScalar squaredDistance = SIMD_INFINITY;
	btScalar delta = 0.f;

	const btScalar margin     = g_pConvexShapes[ 0 ]->getMargin() + g_pConvexShapes[ 1 ]->getMargin();
	const btScalar marginSqrd = margin * margin;

	btScalar maxRelErrorSqrd = 1e-3 * 1e-3;

	simplexSolver.reset();

	while ( true )
	{
		assert( ( v.length2() > 0 ) && "Warning: v is the zero vector!" );

		btVector3 seperatingAxisInA = -v * g_convexShapesTransform[ 0 ].getBasis();
		btVector3 seperatingAxisInB =  v * g_convexShapesTransform[ 1 ].getBasis();

		btVector3 pInA = g_pConvexShapes[ 0 ]->localGetSupportingVertexWithoutMargin( seperatingAxisInA );
		btVector3 qInB = g_pConvexShapes[ 1 ]->localGetSupportingVertexWithoutMargin( seperatingAxisInB );

		btPoint3  pWorld = g_convexShapesTransform[ 0 ]( pInA );
		btPoint3  qWorld = g_convexShapesTransform[ 1 ]( qInB );

		btVector3 w = pWorld - qWorld;
		delta = v.dot( w );

		// potential exit, they don't overlap
		if ( ( delta > 0 ) && ( ( delta * delta / squaredDistance ) > marginSqrd ) )
		{
			// Convex shapes do not overlap
			return false;
		}

		//exit 0: the new point is already in the simplex, or we didn't come any closer
		if ( ( squaredDistance - delta <= squaredDistance * maxRelErrorSqrd ) || simplexSolver.inSimplex( w ) )
		{
			simplexSolver.compute_points( g_wWitnesses[ 0 ], g_wWitnesses[ 1 ] );

			assert( ( squaredDistance > 0 ) && "squaredDistance is zero!" );
			btScalar vLength = sqrt( squaredDistance );

			g_wWitnesses[ 0 ] -= v * ( g_pConvexShapes[ 0 ]->getMargin() / vLength );
			g_wWitnesses[ 1 ] += v * ( g_pConvexShapes[ 1 ]->getMargin() / vLength );

			return true;
		}

		//add current vertex to simplex
		simplexSolver.addVertex( w, pWorld, qWorld );

		//calculate the closest point to the origin (update vector v)
		if ( !simplexSolver.closest( v ) )
		{
			simplexSolver.compute_points( g_wWitnesses[ 0 ], g_wWitnesses[ 1 ] );

			assert( ( squaredDistance > 0 ) && "squaredDistance is zero!" );
			btScalar vLength = sqrt( squaredDistance );

			g_wWitnesses[ 0 ] -= v * ( g_pConvexShapes[ 0 ]->getMargin() / vLength );
			g_wWitnesses[ 1 ] += v * ( g_pConvexShapes[ 1 ]->getMargin() / vLength );

			return true;
		}

		btScalar previousSquaredDistance = squaredDistance;
		squaredDistance = v.length2();

		//are we getting any closer ?
		if ( previousSquaredDistance - squaredDistance <= SIMD_EPSILON * previousSquaredDistance ) 
		{ 
			simplexSolver.backup_closest( v );
			squaredDistance = v.length2();

			simplexSolver.compute_points( g_wWitnesses[ 0 ], g_wWitnesses[ 1 ] );

			assert( ( squaredDistance > 0 ) && "squaredDistance is zero!" );
			btScalar vLength = sqrt( squaredDistance );

			g_wWitnesses[ 0 ] -= v * ( g_pConvexShapes[ 0 ]->getMargin() / vLength );
			g_wWitnesses[ 1 ] += v * ( g_pConvexShapes[ 1 ]->getMargin() / vLength );

			return true;
		}

		if ( simplexSolver.fullSimplex() || ( squaredDistance <= SIMD_EPSILON * simplexSolver.maxVertex() ) )
		{
			// Run EPA
			break;
		}
	}

	return epaPenDepthSolver.calcPenDepth( simplexSolver, g_pConvexShapes[ 0 ], g_pConvexShapes[ 1 ],
		g_convexShapesTransform[ 0 ], g_convexShapesTransform[ 1 ], v,
		g_wWitnesses[ 0 ], g_wWitnesses[ 1 ], 0 );
}

int main(int argc,char** argv)
{
	srand( time( 0 ) );

	g_shapesType[ 0 ] = randomShapeType( 0, 1 );
	g_shapesType[ 1 ] = randomShapeType( 0, 1 );

	( g_shapesType[ 0 ] == 0 ) ? createBoxShape( 0 ) : createSphereShape( 0 );
	( g_shapesType[ 1 ] == 0 ) ? createBoxShape( 1 ) : createSphereShape( 1 );

	g_shapesPenetrate = calcPenDepth();

	return glutmain( argc, argv, screenWidth, screenHeight, "EPAPenDepthDemo" );
}

void drawShape( int shapeIndex )
{
	float m[ 16 ];
	g_convexShapesTransform[ shapeIndex ].getOpenGLMatrix( m );

	glMultMatrixf( m );

	if ( g_pConvexShapes[ shapeIndex ]->getShapeType() == BOX_SHAPE_PROXYTYPE )
	{
		glutWireCube( ( ( btBoxShape* ) g_pConvexShapes[ shapeIndex ] )->getHalfExtents().x() * 2 );
	}
	else if ( g_pConvexShapes[ shapeIndex ]->getShapeType() == SPHERE_SHAPE_PROXYTYPE )
	{
		glutWireSphere( 1, 16, 16 );
	}
}

void drawPenDepthVector()
{
	glLoadIdentity();

	glColor3f( 1, 1, 0 );

	glPointSize( 5 );
	glBegin( GL_POINTS );

	glVertex3f( g_wWitnesses[ 0 ].getX(), g_wWitnesses[ 0 ].getY(), g_wWitnesses[ 0 ].getZ() );
	glVertex3f( g_wWitnesses[ 1 ].getX(), g_wWitnesses[ 1 ].getY(), g_wWitnesses[ 1 ].getZ() );

	glEnd();

	glColor3f( 1, 0, 0 );

	glBegin( GL_LINES );

	glVertex3f( g_wWitnesses[ 0 ].getX(), g_wWitnesses[ 0 ].getY(), g_wWitnesses[ 0 ].getZ() );
	glVertex3f( g_wWitnesses[ 1 ].getX(), g_wWitnesses[ 1 ].getY(), g_wWitnesses[ 1 ].getZ() );

	glEnd();
}

void clientMoveAndDisplay()
{
	if ( !g_pauseAnim )
	{
		btMatrix3x3 rot;
		rot.setEulerZYX( g_animAngle * 0.05, g_animAngle * 0.05, g_animAngle * 0.05 );

		btTransform t;
		t.setIdentity();
		t.setBasis( rot );

		//g_convexShapesTransform[ 0 ].mult( g_convexShapesTransform[ 0 ], t );
		g_convexShapesTransform[ 1 ].mult( g_convexShapesTransform[ 1 ], t );

		g_shapesPenetrate = calcPenDepth();
	}

	clientDisplay();
}

void clientDisplay(void) {

	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	glDisable( GL_LIGHTING );

	GL_ShapeDrawer::drawCoordSystem();

	glMatrixMode( GL_MODELVIEW );

	for ( int i = 0; i < 2; ++i )
	{
		glPushMatrix();

		drawShape( i );

		glPopMatrix();
	}

	if ( g_shapesPenetrate )
	{
		glPushMatrix();
		drawPenDepthVector();
		glPopMatrix();
	}

	glFlush();
	glutSwapBuffers();
}

void clientResetScene()
{
}

void clientKeyboard(unsigned char key, int x, int y)
{
	if ( key == 'R' || key == 'r' )
	{
		destroyShapes();

		g_shapesType[ 0 ] = randomShapeType( 0, 1 );
		g_shapesType[ 1 ] = randomShapeType( 0, 1 );

		( g_shapesType[ 0 ] == 0 ) ? createBoxShape( 0 ) : createSphereShape( 0 );
		( g_shapesType[ 1 ] == 0 ) ? createBoxShape( 1 ) : createSphereShape( 1 );

		g_shapesPenetrate = calcPenDepth();
	}
	else if ( key == 'Q' || key == 'q' )
	{
		destroyShapes();
	}
	else if ( key == 'T' || key == 't' )
	{
#ifdef DEBUG_ME
		btVector3 shapeAPos = g_convexShapesTransform[ 0 ].getOrigin();
		btVector3 shapeBPos = g_convexShapesTransform[ 1 ].getOrigin();

		btMatrix3x3 shapeARot = g_convexShapesTransform[ 0 ].getBasis();
		btMatrix3x3 shapeBRot = g_convexShapesTransform[ 1 ].getBasis();

		FILE* fp = 0;

		fopen_s( &fp, "shapes.txt", "w" );

		char str[ 256 ];
		sprintf_s( str, 256, "PosA: %f, %f, %f\nPosB: %f, %f, %f\n", shapeAPos.x(), shapeAPos.y(), shapeAPos.z(),
			shapeBPos.x(), shapeBPos.y(), shapeBPos.z() );
		fputs( str, fp );

		sprintf_s( str, 256, "RotA: %f, %f, %f\n%f, %f, %f\n%f, %f, %f\nRotB: %f, %f, %f\n%f, %f, %f\n%f, %f, %f\n\n",
			shapeARot.getRow( 0 ).x(), shapeARot.getRow( 0 ).y(), shapeARot.getRow( 0 ).z(),
			shapeARot.getRow( 1 ).x(), shapeARot.getRow( 1 ).y(), shapeARot.getRow( 1 ).z(),
			shapeARot.getRow( 2 ).x(), shapeARot.getRow( 2 ).y(), shapeARot.getRow( 2 ).z(),
			shapeBRot.getRow( 0 ).x(), shapeBRot.getRow( 0 ).y(), shapeBRot.getRow( 0 ).z(),
			shapeBRot.getRow( 1 ).x(), shapeBRot.getRow( 1 ).y(), shapeBRot.getRow( 1 ).z(),
			shapeBRot.getRow( 2 ).x(), shapeBRot.getRow( 2 ).y(), shapeBRot.getRow( 2 ).z());
		fputs( str, fp );

		fclose( fp );
#endif //DEBUG_ME
	}
	else if ( key == 'P' || key =='p' )
	{
		g_pauseAnim = !g_pauseAnim;
	}

	defaultKeyboard(key, x, y);
}


void clientMouseFunc(int button, int state, int x, int y)
{
}

void clientMotionFunc(int x,int y)
{
}
