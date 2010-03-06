/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Experimental Buoyancy fluid demo written by John McCutchan
*/

#ifdef _WIN32 //needed for glut.h
#include <windows.h>
#endif

//think different
#if defined(__APPLE__) && !defined (VMDMESA)
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include "GlutStuff.h"
#include "HfFluidDemo_GL_ShapeDrawer.h"

#include "BulletCollision/CollisionShapes/btPolyhedralConvexShape.h"
#include "BulletCollision/CollisionShapes/btTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btConeShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"
#include "BulletCollision/CollisionShapes/btTetrahedronShape.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/CollisionShapes/btCapsuleShape.h"
#include "BulletCollision/CollisionShapes/btConvexTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btUniformScalingShape.h"
#include "BulletCollision/CollisionShapes/btStaticPlaneShape.h"
///
#include "BulletCollision/CollisionShapes/btShapeHull.h"
#include "BulletHfFluid/btHfFluidBuoyantConvexShape.h"
#include "BulletHfFluid/btHfFluid.h"
#include "BulletHfFluid/btHfFluidCollisionShape.h"

#include "LinearMath/btTransformUtil.h"


#include "LinearMath/btIDebugDraw.h"
//for debugmodes
#include <stdio.h> //printf debugging

#include <map>

using namespace std;


class GlDrawcallback : public btTriangleCallback
{

public:

	bool	m_wireframe;

	GlDrawcallback()
		:m_wireframe(false)
	{
	}

	virtual void processTriangle(btVector3* triangle,int partId, int triangleIndex)
	{

		(void)triangleIndex;
		(void)partId;


		if (m_wireframe)
		{
			glBegin(GL_LINES);
			glColor3f(1, 0, 0);
			glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ());
			glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ());
			glColor3f(0, 1, 0);
			glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ());
			glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ());
			glColor3f(0, 0, 1);
			glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ());
			glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ());
			glEnd();
		} else
		{
			glBegin(GL_TRIANGLES);
			//glColor3f(1, 1, 1);
			
			
			glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ());
			glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ());
			glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ());

			glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ());
			glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ());
			glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ());
			glEnd();
		}
	}
};

class TriangleGlDrawcallback : public btInternalTriangleIndexCallback
{
public:
	virtual void internalProcessTriangleIndex(btVector3* triangle,int partId,int  triangleIndex)
	{
		(void)triangleIndex;
		(void)partId;


		glBegin(GL_TRIANGLES);//LINES);
		glColor3f(1, 0, 0);
		glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ());
		glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ());
		glColor3f(0, 1, 0);
		glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ());
		glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ());
		glColor3f(0, 0, 1);
		glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ());
		glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ());
		glEnd();
	}
};



void HfFluidDemo_GL_ShapeDrawer::drawOpenGL(btScalar* m, const btCollisionShape* shape, const btVector3& color,int	debugMode,const btVector3& worldBoundsMin,const btVector3& worldBoundsMax)
{


	glPushMatrix(); 
	btglMultMatrix(m);

	if (shape->getShapeType() == UNIFORM_SCALING_SHAPE_PROXYTYPE)
	{
		const btUniformScalingShape* scalingShape = static_cast<const btUniformScalingShape*>(shape);
		const btConvexShape* convexShape = scalingShape->getChildShape();
		float	scalingFactor = (float)scalingShape->getUniformScalingFactor();
		{
			btScalar tmpScaling[4][4]={{scalingFactor,0,0,0},
			{0,scalingFactor,0,0},
			{0,0,scalingFactor,0},
			{0,0,0,1}};

			drawOpenGL( (btScalar*)tmpScaling,convexShape,color,debugMode,worldBoundsMin,worldBoundsMax);
		}
		glPopMatrix();
		return;
	}

	if (shape->getShapeType() == HFFLUID_BUOYANT_CONVEX_SHAPE_PROXYTYPE)
	{
		btConvexShape* convexShape = ((btHfFluidBuoyantConvexShape*)shape)->getConvexShape();
		btTransform I;
		I.setIdentity();
		btScalar mat[16];
		I.getOpenGLMatrix (&mat[0]);
		drawOpenGL (mat, convexShape, color, debugMode, worldBoundsMin, worldBoundsMax);
		return;
	}

	if (shape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE)
	{
		const btCompoundShape* compoundShape = static_cast<const btCompoundShape*>(shape);
		for (int i=compoundShape->getNumChildShapes()-1;i>=0;i--)
		{
			btTransform childTrans = compoundShape->getChildTransform(i);
			const btCollisionShape* colShape = compoundShape->getChildShape(i);
			btScalar childMat[16];
			childTrans.getOpenGLMatrix(childMat);
			drawOpenGL(childMat,colShape,color,debugMode,worldBoundsMin,worldBoundsMax);

		}

	} else
	{
		if(m_textureenabled&&(!m_textureinitialized))
		{
			GLubyte*	image=new GLubyte[256*256*3];
			for(int y=0;y<256;++y)
			{
				const int	t=y>>4;
				GLubyte*	pi=image+y*256*3;
				for(int x=0;x<256;++x)
				{
					const int		s=x>>4;
					const GLubyte	b=180;					
					GLubyte			c=b+((s+t&1)&1)*(255-b);
					pi[0]=pi[1]=pi[2]=c;pi+=3;
				}
			}
			glGenTextures(1,(GLuint*)&m_texturehandle);
			glBindTexture(GL_TEXTURE_2D,m_texturehandle);
			glTexEnvf(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);
			glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_LINEAR);
			glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR_MIPMAP_LINEAR);
			glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
			glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
			gluBuild2DMipmaps(GL_TEXTURE_2D,3,256,256,GL_RGB,GL_UNSIGNED_BYTE,image);
			delete[] image;

			glMatrixMode(GL_TEXTURE);
			glLoadIdentity();
			glScalef(0.025,0.025,0.025);

			static const GLfloat	planex[]={1,0,0,0};
			static const GLfloat	planey[]={0,1,0,0};
			static const GLfloat	planez[]={0,0,1,0};
			glTexGenfv(GL_S,GL_OBJECT_PLANE,planex);
			glTexGenfv(GL_T,GL_OBJECT_PLANE,planez);
			glTexGeni(GL_S,GL_TEXTURE_GEN_MODE,GL_OBJECT_LINEAR);
			glTexGeni(GL_T,GL_TEXTURE_GEN_MODE,GL_OBJECT_LINEAR);
			glEnable(GL_TEXTURE_GEN_S);
			glEnable(GL_TEXTURE_GEN_T);
			glEnable(GL_TEXTURE_GEN_R);
			m_textureinitialized=true;
		}
		//drawCoordSystem();

		//glPushMatrix();
		glEnable(GL_COLOR_MATERIAL);
		if(m_textureenabled) 
		{
			glEnable(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D,m_texturehandle);
		} else
		{
			glDisable(GL_TEXTURE_2D);
		}


		glColor3f(color.x(),color.y(), color.z());		

		bool useWireframeFallback = true;

		if (!(debugMode & btIDebugDraw::DBG_DrawWireframe))
		{
			///you can comment out any of the specific cases, and use the default
			///the benefit of 'default' is that it approximates the actual collision shape including collision margin
			int shapetype=m_textureenabled?MAX_BROADPHASE_COLLISION_TYPES:shape->getShapeType();
			switch (shapetype)
			{
			case BOX_SHAPE_PROXYTYPE:
				{
					const btBoxShape* boxShape = static_cast<const btBoxShape*>(shape);
					btVector3 halfExtent = boxShape->getHalfExtentsWithMargin();
					glScaled(2*halfExtent[0], 2*halfExtent[1], 2*halfExtent[2]);
					glutSolidCube(1.0);
					useWireframeFallback = false;
					break;
				}


			case SPHERE_SHAPE_PROXYTYPE:
				{
					const btSphereShape* sphereShape = static_cast<const btSphereShape*>(shape);
					float radius = sphereShape->getMargin();//radius doesn't include the margin, so draw with margin
					glutSolidSphere(radius,10,10);
					useWireframeFallback = false;
					break;
				}

			case CONE_SHAPE_PROXYTYPE:
				{
					const btConeShape* coneShape = static_cast<const btConeShape*>(shape);
					int upIndex = coneShape->getConeUpIndex();
					float radius = coneShape->getRadius();//+coneShape->getMargin();
					float height = coneShape->getHeight();//+coneShape->getMargin();
					switch (upIndex)
					{
					case 0:
						glRotatef(90.0, 0.0, 1.0, 0.0);
						break;
					case 1:
						glRotatef(-90.0, 1.0, 0.0, 0.0);
						break;
					case 2:
						break;
					default:
						{
						}
					};

					glTranslatef(0.0, 0.0, -0.5*height);
					glutSolidCone(radius,height,10,10);
					useWireframeFallback = false;
					break;

				}


			case STATIC_PLANE_PROXYTYPE:
				{
					const btStaticPlaneShape* staticPlaneShape = static_cast<const btStaticPlaneShape*>(shape);
					btScalar planeConst = staticPlaneShape->getPlaneConstant();
					const btVector3& planeNormal = staticPlaneShape->getPlaneNormal();
					btVector3 planeOrigin = planeNormal * planeConst;
					btVector3 vec0,vec1;
					btPlaneSpace1(planeNormal,vec0,vec1);
					btScalar vecLen = 100.f;
					btVector3 pt0 = planeOrigin + vec0*vecLen;
					btVector3 pt1 = planeOrigin - vec0*vecLen;
					btVector3 pt2 = planeOrigin + vec1*vecLen;
					btVector3 pt3 = planeOrigin - vec1*vecLen;
					glBegin(GL_LINES);
					glVertex3f(pt0.getX(),pt0.getY(),pt0.getZ());
					glVertex3f(pt1.getX(),pt1.getY(),pt1.getZ());
					glVertex3f(pt2.getX(),pt2.getY(),pt2.getZ());
					glVertex3f(pt3.getX(),pt3.getY(),pt3.getZ());
					glEnd();


					break;

				}

			case CYLINDER_SHAPE_PROXYTYPE:
				{
					const btCylinderShape* cylinder = static_cast<const btCylinderShape*>(shape);
					int upAxis = cylinder->getUpAxis();


					float radius = cylinder->getRadius();
					float halfHeight = cylinder->getHalfExtentsWithMargin()[upAxis];

					drawCylinder(radius,halfHeight,upAxis);

					break;
				}

			default:
				{


					if (shape->isConvex())
					{
						ShapeCache*	sc=cache((btConvexShape*)shape);

						//if (shape->getUserPointer())
						{
							//glutSolidCube(1.0);
							btShapeHull* hull = &sc->m_shapehull/*(btShapeHull*)shape->getUserPointer()*/;


							if (hull->numTriangles () > 0)
							{
								int index = 0;
								const unsigned int* idx = hull->getIndexPointer();
								const btVector3* vtx = hull->getVertexPointer();

								glBegin (GL_TRIANGLES);

								for (int i = 0; i < hull->numTriangles (); i++)
								{
									int i1 = index++;
									int i2 = index++;
									int i3 = index++;
									btAssert(i1 < hull->numIndices () &&
										i2 < hull->numIndices () &&
										i3 < hull->numIndices ());

									int index1 = idx[i1];
									int index2 = idx[i2];
									int index3 = idx[i3];
									btAssert(index1 < hull->numVertices () &&
										index2 < hull->numVertices () &&
										index3 < hull->numVertices ());

									btVector3 v1 = vtx[index1];
									btVector3 v2 = vtx[index2];
									btVector3 v3 = vtx[index3];
									btVector3 normal = (v3-v1).cross(v2-v1);
									normal.normalize ();

									glNormal3f(normal.getX(),normal.getY(),normal.getZ());
									glVertex3f (v1.x(), v1.y(), v1.z());
									glVertex3f (v2.x(), v2.y(), v2.z());
									glVertex3f (v3.x(), v3.y(), v3.z());

								}
								glEnd ();

							}
						}
					}
				}
			}

		}




		/// for polyhedral shapes
		if (debugMode==btIDebugDraw::DBG_DrawFeaturesText && (shape->isPolyhedral()))
		{
			btPolyhedralConvexShape* polyshape = (btPolyhedralConvexShape*) shape;

			{
				glRasterPos3f(0.0,  0.0,  0.0);
				//BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),polyshape->getExtraDebugInfo());

				glColor3f(1.f, 1.f, 1.f);
				int i;
				for (i=0;i<polyshape->getNumVertices();i++)
				{
					btVector3 vtx;
					polyshape->getVertex(i,vtx);
					glRasterPos3f(vtx.x(),  vtx.y(),  vtx.z());
					char buf[12];
					sprintf(buf," %d",i);
//					BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
				}

				for (i=0;i<polyshape->getNumPlanes();i++)
				{
					btVector3 normal;
					btVector3 vtx;
					polyshape->getPlane(normal,vtx,i);
					btScalar d = vtx.dot(normal);

					glRasterPos3f(normal.x()*d,  normal.y()*d, normal.z()*d);
					char buf[12];
					sprintf(buf," plane %d",i);
//					BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);

				}
			}

		}


#ifdef USE_DISPLAY_LISTS

		if (shape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE||shape->getShapeType() == GIMPACT_SHAPE_PROXYTYPE)
		{
			GLuint dlist =   OGL_get_displaylist_for_shape((btCollisionShape * )shape);
			if (dlist)
			{
				glCallList(dlist);
			}
			else
			{
#else		
		if (shape->isConcave())//>getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE||shape->getShapeType() == GIMPACT_SHAPE_PROXYTYPE)
			//		if (shape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE)
		{
			btConcaveShape* concaveMesh = (btConcaveShape*) shape;

			GlDrawcallback drawCallback;
			drawCallback.m_wireframe = (debugMode & btIDebugDraw::DBG_DrawWireframe)!=0;

			concaveMesh->processAllTriangles(&drawCallback,worldBoundsMin,worldBoundsMax);

		}
		if (shape->getShapeType() == HFFLUID_SHAPE_PROXYTYPE)
		{
			btHfFluidCollisionShape* hfFluidShape = (btHfFluidCollisionShape*)shape;
			btHfFluid* fluid = hfFluidShape->m_fluid;

			GlDrawcallback drawCallback;
			drawCallback.m_wireframe = (debugMode & btIDebugDraw::DBG_DrawWireframe) != 0;
			fluid->foreachSurfaceTriangle (&drawCallback, worldBoundsMin, worldBoundsMax);
		}
#endif

#ifdef USE_DISPLAY_LISTS
	}
}
#endif

/*
if (shape->getShapeType() == CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE)
{
btConvexTriangleMeshShape* convexMesh = (btConvexTriangleMeshShape*) shape;

//todo: pass camera for some culling			
btVector3 aabbMax(btScalar(BT_LARGE_FLOAT),btScalar(BT_LARGE_FLOAT),btScalar(BT_LARGE_FLOAT));
btVector3 aabbMin(-btScalar(BT_LARGE_FLOAT),-btScalar(BT_LARGE_FLOAT),-btScalar(BT_LARGE_FLOAT));
TriangleGlDrawcallback drawCallback;
convexMesh->getMeshInterface()->InternalProcessAllTriangles(&drawCallback,aabbMin,aabbMax);

}
*/



glDisable(GL_DEPTH_TEST);
glRasterPos3f(0,0,0);//mvtx.x(),  vtx.y(),  vtx.z());
if (debugMode&btIDebugDraw::DBG_DrawText)
{
//	BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),shape->getName());
}

if (debugMode& btIDebugDraw::DBG_DrawFeaturesText)
{
	//BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),shape->getExtraDebugInfo());
}
glEnable(GL_DEPTH_TEST);

//	glPopMatrix();	
if(m_textureenabled) glDisable(GL_TEXTURE_2D);
	}
	glPopMatrix();

}

//
void		HfFluidDemo_GL_ShapeDrawer::drawShadow(btScalar* m,const btVector3& extrusion,const btCollisionShape* shape,const btVector3& worldBoundsMin,const btVector3& worldBoundsMax)
{
	glPushMatrix(); 
	btglMultMatrix(m);
	if(shape->getShapeType() == UNIFORM_SCALING_SHAPE_PROXYTYPE)
	{
		const btUniformScalingShape* scalingShape = static_cast<const btUniformScalingShape*>(shape);
		const btConvexShape* convexShape = scalingShape->getChildShape();
		float	scalingFactor = (float)scalingShape->getUniformScalingFactor();
		btScalar tmpScaling[4][4]={	{scalingFactor,0,0,0},
		{0,scalingFactor,0,0},
		{0,0,scalingFactor,0},
		{0,0,0,1}};
		drawShadow((btScalar*)tmpScaling,extrusion,convexShape,worldBoundsMin,worldBoundsMax);
		glPopMatrix();
		return;
	}
	else if(shape->getShapeType()==COMPOUND_SHAPE_PROXYTYPE)
	{
		const btCompoundShape* compoundShape = static_cast<const btCompoundShape*>(shape);
		for (int i=compoundShape->getNumChildShapes()-1;i>=0;i--)
		{
			btTransform childTrans = compoundShape->getChildTransform(i);
			const btCollisionShape* colShape = compoundShape->getChildShape(i);
			btScalar childMat[16];
			childTrans.getOpenGLMatrix(childMat);
			drawShadow(childMat,extrusion*childTrans.getBasis(),colShape,worldBoundsMin,worldBoundsMax);
		}
	}
	else
	{
		bool useWireframeFallback = true;
		if (shape->isConvex())
		{
			ShapeCache*	sc=cache((btConvexShape*)shape);
			btShapeHull* hull =&sc->m_shapehull;
			glBegin(GL_QUADS);
			for(int i=0;i<sc->m_edges.size();++i)
			{			
				const btScalar		d=btDot(sc->m_edges[i].n[0],extrusion);
				if((d*btDot(sc->m_edges[i].n[1],extrusion))<0)
				{
					const int			q=	d<0?1:0;
					const btVector3&	a=	hull->getVertexPointer()[sc->m_edges[i].v[q]];
					const btVector3&	b=	hull->getVertexPointer()[sc->m_edges[i].v[1-q]];
					glVertex3f(a[0],a[1],a[2]);
					glVertex3f(b[0],b[1],b[2]);
					glVertex3f(b[0]+extrusion[0],b[1]+extrusion[1],b[2]+extrusion[2]);
					glVertex3f(a[0]+extrusion[0],a[1]+extrusion[1],a[2]+extrusion[2]);
				}
			}
			glEnd();
		}

	}




	if (shape->isConcave())//>getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE||shape->getShapeType() == GIMPACT_SHAPE_PROXYTYPE)
		//		if (shape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE)
	{
		btConcaveShape* concaveMesh = (btConcaveShape*) shape;

		GlDrawcallback drawCallback;
		drawCallback.m_wireframe = false;

		concaveMesh->processAllTriangles(&drawCallback,worldBoundsMin,worldBoundsMax);

	}
	glPopMatrix();

}

//
HfFluidDemo_GL_ShapeDrawer::HfFluidDemo_GL_ShapeDrawer()
{
	m_texturehandle		=	0;
	m_textureenabled	=	true;
	m_textureinitialized	=	false;
}

HfFluidDemo_GL_ShapeDrawer::~HfFluidDemo_GL_ShapeDrawer()
{
}

