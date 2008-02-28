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

#ifdef WIN32 //needed for glut.h
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
#include "GL_ShapeDrawer.h"
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
#include "btShapeHull.h"

#include "LinearMath/btTransformUtil.h"


#include "LinearMath/btIDebugDraw.h"
//for debugmodes
#include "BMF_Api.h"
#include <stdio.h> //printf debugging

//#define USE_DISPLAY_LISTS 1
#ifdef USE_DISPLAY_LISTS

#include <map>

using namespace std;

//Set for storing Display list per trimesh
struct TRIMESH_KEY
{
	btCollisionShape* m_shape;
	GLuint m_dlist;//OpenGL display list	
};

typedef map<unsigned long,TRIMESH_KEY> TRIMESH_KEY_MAP;

typedef pair<unsigned long,TRIMESH_KEY> TRIMESH_KEY_PAIR;

TRIMESH_KEY_MAP g_display_lists;

class GlDisplaylistDrawcallback : public btTriangleCallback
{
public:

	virtual void processTriangle(btVector3* triangle,int partId, int triangleIndex)
	{

		btVector3 diff1 = triangle[1] - triangle[0];
		btVector3 diff2 = triangle[2] - triangle[0];
		btVector3 normal = diff1.cross(diff2);

		normal.normalize();

		glBegin(GL_TRIANGLES);
		glColor3f(0, 1, 0);
		glNormal3d(normal.getX(),normal.getY(),normal.getZ());
		glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ());

		glColor3f(0, 1, 0);
		glNormal3d(normal.getX(),normal.getY(),normal.getZ());
		glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ());

		glColor3f(0, 1, 0);
		glNormal3d(normal.getX(),normal.getY(),normal.getZ());
		glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ());
		glEnd();

		/*glBegin(GL_LINES);
		glColor3f(1, 1, 0);
		glNormal3d(normal.getX(),normal.getY(),normal.getZ());
		glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ());
		glNormal3d(normal.getX(),normal.getY(),normal.getZ());
		glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ());
		glColor3f(1, 1, 0);
		glNormal3d(normal.getX(),normal.getY(),normal.getZ());
		glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ());
		glNormal3d(normal.getX(),normal.getY(),normal.getZ());
		glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ());
		glColor3f(1, 1, 0);
		glNormal3d(normal.getX(),normal.getY(),normal.getZ());
		glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ());
		glNormal3d(normal.getX(),normal.getY(),normal.getZ());
		glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ());
		glEnd();*/

		
	}
};

GLuint  OGL_get_displaylist_for_shape(btCollisionShape * shape)
{
	TRIMESH_KEY_MAP::iterator map_iter;
	
	unsigned long key = (unsigned long)shape;
	map_iter = g_display_lists.find(key);
	if(map_iter!=g_display_lists.end())
	{
		return map_iter->second.m_dlist;
	}

	return 0;
}

void OGL_displaylist_clean()
{
	TRIMESH_KEY_MAP::iterator map_iter,map_itend;

	map_iter = g_display_lists.begin();

	while(map_iter!=map_itend)
	{
		glDeleteLists(map_iter->second.m_dlist,1);		
		map_iter++;
	}

	g_display_lists.clear();
}


void OGL_displaylist_register_shape(btCollisionShape * shape)
{
	btVector3 aabbMax(btScalar(1e30),btScalar(1e30),btScalar(1e30));
	btVector3 aabbMin(-btScalar(1e30),-btScalar(1e30),-btScalar(1e30));
	GlDisplaylistDrawcallback drawCallback;
	TRIMESH_KEY dlist;

	dlist.m_dlist = glGenLists(1);
	dlist.m_shape = shape;

	unsigned long key = (unsigned long)shape;

	g_display_lists.insert(TRIMESH_KEY_PAIR(key,dlist));
	
	glNewList(dlist.m_dlist,GL_COMPILE);

	glEnable(GL_CULL_FACE);

	glCullFace(GL_BACK);

	if (shape->isConcave())
	{
		btConcaveShape* concaveMesh = (btConcaveShape*) shape;			
		//todo pass camera, for some culling		
		concaveMesh->processAllTriangles(&drawCallback,aabbMin,aabbMax);
	}

	glDisable(GL_CULL_FACE);	

	glEndList();
}
#endif //USE_DISPLAY_LISTS

void GL_ShapeDrawer::drawCoordSystem()  {
    glBegin(GL_LINES);
    glColor3f(1, 0, 0);
    glVertex3d(0, 0, 0);
    glVertex3d(1, 0, 0);
    glColor3f(0, 1, 0);
    glVertex3d(0, 0, 0);
    glVertex3d(0, 1, 0);
    glColor3f(0, 0, 1);
    glVertex3d(0, 0, 0);
    glVertex3d(0, 0, 1);
    glEnd();
	
}





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
			glColor3f(1, 0, 0);
			glVertex3d(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ());
			glColor3f(0, 1, 0);
			glVertex3d(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ());
			glColor3f(0, 0, 1);
			glVertex3d(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ());
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

void GL_ShapeDrawer::drawCylinder(float radius,float halfHeight, int upAxis)
{
	

	glPushMatrix();
	switch (upAxis)
	{
	case 0:
		glRotatef(-90.0, 0.0, 1.0, 0.0);
		glTranslatef(0.0, 0.0, -halfHeight);
		break;
	case 1:
		glRotatef(-90.0, 1.0, 0.0, 0.0);
		glTranslatef(0.0, 0.0, -halfHeight);
		break;
	case 2:
		
		glTranslatef(0.0, 0.0, -halfHeight);
		break;
	default:
		{
			assert(0);
		}

	}
	
	GLUquadricObj *quadObj = gluNewQuadric();

	//The gluCylinder subroutine draws a cylinder that is oriented along the z axis. 
	//The base of the cylinder is placed at z = 0; the top of the cylinder is placed at z=height. 
	//Like a sphere, the cylinder is subdivided around the z axis into slices and along the z axis into stacks.
	
	gluQuadricDrawStyle(quadObj, (GLenum)GLU_FILL);
	gluQuadricNormals(quadObj, (GLenum)GLU_SMOOTH);
	
	gluDisk(quadObj,0,radius,15, 10);
	
	gluCylinder(quadObj, radius, radius, 2.f*halfHeight, 15, 10);
	glTranslatef(0.0, 0.0, 2.*halfHeight);
	glRotatef(-180.0, 0.0, 1.0, 0.0);
	gluDisk(quadObj,0,radius,15, 10);

	glPopMatrix();
	gluDeleteQuadric(quadObj);
}



void GL_ShapeDrawer::drawOpenGL(btScalar* m, const btCollisionShape* shape, const btVector3& color,int	debugMode)
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
			
			drawOpenGL( (btScalar*)tmpScaling,convexShape,color,debugMode);
		}
		glPopMatrix();
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
			drawOpenGL(childMat,colShape,color,debugMode);
		}

	} else
	{
		//drawCoordSystem();
	    
		//glPushMatrix();
		glEnable(GL_COLOR_MATERIAL);
		glColor3f(color.x(),color.y(), color.z());

		

		bool useWireframeFallback = true;

		if (!(debugMode & btIDebugDraw::DBG_DrawWireframe))
		{
			///you can comment out any of the specific cases, and use the default
			///the benefit of 'default' is that it approximates the actual collision shape including collision margin
			switch (shape->getShapeType())
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
						btConvexShape* convexShape = (btConvexShape*)shape;
						if (!shape->getUserPointer())
						{
							//create a hull approximation
							void* mem = btAlignedAlloc(sizeof(btShapeHull),16);
							btShapeHull* hull = new(mem) btShapeHull(convexShape);
					
							///cleanup memory
							m_shapeHulls.push_back(hull);

							btScalar margin = shape->getMargin();
							hull->buildHull(margin);
							convexShape->setUserPointer(hull);

							
						//	printf("numTriangles = %d\n", hull->numTriangles ());
						//	printf("numIndices = %d\n", hull->numIndices ());
						//	printf("numVertices = %d\n", hull->numVertices ());
							

						}
						
						


						if (shape->getUserPointer())
						{
							//glutSolidCube(1.0);
							btShapeHull* hull = (btShapeHull*)shape->getUserPointer();

							
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
					} else
					{
//						printf("unhandled drawing\n");
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
					btPoint3 vtx;
					polyshape->getVertex(i,vtx);
					glRasterPos3f(vtx.x(),  vtx.y(),  vtx.z());
					char buf[12];
					sprintf(buf," %d",i);
					BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
				}

				for (i=0;i<polyshape->getNumPlanes();i++)
				{
					btVector3 normal;
					btPoint3 vtx;
					polyshape->getPlane(normal,vtx,i);
					btScalar d = vtx.dot(normal);

					glRasterPos3f(normal.x()*d,  normal.y()*d, normal.z()*d);
					char buf[12];
					sprintf(buf," plane %d",i);
					BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
					
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
			//btVector3 aabbMax(btScalar(1e30),btScalar(1e30),btScalar(1e30));
			//btVector3 aabbMax(100,100,100);//btScalar(1e30),btScalar(1e30),btScalar(1e30));

			//todo pass camera, for some culling
			btVector3 aabbMax(btScalar(1e30),btScalar(1e30),btScalar(1e30));
			btVector3 aabbMin(-btScalar(1e30),-btScalar(1e30),-btScalar(1e30));

			GlDrawcallback drawCallback;
			drawCallback.m_wireframe = (debugMode & btIDebugDraw::DBG_DrawWireframe)!=0;

			concaveMesh->processAllTriangles(&drawCallback,aabbMin,aabbMax);

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
			btVector3 aabbMax(btScalar(1e30),btScalar(1e30),btScalar(1e30));
			btVector3 aabbMin(-btScalar(1e30),-btScalar(1e30),-btScalar(1e30));
			TriangleGlDrawcallback drawCallback;
			convexMesh->getMeshInterface()->InternalProcessAllTriangles(&drawCallback,aabbMin,aabbMax);

		}
		*/

		

		glDisable(GL_DEPTH_BUFFER_BIT);
		glRasterPos3f(0,0,0);//mvtx.x(),  vtx.y(),  vtx.z());
		if (debugMode&btIDebugDraw::DBG_DrawText)
		{
			BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),shape->getName());
		}

		if (debugMode& btIDebugDraw::DBG_DrawFeaturesText)
		{
			//BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),shape->getExtraDebugInfo());
		}
		glEnable(GL_DEPTH_BUFFER_BIT);

	//	glPopMatrix();
	}
    glPopMatrix();
	
}


GL_ShapeDrawer::GL_ShapeDrawer()
{
}

GL_ShapeDrawer::~GL_ShapeDrawer()
{
	int i;
	for (i=0;i<m_shapeHulls.size();i++)
	{
		btShapeHull* hull = m_shapeHulls[i];
		hull->~btShapeHull();
		btAlignedFree(hull);
		m_shapeHulls[i] = 0;
	}
	m_shapeHulls.clear();
}

