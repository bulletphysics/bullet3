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

#include "GL_ShapeDrawer.h"
#include "CollisionShapes/PolyhedralConvexShape.h"
#include "CollisionShapes/TriangleMeshShape.h"
#include "CollisionShapes/BoxShape.h"
#include "CollisionShapes/SphereShape.h"
#include "CollisionShapes/ConeShape.h"
#include "CollisionShapes/CylinderShape.h"
#include "CollisionShapes/Simplex1to4Shape.h"
#include "CollisionShapes/CompoundShape.h"

#include "CollisionShapes/ConvexTriangleMeshShape.h"


#include "IDebugDraw.h"
//for debugmodes
#include "BMF_Api.h"
#include <stdio.h> //printf debugging

void GL_ShapeDrawer::DrawCoordSystem()  {
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





class GlDrawcallback : public TriangleCallback
{
public:

	virtual void ProcessTriangle(SimdVector3* triangle,int partId, int triangleIndex)
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

	}
};

class TriangleGlDrawcallback : public InternalTriangleIndexCallback
{
public:
	virtual void InternalProcessTriangleIndex(SimdVector3* triangle,int partId,int  triangleIndex)
	{
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


void GL_ShapeDrawer::DrawOpenGL(float* m, const CollisionShape* shape, const SimdVector3& color,int	debugMode)
{

	
	glPushMatrix(); 
    glMultMatrixf(m);

	if (shape->GetShapeType() == COMPOUND_SHAPE_PROXYTYPE)
	{
		const CompoundShape* compoundShape = static_cast<const CompoundShape*>(shape);
		for (int i=compoundShape->GetNumChildShapes()-1;i>=0;i--)
		{
			SimdTransform childTrans = compoundShape->GetChildTransform(i);
			const CollisionShape* colShape = compoundShape->GetChildShape(i);
			float childMat[16];
			childTrans.getOpenGLMatrix(childMat);
			DrawOpenGL(childMat,colShape,color,debugMode);
		}

	} else
	{
		//DrawCoordSystem();
	    
		//glPushMatrix();
		glEnable(GL_COLOR_MATERIAL);
		glColor3f(color.x(),color.y(), color.z());

		

		bool useWireframeFallback = true;

		if (!(debugMode & IDebugDraw::DBG_DrawWireframe))
		{
			switch (shape->GetShapeType())
			{
			case BOX_SHAPE_PROXYTYPE:
				{
					const BoxShape* boxShape = static_cast<const BoxShape*>(shape);
					SimdVector3 halfExtent = boxShape->GetHalfExtents();
					glScaled(2*halfExtent[0], 2*halfExtent[1], 2*halfExtent[2]);
					glutSolidCube(1.0);
					useWireframeFallback = false;
					break;
				}
			case TRIANGLE_SHAPE_PROXYTYPE:
			case TETRAHEDRAL_SHAPE_PROXYTYPE:
				{
					const BU_Simplex1to4* tetra = static_cast<const BU_Simplex1to4*>(shape);
					//todo:	
					useWireframeFallback = false;
					break;
				}
			case CONVEX_HULL_SHAPE_PROXYTYPE:
				break;
			case SPHERE_SHAPE_PROXYTYPE:
				{
					const SphereShape* sphereShape = static_cast<const SphereShape*>(shape);
					float radius = sphereShape->GetMargin();//radius doesn't include the margin, so draw with margin
					glutSolidSphere(radius,10,10);
					useWireframeFallback = false;
					break;
				}
			case MULTI_SPHERE_SHAPE_PROXYTYPE:
			case CONE_SHAPE_PROXYTYPE:
				{
					const ConeShape* coneShape = static_cast<const ConeShape*>(shape);
					float radius = coneShape->GetRadius();//+coneShape->GetMargin();
					float height = coneShape->GetHeight();//+coneShape->GetMargin();
					//glRotatef(-90.0, 1.0, 0.0, 0.0);
					glTranslatef(0.0, 0.0, -0.5*height);
					glutSolidCone(radius,height,10,10);
					useWireframeFallback = false;
					break;

				}
			case CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE:
				{
					useWireframeFallback = false;
					break;
				}

			case CONVEX_SHAPE_PROXYTYPE:
			case CYLINDER_SHAPE_PROXYTYPE:
				{
					const CylinderShape* cylinder = static_cast<const CylinderShape*>(shape);
					int upAxis = cylinder->GetUpAxis();
					
					GLUquadricObj *quadObj = gluNewQuadric();
					float radius = cylinder->GetRadius();
					float halfHeight = cylinder->GetHalfExtents()[upAxis];

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
					
					//The gluCylinder subroutine draws a cylinder that is oriented along the z axis. 
					//The base of the cylinder is placed at z = 0; the top of the cylinder is placed at z=height. 
					//Like a sphere, the cylinder is subdivided around the z axis into slices and along the z axis into stacks.
					
					gluQuadricDrawStyle(quadObj, (GLenum)GLU_FILL);
					gluQuadricNormals(quadObj, (GLenum)GLU_SMOOTH);
					
					
					gluCylinder(quadObj, radius, radius, 2.f*halfHeight, 15, 10);
					glPopMatrix();
					glEndList();

					break;
				}
			default:
				{
				}

			};

		}
		

		

		if (useWireframeFallback)
		{
			/// for polyhedral shapes
			if (shape->IsPolyhedral())
			{
				PolyhedralConvexShape* polyshape = (PolyhedralConvexShape*) shape;
				
				
				glBegin(GL_LINES);


				int i;
				for (i=0;i<polyshape->GetNumEdges();i++)
				{
					SimdPoint3 a,b;
					polyshape->GetEdge(i,a,b);

					glVertex3f(a.getX(),a.getY(),a.getZ());
					glVertex3f(b.getX(),b.getY(),b.getZ());


				}
				glEnd();

				
				if (debugMode==IDebugDraw::DBG_DrawFeaturesText)
				{
					glRasterPos3f(0.0,  0.0,  0.0);
					BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),polyshape->GetExtraDebugInfo());

					glColor3f(1.f, 1.f, 1.f);
					for (i=0;i<polyshape->GetNumVertices();i++)
					{
						SimdPoint3 vtx;
						polyshape->GetVertex(i,vtx);
						glRasterPos3f(vtx.x(),  vtx.y(),  vtx.z());
						char buf[12];
						sprintf(buf," %d",i);
						BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
					}

					for (i=0;i<polyshape->GetNumPlanes();i++)
					{
						SimdVector3 normal;
						SimdPoint3 vtx;
						polyshape->GetPlane(normal,vtx,i);
						SimdScalar d = vtx.dot(normal);

						glRasterPos3f(normal.x()*d,  normal.y()*d, normal.z()*d);
						char buf[12];
						sprintf(buf," plane %d",i);
						BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
						
					}
				}

				
			}
		}

		if (shape->GetShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE)
		{
			TriangleMeshShape* concaveMesh = (TriangleMeshShape*) shape;
			//SimdVector3 aabbMax(1e30f,1e30f,1e30f);
			//SimdVector3 aabbMax(100,100,100);//1e30f,1e30f,1e30f);
			
			extern float eye[3];
			SimdVector3 aabbMax(eye[0]+100,eye[1]+100,eye[2]+100);//1e30f,1e30f,1e30f);
			SimdVector3 aabbMin(eye[0]-100,eye[1]-100,eye[2]-100);//1e30f,1e30f,1e30f);

			GlDrawcallback drawCallback;

			concaveMesh->ProcessAllTriangles(&drawCallback,aabbMin,aabbMax);


		}

		if (shape->GetShapeType() == CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE)
		{
			ConvexTriangleMeshShape* convexMesh = (ConvexTriangleMeshShape*) shape;
			
			extern float eye[3];
			SimdVector3 aabbMax(eye[0]+100,eye[1]+100,eye[2]+100);
			SimdVector3 aabbMin(eye[0]-100,eye[1]-100,eye[2]-100);
			TriangleGlDrawcallback drawCallback;
			convexMesh->GetStridingMesh()->InternalProcessAllTriangles(&drawCallback,aabbMin,aabbMax);

		}
		
		/*glDisable(GL_DEPTH_BUFFER_BIT);
		if (debugMode==IDebugDraw::DBG_DrawText)
		{
			BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),shape->GetName());
		}

		if (debugMode==IDebugDraw::DBG_DrawFeaturesText)
		{
			BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),shape->GetExtraDebugInfo());
		}
		glEnable(GL_DEPTH_BUFFER_BIT);
		*/

	//	glPopMatrix();
	}
    glPopMatrix();
	
}
