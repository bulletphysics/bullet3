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




#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionShapes/btConvexInternalShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"

#define MAX_NUM_SPU_CONVEX_POINTS 128

struct	SpuConvexPolyhedronVertexData
{
	void*	gSpuConvexShapePtr0;
	void*	gSpuConvexShapePtr1;
	btPoint3* gConvexPoints0;
	btPoint3* gConvexPoints1;
	int gNumConvexPoints0;
	int gNumConvexPoints1;
	ATTRIBUTE_ALIGNED16(btPoint3	g_convexPointBuffer0[MAX_NUM_SPU_CONVEX_POINTS]);
	ATTRIBUTE_ALIGNED16(btPoint3	g_convexPointBuffer1[MAX_NUM_SPU_CONVEX_POINTS]);

};


inline btPoint3 localGetSupportingVertexWithoutMargin(int shapeType, void* shape, btVector3 localDir,struct	SpuConvexPolyhedronVertexData* convexVertexData)//, int *featureIndex)
{
    switch (shapeType)
    {
    case SPHERE_SHAPE_PROXYTYPE:
        {
            return btPoint3(0,0,0);
        }
	case BOX_SHAPE_PROXYTYPE:
		{
//			spu_printf("SPU: getSupport BOX_SHAPE_PROXYTYPE\n");
			btConvexInternalShape* convexShape = (btConvexInternalShape*)shape;
			btVector3 halfExtents = convexShape->getImplicitShapeDimensions();
			float margin = convexShape->getMarginNV();
			halfExtents -= btVector3(margin,margin,margin);
			return btPoint3(
				localDir.getX() < 0.0f ? -halfExtents.x() : halfExtents.x(),
							localDir.getY() < 0.0f ? -halfExtents.y() : halfExtents.y(),
							localDir.getZ() < 0.0f ? -halfExtents.z() : halfExtents.z());
		}

	case TRIANGLE_SHAPE_PROXYTYPE:
		{

			btVector3 dir(localDir.getX(),localDir.getY(),localDir.getZ());
			btVector3* vertices = (btVector3*)shape;
			btVector3 dots(dir.dot(vertices[0]), dir.dot(vertices[1]), dir.dot(vertices[2]));
	  		btVector3 sup = vertices[dots.maxAxis()];
			return btPoint3(sup.getX(),sup.getY(),sup.getZ());
			break;
		}

	case CYLINDER_SHAPE_PROXYTYPE:
		{
			btCylinderShape* cylShape = (btCylinderShape*)shape;

			//mapping of halfextents/dimension onto radius/height depends on how cylinder local orientation is (upAxis)

			btVector3 halfExtents = cylShape->getImplicitShapeDimensions();
			btVector3 v(localDir.getX(),localDir.getY(),localDir.getZ());
			
			int cylinderUpAxis = cylShape->getUpAxis();
			int XX(1),YY(0),ZZ(2);

			switch (cylinderUpAxis)
			{
			case 0:
				{
					XX = 1;
					YY = 0;
					ZZ = 2;
					break;
				}
			case 1:
				{
					XX = 0;
					YY = 1;
					ZZ = 2;
				break;
				}
			case 2:
				{
					XX = 0;
					YY = 2;
					ZZ = 1;
					break;
				}
			default:
				btAssert(0);
				//printf("SPU:localGetSupportingVertexWithoutMargin unknown Cylinder up-axis\n");
			};

			btScalar radius = halfExtents[XX];
			btScalar halfHeight = halfExtents[cylinderUpAxis];

			btVector3 tmp;
			btScalar d ;

			btScalar s = btSqrt(v[XX] * v[XX] + v[ZZ] * v[ZZ]);
			if (s != btScalar(0.0))
			{
				d = radius / s;  
				tmp[XX] = v[XX] * d;
				tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
				tmp[ZZ] = v[ZZ] * d;
				return btPoint3(tmp.getX(),tmp.getY(),tmp.getZ());
			}
			else
			{
				tmp[XX] = radius;
				tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
				tmp[ZZ] = btScalar(0.0);
				return btPoint3(tmp.getX(),tmp.getY(),tmp.getZ());
			}
		}

	case CAPSULE_SHAPE_PROXYTYPE:
	{
		//spu_printf("SPU: todo: getSupport CAPSULE_SHAPE_PROXYTYPE\n");
		btVector3 vec0(localDir.getX(),localDir.getY(),localDir.getZ());

		btConvexInternalShape* cnvxShape = (btConvexInternalShape*)shape;
		btVector3 halfExtents = cnvxShape->getImplicitShapeDimensions();
		btScalar halfHeight = halfExtents.getY();
		btScalar radius = halfExtents.getX();
		btVector3 supVec(0,0,0);

		btScalar maxDot(btScalar(-1e30));

		btVector3 vec = vec0;
		btScalar lenSqr = vec.length2();
		if (lenSqr < btScalar(0.0001))
		{
			vec.setValue(1,0,0);
		} else
		{
			btScalar rlen = btScalar(1.) / btSqrt(lenSqr );
			vec *= rlen;
		}
		btVector3 vtx;
		btScalar newDot;
		{
			btVector3 pos(0,halfHeight,0);
			vtx = pos +vec*(radius);
			newDot = vec.dot(vtx);
			if (newDot > maxDot)
			{
				maxDot = newDot;
				supVec = vtx;
			}
		}
		{
			btVector3 pos(0,-halfHeight,0);
			vtx = pos +vec*(radius);
			newDot = vec.dot(vtx);
			if (newDot > maxDot)
			{
				maxDot = newDot;
				supVec = vtx;
			}
		}
		return btPoint3(supVec.getX(),supVec.getY(),supVec.getZ());
		break;
	};

	case CONVEX_HULL_SHAPE_PROXYTYPE:
		{
			//spu_printf("SPU: todo: getSupport CONVEX_HULL_SHAPE_PROXYTYPE\n");

		

			btPoint3* points = 0;
			int numPoints = 0;
			if (shape==convexVertexData->gSpuConvexShapePtr0)
			{
				points = convexVertexData->gConvexPoints0;
				numPoints = convexVertexData->gNumConvexPoints0;
			}
			if (shape == convexVertexData->gSpuConvexShapePtr1)
			{
				points = convexVertexData->gConvexPoints1;
				numPoints = convexVertexData->gNumConvexPoints1;
			}

		//	spu_printf("numPoints = %d\n",numPoints);

			btVector3 supVec(btScalar(0.),btScalar(0.),btScalar(0.));
			btScalar newDot,maxDot = btScalar(-1e30);

			btVector3 vec0(localDir.getX(),localDir.getY(),localDir.getZ());
			btVector3 vec = vec0;
			btScalar lenSqr = vec.length2();
			if (lenSqr < btScalar(0.0001))
			{
				vec.setValue(1,0,0);
			} else
			{
				btScalar rlen = btScalar(1.) / btSqrt(lenSqr );
				vec *= rlen;
			}


			for (int i=0;i<numPoints;i++)
			{
				btPoint3 vtx = points[i];// * m_localScaling;

				newDot = vec.dot(vtx);
				if (newDot > maxDot)
				{
					maxDot = newDot;
					supVec = vtx;
				}
			}
			return btPoint3(supVec.getX(),supVec.getY(),supVec.getZ());

			break;
		};

    default:

		//spu_printf("SPU:(type %i) missing support function\n",shapeType);

		
#if __ASSERT
        spu_printf("localGetSupportingVertexWithoutMargin() - Unsupported bound type: %d.\n", shapeType);
#endif // __ASSERT
        return btPoint3(0.f, 0.f, 0.f);
    }
}


