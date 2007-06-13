/* [SCE CONFIDENTIAL DOCUMENT]
 * PLAYSTATION(R)3 SPU Optimized Bullet Physics Library (http://bulletphysics.com)
 *                Copyright (C) 2007 Sony Computer Entertainment Inc.
 *                                                All Rights Reserved.
 */




#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"


struct	SpuConvexPolyhedronVertexData
{
	void*	gSpuConvexShapePtr0;
	void*	gSpuConvexShapePtr1;
	btPoint3* gConvexPoints0;
	btPoint3* gConvexPoints1;
	int gNumConvexPoints0;
	int gNumConvexPoints1;
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
			btConvexShape* convexShape = (btConvexShape*)shape;
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

		btConvexShape* cnvxShape = (btConvexShape*)shape;
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


