
#include "CollisionShapes/SphereShape.h"
#include "CollisionShapes/CylinderShape.h"
#include "CollisionShapes/MultiSphereShape.h"//capped cylinder is convex hull of two spheres
#include "CollisionShapes/ConvexTriangleCallback.h"
#include "CollisionShapes/BoxShape.h"

//need access to internals to convert...
#include <../ode/src/collision_kernel.h>


CollisionShape*	CreateShapeFromGeom(dGeomID geom)
{
	CollisionShape* shape = 0;

	switch (geom->type)
	{
	case dPlaneClass:
		break;
	case dBoxClass:
		{
			dVector3 size;
			dGeomBoxGetLengths (geom,size);
			SimdVector3 halfExtents(0.5f*size[0],0.5f*size[1],0.5f*size[2]);
			shape = new BoxShape(halfExtents);
			break;
		}
		case dSphereClass:
		{
			dVector3 size;
			shape = new SphereShape(dGeomSphereGetRadius(geom));
			break;
		}
		case dCylinderClass:
		{
			dVector3 size;
			dGeomBoxGetLengths (geom,size);
			SimdVector3 boxHalfExtents(size[0],size[0],size[1]);
			shape = new CylinderShapeZ(boxHalfExtents);
			break;
		}
		case dCCylinderClass:
		{
			dReal radius,length;
			dGeomCCylinderGetParams (geom, &radius, &length);
			SimdVector3 boxHalfExtents(radius,radius,0.5f*length);
			int numspheres = 2;
			SimdVector3 centers[2]={ SimdVector3(0,0,0.5f*length),SimdVector3(0,0,-0.5f*length)};
			SimdScalar radi[2]={radius,radius};
			shape = new MultiSphereShape(boxHalfExtents,centers,radi,numspheres);
			break;
		} 
		/*case dTriMeshClass:
			{
				dxTriMesh* trimesh = (dxTriMesh*) geom;
				shape = 0;//new BVHTrimeshShape(trimesh);
				break;
			}
*/


	default:
		{
		}
	};

	
	return shape;

}