#include "BulletOdeTransformConvert.h"
#include "../ode/src/collision_kernel.h"

SimdTransform	GetTransformFromGeom(dGeomID geom)
{
	SimdTransform trans;
	trans.setIdentity();


	const dReal* pos = dGeomGetPosition (geom);// pointer to object's position vector
	const dReal* rot = dGeomGetRotation (geom);;		// pointer to object's rotation matrix, 4*3 format!
	
	SimdMatrix3x3 orn(rot[0],rot[1],rot[2],
		rot[4],rot[5],rot[6],
		rot[8],rot[9],rot[10]);

	trans.setOrigin(SimdVector3(pos[0],pos[1],pos[2]));
	trans.setBasis(orn);

  return trans;
}

SimdTransform	GetTransformFromBody(dBodyID body)
{
	SimdTransform trans;

	const dReal* pos = body->pos;
	const dReal* rot = body->R;

	SimdMatrix3x3 orn(rot[0],rot[1],rot[2],
		rot[4],rot[5],rot[6],
		rot[8],rot[9],rot[10]);

	trans.setOrigin(SimdVector3(pos[0],pos[1],pos[2]));
	trans.setBasis(orn);

  return trans;

}
