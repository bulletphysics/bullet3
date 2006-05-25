#ifndef GEOM_TO_SHAPE_H
#define GEOM_TO_SHAPE_H

class CollisionShape;

//need internals to handle dGeomID
#include <../ode/src/collision_kernel.h>

CollisionShape*	CreateShapeFromGeom(dGeomID geom);

#endif //