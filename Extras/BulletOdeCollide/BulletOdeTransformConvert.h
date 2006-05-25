#ifndef BULLET_ODE_TRANSFORM_CONVERT_H
#define  BULLET_ODE_TRANSFORM_CONVERT_H

#include "SimdTransform.h"
#include <ode/common.h>

SimdTransform	GetTransformFromGeom(dGeomID geom);
SimdTransform	GetTransformFromBody(dBodyID body);

#endif //BULLET_ODE_TRANSFORM_CONVERT_H