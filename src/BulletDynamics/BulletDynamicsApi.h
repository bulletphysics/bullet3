#ifndef BULLETDYNAMICSAPI_H
#define BULLETDYNAMICSAPI_H

#include "Bullet3Common/b3Api.h"

#ifdef BulletDynamics_EXPORTS
#define BULLETDYNAMICS_API BULLET_EXPORT
#else
#define BULLETDYNAMICS_API BULLET_IMPORT
#endif

#endif
