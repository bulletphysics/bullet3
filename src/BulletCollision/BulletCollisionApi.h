#ifndef BULLETCOLLISIONAPI_H
#define BULLETCOLLISIONAPI_H

#include "Bullet3Common/b3Api.h"

#ifdef BulletCollision_EXPORTS
#define BULLETCOLLISION_API BULLET_EXPORT
#else
#define BULLETCOLLISION_API BULLET_IMPORT
#endif

#endif
