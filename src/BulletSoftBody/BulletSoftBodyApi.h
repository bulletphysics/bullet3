#ifndef BULLET3SOFTBODYAPI_H
#define BULLET3SOFTBODYAPI_H

#include "Bullet3Common/b3Api.h"

#ifdef BulletSoftBody_EXPORTS
#define BULLETSOFTBODY_API BULLET_EXPORT
#else
#define BULLETSOFTBODY_API BULLET_IMPORT
#endif

#endif
