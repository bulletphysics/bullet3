#ifndef B3_QUAT_H
#define B3_QUAT_H

#include "Bullet3Common/shared/b3PlatformDefinitions.h"

#ifdef __cplusplus
	#include "Bullet3Common/b3Quaternion.h"
	#define b3Quat b3Quaternion
#else
	typedef float4	b3Quat;
#endif 

#endif //B3_QUAT_H
