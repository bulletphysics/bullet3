#ifndef B3_FLOAT4_H
#define B3_FLOAT4_H

#include "Bullet3Common/shared/b3PlatformDefinitions.h"

#ifdef __cplusplus
	#include "Bullet3Common/b3Vector3.h"
	#define b3Float4 b3Vector3
#else
	typedef float4	b3Float4;
#endif 

#endif //B3_FLOAT4_H
