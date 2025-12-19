/*
Bullet Continuous Collision Detection and Physics Library
Precompiled Header for BulletCollision
*/

#ifndef BULLET_COLLISION_PCH_H
#define BULLET_COLLISION_PCH_H

// Standard Library Headers - Most expensive
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <cmath>
#include <cassert>

// Windows headers (if on Windows)
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

// Common Bullet LinearMath headers
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btDefaultMotionState.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btIDebugDraw.h"
#include "LinearMath/btSerializer.h"
#include "LinearMath/btHashMap.h"

#endif // BULLET_COLLISION_PCH_H
