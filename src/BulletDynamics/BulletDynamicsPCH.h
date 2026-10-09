/*
Bullet Continuous Collision Detection and Physics Library
Precompiled Header for BulletDynamics
*/

#ifndef BULLET_DYNAMICS_PCH_H
#define BULLET_DYNAMICS_PCH_H

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

// Common Bullet headers
#include "btBulletCollisionCommon.h"
#include "LinearMath/btHashMap.h"

#endif // BULLET_DYNAMICS_PCH_H
