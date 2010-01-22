/* Copyright (C) 2006-2009 Erwin Coumans & Charlie C
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
*
* 1. The origin of this software must not be misrepresented; you must not
*    claim that you wrote the original software. If you use this software
*    in a product, an acknowledgment in the product documentation would be
*    appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
*    misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/
// Auto generated from makesdna dna.c
#ifndef __BULLETCOMMON_H__
#define __BULLETCOMMON_H__

// put an empty struct in the case
typedef struct bInvalidHandle {
	int unused;
}bInvalidHandle;

namespace Bullet {
    class PointerArray;
    class btPhysicsSystem;
    class ListBase;
    class btVector3Data;
    class btMatrix3x3Data;
    class btTransformData;
    class btCollisionShapeData;
    class btConvexInternalShapeData;
    class btCollisionObjectData;
    class btRigidBodyData;
}
#endif//__BULLETCOMMON_H__