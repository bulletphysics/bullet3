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
#ifndef __BULLET_BTCOLLISIONOBJECTDATA__H__
#define __BULLET_BTCOLLISIONOBJECTDATA__H__


// -------------------------------------------------- //
#include "bullet_Common.h"
#include "bullet_btTransformData.h"
#include "bullet_btVector3Data.h"

namespace Bullet {


    // ---------------------------------------------- //
    class btCollisionObjectData
    {
    public:
        btTransformData m_worldTransform;
        btTransformData m_interpolationWorldTransform;
        btVector3Data m_interpolationLinearVelocity;
        btVector3Data m_interpolationAngularVelocity;
        btVector3Data m_anisotropicFriction;
        int m_hasAnisotropicFriction;
        btScalar m_contactProcessingThreshold;
        void *m_broadphaseHandle;
        void *m_collisionShape;
        btCollisionShapeData *m_rootCollisionShape;
        int m_collisionFlags;
        int m_islandTag1;
        int m_companionId;
        int m_activationState1;
        btScalar m_deactivationTime;
        btScalar m_friction;
        btScalar m_restitution;
        int m_internalType;
        void *m_userObjectPointer;
        btScalar m_hitFraction;
        btScalar m_ccdSweptSphereRadius;
        btScalar m_ccdMotionThreshold;
        int m_checkCollideWith;
    };
}


#endif//__BULLET_BTCOLLISIONOBJECTDATA__H__
