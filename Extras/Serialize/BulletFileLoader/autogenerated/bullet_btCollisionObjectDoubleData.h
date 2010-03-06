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
#ifndef __BULLET_BTCOLLISIONOBJECTDOUBLEDATA__H__
#define __BULLET_BTCOLLISIONOBJECTDOUBLEDATA__H__


// -------------------------------------------------- //
#include "bullet_Common.h"
#include "bullet_btTransformDoubleData.h"
#include "bullet_btVector3DoubleData.h"

namespace Bullet {


    // ---------------------------------------------- //
    class btCollisionObjectDoubleData
    {
    public:
        void *m_broadphaseHandle;
        void *m_collisionShape;
        btCollisionShapeData *m_rootCollisionShape;
        char *m_name;
        btTransformDoubleData m_worldTransform;
        btTransformDoubleData m_interpolationWorldTransform;
        btVector3DoubleData m_interpolationLinearVelocity;
        btVector3DoubleData m_interpolationAngularVelocity;
        btVector3DoubleData m_anisotropicFriction;
        double m_contactProcessingThreshold;
        double m_deactivationTime;
        double m_friction;
        double m_restitution;
        double m_hitFraction;
        double m_ccdSweptSphereRadius;
        double m_ccdMotionThreshold;
        int m_hasAnisotropicFriction;
        int m_collisionFlags;
        int m_islandTag1;
        int m_companionId;
        int m_activationState1;
        int m_internalType;
        int m_checkCollideWith;
        char m_padding[4];
    };
}


#endif//__BULLET_BTCOLLISIONOBJECTDOUBLEDATA__H__
