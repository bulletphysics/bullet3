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
#ifndef __BULLET_BTRIGIDBODYDATA__H__
#define __BULLET_BTRIGIDBODYDATA__H__


// -------------------------------------------------- //
#include "bullet_Common.h"
#include "bullet_btCollisionObjectData.h"
#include "bullet_btMatrix3x3Data.h"
#include "bullet_btVector3Data.h"

namespace Bullet {


    // ---------------------------------------------- //
    class btRigidBodyData
    {
    public:
        btCollisionObjectData m_collisionObjectData;
        btMatrix3x3Data m_invInertiaTensorWorld;
        btVector3Data m_linearVelocity;
        btVector3Data m_angularVelocity;
        btScalar m_inverseMass;
        btVector3Data m_angularFactor;
        btVector3Data m_linearFactor;
        btVector3Data m_gravity;
        btVector3Data m_gravity_acceleration;
        btVector3Data m_invInertiaLocal;
        btVector3Data m_totalForce;
        btVector3Data m_totalTorque;
        btScalar m_linearDamping;
        btScalar m_angularDamping;
        int m_additionalDamping;
        btScalar m_additionalDampingFactor;
        btScalar m_additionalLinearDampingThresholdSqr;
        btScalar m_additionalAngularDampingThresholdSqr;
        btScalar m_additionalAngularDampingFactor;
        btScalar m_linearSleepingThreshold;
        btScalar m_angularSleepingThreshold;
    };
}


#endif//__BULLET_BTRIGIDBODYDATA__H__
