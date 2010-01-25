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
#ifndef __BULLET_BTRIGIDBODYDOUBLEDATA__H__
#define __BULLET_BTRIGIDBODYDOUBLEDATA__H__


// -------------------------------------------------- //
#include "bullet_Common.h"
#include "bullet_btCollisionObjectDoubleData.h"
#include "bullet_btMatrix3x3DoubleData.h"
#include "bullet_btVector3DoubleData.h"

namespace Bullet {


    // ---------------------------------------------- //
    class btRigidBodyDoubleData
    {
    public:
        btCollisionObjectDoubleData m_collisionObjectData;
        btMatrix3x3DoubleData m_invInertiaTensorWorld;
        btVector3DoubleData m_linearVelocity;
        btVector3DoubleData m_angularVelocity;
        btVector3DoubleData m_angularFactor;
        btVector3DoubleData m_linearFactor;
        btVector3DoubleData m_gravity;
        btVector3DoubleData m_gravity_acceleration;
        btVector3DoubleData m_invInertiaLocal;
        btVector3DoubleData m_totalForce;
        btVector3DoubleData m_totalTorque;
        double m_inverseMass;
        double m_linearDamping;
        double m_angularDamping;
        double m_additionalDampingFactor;
        double m_additionalLinearDampingThresholdSqr;
        double m_additionalAngularDampingThresholdSqr;
        double m_additionalAngularDampingFactor;
        double m_linearSleepingThreshold;
        double m_angularSleepingThreshold;
        int m_additionalDamping;
        char m_padding[4];
    };
}


#endif//__BULLET_BTRIGIDBODYDOUBLEDATA__H__
