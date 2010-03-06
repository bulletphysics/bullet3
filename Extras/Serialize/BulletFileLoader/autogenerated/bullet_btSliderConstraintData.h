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
#ifndef __BULLET_BTSLIDERCONSTRAINTDATA__H__
#define __BULLET_BTSLIDERCONSTRAINTDATA__H__


// -------------------------------------------------- //
#include "bullet_Common.h"
#include "bullet_btTransformFloatData.h"
#include "bullet_btTypedConstraintData.h"

namespace Bullet {


    // ---------------------------------------------- //
    class btSliderConstraintData
    {
    public:
        btTypedConstraintData m_typeConstraintData;
        btTransformFloatData m_rbAFrame;
        btTransformFloatData m_rbBFrame;
        float m_linearUpperLimit;
        float m_linearLowerLimit;
        float m_angularUpperLimit;
        float m_angularLowerLimit;
        int m_useLinearReferenceFrameA;
        int m_useOffsetForConstraintFrame;
    };
}


#endif//__BULLET_BTSLIDERCONSTRAINTDATA__H__
