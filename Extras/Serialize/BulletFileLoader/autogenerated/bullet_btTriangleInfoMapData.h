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
#ifndef __BULLET_BTTRIANGLEINFOMAPDATA__H__
#define __BULLET_BTTRIANGLEINFOMAPDATA__H__


// -------------------------------------------------- //
#include "bullet_Common.h"

namespace Bullet {


    // ---------------------------------------------- //
    class btTriangleInfoMapData
    {
    public:
        int *m_hashTablePtr;
        int *m_nextPtr;
        btTriangleInfoData *m_valueArrayPtr;
        int *m_keyArrayPtr;
        float m_convexEpsilon;
        float m_planarEpsilon;
        float m_equalVertexThreshold;
        float m_edgeDistanceThreshold;
        float m_zeroAreaThreshold;
        int m_nextSize;
        int m_hashTableSize;
        int m_numValues;
        int m_numKeys;
        char m_padding[4];
    };
}


#endif//__BULLET_BTTRIANGLEINFOMAPDATA__H__
