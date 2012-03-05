/*
Copyright (c) 2012 Advanced Micro Devices, Inc.  

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Takahiro Harada


#ifndef ADL_RIGID_BODY_H
#define ADL_RIGID_BODY_H

#include "AdlQuaternion.h"

class RigidBodyBase
{
	public:

		_MEM_CLASSALIGN16
		struct Body
		{
			_MEM_ALIGNED_ALLOCATOR16;

			float4 m_pos;
			Quaternion m_quat;
			float4 m_linVel;
			float4 m_angVel;

			u32 m_shapeIdx;
			u32 m_shapeType;

			float m_invMass;
			float m_restituitionCoeff;
			float m_frictionCoeff;
			
		};

		struct Inertia
		{
/*			u16 m_shapeType;
			u16 m_shapeIdx;
			float m_restituitionCoeff;
			float m_frictionCoeff;
			int m_padding;
*/
			Matrix3x3 m_invInertia;
			Matrix3x3 m_initInvInertia;
		};
};

#endif// ADL_RIGID_BODY_H

