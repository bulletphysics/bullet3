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


#ifndef ADL_CONSTRAINT4_H
#define ADL_CONSTRAINT4_H



struct Constraint4
		{
			_MEM_ALIGNED_ALLOCATOR16;

			float4 m_linear;
			float4 m_worldPos[4];
			float4 m_center;	//	friction
			float m_jacCoeffInv[4];
			float m_b[4];
			float m_appliedRambdaDt[4];

			float m_fJacCoeffInv[2];	//	friction
			float m_fAppliedRambdaDt[2];	//	friction

			u32 m_bodyA;
			u32 m_bodyB;

			u32 m_batchIdx;
			u32 m_paddings[1];

			__inline
			void setFrictionCoeff(float value) { m_linear.w = value; }
			__inline
			float getFrictionCoeff() const { return m_linear.w; }
		};

#endif //ADL_CONSTRAINT4_H
		