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


#ifndef ADL_CONTACT4_H
#define ADL_CONTACT4_H

#ifdef CL_PLATFORM_AMD
#include "AdlConstraint4.h"
#include "Adl/Adl.h"

typedef adl::Buffer<Constraint4>* SolverData;
#else
typedef void* SolverData;
#endif

typedef void* ShapeDataType;


struct Contact4
{
	_MEM_ALIGNED_ALLOCATOR16;

	float4 m_worldPos[4];
	float4 m_worldNormal;
//	float m_restituitionCoeff;
//	float m_frictionCoeff;
	u16 m_restituitionCoeffCmp;
	u16 m_frictionCoeffCmp;
	int m_batchIdx;

	u32 m_bodyAPtr;
	u32 m_bodyBPtr;

	//	todo. make it safer
	int& getBatchIdx() { return m_batchIdx; }
	float getRestituitionCoeff() const { return ((float)m_restituitionCoeffCmp/(float)0xffff); }
	void setRestituitionCoeff( float c ) { ADLASSERT( c >= 0.f && c <= 1.f ); m_restituitionCoeffCmp = (u16)(c*0xffff); }
	float getFrictionCoeff() const { return ((float)m_frictionCoeffCmp/(float)0xffff); }
	void setFrictionCoeff( float c ) { ADLASSERT( c >= 0.f && c <= 1.f ); m_frictionCoeffCmp = (u16)(c*0xffff); }

	float& getNPoints() { return m_worldNormal.w; }
	float getNPoints() const { return m_worldNormal.w; }

	float getPenetration(int idx) const { return m_worldPos[idx].w; }

	bool isInvalid() const { return ((u32)m_bodyAPtr+(u32)m_bodyBPtr) == 0; }
};

struct ContactPoint4
		{
			float4 m_worldPos[4];
			union
			{
				float4 m_worldNormal;

				struct Data
				{
					int m_padding[3];
					float m_nPoints;	//	for cl
				}m_data;

			};
			float m_restituitionCoeff;
			float m_frictionCoeff;
//			int m_nPoints;
//			int m_padding0;

			void* m_bodyAPtr;
			void* m_bodyBPtr;
//			int m_padding1;
//			int m_padding2;

			float& getNPoints() { return m_data.m_nPoints; }
			float getNPoints() const { return m_data.m_nPoints; }

			float getPenetration(int idx) const { return m_worldPos[idx].w; }

//			__inline
//			void load(int idx, const ContactPoint& src);
//			__inline
//			void store(int idx, ContactPoint& dst) const;

			bool isInvalid() const { return ((u32)m_bodyAPtr+(u32)m_bodyBPtr) == 0; }

		};


#endif //ADL_CONTACT4_H

