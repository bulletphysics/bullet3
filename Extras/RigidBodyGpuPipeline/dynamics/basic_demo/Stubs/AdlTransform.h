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


#ifndef _ADL_TRANSFORM_H
#define _ADL_TRANSFORM_H

#include "AdlMath.h"
#include "AdlQuaternion.h"
#include "AdlMatrix3x3.h"

struct Transform
{
	float4 m_translation;
	Matrix3x3 m_rotation;
};

Transform trSetTransform(const float4& translation, const Quaternion& quat)
{
	Transform tr;
	tr.m_translation = translation;
	tr.m_rotation = qtGetRotationMatrix( quat );
	return tr;
}

Transform trInvert( const Transform& tr )
{
	Transform ans;
	ans.m_rotation = mtTranspose( tr.m_rotation );
	ans.m_translation = mtMul1( ans.m_rotation, -tr.m_translation );
	return ans;
}

Transform trMul(const Transform& trA, const Transform& trB)
{
	Transform ans; 
	ans.m_rotation = mtMul( trA.m_rotation, trB.m_rotation );
	ans.m_translation = mtMul1( trA.m_rotation, trB.m_translation ) + trA.m_translation;
	return ans;
}

float4 trMul1(const Transform& tr, const float4& p)
{
	return mtMul1( tr.m_rotation, p ) + tr.m_translation;
}


#endif //_ADL_TRANSFORM_H

