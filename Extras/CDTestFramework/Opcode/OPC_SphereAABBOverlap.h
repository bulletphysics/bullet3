/*
 *	OPCODE - Optimized Collision Detection
 * http://www.codercorner.com/Opcode.htm
 * 
 * Copyright (c) 2001-2008 Pierre Terdiman,  pierre@codercorner.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Sphere-AABB overlap test, based on Jim Arvo's code.
 *	\param		center		[in] box center
 *	\param		extents		[in] box extents
 *	\return		TRUE on overlap
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline_ BOOL SphereCollider::SphereAABBOverlap(const Point& center, const Point& extents)
{ 
	// Stats
	mNbVolumeBVTests++;

	float d = 0.0f;

	//find the square of the distance
	//from the sphere to the box
#ifdef OLDIES
	for(udword i=0;i<3;i++)
	{
		float tmp = mCenter[i] - center[i];
		float s = tmp + extents[i];

		if(s<0.0f)	d += s*s;
		else
		{
			s = tmp - extents[i];
			if(s>0.0f)	d += s*s;
		}
	}
#endif

//#ifdef NEW_TEST

//	float tmp = mCenter.x - center.x;
//	float s = tmp + extents.x;

	float tmp,s;

	tmp = mCenter.x - center.x;
	s = tmp + extents.x;

	if(s<0.0f)
	{
		d += s*s;
		if(d>mRadius2)	return FALSE;
	}
	else
	{
		s = tmp - extents.x;
		if(s>0.0f)
		{
			d += s*s;
			if(d>mRadius2)	return FALSE;
		}
	}

	tmp = mCenter.y - center.y;
	s = tmp + extents.y;

	if(s<0.0f)
	{
		d += s*s;
		if(d>mRadius2)	return FALSE;
	}
	else
	{
		s = tmp - extents.y;
		if(s>0.0f)
		{
			d += s*s;
			if(d>mRadius2)	return FALSE;
		}
	}

	tmp = mCenter.z - center.z;
	s = tmp + extents.z;

	if(s<0.0f)
	{
		d += s*s;
		if(d>mRadius2)	return FALSE;
	}
	else
	{
		s = tmp - extents.z;
		if(s>0.0f)
		{
			d += s*s;
			if(d>mRadius2)	return FALSE;
		}
	}
//#endif

#ifdef OLDIES
//	Point Min = center - extents;
//	Point Max = center + extents;

	float d = 0.0f;

	//find the square of the distance
	//from the sphere to the box
	for(udword i=0;i<3;i++)
	{
float Min = center[i] - extents[i];

//		if(mCenter[i]<Min[i])
		if(mCenter[i]<Min)
		{
//			float s = mCenter[i] - Min[i];
			float s = mCenter[i] - Min;
			d += s*s;
		}
		else
		{
float Max = center[i] + extents[i];

//			if(mCenter[i]>Max[i])
			if(mCenter[i]>Max)
			{
				float s = mCenter[i] - Max;
				d += s*s;
			}
		}
	}
#endif
	return d <= mRadius2;
}
