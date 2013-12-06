/*
CDTestFramework http://codercorner.com
Copyright (c) 2007-2008 Pierre Terdiman,  pierre@codercorner.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#ifndef ICEHELPERS_H
#define ICEHELPERS_H

	void RotX(Matrix3x3& m, float angle);
	void RotY(Matrix3x3& m, float angle);
	void RotZ(Matrix3x3& m, float angle);

	udword	RayCapsuleOverlap(const Point& origin, const Point& dir, const LSS& capsule, float s[2]);
	bool	SegmentSphere(const Point& origin, const Point& dir, float length, const Point& center, float radius, float& dist, Point& hit_pos);
	bool	RayAABB2(const Point& min, const Point& max, const Point& origin, const Point& dir, Point& coord);

	inline_ bool RayOBB(const Point& origin, const Point& dir, const OBB& box, float& dist, Point& hit_pos)
	{
		Point LocalOrigin = box.mRot * (origin - box.mCenter);
		Point LocalDir = box.mRot * dir;

		Point LocalImpact;
		if(RayAABB2(-box.mExtents, box.mExtents, LocalOrigin, LocalDir, LocalImpact))
		{
			dist = LocalImpact.Distance(LocalOrigin);
			hit_pos = LocalImpact * box.mRot + box.mCenter;
			return true;
		}
		return false;
	}

#endif	// ICEHELPERS_H
