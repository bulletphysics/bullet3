/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef B_SCREWING_H
#define B_SCREWING_H


#include <LinearMath/btVector3.h>
#include <LinearMath/btPoint3.h>
#include <LinearMath/btTransform.h>


#define SCREWEPSILON 0.00001f

///BU_Screwing implements screwing motion interpolation.
class BU_Screwing
{
public:

	
	BU_Screwing(const btVector3& relLinVel,const btVector3& relAngVel);

	~BU_Screwing() {
	};
	
	btScalar CalculateF(btScalar t) const;
	//gives interpolated position for time in [0..1] in screwing frame

	inline btPoint3	InBetweenPosition(const btPoint3& pt,btScalar t) const
	{
		return btPoint3(
		pt.x()*btCos(m_w*t)-pt.y()*btSin(m_w*t),
		pt.x()*btSin(m_w*t)+pt.y()*btCos(m_w*t),
		pt.z()+m_s*CalculateF(t));
	}

	inline btVector3	InBetweenVector(const btVector3& vec,btScalar t) const
	{
		return btVector3(
		vec.x()*btCos(m_w*t)-vec.y()*btSin(m_w*t),
		vec.x()*btSin(m_w*t)+vec.y()*btCos(m_w*t),
		vec.z());
	}

	//gives interpolated transform for time in [0..1] in screwing frame
	btTransform	InBetweenTransform(const btTransform& tr,btScalar t) const;

	
	//gives matrix from global frame into screwing frame
	void	LocalMatrix(btTransform &t) const;

	inline const btVector3& GetU() const {	return m_u;}
	inline const btVector3& GetO() const {return m_o;}
	inline const btScalar GetS() const{ return m_s;}
	inline const btScalar GetW() const { return m_w;}
	
private:
	float		m_w;
	float		m_s;
	btVector3 m_u;
	btVector3	m_o;
};

#endif //B_SCREWING_H
