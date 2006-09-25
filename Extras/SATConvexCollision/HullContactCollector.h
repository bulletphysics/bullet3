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


#ifndef HULL_CONTACT_COLLECTOR_H
#define HULL_CONTACT_COLLECTOR_H

class Vector3;
class Point3;
class Scalar;
struct Separation;

///HullContactCollector  collects the Hull computation to the contact point results
class HullContactCollector
{
public:

	virtual ~HullContactCollector() {};

	virtual int	BatchAddContactGroup(const Separation& sep,int numContacts,const Vector3& normalWorld,const Vector3& tangent,const Point3* positionsWorld,const float* depths)=0;

	virtual int		GetMaxNumContacts() const = 0;

};

#endif //HULL_CONTACT_COLLECTOR_H