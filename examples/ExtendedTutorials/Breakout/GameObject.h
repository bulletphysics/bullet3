/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef EXAMPLES_SRC_EXTENDEDTUTORIALS_BREAKOUT_GAMEOBJECT_H_
#define EXAMPLES_SRC_EXTENDEDTUTORIALS_BREAKOUT_GAMEOBJECT_H_

#include "btBulletDynamicsCommon.h"

struct vec2
{
	vec2(float x, float y)
	{
		p[0] = x;
		p[1] = y;
	}
	float p[2];
};

struct vec3
{
	vec3(float x,float y, float z)
	{
		p[0] = x;
		p[1] = y;
		p[2] = z;

	}

	float p[4];
};

struct Vertex
{
	Vertex(vec3 p):position(p), uv(0,0), color(0,0,0) {
	}

    vec3 position;
    vec2 uv; // bullet physics browser does not take any textures
	vec3 color; // bullet physics browser does not take any colors
};

/*
 *
 */
class GameObject {
public:
	GameObject();
	virtual ~GameObject();
	void createShapeWithVertices(const Vertex* vertices, unsigned int vertexCount,bool convex);
	void createBodyWithMass(float mass);

	void setPosition(btVector3 position);
	void setOrientation(btQuaternion orientation);

	btRigidBody* m_body;

	enum {
		BALL,
		BORDER,
		BRICK,
		PADDLE
	} m_tag;

	btCollisionShape* m_shape;

	btVector3 m_position;
	btQuaternion m_rotation;
};

#endif /* EXAMPLES_SRC_EXTENDEDTUTORIALS_BREAKOUT_GAMEOBJECT_H_ */
