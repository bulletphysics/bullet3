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

#ifndef ET_BREAKOUT_EXAMPLE_H
#define ET_BREAKOUT_EXAMPLE_H

#include "btBulletDynamicsCommon.h"

struct vec2 // your engine-independent object representation 2D vector struct
{
	vec2(float x, float y)
	{
		p[0] = x;
		p[1] = y;
	}
	float p[2];
};

struct vec3 // your engine-independent object representation 3D vector struct
{
	vec3(float x,float y, float z)
	{
		p[0] = x;
		p[1] = y;
		p[2] = z;

	}

	float p[3];
};

struct Vertex // you engine-independent object representation vertex struct
{
	Vertex(vec3 p):position(p), uv(0,0), color(0,0,0) {
	}

    vec3 position; // vertex position
    vec2 uv; // bullet physics browser does not take any textures
	vec3 color; // bullet physics browser does not take any colors
};

/**
 * Game Object base class
 *
 * All game objects inherit from this class.
 */
class GameObject {
public:
	GameObject():m_position(0,0,0),m_orientation(0,0,0,1),m_body(0),m_shape(0),m_tag(PADDLE) {}
	virtual ~GameObject() {}
	void createShapeWithVertices(const Vertex* vertices, unsigned int vertexCount,bool convex);
	void createBodyWithMass(float mass);

	void setPosition(btVector3 position);
	void setOrientation(btQuaternion orientation);

	btRigidBody* m_body;

	enum {
		BALL, 	// ball to hit the bricks
		BORDER, // game area border
		BRICK,	// brick to hit to get points
		PADDLE	// player paddle to hit the ball
	} m_tag; // game object identification tag

	btCollisionShape* m_shape; // collision shape for game object

	btVector3 m_position; // position of game object

	btQuaternion m_orientation; // orientation of game object
};

/**
 * Border Object
 *
 * Game area border to keep the ball within the area.
 */
class Border: public GameObject {
public:
	Border();
	virtual ~Border();

	static Vertex m_border_vertices[132]; // border shape vertices
};

/**
 * Paddle Object
 *
 * Player Paddle to hit the ball back into the area.
 */
class Paddle: public GameObject {
public:
	Paddle();
	virtual ~Paddle();
};

/**
 * Ball Object
 *
 * Ball to hit the bricks within the area.
 */
class Ball: public GameObject {
public:
	Ball();
	virtual ~Ball();
};

/**
 * Brick Object
 *
 * Brick to be hit by the ball to get points.
 */
class Brick: public GameObject {
public:
	Brick();
	virtual ~Brick();
};


class CommonExampleInterface*    ET_BreakoutCreateFunc(struct CommonExampleOptions& options);


#endif //ET_BREAKOUT_EXAMPLE_H
