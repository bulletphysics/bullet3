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
#ifndef CONCAVE_DEMO_H
#define CONCAVE_DEMO_H

#include "GlutDemoApplication.h"

struct btCollisionAlgorithmCreateFunc;

///ConcaveDemo shows usage of static concave triangle meshes
///It also shows per-triangle material (friction/restitution) through CustomMaterialCombinerCallback
class ConcaveDemo : public GlutDemoApplication
{

	btCollisionShape * m_trimeshShape;

	public:

	void	initGImpactCollision();
	void	initPhysics();

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	virtual 	void renderme();
	virtual void keyboardCallback(unsigned char key, int x, int y);

	///Demo functions
	void	shootTrimesh(const btVector3& startPosition,const btVector3& destination);


};

#endif //CONCAVE_DEMO_H

