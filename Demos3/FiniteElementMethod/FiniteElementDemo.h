/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2011-2014 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///the Finite Element Method is extracted from the OpenTissue library,
///under the zlib license: http://www.opentissue.org/mediawiki/index.php/Main_Page


#ifndef FINITE_ELEMENT_DEMO_H
#define FINITE_ELEMENT_DEMO_H

#include "Bullet3AppSupport/BulletDemoInterface.h"
#include "OpenGLWindow/CommonGraphicsApp.h"


///quick demo showing the right-handed coordinate system and positive rotations around each axis
class FiniteElementDemo : public BulletDemoInterface
{
    CommonGraphicsApp* m_app;
    float m_x;
    float m_y;
    float m_z;
    
	struct FiniteElementDemoInternalData* m_data;
public:
    
    FiniteElementDemo(CommonGraphicsApp* app);

    virtual ~FiniteElementDemo();

    static BulletDemoInterface*    CreateFunc(CommonGraphicsApp* app)
    {
        return new FiniteElementDemo(app);
    }
    
    virtual void    initPhysics();
    virtual void    exitPhysics();
    virtual void	stepSimulation(float deltaTime);
    virtual void	renderScene();

	
    virtual void	physicsDebugDraw();
    virtual bool	mouseMoveCallback(float x,float y);
    virtual bool	mouseButtonCallback(int button, int state, float x, float y);
    virtual bool	keyboardCallback(int key, int state);
};
#endif //FINITE_ELEMENT_DEMO_H

