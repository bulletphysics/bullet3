/*
Bullet Continuous Collision Detection and Physics Library Maya Plugin
Copyright (c) 2008 Walt Disney Studios
 
This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising
from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:
 
1. The origin of this software must not be misrepresented; you must
not claim that you wrote the original software. If you use this
software in a product, an acknowledgment in the product documentation
would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must
not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 
Written by: Nicola Candussi <nicola@fluidinteractive.com>
*/

//bt_solver.cpp

#include "bt_solver.h"


btVector3 worldAabbMin(-10000, -10000, -10000);
btVector3 worldAabbMax(10000, 10000, 10000);
int	maxProxies = 32000;

bt_solver_t::bt_solver_t():
            m_broadphase(new btAxisSweep3(worldAabbMin, worldAabbMax, maxProxies)),
            m_solver(new btSequentialImpulseConstraintSolver),
            m_collisionConfiguration(new btDefaultCollisionConfiguration),
            m_dispatcher(new btCollisionDispatcher(m_collisionConfiguration.get())),
            m_dynamicsWorld(new btDiscreteDynamicsWorld(m_dispatcher.get(),
                                                        m_broadphase.get(),
                                                        m_solver.get(),
                                                        m_collisionConfiguration.get()))
{
    //register algorithm for concave meshes
    btGImpactCollisionAlgorithm::registerAlgorithm(m_dispatcher.get());

    m_dynamicsWorld->setGravity(btVector3(0, -9.81, 0));

  //  m_dynamicsWorld->getSolverInfo().m_splitImpulse = true;
}
