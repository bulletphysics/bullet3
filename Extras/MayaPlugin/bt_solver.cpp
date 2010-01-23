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

Modified by Roman Ponomarev <rponom@gmail.com>
01/22/2010 : Constraints reworked
*/

//bt_solver.cpp

#include "bt_solver.h"
#include "../BulletColladaConverter/ColladaConverter.h"

btVector3 minWorld(-10000,-10000,-10000);
btVector3 maxWorld(10000,10000,10000);
int maxNumObj=32768;


void bt_solver_t::export_collada_file(const char* fileName)
{
	ColladaConverter tmpConverter(m_dynamicsWorld.get());
	tmpConverter.save(fileName);

}

void bt_solver_t::import_collada_file(const char* filename)
{
//todo: need to create actual bodies etc
	ColladaConverter tmpConverter(m_dynamicsWorld.get());
	tmpConverter.load(filename);
}


bt_solver_t::bt_solver_t():
//            m_broadphase(new btAxisSweep3(minWorld,maxWorld,maxNumObj)),
	m_broadphase(new btDbvtBroadphase()),  
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

    m_dynamicsWorld->setGravity(btVector3(0, -9.81f, 0));

  //  m_dynamicsWorld->getSolverInfo().m_splitImpulse = true;
}
