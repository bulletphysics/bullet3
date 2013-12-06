/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef GL_DIALOG_DYNAMICS_WORLD_H
#define GL_DIALOG_DYNAMICS_WORLD_H

class btDiscreteDynamicsWorld;
class GL_DialogWindow;
class btDefaultCollisionConfiguration;
struct btDbvtBroadphase;
class btSequentialImpulseConstraintSolver;
class btCollisionDispatcher;
class btVoronoiSimplexSolver;
class btMinkowskiPenetrationDepthSolver;
class btCollisionObject;
class btTypedConstraint;
struct GL_ToggleControl;
struct GL_SliderControl;


#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btVector3.h"

class GL_DialogDynamicsWorld
{

	btDefaultCollisionConfiguration* m_collisionConfiguration;
	btDbvtBroadphase* m_broadphase;
	btSequentialImpulseConstraintSolver* m_constraintSolver;
	btCollisionDispatcher* m_dispatcher;
	btVoronoiSimplexSolver* m_simplexSolver;
	btMinkowskiPenetrationDepthSolver* m_pdSolver;

	btDiscreteDynamicsWorld* m_dynamicsWorld;
	
	btCollisionObject*	m_lowerBorder;
	btCollisionObject*	m_upperBorder;
	btCollisionObject*	m_leftBorder;
	btCollisionObject*	m_rightBorder;

	btAlignedObjectArray<GL_DialogWindow*> m_dialogs;

	int	m_screenWidth;
	int	m_screenHeight;


	///for picking
	int	m_mouseOldX;
	int	m_mouseOldY;
	int	m_mouseButtons;
	///constraint for mouse picking
	btTypedConstraint*		m_pickConstraint;

	btVector3	getRayTo(int x,int y);

public:

	GL_DialogDynamicsWorld();

	virtual ~GL_DialogDynamicsWorld();

	virtual	void	setScreenSize(int width, int height);

	virtual	GL_DialogWindow* createDialog(int horPos,int vertPos,int dialogWidth,int dialogHeight, const char* dialogTitle );

	GL_ToggleControl* createToggle(GL_DialogWindow* dialog, const char* toggleText);

	GL_SliderControl* createSlider(GL_DialogWindow* dialog, const char* sliderText, btScalar initialFraction = btScalar(0.5f));

	virtual	void	draw(btScalar timeStep);

	virtual	bool	mouseFunc(int button, int state, int x, int y);

	virtual	void	mouseMotionFunc(int x,int y);
		
};

#endif //GL_DIALOG_DYNAMICS_WORLD_H
