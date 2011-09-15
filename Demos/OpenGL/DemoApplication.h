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

#ifndef DEMO_APPLICATION_H
#define DEMO_APPLICATION_H


#include "GlutStuff.h"
#include "GL_ShapeDrawer.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>


#include "LinearMath/btVector3.h"
#include "LinearMath/btMatrix3x3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btAlignedObjectArray.h"

class	btCollisionShape;
class	btDynamicsWorld;
class	btRigidBody;
class	btTypedConstraint;



class DemoApplication
{
protected:
	void	displayProfileString(int xOffset,int yStart,char* message);
	class CProfileIterator* m_profileIterator;

	protected:
#ifdef USE_BT_CLOCK
	btClock m_clock;
#endif //USE_BT_CLOCK

	///this is the most important class
	btDynamicsWorld*		m_dynamicsWorld;

	///constraint for mouse picking
	btTypedConstraint*		m_pickConstraint;

	virtual void removePickingConstraint();

	btCollisionShape*	m_shootBoxShape;

	float	m_cameraDistance;
	int	m_debugMode;
	
	float m_ele;
	float m_azi;
	btVector3 m_cameraPosition;
	btVector3 m_cameraTargetPosition;//look at

	int	m_mouseOldX;
	int	m_mouseOldY;
	int	m_mouseButtons;
public:
	int	m_modifierKeys;
protected:

	float m_scaleBottom;
	float m_scaleFactor;
	btVector3 m_cameraUp;
	int	m_forwardAxis;
	float m_zoomStepSize;

	int m_glutScreenWidth;
	int m_glutScreenHeight;

	float	m_frustumZNear;
	float	m_frustumZFar;

	int	m_ortho;

	float	m_ShootBoxInitialSpeed;
	
	bool	m_stepping;
	bool m_singleStep;
	bool m_idle;
	int m_lastKey;

	void showProfileInfo(int& xOffset,int& yStart, int yIncr);
	void renderscene(int pass);

	GL_ShapeDrawer*	m_shapeDrawer;
	bool			m_enableshadows;
	btVector3		m_sundirection;
	btScalar		m_defaultContactProcessingThreshold;

public:
		
	DemoApplication();
	
	virtual ~DemoApplication();

	btDynamicsWorld*		getDynamicsWorld()
	{
		return m_dynamicsWorld;
	}

	virtual	void initPhysics() = 0;

	virtual	void setDrawClusters(bool drawClusters)
	{

	}

	void overrideGLShapeDrawer (GL_ShapeDrawer* shapeDrawer);
	
	void setOrthographicProjection();
	void resetPerspectiveProjection();
	
	bool	setTexturing(bool enable) { return(m_shapeDrawer->enableTexture(enable)); }
	bool	setShadows(bool enable)	{ bool p=m_enableshadows;m_enableshadows=enable;return(p); }
	bool	getTexturing() const
	{
		return m_shapeDrawer->hasTextureEnabled();
	}
	bool	getShadows() const
	{
		return m_enableshadows;
	}


	int		getDebugMode()
	{
		return m_debugMode ;
	}
	
	void	setDebugMode(int mode);
	
	void	setAzi(float azi)
	{
		m_azi = azi;
	}
	
	void	setCameraUp(const btVector3& camUp)
	{
		m_cameraUp = camUp;
	}
	void	setCameraForwardAxis(int axis)
	{
		m_forwardAxis = axis;
	}

	virtual void myinit();

	void toggleIdle();
	
	virtual void updateCamera();

	btVector3	getCameraPosition()
	{
		return m_cameraPosition;
	}
	btVector3	getCameraTargetPosition()
	{
		return m_cameraTargetPosition;
	}

	btScalar	getDeltaTimeMicroseconds()
	{
#ifdef USE_BT_CLOCK
		btScalar dt = (btScalar)m_clock.getTimeMicroseconds();
		m_clock.reset();
		return dt;
#else
		return btScalar(16666.);
#endif
	}
	void setFrustumZPlanes(float zNear, float zFar)
	{
		m_frustumZNear = zNear;
		m_frustumZFar = zFar;
	}

	///glut callbacks
				
	float	getCameraDistance();
	void	setCameraDistance(float dist);	
	void	moveAndDisplay();

	virtual void clientMoveAndDisplay() = 0;

	virtual void	clientResetScene();

	///Demo functions
	virtual void setShootBoxShape ();
	virtual void	shootBox(const btVector3& destination);


	btVector3	getRayTo(int x,int y);

	btRigidBody*	localCreateRigidBody(float mass, const btTransform& startTransform,btCollisionShape* shape);

	///callback methods by glut	

	virtual void keyboardCallback(unsigned char key, int x, int y);
	
	virtual void keyboardUpCallback(unsigned char key, int x, int y) {}
	
	virtual void specialKeyboard(int key, int x, int y){}

	virtual void specialKeyboardUp(int key, int x, int y){}

	virtual void reshape(int w, int h);

	virtual void mouseFunc(int button, int state, int x, int y);

	virtual void	mouseMotionFunc(int x,int y);
	
	virtual void displayCallback();

	virtual 	void renderme();

	virtual		void swapBuffers() = 0;

	virtual		void	updateModifierKeys() = 0;

	void stepLeft();
	void stepRight();
	void stepFront();
	void stepBack();
	void zoomIn();
	void zoomOut();

	bool	isIdle() const
	{
		return	m_idle;
	}

	void	setIdle(bool idle)
	{
		m_idle = idle;
	}


};

#endif //DEMO_APPLICATION_H


