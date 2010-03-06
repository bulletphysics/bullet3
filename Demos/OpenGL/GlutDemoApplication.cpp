
#ifndef _WINDOWS

#include "GlutDemoApplication.h"

#include "GlutStuff.h"

#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

void	GlutDemoApplication::updateModifierKeys()
{
	m_modifierKeys = 0;
	if (glutGetModifiers() & GLUT_ACTIVE_ALT)
		m_modifierKeys |= BT_ACTIVE_ALT;

	if (glutGetModifiers() & GLUT_ACTIVE_CTRL)
		m_modifierKeys |= BT_ACTIVE_CTRL;
	
	if (glutGetModifiers() & GLUT_ACTIVE_SHIFT)
		m_modifierKeys |= BT_ACTIVE_SHIFT;
}

void GlutDemoApplication::specialKeyboard(int key, int x, int y)	
{
	(void)x;
	(void)y;

	switch (key) 
	{
	case GLUT_KEY_F1:
		{

			break;
		}

	case GLUT_KEY_F2:
		{

			break;
		}


	case GLUT_KEY_END:
		{
			int numObj = getDynamicsWorld()->getNumCollisionObjects();
			if (numObj)
			{
				btCollisionObject* obj = getDynamicsWorld()->getCollisionObjectArray()[numObj-1];

				getDynamicsWorld()->removeCollisionObject(obj);
				btRigidBody* body = btRigidBody::upcast(obj);
				if (body && body->getMotionState())
				{
					delete body->getMotionState();					
				}
				delete obj;


			}
			break;
		}
	case GLUT_KEY_LEFT : stepLeft(); break;
	case GLUT_KEY_RIGHT : stepRight(); break;
	case GLUT_KEY_UP : stepFront(); break;
	case GLUT_KEY_DOWN : stepBack(); break;
	case GLUT_KEY_PAGE_UP : zoomIn(); break;
	case GLUT_KEY_PAGE_DOWN : zoomOut(); break;
	case GLUT_KEY_HOME : toggleIdle(); break;
	default:
		//        std::cout << "unused (special) key : " << key << std::endl;
		break;
	}

	glutPostRedisplay();

}

void GlutDemoApplication::swapBuffers()
{
	glutSwapBuffers();

}

#endif //_WINDOWS


