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


#include "GL_DialogDynamicsWorld.h"
#include "GL_DialogWindow.h"
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.h"
#include "BulletCollision/CollisionShapes/btBox2dShape.h"
#include "BulletCollision/CollisionShapes/btConvex2dShape.h"
#include "BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.h"

GL_DialogDynamicsWorld::GL_DialogDynamicsWorld()
{
	m_upperBorder = 0;
	m_lowerBorder =0;

	m_pickConstraint = 0;
	m_screenWidth = 0;
	m_screenHeight = 0;

	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_broadphase = new btDbvtBroadphase();
	m_constraintSolver = new btSequentialImpulseConstraintSolver();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_constraintSolver,m_collisionConfiguration);
	m_dynamicsWorld ->getSolverInfo().m_splitImpulse = true;
	//m_dynamicsWorld->setGravity(btVector3(0,10,0));
	m_dynamicsWorld->setGravity(btVector3(0,0,0));

	m_simplexSolver = new btVoronoiSimplexSolver();
	m_pdSolver = new btMinkowskiPenetrationDepthSolver();

	btConvex2dConvex2dAlgorithm::CreateFunc* convexAlgo2d = new btConvex2dConvex2dAlgorithm::CreateFunc(m_simplexSolver,m_pdSolver);
	
	m_dispatcher->registerCollisionCreateFunc(CONVEX_2D_SHAPE_PROXYTYPE,CONVEX_2D_SHAPE_PROXYTYPE,convexAlgo2d);
	m_dispatcher->registerCollisionCreateFunc(BOX_2D_SHAPE_PROXYTYPE,CONVEX_2D_SHAPE_PROXYTYPE,convexAlgo2d);
	m_dispatcher->registerCollisionCreateFunc(CONVEX_2D_SHAPE_PROXYTYPE,BOX_2D_SHAPE_PROXYTYPE,convexAlgo2d);
	m_dispatcher->registerCollisionCreateFunc(BOX_2D_SHAPE_PROXYTYPE,BOX_2D_SHAPE_PROXYTYPE,new btBox2dBox2dCollisionAlgorithm::CreateFunc());
	
	///enable boarders, to avoid 'loosing' menus
#if 1
	btTransform tr;
	tr.setIdentity();
	
	{
		btStaticPlaneShape* plane = new btStaticPlaneShape(btVector3(0,1,0),0);
		m_upperBorder = new btCollisionObject();
		tr.setOrigin(btVector3(0,-BT_LARGE_FLOAT,0));
		m_upperBorder->setWorldTransform(tr);
		m_upperBorder->setCollisionShape(plane);
		m_dynamicsWorld->addCollisionObject(m_upperBorder);
	}

	{
		btStaticPlaneShape* plane = new btStaticPlaneShape(btVector3(0,-1,0),0);
		m_lowerBorder = new btCollisionObject();
		
		tr.setIdentity();
		tr.setOrigin(btVector3(0,BT_LARGE_FLOAT,0));
		m_lowerBorder->setWorldTransform(tr);
		m_lowerBorder->setCollisionShape(plane);
		m_dynamicsWorld->addCollisionObject(m_lowerBorder);
	}
	{
		btStaticPlaneShape* plane = new btStaticPlaneShape(btVector3(1,0,0),0);
		m_leftBorder = new btCollisionObject();
		tr.setIdentity();
		tr.setOrigin(btVector3(-BT_LARGE_FLOAT,0,0));
		m_leftBorder->setWorldTransform(tr);
		m_leftBorder->setCollisionShape(plane);
		m_dynamicsWorld->addCollisionObject(m_leftBorder);
	}
	{
		btStaticPlaneShape* plane = new btStaticPlaneShape(btVector3(-1,0,0),0);
		m_rightBorder = new btCollisionObject();
		tr.setIdentity();
		tr.setOrigin(btVector3(BT_LARGE_FLOAT,0,0));
		m_rightBorder->setWorldTransform(tr);
		m_rightBorder->setCollisionShape(plane);
		m_dynamicsWorld->addCollisionObject(m_rightBorder);
	}
#endif

}

GL_DialogDynamicsWorld::~GL_DialogDynamicsWorld()
{
	delete m_dynamicsWorld;
	delete m_dispatcher;
	delete m_constraintSolver;
	delete m_broadphase;
	delete m_collisionConfiguration;
}

void GL_DialogDynamicsWorld::setScreenSize(int width, int height)
{
	
	
	int i;

	for ( i=0;i<m_dynamicsWorld->getCollisionObjectArray().size();i++)
	{
		btCollisionObject* colObj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(colObj);
		if (body)
		{
			m_dynamicsWorld->removeRigidBody(body);
			btVector3 newPos = colObj->getWorldTransform().getOrigin() + btVector3(btScalar(m_screenWidth/2.),btScalar(m_screenHeight/2),btScalar(0))-btVector3(btScalar(width/2.),btScalar(height/2.),btScalar(0));
			colObj->getWorldTransform().setOrigin(newPos);
			m_dynamicsWorld->addRigidBody(body);
		} else
		{
			m_dynamicsWorld->removeCollisionObject(colObj);
			btVector3 newPos = colObj->getWorldTransform().getOrigin() + btVector3(btScalar(m_screenWidth/2.),btScalar(m_screenHeight/2.),btScalar(0))-btVector3(btScalar(width/2.),btScalar(height/2.),btScalar(0));
			colObj->getWorldTransform().setOrigin(newPos);
			m_dynamicsWorld->addCollisionObject(colObj);
		}
	}

	for ( i=0;i<m_dialogs.size();i++)
	{
		m_dialogs[i]->setScreenSize(width,height);
	}
	if (width && height)
	{
		if (m_upperBorder)
		{
			m_dynamicsWorld->removeCollisionObject(m_upperBorder);

			btTransform tr;
			tr.setIdentity();
			tr.setOrigin(btVector3(btScalar(0),btScalar(-height/2.),btScalar(0.)));
			m_upperBorder->setWorldTransform(tr);
			m_dynamicsWorld->addCollisionObject(m_upperBorder);
		}

		if (m_lowerBorder)
		{
			m_dynamicsWorld->removeCollisionObject(m_lowerBorder);

			btTransform tr;
			tr.setIdentity();
			tr.setOrigin(btVector3(btScalar(0),btScalar(height/2.),btScalar(0)));
			m_lowerBorder->setWorldTransform(tr);
			m_dynamicsWorld->addCollisionObject(m_lowerBorder);
		}
		if (m_leftBorder)
		{
			m_dynamicsWorld->removeCollisionObject(m_leftBorder);
			btTransform tr;
			tr.setIdentity();
			tr.setOrigin(btVector3(btScalar(-width/2.),btScalar(0),btScalar(0)));
			m_leftBorder->setWorldTransform(tr);
			m_dynamicsWorld->addCollisionObject(m_leftBorder);
		}
		if (m_rightBorder)
		{
			m_dynamicsWorld->removeCollisionObject(m_rightBorder);
			btTransform tr;
			tr.setIdentity();
			tr.setOrigin(btVector3(btScalar(width/2.),btScalar(0),btScalar(0)));
			m_rightBorder->setWorldTransform(tr);
			m_dynamicsWorld->addCollisionObject(m_rightBorder);

		}
		
	}


	m_screenWidth = width;
	m_screenHeight = height;
}

GL_DialogWindow*	GL_DialogDynamicsWorld::createDialog(int horPos,int vertPos,int dialogWidth,int dialogHeight, const char* dialogTitle )
{
	btBox2dShape* boxShape = new btBox2dShape(btVector3(dialogWidth/2.f,dialogHeight/2.f,0.4f));
	btScalar mass = 100.f;
	btVector3 localInertia;
	boxShape->calculateLocalInertia(mass,localInertia);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,0,boxShape,localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);
	btTransform trans;
	trans.setIdentity();
//	trans.setOrigin(btVector3(btScalar(horPos-m_screenWidth/2+dialogWidth/2), btScalar(vertPos+m_screenHeight/2.+dialogHeight/2),btScalar(0.)));
	trans.setOrigin(btVector3(btScalar(horPos-m_screenWidth/2+dialogWidth/2), btScalar(vertPos-m_screenHeight/2.+dialogHeight/2),btScalar(0.)));


	
	body->setWorldTransform(trans);
	body->setDamping(0.999f,0.99f);

	//body->setActivationState(ISLAND_SLEEPING);
	body->setLinearFactor(btVector3(1,1,0));
	//body->setAngularFactor(btVector3(0,0,1));
	body->setAngularFactor(btVector3(0,0,0));

	GL_DialogWindow* dialogWindow = new GL_DialogWindow(horPos,vertPos,dialogWidth,dialogHeight,body,dialogTitle);
	m_dialogs.push_back(dialogWindow);
	m_dynamicsWorld->addRigidBody(body);
	
	return dialogWindow;
	
}

GL_SliderControl* GL_DialogDynamicsWorld::createSlider(GL_DialogWindow* dialog, const char* sliderText, btScalar initialFraction)
{
	btBox2dShape* boxShape = new btBox2dShape(btVector3(6.f,6.f,0.4f));
	btScalar mass = .1f;
	btVector3 localInertia;
	boxShape->calculateLocalInertia(mass,localInertia);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,0,boxShape,localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);
	btTransform trans;
	trans.setIdentity();
	int sliderX = dialog->getDialogHorPos() - m_screenWidth/2 + dialog->getDialogWidth()/2;
//	int sliderY = dialog->getDialogVertPos() + m_screenHeight/2 + dialog->getDialogHeight()/2 + dialog->getNumControls()*20;
	int sliderY = dialog->getDialogVertPos() - m_screenHeight/2 + dialog->getDialogHeight()/2 + dialog->getNumControls()*20;
	trans.setOrigin(btVector3((btScalar)sliderX, (btScalar)sliderY,(btScalar)-0.2f));
	
	body->setWorldTransform(trans);
	//body->setDamping(0.999,0.99);

	//body->setActivationState(ISLAND_SLEEPING);
	body->setLinearFactor(btVector3(1,1,0));
	//body->setAngularFactor(btVector3(0,0,1));
	body->setAngularFactor(btVector3(0,0,0));

	m_dynamicsWorld->addRigidBody(body);
	body->setCollisionFlags(body->getFlags()|btCollisionObject::CF_NO_CONTACT_RESPONSE);
	
	btRigidBody* dialogBody = btRigidBody::upcast(dialog->getCollisionObject());
	btAssert(dialogBody);
	
	
	
	btTransform frameInA;
	frameInA.setIdentity();
	int offsX = -dialog->getDialogWidth()/2 + 16;
	int offsY = -dialog->getDialogHeight()/2 + dialog->getNumControls()*20 + 36;
	btVector3 offset(btVector3((btScalar)offsX, (btScalar)offsY, (btScalar)0.2f));
	frameInA.setOrigin(offset);
	
	
	btTransform frameInB;
	frameInB.setIdentity();
	//frameInB.setOrigin(-offset/2);
	
//	btScalar lowerLimit = 80.f;
//	btScalar upperLimit = 170.f;
	btScalar lowerLimit = 141.f;
	btScalar upperLimit = 227.f;

	btScalar actualLimit = lowerLimit+initialFraction*(upperLimit-lowerLimit);


#if 0 
	bool useFrameA = false;

	btGeneric6DofConstraint* constraint = new btGeneric6DofConstraint(*dialogBody,*body,frameInA,frameInB,useFrameA);
	m_dynamicsWorld->addConstraint(constraint,true);
	constraint->setLimit(0,lowerLimit,upperLimit);
#else
	btSliderConstraint* sliderConstraint = new btSliderConstraint(*dialogBody,*body,frameInA,frameInB,true);//useFrameA);
	sliderConstraint->setLowerLinLimit(actualLimit);
	sliderConstraint->setUpperLinLimit(actualLimit);
	m_dynamicsWorld->addConstraint(sliderConstraint,true);

#endif

	
	GL_SliderControl* slider = new GL_SliderControl(sliderText, body,dialog,lowerLimit,upperLimit, sliderConstraint);
	body->setUserPointer(slider);
	dialog->addControl(slider);

	slider->m_fraction = initialFraction;

	return slider;
}



GL_ToggleControl* GL_DialogDynamicsWorld::createToggle(GL_DialogWindow* dialog, const char* toggleText)
{
	

	btBox2dShape* boxShape = new btBox2dShape(btVector3(6.f,6.f,0.4f));
	btScalar mass = 0.1f;
	btVector3 localInertia;
	boxShape->calculateLocalInertia(mass,localInertia);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,0,boxShape,localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);
	btTransform trans;
	trans.setIdentity();

	int toggleX = dialog->getDialogHorPos() - m_screenWidth/2 + dialog->getDialogWidth()/2;
//	int toggleY = dialog->getDialogVertPos() + m_screenHeight/2 + dialog->getDialogHeight()/2 + dialog->getNumControls()*20;
	int toggleY = dialog->getDialogVertPos() - m_screenHeight/2 + dialog->getDialogHeight()/2 + dialog->getNumControls()*20;
	trans.setOrigin(btVector3((btScalar)toggleX, (btScalar)toggleY,(btScalar) -0.2f));
	
	body->setWorldTransform(trans);
	body->setDamping(0.999f,0.99f);

	//body->setActivationState(ISLAND_SLEEPING);
	body->setLinearFactor(btVector3(1,1,0));
	//body->setAngularFactor(btVector3(0,0,1));
	body->setAngularFactor(btVector3(0,0,0));

	m_dynamicsWorld->addRigidBody(body);
	body->setCollisionFlags(body->getFlags()|btCollisionObject::CF_NO_CONTACT_RESPONSE);
	
	btRigidBody* dialogBody = btRigidBody::upcast(dialog->getCollisionObject());
	btAssert(dialogBody);
	
	
	
	btTransform frameInA;
	frameInA.setIdentity();
	btVector3 offset(btVector3(+dialog->getDialogWidth()/2.f-32.f,-dialog->getDialogHeight()/2.f+dialog->getNumControls()*20.f+36.f,0.2f));
	frameInA.setOrigin(offset);
	
	
	btTransform frameInB;
	frameInB.setIdentity();
	//frameInB.setOrigin(-offset/2);
	bool useFrameA = true;

	btGeneric6DofConstraint* constraint = new btGeneric6DofConstraint(*dialogBody,*body,frameInA,frameInB,useFrameA);
	m_dynamicsWorld->addConstraint(constraint,true);


	GL_ToggleControl* toggle = new GL_ToggleControl(toggleText, body,dialog);
	body->setUserPointer(toggle);
	dialog->addControl(toggle);
	return toggle;
}

void	GL_DialogDynamicsWorld::draw(btScalar timeStep)
{
	if (timeStep)
	{
		m_dynamicsWorld->stepSimulation(timeStep);
	}

	for (int i=0;i<m_dialogs.size();i++)
	{
		m_dialogs[i]->draw(timeStep);
	}
}

static btRigidBody* pickedBody = 0;//for deactivation state
static btScalar mousePickClamping = 111130.f;

//static int gPickingConstraintId = 0;
static btVector3 gOldPickingPos;
static btVector3 gHitPos(-1,-1,-1);

static btScalar gOldPickingDist  = 0.f;

bool GL_DialogDynamicsWorld::mouseFunc(int button, int state, int x, int y)
{
	if (state == 0) 
	{
        m_mouseButtons |= 1<<button;
    } else
	{
        m_mouseButtons = 0;
    }

	m_mouseOldX = x;
    m_mouseOldY = y;


	//printf("button %i, state %i, x=%i,y=%i\n",button,state,x,y);
	//button 0, state 0 means left mouse down

	btVector3 rayTo = getRayTo(x,y);

	switch (button)
	{
	case 1:
		{


			if (state==0)
			{

#if 0
				//apply an impulse
				if (m_dynamicsWorld)
				{
					btCollisionWorld::ClosestRayResultCallback rayCallback(m_cameraPosition,rayTo);
					m_dynamicsWorld->rayTest(m_cameraPosition,rayTo,rayCallback);
					if (rayCallback.hasHit())
					{

						btRigidBody* body = btRigidBody::upcast(rayCallback.m_collisionObject);
						if (body)
						{
							body->setActivationState(ACTIVE_TAG);
							btVector3 impulse = rayTo;
							impulse.normalize();
							float impulseStrength = 10.f;
							impulse *= impulseStrength;
							btVector3 relPos = rayCallback.m_hitPointWorld - body->getCenterOfMassPosition();
							body->applyImpulse(impulse,relPos);
						}
					}
				}
#endif



			} else
			{

			}
			break;	
		}
	
	case 0:
		{
			if (state==0)
			{


				//add a point to point constraint for picking
				if (m_dynamicsWorld)
				{
					
					btVector3 rayFrom;
					if (1)//m_ortho)
					{
						rayFrom = rayTo;
						rayFrom.setZ(-100.f);
					}
					//else
					//{
					//	rayFrom = m_cameraPosition;
					//}
					
					btCollisionWorld::ClosestRayResultCallback rayCallback(rayFrom,rayTo);
					m_dynamicsWorld->rayTest(rayFrom,rayTo,rayCallback);
					if (rayCallback.hasHit())
					{


						btScalar maxPickingClamp = mousePickClamping;

						btRigidBody* body = btRigidBody::upcast(rayCallback.m_collisionObject);
						if (body)
						{
							bool doPick = true;
							if (body->getUserPointer())
							{
								///deal with controls in a special way
								GL_DialogControl* ctrl = (GL_DialogControl*)body->getUserPointer();
								
								switch(ctrl->getType())
								{
								case GL_TOGGLE_CONTROL:
									{
										GL_ToggleControl* toggle = (GL_ToggleControl*) ctrl;
										toggle->m_active = !toggle->m_active;
										doPick = false;
										break;
									}
								case GL_SLIDER_CONTROL:
									{
										GL_SliderControl* slider = (GL_SliderControl*) ctrl;
										btTypedConstraint* constraint = slider->getConstraint();
										if (constraint->getConstraintType() == SLIDER_CONSTRAINT_TYPE)
										{
											btSliderConstraint* sliderConstraint = (btSliderConstraint*) constraint;
											sliderConstraint->setLowerLinLimit(slider->getLowerLimit());
											sliderConstraint->setUpperLinLimit(slider->getUpperLimit());
										}
										maxPickingClamp = 100;
									}
								default:
									{
									}
								};

							};

							if (doPick)
							{
								//other exclusions?
								if (!(body->isStaticObject() || body->isKinematicObject()))
								{
									pickedBody = body;
									pickedBody->setActivationState(DISABLE_DEACTIVATION);

									

									btVector3 pickPos = rayCallback.m_hitPointWorld;
									//printf("pickPos=%f,%f,%f\n",pickPos.getX(),pickPos.getY(),pickPos.getZ());


									btVector3 localPivot = body->getCenterOfMassTransform().inverse() * pickPos;

									btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*body,localPivot);
									p2p->m_setting.m_impulseClamp = maxPickingClamp;

									m_dynamicsWorld->addConstraint(p2p);
									m_pickConstraint = p2p;

									//save mouse position for dragging
									gOldPickingPos = rayTo;
									gHitPos = pickPos;

									gOldPickingDist  = (pickPos-rayFrom).length();

									//very weak constraint for picking
									p2p->m_setting.m_tau = 0.1f;
								}
							}
							return true;
						}
					}
				}

			} else
			{

				if (m_pickConstraint && m_dynamicsWorld)
				{
					m_dynamicsWorld->removeConstraint(m_pickConstraint);
					delete m_pickConstraint;
					//printf("removed constraint %i",gPickingConstraintId);
					m_pickConstraint = 0;
					pickedBody->forceActivationState(ACTIVE_TAG);
					pickedBody->setDeactivationTime( 0.f );
					
					
					if (pickedBody->getUserPointer())
					{
						///deal with controls in a special way
						GL_DialogControl* ctrl = (GL_DialogControl*)pickedBody->getUserPointer();
						if (ctrl->getType()==GL_SLIDER_CONTROL)
						{
							GL_SliderControl* sliderControl = (GL_SliderControl*) ctrl;
							
							btSliderConstraint* slider = 0;
							btTypedConstraint* constraint = sliderControl->getConstraint();
							if (constraint->getConstraintType() == SLIDER_CONSTRAINT_TYPE)
							{
								slider = (btSliderConstraint*)constraint;
							}
							if (slider)
							{
								btScalar linDepth = slider->getLinearPos();
								btScalar lowLim = slider->getLowerLinLimit();
								btScalar hiLim = slider->getUpperLinLimit();
								slider->setPoweredLinMotor(false);
								if(linDepth <= lowLim)
								{
									slider->setLowerLinLimit(lowLim);
									slider->setUpperLinLimit(lowLim);
								}
								else if(linDepth > hiLim)
								{
									slider->setLowerLinLimit(hiLim);
									slider->setUpperLinLimit(hiLim);
								}
								else
								{
									slider->setLowerLinLimit(linDepth);
									slider->setUpperLinLimit(linDepth);
								}
							}
						}
								
					}
					
					pickedBody = 0;

				}
				


			}

			break;

		}
	
		default:
			{
			}
	}
	
	return false;

}



btVector3	GL_DialogDynamicsWorld::getRayTo(int x,int y)
{
	float cameraDistance = m_screenHeight/2.f;//m_screenWidth/2;//1.f;
	btVector3 cameraTargetPosition(0,0,0);
	btVector3 cameraUp(0,-1,0);

	if (1)//_ortho)
	{

		btScalar aspect;
		btVector3 extents;
		if (m_screenWidth> m_screenHeight) 
		{
			
			aspect = m_screenWidth / (btScalar)m_screenHeight;
			extents.setValue(aspect * 1.0f, 1.0f,0);
		} else 
		{
			cameraDistance = m_screenWidth/2.f;
			aspect = m_screenHeight / (btScalar)m_screenWidth;
			extents.setValue(1.0f, aspect*1.f,0);
		}
		
	

		extents *= cameraDistance;
		btVector3 lower = cameraTargetPosition - extents;
		btVector3 upper = cameraTargetPosition + extents;

		btScalar u = x / btScalar(m_screenWidth);
		btScalar v = (m_screenHeight - y) / btScalar(m_screenHeight);
		
		btVector3	p(0,0,0);
		p.setValue(
			(1.0f - u) * lower.getX() + u * upper.getX(),
			-((1.0f - v) * lower.getY() + v * upper.getY()),
			cameraTargetPosition.getZ());
		return p;
	}

	float top = 1.f;
	float bottom = -1.f;
	float nearPlane = 1.f;
	float tanFov = (top-bottom)*0.5f / nearPlane;
	float fov = 2 * atanf (tanFov);

	btVector3 cameraPosition(0,0,-100);
	btVector3	rayFrom = cameraPosition;
	btVector3 rayForward = (cameraTargetPosition-cameraPosition);
	rayForward.normalize();
	float farPlane = 10000.f;
	rayForward*= farPlane;

	btVector3 rightOffset;
	btVector3 vertical = cameraUp;

	btVector3 hor;
	hor = rayForward.cross(vertical);
	hor.normalize();
	vertical = hor.cross(rayForward);
	vertical.normalize();

	float tanfov = tanf(0.5f*fov);


	hor *= 2.f * farPlane * tanfov;
	vertical *= 2.f * farPlane * tanfov;

	btScalar aspect;
	
	if (m_screenWidth > m_screenHeight) 
	{
		aspect = m_screenWidth / (btScalar)m_screenHeight;
		
		hor*=aspect;
	} else 
	{
		aspect = m_screenHeight / (btScalar)m_screenWidth;
		vertical*=aspect;
	}


	btVector3 rayToCenter = rayFrom + rayForward;
	btVector3 dHor = hor * 1.f/float(m_screenWidth);
	btVector3 dVert = vertical * 1.f/float(m_screenHeight);


	btVector3 rayTo = rayToCenter - 0.5f * hor + 0.5f * vertical;
	rayTo += btScalar(x) * dHor;
	rayTo -= btScalar(y) * dVert;
	//rayTo += y * dVert;

	return rayTo;
}



void	GL_DialogDynamicsWorld::mouseMotionFunc(int x,int y)
{

	if (m_pickConstraint)
	{
		//move the constraint pivot
		btPoint2PointConstraint* p2p = static_cast<btPoint2PointConstraint*>(m_pickConstraint);
		if (p2p)
		{
			//keep it at the same picking distance

			btVector3 newRayTo = getRayTo(x,y);
			btVector3 rayFrom;
			btVector3 oldPivotInB = p2p->getPivotInB();
			btVector3 newPivotB;
			if (1)//_ortho)
			{
				newPivotB = oldPivotInB;
				newPivotB.setX(newRayTo.getX());
				newPivotB.setY(newRayTo.getY());
			} else
			{
				//rayFrom = m_cameraPosition;
			//	btVector3 dir = newRayTo-rayFrom;
			//	dir.normalize();
			//	dir *= gOldPickingDist;

			//	newPivotB = rayFrom + dir;
			}

			
		
			p2p->setPivotB(newPivotB);
		}

	}

	btScalar dx, dy;
    dx = btScalar(x) - m_mouseOldX;
    dy = btScalar(y) - m_mouseOldY;


	

	m_mouseOldX = x;
    m_mouseOldY = y;
//	updateCamera();


}


