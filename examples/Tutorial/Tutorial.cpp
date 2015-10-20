
#include "Tutorial.h"
#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "../CommonInterfaces/CommonRenderInterface.h"

#include "../CommonInterfaces/CommonExampleInterface.h"
#include "LinearMath/btTransform.h"

#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../RenderingExamples/TimeSeriesCanvas.h"
#include "stb_image/stb_image.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3Matrix3x3.h"
#include "../CommonInterfaces/CommonParameterInterface.h"

#include "LinearMath/btAlignedObjectArray.h"
#define  stdvector btAlignedObjectArray

#define SPHERE_RADIUS 1
static btScalar gRestitution = 0.f;
static btScalar gMassA = 1.f;
static btScalar gMassB = 0.f;

enum LWEnumCollisionTypes
{
	LW_PLANE_TYPE,
	LW_SPHERE_TYPE,
	LW_BOX_TYPE
};

struct LWPlane
{
	BT_DECLARE_ALIGNED_ALLOCATOR();

	b3Vector3   m_normal;
	btScalar    m_planeConstant;
};

struct LWSphere
{
	btScalar    m_radius;
	
	void computeLocalInertia(b3Scalar mass, b3Vector3& localInertia)
	{
		btScalar elem = b3Scalar(0.4) * mass * m_radius*m_radius;
		localInertia.setValue(elem,elem,elem);
	}
};

struct LWBox
{
	BT_DECLARE_ALIGNED_ALLOCATOR();
	b3Vector3 m_halfExtents;
};

struct LWCollisionShape
{
	LWEnumCollisionTypes m_type;
	union
	{
		LWPlane     m_plane;
		LWSphere    m_sphere;
		LWBox       m_box;
	};
	
};

struct LWPose
{
	BT_DECLARE_ALIGNED_ALLOCATOR();

	b3Vector3		m_position;
	b3Quaternion	m_orientation;
	LWPose()
	:m_position(b3MakeVector3(0,0,0)),
	m_orientation(0,0,0,1)
	{
	}
	
	b3Vector3 transformPoint(const b3Vector3& pointIn)
	{
		b3Vector3 rotPoint = b3QuatRotate(m_orientation,pointIn);
		return rotPoint+m_position;
	}
	
};
struct LWContactPoint
{
	b3Vector3 m_ptOnAWorld;
	b3Vector3 m_ptOnBWorld;
	b3Vector3 m_normalOnB;
	btScalar  m_distance;
};

///returns true if we found a pair of closest points
void ComputeClosestPointsPlaneSphere(const LWPlane& planeWorld, const LWSphere& sphere, const LWPose& spherePose, LWContactPoint& pointOut) {
	b3Vector3 spherePosWorld = spherePose.m_position;
	btScalar t = -(spherePosWorld.dot(-planeWorld.m_normal)+planeWorld.m_planeConstant);
	b3Vector3 intersectionPoint = spherePosWorld+t*-planeWorld.m_normal;
	b3Scalar distance = t-sphere.m_radius;
	pointOut.m_distance = distance;
	pointOut.m_ptOnBWorld = intersectionPoint;
	pointOut.m_ptOnAWorld = spherePosWorld+sphere.m_radius*-planeWorld.m_normal;
	pointOut.m_normalOnB = planeWorld.m_normal;
}

void ComputeClosestPointsSphereSphere(const LWSphere& sphereA, const LWPose& sphereAPose, const LWSphere& sphereB, const LWPose& sphereBPose, LWContactPoint& pointOut) {
	b3Vector3 diff = sphereAPose.m_position-sphereBPose.m_position;
	btScalar len = diff.length();
	pointOut.m_distance = len - (sphereA.m_radius+sphereB.m_radius);
	pointOut.m_normalOnB = b3MakeVector3(1,0,0);
	if (len > B3_EPSILON) {
		pointOut.m_normalOnB = diff / len;
	}
	pointOut.m_ptOnAWorld = sphereAPose.m_position - sphereA.m_radius*pointOut.m_normalOnB;
	pointOut.m_ptOnBWorld = pointOut.m_ptOnAWorld-pointOut.m_normalOnB*pointOut.m_distance;
}


enum LWRIGIDBODY_FLAGS
{
	LWFLAG_USE_QUATERNION_DERIVATIVE = 1,
	
};
struct LWRigidBody
{
	BT_DECLARE_ALIGNED_ALLOCATOR();

	LWPose	m_worldPose;
	b3Vector3 m_linearVelocity;
	b3Vector3 m_angularVelocity;
	b3Vector3 m_gravityAcceleration;
	b3Vector3 m_localInertia;
	b3Scalar m_invMass;

	b3Matrix3x3 m_invInertiaTensorWorld;

	void computeInvInertiaTensorWorld()
	{
		b3Vector3 invInertiaLocal;
		invInertiaLocal.setValue(m_localInertia.x != btScalar(0.0) ? btScalar(1.0) / m_localInertia.x: btScalar(0.0),
								   m_localInertia.y != btScalar(0.0) ? btScalar(1.0) / m_localInertia.y: btScalar(0.0),
								   m_localInertia.z != btScalar(0.0) ? btScalar(1.0) / m_localInertia.z: btScalar(0.0));
		b3Matrix3x3 m (m_worldPose.m_orientation);
		m_invInertiaTensorWorld = m.scaled(invInertiaLocal) * m.transpose();
	}
	
	int				m_graphicsIndex;
	LWCollisionShape m_collisionShape;
	
	
	LWRIGIDBODY_FLAGS				m_flags;
	
	LWRigidBody()
	:m_linearVelocity(b3MakeVector3(0,0,0)),
	m_angularVelocity(b3MakeVector3(0,0,0)),
	m_gravityAcceleration(b3MakeVector3(0,0,0)),//-10,0)),
	m_flags(LWFLAG_USE_QUATERNION_DERIVATIVE)
	{
		
	}
	
	const b3Vector3& getPosition() const
	{
		return m_worldPose.m_position;
	}
	
	b3Vector3 getVelocity(const b3Vector3& relPos) const
	{
		return m_linearVelocity + m_angularVelocity.cross(relPos);
	}
	
	void	integrateAcceleration(double deltaTime) {
		m_linearVelocity += m_gravityAcceleration*deltaTime;
	}
	
	void applyImpulse(const b3Vector3& impulse, const b3Vector3& rel_pos)
	{
		m_linearVelocity += impulse * m_invMass;
		b3Vector3 torqueImpulse = rel_pos.cross(impulse);
		m_angularVelocity += m_invInertiaTensorWorld * torqueImpulse;
	}
	
	void	integrateVelocity(double deltaTime)
	{
		LWPose newPose;
		
		newPose.m_position = m_worldPose.m_position + m_linearVelocity*deltaTime;
		
		if (m_flags & LWFLAG_USE_QUATERNION_DERIVATIVE)
		{
			newPose.m_orientation = m_worldPose.m_orientation;
			newPose.m_orientation += (m_angularVelocity * newPose.m_orientation) * (deltaTime * btScalar(0.5));
			newPose.m_orientation.normalize();
			m_worldPose = newPose;
		} else
		{
			//Exponential map
			//google for "Practical Parameterization of Rotations Using the Exponential Map", F. Sebastian Grassia
			
			//btQuaternion q_w = [ sin(|w|*dt/2) * w/|w| , cos(|w|*dt/2)]
			//btQuaternion q_new =  q_w * q_old;
			
			b3Vector3 axis;
			b3Scalar	fAngle = m_angularVelocity.length();
			//limit the angular motion
			const btScalar angularMotionThreshold =  btScalar(0.5)*SIMD_HALF_PI;
			
			if (fAngle*deltaTime > angularMotionThreshold)
			{
				fAngle = angularMotionThreshold / deltaTime;
			}
			
			if ( fAngle < btScalar(0.001) )
			{
				// use Taylor's expansions of sync function
				axis   = m_angularVelocity*( btScalar(0.5)*deltaTime-(deltaTime*deltaTime*deltaTime)*(btScalar(0.020833333333))*fAngle*fAngle );
			}
			else
			{
				// sync(fAngle) = sin(c*fAngle)/t
				axis   = m_angularVelocity*( btSin(btScalar(0.5)*fAngle*deltaTime)/fAngle );
			}
			b3Quaternion dorn (axis.x,axis.y,axis.z,btCos( fAngle*deltaTime*b3Scalar(0.5) ));
			b3Quaternion orn0 = m_worldPose.m_orientation;
			
			b3Quaternion predictedOrn = dorn * orn0;
			predictedOrn.normalize();
			m_worldPose.m_orientation = predictedOrn;
		}
		
	}
	
	
	void	stepSimulation(double deltaTime)
	{
		integrateVelocity(deltaTime);
	}
};


b3Scalar resolveCollision(LWRigidBody& bodyA,
					  LWRigidBody& bodyB,
					 LWContactPoint& contactPoint)
{
	b3Assert(contactPoint.m_distance<=0);
	
	
	btScalar appliedImpulse = 0.f;
	
	b3Vector3 rel_pos1 = contactPoint.m_ptOnAWorld - bodyA.m_worldPose.m_position;
	b3Vector3 rel_pos2 = contactPoint.m_ptOnBWorld - bodyB.getPosition();
	
	btScalar rel_vel = contactPoint.m_normalOnB.dot(bodyA.getVelocity(rel_pos1) - bodyB.getVelocity(rel_pos2));
	if (rel_vel < -B3_EPSILON) 
	{
		b3Vector3 temp1 = bodyA.m_invInertiaTensorWorld * rel_pos1.cross(contactPoint.m_normalOnB); 
		b3Vector3 temp2 = bodyB.m_invInertiaTensorWorld * rel_pos2.cross(contactPoint.m_normalOnB); 
	
		btScalar impulse = -(1.0f + gRestitution) * rel_vel / 
		(bodyA.m_invMass + bodyB.m_invMass + contactPoint.m_normalOnB.dot(temp1.cross(rel_pos1) + temp2.cross(rel_pos2)));
		
		b3Vector3 impulse_vector = contactPoint.m_normalOnB * impulse;
		b3Printf("impulse = %f\n", impulse);
		appliedImpulse = impulse;
		bodyA.applyImpulse(impulse_vector, rel_pos1);
		bodyB.applyImpulse(-impulse_vector, rel_pos2);
	}
	return appliedImpulse;
}



class Tutorial : public CommonExampleInterface
{
    CommonGraphicsApp* m_app;
	GUIHelperInterface* m_guiHelper;
    int m_tutorialIndex;

	stdvector<LWRigidBody*> m_bodies;
	
	TimeSeriesCanvas*			m_timeSeriesCanvas0;
	TimeSeriesCanvas*			m_timeSeriesCanvas1;

	stdvector<LWContactPoint> m_contactPoints;
	
	int m_stage;
	int m_counter;
public:
    
    Tutorial(GUIHelperInterface* guiHelper, int tutorialIndex)
    :m_app(guiHelper->getAppInterface()),
	m_guiHelper(guiHelper),
	m_tutorialIndex(tutorialIndex),
	m_stage(0),
	m_counter(0),
	m_timeSeriesCanvas0(0),
	m_timeSeriesCanvas1(0)
    {
		int numBodies = 1;
		
		m_app->setUpAxis(1);
		m_app->m_renderer->enableBlend(true);
		
		switch (m_tutorialIndex)
		{
			case TUT_VELOCITY:
			{
				numBodies=10;
				m_timeSeriesCanvas0 = new TimeSeriesCanvas(m_app->m_2dCanvasInterface,512,256,"Constant Velocity");
				
				m_timeSeriesCanvas0 ->setupTimeSeries(2,60, 0);
				m_timeSeriesCanvas0->addDataSource("X position (m)", 255,0,0);
				m_timeSeriesCanvas0->addDataSource("X velocity (m/s)", 0,0,255);
				m_timeSeriesCanvas0->addDataSource("dX/dt (m/s)", 0,0,0);
				break;
			}
			case TUT_ACCELERATION:
			{
				numBodies=10;
				m_timeSeriesCanvas1 = new TimeSeriesCanvas(m_app->m_2dCanvasInterface,256,512,"Constant Acceleration");
				
				m_timeSeriesCanvas1 ->setupTimeSeries(50,60, 0);
				m_timeSeriesCanvas1->addDataSource("Y position (m)", 255,0,0);
				m_timeSeriesCanvas1->addDataSource("Y velocity (m/s)", 0,0,255);
				m_timeSeriesCanvas1->addDataSource("dY/dt (m/s)", 0,0,0);
				break;
			}
			case TUT_COLLISION:
			{
				numBodies=2;
				m_timeSeriesCanvas1 = new TimeSeriesCanvas(m_app->m_2dCanvasInterface,512,200,"Distance");
				m_timeSeriesCanvas1 ->setupTimeSeries(1.5,60, 0);
				m_timeSeriesCanvas1->addDataSource("distance", 255,0,0);
				break;
			}
			
			case TUT_SOLVE_CONTACT_CONSTRAINT:
			{
				numBodies=2;
				m_timeSeriesCanvas1 = new TimeSeriesCanvas(m_app->m_2dCanvasInterface,512,200,"Collision Impulse");
				m_timeSeriesCanvas1 ->setupTimeSeries(1.5,60, 0);
				m_timeSeriesCanvas1->addDataSource("Distance", 0,0,255);
				m_timeSeriesCanvas1->addDataSource("Impulse magnutide", 255,0,0);
				
				{
					SliderParams slider("Restitution",&gRestitution);
					slider.m_minVal=0;
					slider.m_maxVal=1;
					m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
				}
				{
					SliderParams slider("Mass A",&gMassA);
					slider.m_minVal=0;
					slider.m_maxVal=100;
					m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
				}
				
				{
					SliderParams slider("Mass B",&gMassB);
					slider.m_minVal=0;
					slider.m_maxVal=100;
					m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
				}
				
				
				
				break;
			}
				
			default:
			{
				
				m_timeSeriesCanvas0 = new TimeSeriesCanvas(m_app->m_2dCanvasInterface,512,256,"Unknown");
				m_timeSeriesCanvas0 ->setupTimeSeries(1,60, 0);
				
			}
		};

		
		
		if (m_tutorialIndex==TUT_VELOCITY)
		{

		 int boxId = m_app->registerCubeShape(100,1,100);
            b3Vector3 pos = b3MakeVector3(0,-3.5,0);
            b3Quaternion orn(0,0,0,1);
            b3Vector4 color = b3MakeVector4(1,1,1,1);
            b3Vector3 scaling = b3MakeVector3(1,1,1);
            m_app->m_renderer->registerGraphicsInstance(boxId,pos,orn,color,scaling);
		}

		for (int i=0;i<numBodies;i++)
		{
			m_bodies.push_back(new LWRigidBody());
		}
		for (int i=0;i<m_bodies.size();i++)
		{
			m_bodies[i]->m_worldPose.m_position.setValue((i/4)*5,3,(i&3)*5);
		}
		{
			int textureIndex = -1;
			
			if (1)
			{
				int width,height,n;
				
				const char* filename = "data/cube.png";
				const unsigned char* image=0;
				
				const char* prefix[]={"./","../","../../","../../../","../../../../"};
				int numprefix = sizeof(prefix)/sizeof(const char*);
				
				for (int i=0;!image && i<numprefix;i++)
				{
					char relativeFileName[1024];
					sprintf(relativeFileName,"%s%s",prefix[i],filename);
					image = stbi_load(relativeFileName, &width, &height, &n, 0);
				}
				
				b3Assert(image);
				if (image)
				{
					textureIndex = m_app->m_renderer->registerTexture(image,width,height);
				}
			}
			
			//            int boxId = m_app->registerCubeShape(1,1,1,textureIndex);
			int boxId = m_app->registerGraphicsUnitSphereShape(SPHERE_LOD_HIGH, textureIndex);
			b3Vector4 color = b3MakeVector4(1,1,1,0.8);
			b3Vector3 scaling = b3MakeVector3(SPHERE_RADIUS,SPHERE_RADIUS,SPHERE_RADIUS);
			for (int i=0;i<m_bodies.size();i++)
			{
				m_bodies[i]->m_collisionShape.m_sphere.m_radius = SPHERE_RADIUS;
				m_bodies[i]->m_collisionShape.m_type = LW_SPHERE_TYPE;
				
				m_bodies[i]->m_graphicsIndex = m_app->m_renderer->registerGraphicsInstance(boxId,m_bodies[i]->m_worldPose.m_position, m_bodies[i]->m_worldPose.m_orientation,color,scaling);
				m_app->m_renderer->writeSingleInstanceTransformToCPU(m_bodies[i]->m_worldPose.m_position, m_bodies[i]->m_worldPose.m_orientation, m_bodies[i]->m_graphicsIndex);
			}
		}

		
		if (m_tutorialIndex == TUT_SOLVE_CONTACT_CONSTRAINT)
		{
			m_bodies[0]->m_invMass = gMassA? 1./gMassA : 0;
			m_bodies[0]->m_collisionShape.m_sphere.computeLocalInertia(gMassA,m_bodies[0]->m_localInertia);
			
			m_bodies[1]->m_invMass =gMassB? 1./gMassB : 0;
			m_bodies[1]->m_collisionShape.m_sphere.computeLocalInertia(gMassB,m_bodies[1]->m_localInertia);

			if (gMassA)
				m_bodies[0]->m_linearVelocity.setValue(0,0,1);
			if (gMassB)
				m_bodies[1]->m_linearVelocity.setValue(0,0,-1);

		}
		

			
		 m_app->m_renderer->writeTransforms();
    }
    virtual ~Tutorial()
    {
		delete m_timeSeriesCanvas0;
		delete m_timeSeriesCanvas1;

		m_timeSeriesCanvas0 = 0;
		m_timeSeriesCanvas1 = 0;

		m_app->m_renderer->enableBlend(false);
    }
    
    
    virtual void    initPhysics()
    {
    }
    virtual void    exitPhysics()
    {
        
    }
	
	void tutorial1Update(float deltaTime);
	void tutorial2Update(float deltaTime);
	void tutorialCollisionUpdate(float deltaTime,LWContactPoint& contact);
	void tutorialSolveContactConstraintUpdate(float deltaTime,LWContactPoint& contact);
	
    virtual void	stepSimulation(float deltaTime)
    {
		switch (m_tutorialIndex)
		{
			case TUT_VELOCITY:
			{
				tutorial1Update(deltaTime);
				float xPos = m_bodies[0]->m_worldPose.m_position.x;
				float xVel = m_bodies[0]->m_linearVelocity.x;
				m_timeSeriesCanvas0->insertDataAtCurrentTime(xPos,0,true);
				m_timeSeriesCanvas0->insertDataAtCurrentTime(xVel,1,true);
				break;
			}
			case TUT_ACCELERATION:
			{
				tutorial2Update(deltaTime);
				float yPos = m_bodies[0]->m_worldPose.m_position.y;
				float yVel = m_bodies[0]->m_linearVelocity.y;
				m_timeSeriesCanvas1->insertDataAtCurrentTime(yPos,0,true);
				m_timeSeriesCanvas1->insertDataAtCurrentTime(yVel,1,true);
				
				break;
			}
			case TUT_COLLISION:
			{
				m_contactPoints.clear();
				LWContactPoint contactPoint;
				tutorialCollisionUpdate(deltaTime, contactPoint);
				m_contactPoints.push_back(contactPoint);
				m_timeSeriesCanvas1->insertDataAtCurrentTime(contactPoint.m_distance,0,true);
				
				break;
			}
			case TUT_SOLVE_CONTACT_CONSTRAINT:
			{
				m_contactPoints.clear();
				LWContactPoint contactPoint;
				tutorialSolveContactConstraintUpdate(deltaTime, contactPoint);
				m_contactPoints.push_back(contactPoint);
				if (contactPoint.m_distance<0)
				{
					m_bodies[0]->computeInvInertiaTensorWorld();
					m_bodies[1]->computeInvInertiaTensorWorld();
					
					b3Scalar appliedImpulse = resolveCollision(*m_bodies[0],
									 *m_bodies[1],
									 contactPoint
									 );
				
					m_timeSeriesCanvas1->insertDataAtCurrentTime(appliedImpulse,1,true);
					
				} else
				{
					m_timeSeriesCanvas1->insertDataAtCurrentTime(0.,1,true);
				}
				m_timeSeriesCanvas1->insertDataAtCurrentTime(contactPoint.m_distance,0,true);
				
				break;
			}

			default:
			{
			}
			
		};
		
		
		if (m_timeSeriesCanvas0)
			m_timeSeriesCanvas0->nextTick();
		
		if (m_timeSeriesCanvas1)
			m_timeSeriesCanvas1->nextTick();

		
		for (int i=0;i<m_bodies.size();i++)
		{
			
			m_bodies[i]->integrateAcceleration(deltaTime);
			m_bodies[i]->integrateVelocity(deltaTime);
			
			m_app->m_renderer->writeSingleInstanceTransformToCPU(m_bodies[i]->m_worldPose.m_position, m_bodies[i]->m_worldPose.m_orientation, m_bodies[i]->m_graphicsIndex);
		}

		
		 m_app->m_renderer->writeTransforms();
    }
    virtual void	renderScene()
    {
		m_app->m_renderer->renderScene();
		m_app->drawText3D("X",1,0,0,1);
		m_app->drawText3D("Y",0,1,0,1);
		m_app->drawText3D("Z",0,0,1,1);

		for (int i=0;i<m_contactPoints.size();i++)
		{
			const LWContactPoint& contact = m_contactPoints[i];
			b3Vector3 color=b3MakeVector3(1,1,0);
			float lineWidth=3;
			if (contact.m_distance<0)
			{
				color.setValue(1,0,0);
			}
			m_app->m_renderer->drawLine(contact.m_ptOnAWorld,contact.m_ptOnBWorld,color,lineWidth);
		}
    }

	

    virtual void	physicsDebugDraw(int debugDrawFlags)
    {
      
		
    }
    virtual bool	mouseMoveCallback(float x,float y)
    {
		return false;   
    }
    virtual bool	mouseButtonCallback(int button, int state, float x, float y)
    {
        return false;   
    }
    virtual bool	keyboardCallback(int key, int state)
    {
        return false;   
    }
  

	virtual void resetCamera()
	{
		float dist = 10.5;
		float pitch = 136;
		float yaw = 32;
		float targetPos[3]={0,0,0};
		if (m_app->m_renderer  && m_app->m_renderer->getActiveCamera())
		{
			m_app->m_renderer->getActiveCamera()->setCameraDistance(dist);
			m_app->m_renderer->getActiveCamera()->setCameraPitch(pitch);
			m_app->m_renderer->getActiveCamera()->setCameraYaw(yaw);
			m_app->m_renderer->getActiveCamera()->setCameraTargetPosition(targetPos[0],targetPos[1],targetPos[2]);
		}
	}
};

void Tutorial::tutorial2Update(float deltaTime)
{
	for (int i=0;i<m_bodies.size();i++)
	{
		m_bodies[i]->m_gravityAcceleration.setValue(0,-10,0);
	}
}
void Tutorial::tutorial1Update(float deltaTime)
{
	for (int i=0;i<m_bodies.size();i++)
	{
	switch (m_stage)
	{
		case 0:
		{
			m_bodies[i]->m_angularVelocity=b3MakeVector3(0,0,0);
			m_bodies[i]->m_linearVelocity=b3MakeVector3(1,0,0);
			break;
		}
		case 1:
		{
			m_bodies[i]->m_linearVelocity=b3MakeVector3(-1,0,0);
			break;
		}
		case 2:
		{
			m_bodies[i]->m_linearVelocity=b3MakeVector3(0,1,0);
			break;
		}
		case 3:
		{
			m_bodies[i]->m_linearVelocity=b3MakeVector3(0,-1,0);
			break;
		}
		case 4:
		{
			m_bodies[i]->m_linearVelocity=b3MakeVector3(0,0,1);
			break;
		}
		case 5:
		{
			m_bodies[i]->m_linearVelocity=b3MakeVector3(0,0,-1);
			break;
		}
		case 6:
		{
			m_bodies[i]->m_linearVelocity=b3MakeVector3(0,0,0);
			m_bodies[i]->m_angularVelocity=b3MakeVector3(1,0,0);
			break;
		}
		case 7:
		{
			m_bodies[i]->m_angularVelocity=b3MakeVector3(-1,0,0);
			break;
		}
		case 8:
		{
			m_bodies[i]->m_angularVelocity=b3MakeVector3(0,1,0);
			break;
		}
		case 9:
		{
			m_bodies[i]->m_angularVelocity=b3MakeVector3(0,-1,0);
			break;
		}
		case 10:
		{
			m_bodies[i]->m_angularVelocity=b3MakeVector3(0,0,1);
			break;
		}
		case 11:
		{
			m_bodies[i]->m_angularVelocity=b3MakeVector3(0,0,-1);
			break;
		}
		default:
		{
			m_bodies[i]->m_angularVelocity=b3MakeVector3(0,0,0);
		}
	};
	}
	
	m_counter++;
	if (m_counter>60)
	{
		m_counter=0;
		m_stage++;
		if (m_stage>11)
				m_stage=0;
		b3Printf("Stage = %d\n",m_stage);
		b3Printf("linVel = %f,%f,%f\n",m_bodies[0]->m_linearVelocity.x,m_bodies[0]->m_linearVelocity.y,m_bodies[0]->m_linearVelocity.z);
		b3Printf("angVel = %f,%f,%f\n",m_bodies[0]->m_angularVelocity.x,m_bodies[0]->m_angularVelocity.y,m_bodies[0]->m_angularVelocity.z);
		
	}
}


void Tutorial::tutorialSolveContactConstraintUpdate(float deltaTime,LWContactPoint& contact)
{
	ComputeClosestPointsSphereSphere(m_bodies[0]->m_collisionShape.m_sphere,
									 m_bodies[0]->m_worldPose,
									 m_bodies[1]->m_collisionShape.m_sphere,
									 m_bodies[1]->m_worldPose,
									 contact);
	
}
	
void Tutorial::tutorialCollisionUpdate(float deltaTime,LWContactPoint& contact)
{
	m_bodies[1]->m_worldPose.m_position.z = 3;

	
	ComputeClosestPointsSphereSphere(m_bodies[0]->m_collisionShape.m_sphere,
									 m_bodies[0]->m_worldPose,
									 m_bodies[1]->m_collisionShape.m_sphere,
									 m_bodies[1]->m_worldPose,
									 contact);
						
	switch (m_stage)
	{
		case 0:
		{
			m_bodies[0]->m_angularVelocity=b3MakeVector3(0,0,0);
			m_bodies[0]->m_linearVelocity=b3MakeVector3(1,0,0);
			break;
		}
		case 1:
		{
			m_bodies[0]->m_linearVelocity=b3MakeVector3(-1,0,0);
			break;
		}
		case 2:
		{
			m_bodies[0]->m_linearVelocity=b3MakeVector3(0,1,0);
			break;
		}
		case 3:
		{
			m_bodies[0]->m_linearVelocity=b3MakeVector3(0,-1,0);
			break;
		}
		case 4:
		{
			m_bodies[0]->m_linearVelocity=b3MakeVector3(0,0,1);
			break;
		}
		case 5:
		{
			m_bodies[0]->m_linearVelocity=b3MakeVector3(0,0,-1);
			break;
		}
		default:{}
	};
	m_counter++;
	if (m_counter>120)
	{
		m_counter=0;
		m_stage++;
		if (m_stage>5)
			m_stage=0;
	
	}
}



class CommonExampleInterface*    TutorialCreateFunc(struct CommonExampleOptions& options)
{
	return new Tutorial(options.m_guiHelper, options.m_option);
}

