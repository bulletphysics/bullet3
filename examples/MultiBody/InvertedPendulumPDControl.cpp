
#include "InvertedPendulumPDControl.h"

#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointFeedback.h"
							
#include "../CommonInterfaces/CommonMultiBodyBase.h"
#include "../Utils/b3ResourcePath.h"
#include "../CommonInterfaces/CommonParameterInterface.h"
static btScalar radius(0.2);
static btScalar kp = 100;
static btScalar kd = 20;
static btScalar maxForce = 100;

struct InvertedPendulumPDControl : public CommonMultiBodyBase
{
    btMultiBody* m_multiBody;
	btAlignedObjectArray<btMultiBodyJointFeedback*> m_jointFeedbacks;
    bool m_once;
	int m_frameCount;
public:

    InvertedPendulumPDControl(struct GUIHelperInterface* helper);
    virtual ~InvertedPendulumPDControl();

    virtual void initPhysics();

    virtual void stepSimulation(float deltaTime);

	virtual void resetCamera()
	{
		float dist = 5;
		float pitch = -21;
		float yaw = 270;
		float targetPos[3]={-1.34,1.4,3.44};
		m_guiHelper->resetCamera(dist,yaw,pitch,targetPos[0],targetPos[1],targetPos[2]);
	}


};

InvertedPendulumPDControl::InvertedPendulumPDControl(struct GUIHelperInterface* helper)
:CommonMultiBodyBase(helper),
m_once(true),
m_frameCount(0)
{
}

InvertedPendulumPDControl::~InvertedPendulumPDControl()
{

}

///this is a temporary global, until we determine if we need the option or not
extern  bool gJointFeedbackInWorldSpace;
extern bool gJointFeedbackInJointFrame;

btMultiBody* createInvertedPendulumMultiBody(btMultiBodyDynamicsWorld* world, GUIHelperInterface* guiHelper, const btTransform& baseWorldTrans, bool fixedBase)
{
	btVector4 colors[4] =
    {
        btVector4(1,0,0,1),
        btVector4(0,1,0,1),
        btVector4(0,1,1,1),
        btVector4(1,1,0,1),
    };
    int curColor = 0;

    bool damping = false;
    bool gyro = false;
    int numLinks = 2;
    bool spherical = false;					//set it ot false -to use 1DoF hinges instead of 3DoF sphericals
    bool canSleep = false;
    bool selfCollide = false;
    btVector3 linkHalfExtents(0.05, 0.37, 0.1);
    btVector3 baseHalfExtents(0.04, 0.35, 0.08);

    
    //mbC->forceMultiDof();							//if !spherical, you can comment this line to check the 1DoF algorithm
    //init the base
    btVector3 baseInertiaDiag(0.f, 0.f, 0.f);
    float baseMass = fixedBase ? 0.f : 10.f;

    if(baseMass)
    {
        //btCollisionShape *shape = new btSphereShape(baseHalfExtents[0]);// btBoxShape(btVector3(baseHalfExtents[0], baseHalfExtents[1], baseHalfExtents[2]));
		btCollisionShape *shape = new btBoxShape(btVector3(baseHalfExtents[0], baseHalfExtents[1], baseHalfExtents[2]));
        shape->calculateLocalInertia(baseMass, baseInertiaDiag);
        delete shape;
    }

        
    btMultiBody *pMultiBody = new btMultiBody(numLinks, 0, baseInertiaDiag, fixedBase, canSleep);
		
        
    
	pMultiBody->setBaseWorldTransform(baseWorldTrans);
    btVector3 vel(0, 0, 0);
//	pMultiBody->setBaseVel(vel);

    //init the links
    btVector3 hingeJointAxis(1, 0, 0);
        
    //y-axis assumed up
    btVector3 parentComToCurrentCom(0, -linkHalfExtents[1] * 2.f, 0);						//par body's COM to cur body's COM offset
    btVector3 currentPivotToCurrentCom(0, -linkHalfExtents[1], 0);							//cur body's COM to cur body's PIV offset
    btVector3 parentComToCurrentPivot = parentComToCurrentCom - currentPivotToCurrentCom;	//par body's COM to cur body's PIV offset

    //////
    btScalar q0 = 1.f * SIMD_PI/ 180.f;
    btQuaternion quat0(btVector3(1, 0, 0).normalized(), q0);
    quat0.normalize();
    /////

    for(int i = 0; i < numLinks; ++i)
    {
		float linkMass = 1.f;
		//if (i==3 || i==2)
		//	linkMass= 1000;
		btVector3 linkInertiaDiag(0.f, 0.f, 0.f);

		btCollisionShape* shape = 0;
		if (i==0)
		{
			shape = new btBoxShape(btVector3(linkHalfExtents[0], linkHalfExtents[1], linkHalfExtents[2]));//
		} else
		{
			shape = new btSphereShape(radius);
		}
		shape->calculateLocalInertia(linkMass, linkInertiaDiag);
		delete shape;


        if(!spherical)
		{
            //pMultiBody->setupRevolute(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f), hingeJointAxis, parentComToCurrentPivot, currentPivotToCurrentCom, false);
		
			if (i==0)
			{
			pMultiBody->setupRevolute(i, linkMass, linkInertiaDiag, i - 1, 
				btQuaternion(0.f, 0.f, 0.f, 1.f), 
				hingeJointAxis, 
				parentComToCurrentPivot, 
				currentPivotToCurrentCom, false);
			} else
			{
				btVector3 parentComToCurrentCom(0, -radius * 2.f, 0);						//par body's COM to cur body's COM offset
				btVector3 currentPivotToCurrentCom(0, -radius, 0);							//cur body's COM to cur body's PIV offset
				btVector3 parentComToCurrentPivot = parentComToCurrentCom - currentPivotToCurrentCom;	//par body's COM to cur body's PIV offset


				pMultiBody->setupFixed(i, linkMass, linkInertiaDiag, i - 1, 
				btQuaternion(0.f, 0.f, 0.f, 1.f), 
				parentComToCurrentPivot, 
				currentPivotToCurrentCom);
			}
					
			//pMultiBody->setupFixed(i,linkMass,linkInertiaDiag,i-1,btQuaternion(0,0,0,1),parentComToCurrentPivot,currentPivotToCurrentCom,false);
		
		}
        else
		{
            //pMultiBody->setupPlanar(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f)/*quat0*/, btVector3(1, 0, 0), parentComToCurrentPivot*2, false);
            pMultiBody->setupSpherical(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f), parentComToCurrentPivot, currentPivotToCurrentCom, false);
		}
    }

    pMultiBody->finalizeMultiDof();

		

    ///
    world->addMultiBody(pMultiBody);
    btMultiBody* mbC = pMultiBody;
    mbC->setCanSleep(canSleep);
    mbC->setHasSelfCollision(selfCollide);
    mbC->setUseGyroTerm(gyro);
    //
    if(!damping)
    {
        mbC->setLinearDamping(0.f);
        mbC->setAngularDamping(0.f);
    }else
    {	mbC->setLinearDamping(0.1f);
        mbC->setAngularDamping(0.9f);
    }
    //
    	

    //////////////////////////////////////////////
    if(numLinks > 0)
    {
        btScalar q0 = 180.f * SIMD_PI/ 180.f;
        if(!spherical)
		{
			mbC->setJointPosMultiDof(0, &q0);
		}
        else
        {
            btQuaternion quat0(btVector3(1, 1, 0).normalized(), q0);
            quat0.normalize();
            mbC->setJointPosMultiDof(0, quat0);
        }
    }
    ///

    btAlignedObjectArray<btQuaternion> world_to_local;
    world_to_local.resize(pMultiBody->getNumLinks() + 1);

    btAlignedObjectArray<btVector3> local_origin;
    local_origin.resize(pMultiBody->getNumLinks() + 1);
    world_to_local[0] = pMultiBody->getWorldToBaseRot();
    local_origin[0] = pMultiBody->getBasePos();
  //  double friction = 1;
    {

    //	float pos[4]={local_origin[0].x(),local_origin[0].y(),local_origin[0].z(),1};
       // float quat[4]={-world_to_local[0].x(),-world_to_local[0].y(),-world_to_local[0].z(),world_to_local[0].w()};


        if (1)
        {
            btCollisionShape* shape = new btBoxShape(btVector3(baseHalfExtents[0],baseHalfExtents[1],baseHalfExtents[2]));//new btSphereShape(baseHalfExtents[0]);
            guiHelper->createCollisionShapeGraphicsObject(shape);

            btMultiBodyLinkCollider* col= new btMultiBodyLinkCollider(pMultiBody, -1);
            col->setCollisionShape(shape);

            btTransform tr;
            tr.setIdentity();
//if we don't set the initial pose of the btCollisionObject, the simulator will do this 
			//when syncing the btMultiBody link transforms to the btMultiBodyLinkCollider
               
            tr.setOrigin(local_origin[0]);
			btQuaternion orn(btVector3(0,0,1),0.25*3.1415926538);
				
            tr.setRotation(orn);
            col->setWorldTransform(tr);

			bool isDynamic = (baseMass > 0 && !fixedBase);
			int collisionFilterGroup = isDynamic? int(btBroadphaseProxy::DefaultFilter) : int(btBroadphaseProxy::StaticFilter);
			int collisionFilterMask = isDynamic? 	int(btBroadphaseProxy::AllFilter) : 	int(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);


            world->addCollisionObject(col,collisionFilterGroup,collisionFilterMask);//, 2,1+2);

            btVector4 color(0.0,0.0,0.5,1);
            guiHelper->createCollisionObjectGraphicsObject(col,color);

//                col->setFriction(friction);
            pMultiBody->setBaseCollider(col);

        }
    }


    for (int i=0; i < pMultiBody->getNumLinks(); ++i)
    {
        const int parent = pMultiBody->getParent(i);
        world_to_local[i+1] = pMultiBody->getParentToLocalRot(i) * world_to_local[parent+1];
        local_origin[i+1] = local_origin[parent+1] + (quatRotate(world_to_local[i+1].inverse() , pMultiBody->getRVector(i)));
    }


    for (int i=0; i < pMultiBody->getNumLinks(); ++i)
    {

        btVector3 posr = local_origin[i+1];
    //	float pos[4]={posr.x(),posr.y(),posr.z(),1};

		const btScalar quat[4]={-world_to_local[i+1].x(),-world_to_local[i+1].y(),-world_to_local[i+1].z(),world_to_local[i+1].w()};
		btCollisionShape* shape =0;

		if (i==0)
		{
			shape = new btBoxShape(btVector3(linkHalfExtents[0],linkHalfExtents[1],linkHalfExtents[2]));//btSphereShape(linkHalfExtents[0]);
		} else
		{
				
			shape = new btSphereShape(radius);
		}

        guiHelper->createCollisionShapeGraphicsObject(shape);
        btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(pMultiBody, i);

        col->setCollisionShape(shape);
        btTransform tr;
        tr.setIdentity();
        tr.setOrigin(posr);
        tr.setRotation(btQuaternion(quat[0],quat[1],quat[2],quat[3]));
        col->setWorldTransform(tr);
    //       col->setFriction(friction);
		bool isDynamic = 1;//(linkMass > 0);
		int collisionFilterGroup = isDynamic? int(btBroadphaseProxy::DefaultFilter) : int(btBroadphaseProxy::StaticFilter);
		int collisionFilterMask = isDynamic? 	int(btBroadphaseProxy::AllFilter) : 	int(btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);

		//if (i==0||i>numLinks-2)
		{
			world->addCollisionObject(col,collisionFilterGroup,collisionFilterMask);//,2,1+2);
				btVector4 color = colors[curColor];
		curColor++;
		curColor&=3;
        guiHelper->createCollisionObjectGraphicsObject(col,color);


        pMultiBody->getLink(i).m_collider=col;
		}
         
    }
    
	return pMultiBody;
}

void InvertedPendulumPDControl::initPhysics()
{
	{
		SliderParams slider("Kp",&kp);
		slider.m_minVal=-200;
		slider.m_maxVal=200;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
    }
	{
		SliderParams slider("Kd",&kd);
		slider.m_minVal=-50;
		slider.m_maxVal=50;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
    }
	{
		SliderParams slider("max force",&maxForce);
		slider.m_minVal=0;
		slider.m_maxVal=100;
		m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
    }

	


    int upAxis = 1;
	gJointFeedbackInWorldSpace = true;
	gJointFeedbackInJointFrame = true;

	m_guiHelper->setUpAxis(upAxis);

    
	this->createEmptyDynamicsWorld();
    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
    m_dynamicsWorld->getDebugDrawer()->setDebugMode(
        //btIDebugDraw::DBG_DrawConstraints
        +btIDebugDraw::DBG_DrawWireframe
        +btIDebugDraw::DBG_DrawContactPoints
        +btIDebugDraw::DBG_DrawAabb
        );//+btIDebugDraw::DBG_DrawConstraintLimits);

	m_dynamicsWorld->setGravity(btVector3(0,-10,0));
	btTransform baseWorldTrans;
	baseWorldTrans.setIdentity();
	baseWorldTrans.setOrigin(btVector3(1,2,3));
	m_multiBody = createInvertedPendulumMultiBody(m_dynamicsWorld, m_guiHelper, baseWorldTrans, true);

	//for (int i=pMultiBody->getNumLinks()-1;i>=0;i--)//
	for (int i=0;i<m_multiBody->getNumLinks();i++)
	{
		btMultiBodyJointFeedback* fb = new btMultiBodyJointFeedback();
		m_multiBody->getLink(i).m_jointFeedback = fb;
		m_jointFeedbacks.push_back(fb);
		//break;
	}
        
	
}
char fileName[1024];

static btAlignedObjectArray<btScalar> qDesiredArray;
void InvertedPendulumPDControl::stepSimulation(float deltaTime)
{
	
	static btScalar offset = -0.1*SIMD_PI;

	m_frameCount++;
	if ((m_frameCount&0xff)==0 )
	{
		offset = -offset;
	}
	btScalar target= SIMD_PI+offset;
	qDesiredArray.resize(0);
	qDesiredArray.resize(m_multiBody->getNumLinks(),target);

	for (int joint = 0; joint<m_multiBody->getNumLinks();joint++)
	{
		int dof1 = 0;
		btScalar qActual = m_multiBody->getJointPosMultiDof(joint)[dof1];
		btScalar qdActual = m_multiBody->getJointVelMultiDof(joint)[dof1];
		btScalar positionError = (qDesiredArray[joint]-qActual);
		double desiredVelocity = 0;
		btScalar velocityError = (desiredVelocity-qdActual);
		btScalar force = kp * positionError + kd*velocityError;
		btClamp(force,-maxForce,maxForce);
		m_multiBody->addJointTorque(joint, force);
	}

	
	

	if (m_frameCount==100)
	{
		const char* gPngFileName = "pendulum";
	

		if (gPngFileName)
        {

			
            //printf("gPngFileName=%s\n",gPngFileName);

            sprintf(fileName,"%s%d.png",gPngFileName,m_frameCount);
			b3Printf("Made screenshot %s",fileName);
			this->m_guiHelper->getAppInterface()->dumpNextFrameToPng(fileName);
        }
	}
    m_dynamicsWorld->stepSimulation(1./60.,0);//240,0);

	static int count = 0;
	if ((count& 0x0f)==0)
	{
#if 0
	for (int i=0;i<m_jointFeedbacks.size();i++)
	{

			b3Printf("F_reaction[%i] linear:%f,%f,%f, angular:%f,%f,%f",
			i,
			m_jointFeedbacks[i]->m_reactionForces.m_topVec[0],
			m_jointFeedbacks[i]->m_reactionForces.m_topVec[1],
			m_jointFeedbacks[i]->m_reactionForces.m_topVec[2],

		m_jointFeedbacks[i]->m_reactionForces.m_bottomVec[0],
			m_jointFeedbacks[i]->m_reactionForces.m_bottomVec[1],
			m_jointFeedbacks[i]->m_reactionForces.m_bottomVec[2]

		);

	}
#endif
	}
	count++;


	/*
    b3Printf("base angvel = %f,%f,%f",m_multiBody->getBaseOmega()[0],
             m_multiBody->getBaseOmega()[1],
             m_multiBody->getBaseOmega()[2]
             );
    */
 //   btScalar jointVel =m_multiBody->getJointVel(0);
    
//    b3Printf("child angvel = %f",jointVel);
    
    
    
}


class CommonExampleInterface*    InvertedPendulumPDControlCreateFunc(struct CommonExampleOptions& options)
{
	return new InvertedPendulumPDControl(options.m_guiHelper);
}
