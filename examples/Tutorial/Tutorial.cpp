
#include "Tutorial.h"
#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "../CommonInterfaces/CommonRenderInterface.h"

#include "../CommonInterfaces/CommonExampleInterface.h"
#include "LinearMath/btTransform.h"

#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../RenderingExamples/TimeSeriesCanvas.h"
#include "stb_image/stb_image.h"

struct LWPose
{
	btVector3		m_worldPosition;
	btQuaternion	m_worldOrientation;
	LWPose()
		:m_worldPosition(0,0,0),
		m_worldOrientation(0,0,0,1)
	{
	}

};

enum LWRIGIDBODY_FLAGS
{
	LWFLAG_USE_QUATERNION_DERIVATIVE = 1,

};
struct LWRightBody
{
	LWPose	m_worldPose;
	btVector3 m_linearVelocity;
	btVector3 m_angularVelocity;

	int				m_graphicsIndex;
	int				m_flags;

	LWRightBody()
		:m_linearVelocity(0,0,0),
		m_angularVelocity(0,0,0),
		m_flags(LWFLAG_USE_QUATERNION_DERIVATIVE)
	{

	}

	void	integrateTransform(double deltaTime)
	{
		LWPose newPose;

		newPose.m_worldPosition = m_worldPose.m_worldPosition + m_linearVelocity*deltaTime;

		if (m_flags & LWFLAG_USE_QUATERNION_DERIVATIVE)
		{
			newPose.m_worldOrientation = m_worldPose.m_worldOrientation;
			newPose.m_worldOrientation += (m_angularVelocity * newPose.m_worldOrientation) * (deltaTime * btScalar(0.5));
			newPose.m_worldOrientation.normalize();
			m_worldPose = newPose;
		} else
		{
			//Exponential map
			//google for "Practical Parameterization of Rotations Using the Exponential Map", F. Sebastian Grassia

			//btQuaternion q_w = [ sin(|w|*dt/2) * w/|w| , cos(|w|*dt/2)]
			//btQuaternion q_new =  q_w * q_old;
			
			btVector3 axis;
			btScalar	fAngle = m_angularVelocity.length(); 
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
			btQuaternion dorn (axis.x(),axis.y(),axis.z(),btCos( fAngle*deltaTime*btScalar(0.5) ));
			btQuaternion orn0 = m_worldPose.m_worldOrientation;

			btQuaternion predictedOrn = dorn * orn0;
			predictedOrn.normalize();
			m_worldPose.m_worldOrientation = predictedOrn;
		}

	}
	

	void	stepSimulation(double deltaTime)
	{
		integrateTransform(deltaTime);
	}
};



class Tutorial : public CommonExampleInterface
{
    CommonGraphicsApp* m_app;
    int m_tutorialIndex;

	LWRightBody*	m_body;
	
	TimeSeriesCanvas*			m_timeSeriesCanvas;
	

public:
    
    Tutorial(CommonGraphicsApp* app, int tutorialIndex)
    :m_app(app),
	m_tutorialIndex(tutorialIndex)
    {
		m_app->setUpAxis(2);

		m_timeSeriesCanvas = new TimeSeriesCanvas(app->m_2dCanvasInterface,512,256,"Position and Velocity");
		m_timeSeriesCanvas ->setupTimeSeries(5,100, 0);
		m_timeSeriesCanvas->addDataSource("X position (m)", 255,0,0);
		m_timeSeriesCanvas->addDataSource("X velocity (m/s)", 0,0,255);
		m_timeSeriesCanvas->addDataSource("dX/dt (m/s)", 0,0,0);

		
		switch (m_tutorialIndex)
		{
				
		}
		{

		 int boxId = m_app->registerCubeShape(100,100,1);
            btVector3 pos(0,0,-1);
            btQuaternion orn(0,0,0,1);
            btVector4 color(1,1,1,1);
            btVector3 scaling(1,1,1);
            m_app->m_renderer->registerGraphicsInstance(boxId,pos,orn,color,scaling);
		}

		m_body = new LWRightBody();
		m_body->m_worldPose.m_worldPosition.setValue(0,0,3);

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

            int boxId = m_app->registerCubeShape(1,1,1,textureIndex);//>registerGraphicsUnitSphereShape(SPHERE_LOD_HIGH, textureIndex);
            btVector4 color(1,1,1,1);
            btVector3 scaling(1,1,1);
            m_body->m_graphicsIndex = m_app->m_renderer->registerGraphicsInstance(boxId,m_body->m_worldPose.m_worldPosition, m_body->m_worldPose.m_worldOrientation,color,scaling);
			m_app->m_renderer->writeSingleInstanceTransformToCPU(m_body->m_worldPose.m_worldPosition, m_body->m_worldPose.m_worldOrientation, m_body->m_graphicsIndex);
        }

		 m_app->m_renderer->writeTransforms();
    }
    virtual ~Tutorial()
    {
		delete m_timeSeriesCanvas;
		m_timeSeriesCanvas = 0;
        m_app->m_renderer->enableBlend(false);
    }
    
    
    virtual void    initPhysics()
    {
    }
    virtual void    exitPhysics()
    {
        
    }
    virtual void	stepSimulation(float deltaTime)
    {

		m_body->m_linearVelocity=btVector3(1,0,0);

		
		float time = m_timeSeriesCanvas->getCurrentTime();
		
		float xPos = m_body->m_worldPose.m_worldPosition.x();
		
		float xVel = m_body->m_linearVelocity.x();
		
		m_timeSeriesCanvas->insertDataAtCurrentTime(xPos,0,true);
		m_timeSeriesCanvas->insertDataAtCurrentTime(xVel,1,true);
		
		m_timeSeriesCanvas->nextTick();

		
		m_body->integrateTransform(deltaTime);

		m_app->m_renderer->writeSingleInstanceTransformToCPU(m_body->m_worldPose.m_worldPosition, m_body->m_worldPose.m_worldOrientation, m_body->m_graphicsIndex);
		 m_app->m_renderer->writeTransforms();
    }
    virtual void	renderScene()
    {
		m_app->m_renderer->renderScene();
		m_app->drawText3D("X",1,0,0,1);
		m_app->drawText3D("Y",0,1,0,1);
		m_app->drawText3D("Z",0,0,1,1);
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

class CommonExampleInterface*    TutorialCreateFunc(struct CommonExampleOptions& options)
{
	return new Tutorial(options.m_guiHelper->getAppInterface(), options.m_option);
}

