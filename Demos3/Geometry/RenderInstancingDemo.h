#ifndef RENDER_INSTANCING_DEMO_H
#define RENDER_INSTANCING_DEMO_H

#include "Bullet3AppSupport/BulletDemoInterface.h"
#include "OpenGLWindow/CommonGraphicsApp.h"
#include "Bullet3Common/b3Quaternion.h"



///quick demo showing the right-handed coordinate system and positive rotations around each axis
class RenderInstancingDemo : public BulletDemoInterface
{
    CommonGraphicsApp* m_app;
    float m_x;
    float m_y;
    float m_z;
	b3AlignedObjectArray<int> m_movingInstances;
	enum
	{
		numCubesX = 20,
		numCubesY = 20
	};
public:
    
    RenderInstancingDemo(CommonGraphicsApp* app)
    :m_app(app),
    m_x(0),
    m_y(0),
	m_z(0)
    {
		m_app->setUpAxis(2);
        
		 {
             b3Vector3 extents=b3MakeVector3(100,100,100);
             extents[m_app->getUpAxis()]=1;
             
			 int xres = 20;
			 int yres = 20;
			 
			 b3Vector4 color0=b3MakeVector4(0.1, 0.1, 0.1,1);
			 b3Vector4 color1=b3MakeVector4(0.6, 0.6, 0.6,1);
            m_app->registerGrid(xres, yres, color0, color1);
        }
		 
        {
            int boxId = m_app->registerCubeShape(0.1,0.1,0.1);
           
            
            
            for (int i=-numCubesX/2;i<numCubesX/2;i++)
            {
                for (int j = -numCubesY/2;j<numCubesY/2;j++)
                {
                    b3Vector3 pos=b3MakeVector3(i,j,j);
                    pos[app->getUpAxis()] = 1;
                    b3Quaternion orn(0,0,0,1);
                    b3Vector4 color=b3MakeVector4(0.3,0.3,0.3,1);
                    b3Vector3 scaling=b3MakeVector3(1,1,1);
                   int instanceId = m_app->m_renderer->registerGraphicsInstance(boxId,pos,orn,color,scaling);
				   m_movingInstances.push_back(instanceId);
                }
            }
        }

		 m_app->m_renderer->writeTransforms();
    }
    virtual ~RenderInstancingDemo()
    {
        m_app->m_renderer->enableBlend(false);
    }

    static BulletDemoInterface*    CreateFunc(CommonGraphicsApp* app)
    {
        return new RenderInstancingDemo(app);
    }
    virtual void physicsDebugDraw(int debugDrawMode)
    {
        
    }
    virtual void    initPhysics()
    {
    }
    virtual void    exitPhysics()
    {
        
    }
    virtual void	stepSimulation(float deltaTime)
    {
        m_x+=0.01f;
        m_y+=0.01f;
		m_z+=0.01f;
		int index=0;
        for (int i=-numCubesX/2;i<numCubesX/2;i++)
        {
            for (int j = -numCubesY/2;j<numCubesY/2;j++)
            {
                b3Vector3 pos=b3MakeVector3(i,j,j);
                pos[m_app->getUpAxis()] = 1+1*b3Sin(m_x+i-j);
                float orn[4]={0,0,0,1};
                m_app->m_renderer->writeSingleInstanceTransformToCPU(pos,orn,m_movingInstances[index++]);
            }
        }
        m_app->m_renderer->writeTransforms();
        
    }
    virtual void	renderScene()
    {
		m_app->m_renderer->renderScene();
    }

	
    virtual void	physicsDebugDraw()
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
    
};
#endif //RENDER_INSTANCING_DEMO_H

