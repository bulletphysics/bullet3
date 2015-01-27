#ifndef SUPPORT_FUNC_DEMO_H
#define SUPPORT_FUNC_DEMO_H

#include "Bullet3AppSupport/BulletDemoInterface.h"
#include "OpenGLWindow/CommonGraphicsApp.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"


///quick demo showing the support map function (localGetSupportingVertex)
class MySupportFuncDemo : public BulletDemoInterface
{
    CommonGraphicsApp* m_app;
    btSphereShape* m_sphere;
    float m_x;
    float m_y;
    
    
public:
    
    MySupportFuncDemo(CommonGraphicsApp* app)
    :m_app(app),
    m_x(0),
    m_y(0)
    {
		m_app->setUpAxis(1);
        m_sphere = new btSphereShape(1);
        {
            int boxId = m_app->registerCubeShape(10,0.1,10);
            btVector3 pos(0,-2,0);
            btQuaternion orn(0,0,0,1);
            btVector4 color(0.3,0.3,0.3,1);
            
            btVector3 scaling(1,1,1);
            m_app->m_renderer->registerGraphicsInstance(boxId,pos,orn,color,scaling);
        }
        
        {
            int sphereId = m_app->registerGraphicsSphereShape(1,false,0,0);
            btVector3 pos(0,0,0);
            btQuaternion orn(0,0,0,1);
            btVector4 color(0,1,0,0.6);
            
            btVector3 scaling(1,1,1);
            m_app->m_renderer->registerGraphicsInstance(sphereId,pos,orn,color,scaling);
        }
        m_app->m_renderer->writeTransforms();
        m_app->m_renderer->enableBlend(true);
    }
    virtual ~MySupportFuncDemo()
    {
        m_app->m_renderer->enableBlend(false);
        delete m_sphere;
    }
    static BulletDemoInterface*    CreateFunc(CommonGraphicsApp* app)
    {
        return new MySupportFuncDemo(app);
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
        m_y+=0.02f;

    }
    virtual void	renderScene()
    {
        m_app->m_renderer->renderScene();
        
    }
    virtual void	physicsDebugDraw(int debugDrawFlags)
    {
        int width=3;
        btVector3 from(0,0,0);
        btVector3 to(10.*btSin(m_x),10.*btCos(m_x),10.*btSin(m_y));
       
        
        {
            btVector3 color(1,0,0);
            m_app->m_renderer->drawLine(from,to,color,width);
        }
        btVector3 dir(to-from);
        btVector3 sup = m_sphere->btConvexInternalShape::localGetSupportingVertex(dir);
        btVector3 orth0,orth1;
        btPlaneSpace1(dir,orth0,orth1);
        btVector3 color(0,0,1);
        orth0.normalize();
        orth1.normalize();
        
        m_app->m_renderer->drawLine(sup,sup+orth0*0.4,color,3);
        m_app->m_renderer->drawLine(sup,sup+orth1*0.4,color,3);
        
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
#endif //SUPPORT_FUNC_DEMO

