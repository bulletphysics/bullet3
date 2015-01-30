#ifndef SPHERE_CREATION_H
#define SPHERE_CREATION_H

#include "Bullet3AppSupport/BulletDemoInterface.h"
#include "OpenGLWindow/CommonGraphicsApp.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"
#include "BulletCollision/CollisionShapes/btConvexPolyhedron.h"


class SphereCreation : public BulletDemoInterface
{
    CommonGraphicsApp* m_app;
    float m_x;
    float m_y;
    
	btAlignedObjectArray<btVector3> m_vertices;
	btConvexHullShape* m_convexHull;
	
public:
    
    SphereCreation(CommonGraphicsApp* app)
    :m_app(app),
    m_x(0),
    m_y(0)
    {
		m_app->setUpAxis(1);
        m_app->m_renderer->writeTransforms();
    }
    virtual ~SphereCreation()
    {
    }
    static BulletDemoInterface*    CreateFunc(CommonGraphicsApp* app)
    {
        return new SphereCreation(app);
    }
    
	inline btScalar randRange(btScalar minRange, btScalar maxRange)
	{
		return (rand() / (btScalar(RAND_MAX) + btScalar(1.0))) * (maxRange - minRange) + minRange;
	}
	
    virtual void    initPhysics()
    {
		srand(0);
		//create random vertices 
		int numVerts = 256;
		for (int i=0;i<numVerts;i++)
		{
			btVector3 v(randRange(-1,1),randRange(-1,1),randRange(-1,1));
			v.safeNormalize();
			m_vertices.push_back(v);
		}
		
		for (int i=0;i<1;i++)
		{
			distributeVerticesOnSphere(5,0.2);
		}
		
		m_convexHull = new btConvexHullShape(&m_vertices[0].x(),m_vertices.size(),sizeof(btVector3));
		m_convexHull->initializePolyhedralFeatures();
		
    }
    virtual void    exitPhysics()
    {
        //dump vertices

		printf("indices:\n");
		const btConvexPolyhedron*	poly = m_convexHull->getConvexPolyhedron();
		for (int i=0;i<poly->m_vertices.size();i++)
		{
			printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,\n",poly->m_vertices[i].x(),
				   poly->m_vertices[i].y(),
				   poly->m_vertices[i].z(),
				   1.f,
				   poly->m_vertices[i].x(),
				   poly->m_vertices[i].y(),
				   poly->m_vertices[i].z(),
				   0.5f,0.5f);
		}

		for (int p=0;p<poly->m_faces.size();p++)
		{
			for (int f=2;f<poly->m_faces[p].m_indices.size();f++)
			{
				int index0=poly->m_faces[p].m_indices[f-2];
				int index1=poly->m_faces[p].m_indices[f-1];
				int index2=poly->m_faces[p].m_indices[f];
				printf("%d,%d,%d,\n",index0,index1,index2);
			}
		}
}
	void distributeVerticesOnSphere(int iterations,btScalar relaxation)
	{
		btScalar previousAverage=0;
		
		btScalar averageDist = 0;
		btScalar previousAverageError = 0.0f;
		btScalar maxError = -1e30f;
		btScalar averageError = 0.f;
		
		///bring closest the closest neighbor of each vertex closer to the 'average' neighbor distance
		for (int i=0;i<iterations;i++)
		{
			maxError = -1e30f;
			averageDist = 0.f;
			averageError = 0.f;
			int numActualVerts = 0;
			
			for (int v=0;v<m_vertices.size();v++)
			{
				btScalar minDist = 1e30f;
				int closestOther = -1;
				//find closest vertex
				for (int w=0;w<m_vertices.size();w++)
				{
					if (w!=v)
					{
						numActualVerts++;
						btVector3 vec = m_vertices[w]-m_vertices[v];
						btScalar d = vec.length();
						if (d<minDist)
						{
							minDist = d;
							closestOther = w;
						}
					}
				}
				btScalar errorDist = previousAverage-minDist;
				if (maxError < errorDist)
				{
					maxError = errorDist;
				}
				averageDist+=minDist;
				if (closestOther>=0)
				{
					btVector3 vec = m_vertices[closestOther]-m_vertices[v];
					vec.safeNormalize();
					if (previousAverage)
					{
						averageError+=btFabs(errorDist);
						m_vertices[v] -= vec*previousAverageError*relaxation;
						m_vertices[closestOther] += vec*previousAverageError*relaxation;
						m_vertices[v].normalize();
						m_vertices[closestOther].normalize();
					}
				}
				
			}
			averageDist /= btScalar(m_vertices.size());
			averageError /= btScalar(m_vertices.size());
			previousAverageError = averageError;
			previousAverage = averageDist;
		}
		char msg[1024];
		btScalar targetDist = 2.0f*sqrtf(4.0f/(btScalar)m_vertices.size());
		sprintf(msg,"averageDist = %f, previousAverageError = %f, maxError = %f, target = %f\n",averageDist,previousAverageError,maxError,targetDist);
		b3Printf(msg);
	}
    virtual void	stepSimulation(float deltaTime)
    {
        m_x+=0.01f;
        m_y+=0.02f;
		//bringVerticesTogether();

    }
    virtual void	renderScene()
    {
        m_app->m_renderer->renderScene();
        
    }
    virtual void	physicsDebugDraw(int debugDrawFlags)
    {
   		int lineWidth = 1;
		int pointSize = 2;
		
		distributeVerticesOnSphere(12,0.2f);
		
		delete m_convexHull;
		m_convexHull = new btConvexHullShape(&m_vertices[0].x(),m_vertices.size(),sizeof(btVector3));
		m_convexHull->initializePolyhedralFeatures();
		const btConvexPolyhedron*	poly = m_convexHull->getConvexPolyhedron();
		
		if (m_vertices.size() != poly->m_vertices.size())
		{
			printf("Warning: m_vertices.size()=%d and poly->m_vertices.size()=%d\n", m_vertices.size(),
				   poly->m_vertices.size());
			return;
		} else
		{
			for (int i=0;i<m_vertices.size();i++)
			{
				m_vertices[i] = poly->m_vertices[i];
			}
		}
		
		btScalar maxEdgeLength = -1e30f;
		btScalar averageEdgeLength=0.f;
		btScalar numLengts = 0;
		
		for (int p=0;p<poly->m_faces.size();p++)
		{
			for (int f=2;f<poly->m_faces[p].m_indices.size();f++)
			{
				btVector4 color0(0,0,1,1);
				btVector4 color1(0,0,1,1);
				btVector4 color2(0,0,1,1);
				
				int index0=poly->m_faces[p].m_indices[f-2];
				int index1=poly->m_faces[p].m_indices[f-1];
				int index2=poly->m_faces[p].m_indices[f];
				
				btVector3 v0 = poly->m_vertices[index0];
				btVector3 v1 = poly->m_vertices[index1];
				btVector3 v2 = poly->m_vertices[index2];
				btVector3 e0 = v1-v0;
				btVector3 e1 = v2-v1;
				btVector3 e2 = v0-v2;
				btScalar e0Length = e0.length();
				btScalar e1Length = e1.length();
				btScalar e2Length = e2.length();
				averageEdgeLength+= e0Length;
				averageEdgeLength+= e1Length;
				averageEdgeLength+= e2Length;
				numLengts+=3.f;
				
			}
		}

		averageEdgeLength/=numLengts;
		
		for (int p=0;p<poly->m_faces.size();p++)
		{
			for (int f=2;f<poly->m_faces[p].m_indices.size();f++)
			{
				btVector4 color0(0,0,1,1);
				btVector4 color1(0,0,1,1);
				btVector4 color2(0,0,1,1);
				
				int index0=poly->m_faces[p].m_indices[f-2];
				int index1=poly->m_faces[p].m_indices[f-1];
				int index2=poly->m_faces[p].m_indices[f];
				
				btVector3 v0 = m_vertices[index0];
				btVector3 v1 = m_vertices[index1];
				btVector3 v2 = m_vertices[index2];
				btVector3 e0 = v1-v0;
				btVector3 e1 = v2-v1;
				btVector3 e2 = v0-v2;
				btScalar e0Length = e0.length();
				btScalar e1Length = e1.length();
				btScalar e2Length = e2.length();
				btScalar scaling = -0.01;//-0.01f;//0;//-0.02f;
				if (e0Length>maxEdgeLength)
					maxEdgeLength = e0Length;
				{
					btScalar errorLength = averageEdgeLength-e0Length;
					m_vertices[index0] += e0.normalized()*errorLength*scaling;
					m_vertices[index1] -= e0.normalized()*errorLength*scaling;
					m_vertices[index0].normalize();
					m_vertices[index1].normalize();
					color0.setValue(1,0,0,1);
					//shift vertices
					
				}
				if (e1Length>maxEdgeLength)
					maxEdgeLength = e1Length;
				
				{
					btScalar errorLength = averageEdgeLength-e1Length;
					m_vertices[index1] += e1.normalized()*errorLength*scaling;
					m_vertices[index2] -= e1.normalized()*errorLength*scaling;
					m_vertices[index1].normalize();
					m_vertices[index2].normalize();
					
					color1.setValue(1,0,0,1);
					
				}
				if (e2Length>maxEdgeLength)
					maxEdgeLength = e2Length;
				{
					btScalar errorLength = averageEdgeLength-e2Length;
					m_vertices[index2] += e2.normalized()*errorLength*scaling;
					m_vertices[index0] -= e2.normalized()*errorLength*scaling;
					color2.setValue(1,0,0,1);

					m_vertices[index2].normalize();
					m_vertices[index0].normalize();
				}
			}
		}
		
		for (int p=0;p<poly->m_faces.size();p++)
		{
			for (int f=2;f<poly->m_faces[p].m_indices.size();f++)
			{
				btVector4 color0(0,0,1,1);
				btVector4 color1(0,0,1,1);
				btVector4 color2(0,0,1,1);

				int index0=poly->m_faces[p].m_indices[f-2];
				int index1=poly->m_faces[p].m_indices[f-1];
				int index2=poly->m_faces[p].m_indices[f];
				
				btVector3 v0 = m_vertices[index0];
				btVector3 v1 = m_vertices[index1];
				btVector3 v2 = m_vertices[index2];
				m_app->m_renderer->drawLine(&v0.x(),
											&v1.x(),
											color0,lineWidth);
				m_app->m_renderer->drawLine(&v1.x(),
											&v2.x(),
											color1,lineWidth);
				m_app->m_renderer->drawLine(&v2.x(),
											&v0.x(),
											color2,lineWidth);

			}
		//printf("maxEdgeLength=%f\n",maxEdgeLength);
		}
		for (int i=0;i<m_vertices.size();i++)
		{
			btVector4 color(1,0,0,1);
			
			m_app->m_renderer->drawPoint(m_vertices[i],color,pointSize);
			
		}
		
               
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
#endif //SPHERE_CREATION_H

