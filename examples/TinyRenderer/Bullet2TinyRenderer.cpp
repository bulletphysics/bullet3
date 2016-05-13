

#include "TinyRenderer.h"
#include <stdio.h>
#include "LinearMath/btHashMap.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"

class Bullet2TinyRenderer
{
    
	btAlignedObjectArray<TinyRenderObjectData*> m_swRenderObjects;
	
	btHashMap<btHashPtr,int> m_colObject2gfxIndex;
	btHashMap<btHashPtr,int> m_colShape2gfxIndex;
	
	
	int m_swWidth;
	int m_swHeight;
	TGAImage m_rgbColorBuffer;
	
	b3AlignedObjectArray<float> m_depthBuffer;
	int m_textureHandle;
	unsigned char*	m_image;
	
    
public:
    Bullet2TinyRenderer (int swWidth, int swHeight):
    m_swWidth(swWidth),
    m_swHeight(swHeight),
	m_rgbColorBuffer(swWidth,swHeight,TGAImage::RGB)	
    {

		m_depthBuffer.resize(swWidth*swHeight);
		m_image=new unsigned char[m_swWidth*m_swHeight*4];
		
    }
	
	virtual ~Bullet2TinyRenderer()
    {
        for (int i=0;i<m_swRenderObjects.size();i++)
        {
            TinyRenderObjectData* d = m_swRenderObjects[i];
            delete d;
        }
    }
    
	void clearBuffers(TGAColor& clearColor)
	{
		for(int y=0;y<m_swHeight;++y)
		{
			for(int x=0;x<m_swWidth;++x)
			{
				m_rgbColorBuffer.set(x,y,clearColor);
				m_depthBuffer[x+y*m_swWidth] = -1e30f;
			}
		}
		
	}
    
	const TGAImage& getFrameBuffer() const
	{
		return m_rgbColorBuffer;
	}
	
  
    
	virtual void createCollisionObjectGraphicsObject(btCollisionObject* obj,const btVector3& color)
	{
		int* colIndexPtr = m_colObject2gfxIndex[obj];
		int* shapeIndexPtr = m_colShape2gfxIndex[obj->getCollisionShape()];
		
		
		
		//if (colIndex>=0 && shapeIndex>=0)
		//{
		//	m_col2gfxIndex.insert(colIndex,shapeIndex);
		//}
		
	}
	
   /*	virtual int registerGraphicsShape(const float* vertices, int numvertices, const int* indices, int numIndices)
   	{
   	    
   	    
   	    if (shapeIndex>=0)
        {
            TinyRenderObjectData* swObj = new TinyRenderObjectData(m_swWidth,m_swHeight,m_rgbColorBuffer,m_depthBuffer);
            swObj->registerMeshShape(vertices,numvertices,indices,numIndices);
			//swObj->createCube(1,1,1);
            m_swRenderObjects.insert(shapeIndex,swObj);
        }
   	    return shapeIndex;
   	}
*/
	virtual void render(const btDiscreteDynamicsWorld* rbWorld, float viewMat[16])
	{
		//clear the color buffer
		TGAColor clearColor;
		clearColor.bgra[0] = 255;
		clearColor.bgra[1] = 255;
		clearColor.bgra[2] = 255;
		clearColor.bgra[3] = 255;
		
		clearBuffers(clearColor);
		
		ATTRIBUTE_ALIGNED16(float modelMat[16]);
		
		
		for (int i=0;i<rbWorld->getNumCollisionObjects();i++)
		{
			btCollisionObject* colObj = rbWorld->getCollisionObjectArray()[i];
			
			int* colObjIndexPtr = m_colObject2gfxIndex[colObj];
			
			if (colObjIndexPtr)
			{
				int colObjIndex = *colObjIndexPtr;
				TinyRenderObjectData* renderObj = m_swRenderObjects[colObjIndex];
								
				//sync the object transform
				const btTransform& tr = colObj->getWorldTransform();
				tr.getOpenGLMatrix(modelMat);
				
				for (int i=0;i<4;i++)
				{
					for (int j=0;j<4;j++)
					{
						renderObj->m_modelMatrix[i][j] = modelMat[i+4*j];
						renderObj->m_viewMatrix[i][j] = viewMat[i+4*j];
					}
				}
				TinyRenderer::renderObject(*renderObj);
			}
		}
		
		for(int y=0;y<m_swHeight;++y)
		{
			unsigned char*	pi=m_image+(y)*m_swWidth*3;
			for(int x=0;x<m_swWidth;++x)
			{
				
				const TGAColor& color = getFrameBuffer().get(x,y);
				pi[0] = color.bgra[2];
				pi[1] = color.bgra[1];
				pi[2] = color.bgra[0];
				pi+=3;
				
			}
		} 
		
		static int counter=0;
		counter++;
		if (counter>10)
		{
			counter=0;
			getFrameBuffer().write_tga_file("/Users/erwincoumans/develop/bullet3/framebuf.tga",true);
		}
		
	}

};

