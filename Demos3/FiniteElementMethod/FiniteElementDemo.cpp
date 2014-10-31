/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2011-2014 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///the Finite Element Method is extracted from the OpenTissue library,
///under the zlib license: http://www.opentissue.org/mediawiki/index.php/Main_Page


#include "FiniteElementDemo.h"
#include "OpenGLWindow/CommonRenderInterface.h"
#include "LinearMath/btQuaternion.h"
//#include "OpenGLWindow/ShapeData.h"

#include "MyFemMesh.h"
#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/dynamics/fem/fem.h>
#include <OpenTissue/core/containers/t4mesh/util/t4mesh_block_generator.h>
#include "LinearMath/btAlignedObjectArray.h"
#include "Bullet3AppSupport/CommonParameterInterface.h"

//typedef OpenTissue::math::BasicMathTypes<float,size_t>    math_types;
typedef OpenTissue::math::BasicMathTypes<double,size_t>    math_types;
typedef OpenTissue::fem::Mesh<math_types>                  mesh_type;
typedef math_types::vector3_type                           vector3_type;
typedef math_types::real_type                              real_type;


static int fixedNodes = 1;

struct FiniteElementDemoInternalData
{
	mesh_type		m_mesh1;

	bool			m_stiffness_warp_on;   ///< Boolean value indicating whether stiffness warping is turned on or off.

	bool			m_collideGroundPlane;

	bool			m_fixNodes;

	real_type		m_gravity;

	btScalar		m_young;// = 500000;
	btScalar           m_poisson;// = 0.33;
	real_type		m_density;// = 1000;

	//--- infinite m_c_yield plasticity settings means that plasticity is turned off
	real_type		m_c_yield;// = .04;  //--- should be less than maximum expected elastic strain in order to see effect (works as a minimum).
	real_type		m_c_creep;// = .20;  //--- controls how fast the plasticity effect occurs (it is a rate-like control).
	real_type		m_c_max;// = 0.2;    //--- This is maximum allowed plasticity strain (works as a maximum).
	double			m_damp;
    int m_tetrahedralMeshRenderIndex;
    
	FiniteElementDemoInternalData()
	{
        m_stiffness_warp_on= true;
        m_collideGroundPlane = true;
        m_fixNodes = fixedNodes==1;
        fixedNodes=1-fixedNodes;
		m_gravity = 9.81;
		m_young = 500000;//47863;//100000;
		m_poisson = 0.33;
		
		m_density = 1054.00;//1000;
		//--- infinite m_c_yield plasticity settings means that plasticity is turned off
		m_c_yield = 0;//0.03;//.04;  //--- should be less than maximum expected elastic strain in order to see effect (works as a minimum).
		m_c_creep = 0;//0.20;//.20;  //--- controls how fast the plasticity effect occurs (it is a rate-like control).
		m_c_max = 1e30f;//0.2;    //--- This is maximum allowed plasticity strain (works as a maximum).
		m_damp=0.2f;
        m_tetrahedralMeshRenderIndex=-1;
	}

};

FiniteElementDemo::FiniteElementDemo(CommonGraphicsApp* app)
:m_app(app),
m_x(0),
m_y(0),
m_z(0)
{
	m_app->setUpAxis(2);
	m_data = new FiniteElementDemoInternalData;
    

}
FiniteElementDemo::~FiniteElementDemo()
{
	delete m_data;
    m_app->m_renderer->enableBlend(false);
    m_app->m_renderer->removeAllInstances();
}

struct MyTetVertex
{
    float x,y,z,w;
    float nx,ny,nz;
    float u,v;
};

void    FiniteElementDemo::initPhysics()
{
	{
		
		OpenTissue::t4mesh::generate_blocks(10,3,3,0.1,0.1,0.1,m_data->m_mesh1);
		
		for (int n=0;n<m_data->m_mesh1.m_nodes.size();n++)
		{
			m_data->m_mesh1.m_nodes[n].m_coord(m_app->getUpAxis())+=.5f;
			m_data->m_mesh1.m_nodes[n].m_model_coord = m_data->m_mesh1.m_nodes[n].m_coord;

		}
		OpenTissue::fem::init(m_data->m_mesh1,double(m_data->m_young),double(m_data->m_poisson),m_data->m_density,m_data->m_c_yield,m_data->m_c_creep,m_data->m_c_max);

       
        
	}
    
    {
        
        SliderParams slider("Young",&m_data->m_young);
//        slider.m_showValues = false;
        slider.m_minVal=50000;
        slider.m_maxVal=1000000;
        m_app->m_parameterInterface->registerSliderFloatParameter(slider);
    }
    
    {
        
        SliderParams slider("Poisson",&m_data->m_poisson);
        //        slider.m_showValues = false;
        slider.m_minVal=0.01;
        slider.m_maxVal=0.49;
        m_app->m_parameterInterface->registerSliderFloatParameter(slider);
    }
    
    {
        
     
        
        
        int strideInBytes = 9*sizeof(float);
        int numVertices =m_data->m_mesh1.m_nodes.size();
        
        btAlignedObjectArray<MyTetVertex> verts;
        verts.resize(numVertices);
        for (int n=0;n<m_data->m_mesh1.m_nodes.size();n++)
        {
            verts[n].x = m_data->m_mesh1.m_nodes[n].m_coord(0);
             verts[n].y = m_data->m_mesh1.m_nodes[n].m_coord(1);
             verts[n].z = m_data->m_mesh1.m_nodes[n].m_coord(2);
            verts[n].w = 1;
            verts[n].nx = 0;
            verts[n].ny = 1;
            verts[n].nz = 0;
            verts[n].u = 0.5;
            verts[n].v = 0.4;
            
        }
        btAlignedObjectArray<int> indices;
        for (int t=0;t<m_data->m_mesh1.m_tetrahedra.size();t++)
        {
            int index0 =m_data->m_mesh1.m_tetrahedra[t].m_nodes[0];
            int index1 =m_data->m_mesh1.m_tetrahedra[t].m_nodes[1];
            int index2 =m_data->m_mesh1.m_tetrahedra[t].m_nodes[2];
            int index3 =m_data->m_mesh1.m_tetrahedra[t].m_nodes[3];
            indices.push_back(index0);         indices.push_back(index1);           indices.push_back(index2);
            indices.push_back(index2);         indices.push_back(index1);           indices.push_back(index3);
            indices.push_back(index1);         indices.push_back(index0);           indices.push_back(index3);
            indices.push_back(index0);         indices.push_back(index2);           indices.push_back(index3);
            
        }
        
        m_data->m_tetrahedralMeshRenderIndex = m_app->m_renderer->registerShape(&verts[0].x,verts.size(),&indices[0],indices.size());
        
        float pos[4] = {0,0,0,1};
        float orn[4] = {0,0,0,1};
        float color[4] = {0,1,1,1};
        float scaling[4] = {1,1,1,1};
        m_app->m_renderer->registerGraphicsInstance(m_data->m_tetrahedralMeshRenderIndex,pos,orn,color,scaling);
            }
    
    {
        //ground shape
        btVector3 cubeHalfExtents(10,10,10);
        cubeHalfExtents[m_app->getUpAxis()] = 0.01;
        int cubeIn = m_app->registerCubeShape(cubeHalfExtents[0],cubeHalfExtents[1],cubeHalfExtents[2]);
        
        float pos[4] = {0,0,0,1};
        pos[m_app->getUpAxis()]=-0.02;
        float orn[4] = {0,0,0,1};
        float color[4] = {0,1,1,1};
        float scaling[4] = {1,1,1,1};
        m_app->m_renderer->registerGraphicsInstance(cubeIn,pos,orn,color,scaling);
       

    }
     m_app->m_renderer->writeTransforms();
}
void    FiniteElementDemo::exitPhysics()
{
        
}
void	FiniteElementDemo::stepSimulation(float deltaTime)
{
    m_x+=0.01f;
    m_y+=0.01f;
	m_z+=0.01f;
	double dt = 1./60.;//double (deltaTime);
    double poisson =m_data->m_poisson;
    OpenTissue::fem::init(m_data->m_mesh1,double(m_data->m_young),poisson,m_data->m_density,m_data->m_c_yield,m_data->m_c_creep,m_data->m_c_max);

    
	for (int n=0;n<m_data->m_mesh1.m_nodes.size();n++)
	{
		
	
		if (m_data->m_fixNodes)
		{
			if (m_data->m_mesh1.m_nodes[n].m_model_coord(0) < 0.01)
			{
				m_data->m_mesh1.m_nodes[n].m_fixed = true;
			}
		} else
		{
			if (m_data->m_mesh1.m_nodes[n].m_model_coord(0) < 0.01)
			{
				m_data->m_mesh1.m_nodes[n].m_fixed = false;
			}
		}
		if (m_data->m_collideGroundPlane && m_data->m_mesh1.m_nodes[n].m_coord(m_app->getUpAxis())<0.f)
        {
            float depth = -m_data->m_mesh1.m_nodes[n].m_coord(m_app->getUpAxis());
            if (depth>0.1)
                    depth=0.1;

            m_data->m_mesh1.m_nodes[n].m_f_external(m_app->getUpAxis()) = depth*1000;
            
			if (m_data->m_mesh1.m_nodes[n].m_velocity(m_app->getUpAxis()) < 0.f)
                        {
                                m_data->m_mesh1.m_nodes[n].m_velocity(m_app->getUpAxis())=0.f;
                        }
		
			int frictionAxisA=0;
			int frictionAxisB=2;
			if (m_app->getUpAxis()==1)
			{
				frictionAxisA=0;
				frictionAxisB=2;
			} else
			{
				frictionAxisA=0;
				frictionAxisB=1;
			}	
            m_data->m_mesh1.m_nodes[n].m_velocity(frictionAxisA)=0.f;
            m_data->m_mesh1.m_nodes[n].m_velocity(frictionAxisB)=0.f;

		} else
		{
            vector3_type gravity = vector3_type(0.0, 0.0 , 0.0);
            gravity(m_app->getUpAxis()) = -(m_data->m_mesh1.m_nodes[n].m_mass * m_data->m_gravity);
            m_data->m_mesh1.m_nodes[n].m_f_external =gravity;
		}	
		//m_data->m_mesh1.m_nodes[n].m_velocity.clear();
	}

	OpenTissue::fem::simulate(m_data->m_mesh1,dt,m_data->m_stiffness_warp_on,m_data->m_damp);//,0.1,20,20);//,1.0,20,20);

}
void	FiniteElementDemo::renderScene()
{
    {
        int strideInBytes = 9*sizeof(float);
        int numVertices =m_data->m_mesh1.m_nodes.size();
        
        btAlignedObjectArray<MyTetVertex> verts;
        verts.resize(numVertices);
        for (int n=0;n<m_data->m_mesh1.m_nodes.size();n++)
        {
            verts[n].x = m_data->m_mesh1.m_nodes[n].m_coord(0);
            verts[n].y = m_data->m_mesh1.m_nodes[n].m_coord(1);
            verts[n].z = m_data->m_mesh1.m_nodes[n].m_coord(2);
            verts[n].w = 1;
            verts[n].nx = 0;
            verts[n].ny = 1;
            verts[n].nz = 0;
            verts[n].u = 0.5;
            verts[n].v = 0.4;
            
        }
        btAlignedObjectArray<int> indices;
        for (int t=0;t<m_data->m_mesh1.m_tetrahedra.size();t++)
        {
            int index0 =m_data->m_mesh1.m_tetrahedra[t].m_nodes[0];
            int index1 =m_data->m_mesh1.m_tetrahedra[t].m_nodes[1];
            int index2 =m_data->m_mesh1.m_tetrahedra[t].m_nodes[2];
            int index3 =m_data->m_mesh1.m_tetrahedra[t].m_nodes[3];
            indices.push_back(index0);         indices.push_back(index1);           indices.push_back(index2);
            indices.push_back(index2);         indices.push_back(index1);           indices.push_back(index3);
            indices.push_back(index1);         indices.push_back(index0);           indices.push_back(index3);
            indices.push_back(index0);         indices.push_back(index2);           indices.push_back(index3);
            
        }
        
        m_app->m_renderer->updateShape(m_data->m_tetrahedralMeshRenderIndex,&verts[0].x);
        
    }
	m_app->m_renderer->renderScene();
}

void	FiniteElementDemo::physicsDebugDraw()
{
    {
		btAlignedObjectArray<btVector3FloatData> m_linePoints;
		btAlignedObjectArray<unsigned int> m_lineIndices;

		//geometry::Tetrahedron<math::default_math_types> tet;
		for (int t=0;t<m_data->m_mesh1.m_tetrahedra.size();t++)
		{
			vector3_type v0d = m_data->m_mesh1.m_nodes[m_data->m_mesh1.m_tetrahedra[t].m_nodes[0]].m_coord;
			vector3_type v1d = m_data->m_mesh1.m_nodes[m_data->m_mesh1.m_tetrahedra[t].m_nodes[1]].m_coord;
			vector3_type v2d = m_data->m_mesh1.m_nodes[m_data->m_mesh1.m_tetrahedra[t].m_nodes[2]].m_coord;
			vector3_type v3d = m_data->m_mesh1.m_nodes[m_data->m_mesh1.m_tetrahedra[t].m_nodes[3]].m_coord;
			btVector3 v0(v0d(0),v0d(1),v0d(2));
			btVector3 v1(v1d(0),v1d(1),v1d(2));
			btVector3 v2(v2d(0),v2d(1),v2d(2));
			btVector3 v3(v3d(0),v3d(1),v3d(2));
			btVector3FloatData vf0,vf1,vf2,vf3;
			v0.serializeFloat(vf0);
			v1.serializeFloat(vf1);
			v2.serializeFloat(vf2);
			v3.serializeFloat(vf3);
			unsigned int baseIndex = m_linePoints.size();
			m_linePoints.push_back(vf0);
			m_linePoints.push_back(vf1);
			m_linePoints.push_back(vf2);
			m_linePoints.push_back(vf3);
			m_lineIndices.push_back(baseIndex+0);
			m_lineIndices.push_back(baseIndex+1);
			m_lineIndices.push_back(baseIndex+0);
			m_lineIndices.push_back(baseIndex+2);
			m_lineIndices.push_back(baseIndex+0);
			m_lineIndices.push_back(baseIndex+3);

			m_lineIndices.push_back(baseIndex+1);
			m_lineIndices.push_back(baseIndex+2);
			m_lineIndices.push_back(baseIndex+2);
			m_lineIndices.push_back(baseIndex+3);
			m_lineIndices.push_back(baseIndex+1);
			m_lineIndices.push_back(baseIndex+3);
      }

		float debugColor[4]={0,0,0.4,1};
		m_app->m_renderer->drawLines(&m_linePoints[0].m_floats[0],debugColor,
                                                     m_linePoints.size(),sizeof(btVector3FloatData),
                                                     &m_lineIndices[0],
                                                     m_lineIndices.size(),
                                                     1);

		
    };

}
bool	FiniteElementDemo::mouseMoveCallback(float x,float y)
{
	return false;   
}
bool	FiniteElementDemo::mouseButtonCallback(int button, int state, float x, float y)
{
    return false;   
}
bool	FiniteElementDemo::keyboardCallback(int key, int state)
{
    return false;   
}

