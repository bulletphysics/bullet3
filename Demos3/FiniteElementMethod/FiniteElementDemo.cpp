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

#include "MyFemMesh.h"
#include <OpenTissue/core/math/math_basic_types.h>
#include <OpenTissue/dynamics/fem/fem.h>
#include <OpenTissue/core/containers/t4mesh/util/t4mesh_block_generator.h>
#include "LinearMath/btAlignedObjectArray.h"

//typedef OpenTissue::math::BasicMathTypes<float,size_t>    math_types;
typedef OpenTissue::math::BasicMathTypes<double,size_t>    math_types;
typedef OpenTissue::fem::Mesh<math_types>                  mesh_type;
typedef math_types::vector3_type                           vector3_type;
typedef math_types::real_type                              real_type;




struct FiniteElementDemoInternalData
{
	mesh_type		m_mesh1;

	bool			m_stiffness_warp_on;   ///< Boolean value indicating whether stiffness warping is turned on or off.

	bool			m_collideGroundPlane;

	bool			m_fixNodes;

	real_type		m_gravity;

	real_type		m_young;// = 500000;
	real_type		m_poisson;// = 0.33;
	real_type		m_density;// = 1000;

	//--- infinite m_c_yield plasticity settings means that plasticity is turned off
	real_type		m_c_yield;// = .04;  //--- should be less than maximum expected elastic strain in order to see effect (works as a minimum).
	real_type		m_c_creep;// = .20;  //--- controls how fast the plasticity effect occurs (it is a rate-like control).
	real_type		m_c_max;// = 0.2;    //--- This is maximum allowed plasticity strain (works as a maximum).
	double			m_damp;

	FiniteElementDemoInternalData()
	{
		m_gravity = 9.81;
		m_collideGroundPlane = true;
		m_stiffness_warp_on= true;
		m_young = 500000;//47863;//100000;
		m_poisson = 0.33;
		
		m_density = 1054.00;//1000;
		//--- infinite m_c_yield plasticity settings means that plasticity is turned off
		m_c_yield = 0;//0.03;//.04;  //--- should be less than maximum expected elastic strain in order to see effect (works as a minimum).
		m_c_creep = 0;//0.20;//.20;  //--- controls how fast the plasticity effect occurs (it is a rate-like control).
		m_c_max = 1e30f;//0.2;    //--- This is maximum allowed plasticity strain (works as a maximum).
		m_damp=0.2f;
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
}
    
void    FiniteElementDemo::initPhysics()
{
	{
		
		OpenTissue::t4mesh::generate_blocks(10,3,3,0.1,0.1,0.1,m_data->m_mesh1);
		
		for (int n=0;n<m_data->m_mesh1.m_nodes.size();n++)
		{
			m_data->m_mesh1.m_nodes[n].m_coord(1)+=0.1f;
			m_data->m_mesh1.m_nodes[n].m_model_coord = m_data->m_mesh1.m_nodes[n].m_coord;

		}
		OpenTissue::fem::init(m_data->m_mesh1,m_data->m_young,m_data->m_poisson,m_data->m_density,m_data->m_c_yield,m_data->m_c_creep,m_data->m_c_max);

	}
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
	//normal gravity
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
		vector3_type gravity = vector3_type(0.0, 0.0 , 0.0);
		gravity(m_app->getUpAxis()) = -(m_data->m_mesh1.m_nodes[n].m_mass * m_data->m_gravity);
		m_data->m_mesh1.m_nodes[n].m_f_external =gravity;
		//m_data->m_mesh1.m_nodes[n].m_velocity.clear();
	}

	OpenTissue::fem::simulate(m_data->m_mesh1,dt,m_data->m_stiffness_warp_on,m_data->m_damp);//,0.1,20,20);//,1.0,20,20);

}
void	FiniteElementDemo::renderScene()
{
	m_app->m_renderer->renderScene();
}

void	FiniteElementDemo::physicsDebugDraw()
{
#if 0
	btVector3 xUnit(1,0,0);
	btVector3 yUnit(0,1,0);
	btVector3 zUnit(0,0,1);

	btScalar lineWidth=3;

	btQuaternion rotAroundX(xUnit,m_x);
	btQuaternion rotAroundY(yUnit,m_y);
	btQuaternion rotAroundZ(zUnit,m_z);

	btScalar radius=0.5;
	btVector3 toX=radius*quatRotate(rotAroundX,yUnit);
	btVector3 toY=radius*quatRotate(rotAroundY,xUnit);
	btVector3 toZ=radius*quatRotate(rotAroundZ,xUnit);
		
	m_app->m_renderer->drawLine(xUnit+toX+quatRotate(rotAroundX,btVector3(0,0.1,-0.2)),xUnit+toX,xUnit,lineWidth);
	m_app->m_renderer->drawLine(xUnit+toX+quatRotate(rotAroundX,btVector3(0,-0.2,-0.2)),xUnit+toX,xUnit,lineWidth);
	//draw the letter 'x' on the x-axis
	//m_app->m_renderer->drawLine(xUnit-0.1*zUnit+0.1*yUnit,xUnit+0.1*zUnit-0.1*yUnit,xUnit,lineWidth);
	//m_app->m_renderer->drawLine(xUnit+0.1*zUnit+0.1*yUnit,xUnit-0.1*zUnit-0.1*yUnit,xUnit,lineWidth);

	m_app->m_renderer->drawLine(xUnit+toX+quatRotate(rotAroundX,btVector3(0,-0.2,-0.2)),xUnit+toX,xUnit,lineWidth);

	m_app->m_renderer->drawLine(yUnit+toY+quatRotate(rotAroundY,btVector3(-0.2,0,0.2)),yUnit+toY,yUnit,lineWidth);
	m_app->m_renderer->drawLine(yUnit+toY+quatRotate(rotAroundY,btVector3(0.1,0,0.2)),yUnit+toY,yUnit,lineWidth);
	m_app->m_renderer->drawLine(zUnit+toZ+quatRotate(rotAroundZ,btVector3(0.1,-0.2,0)),zUnit+toZ,zUnit,lineWidth);
	m_app->m_renderer->drawLine(zUnit+toZ+quatRotate(rotAroundZ,btVector3(-0.2,-0.2,0)),zUnit+toZ,zUnit,lineWidth);
#endif

	 //template <typename point_container, typename t4mesh >
    //inline void DrawPointsT4Mesh( point_container const& points, t4mesh const& mesh, double const& scale = 0.95, bool wireframe = false)
    {
      //geometry::Tetrahedron<math::default_math_types> T; // From OpenTissue/core/geometry/geometry_tetrahederon.h
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

