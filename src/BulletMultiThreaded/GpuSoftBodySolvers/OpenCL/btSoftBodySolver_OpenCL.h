/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_SOFT_BODY_SOLVER_OPENCL_H
#define BT_SOFT_BODY_SOLVER_OPENCL_H

#include "stddef.h" //for size_t
#include "vectormath/vmInclude.h"

#include "BulletSoftBody/btSoftBodySolvers.h"
#include "btSoftBodySolverBuffer_OpenCL.h"
#include "btSoftBodySolverLinkData_OpenCL.h"
#include "btSoftBodySolverVertexData_OpenCL.h"
#include "btSoftBodySolverTriangleData_OpenCL.h"


/**
 * SoftBody class to maintain information about a soft body instance
 * within a solver.
 * This data addresses the main solver arrays.
 */
class btOpenCLAcceleratedSoftBodyInterface
{
protected:
	/** Current number of vertices that are part of this cloth */
	int m_numVertices;
	/** Maximum number of vertices allocated to be part of this cloth */
	int m_maxVertices;
	/** Current number of triangles that are part of this cloth */
	int m_numTriangles;
	/** Maximum number of triangles allocated to be part of this cloth */
	int m_maxTriangles;
	/** Index of first vertex in the world allocated to this cloth */
	int m_firstVertex;
	/** Index of first triangle in the world allocated to this cloth */
	int m_firstTriangle;
	/** Index of first link in the world allocated to this cloth */
	int m_firstLink;
	/** Maximum number of links allocated to this cloth */
	int m_maxLinks;
	/** Current number of links allocated to this cloth */
	int m_numLinks;

	/** The actual soft body this data represents */
	btSoftBody *m_softBody;


public:
	btOpenCLAcceleratedSoftBodyInterface( btSoftBody *softBody ) :
	  m_softBody( softBody )
	{
		m_numVertices = 0;
		m_maxVertices = 0;
		m_numTriangles = 0;
		m_maxTriangles = 0;
		m_firstVertex = 0;
		m_firstTriangle = 0;
		m_firstLink = 0;
		m_maxLinks = 0;
		m_numLinks = 0;
	}
	int getNumVertices()
	{
		return m_numVertices;
	}

	int getNumTriangles()
	{
		return m_numTriangles;
	}

	int getMaxVertices()
	{
		return m_maxVertices;
	}

	int getMaxTriangles()
	{
		return m_maxTriangles;
	}

	int getFirstVertex()
	{
		return m_firstVertex;
	}

	int getFirstTriangle()
	{
		return m_firstTriangle;
	}

	// TODO: All of these set functions will have to do checks and
	// update the world because restructuring of the arrays will be necessary
	// Reasonable use of "friend"?
	void setNumVertices( int numVertices )
	{
		m_numVertices = numVertices;
	}	
	
	void setNumTriangles( int numTriangles )
	{
		m_numTriangles = numTriangles;
	}

	void setMaxVertices( int maxVertices )
	{
		m_maxVertices = maxVertices;
	}

	void setMaxTriangles( int maxTriangles )
	{
		m_maxTriangles = maxTriangles;
	}

	void setFirstVertex( int firstVertex )
	{
		m_firstVertex = firstVertex;
	}

	void setFirstTriangle( int firstTriangle )
	{
		m_firstTriangle = firstTriangle;
	}

	void setMaxLinks( int maxLinks )
	{
		m_maxLinks = maxLinks;
	}

	void setNumLinks( int numLinks )
	{
		m_numLinks = numLinks;
	}

	void setFirstLink( int firstLink )
	{
		m_firstLink = firstLink;
	}

	int getMaxLinks()
	{
		return m_maxLinks;
	}

	int getNumLinks()
	{
		return m_numLinks;
	}

	int getFirstLink()
	{
		return m_firstLink;
	}

	btSoftBody* getSoftBody()
	{
		return m_softBody;
	}

};


class btOpenCLSoftBodySolver : public btSoftBodySolver
{
private:

	btSoftBodyLinkDataOpenCL m_linkData;
	btSoftBodyVertexDataOpenCL m_vertexData;
	btSoftBodyTriangleDataOpenCL m_triangleData;

	/** Variable to define whether we need to update solver constants on the next iteration */
	bool m_updateSolverConstants;

	bool m_shadersInitialized;


	/** 
	 * Cloths owned by this solver.
	 * Only our cloths are in this array.
	 */
	btAlignedObjectArray< btOpenCLAcceleratedSoftBodyInterface * > m_softBodySet;

	/** Acceleration value to be applied to all non-static vertices in the solver. 
	 * Index n is cloth n, array sized by number of cloths in the world not the solver. 
	 */
	btAlignedObjectArray< Vectormath::Aos::Vector3 >	m_perClothAcceleration;
	btOpenCLBuffer<Vectormath::Aos::Vector3>			m_clPerClothAcceleration;

	/** Wind velocity to be applied normal to all non-static vertices in the solver. 
	 * Index n is cloth n, array sized by number of cloths in the world not the solver. 
	 */
	btAlignedObjectArray< Vectormath::Aos::Vector3 >	m_perClothWindVelocity;
	btOpenCLBuffer<Vectormath::Aos::Vector3>			m_clPerClothWindVelocity;

	/** Velocity damping factor */
	btAlignedObjectArray< float >						m_perClothDampingFactor;
	btOpenCLBuffer<float>								m_clPerClothDampingFactor;

	/** Velocity correction coefficient */
	btAlignedObjectArray< float >						m_perClothVelocityCorrectionCoefficient;
	btOpenCLBuffer<float>								m_clPerClothVelocityCorrectionCoefficient;

	/** Lift parameter for wind effect on cloth. */
	btAlignedObjectArray< float >						m_perClothLiftFactor;
	btOpenCLBuffer<float>								m_clPerClothLiftFactor;
	
	/** Drag parameter for wind effect on cloth. */
	btAlignedObjectArray< float >						m_perClothDragFactor;
	btOpenCLBuffer<float>								m_clPerClothDragFactor;

	/** Density of the medium in which each cloth sits */
	btAlignedObjectArray< float >						m_perClothMediumDensity;
	btOpenCLBuffer<float>								m_clPerClothMediumDensity;

	cl_kernel		prepareLinksKernel;
	cl_kernel		solvePositionsFromLinksKernel;
	cl_kernel		updateConstantsKernel;
	cl_kernel		integrateKernel;
	cl_kernel		addVelocityKernel;
	cl_kernel		updatePositionsFromVelocitiesKernel;
	cl_kernel		updateVelocitiesFromPositionsWithoutVelocitiesKernel;
	cl_kernel		updateVelocitiesFromPositionsWithVelocitiesKernel;
	cl_kernel		vSolveLinksKernel;
	cl_kernel		resetNormalsAndAreasKernel;
	cl_kernel		normalizeNormalsAndAreasKernel;
	cl_kernel		updateSoftBodiesKernel;
	cl_kernel		outputToVertexArrayWithNormalsKernel;
	cl_kernel		outputToVertexArrayWithoutNormalsKernel;

	cl_kernel		outputToVertexArrayKernel;
	cl_kernel		applyForcesKernel;
	cl_kernel		collideSphereKernel;
	cl_kernel		collideCylinderKernel;

	cl_command_queue	m_cqCommandQue;
	cl_context			m_cxMainContext;

	size_t				m_defaultWorkGroupSize;


	/**
	 * Compile a compute shader kernel from a string and return the appropriate cl_kernel object.
	 */
	cl_kernel compileCLKernelFromString( const char *shaderString, const char *shaderName );

	bool buildShaders();

	void resetNormalsAndAreas( int numVertices );

	void normalizeNormalsAndAreas( int numVertices );

	void executeUpdateSoftBodies( int firstTriangle, int numTriangles );
	
	Vectormath::Aos::Vector3 ProjectOnAxis( const Vectormath::Aos::Vector3 &v, const Vectormath::Aos::Vector3 &a );

	void ApplyClampedForce( float solverdt, const Vectormath::Aos::Vector3 &force, const Vectormath::Aos::Vector3 &vertexVelocity, float inverseMass, Vectormath::Aos::Vector3 &vertexForce );
	
	btOpenCLAcceleratedSoftBodyInterface *findSoftBodyInterface( const btSoftBody* const softBody );

	virtual void applyForces( float solverdt );

	/**
	 * Integrate motion on the solver.
	 */
	virtual void integrate( float solverdt );

	void updateConstants( float timeStep );

	float computeTriangleArea( 
		const Vectormath::Aos::Point3 &vertex0,
		const Vectormath::Aos::Point3 &vertex1,
		const Vectormath::Aos::Point3 &vertex2 );


	//////////////////////////////////////
	// Kernel dispatches
	void prepareLinks();

	void solveLinksForVelocity( int startLink, int numLinks, float kst );

	void updatePositionsFromVelocities( float solverdt );

	void solveLinksForPosition( int startLink, int numLinks, float kst, float ti );
	
	void updateVelocitiesFromPositionsWithVelocities( float isolverdt );

	void updateVelocitiesFromPositionsWithoutVelocities( float isolverdt );

	// End kernel dispatches
	/////////////////////////////////////
	

public:
	btOpenCLSoftBodySolver(cl_command_queue queue,cl_context	ctx);

	virtual ~btOpenCLSoftBodySolver();




	virtual btSoftBodyLinkData &getLinkData();

	virtual btSoftBodyVertexData &getVertexData();

	virtual btSoftBodyTriangleData &getTriangleData();




	virtual bool checkInitialized();

	virtual void updateSoftBodies( );

	virtual void optimize( btAlignedObjectArray< btSoftBody * > &softBodies );

	virtual void solveConstraints( float solverdt );

	virtual void predictMotion( float solverdt );

	virtual void copySoftBodyToVertexBuffer( const btSoftBody *const softBody, btVertexBufferDescriptor *vertexBuffer );

	virtual void	setDefaultWorkgroupSize(size_t workGroupSize)
	{
		m_defaultWorkGroupSize = workGroupSize;
	}
	virtual size_t	getDefaultWorkGroupSize() const
	{
		return m_defaultWorkGroupSize;
	}
}; // btOpenCLSoftBodySolver

#endif // #ifndef BT_SOFT_BODY_SOLVER_OPENCL_H
