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

#ifndef BT_SOFT_BODY_SOLVERS_H
#define BT_SOFT_BODY_SOLVERS_H

#include "BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h"


class btSoftBodyTriangleData;
class btSoftBodyLinkData;
class btSoftBodyVertexData;
class btVertexBufferDescriptor;
class btCollisionObject;
class btSoftBody;


class btSoftBodySolver
{

protected:
	int m_numberOfPositionIterations;
	int m_numberOfVelocityIterations;
	// Simulation timescale
	float m_timeScale;
	
public:
	btSoftBodySolver() :
		m_numberOfPositionIterations( 10 ),
		m_timeScale( 1 )
	{
		m_numberOfVelocityIterations = 0;
		m_numberOfPositionIterations = 5;
	}

	virtual ~btSoftBodySolver()
	{
	}

#if 0
	/** Acceleration for all cloths in the solver. Can be used to efficiently apply gravity. */
	virtual void setPerClothAcceleration( int clothIdentifier, Vectormath::Aos::Vector3 acceleration ) = 0;

	/** A wind velocity applied normal to the cloth for all cloths in the solver. */
	virtual void setPerClothWindVelocity( int clothIdentifier, Vectormath::Aos::Vector3 windVelocity ) = 0;

	/** Set the density of the medium a given cloth is situated in. This could be air or possibly water. */
	virtual void setPerClothMediumDensity( int clothIdentifier, float mediumDensity ) = 0;		

	/** A damping factor specific to each cloth applied for all cloths. */
	virtual void setPerClothDampingFactor( int clothIdentifier, float dampingFactor ) = 0;

	/** A damping factor specific to each cloth applied for all cloths. */
	virtual void setPerClothVelocityCorrectionCoefficient( int clothIdentifier, float velocityCorrectionCoefficient ) = 0;

	/** Lift parameter for wind action on cloth. */
	virtual void setPerClothLiftFactor( int clothIdentifier, float liftFactor ) = 0;

	/** Drag parameter for wind action on cloth. */
	virtual void setPerClothDragFactor( int clothIdentifier, float dragFactor ) = 0;

	/**
	 * Add a velocity to all soft bodies in the solver - useful for doing world-wide velocities such as a change due to gravity 
	 * Only add a velocity to nodes with a non-zero inverse mass.
	 */
	virtual void addVelocity( Vectormath::Aos::Vector3 velocity ) = 0;
#endif



	/** Ensure that this solver is initialized. */
	virtual bool checkInitialized() = 0;

	/** Optimize soft bodies in this solver. */
	virtual void optimize( btAlignedObjectArray< btSoftBody * > &softBodies ) = 0;

	/** Predict motion of soft bodies into next timestep */
	virtual void predictMotion( float solverdt ) = 0;

	/** Solve constraints for a set of soft bodies */
	virtual void solveConstraints( float solverdt ) = 0;

	/** Perform necessary per-step updates of soft bodies such as recomputing normals and bounding boxes */
	virtual void updateSoftBodies() = 0;

	/** Output current computed vertex data to the vertex buffers for all cloths in the solver. */
	virtual void copySoftBodyToVertexBuffer( const btSoftBody * const softBody, btVertexBufferDescriptor *vertexBuffer ) = 0;



	/** Set the number of velocity constraint solver iterations this solver uses. */
	virtual void setNumberOfPositionIterations( int iterations )
	{
		m_numberOfPositionIterations = iterations;
	}

	/** Get the number of velocity constraint solver iterations this solver uses. */
	virtual int getNumberOfPositionIterations()
	{
		return m_numberOfPositionIterations;
	}

	/** Set the number of velocity constraint solver iterations this solver uses. */
	virtual void setNumberOfVelocityIterations( int iterations )
	{
		m_numberOfVelocityIterations = iterations;
	}

	/** Get the number of velocity constraint solver iterations this solver uses. */
	virtual int getNumberOfVelocityIterations()
	{
		return m_numberOfVelocityIterations;
	}

	/** Return the timescale that the simulation is using */
	float getTimeScale()
	{
		return m_timeScale;
	}

#if 0
	/**
	 * Add a collision object to be used by the indicated softbody.
	 */
	virtual void addCollisionObjectForSoftBody( int clothIdentifier, btCollisionObject *collisionObject ) = 0;
#endif
};


#endif // #ifndef BT_SOFT_BODY_SOLVERS_H
