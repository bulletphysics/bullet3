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

#ifndef BT_DYNAMICS_WORLD_H
#define BT_DYNAMICS_WORLD_H

#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"

class btTypedConstraint;
class btActionInterface;
class btConstraintSolver;
class btDynamicsWorld;


/// @brief A function pointer to a callback which is called each tick.
typedef void (*btInternalTickCallback)(btDynamicsWorld *world, btScalar timeStep);

/// @brief Specifies the type of a btDynamicsWorld.
enum btDynamicsWorldType
{
	BT_SIMPLE_DYNAMICS_WORLD = 1,      ///< @brief btSimpleDynamicsWorld.
	BT_DISCRETE_DYNAMICS_WORLD = 2,    ///< @brief btDiscreteDynamicsWorld.
	BT_CONTINUOUS_DYNAMICS_WORLD = 3,
	BT_SOFT_RIGID_DYNAMICS_WORLD = 4,  ///< @brief btSoftRigidDynamicsWorld.
	BT_GPU_DYNAMICS_WORLD = 5
};

/// @brief The interface class for several dynamics implementations.
class btDynamicsWorld : public btCollisionWorld
{

protected:
        /// @brief The callback which is executed immediately after a tick (substep) occurs.
		btInternalTickCallback m_internalTickCallback;

		/// @brief The callback which is executed right before a tick (substep) occurs.
		btInternalTickCallback m_internalPreTickCallback;

		/// @brief The user pointer.
		void*	m_worldUserInfo;

		/// @brief Information about the solver.
		btContactSolverInfo	m_solverInfo;

public:

        /// @brief Creates a dynamics world.
        /// @param dispatcher The dispatcher.
        /// @param broadphase The broad phase algorithm.
        /// @param collisionConfiguration The collision configuration.
		btDynamicsWorld(btDispatcher* dispatcher,btBroadphaseInterface* broadphase,btCollisionConfiguration* collisionConfiguration)
		:btCollisionWorld(dispatcher,broadphase,collisionConfiguration), m_internalTickCallback(0),m_internalPreTickCallback(0), m_worldUserInfo(0)
		{
		}

		/// @brief Destroys the dynamics world.
		virtual ~btDynamicsWorld()
		{
		}

		/// @brief Advances the simulation in time.
		/// @param timeStep The total amount time to advance into the future, preferably in seconds.
		/// @param maxSubSteps The maximum number of substeps to advance.
		/// @param fixedTimeStep The time which timeStep will be subdivided into. This can be thought of as the granularity of the time step. Bullet will continue advancing time
		///                      by a factor of fixedTimeStep until a total amount of timeStep has passed.
		/// @note You can disable subdividing the timestep/substepping by passing maxSubSteps=0 as the second argument, but in that can you will have to keep timeStep constant.
		virtual int		stepSimulation( btScalar timeStep,int maxSubSteps=1, btScalar fixedTimeStep=btScalar(1.)/btScalar(60.))=0;

		virtual void	debugDrawWorld() = 0;


		/// @brief Adds a constraint to the world.
		/// @param constraint The constraint.
		virtual void	addConstraint(btTypedConstraint* constraint, bool disableCollisionsBetweenLinkedBodies=false)
		{
			(void)constraint; (void)disableCollisionsBetweenLinkedBodies;
		}

		/// @brief Removes a constraint from the world.
		/// @param constraint The constraint.
		virtual void	removeConstraint(btTypedConstraint* constraint) {(void)constraint;}


		/// @brief Adds an action to the world.
		/// @brief action The action.
		virtual void	addAction(btActionInterface* action) = 0;

		/// @brief Removes an action from the world.
		/// @brief action The action.
		virtual void	removeAction(btActionInterface* action) = 0;


		/// @brief Sets the acceleration due to gravity of the world.
		/// Upon calling this function, the gravity of each and every object in the world will have its gravity updated to the new value.
		/// When objects are added to the world, they will have their gravity changed to this value too.
		/// @param gravity The acceleration due to gravity which acts on every object in the world.
		virtual void	setGravity(const btVector3& gravity) = 0;

		/// @brief Gets the acceleration due to gravity which is acting on the objects in the world.
		virtual btVector3 getGravity () const = 0;


		virtual void	synchronizeMotionStates() = 0;

		/// @brief Adds a rigid body to the world.
		/// @param body The rigid body.
		virtual void	addRigidBody(btRigidBody* body) = 0;

		/// @brief Adds a rigid body to the world.
		/// @param body The rigid body.
		/// @param group The group.
		/// @param mask The mask.
		virtual void	addRigidBody(btRigidBody* body, short group, short mask) = 0;

		/// @brief Removes a rigid body from the world.
		/// @param body The rigid body/
		virtual void	removeRigidBody(btRigidBody* body) = 0;

		/// @brief Sets the constraint solver.
		/// @param solver The constraint solver.
		virtual void    setConstraintSolver(btConstraintSolver* solver) = 0;

		/// @brief Gets the constraint solver.
		virtual btConstraintSolver* getConstraintSolver() = 0;

		/// @brief Gets the number of constraints in the world.
		virtual	int getNumConstraints() const {	return 0;		}

		/// @brief Gets a constraint in the world.
		/// @param index The index of the constraint.
		virtual btTypedConstraint* getConstraint(int index)		{	(void)index;		return 0;		}

		/// @brief Gets a constraint in the world.
		/// @param index The index of the constraint.
		virtual const btTypedConstraint* getConstraint(int index) const	{	(void)index;	return 0;	}

		/// @brief Gets the type of the world.
		virtual btDynamicsWorldType	getWorldType() const = 0;

		/// @brief Clears all of the forces and torques that have been applied to each object in the world.
		virtual void	clearForces() = 0;

		/// @brief Sets the callback which is called when an internal tick (simulation substep) happens.
		/// @param cb The callback function.
		/// @param worldUserInfo The user pointer for the world object.
		/// @param Whether the callback will be called before (pre) tick occurs or after.
		void setInternalTickCallback(btInternalTickCallback cb,	void* worldUserInfo = 0, bool isPreTick = false)
		{
			if (isPreTick)
			{
				m_internalPreTickCallback = cb;
			} else
			{
				m_internalTickCallback = cb;
			}
			m_worldUserInfo = worldUserInfo;
		}

		/// @brief Sets the user pointer.
		/// @param worldUserInfo The pointer.
		void	setWorldUserInfo(void* worldUserInfo)
		{
			m_worldUserInfo = worldUserInfo;
		}

		/// @brief Gets the user pointer.
		void*	getWorldUserInfo() const
		{
			return m_worldUserInfo;
		}

		/// @brief Gets information about the solver.
		btContactSolverInfo& getSolverInfo()
		{
			return m_solverInfo;
		}


		/// @brief Adds a vehicle to the world.
		/// @param vehicle The vehicle.
		/// @deprecated Use addAction() instead.
		virtual void	addVehicle(btActionInterface* vehicle) {(void)vehicle;}

		/// @brief Removes a vehicle from the world.
		/// @param vehicle The vehicle.
		/// @deprecated Use removeAction() instead.
		virtual void	removeVehicle(btActionInterface* vehicle) {(void)vehicle;}

		/// @brief Adds a character to the world.
		/// @param character The character.
		/// @deprecated Use addAction() instead.
		virtual void	addCharacter(btActionInterface* character) {(void)character;}

		/// @brief Removes a character from the world.
		/// @param character The character.
		/// @deprecated Use removeAction() instead.
		virtual void	removeCharacter(btActionInterface* character) {(void)character;}


};

/// @note Do not change this serialization structure, it requires an updated sBulletDNAstr/sBulletDNAstr64.
struct btDynamicsWorldDoubleData
{
	btContactSolverInfoDoubleData	m_solverInfo;
	btVector3DoubleData	m_gravity;
};

/// @note Do not change this serialization structure, it requires an updated sBulletDNAstr/sBulletDNAstr64.
struct btDynamicsWorldFloatData
{
	btContactSolverInfoFloatData	m_solverInfo;
	btVector3FloatData	m_gravity;
};


#endif //BT_DYNAMICS_WORLD_H


