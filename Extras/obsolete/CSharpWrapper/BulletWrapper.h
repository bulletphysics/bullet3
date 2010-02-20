/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2008 SCEI

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/*
	This is a C++/CLI wrapper of the high-level generic physics C-API.
	You will be able to use it in C#
*/

#pragma once

#include "Bullet-C-Api.h"

namespace Scea
{
	namespace BulletPhysics
	{
		/// <summary>
		/// Base Bullet Class</summary>
		public __gc class Bullet;
		public __gc class RigidBodyHandle;
		public __gc class CollisionShapeHandle;
		public __gc class MeshInterfaceHandle;

		/// <summary>
		/// A Mesh Interface Handle for Bullet</summary>
		public __gc class MeshInterfaceHandle
		{
		public:
			/// <summary>
			/// Constructor for Mesh Interface Handle</summary>
			MeshInterfaceHandle(plMeshInterfaceHandle handle)
			{
				_plMeshInterfaceHandle = handle;
			}
			/// <summary>
			/// Destructor for Mesh Interface Handle</summary>
			~MeshInterfaceHandle()
			{
				_plMeshInterfaceHandle = 0;
			};
			plMeshInterfaceHandle _plMeshInterfaceHandle;
		};

		/// <summary>
		/// A RigidBody handle for Bullet</summary>
		public __gc class RigidBodyHandle
		{
		public:
			/// <summary>
			/// Constructor for RigidBody handle</summary>
			RigidBodyHandle(plRigidBodyHandle handle)
			{
				_plRigidBodyHandle = handle;
			}
			/// <summary>
			/// Destructor for RigidBody handle</summary>
			~RigidBodyHandle()
			{
				_plRigidBodyHandle = 0;
			};
			plRigidBodyHandle _plRigidBodyHandle;
		};

		/// <summary>
		/// A Collision Shape handle for Bullet</summary>
		public __gc class CollisionShapeHandle
		{
		public:
			/// <summary>
			/// Constructor for Collision Shape handle</summary>
			CollisionShapeHandle(plCollisionShapeHandle handle)
			{
				_plCollisionShapeHandle = handle;
			}

			/// <summary>
			/// Destructor for Collision Shape handle</summary>
			~CollisionShapeHandle()
			{
				_plCollisionShapeHandle = 0;
			};
			plCollisionShapeHandle _plCollisionShapeHandle;
		};

		/// <summary>
		/// A RigidBody class in Bullet</summary>
		public __gc class RigidBody
		{
		public: 
			/// <summary>
			/// Constructor for RigidBody</summary>
            /// <param name="mass">The mass of this RigidBody</param>
            /// <param name="shape">The Collision Shape handle of this RigidBody</param>
            /// <remarks>stationary object has mass=0.f</remarks>
			RigidBody(float mass, CollisionShapeHandle * shape)
			{
				_plCollisionShapeHandle = shape->_plCollisionShapeHandle;
				_plRigidBodyHandle = plCreateRigidBody(0, mass, _plCollisionShapeHandle);
			}

			/// <summary>
			/// Destructor for RigidBody</summary>
			~RigidBody()
			{
				if (_plRigidBodyHandle != 0)
				{
					plDeleteRigidBody(_plRigidBodyHandle );
					_plRigidBodyHandle = 0;
				}
			}

			/// <summary>
			/// Dispose function for RigidBody</summary>
			void Dispose()
			{
				if (_plRigidBodyHandle != 0)
				{
					plDeleteRigidBody(_plRigidBodyHandle );
					_plRigidBodyHandle = 0;
				}
			}
			
			/// <summary>
			/// Get the Position for this RigidBody</summary>
			/// <param name="float_array">The float[3] position of this RigidBody</param>
			void GetPosition(float float_array __gc[])
			{
				float position[3];
				plGetPosition(_plRigidBodyHandle, position);				
				for (int i=0; i<3; i++)
					float_array[i] = position[i];
			}

			/// <summary>
			/// Get the Open GL World Matrix for this RigidBody</summary>
			/// <param name="float_array">The float[16] GL world matrix of this RigidBody</param>
			void GetOpenGLMatrix(float float_array __gc[])
			{
				float matrix[16];
				plGetOpenGLMatrix(_plRigidBodyHandle, matrix);
				for (int i=0; i<16; i++)
					float_array[i] = matrix[i];
			}

			/// <summary>
			/// Get the orientation for this RigidBody</summary>
			/// <param name="float_array">The float[4] orientation of this RigidBody</param>
			void GetOrientation(float float_array __gc[])
			{
				float orientation[4];
				plGetOrientation(_plRigidBodyHandle, orientation);
				for (int i=0; i<4; i++)
					float_array[i] = orientation[i];
			}

			/// <summary>
			/// Set the position for this RigidBody</summary>
			/// <param name="float_array">The float[3] position of this RigidBody</param>
			void SetPosition(float float_array __gc[])
			{
				float position[3];
				for (int i=0; i<3; i++)
					position[i] = float_array[i];
				plSetPosition(_plRigidBodyHandle, position);
			}

			/// <summary>
			/// Set the orientation for this RigidBody</summary>
			/// <param name="float_array">The float[3] orientation of this RigidBody</param>
			void SetOrientation(float float_array __gc[])
			{
				float orientation[3];
				for (int i=0; i<3; i++)
					orientation[i] = float_array[i];
				plSetOrientation(_plRigidBodyHandle, orientation);
			}

			/// <summary>
			/// Set the orientation for this RigidBody</summary>
			/// <param name="yaw">The yaw orientation of this RigidBody</param>
			/// <param name="pitch">The pitch orientation of this RigidBody</param>
			/// <param name="roll">The roll orientation of this RigidBody</param>
			void SetOrientation(float yaw, float pitch, float roll)
			{
				float orient[4];
				plSetEuler(yaw, pitch, roll, orient);
				plSetOrientation(_plRigidBodyHandle, orient);
			}

			/// <summary>
			/// Set the Open GL World Matrix for this RigidBody</summary>
			/// <param name="float_array">The float[16] GL world matrix of this RigidBody</param>
			void SetOpenGLMatrix(float float_array __gc[])
			{
				float matrix[16];
				for (int i=0; i<16; i++)
					matrix[i] = float_array[i];
				plSetOpenGLMatrix(_plRigidBodyHandle, matrix);
			}

			plRigidBodyHandle _plRigidBodyHandle;
			plCollisionShapeHandle _plCollisionShapeHandle;
		};


		/// <summary>
		/// Base Bullet Class</summary>
		public __gc class Bullet
		{
		public:
			/// <summary>
			/// Constructor for the Bullet</summary>
			Bullet()
			{
				m_rigidbody_count = 0;
				_plDynamicsWorldHandle = 0;
				_plPhysicsSdkHandle = plNewBulletSdk();
			}
			/// <summary>
			/// Destructor for the Bullet</summary>
			~Bullet()
			{
				plDeletePhysicsSdk(_plPhysicsSdkHandle);
				_plDynamicsWorldHandle = 0;
				_plPhysicsSdkHandle = 0;
			}

			/// <summary>
			/// Step Simulation for Bullet
			/// </summary>
			/// <param name="timeStep">The amount of time to step in this simulation</param>
			void StepSimulation(plReal	timeStep)
			{
				plStepSimulation(_plDynamicsWorldHandle, timeStep);
			}

			/// <summary>
			/// Create collision shape handle for sphere
			/// </summary>
			/// <param name="radius">The radius of the sphere</param>
			CollisionShapeHandle * CreateSphere(float radius)
			{
				return new CollisionShapeHandle(plNewSphereShape(radius));
			}

			/// <summary>
			/// Create collision shape handle for Box
			/// </summary>
			/// <param name="x">extend x of the box</param>
			/// <param name="y">extend y of the box</param>
			/// <param name="z">extend z of the box</param>
			CollisionShapeHandle * CreateBox(float x, float y, float z)
			{
				return new CollisionShapeHandle( plNewBoxShape(x, y, z));
			}

			/// <summary>
			/// Create collision shape handle for Capsule
			/// </summary>
			/// <param name="radius">The radius of the capsule</param>
			/// <param name="height">The height of the capsule</param>
			CollisionShapeHandle * CreateCapsule(float radius, float height)
			{
				return new CollisionShapeHandle( plNewCapsuleShape(radius, height));	
			}

			/// <summary>
			/// Create collision shape handle for cone
			/// </summary>
			/// <param name="radius">The radius of the cone</param>
			/// <param name="height">The height of the cone</param>
			CollisionShapeHandle * CreateCone(float radius, float height)
			{
				return new CollisionShapeHandle( plNewConeShape(radius, height));
			}

			/// <summary>
			/// Create collision shape handle for cylinder
			/// </summary>
			/// <param name="radius">The radius of the cylinder</param>
			/// <param name="height">The height of the cylinder</param>
			CollisionShapeHandle * CreateCylinder(float radius, float height)
			{
				return new CollisionShapeHandle( plNewCylinderShape(radius, height));
			}

			/// <summary>
			/// Create a Compound Shape collision shape handle
			/// </summary>
			CollisionShapeHandle * CreateCompoundShape()
			{
				return new CollisionShapeHandle( plNewCompoundShape());
			}

			/// <summary>
			/// Add shape to this Compound Shape collision shape handle
			/// </summary>
			/// <param name="compoundShape">compoundShape</param>
			/// <param name="childShape">childShape</param>
			/// <param name="childPos">childPos</param>
			/// <param name="childOrn">childOrn</param>
			void AddCompoundShape(CollisionShapeHandle * compoundShape, CollisionShapeHandle * childShape, plVector3 childPos,plQuaternion childOrn)
			{
				plAddChildShape(compoundShape->_plCollisionShapeHandle, childShape->_plCollisionShapeHandle, childPos,childOrn);
			}

			/// <summary>
			/// Destroy a collision shape handle
			/// </summary>
			void DestroyShape(CollisionShapeHandle * shape)
			{
				plDeleteShape(shape->_plCollisionShapeHandle);
				delete shape;
			}

			/// <summary>
			/// Create collision shape handle for Convex Meshes
			/// </summary>
			CollisionShapeHandle * CreateConvexHull()
			{
				return new CollisionShapeHandle(plNewConvexHullShape());
			}

			/// <summary>
			/// Add Convex Hull Vertex to the Convex Mesh shape handle
			/// </summary>
			void AddConvexHullVertex(CollisionShapeHandle * handle, float x, float y, float z)
			{
				plAddVertex(handle->_plCollisionShapeHandle, x, y, z);
			}

/*			CollisionShapeHandle * CreateTriangleMesh()
			{
				MeshInterfaceHandle * handle = new MeshInterfaceHandle(plNewMeshInterface());

				return new CollisionShapeHandle(plNewStaticTriangleMeshShape(handle->_plMeshInterfaceHandle));
			}

			MeshInterfaceHandle * CreateMeshInterface()
			{

				return new MeshInterfaceHandle(plNewMeshInterface());
			}
/*
			void AddMeshTriangle(MeshInterfaceHandle * handle, float * v0, float * v1,float * v2)
			{
				plAddTriangle(handle->_plMeshInterfaceHandle, v0, v1, v2);
			}
/*
			CollisionShapeHandle * CreateTriangleMesh(MeshInterfaceHandle * handle)
			{
				return new CollisionShapeHandle(plNewStaticTriangleMeshShape(handle->_plMeshInterfaceHandle));
			}
*/		/* Concave static triangle meshes */
/*			extern  void		plAddTriangle(plMeshInterfaceHandle meshHandle, plVector3 v0,plVector3 v1,plVector3 v2);
			extern  plCollisionShapeHandle plNewStaticTriangleMeshShape(plMeshInterfaceHandle);
			extern  void plSetScaling(plCollisionShapeHandle shape, plVector3 scaling);
*/

			void SetEuler(float yaw, float pitch, float roll, float * orient)
			{
				plSetEuler(yaw, pitch, roll, orient);
			}
		
			/// <summary>
			/// Add RigidBody, If won't haven't created a dynamic world, create one
			/// </summary>
			void AddRigidBody(RigidBody * body)
			{
				if (m_rigidbody_count == 0)
					CreateDynamicsWorld();
				plAddRigidBody(_plDynamicsWorldHandle, body->_plRigidBodyHandle);
				m_rigidbody_count++;
			}

			/// <summary>
			/// Remove RigidBody, If we remove everything in this dynamic world, we destroy the dynamic world
			/// </summary>
			void RemoveRigidBody(RigidBody * body)
			{
				plRemoveRigidBody(_plDynamicsWorldHandle, body->_plRigidBodyHandle);
				m_rigidbody_count--;
				if (m_rigidbody_count == 0)
					DestroyDynamicsWorld();
			}
		private:
			/// <summary>
			/// Create a Dynamic World
			/// </summary>
			void CreateDynamicsWorld ()
			{
				_plDynamicsWorldHandle = plCreateDynamicsWorld(_plPhysicsSdkHandle);
			}
			/// <summary>
			/// Destroy a Dynamic World
			/// </summary>
			void DestroyDynamicsWorld() 
			{
				plDeleteDynamicsWorld(_plDynamicsWorldHandle);
			}

			// The count of rigidbody in this world.
			int m_rigidbody_count;
			plPhysicsSdkHandle _plPhysicsSdkHandle;

			// This wrapper will let you support only one world at a time.
			plDynamicsWorldHandle _plDynamicsWorldHandle;
		};
	}
}