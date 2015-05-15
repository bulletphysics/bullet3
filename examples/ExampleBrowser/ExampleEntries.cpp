

#include "ExampleEntries.h"

#include "LinearMath/btAlignedObjectArray.h"
#include "EmptyExample.h"
#include "../RenderingExamples/RenderInstancingDemo.h"
#include "../RenderingExamples/CoordinateSystemDemo.h"
#include "../RenderingExamples/RaytracerSetup.h"
#include "../ForkLift/ForkLiftDemo.h"
#include "../MultiThreadedDemo/MultiThreadedDemo.h"
#include "../BasicDemo/BasicExample.h"
#include "../Planar2D/Planar2D.h"
#include "../Benchmarks/BenchmarkDemo.h"
#include "../Importers/ImportObjDemo/ImportObjExample.h"
#include "../Importers/ImportBsp/ImportBspExample.h"
#include "../Importers/ImportColladaDemo/ImportColladaSetup.h"
#include "../Importers/ImportSTLDemo/ImportSTLSetup.h"
#include "../Importers/ImportURDFDemo/ImportURDFSetup.h"
#include "../GyroscopicDemo/GyroscopicSetup.h"
#include "../Constraints/Dof6Spring2Setup.h"
#include "../Constraints/ConstraintPhysicsSetup.h"
#include "../MultiBody/TestJointTorqueSetup.h"
#include "../MultiBody/MultiDofDemo.h"
#include "../VoronoiFracture/VoronoiFractureDemo.h"
#include "../SoftDemo/SoftDemo.h"
#include "../Constraints/ConstraintDemo.h"
#include "../Vehicles/Hinge2Vehicle.h"
#include "../Importers/ImportBullet/SerializeSetup.h"
#include "../Raycast/RaytestDemo.h"
#include "../FractureDemo/FractureDemo.h"
#include "../DynamicControlDemo/MotorDemo.h"
#include "../RollingFrictionDemo/RollingFrictionDemo.h"

#ifdef B3_USE_CLEW
#ifndef NO_OPENGL3
#include "../OpenCL/broadphase/PairBench.h"
#include "../OpenCL/rigidbody/GpuConvexScene.h"
#endif
#endif //B3_USE_CLEW



struct ExampleEntry
{
	int									m_menuLevel;
	const char*							m_name;
	const char*							m_description;
	CommonExampleInterface::CreateFunc*		m_createFunc;
	int									m_option;

	ExampleEntry(int menuLevel, const char* name)
		:m_menuLevel(menuLevel), m_name(name), m_description(0), m_createFunc(0), m_option(0)
	{
	}

	ExampleEntry(int menuLevel, const char* name,const char* description, CommonExampleInterface::CreateFunc* createFunc, int option=0)
		:m_menuLevel(menuLevel), m_name(name), m_description(description), m_createFunc(createFunc), m_option(option)
	{
	}
};


static ExampleEntry gDefaultExamples[]=
{
	
	
	ExampleEntry(0,"API"),
	ExampleEntry(1,"Basic Example","Create some rigid bodies using box collision shapes. This is a good example to familiarize with the basic initialization of Bullet. The Basic Example can also be compiled without graphical user interface, as a console application. Press W for wireframe, A to show AABBs, I to suspend/restart physics simulation. Press D to toggle auto-deactivation of the simulation. ", BasicExampleCreateFunc),

	ExampleEntry(1,"Rolling Friction", "Damping is often not good enough to keep rounded objects from rolling down a sloped surface. Instead, you can set the rolling friction of a rigid body. Generally it is best to leave the rolling friction to zero, to avoid artifacts.", RollingFrictionCreateFunc),
	
	ExampleEntry(1,"Constraints","Show the use of the various constraints in Bullet. Press the L key to visualize the constraint limits. Press the C key to visualize the constraint frames.", 
				 AllConstraintCreateFunc),

	ExampleEntry(1,"Motorized Hinge","Use of a btHingeConstraint. You can adjust the first slider to change the target velocity, and the second slider to adjust the maximum impulse applied to reach the target velocity. Note that the hinge angle can reach beyond -360 and 360 degrees.", ConstraintCreateFunc),

//	ExampleEntry(0,"What's new in 2.83"),
	
	ExampleEntry(1,"6DofSpring2","Show the use of the btGeneric6DofSpring2Constraint. This is a replacement of the btGeneric6DofSpringConstraint, it has various improvements. This includes improved spring implementation and better control over the restitution (bounce) when the constraint hits its limits.", 
				 Dof6Spring2CreateFunc),

	ExampleEntry(1,"Motor Demo", "Dynamic control the target velocity of a motor of a btHingeConstraint. This demo makes use of the 'internal tick callback'. You can press W for wireframe, C and L to visualize constraint frame and limits.", MotorControlCreateFunc),
	
	ExampleEntry(1,"Gyroscopic", "Show the Dzhanibekov effect using various settings of the gyroscopic term. You can select the gyroscopic term computation using btRigidBody::setFlags, with arguments BT_ENABLE_GYROSCOPIC_FORCE_EXPLICIT (using explicit integration, which adds energy and can lead to explosions), BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_WORLD, BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY. If you don't set any of these flags, there is no gyroscopic term used.", GyroscopicCreateFunc),




	ExampleEntry(0,"MultiBody"),
	ExampleEntry(1,"MultiDofCreateFunc","Create a basic btMultiBody with 3-DOF spherical joints (mobilizers). The demo uses a fixed base or a floating base at restart.", MultiDofCreateFunc),
	ExampleEntry(1,"TestJointTorque","Apply a torque to a btMultiBody with 1-DOF joints (mobilizers).", TestJointTorqueCreateFunc),
	
	
#ifdef INCLUDE_CLOTH_DEMOS
	ExampleEntry(0,"Soft Body"),
	ExampleEntry(1,"Cloth","Simulate a patch of cloth.", SoftDemoCreateFunc,0),
	
	ExampleEntry(1,"Pressure","Simulate 3d soft body using a pressure constraint.",SoftDemoCreateFunc,1),
	ExampleEntry(1,"Volume","Simulate 3d soft body using a volume constraint.",SoftDemoCreateFunc,2),
	ExampleEntry(1,"Ropes","Simulate ropes", SoftDemoCreateFunc,3),
	ExampleEntry(1,"Rope Attach","Simulate a rigid body connected to a rope.", SoftDemoCreateFunc,4),
	ExampleEntry(1,"Cloth Attach","A rigid body attached to a cloth.", SoftDemoCreateFunc,5),
	ExampleEntry(1,"Sticks","Show simulation of ropes fixed to the ground.", SoftDemoCreateFunc,6),
	ExampleEntry(1,"Capsule Collision","Collision detection between a capsule shape and cloth.", SoftDemoCreateFunc,7),
	
	ExampleEntry(1,"Collide","Soft body collision", SoftDemoCreateFunc,8),
	ExampleEntry(1,"Collide 2","Soft body collision",SoftDemoCreateFunc,9),
	ExampleEntry(1,"Collide 3","Soft body collision",SoftDemoCreateFunc,10),
	ExampleEntry(1,"Impact","Soft body impact",SoftDemoCreateFunc,11),
	ExampleEntry(1,"Aero","Rudimentary aero dynamics simulation", SoftDemoCreateFunc,12),
	ExampleEntry(1,"Aero 2","Rudimentary aero dynamics simulation",SoftDemoCreateFunc,13),
	ExampleEntry(1,"Friction","Simulate soft body friction with friction coefficients ranging from 0 to 1.", SoftDemoCreateFunc,14),
	ExampleEntry(1,"Torus","Simulate a soft body torus.",SoftDemoCreateFunc,15),
	ExampleEntry(1,"Torus (Shape Match)","Simulate a soft body torus using shape matching.", SoftDemoCreateFunc,16),
	ExampleEntry(1,"Bunny","Simulate the Stanford bunny as deformable object.", SoftDemoCreateFunc,17),
	ExampleEntry(1,"Bunny (Shape Match)","Simulate the Stanford bunny as deformable object including shape matching.", SoftDemoCreateFunc,18),
	ExampleEntry(1,"Cutting","Allow cutting of the soft body, by clicking on the cloth", SoftDemoCreateFunc,19),
	ExampleEntry(1,"Cluster Deform","Soft body collision detection using convex collision clusters.", SoftDemoCreateFunc,20),
	ExampleEntry(1,"Cluster Collide1","Collision detection between soft bodies using convex collision clusters.", SoftDemoCreateFunc,21),
	ExampleEntry(1,"Cluster Collide2","Collision detection between soft bodies using convex collision clusters.",SoftDemoCreateFunc,22),
	ExampleEntry(1,"Cluster Socket","Soft bodies connected by a point to point (ball-socket) constraints. This requires collision clusters, in order to define a frame of reference for the constraint."
					, SoftDemoCreateFunc,23),
	ExampleEntry(1,"Cluster Hinge","Soft bodies connected by a hinge constraints. This requires collision clusters, in order to define a frame of reference for the constraint.", SoftDemoCreateFunc,24),
	ExampleEntry(1,"Cluster Combine","Simulate soft bodies using collision clusters.", SoftDemoCreateFunc,25),
	ExampleEntry(1,"Cluster Car","Simulate the Stanford bunny by multiple soft bodies connected by constraints.", SoftDemoCreateFunc,26),
	ExampleEntry(1,"Cluster Robot","A rigid body base connected by soft body wheels, connected by constraints.", SoftDemoCreateFunc,27),
	ExampleEntry(1,"Cluster Stack Soft","Stacking of soft bodies.", SoftDemoCreateFunc,28),
	ExampleEntry(1,"Cluster Stack Mixed","Stacking of soft bodies and rigid bodies.",SoftDemoCreateFunc,29),
	ExampleEntry(1,"Tetra Cube","Simulate a volumetric soft body cube defined by tetrahedra.", SoftDemoCreateFunc,30),
	ExampleEntry(1,"Tetra Bunny","Simulate a volumetric soft body Stanford bunny defined by tetrahedra.", SoftDemoCreateFunc,31),
	
#endif //INCLUDE_CLOTH_DEMOS

	///we disable the benchmarks in debug mode, they are way too slow and benchmarking in debug mode is not recommended
#ifndef _DEBUG
	ExampleEntry(0,"Benchmarks"),
	ExampleEntry(1,"3000 boxes", "Benchmark a stack of 3000 boxes. It will stress the collision detection, a specialized box-box implementation based on the separating axis test, and the constraint solver. ", BenchmarkCreateFunc, 1),
	ExampleEntry(1,"1000 stack", "Benchmark a stack of 3000 boxes. It will stress the collision detection, a specialized box-box implementation based on the separating axis test, and the constraint solver. ",
	BenchmarkCreateFunc, 2),
	ExampleEntry(1,"Ragdolls", "Benchmark the performance of the ragdoll constraints, btHingeConstraint and btConeTwistConstraint, in addition to capsule collision detection.", BenchmarkCreateFunc, 3),
	ExampleEntry(1,"Convex stack", "Benchmark the performance and stability of rigid bodies using btConvexHullShape.", BenchmarkCreateFunc, 4),
	ExampleEntry(1,"Prim vs Mesh", "Benchmark the performance and stability of rigid bodies using primitive collision shapes (btSphereShape, btBoxShape), resting on a triangle mesh, btBvhTriangleMeshShape.", BenchmarkCreateFunc, 5),
	ExampleEntry(1,"Convex vs Mesh", "Benchmark the performance and stability of rigid bodies using convex hull collision shapes (btConvexHullShape), resting on a triangle mesh, btBvhTriangleMeshShape.", BenchmarkCreateFunc, 6),
	ExampleEntry(1,"Raycast", "Benchmark the performance of the btCollisionWorld::rayTest. Note that currently the rays are not rendered.", BenchmarkCreateFunc, 7),
#endif


	

	ExampleEntry(0,"Importers"),
	ExampleEntry(1,"Import .bullet", "Load a binary .bullet file. The serialization mechanism can deal with versioning, differences in endianess, 32 and 64bit, double/single precision. It is easy to save a .bullet file, see the examples/Importers/ImportBullet/SerializeDemo.cpp for a code example how to export a .bullet file.", SerializeBulletCreateFunc),
	
	ExampleEntry(1,"Wavefront Obj", "Import a Wavefront .obj file", ImportObjCreateFunc, 0),

	ExampleEntry(1,"Quake BSP", "Import a Quake .bsp file", ImportBspCreateFunc, 0),
	ExampleEntry(1,"COLLADA dae", "Import the geometric mesh data from a COLLADA file. This is used as part of the URDF importer. This loader can also be used to import collision geometry in general. ", 
					ImportColladaCreateFunc, 0),
	ExampleEntry(1,"STL", "Import the geometric mesh data from a STL file. This is used as part of the URDF importer. This loader can also be used to import collision geometry in general. ",ImportSTLCreateFunc, 0),
	ExampleEntry(1,"URDF (RigidBody)", "Import a URDF file, and create rigid bodies (btRigidBody) connected by constraints.", ImportURDFCreateFunc, 0),
	ExampleEntry(1,"URDF (MultiBody)", "Import a URDF file and create a single multibody (btMultiBody) with tree hierarchy of links (mobilizers).", 
					ImportURDFCreateFunc, 1),

	ExampleEntry(0,"Vehicles"),
	ExampleEntry(1,"Hinge2 Vehicle", "A rigid body chassis with 4 rigid body wheels attached by a btHinge2Constraint",Hinge2VehicleCreateFunc),
	ExampleEntry(1,"ForkLift","Simulate a fork lift vehicle with a working fork lift that can be moved using the cursor keys. The wheels collision is simplified using ray tests."
					"There are currently some issues with the wheel rendering, the wheels rotate when picking up the object."
					"The demo implementation allows to choose various MLCP constraint solvers.", 
					ForkLiftCreateFunc),

	ExampleEntry(0,"Raycast"),
	ExampleEntry(1,"Raytest", "Cast rays using the btCollisionWorld::rayTest method. The example shows how to receive the hit position and normal along the ray against the first object. Also it shows how to receive all the hits along a ray.", RaytestCreateFunc),
	ExampleEntry(1,"Raytracer","Implement an extremely simple ray tracer using the ray trace functionality in btCollisionWorld.", 
					RayTracerCreateFunc),
	

	ExampleEntry(0,"Experiments"),

	ExampleEntry(1,"Voronoi Fracture", "Automatically create a compound rigid body using voronoi tesselation. Individual parts are modeled as rigid bodies using a btConvexHullShape.",
				 VoronoiFractureCreateFunc),

	ExampleEntry(1,"Fracture demo", "Create a basic custom implementation to model fracturing objects, based on a btCompoundShape. It explicitly propagates the collision impulses and breaks the rigid body into multiple rigid bodies. Press F to toggle fracture and glue mode.", FractureDemoCreateFunc),
				 
	ExampleEntry(1,"Planar 2D","Show the use of 2D collision shapes and rigid body simulation. The collision shape is wrapped into a btConvex2dShape. The rigid bodies are restricted in a plane using the 'setAngularFactor' and 'setLinearFactor' API call.",Planar2DCreateFunc),
#if BT_USE_OPENMP || BT_USE_TBB || BT_USE_PPL
    // only enable MultiThreaded demo if a task scheduler is available
    ExampleEntry( 1, "Multithreaded Demo",
    "Multithreaded demo using OpenMP, Intel Threading Building Blocks (TBB), or Microsoft Parallel Patterns Library (PPL). "
    "Press o to use OpenMP. "
    "Press t to use TBB. "
    "Press p to use PPL. "
    "Press n for single-threaded. "
    "Press + or - to change thread count. "
    ,
    MultiThreadedDemoCreateFunc ),
#endif


	ExampleEntry(0,"Rendering"),
	ExampleEntry(1,"Instanced Rendering", "Simple example of fast instanced rendering, only active when using OpenGL3+.",RenderInstancingCreateFunc),
	ExampleEntry(1,"CoordinateSystemDemo","Show the axis and positive rotation direction around the axis.", CoordinateSystemCreateFunc),
	
};

#ifdef B3_USE_CLEW
#ifndef NO_OPENGL3
static ExampleEntry gOpenCLExamples[]=
{
	ExampleEntry(0,"OpenCL (experimental)"),
	ExampleEntry(1,"Box-Box", "Full OpenCL implementation of the entire physics and collision detection pipeline, showing box-box rigid body",
	OpenCLBoxBoxCreateFunc),
	ExampleEntry(1,"Pair Bench", "Benchmark of overlapping pair search using OpenCL.", PairBenchOpenCLCreateFunc),

};
#endif
#endif //
static btAlignedObjectArray<ExampleEntry> gAdditionalRegisteredExamples;


struct ExampleEntriesInternalData
{
	btAlignedObjectArray<ExampleEntry> m_allExamples;
};

ExampleEntries::ExampleEntries()
{
	m_data = new ExampleEntriesInternalData;
}

ExampleEntries::~ExampleEntries()
{
	delete m_data;
}

void ExampleEntries::initOpenCLExampleEntries()
{
#ifdef B3_USE_CLEW
#ifndef NO_OPENGL3
	int numDefaultEntries = sizeof(gOpenCLExamples)/sizeof(ExampleEntry);
	for (int i=0;i<numDefaultEntries;i++)
	{
		m_data->m_allExamples.push_back(gOpenCLExamples[i]);
	}
#endif
#endif //B3_USE_CLEW
}

void ExampleEntries::initExampleEntries()
{
	m_data->m_allExamples.clear();

	
	

	int numDefaultEntries = sizeof(gDefaultExamples)/sizeof(ExampleEntry);
	for (int i=0;i<numDefaultEntries;i++)
	{
		m_data->m_allExamples.push_back(gDefaultExamples[i]);
	}

	if (m_data->m_allExamples.size()==0)
	{

		{
			ExampleEntry e(0,"Empty");
			m_data->m_allExamples.push_back(e);
		}

		{
			ExampleEntry e(1,"Empty","Empty Description", EmptyExample::CreateFunc);
			m_data->m_allExamples.push_back(e);
		}
	}

}

void ExampleEntries::registerExampleEntry(int menuLevel, const char* name,const char* description, CommonExampleInterface::CreateFunc* createFunc, int option)
{
	ExampleEntry e( menuLevel,name,description, createFunc, option);
	gAdditionalRegisteredExamples.push_back(e);
}

int ExampleEntries::getNumRegisteredExamples()
{
	return m_data->m_allExamples.size();
}

CommonExampleInterface::CreateFunc* ExampleEntries::getExampleCreateFunc(int index)
{
	return m_data->m_allExamples[index].m_createFunc;
}

int ExampleEntries::getExampleOption(int index)
{
	return m_data->m_allExamples[index].m_option;
}

const char* ExampleEntries::getExampleName(int index)
{
	return m_data->m_allExamples[index].m_name;
}

const char* ExampleEntries::getExampleDescription(int index)
{
	return m_data->m_allExamples[index].m_description;
}
