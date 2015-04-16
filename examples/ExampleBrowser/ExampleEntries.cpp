

#include "ExampleEntries.h"

#include "LinearMath/btAlignedObjectArray.h"
#include "EmptyExample.h"
#include "../RenderingExamples/RenderInstancingDemo.h"
#include "../RenderingExamples/CoordinateSystemDemo.h"
#include "../RenderingExamples/RaytracerSetup.h"
#include "../ForkLift/ForkLiftDemo.h"
#include "../BasicDemo/BasicExample.h"
#include "../Planar2D/Planar2D.h"
#include "../Benchmarks/BenchmarkDemo.h"
#include "../Importers/ImportObjDemo/ImportObjExample.h"
#include "../Importers/ImportBsp/ImportBspExample.h"
#include "../Importers/ImportColladaDemo/ImportColladaSetup.h"
#include "../Importers/ImportSTLDemo/ImportSTLSetup.h"
#include "../Importers/ImportURDFDemo/ImportURDFSetup.h"
#include "../GyroscopicDemo/GyroscopicSetup.h"

struct ExampleEntry
{
	int									m_menuLevel;
	const char*							m_name;
	ExampleInterface::CreateFunc*		m_createFunc;
	int									m_option;

	ExampleEntry(int menuLevel, const char* name,ExampleInterface::CreateFunc* createFunc, int option=0)
		:m_menuLevel(menuLevel), m_name(name), m_createFunc(createFunc), m_option(option)
	{
	}
};


static ExampleEntry gDefaultExamples[]=
{
	ExampleEntry(0,"Rendering",0),
	ExampleEntry(1,"Instanced Rendering", RenderInstancingCreateFunc),
	ExampleEntry(1,"CoordinateSystemDemo",CoordinateSystemCreateFunc),
	ExampleEntry(1,"Raytracer",RayTracerCreateFunc),
	
	ExampleEntry(0,"API",0),
	ExampleEntry(1,"Basic Example",BasicExampleCreateFunc),
	ExampleEntry(1,"Gyroscopic", GyroscopicCreateFunc),
	ExampleEntry(1,"Planar 2D",Planar2DCreateFunc),
	
	ExampleEntry(0,"Benchmarks", 0),
	ExampleEntry(1,"3000 boxes", BenchmarkCreateFunc, 1),
	ExampleEntry(1,"1000 stack", BenchmarkCreateFunc, 2),
	ExampleEntry(1,"Ragdolls", BenchmarkCreateFunc, 3),
	ExampleEntry(1,"Convex stack", BenchmarkCreateFunc, 4),
	ExampleEntry(1,"Prim vs Mesh", BenchmarkCreateFunc, 5),
	ExampleEntry(1,"Convex vs Mesh", BenchmarkCreateFunc, 6),
	ExampleEntry(1,"7", BenchmarkCreateFunc, 7),

	ExampleEntry(0,"Importers", 0),
	ExampleEntry(1,"Wavefront Obj", ImportObjCreateFunc, 0),
	ExampleEntry(1,"Quake BSP", ImportBspCreateFunc, 0),
	ExampleEntry(1,"COLLADA dae", ImportColladaCreateFunc, 0),
	ExampleEntry(1,"STL", ImportSTLCreateFunc, 0),
	ExampleEntry(1,"URDF (RigidBody)", ImportURDFCreateFunc, 0),
	ExampleEntry(1,"URDF (MultiBody)", ImportURDFCreateFunc, 1),
	
	

	ExampleEntry(0,"Vehicles",0),
	ExampleEntry(1,"ForkLift",ForkLiftCreateFunc),
	
	
};


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

void ExampleEntries::initExampleEntries()
{
	m_data->m_allExamples.clear();

	{
		ExampleEntry e(0,"Basic Concepts", 0);
		m_data->m_allExamples.push_back(e);
	}
	{
		ExampleEntry e(1,"Empty",EmptyExample::CreateFunc);
		m_data->m_allExamples.push_back(e);
	}

	int numDefaultEntries = sizeof(gDefaultExamples)/sizeof(ExampleEntry);
	for (int i=0;i<numDefaultEntries;i++)
	{
		m_data->m_allExamples.push_back(gDefaultExamples[i]);
	}

}

void ExampleEntries::registerExampleEntry(int menuLevel, const char* name,ExampleInterface::CreateFunc* createFunc, int option)
{
	ExampleEntry e( menuLevel,name,createFunc, option);
	gAdditionalRegisteredExamples.push_back(e);
}

int ExampleEntries::getNumRegisteredExamples()
{
	return m_data->m_allExamples.size();
}

ExampleInterface::CreateFunc* ExampleEntries::getExampleCreateFunc(int index)
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
