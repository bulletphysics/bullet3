
#ifndef BULLET_DEMO_ENTRIES_H
#define BULLET_DEMO_ENTRIES_H

#include "Bullet3AppSupport/BulletDemoInterface.h"
#include "../bullet2/BasicDemo/BasicDemo.h"

#include "../bullet2/CoordinateFrameDemo/CoordinateFrameDemoPhysicsSetup.h"

#include "../bullet2/BasicDemo/HingeDemo.h"
#include "../bullet2/BasicDemo/HingeDemo.h"

#include "../bullet2/FeatherstoneMultiBodyDemo/BulletMultiBodyDemos.h"
#include "../bullet2/FeatherstoneMultiBodyDemo/MultiDofDemo.h"

#include "../bullet2/RagdollDemo/RagdollDemo.h"
#include "../bullet2/LuaDemo/LuaPhysicsSetup.h"
#include "../bullet2/ChainDemo/ChainDemo.h"
#include "../../Demos/CcdPhysicsDemo/CcdPhysicsSetup.h"
#include "../bullet2/ConstraintDemo/ConstraintPhysicsSetup.h"
#include "../ImportURDFDemo/ImportURDFSetup.h"
#include "../ImportObjDemo/ImportObjSetup.h"
#include "../ImportSTLDemo/ImportSTLSetup.h"
#include "../ImportColladaDemo/ImportColladaSetup.h"

#include "../../Demos/SerializeDemo/SerializeSetup.h"
#include "../bullet2/MultiBodyDemo/TestJointTorqueSetup.h"
#include "../bullet2/MultiBodyDemo/MultiBodyVehicle.h"

#include "../bullet2/CollisionDetection/SupportFuncDemo.h"
#include "../bullet2/BasicConcepts/CoordinateSystemDemo.h"
#include "../../Demos3/FiniteElementMethod/FiniteElementDemo.h"
//#include "../../Demos3/bullet2/SoftDemo/SoftDemo.h"
#include "../Geometry/SphereCreation.h"
#include "../Geometry/DistributePoints.h"

#define MYCREATEFUNC(func) \
static BulletDemoInterface* func##CreateFunc(CommonGraphicsApp* app)\
{\
	CommonPhysicsSetup* physicsSetup = new func##Setup();\
       return new BasicDemo(app, physicsSetup);\
}

#define MYCREATEFUNC2(func,setup) \
static BulletDemoInterface* func(CommonGraphicsApp* app)\
{\
	CommonPhysicsSetup* physicsSetup = new setup(app);\
       return new BasicDemo(app, physicsSetup);\
}

MYCREATEFUNC(TestJointTorque);
MYCREATEFUNC(MultiBodyVehicle);
MYCREATEFUNC2(LuaDemoCreateFunc,LuaPhysicsSetup);
MYCREATEFUNC(Serialize);
MYCREATEFUNC(CcdPhysics);
MYCREATEFUNC(KinematicObject);
MYCREATEFUNC(ConstraintPhysics);
MYCREATEFUNC(ImportUrdf);
MYCREATEFUNC2(ImportObjCreateFunc,ImportObjSetup);
MYCREATEFUNC2(ImportSTLCreateFunc,ImportSTLSetup);
MYCREATEFUNC(CoordinateFrameDemoPhysics);


static BulletDemoInterface* MyImportColladaCreateFunc(CommonGraphicsApp* app)
{
    CommonPhysicsSetup* physicsSetup = new ImportColladaSetup(app);
	return new BasicDemo(app, physicsSetup);
}




struct BulletDemoEntry
{
	int									m_menuLevel;
	const char*							m_name;
	BulletDemoInterface::CreateFunc*	m_createFunc;
};


static BulletDemoEntry allDemos[]=
{

    {0,"Basic Concepts",0},
    {1,"Basis Frame", &CoordinateSystemDemo::CreateFunc},
	{1,"SupportFunc", &MySupportFuncDemo::CreateFunc},
	
	{0,"API Demos", 0},

	{1,"BasicDemo",BasicDemo::MyCreateFunc},
	{ 1, "CcdDemo", CcdPhysicsCreateFunc },
	{ 1, "Kinematic", KinematicObjectCreateFunc },
	{ 1, "Constraints", ConstraintPhysicsCreateFunc },
	{ 1, "LuaDemo",LuaDemoCreateFunc},

	{0,"File Formats", 0},

    { 1, ".bullet",SerializeCreateFunc},
	{ 1, "Wavefront Obj", ImportObjCreateFunc},
    { 1, "URDF", ImportUrdfCreateFunc },
	{ 1, "STL", ImportSTLCreateFunc},
	{ 1, "COLLADA", MyImportColladaCreateFunc},
	{0,"Experiments", 0},
	{1, "Finite Element Demo", FiniteElementDemo::CreateFunc},
	{1,"SphereCreation", &SphereCreation::CreateFunc},
	{1,"DistributePoints", &DistributePoints::CreateFunc},
	{1,"Coordinate Frames", CoordinateFrameDemoPhysicsCreateFunc},
//    {0,"Soft Body", 0},
    
//	{1,"Cloth1", SoftDemo::CreateFunc},
/*	{1,"ChainDemo",ChainDemo::MyCreateFunc},
//	{0, "Stress tests", 0 },

	{1,"SIHingeDemo",HingeDemo::SICreateFunc},
	{1,"PGSHingeDemo",HingeDemo::PGSCreateFunc},
	{1,"DantzigHingeDemo",HingeDemo::DantzigCreateFunc},
	{1,"LemkeHingeDemo",HingeDemo::LemkeCreateFunc},
	{1,"InertiaHingeDemo",HingeDemo::InertiaCreateFunc},
	{1,"ABMHingeDemo",HingeDemo::FeatherstoneCreateFunc},

	{1,"Ragdoll",RagDollDemo::MyCreateFunc},
	*/
	{ 0, "Multibody" ,0},
	{1,"MultiBody1",FeatherstoneDemo1::MyCreateFunc},
//	{"MultiBody2",FeatherstoneDemo2::MyCreateFunc},
	{1,"MultiDofDemo",MultiDofDemo::MyCreateFunc},
	{1,"TestJointTorque",TestJointTorqueCreateFunc},
    {1,"MultiBodyVehicle", MultiBodyVehicleCreateFunc},


};

#include <stdio.h>


static void saveCurrentDemoEntry(int currentEntry,const char* startFileName)
{
	FILE* f = fopen(startFileName,"w");
	if (f)
	{
		fprintf(f,"%d\n",currentEntry);
		fclose(f);
	}
};

static int loadCurrentDemoEntry(const char* startFileName)
{
	int currentEntry= 0;
	FILE* f = fopen(startFileName,"r");
	if (f)
	{
		int result;
		result = fscanf(f,"%d",&currentEntry);
		if (result)
		{
			return currentEntry;
		}
		fclose(f);
	}
	return 0;
};

#endif//BULLET_DEMO_ENTRIES_H

