
#ifndef BULLET_DEMO_ENTRIES_H
#define BULLET_DEMO_ENTRIES_H

#include "BulletDemoInterface.h"
#include "../bullet2/BasicDemo/BasicDemo.h"
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
#include "../../Demos/SerializeDemo/SerializeSetup.h"
#include "../bullet2/MultiBodyDemo/TestJointTorqueSetup.h"

static BulletDemoInterface* TestJointTorqueCreateFunc(SimpleOpenGL3App* app)
{
       CommonPhysicsSetup* physicsSetup = new TestJointTorqueSetup();
       return new BasicDemo(app, physicsSetup);
}

static BulletDemoInterface* LuaDemoCreateFunc(SimpleOpenGL3App* app)
{
       CommonPhysicsSetup* physicsSetup = new LuaPhysicsSetup(app);
       return new BasicDemo(app, physicsSetup);
}

static BulletDemoInterface* MyCcdPhysicsDemoCreateFunc(SimpleOpenGL3App* app)
{
	CommonPhysicsSetup* physicsSetup = new CcdPhysicsSetup();
	return new BasicDemo(app, physicsSetup);
}

static BulletDemoInterface* MyKinematicObjectCreateFunc(SimpleOpenGL3App* app)
{
	CommonPhysicsSetup* physicsSetup = new KinematicObjectSetup();
	return new BasicDemo(app, physicsSetup);
}
static BulletDemoInterface* MySerializeCreateFunc(SimpleOpenGL3App* app)
{
    CommonPhysicsSetup* physicsSetup = new SerializeSetup();
	return new BasicDemo(app, physicsSetup);
}
static BulletDemoInterface* MyConstraintCreateFunc(SimpleOpenGL3App* app)
{
	CommonPhysicsSetup* physicsSetup = new ConstraintPhysicsSetup();
	return new BasicDemo(app, physicsSetup);
}

static BulletDemoInterface* MyImportUrdfCreateFunc(SimpleOpenGL3App* app)
{
    CommonPhysicsSetup* physicsSetup = new ImportUrdfDemo();
	return new BasicDemo(app, physicsSetup);
}
static BulletDemoInterface* MyImportObjCreateFunc(SimpleOpenGL3App* app)
{
    CommonPhysicsSetup* physicsSetup = new ImportObjDemo(app);
	return new BasicDemo(app, physicsSetup);
}
static BulletDemoInterface* MyImportSTLCreateFunc(SimpleOpenGL3App* app)
{
    CommonPhysicsSetup* physicsSetup = new ImportSTLDemo(app);
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

	//{"emptydemo",EmptyBulletDemo::MyCreateFunc},
	{0,"API Demos", 0},

	{1,"BasicDemo",BasicDemo::MyCreateFunc},
	{ 1, "CcdDemo", MyCcdPhysicsDemoCreateFunc },
	{ 1, "Kinematic", MyKinematicObjectCreateFunc },
	{ 1, "Constraints", MyConstraintCreateFunc },
	{ 1, "LuaDemo",LuaDemoCreateFunc},

	{0,"File Formats", 0},
//@todo(erwincoumans)	{ 1, "bullet", MyImportSTLCreateFunc},
    { 1, ".bullet",MySerializeCreateFunc},
	{ 1, "Wavefront Obj", MyImportObjCreateFunc},
    { 1, "URDF", MyImportUrdfCreateFunc },
	{ 1, "STL", MyImportSTLCreateFunc},

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

