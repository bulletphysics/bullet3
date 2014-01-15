
#ifndef BULLET_DEMO_ENTRIES_H
#define BULLET_DEMO_ENTRIES_H

#include "BulletDemoInterface.h"
#include "../bullet2/BasicDemo/BasicDemo.h"
#include "../bullet2/FeatherstoneMultiBodyDemo/BulletMultiBodyDemos.h"
#include "../bullet2/FeatherstoneMultiBodyDemo/MultiDofDemo.h"

#include "../bullet2/RagdollDemo/RagdollDemo.h"


struct BulletDemoEntry
{
	const char*							m_name;
	BulletDemoInterface::CreateFunc*	m_createFunc;
};


static BulletDemoEntry allDemos[]=
{
	
	//{"emptydemo",EmptyBulletDemo::MyCreateFunc},
	{"BasicDemo",BasicDemo::MyCreateFunc},
	{"Ragdoll",RagDollDemo::MyCreateFunc},
	{"MultiBody1",FeatherstoneDemo1::MyCreateFunc},
	{"MultiDofDemo",MultiDofDemo::MyCreateFunc},
	

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
		fscanf(f,"%d",&currentEntry);
		fclose(f);
	}
	return currentEntry;
};

#endif//BULLET_DEMO_ENTRIES_H

