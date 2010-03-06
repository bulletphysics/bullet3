#include "oecakeLoader.h"
#include <stdio.h> //printf debugging
#include <stdlib.h>




btCompoundShape* shiftTransform(btCompoundShape* boxCompound,btScalar mass,btTransform& shift)
{
	btTransform principal;
	btVector3 principalInertia;
	btScalar* masses = new btScalar[boxCompound->getNumChildShapes()];
	for (int j=0;j<boxCompound->getNumChildShapes();j++)
	{
		//evenly distribute mass
		masses[j]=mass/boxCompound->getNumChildShapes();
	}


	boxCompound->calculatePrincipalAxisTransform(masses,principal,principalInertia);


	///create a new compound with world transform/center of mass properly aligned with the principal axis

	///non-recursive compound shapes perform better
	//#define USE_RECURSIVE_COMPOUND 1
#ifdef USE_RECURSIVE_COMPOUND

	btCompoundShape* newCompound = new btCompoundShape();
	newCompound->addChildShape(principal.inverse(),boxCompound);
	m_collisionShapes.push_back(newCompound);

	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,newCompound,principalInertia);

#else
#ifdef CHANGE_COMPOUND_INPLACE
	for (int i=0;i<boxCompound->getNumChildShapes();i++)
	{
		btTransform newChildTransform = principal.inverse()*boxCompound->getChildTransform(i);
		///updateChildTransform is really slow, because it re-calculates the AABB each time. todo: add option to disable this update
		boxCompound->updateChildTransform(i,newChildTransform);
	}
	if (isDynamic)
		boxCompound->calculateLocalInertia(mass,localInertia);
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,boxCompound,localInertia);
#else
	///creation is faster using a new compound to store the shifted children
	btCompoundShape* newBoxCompound = new btCompoundShape();
	for (int i=0;i<boxCompound->getNumChildShapes();i++)
	{
		btTransform newChildTransform = principal.inverse()*boxCompound->getChildTransform(i);
		///updateChildTransform is really slow, because it re-calculates the AABB each time. todo: add option to disable this update
		newBoxCompound->addChildShape(newChildTransform,boxCompound->getChildShape(i));
	}



#endif

#endif//USE_RECURSIVE_COMPOUND

	shift = principal;
	return newBoxCompound;
}


void BasicOECakeReader::addParticle(int materialType, int pIndex, int pColor, float pPosX, float pPosY,float radius)
{
	//determine that we have a new shape?
	if (m_particlePositions.size())
	{
		if (
			(materialType != m_materialType)
			 ||
			 (pIndex != m_particleObjectIndex)
			)
		{
			convertParticleGroup();
		}
	}
	

	//add to array
	m_materialType = materialType;
	m_particleObjectIndex = pIndex;
	m_particleColor = pColor;
	m_particlePositions.push_back(btVector3(pPosX,pPosY,0.));
	m_particleRadii.push_back(radius);

}

void	BasicOECakeReader::convertParticleGroup()
{
	printf("found a particle group of %d particles\n",m_particlePositions.size());
	if (m_particlePositions.size()>0)
	{
		addNewCollisionShape(m_particlePositions.size(),&m_particlePositions[0],&m_particleRadii[0],m_materialType,m_particleObjectIndex,m_particleColor);
		m_particlePositions.clear();
		m_particleRadii.clear();
	}
}

void	BasicOECakeReader::addNewCollisionShape(int numParticles, btVector3* particlePositions, btScalar* radii, int materialType, int objectIndex,int color )
{
	//create Bullet stuff
	btCompoundShape* colShape = 0;
	btScalar mass;

	bool addConstraint = false;


	if (materialType&0x800000)
	{
		addConstraint = true;
	}
	
	if ((materialType & 0x20000) ||(materialType & 0x12))
	{
		mass = 1.f;
	} else
	{
			mass = 0.f;
	}
	btTransform	startTransform;
	startTransform.setIdentity();

	int numCurSpheres = 0;

	{
		
	
		btTransform localTrans;
		localTrans.setIdentity();

		//static
		btCompoundShape* compound = new btCompoundShape();
		
		for (int i=0;i<numParticles;i++)
		{
			numCurSpheres++;
			localTrans.setOrigin(particlePositions[i]);
			btSphereShape* particle = new btSphereShape(radii[i]);
			compound->addChildShape(localTrans,particle);
			if (mass==0.f && (numCurSpheres>=7))
			{
				createBodyForCompoundShape(compound,false,startTransform,mass);
				compound = new btCompoundShape();
				numCurSpheres = 0;
			}
		}
		if (mass)
		{
			//shift the center of mass, based on all spheres
			btCompoundShape* newCompound = shiftTransform(compound,mass,startTransform);
			colShape = newCompound;
		} else
		{
			//use unmodified
			colShape = compound;
		}
	}

	btDefaultMotionState* myMotionState  = 0;

	if (colShape && numCurSpheres)
	{
		createBodyForCompoundShape(colShape,addConstraint,startTransform,mass);
	}
}



int BasicOECakeReader::processLine(char * buffer, int size)
{
	int numBytesRead  = 0;

	if (buffer[0] == 'p') 
	{
		int materialType;
		int	particleObjectIndex;
		int particleColor;
		int dummy1;
		float particlePosX;
		float particlePosY;

		if (sscanf (buffer, "p %x %x %x %x %f %f", &materialType,&particleObjectIndex,&dummy1, &particleColor,  &particlePosX, &particlePosY) == 6)
		{
			addParticle(materialType,particleObjectIndex,particleColor,particlePosX,particlePosY);
		}
		else
		{
			printf("ERROR: invalid line (%s)\n", buffer);
		}
	}
	
	while (*buffer != '\n' && size != 0)
	{
		buffer++;
		numBytesRead++;
	}

	
	if (buffer[0]==0x0a)
	{
		buffer++;
		numBytesRead++;
	}
	

	return numBytesRead;
}

bool BasicOECakeReader::processFile(char * fileName)
{
	FILE * fp = fopen(fileName, "rb");
	if (fp == NULL)
	{
		printf("ERROR: file(%s) not found", fileName);
		return false;
	}

	int size;
	if (fseek(fp, 0, SEEK_END) || (size = ftell(fp)) == EOF || fseek(fp, 0, SEEK_SET)) 
	{
		printf("ERROR: problem reading file(%s)", fileName);
		fclose(fp);
		return false;
	}
	else
	{
		rewind (fp);
		char * buffer = (char *) malloc(size+1);
		memset(buffer,0,size);
		
		if (fread(buffer,1,size,fp) != size)
		{
			printf("Error reading file %s!\n",fileName);
			fclose(fp);
			return false;
		}

		int totalBytesRead = 0;

		while(totalBytesRead<size)
		{
			int remainingSize = size-totalBytesRead;
			if (remainingSize<1229)

			{
				printf("..");
			}

			
			totalBytesRead +=processLine(&buffer[totalBytesRead],remainingSize);
		}
	}

	convertParticleGroup();

	fclose(fp);
	return false;
}
