/* Copyright (C) 2015 Google

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "MultiBodyCustomURDFDemo.h"

#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"

#include "../CommonInterfaces/CommonMultiBodyBase.h"

#include "../Importers/ImportURDFDemo/URDF2Bullet.h"
#include "../Importers/ImportURDFDemo/MyURDFImporter.h"
#include "../Importers/ImportURDFDemo/MyMultiBodyCreator.h"

#include "CustomMultiBodyCreationCallback.h"


struct MultiBodyCustomURDFDemo : public CommonMultiBodyBase
{
    btMultiBody* m_multiBody;

public:

    MultiBodyCustomURDFDemo(struct GUIHelperInterface* helper);
    virtual ~MultiBodyCustomURDFDemo();

    virtual void initPhysics();

    virtual void stepSimulation(float deltaTime);

};

MultiBodyCustomURDFDemo::MultiBodyCustomURDFDemo(struct GUIHelperInterface* helper)
:CommonMultiBodyBase(helper)
{
}

MultiBodyCustomURDFDemo::~MultiBodyCustomURDFDemo()
{

}


static void myPrintTree(const URDFImporterInterface& u2b, int linkIndex, int indentationLevel)
{
    btAlignedObjectArray<int> childIndices;
    u2b.getLinkChildIndices(linkIndex,childIndices);
    
    int numChildren = childIndices.size();
    
    indentationLevel+=2;
    int count = 0;
    for (int i=0;i<numChildren;i++)
    {
        int childLinkIndex = childIndices[i];
        std::string name = u2b.getLinkName(childLinkIndex);
        for(int j=0;j<indentationLevel;j++) printf("  "); //indent
        printf("child(%d).name=%s with childIndex=%d\n",(count++)+1, name.c_str(),childLinkIndex);
        // first grandchild
        printTree(u2b,childLinkIndex,indentationLevel);
    }
}




class TestMultiBodyCreationCallback : public CustomMultiBodyCreationCallback
{
	virtual int allocateMultiBodyBase(int urdfLinkIndex, int totalNumJoints,ScalarType baseMass, const Vector3dType& localInertiaDiagonal, bool isFixedBase)
	{
		printf("added base\n");
	}

    virtual void addLinkAndJoint(int jointType, int linkIndex,            // 0 to num_links-1
                       int parentIndex,
                       double mass,
                       const Vector3dType&	inertia,
                       const QuaternionType &rotParentFrameToLinkFrame,  // rotate points in parent frame to this frame, when q = 0
                       const Vector3dType&	jointAxisInLinkFrame,    // in Link frame
                       const Vector3dType&	parentComToThisJointOffset,    // vector from parent COM to joint frame, in Parent frame
                       const Vector3dType&	thisJointToThisComOffset)       // vector from joint frame to my COM, in Link frame);
	{
		printf("added link\n");
	}
                 
};

void MultiBodyCustomURDFDemo::initPhysics()
{
    int upAxis = 2;
    m_guiHelper->setUpAxis(upAxis);
	createEmptyDynamicsWorld();
	btVector3 grav(0,0,0);
	//grav[upAxis] = -10.f;

	m_dynamicsWorld->setGravity(grav);

    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);


	
    MyURDFImporter u2b(m_guiHelper);
	bool loadOk =  u2b.loadURDF("r2d2.urdf");

	if (loadOk)
	{
		u2b.printTree();
		int urdfRootLinkIndex = u2b.getRootLinkIndex();
		myPrintTree(u2b,urdfRootLinkIndex,0);

		btTransform identityTrans;
		identityTrans.setIdentity();
	
		btMultiBody* mb = 0;
        

		//todo: move these internal API called inside the 'ConvertURDF2Bullet' call, hidden from the user
		int rootLinkIndex = u2b.getRootLinkIndex();
		printf("urdf root link index = %d\n",rootLinkIndex);
		MyMultiBodyCreator creation(m_guiHelper);

		bool useMultiBody = true;

		ConvertURDF2Bullet(u2b,creation, identityTrans,m_dynamicsWorld,useMultiBody,u2b.getPathPrefix());
		mb = creation.getBulletMultiBody();

	}
}

void MultiBodyCustomURDFDemo::stepSimulation(float deltaTime)
{
    m_dynamicsWorld->stepSimulation(deltaTime);
}


class CommonExampleInterface*    MultiBodyCustomURDFDemoCreateFunc(struct PhysicsInterface* pint, struct GUIHelperInterface* helper, int option)
{
	return new MultiBodyCustomURDFDemo(helper);
}