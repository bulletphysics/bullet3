#include "InverseKinematicsExample.h"

#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3Transform.h"
#include "Bullet3Common/b3AlignedObjectArray.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "../CommonInterfaces/CommonExampleInterface.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../OpenGLWindow/OpenGLInclude.h"

#include "BussIK/Node.h"
#include "BussIK/Tree.h"
#include "BussIK/Jacobian.h"
#include "BussIK/VectorRn.h"

#define RADIAN(X)	((X)*RadiansToDegrees)

#define MAX_NUM_NODE	1000
#define MAX_NUM_THETA	1000
#define MAX_NUM_EFFECT	100

double T = 0;
VectorR3 targetaa[MAX_NUM_EFFECT];



// Make slowdown factor larger to make the simulation take larger, less frequent steps
// Make the constant factor in Tstep larger to make time pass more quickly
//const int SlowdownFactor = 40;
const int SlowdownFactor = 0;		// Make higher to take larger steps less frequently
const int SleepsPerStep=SlowdownFactor;
int SleepCounter=0;
//const double Tstep = 0.0005*(double)SlowdownFactor;		// Time step


int	AxesList;		/* list to hold the axes		*/
int	AxesOn;			/* ON or OFF				*/

float	Scale, Scale2;		/* scaling factors			*/



int JointLimitsOn;
int RestPositionOn;
int UseJacobianTargets1;


int numIteration = 1;
double error = 0.0;
double errorDLS = 0.0;
double errorSDLS = 0.0;
double sumError = 0.0;
double sumErrorDLS = 0.0;
double sumErrorSDLS = 0.0;

#ifdef _DYNAMIC
bool initMaxDist = true;
extern double Excess[];
extern double dsnorm[];
#endif




void Reset(Tree &tree, Jacobian* m_ikJacobian)
{
	AxesOn = false;
	
	Scale  = 1.0;
	Scale2 = 0.0;		/* because add 1. to it in Display()	*/
	
	JointLimitsOn = true;
	RestPositionOn = false;
	UseJacobianTargets1 = false;
	
	                  
	tree.Init();
	tree.Compute();
	m_ikJacobian->Reset();

}

// Update target positions

void UpdateTargets( double T2, Tree & treeY) {
	double T = T2 / 5.;
	targetaa[0].Set(0.6*b3Sin(0), 0.6*b3Cos(0), 0.5+0.4*b3Sin(3 * T));
}


// Does a single update (on one kind of tree)
void DoUpdateStep(double Tstep, Tree & treeY, Jacobian *jacob, int ikMethod) {
	
	if ( SleepCounter==0 ) {
		T += Tstep;
		UpdateTargets( T , treeY);
	} 

	if ( UseJacobianTargets1 ) {
		jacob->SetJtargetActive();
	}
	else {
		jacob->SetJendActive();
	}
	jacob->ComputeJacobian(targetaa);						// Set up Jacobian and deltaS vectors

	// Calculate the change in theta values 
	switch (ikMethod) {
		case IK_JACOB_TRANS:
			jacob->CalcDeltaThetasTranspose();		// Jacobian transpose method
			break;
		case IK_DLS:
			jacob->CalcDeltaThetasDLS();			// Damped least squares method
			break;
        case IK_DLS_SVD:
            jacob->CalcDeltaThetasDLSwithSVD();
            break;
		case IK_PURE_PSEUDO:
			jacob->CalcDeltaThetasPseudoinverse();	// Pure pseudoinverse method
			break;
		case IK_SDLS:
			jacob->CalcDeltaThetasSDLS();			// Selectively damped least squares method
			break;
		default:
			jacob->ZeroDeltaThetas();
			break;
	}

	if ( SleepCounter==0 ) {
		jacob->UpdateThetas();							// Apply the change in the theta values
		jacob->UpdatedSClampValue(targetaa);
		SleepCounter = SleepsPerStep;
	}
	else { 
		SleepCounter--;
	}


}








///quick demo showing the right-handed coordinate system and positive rotations around each axis
class InverseKinematicsExample : public CommonExampleInterface
{
    CommonGraphicsApp* m_app;
	int m_ikMethod;
	Tree m_ikTree;
	b3AlignedObjectArray<Node*> m_ikNodes;
	Jacobian* m_ikJacobian;

 	b3AlignedObjectArray<int> m_movingInstances;
	int m_targetInstance;
	enum
	{
		numCubesX = 20,
		numCubesY = 20
	};
public:
    
    InverseKinematicsExample(CommonGraphicsApp* app, int option)
    :m_app(app),
	m_ikMethod(option),
	m_targetInstance(-1)
	{
		m_app->setUpAxis(2);
        
		 {
             b3Vector3 extents=b3MakeVector3(100,100,100);
             extents[m_app->getUpAxis()]=1;
             
			 int xres = 20;
			 int yres = 20;
			 
			 b3Vector4 color0=b3MakeVector4(0.4, 0.4, 0.4,1);
			 b3Vector4 color1=b3MakeVector4(0.6, 0.6, 0.6,1);
            m_app->registerGrid(xres, yres, color0, color1);
        }
		 
		 ///create some graphics proxy for the tracking target 
		 ///the endeffector tries to track it using Inverse Kinematics
		 {
			int sphereId = m_app->registerGraphicsUnitSphereShape(SPHERE_LOD_MEDIUM);
			b3Vector3 pos = b3MakeVector3(1,1,1);
			pos[app->getUpAxis()] = 1;
			b3Quaternion orn(0, 0, 0, 1);
			b3Vector4 color = b3MakeVector4(1., 0.3, 0.3, 1);
			b3Vector3 scaling = b3MakeVector3(.02, .02, .02);
			m_targetInstance = m_app->m_renderer->registerGraphicsInstance(sphereId, pos, orn, color, scaling);
		 }
		 m_app->m_renderer->writeTransforms();
    }
    virtual ~InverseKinematicsExample()
    {
    }

    
    virtual void physicsDebugDraw(int debugDrawMode)
    {
        
    }
    virtual void    initPhysics()
    {
		BuildKukaIIWAShape();
		m_ikJacobian = new Jacobian(&m_ikTree);
		Reset(m_ikTree,m_ikJacobian);
    }
    virtual void    exitPhysics()
    {
		delete m_ikJacobian;
		m_ikJacobian = 0;
    }

	void BuildKukaIIWAShape();

	void getLocalTransform(const Node* node, b3Transform& act)
	{
		b3Vector3 axis = b3MakeVector3(node->v.x, node->v.y, node->v.z);
		b3Quaternion rot(0, 0, 0, 1);
		if (axis.length())
		{
			rot = b3Quaternion (axis, node->theta);
		}
		act.setIdentity();
		act.setRotation(rot);
		act.setOrigin(b3MakeVector3(node->r.x, node->r.y, node->r.z));
	}
	void MyDrawTree(Node* node, const b3Transform& tr)
	{
		b3Vector3 lineColor = b3MakeVector3(0, 0, 0);
		int lineWidth = 2;
		if (node != 0) {
		//	glPushMatrix();
			b3Vector3 pos = b3MakeVector3(tr.getOrigin().x, tr.getOrigin().y, tr.getOrigin().z);
			b3Vector3 color = b3MakeVector3(0, 1, 0);
			int pointSize = 10;
			m_app->m_renderer->drawPoint(pos, color, pointSize);

			m_app->m_renderer->drawLine(pos, pos+ 0.05*tr.getBasis().getColumn(0), b3MakeVector3(1,0,0), lineWidth);
			m_app->m_renderer->drawLine(pos, pos + 0.05*tr.getBasis().getColumn(1), b3MakeVector3(0, 1, 0), lineWidth);
			m_app->m_renderer->drawLine(pos, pos + 0.05*tr.getBasis().getColumn(2), b3MakeVector3(0, 0, 1), lineWidth);
			
			b3Vector3 axisLocal = b3MakeVector3(node->v.x, node->v.y, node->v.z);
			b3Vector3 axisWorld = tr.getBasis()*axisLocal;

			m_app->m_renderer->drawLine(pos, pos + 0.1*axisWorld, b3MakeVector3(.2, 0.2, 0.7), 5);

			//node->DrawNode(node == root);	// Recursively draw node and update ModelView matrix
			if (node->left) {
				b3Transform act;
				getLocalTransform(node->left, act);
				
				b3Transform trl = tr*act;
				m_app->m_renderer->drawLine(tr.getOrigin(), trl.getOrigin(), lineColor, lineWidth);
				MyDrawTree(node->left, trl);		// Draw tree of children recursively
			}
		//	glPopMatrix();
			if (node->right) {
				b3Transform act;
				getLocalTransform(node->right, act);
				b3Transform trr = tr*act;
				m_app->m_renderer->drawLine(tr.getOrigin(), trr.getOrigin(), lineColor, lineWidth);
				MyDrawTree(node->right,trr);		// Draw right siblings recursively
			}
		}

	}
    virtual void	stepSimulation(float deltaTime)
    {
		DoUpdateStep(deltaTime, m_ikTree, m_ikJacobian, m_ikMethod);
    }
    virtual void	renderScene()
    {
		
		
		b3Transform act;
		getLocalTransform(m_ikTree.GetRoot(), act);
		MyDrawTree(m_ikTree.GetRoot(), act);
		
		b3Vector3 pos = b3MakeVector3(targetaa[0].x, targetaa[0].y, targetaa[0].z);
		b3Quaternion orn(0, 0, 0, 1);

		m_app->m_renderer->writeSingleInstanceTransformToCPU(pos, orn, m_targetInstance);
		m_app->m_renderer->writeTransforms();
		m_app->m_renderer->renderScene();
    }

	
    virtual void	physicsDebugDraw()
    {
      		
    }
    virtual bool	mouseMoveCallback(float x,float y)
    {
		return false;   
    }
    virtual bool	mouseButtonCallback(int button, int state, float x, float y)
    {
        return false;   
    }
    virtual bool	keyboardCallback(int key, int state)
    {
        return false;   
    }
    
	virtual void resetCamera()
	{
		float dist = 1.3;
		float pitch = -13;
		float yaw = 120;
		float targetPos[3]={-0.35,0.14,0.25};
		if (m_app->m_renderer  && m_app->m_renderer->getActiveCamera())
		{
			
			m_app->m_renderer->getActiveCamera()->setCameraDistance(dist);
			m_app->m_renderer->getActiveCamera()->setCameraPitch(pitch);
			m_app->m_renderer->getActiveCamera()->setCameraYaw(yaw);
			m_app->m_renderer->getActiveCamera()->setCameraTargetPosition(targetPos[0],targetPos[1],targetPos[2]);
		}
	}

};

void InverseKinematicsExample::BuildKukaIIWAShape()
{
	//const VectorR3& unitx = VectorR3::UnitX;
	const VectorR3& unity = VectorR3::UnitY;
	const VectorR3& unitz = VectorR3::UnitZ;
	const VectorR3 unit1(sqrt(14.0) / 8.0, 1.0 / 8.0, 7.0 / 8.0);
	const VectorR3& zero = VectorR3::Zero;

	float minTheta = -4 * PI;
	float maxTheta = 4 * PI;

	m_ikNodes.resize(8);//7DOF+additional endeffector

	m_ikNodes[0] = new Node(VectorR3(0.100000, 0.000000, 0.087500), unitz, 0.08, JOINT, -1e30, 1e30, RADIAN(0.));
	m_ikTree.InsertRoot(m_ikNodes[0]);

	m_ikNodes[1] = new Node(VectorR3(0.100000, -0.000000, 0.290000), unity, 0.08, JOINT, -0.5, 0.4, RADIAN(0.));
	m_ikTree.InsertLeftChild(m_ikNodes[0], m_ikNodes[1]);

	m_ikNodes[2] = new Node(VectorR3(0.100000, -0.000000, 0.494500), unitz, 0.08, JOINT, minTheta, maxTheta, RADIAN(0.));
	m_ikTree.InsertLeftChild(m_ikNodes[1], m_ikNodes[2]);

	m_ikNodes[3] = new Node(VectorR3(0.100000, 0.000000, 0.710000), -unity, 0.08, JOINT, minTheta, maxTheta, RADIAN(0.));
	m_ikTree.InsertLeftChild(m_ikNodes[2], m_ikNodes[3]);

	m_ikNodes[4] = new Node(VectorR3(0.100000, 0.000000, 0.894500), unitz, 0.08, JOINT, minTheta, maxTheta, RADIAN(0.));
	m_ikTree.InsertLeftChild(m_ikNodes[3], m_ikNodes[4]);

	m_ikNodes[5] = new Node(VectorR3(0.100000, 0.000000, 1.110000), unity, 0.08, JOINT, minTheta, maxTheta, RADIAN(0.));
	m_ikTree.InsertLeftChild(m_ikNodes[4], m_ikNodes[5]);

	m_ikNodes[6] = new Node(VectorR3(0.100000, 0.000000, 1.191000), unitz, 0.08, JOINT, minTheta, maxTheta, RADIAN(0.));
	m_ikTree.InsertLeftChild(m_ikNodes[5], m_ikNodes[6]);

	m_ikNodes[7] = new Node(VectorR3(0.100000, 0.000000, 1.20000), zero, 0.08, EFFECTOR);
	m_ikTree.InsertLeftChild(m_ikNodes[6], m_ikNodes[7]);

}


class	CommonExampleInterface*    InverseKinematicsExampleCreateFunc(struct CommonExampleOptions& options)
{
	return new InverseKinematicsExample(options.m_guiHelper->getAppInterface(), options.m_option);
}





