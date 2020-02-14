#include "InverseKinematicsExample.h"

#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3Transform.h"
#include "Bullet3Common/b3AlignedObjectArray.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "../CommonInterfaces/CommonExampleInterface.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"

#include "BussIK/Node.h"
#include "BussIK/Tree.h"
#include "BussIK/Jacobian.h"
#include "BussIK/VectorRn.h"

#define RADIAN(X) ((X)*RadiansToDegrees)

#define MAX_NUM_NODE 1000
#define MAX_NUM_THETA 1000
#define MAX_NUM_EFFECT 100

double T = 0;
VectorR3 targetaa[MAX_NUM_EFFECT];

// Make slowdown factor larger to make the simulation take larger, less frequent steps
// Make the constant factor in Tstep larger to make time pass more quickly
//const int SlowdownFactor = 40;
const int SlowdownFactor = 0;  // Make higher to take larger steps less frequently
const int SleepsPerStep = SlowdownFactor;
int SleepCounter = 0;
//const double Tstep = 0.0005*(double)SlowdownFactor;		// Time step

int AxesList; /* list to hold the axes		*/
int AxesOn;   /* ON or OFF				*/

float Scale, Scale2; /* scaling factors			*/

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

void Reset(Tree& tree, Jacobian* m_ikJacobian)
{
	AxesOn = false;

	Scale = 1.0;
	Scale2 = 0.0; /* because add 1. to it in Display()	*/

	JointLimitsOn = true;
	RestPositionOn = false;
	UseJacobianTargets1 = false;

	tree.Init();
	tree.Compute();
	m_ikJacobian->Reset();
}

// Update target positions

void UpdateTargets(double T, Tree& treeY)
{
	targetaa[0].Set(2.0f + 1.5*sin(3 * T) * 2, -0.5 + 1.0f + 0.2*sin(7 * T) * 2, 0.3f + 0.7*sin(5 * T) * 2);
	targetaa[1].Set(0.5f + 0.4*sin(4 * T) * 2, -0.5 + 0.9f + 0.3*sin(4 * T) * 2, -0.2f + 1.0*sin(3 * T) * 2);
	targetaa[2].Set(-0.5f + 0.8*sin(6 * T) * 2, -0.5 + 1.1f + 0.2*sin(7 * T) * 2, 0.3f + 0.5*sin(8 * T) * 2);
	targetaa[3].Set(-1.6f + 0.8*sin(4 * T) * 2, -0.5 + 0.8f + 0.3*sin(4 * T) * 2, -0.2f + 0.3*sin(3 * T) * 2);

}

// Does a single update (on one kind of m_ikTree)
void DoUpdateStep(double Tstep, Tree& treeY, Jacobian* jacob, int ikMethod)
{
	B3_PROFILE("IK_DoUpdateStep");
	if (SleepCounter == 0)
	{
		T += Tstep*0.1;
		UpdateTargets(T, treeY);
	}

	if (UseJacobianTargets1)
	{
		jacob->SetJtargetActive();
	}
	else
	{
		jacob->SetJendActive();
	}
	jacob->ComputeJacobian(targetaa);  // Set up Jacobian and deltaS vectors
	MatrixRmn AugMat;
	// Calculate the change in theta values
	switch (ikMethod)
	{
		case IK_JACOB_TRANS:
			jacob->CalcDeltaThetasTranspose();  // Jacobian transpose method
			break;
		case IK_DLS:
			jacob->CalcDeltaThetasDLS(AugMat);  // Damped least squares method
			break;
		case IK_DLS_SVD:
			jacob->CalcDeltaThetasDLSwithSVD();
			break;
		case IK_PURE_PSEUDO:
			jacob->CalcDeltaThetasPseudoinverse();  // Pure pseudoinverse method
			break;
		case IK_SDLS:
			jacob->CalcDeltaThetasSDLS();  // Selectively damped least squares method
			break;
		default:
			jacob->ZeroDeltaThetas();
			break;
	}

	if (SleepCounter == 0)
	{
		jacob->UpdateThetas();  // Apply the change in the theta values
		jacob->UpdatedSClampValue(targetaa);
		SleepCounter = SleepsPerStep;
	}
	else
	{
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
	b3AlignedObjectArray<int> m_targetInstances;
	enum
	{
		numCubesX = 20,
		numCubesY = 20
	};

public:
	InverseKinematicsExample(CommonGraphicsApp* app, int option)
		: m_app(app),
		  m_ikMethod(option)		  
	{
		m_app->setUpAxis(1);

		{
			b3Vector3 extents = b3MakeVector3(100, 100, 100);
			extents[m_app->getUpAxis()] = 1;

			int xres = 20;
			int yres = 20;

			b3Vector4 color0 = b3MakeVector4(0.4, 0.4, 0.4, 1);
			b3Vector4 color1 = b3MakeVector4(0.6, 0.6, 0.6, 1);
			//m_app->registerGrid(xres, yres, color0, color1);
		}

		///create some graphics proxy for the tracking target
		///the endeffector tries to track it using Inverse Kinematics
		{
			int sphereId = m_app->registerGraphicsUnitSphereShape(SPHERE_LOD_MEDIUM);
			b3Vector3 pos = b3MakeVector3(1, 1, 1);
			pos[app->getUpAxis()] = 1;
			b3Quaternion orn(0, 0, 0, 1);
			b3Vector4 color = b3MakeVector4(1., 0.3, 0.3, 1);
			b3Vector3 scaling = b3MakeVector3(.1, .1, .1);
			m_targetInstances.push_back(m_app->m_renderer->registerGraphicsInstance(sphereId, pos, orn, color, scaling));
			m_targetInstances.push_back(m_app->m_renderer->registerGraphicsInstance(sphereId, pos, orn, color, scaling));
			m_targetInstances.push_back(m_app->m_renderer->registerGraphicsInstance(sphereId, pos, orn, color, scaling));
			m_targetInstances.push_back(m_app->m_renderer->registerGraphicsInstance(sphereId, pos, orn, color, scaling));
		}
		m_app->m_renderer->writeTransforms();
	}
	virtual ~InverseKinematicsExample()
	{
	}

	virtual void physicsDebugDraw(int debugDrawMode)
	{
	}
	virtual void initPhysics()
	{
		BuildKukaIIWAShape();
		m_ikJacobian = new Jacobian(&m_ikTree);
		Reset(m_ikTree, m_ikJacobian);
	}
	virtual void exitPhysics()
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
			rot = b3Quaternion(axis, node->theta);
		}
		act.setIdentity();
		act.setRotation(rot);
		act.setOrigin(b3MakeVector3(node->r.x, node->r.y, node->r.z));
	}
	void MyDrawTree(Node* node, const b3Transform& tr, const b3Transform& parentTr)
	{
		
		int lineWidth = 2;
		if (node != 0)
		{
			b3Vector3 pos = b3MakeVector3(tr.getOrigin().x, tr.getOrigin().y, tr.getOrigin().z);
			b3Vector3 color1 = b3MakeVector3(0, 1, 0);
			int pointSize = 10;
			m_app->m_renderer->drawPoint(pos, color1, pointSize);

			m_app->m_renderer->drawLine(pos, pos + 0.05 * tr.getBasis().getColumn(0), b3MakeVector3(1, 0, 0), lineWidth);
			m_app->m_renderer->drawLine(pos, pos + 0.05 * tr.getBasis().getColumn(1), b3MakeVector3(0, 1, 0), lineWidth);
			m_app->m_renderer->drawLine(pos, pos + 0.05 * tr.getBasis().getColumn(2), b3MakeVector3(0, 0, 1), lineWidth);

			b3Vector3 axisLocal = b3MakeVector3(node->v.x, node->v.y, node->v.z);
			b3Vector3 axisWorld = tr.getBasis() * axisLocal;

			m_app->m_renderer->drawLine(pos, pos + 0.1 * axisWorld, b3MakeVector3(.2, 0.2, 0.7), 5);

			if (node->right)
			{
				b3Transform act;
				getLocalTransform(node->right, act);
				b3Transform trr = tr * act;
				b3Transform ptrr = parentTr * act;
				b3Vector3 lineColor = b3MakeVector3(0, 1, 0);
				m_app->m_renderer->drawLine(tr.getOrigin(), ptrr.getOrigin(), lineColor, lineWidth);
				MyDrawTree(node->right, ptrr, parentTr);  // Draw right siblings recursively
			}

			//node->DrawNode(node == root);	// Recursively draw node and update ModelView matrix
			if (node->left)
			{
				b3Transform act;
				getLocalTransform(node->left, act);
				b3Vector3 lineColor = b3MakeVector3(1, 0, 0);
				b3Transform trl = tr * act;
				m_app->m_renderer->drawLine(tr.getOrigin(), trl.getOrigin(), lineColor, lineWidth);
				MyDrawTree(node->left, trl, tr);  // Draw m_ikTree of children recursively
			}
			
		}
	}
	virtual void stepSimulation(float deltaTime)
	{
		DoUpdateStep(deltaTime, m_ikTree, m_ikJacobian, m_ikMethod);
	}
	virtual void renderScene()
	{
		b3Transform act;
		getLocalTransform(m_ikTree.GetRoot(), act);
		MyDrawTree(m_ikTree.GetRoot(), act, b3Transform::getIdentity());

		for (int i = 0; i < m_targetInstances.size(); i++)
		{
			b3Vector3 pos = b3MakeVector3(targetaa[i].x, targetaa[i].y, targetaa[i].z);
			b3Quaternion orn(0, 0, 0, 1);

			m_app->m_renderer->writeSingleInstanceTransformToCPU(pos, orn, m_targetInstances[i]);
		}
		m_app->m_renderer->writeTransforms();
		m_app->m_renderer->renderScene();
	}

	virtual void physicsDebugDraw()
	{
	}
	virtual bool mouseMoveCallback(float x, float y)
	{
		return false;
	}
	virtual bool mouseButtonCallback(int button, int state, float x, float y)
	{
		return false;
	}
	virtual bool keyboardCallback(int key, int state)
	{
		return false;
	}

	virtual void resetCamera()
	{
		float dist = 1.3;
		float pitch = -13;
		float yaw = 120;
		float targetPos[3] = {-0.35, 0.14, 0.25};
		if (m_app->m_renderer && m_app->m_renderer->getActiveCamera())
		{
			m_app->m_renderer->getActiveCamera()->setCameraDistance(dist);
			m_app->m_renderer->getActiveCamera()->setCameraPitch(pitch);
			m_app->m_renderer->getActiveCamera()->setCameraYaw(yaw);
			m_app->m_renderer->getActiveCamera()->setCameraTargetPosition(targetPos[0], targetPos[1], targetPos[2]);
		}
	}
};

void InverseKinematicsExample::BuildKukaIIWAShape()
{
	m_ikNodes.resize(29);
	const VectorR3& unitx = VectorR3::UnitX;
	const VectorR3& unity = VectorR3::UnitY;
	const VectorR3& unitz = VectorR3::UnitZ;
	const VectorR3 unit1(sqrt(14.0) / 8.0, 1.0 / 8.0, 7.0 / 8.0);
	const VectorR3& zero = VectorR3::Zero;
	VectorR3 p0(0.0f, -1.5f, 0.0f);
	VectorR3 p1(0.0f, -1.0f, 0.0f);
	VectorR3 p2(0.0f, -0.5f, 0.0f);
	VectorR3 p3(0.5f*Root2Inv, -0.5 + 0.5*Root2Inv, 0.0f);
	VectorR3 p4(0.5f*Root2Inv + 0.5f*HalfRoot3, -0.5 + 0.5*Root2Inv + 0.5f*0.5, 0.0f);
	VectorR3 p5(0.5f*Root2Inv + 1.0f*HalfRoot3, -0.5 + 0.5*Root2Inv + 1.0f*0.5, 0.0f);
	VectorR3 p6(0.5f*Root2Inv + 1.5f*HalfRoot3, -0.5 + 0.5*Root2Inv + 1.5f*0.5, 0.0f);
	VectorR3 p7(0.5f*Root2Inv + 0.5f*HalfRoot3, -0.5 + 0.5*Root2Inv + 0.5f*HalfRoot3, 0.0f);
	VectorR3 p8(0.5f*Root2Inv + 1.0f*HalfRoot3, -0.5 + 0.5*Root2Inv + 1.0f*HalfRoot3, 0.0f);
	VectorR3 p9(0.5f*Root2Inv + 1.5f*HalfRoot3, -0.5 + 0.5*Root2Inv + 1.5f*HalfRoot3, 0.0f);
	VectorR3 p10(-0.5f*Root2Inv, -0.5 + 0.5*Root2Inv, 0.0f);
	VectorR3 p11(-0.5f*Root2Inv - 0.5f*HalfRoot3, -0.5 + 0.5*Root2Inv + 0.5f*HalfRoot3, 0.0f);
	VectorR3 p12(-0.5f*Root2Inv - 1.0f*HalfRoot3, -0.5 + 0.5*Root2Inv + 1.0f*HalfRoot3, 0.0f);
	VectorR3 p13(-0.5f*Root2Inv - 1.5f*HalfRoot3, -0.5 + 0.5*Root2Inv + 1.5f*HalfRoot3, 0.0f);
	VectorR3 p14(-0.5f*Root2Inv - 0.5f*HalfRoot3, -0.5 + 0.5*Root2Inv + 0.5f*0.5, 0.0f);
	VectorR3 p15(-0.5f*Root2Inv - 1.0f*HalfRoot3, -0.5 + 0.5*Root2Inv + 1.0f*0.5, 0.0f);
	VectorR3 p16(-0.5f*Root2Inv - 1.5f*HalfRoot3, -0.5 + 0.5*Root2Inv + 1.5f*0.5, 0.0f);

	m_ikNodes[0] = new Node(p0, unit1, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
	m_ikTree.InsertRoot(m_ikNodes[0]);

	m_ikNodes[1] = new Node(p1, unitx, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
	m_ikTree.InsertLeftChild(m_ikNodes[0], m_ikNodes[1]);

	m_ikNodes[2] = new Node(p1, unitz, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
	m_ikTree.InsertLeftChild(m_ikNodes[1], m_ikNodes[2]);

	m_ikNodes[3] = new Node(p2, unitz, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
	m_ikTree.InsertLeftChild(m_ikNodes[2], m_ikNodes[3]);

	m_ikNodes[4] = new Node(p2, unitz, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
	m_ikTree.InsertRightSibling(m_ikNodes[3], m_ikNodes[4]);

	m_ikNodes[5] = new Node(p3, unity, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
	m_ikTree.InsertLeftChild(m_ikNodes[3], m_ikNodes[5]);

	m_ikNodes[6] = new Node(p3, unity, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
	m_ikTree.InsertRightSibling(m_ikNodes[5], m_ikNodes[6]);

	m_ikNodes[7] = new Node(p3, unitx, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
	m_ikTree.InsertLeftChild(m_ikNodes[5], m_ikNodes[7]);

	m_ikNodes[8] = new Node(p4, unitz, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
	m_ikTree.InsertLeftChild(m_ikNodes[7], m_ikNodes[8]);

	m_ikNodes[9] = new Node(p5, unitx, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
	m_ikTree.InsertLeftChild(m_ikNodes[8], m_ikNodes[9]);

	m_ikNodes[10] = new Node(p5, unity, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
	m_ikTree.InsertLeftChild(m_ikNodes[9], m_ikNodes[10]);

	m_ikNodes[11] = new Node(p6, zero, 0.08, EFFECTOR);
	m_ikTree.InsertLeftChild(m_ikNodes[10], m_ikNodes[11]);

	m_ikNodes[12] = new Node(p3, unitx, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
	m_ikTree.InsertLeftChild(m_ikNodes[6], m_ikNodes[12]);

	m_ikNodes[13] = new Node(p7, unitz, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
	m_ikTree.InsertLeftChild(m_ikNodes[12], m_ikNodes[13]);

	m_ikNodes[14] = new Node(p8, unitx, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
	m_ikTree.InsertLeftChild(m_ikNodes[13], m_ikNodes[14]);

	m_ikNodes[15] = new Node(p8, unity, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
	m_ikTree.InsertLeftChild(m_ikNodes[14], m_ikNodes[15]);

	m_ikNodes[16] = new Node(p9, zero, 0.08, EFFECTOR);
	m_ikTree.InsertLeftChild(m_ikNodes[15], m_ikNodes[16]);

	m_ikNodes[17] = new Node(p10, unity, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
	m_ikTree.InsertLeftChild(m_ikNodes[4], m_ikNodes[17]);

	m_ikNodes[18] = new Node(p10, unitx, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
	m_ikTree.InsertLeftChild(m_ikNodes[17], m_ikNodes[18]);

	m_ikNodes[19] = new Node(p10, unity, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
	m_ikTree.InsertRightSibling(m_ikNodes[17], m_ikNodes[19]);

	m_ikNodes[20] = new Node(p11, unitz, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
	m_ikTree.InsertLeftChild(m_ikNodes[18], m_ikNodes[20]);

	m_ikNodes[21] = new Node(p12, unitx, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
	m_ikTree.InsertLeftChild(m_ikNodes[20], m_ikNodes[21]);

	m_ikNodes[22] = new Node(p12, unity, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
	m_ikTree.InsertLeftChild(m_ikNodes[21], m_ikNodes[22]);

	m_ikNodes[23] = new Node(p13, zero, 0.08, EFFECTOR);
	m_ikTree.InsertLeftChild(m_ikNodes[22], m_ikNodes[23]);

	m_ikNodes[24] = new Node(p10, unitx, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
	m_ikTree.InsertLeftChild(m_ikNodes[19], m_ikNodes[24]);

	m_ikNodes[25] = new Node(p14, unitz, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
	m_ikTree.InsertLeftChild(m_ikNodes[24], m_ikNodes[25]);

	m_ikNodes[26] = new Node(p15, unitx, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
	m_ikTree.InsertLeftChild(m_ikNodes[25], m_ikNodes[26]);

	m_ikNodes[27] = new Node(p15, unity, 0.08, JOINT, RADIAN(-180.), RADIAN(180.), RADIAN(30.));
	m_ikTree.InsertLeftChild(m_ikNodes[26], m_ikNodes[27]);

	m_ikNodes[28] = new Node(p16, zero, 0.08, EFFECTOR);
	m_ikTree.InsertLeftChild(m_ikNodes[27], m_ikNodes[28]);

}

class CommonExampleInterface* InverseKinematicsExampleCreateFunc(struct CommonExampleOptions& options)
{
	return new InverseKinematicsExample(options.m_guiHelper->getAppInterface(), options.m_option);
}
