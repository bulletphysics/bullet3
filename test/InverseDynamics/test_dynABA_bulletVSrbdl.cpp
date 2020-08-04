/// Then - run forward dynamics on random input data (q, u, dot_u) to get forces with rbdl
///      - run forward dynamics on random input data (q, u, dot_u) to get forces with bullet
///      - compare input accelerations

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <string>

#include <btBulletDynamicsCommon.h>
#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBodyPoint2Point.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <gtest/gtest.h>
#include "../../examples/CommonInterfaces/CommonGUIHelperInterface.h"
#include "../../examples/Importers/ImportURDFDemo/BulletUrdfImporter.h"
#include "../../examples/Importers/ImportURDFDemo/URDF2Bullet.h"
#include "../../examples/Importers/ImportURDFDemo/MyMultiBodyCreator.h"
#include "../../examples/Importers/ImportURDFDemo/URDF2Bullet.h"
#include "../../examples/Utils/b3ResourcePath.h"
#include "../../examples/SharedMemory/SharedMemoryPublic.h"
#include <btMultiBodyFromURDF.hpp>
#include <MultiBodyTreeDebugGraph.hpp>
#include "Bullet3Common/b3CommandLineArgs.h"
#include "Bullet3Common/b3Random.h"

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include "urdfreader.h"

using namespace std;
using namespace btInverseDynamics;
using namespace RigidBodyDynamics::Math;

bool verbose = false;
bool floatbase = false;

bool FLAGS_verbose = false;

static btVector3 gravity(0., 0., -10.);
static const bool kBaseFixed = true;
static const char kUrdfFile[] = "pantilt.urdf";


int compareABAForwardDynamicsBulletAndRbdl(btAlignedObjectArray<btScalar> &q, btAlignedObjectArray<btScalar> &u, btAlignedObjectArray<btScalar> &joint_forces, btAlignedObjectArray<btScalar> &dot_u, btVector3 &gravity, bool verbose,
                                           btMultiBody *btmb, double *acc_error)
{
// call function and return -1 if it does, printing an bt_id_error_message
#define RETURN_ON_FAILURE(x)                         \
	do                                               \
	{                                                \
		if (-1 == x)                                 \
		{                                            \
			bt_id_error_message("calling " #x "\n"); \
			return -1;                               \
		}                                            \
	} while (0)

	if (verbose)
	{
		printf("\n ===================================== \n");
	}

	// set positions and velocities for btMultiBody
	// base link
	btMatrix3x3 world_T_base;
	btVector3   world_pos_base;
	btVector3   base_velocity;
	btVector3   base_angular_velocity;

        world_T_base.setIdentity();
        base_velocity.setZero();
        base_angular_velocity.setZero();

	btmb->setBaseOmega(base_angular_velocity);
	btmb->setBaseVel(base_velocity);
	btmb->setLinearDamping(0);
	btmb->setAngularDamping(0);

	// remaining links
	int q_index;
	if (btmb->hasFixedBase())
	{
		q_index = 0;
	}
	else
	{
		q_index = 6;
	}
	if (verbose)
	{
		printf("bt:num_links= %d, num_dofs= %d\n", btmb->getNumLinks(), btmb->getNumDofs());
	}
	for (int l = 0; l < btmb->getNumLinks(); l++)
	{
		const btMultibodyLink &link = btmb->getLink(l);
		if (verbose)
		{
			printf("link %d, pos_var_count= %d, dof_count= %d\n", l, link.m_posVarCount,
				   link.m_dofCount);
		}
		if (link.m_posVarCount == 1)
		{
			btmb->setJointPosMultiDof(l, &q[q_index]);
			btmb->setJointVelMultiDof(l, &u[q_index]);
			if (verbose)
			{
				printf("set q[%d]= %f, u[%d]= %f\n", q_index, q[q_index], q_index, u[q_index]);
			}
			q_index++;
		}
	}
	// sanity check
	if (q_index != q.size())
	{
		bt_id_error_message("error in number of dofs for btMultibody and MultiBodyTree\n");
		return -1;
	}

	// set up bullet forward dynamics model
	btScalar dt = 0;
	btAlignedObjectArray<btScalar> scratch_r;
	btAlignedObjectArray<btVector3> scratch_v;
	btAlignedObjectArray<btMatrix3x3> scratch_m;
	// this triggers switch between using either appliedConstraintForce or appliedForce
	bool isConstraintPass = false;
	// apply gravity forces for btMultiBody model. Must be done manually.
	btmb->addBaseForce(btmb->getBaseMass() * gravity);

	for (int link = 0; link < btmb->getNumLinks(); link++)
	{
		btmb->addLinkForce(link, gravity * btmb->getLinkMass(link));
		if (verbose)
		{
			printf("link %d, applying gravity %f %f %f\n", link,
				   gravity[0] * btmb->getLinkMass(link), gravity[1] * btmb->getLinkMass(link),
				   gravity[2] * btmb->getLinkMass(link));
		}
	}

	// apply generalized forces
	if (btmb->hasFixedBase())
	{
		q_index = 0;
	}
	else
	{
		btVector3 base_force;
		base_force = btVector3(joint_forces[3], joint_forces[4], joint_forces[5]);

		btVector3 base_moment;
		base_moment = btVector3(joint_forces[0], joint_forces[1], joint_forces[2]);

		btmb->addBaseForce(world_T_base * base_force);
		btmb->addBaseTorque(world_T_base * base_moment);
		if (verbose)
		{
			printf("base force from id: %f %f %f\n", joint_forces[3], joint_forces[4],
				   joint_forces[5]);
			printf("base moment from id: %f %f %f\n", joint_forces[0], joint_forces[1],
				   joint_forces[2]);
		}
		q_index = 6;
	}

	for (int l = 0; l < btmb->getNumLinks(); l++)
	{
		const btMultibodyLink &link = btmb->getLink(l);
		if (link.m_posVarCount == 1)
		{
			if (verbose)
			{
				printf("id:joint_force[%d]= %f, applied to link %d\n", q_index,
					   joint_forces[q_index], l);
			}
			btmb->addJointTorque(l, joint_forces[q_index]);
			q_index++;
		}
	}

	// sanity check
	if (q_index != q.size())
	{
		bt_id_error_message("error in number of dofs for btMultibody and MultiBodyTree\n");
		return -1;
	}

	// run forward kinematics & forward dynamics
	btAlignedObjectArray<btQuaternion> world_to_local;
	btAlignedObjectArray<btVector3> local_origin;
	btmb->forwardKinematics(world_to_local, local_origin);
	btmb->computeAccelerationsArticulatedBodyAlgorithmMultiDof(dt, scratch_r, scratch_v, scratch_m, isConstraintPass, false, false);

	// read generalized accelerations back from btMultiBody
	// the mapping from scratch variables to accelerations is taken from the implementation
	// of stepVelocitiesMultiDof
	btScalar *base_accel = &scratch_r[btmb->getNumDofs()];
	btScalar *joint_accel = base_accel + 6;
        btVector3 * spatVel = &scratch_v[0];
        int linkNum = btmb->getNumLinks();
        btVector3 * spatAcc = &scratch_v[0] + (linkNum * 2 *3 + 2 + 2 ) * 1;  
        if (verbose)
        {
            printf("linkNum = %d nomdofs=%d\n", linkNum, btmb->getNumDofs());
        }
	*acc_error = 0;
	int dot_u_offset = 0;
	if (btmb->hasFixedBase())
	{
		dot_u_offset = 0;
	}
	else
	{
		dot_u_offset = 6;
	}

	if (true == btmb->hasFixedBase())
	{
		for (int i = 0; i < btmb->getNumDofs(); i++)
		{
			if (verbose)
			{
				printf("bt:ddot_q[%d]= %.20f, id:ddot_q= %.20f, diff= %.20f\n", i, joint_accel[i],
					   dot_u[i + dot_u_offset], joint_accel[i] - dot_u[i]);
			}
			*acc_error += BT_ID_POW(joint_accel[i] - dot_u[i + dot_u_offset], 2);
		}
	}
	else
	{
		btVector3 base_dot_omega;
		btVector3 world_dot_omega;
		world_dot_omega = btVector3(base_accel[0], base_accel[1], base_accel[2]);
		base_dot_omega  = world_T_base.transpose() * world_dot_omega;

		// com happens to coincide with link origin here. If that changes, we need to calculate
		// ddot_com
		btVector3 base_ddot_com;
		btVector3 world_ddot_com;
		world_ddot_com = btVector3(base_accel[3], base_accel[4], base_accel[5]);
		base_ddot_com  = world_T_base.transpose() * world_ddot_com;

		for (int i = 0; i < 3; i++)
		{
			if (verbose)
			{
				printf("bt::base_dot_omega(%d)= %e dot_u[%d]= %e, diff= %e\n", i, ((btScalar*)base_dot_omega)[i],
					   i, dot_u[i], ((btScalar*)base_dot_omega)[i] - dot_u[i]);
			}
			*acc_error += BT_ID_POW(((btScalar*)base_dot_omega)[i] - dot_u[i], 2);
		}
		for (int i = 0; i < 3; i++)
		{
			if (verbose)
			{
				printf("bt::base_ddot_com(%d)= %e dot_u[%d]= %e, diff= %e\n", i, ((btScalar*)base_ddot_com)[i],
					   i, dot_u[i + 3], ((btScalar*)base_ddot_com)[i] - dot_u[i + 3]);
			}
			*acc_error += BT_ID_POW(((btScalar*)base_ddot_com)[i] - dot_u[i + 3], 2);
		}

		for (int i = 0; i < btmb->getNumDofs(); i++)
		{
			if (verbose)
			{
				printf("bt:ddot_q[%d]= %.10f, id:ddot_q= %e, diff= %e\n", i, joint_accel[i],
					   dot_u[i + 6], joint_accel[i] - dot_u[i + 6]);
			}
			*acc_error += BT_ID_POW(joint_accel[i] - dot_u[i + 6], 2);
		}
	}
	*acc_error = std::sqrt(*acc_error);
	if (verbose)
	{
		printf("======dynamics-err: %e\n", *acc_error);
	}

	return 0;
}

/// this test loads the a urdf model with fixed, floating, prismatic and rotational joints,
/// converts in to an inverse dynamics model and compares forward to inverse dynamics for
/// random input
TEST(DynCompareBulletAndRbdl, UrdfPantilt)
{
	MyBtMultiBodyFromURDF mb_load(gravity, kBaseFixed);

	char relativeFileName[1024];

	ASSERT_TRUE(b3ResourcePath::findResourcePath(kUrdfFile, relativeFileName, 1024,0));

	mb_load.setFileName(relativeFileName);
        mb_load.setFlag(URDF_USE_INERTIA_FROM_FILE);
	mb_load.init();

	btMultiBody *btmb = mb_load.getBtMultiBody();
        int numDofs = btmb->getNumDofs();

	btAlignedObjectArray<btScalar> q;
	btAlignedObjectArray<btScalar> u;
	btAlignedObjectArray<btScalar> dot_u;
	btAlignedObjectArray<btScalar> joint_forces;
        q.resizeNoInitialize(numDofs);
        u.resizeNoInitialize(numDofs);
        dot_u.resizeNoInitialize(numDofs);
        joint_forces.resizeNoInitialize(numDofs);

	const int kNLoops = 5;
	double max_pos_error = 0;
	double max_acc_error = 0;

	b3Srand(0);

        RigidBodyDynamics::Model model;

        if (!RigidBodyDynamics::Addons::URDFReadFromFile(relativeFileName, &model, floatbase, verbose)) {
           printf("Loading of urdf model failed!\n");
           return;
        }
       
        model.gravity = Vector3d (0.,0., -10.); 

	for (int loop = 0; loop < kNLoops; loop++)
	{
		for (int i = 0; i < q.size(); i++)
		{
			q[i] = b3RandRange(-B3_PI, B3_PI);
			u[i] = b3RandRange(-B3_PI, B3_PI);
			joint_forces[i] = b3RandRange(0, 20);
		}
                // Initialization of the input vectors
                VectorNd Q     = VectorNd::Constant((size_t)model.dof_count, 0.);
                VectorNd QDot  = VectorNd::Constant((size_t)model.dof_count, 0.);
                VectorNd QDDot = VectorNd::Constant((size_t)model.dof_count, 0.);
                VectorNd Tau   = VectorNd::Constant((size_t)model.dof_count, 0.);
		for (int i = 0; i < q.size(); i++)
                {
                    Q[i]    = q[i];
                    QDot[i] = u[i];
                    Tau[i]  = joint_forces[i];
                }
                ForwardDynamics(model, Q, QDot, Tau, QDDot);                
                unsigned int i;

		for (int i = 0; i < q.size(); i++)
                {
                    dot_u[i]    = QDDot[i];
                }
		double acc_error;
		btmb->clearForcesAndTorques();
                EXPECT_EQ(compareABAForwardDynamicsBulletAndRbdl(q, u, joint_forces, dot_u, gravity, FLAGS_verbose, btmb,
                                                                 &acc_error),
                                                                 0);
		if (acc_error > max_acc_error)
		{
			max_acc_error = acc_error;
		}
	}

	if (FLAGS_verbose)
	{
		printf("max_acc_error= %e\n", max_acc_error);
	}

	EXPECT_LT(max_acc_error, std::numeric_limits<idScalar>::epsilon() * 1e5);
}

int main(int argc, char **argv)
{
	b3CommandLineArgs myArgs(argc, argv);
	FLAGS_verbose = myArgs.CheckCmdLineFlag("verbose");
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
