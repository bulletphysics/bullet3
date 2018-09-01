#include "ConvertGRPCBullet.h"
#include "PhysicsClientC_API.h"
#include "SharedMemoryCommands.h"
#include <memory>
#include <iostream>
#include <string>
#include <thread>
#include <grpc++/grpc++.h>
#include <grpc/support/log.h>
#include "pybullet.grpc.pb.h"
#include "LinearMath/btMinMax.h"

using grpc::Server;
using grpc::ServerAsyncResponseWriter;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::ServerCompletionQueue;
using grpc::Status;
using pybullet_grpc::PyBulletCommand;
using pybullet_grpc::PyBulletStatus;
using pybullet_grpc::PyBulletAPI;


SharedMemoryCommand* convertGRPCAndSubmitCommand(PyBulletCommand& grpcCommand, SharedMemoryCommand& cmd)
{
	SharedMemoryCommand* cmdPtr = 0;

	if (grpcCommand.has_loadurdfcommand())
	{
		const ::pybullet_grpc::LoadUrdfCommand& grpcCmd = grpcCommand.loadurdfcommand();

		std::string fileName = grpcCmd.filename();
		if (fileName.length())
		{
			cmdPtr = &cmd;
			b3SharedMemoryCommandHandle commandHandle = (b3SharedMemoryCommandHandle)cmdPtr;
			b3LoadUrdfCommandInit2(commandHandle, fileName.c_str());

			if (grpcCmd.has_initialposition())
			{
				const ::pybullet_grpc::vec3& pos = grpcCmd.initialposition();
				b3LoadUrdfCommandSetStartPosition(commandHandle, pos.x(), pos.y(), pos.z());
			}
			if (grpcCmd.has_initialorientation())
			{
				const ::pybullet_grpc::quat4& orn = grpcCmd.initialorientation();
				b3LoadUrdfCommandSetStartOrientation(commandHandle, orn.x(), orn.y(), orn.z(), orn.w());
			}
			if (grpcCmd.hasUseMultiBody_case() == ::pybullet_grpc::LoadUrdfCommand::HasUseMultiBodyCase::kUseMultiBody)
			{
				b3LoadUrdfCommandSetUseMultiBody(commandHandle, grpcCmd.usemultibody());
			}
			if (grpcCmd.hasGlobalScaling_case() == ::pybullet_grpc::LoadUrdfCommand::HasGlobalScalingCase::kGlobalScaling)
			{
				b3LoadUrdfCommandSetGlobalScaling(commandHandle, grpcCmd.globalscaling());
			}
			if (grpcCmd.hasUseFixedBase_case() == ::pybullet_grpc::LoadUrdfCommand::HasUseFixedBaseCase::kUseFixedBase)
			{
				b3LoadUrdfCommandSetUseFixedBase(commandHandle, grpcCmd.usefixedbase());
			}
			if (grpcCmd.flags())
			{
				b3LoadUrdfCommandSetFlags(commandHandle, grpcCmd.flags());
			}

		}
	}

	if (grpcCommand.has_loadsdfcommand())
	{
		const ::pybullet_grpc::LoadSdfCommand&  grpcCmd = grpcCommand.loadsdfcommand();

		std::string fileName = grpcCmd.filename();
		if (fileName.length())
		{
			cmdPtr = &cmd;
			b3SharedMemoryCommandHandle commandHandle = (b3SharedMemoryCommandHandle)cmdPtr;
			b3LoadSdfCommandInit2(commandHandle, fileName.c_str());


			if (grpcCmd.hasUseMultiBody_case() == ::pybullet_grpc::LoadSdfCommand::HasUseMultiBodyCase::kUseMultiBody)
			{
				b3LoadSdfCommandSetUseMultiBody(commandHandle, grpcCmd.usemultibody());
			}
			if (grpcCmd.hasGlobalScaling_case() == ::pybullet_grpc::LoadSdfCommand::HasGlobalScalingCase::kGlobalScaling)
			{
				b3LoadSdfCommandSetUseGlobalScaling(commandHandle, grpcCmd.globalscaling());
			}
		}
	}

	if (grpcCommand.has_loadmjcfcommand())
	{
		const pybullet_grpc::LoadMjcfCommand& grpcCmd = grpcCommand.loadmjcfcommand();

		std::string fileName = grpcCmd.filename();
		if (fileName.length())
		{
			cmdPtr = &cmd;
			b3SharedMemoryCommandHandle commandHandle = (b3SharedMemoryCommandHandle)cmdPtr;
			b3LoadMJCFCommandInit2(commandHandle, fileName.c_str());

			if (grpcCmd.flags())
			{
				b3LoadMJCFCommandSetFlags(commandHandle, grpcCmd.flags());
			}
		}
	}

	if (grpcCommand.has_changedynamicscommand())
	{
		cmdPtr = &cmd;
		b3SharedMemoryCommandHandle commandHandle = (b3SharedMemoryCommandHandle)cmdPtr;
		const ::pybullet_grpc::ChangeDynamicsCommand& grpcCmd = grpcCommand.changedynamicscommand();
		int bodyUniqueId = grpcCmd.bodyuniqueid();
		int linkIndex = grpcCmd.linkindex();
		b3InitChangeDynamicsInfo2(commandHandle);
		if (grpcCmd.hasMass_case() == ::pybullet_grpc::ChangeDynamicsCommand::HasMassCase::kMass)
		{
			b3ChangeDynamicsInfoSetMass(commandHandle, bodyUniqueId, linkIndex, grpcCmd.mass());
		}
		if (grpcCmd.hasLateralFriction_case() == ::pybullet_grpc::ChangeDynamicsCommand::HasLateralFrictionCase::kLateralFriction)
		{
			b3ChangeDynamicsInfoSetLateralFriction(commandHandle, bodyUniqueId, linkIndex, grpcCmd.lateralfriction());
		}
		if (grpcCmd.hasSpinningFriction_case() == ::pybullet_grpc::ChangeDynamicsCommand::HasSpinningFrictionCase::kSpinningFriction)
		{
			b3ChangeDynamicsInfoSetSpinningFriction(commandHandle, bodyUniqueId, linkIndex, grpcCmd.spinningfriction());
		}
		if (grpcCmd.hasRollingFriction_case() == ::pybullet_grpc::ChangeDynamicsCommand::HasRollingFrictionCase::kRollingFriction)
		{
			b3ChangeDynamicsInfoSetRollingFriction(commandHandle, bodyUniqueId, linkIndex, grpcCmd.rollingfriction());
		}

		if (grpcCmd.hasRestitution_case() == ::pybullet_grpc::ChangeDynamicsCommand::HasRestitutionCase::kRestitution)
		{
			b3ChangeDynamicsInfoSetRestitution(commandHandle, bodyUniqueId, linkIndex, grpcCmd.restitution());
		}
		if (grpcCmd.haslinearDamping_case() == ::pybullet_grpc::ChangeDynamicsCommand::HaslinearDampingCase::kLinearDamping)
		{
			b3ChangeDynamicsInfoSetLinearDamping(commandHandle, bodyUniqueId, grpcCmd.lineardamping());
		}

		if (grpcCmd.hasangularDamping_case() == ::pybullet_grpc::ChangeDynamicsCommand::HasangularDampingCase::kAngularDamping)
		{
			b3ChangeDynamicsInfoSetAngularDamping(commandHandle, bodyUniqueId, grpcCmd.angulardamping());
		}

		bool hasContactDamping = grpcCmd.hasContactDamping_case() == ::pybullet_grpc::ChangeDynamicsCommand::HasContactDampingCase::kContactDamping;
		bool hasContactStiffness = grpcCmd.hasContactStiffness_case() == ::pybullet_grpc::ChangeDynamicsCommand::HasContactStiffnessCase::kContactStiffness;
		if (hasContactDamping && hasContactStiffness)
		{
			b3ChangeDynamicsInfoSetContactStiffnessAndDamping(commandHandle, bodyUniqueId, linkIndex, grpcCmd.contactstiffness(), grpcCmd.contactdamping());
		}
		if (grpcCmd.hasLocalInertiaDiagonal_case() == ::pybullet_grpc::ChangeDynamicsCommand::HasLocalInertiaDiagonalCase::kLocalInertiaDiagonal)
		{
			double localInertiaDiag[3] = { grpcCmd.localinertiadiagonal().x(), grpcCmd.localinertiadiagonal().y(), grpcCmd.localinertiadiagonal().z() };
			b3ChangeDynamicsInfoSetLocalInertiaDiagonal(commandHandle, bodyUniqueId, linkIndex, localInertiaDiag);
		}

		if (grpcCmd.hasFrictionAnchor_case() == ::pybullet_grpc::ChangeDynamicsCommand::HasFrictionAnchorCase::kFrictionAnchor)
		{
			b3ChangeDynamicsInfoSetFrictionAnchor(commandHandle, bodyUniqueId, linkIndex, grpcCmd.frictionanchor());
		}
		if (grpcCmd.hasccdSweptSphereRadius_case() == ::pybullet_grpc::ChangeDynamicsCommand::HasccdSweptSphereRadiusCase::kCcdSweptSphereRadius)
		{
			b3ChangeDynamicsInfoSetCcdSweptSphereRadius(commandHandle, bodyUniqueId, linkIndex, grpcCmd.ccdsweptsphereradius());
		}


		if (grpcCmd.hasContactProcessingThreshold_case() == ::pybullet_grpc::ChangeDynamicsCommand::HasContactProcessingThresholdCase::kContactProcessingThreshold)
		{
			b3ChangeDynamicsInfoSetContactProcessingThreshold(commandHandle, bodyUniqueId, linkIndex, grpcCmd.contactprocessingthreshold());
		}

		if (grpcCmd.hasActivationState_case() == ::pybullet_grpc::ChangeDynamicsCommand::HasActivationStateCase::kActivationState)
		{
			b3ChangeDynamicsInfoSetActivationState(commandHandle, bodyUniqueId, grpcCmd.activationstate());
		}
		

	}
	if (grpcCommand.has_getdynamicscommand())
	{
		cmdPtr = &cmd;
		b3SharedMemoryCommandHandle commandHandle = (b3SharedMemoryCommandHandle)cmdPtr;
		const ::pybullet_grpc::GetDynamicsCommand& grpcCmd = grpcCommand.getdynamicscommand();
		b3GetDynamicsInfoCommandInit2(commandHandle, grpcCmd.bodyuniqueid(), grpcCmd.linkindex());
	}

	if (grpcCommand.has_initposecommand())
	{
		cmdPtr = &cmd;
		b3SharedMemoryCommandHandle commandHandle = (b3SharedMemoryCommandHandle)cmdPtr;
		
		const ::pybullet_grpc::InitPoseCommand& grpcCmd = grpcCommand.initposecommand();
		b3CreatePoseCommandInit2(commandHandle, grpcCmd.bodyuniqueid());
		double initialQ[MAX_DEGREE_OF_FREEDOM] = { 0 };
		double initialQdot[MAX_DEGREE_OF_FREEDOM] = { 0 };
		int hasInitialQ[MAX_DEGREE_OF_FREEDOM] = { 0 };
		int hasInitialQdot[MAX_DEGREE_OF_FREEDOM] = { 0 };

		if (grpcCmd.initialstateq_size() == grpcCmd.hasinitialstateq_size())
		{
			int numInitial = btMin(grpcCmd.initialstateq_size(), MAX_DEGREE_OF_FREEDOM);
			for (int i = 0; i < numInitial; i++)
			{
				initialQ[i] = grpcCmd.initialstateq(i);
				hasInitialQ[i] = grpcCmd.hasinitialstateq(i);
			}
			if (numInitial)
			{
				b3CreatePoseCommandSetQ(commandHandle, numInitial, initialQ, hasInitialQ);
			}
		}
		else
		{
			printf("Error: if (grpcCmd.initialstateq_size() != grpcCmd.hasinitialstateq_size())\n");
		}
		if (grpcCmd.initialstateqdot_size() == grpcCmd.hasinitialstateqdot_size())
		{
			int numInitial = btMin(grpcCmd.initialstateqdot_size(), MAX_DEGREE_OF_FREEDOM);
			for (int i = 0; i < numInitial; i++)
			{
				initialQdot[i] = grpcCmd.initialstateqdot(i);
				hasInitialQdot[i] = grpcCmd.hasinitialstateqdot(i);
			}
			if (numInitial)
			{
				b3CreatePoseCommandSetQdots(commandHandle, numInitial, initialQdot, hasInitialQdot);
			}
		}
		else
		{
			printf("Error: (grpcCmd.initialstateqdot_size() != grpcCmd.hasinitialstateqdot_size())\n");
		}
	}

	if (grpcCommand.has_requestactualstatecommand())
	{
		cmdPtr = &cmd;
		b3SharedMemoryCommandHandle commandHandle = (b3SharedMemoryCommandHandle)cmdPtr;
		const ::pybullet_grpc::RequestActualStateCommand& grpcCmd = grpcCommand.requestactualstatecommand();
		b3RequestActualStateCommandInit2(commandHandle, grpcCmd.bodyuniqueid());
		if (grpcCmd.computeforwardkinematics())
		{
			b3RequestActualStateCommandComputeForwardKinematics(commandHandle, grpcCmd.computeforwardkinematics());
		}
		if (grpcCmd.computelinkvelocities())
		{
			b3RequestActualStateCommandComputeLinkVelocity(commandHandle, grpcCmd.computelinkvelocities());
		}
	}

	if (grpcCommand.has_stepsimulationcommand())
	{
		cmdPtr = &cmd;
		b3SharedMemoryCommandHandle commandHandle = (b3SharedMemoryCommandHandle)cmdPtr;
		b3InitStepSimulationCommand2(commandHandle);
	}

	return cmdPtr;
}

bool convertStatusToGRPC(const SharedMemoryStatus& serverStatus, char* bufferServerToClient, int bufferSizeInBytes, PyBulletStatus& grpcReply)
{
	bool converted = false;
	grpcReply.set_statustype(serverStatus.m_type);

	switch (serverStatus.m_type)
	{ 
	case CMD_URDF_LOADING_COMPLETED:
	{
		::pybullet_grpc::LoadUrdfStatus* stat = grpcReply.mutable_urdfstatus();
		b3SharedMemoryStatusHandle statusHandle = (b3SharedMemoryStatusHandle)&serverStatus;
		int objectUniqueId = b3GetStatusBodyIndex(statusHandle);
		stat->set_bodyuniqueid(objectUniqueId);
		break;
	}
	case CMD_SDF_LOADING_COMPLETED:
	{
		int bodyIndicesOut[MAX_SDF_BODIES];
		::pybullet_grpc::SdfLoadedStatus* stat = grpcReply.mutable_sdfstatus();
		b3SharedMemoryStatusHandle statusHandle = (b3SharedMemoryStatusHandle)&serverStatus;
		int numBodies = b3GetStatusBodyIndices(statusHandle, bodyIndicesOut, MAX_SDF_BODIES);
		if (numBodies > MAX_SDF_BODIES)
		{
			printf("SDF exceeds body capacity: %d > %d", numBodies, MAX_SDF_BODIES);
		}
		for (int i = 0; i < numBodies; i++)
		{
			stat->add_bodyuniqueids(bodyIndicesOut[i]);
		}
		break;
	}
	case CMD_MJCF_LOADING_COMPLETED:
	{
		int bodyIndicesOut[MAX_SDF_BODIES];
		::pybullet_grpc::MjcfLoadedStatus* stat = grpcReply.mutable_mjcfstatus();
		b3SharedMemoryStatusHandle statusHandle = (b3SharedMemoryStatusHandle)&serverStatus;
		int numBodies = b3GetStatusBodyIndices(statusHandle, bodyIndicesOut, MAX_SDF_BODIES);
		if (numBodies > MAX_SDF_BODIES)
		{
			printf("MJCF exceeds body capacity: %d > %d", numBodies, MAX_SDF_BODIES);
		}
		for (int i = 0; i < numBodies; i++)
		{
			stat->add_bodyuniqueids(bodyIndicesOut[i]);
		}
		break;
	}
	case CMD_GET_DYNAMICS_INFO_COMPLETED:
	{
		b3DynamicsInfo info;
		b3SharedMemoryStatusHandle statusHandle = (b3SharedMemoryStatusHandle)&serverStatus;
		if (b3GetDynamicsInfo(statusHandle, &info))
		{
			::pybullet_grpc::GetDynamicsStatus*stat = grpcReply.mutable_getdynamicsstatus();
			
			stat->set_mass(info.m_mass);
			stat->set_lateralfriction(info.m_lateralFrictionCoeff);
			stat->set_spinningfriction(info.m_spinningFrictionCoeff);
			stat->set_rollingfriction(info.m_rollingFrictionCoeff);
			stat->set_restitution(info.m_restitution);
			stat->set_lineardamping(info.m_linearDamping);
			stat->set_angulardamping(info.m_angularDamping);
			
			stat->set_contactstiffness(info.m_contactStiffness);
			stat->set_contactdamping(info.m_contactDamping);
			
			pybullet_grpc::vec3* localInertia = stat->mutable_localinertiadiagonal();
			localInertia->set_x(info.m_localInertialDiagonal[0]);
			localInertia->set_y(info.m_localInertialDiagonal[1]);
			localInertia->set_z(info.m_localInertialDiagonal[2]);

			stat->set_frictionanchor(info.m_frictionAnchor);
			stat->set_ccdsweptsphereradius(info.m_ccdSweptSphereRadius);
			stat->set_contactprocessingthreshold(info.m_contactProcessingThreshold);
			

			stat->set_activationstate(info.m_activationState);
		}
		break;
	}
	case CMD_ACTUAL_STATE_UPDATE_COMPLETED:
	{
		b3SharedMemoryStatusHandle statusHandle = (b3SharedMemoryStatusHandle)&serverStatus;
		int bodyUniqueId;
		int numLinks;

		int numDegreeOfFreedomQ;
		int numDegreeOfFreedomU;
		
		const double* rootLocalInertialFramePtr=0;
		const double* actualStateQptr=0;
		const double* actualStateQdotPtr=0;
		const double* jointReactionForcesPtr = 0;

		const double* linkLocalInertialFrames = 0;
		const double* jointMotorForces = 0;
		const double* linkStates = 0;
		const double* linkWorldVelocities = 0;


		if (b3GetStatusActualState2(
			statusHandle, &bodyUniqueId,&numLinks,
			&numDegreeOfFreedomQ,
			&numDegreeOfFreedomU,
			&rootLocalInertialFramePtr,
			&actualStateQptr,
			&actualStateQdotPtr,
			&jointReactionForcesPtr,
			&linkLocalInertialFrames,
			&jointMotorForces,
			&linkStates,
			&linkWorldVelocities))
		{
			::pybullet_grpc::SendActualStateStatus* stat = grpcReply.mutable_actualstatestatus();
			stat->set_bodyuniqueid(bodyUniqueId);
			stat->set_numlinks(numLinks);

			stat->set_numdegreeoffreedomq(numDegreeOfFreedomQ);
			stat->set_numdegreeoffreedomu(numDegreeOfFreedomU);
			for (int i = 0; i < numDegreeOfFreedomQ; i++)
			{
				stat->add_actualstateq( actualStateQptr[i]);
			}
			for (int i = 0; i < numDegreeOfFreedomU; i++)
			{
				stat->add_actualstateqdot( actualStateQdotPtr[i]);
			}
			for (int i = 0; i < 7; i++)
			{
				stat->add_rootlocalinertialframe(rootLocalInertialFramePtr[i]);
			}
			for (int i = 0; i < numLinks * 6; i++)
			{
				stat->add_linklocalinertialframes( linkLocalInertialFrames[i]);
			}
			for (int i = 0; i < numLinks * 6; i++)
			{
				stat->add_jointreactionforces(jointReactionForcesPtr[i]);
			}
			for (int i = 0; i < numLinks; i++)
			{
				stat->add_jointmotorforce(jointMotorForces[i]);
			}
			for (int i = 0; i < numLinks * 7; i++)
			{
				stat->add_linkstate(linkStates[i]);
			}
			for (int i = 0; i < numLinks * 6; i++)
			{
				stat->add_linkworldvelocities(linkWorldVelocities[i]);
			}
		}
		break;
	}
	case CMD_CLIENT_COMMAND_COMPLETED:
	{
		//no action needed?
		break;
	}
	default:
	{
		printf("convertStatusToGRPC: unknown status");
	}
	
	}
	
	return converted;
}
