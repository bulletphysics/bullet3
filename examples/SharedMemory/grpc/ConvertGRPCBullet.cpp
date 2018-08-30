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
		auto grpcCmd = grpcCommand.loadurdfcommand();

		std::string fileName = grpcCmd.urdffilename();
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
			if (grpcCmd.hasUseMultiBody_case()== ::pybullet_grpc::LoadUrdfCommand::HasUseMultiBodyCase::kUseMultiBody)
			{
				b3LoadUrdfCommandSetUseMultiBody( commandHandle, grpcCmd.usemultibody());
			}
			if (grpcCmd.hasGlobalScaling_case() == ::pybullet_grpc::LoadUrdfCommand::HasGlobalScalingCase::kGlobalScaling)
			{
				b3LoadUrdfCommandSetGlobalScaling(commandHandle, grpcCmd.globalscaling());
			}
			if (grpcCmd.hasUseFixedBase_case() == ::pybullet_grpc::LoadUrdfCommand::HasUseFixedBaseCase::kUseFixedBase)
			{
				b3LoadUrdfCommandSetUseFixedBase(commandHandle, grpcCmd.usefixedbase());
			}
			if (grpcCmd.urdfflags())
			{
				b3LoadUrdfCommandSetFlags(commandHandle, grpcCmd.urdfflags());
			}

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
		stat->set_objectuniqueid(objectUniqueId);
	}

	break;
	
	}
	
	return converted;
}
