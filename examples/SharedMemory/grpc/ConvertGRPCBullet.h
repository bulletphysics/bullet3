
#ifndef BT_CONVERT_GRPC_BULLET_H
#define BT_CONVERT_GRPC_BULLET_H

#include "../PhysicsClientC_API.h"

namespace pybullet_grpc
{
	class PyBulletCommand;
	class PyBulletStatus;
};

struct SharedMemoryCommand* convertGRPCAndSubmitCommand(pybullet_grpc::PyBulletCommand& grpcCommand, struct SharedMemoryCommand& cmd);

bool convertStatusToGRPC(const struct SharedMemoryStatus& serverStatus, char* bufferServerToClient, int bufferSizeInBytes, pybullet_grpc::PyBulletStatus& grpcReply);

#endif //BT_CONVERT_GRPC_BULLET_H