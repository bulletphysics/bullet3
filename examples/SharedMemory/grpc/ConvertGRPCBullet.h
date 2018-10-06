
#ifndef BT_CONVERT_GRPC_BULLET_H
#define BT_CONVERT_GRPC_BULLET_H

#include "../PhysicsClientC_API.h"

namespace pybullet_grpc
{
class PyBulletCommand;
class PyBulletStatus;
};  // namespace pybullet_grpc

struct SharedMemoryCommand* convertGRPCToBulletCommand(const pybullet_grpc::PyBulletCommand& grpcCommand, struct SharedMemoryCommand& cmd);

pybullet_grpc::PyBulletCommand* convertBulletToGRPCCommand(const struct SharedMemoryCommand& clientCmd, pybullet_grpc::PyBulletCommand& grpcCommand);

bool convertGRPCToStatus(const pybullet_grpc::PyBulletStatus& grpcReply, struct SharedMemoryStatus& serverStatus, char* bufferServerToClient, int bufferSizeInBytes);

bool convertStatusToGRPC(const struct SharedMemoryStatus& serverStatus, char* bufferServerToClient, int bufferSizeInBytes, pybullet_grpc::PyBulletStatus& grpcReply);

#endif  //BT_CONVERT_GRPC_BULLET_H