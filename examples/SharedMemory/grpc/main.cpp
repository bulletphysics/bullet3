///PyBullet / BulletRobotics GRPC server.
///works as standalone GRPC server as as a GRPC server bridge,
///connecting to a local physics server using shared memory

#include <stdio.h>
#include "../../CommonInterfaces/CommonGUIHelperInterface.h"
#include "Bullet3Common/b3CommandLineArgs.h"
#include "PhysicsClientC_API.h"
#ifdef NO_SHARED_MEMORY
#include "PhysicsServerCommandProcessor.h"
typedef PhysicsServerCommandProcessor MyCommandProcessor;
#else
#include "SharedMemoryCommandProcessor.h"
typedef SharedMemoryCommandProcessor MyCommandProcessor;
#endif  //NO_SHARED_MEMORY

#include "SharedMemoryCommands.h"
#include "Bullet3Common/b3AlignedObjectArray.h"
#include "PhysicsServerCommandProcessor.h"
#include "../Utils/b3Clock.h"

#include <memory>
#include <iostream>
#include <string>
#include <thread>

#include <grpc++/grpc++.h>
#include <grpc/support/log.h>

#include "SharedMemory/grpc/proto/pybullet.grpc.pb.h"

using grpc::Server;
using grpc::ServerAsyncResponseWriter;
using grpc::ServerBuilder;
using grpc::ServerCompletionQueue;
using grpc::ServerContext;
using grpc::Status;
using pybullet_grpc::PyBulletAPI;
using pybullet_grpc::PyBulletCommand;
using pybullet_grpc::PyBulletStatus;

bool gVerboseNetworkMessagesServer = true;
#include "ConvertGRPCBullet.h"

class ServerImpl final
{
public:
	~ServerImpl()
	{
		server_->Shutdown();
		// Always shutdown the completion queue after the server.
		cq_->Shutdown();
	}

	void Run(MyCommandProcessor* comProc, const std::string& hostNamePort)
	{
		ServerBuilder builder;
		// Listen on the given address without any authentication mechanism.
		builder.AddListeningPort(hostNamePort, grpc::InsecureServerCredentials());
		// Register "service_" as the instance through which we'll communicate with
		// clients. In this case it corresponds to an *asynchronous* service.
		builder.RegisterService(&service_);
		// Get hold of the completion queue used for the asynchronous communication
		// with the gRPC runtime.
		cq_ = builder.AddCompletionQueue();
		// Finally assemble the server.
		server_ = builder.BuildAndStart();
		std::cout << "Standalone Bullet Physics GRPC server listening on " << hostNamePort << std::endl;

		// Proceed to the server's main loop.
		HandleRpcs(comProc);
	}

private:
	// Class encompasing the state and logic needed to serve a request.
	class CallData
	{
	public:
		// Take in the "service" instance (in this case representing an asynchronous
		// server) and the completion queue "cq" used for asynchronous communication
		// with the gRPC runtime.
		CallData(PyBulletAPI::AsyncService* service, ServerCompletionQueue* cq, MyCommandProcessor* comProc)
			: service_(service), cq_(cq), responder_(&ctx_), status_(CREATE), m_finished(false), m_comProc(comProc)
		{
			// Invoke the serving logic right away.
			Proceed();
		}

		enum CallStatus
		{
			CREATE,
			PROCESS,
			FINISH,
			TERMINATE
		};

		CallStatus Proceed()
		{
			if (status_ == CREATE)
			{
				// Make this instance progress to the PROCESS state.
				status_ = PROCESS;

				// As part of the initial CREATE state, we *request* that the system
				// start processing SayHello requests. In this request, "this" acts are
				// the tag uniquely identifying the request (so that different CallData
				// instances can serve different requests concurrently), in this case
				// the memory address of this CallData instance.

				service_->RequestSubmitCommand(&ctx_, &m_command, &responder_, cq_, cq_,
											   this);
			}
			else if (status_ == PROCESS)
			{
				// Spawn a new CallData instance to serve new clients while we process
				// the one for this CallData. The instance will deallocate itself as
				// part of its FINISH state.
				new CallData(service_, cq_, m_comProc);
				status_ = FINISH;

				std::string replyString;
				// The actual processing.

				SharedMemoryStatus serverStatus;
				b3AlignedObjectArray<char> buffer;
				buffer.resize(SHARED_MEMORY_MAX_STREAM_CHUNK_SIZE);
				SharedMemoryCommand cmd;
				SharedMemoryCommand* cmdPtr = 0;

				m_status.set_statustype(CMD_UNKNOWN_COMMAND_FLUSHED);

				if (m_command.has_checkversioncommand())
				{
					m_status.set_statustype(CMD_CLIENT_COMMAND_COMPLETED);
					m_status.mutable_checkversionstatus()->set_serverversion(SHARED_MEMORY_MAGIC_NUMBER);
				}
				else
				{
					cmdPtr = convertGRPCToBulletCommand(m_command, cmd);

					if (cmdPtr)
					{
						bool hasStatus = m_comProc->processCommand(*cmdPtr, serverStatus, &buffer[0], buffer.size());
						m_comProc->reportNotifications();
						double timeOutInSeconds = 10;
						b3Clock clock;
						double startTimeSeconds = clock.getTimeInSeconds();
						double curTimeSeconds = clock.getTimeInSeconds();

						while ((!hasStatus) && ((curTimeSeconds - startTimeSeconds) < timeOutInSeconds))
						{
							hasStatus = m_comProc->receiveStatus(serverStatus, &buffer[0], buffer.size());
							curTimeSeconds = clock.getTimeInSeconds();
						}
						if (gVerboseNetworkMessagesServer)
						{
							//printf("buffer.size = %d\n", buffer.size());
							printf("serverStatus.m_numDataStreamBytes = %d\n", serverStatus.m_numDataStreamBytes);
						}
						if (hasStatus)
						{
							b3AlignedObjectArray<unsigned char> packetData;
							unsigned char* statBytes = (unsigned char*)&serverStatus;

							convertStatusToGRPC(serverStatus, &buffer[0], buffer.size(), m_status);
						}
					}

					if (m_command.has_terminateservercommand())
					{
						status_ = TERMINATE;
					}
				}

				// And we are done! Let the gRPC runtime know we've finished, using the
				// memory address of this instance as the uniquely identifying tag for
				// the event.

				responder_.Finish(m_status, Status::OK, this);
			}
			else
			{
				GPR_ASSERT(status_ == FINISH);
				// Once in the FINISH state, deallocate ourselves (CallData).
				delete this;
			}
			return status_;
		}

	private:
		// The means of communication with the gRPC runtime for an asynchronous
		// server.
		PyBulletAPI::AsyncService* service_;
		// The producer-consumer queue where for asynchronous server notifications.
		ServerCompletionQueue* cq_;
		// Context for the rpc, allowing to tweak aspects of it such as the use
		// of compression, authentication, as well as to send metadata back to the
		// client.
		ServerContext ctx_;

		// What we get from the client.
		PyBulletCommand m_command;
		// What we send back to the client.
		PyBulletStatus m_status;

		// The means to get back to the client.
		ServerAsyncResponseWriter<PyBulletStatus> responder_;

		// Let's implement a tiny state machine with the following states.

		CallStatus status_;  // The current serving state.

		bool m_finished;

		MyCommandProcessor* m_comProc;  //physics server command processor
	};

	// This can be run in multiple threads if needed.
	void HandleRpcs(MyCommandProcessor* comProc)
	{
		// Spawn a new CallData instance to serve new clients.
		new CallData(&service_, cq_.get(), comProc);
		void* tag;  // uniquely identifies a request.
		bool ok;
		bool finished = false;

		CallData::CallStatus status = CallData::CallStatus::CREATE;

		while (status != CallData::CallStatus::TERMINATE)
		{
			// Block waiting to read the next event from the completion queue. The
			// event is uniquely identified by its tag, which in this case is the
			// memory address of a CallData instance.
			// The return value of Next should always be checked. This return value
			// tells us whether there is any kind of event or cq_ is shutting down.

			grpc::CompletionQueue::NextStatus nextStatus = cq_->AsyncNext(&tag, &ok, gpr_now(GPR_CLOCK_MONOTONIC));
			if (nextStatus == grpc::CompletionQueue::NextStatus::GOT_EVENT)
			{
				//GPR_ASSERT(cq_->Next(&tag, &ok));
				GPR_ASSERT(ok);
				status = static_cast<CallData*>(tag)->Proceed();
			}
		}
	}

	std::unique_ptr<ServerCompletionQueue> cq_;
	PyBulletAPI::AsyncService service_;
	std::unique_ptr<Server> server_;
};

int main(int argc, char** argv)
{
	b3CommandLineArgs parseArgs(argc, argv);
	b3Clock clock;
	double timeOutInSeconds = 10;

	DummyGUIHelper guiHelper;
	MyCommandProcessor* sm = new MyCommandProcessor;
	sm->setGuiHelper(&guiHelper);

	int port = 6667;
	parseArgs.GetCmdLineArgument("port", port);
	std::string hostName = "localhost";
	std::string hostNamePort = hostName;
	if (port >= 0)
	{
		hostNamePort += ":" + std::to_string(port);
	}

	gVerboseNetworkMessagesServer = parseArgs.CheckCmdLineFlag("verbose");

#ifndef NO_SHARED_MEMORY
	int key = 0;
	if (parseArgs.GetCmdLineArgument("sharedMemoryKey", key))
	{
		sm->setSharedMemoryKey(key);
	}
#endif  //NO_SHARED_MEMORY

	bool isPhysicsClientConnected = sm->connect();
	bool exitRequested = false;

	if (isPhysicsClientConnected)
	{
		ServerImpl server;

		server.Run(sm, hostNamePort);
	}
	else
	{
		printf("Couldn't connect to physics server\n");
	}

	delete sm;

	return 0;
}
