
///grpcPlugin add a GRPC server to any PyBullet/BulletRobotics
///physics server. You can connect using PyBullet connect.GRPC method

#include "grpcPlugin.h"
#include "SharedMemory/SharedMemoryPublic.h"
#include "../b3PluginContext.h"
#include "Bullet3Common/b3AlignedObjectArray.h"
#include "SharedMemory/SharedMemoryCommands.h"
#include "SharedMemory/PhysicsCommandProcessorInterface.h"

#include <stdio.h>

#include <mutex>
#include <thread>

#include <grpc++/grpc++.h>
#include <grpc/support/log.h>
#include "../../../Utils/b3Clock.h"
#include "SharedMemory/grpc/proto/pybullet.grpc.pb.h"
#include "SharedMemory/grpc/ConvertGRPCBullet.h"
using grpc::Server;
using grpc::ServerAsyncResponseWriter;
using grpc::ServerBuilder;
using grpc::ServerCompletionQueue;
using grpc::ServerContext;
using grpc::Status;
using pybullet_grpc::PyBulletAPI;
using pybullet_grpc::PyBulletCommand;
using pybullet_grpc::PyBulletStatus;

bool gVerboseNetworkMessagesServer4 = false;

class ServerImpl final
{
public:
	ServerImpl()
	{
	}
	~ServerImpl()
	{
		Exit();
	}

	void Exit()
	{
		if (server_)
		{
			server_->Shutdown();
			m_requestThreadCancelled = true;
			m_requestThread->join();
			delete m_requestThread;
			// Always shutdown the completion queue after the server.
			cq_->Shutdown();
			server_ = 0;
		}
	}

	void Init(PhysicsCommandProcessorInterface* comProc, const std::string& hostNamePort)
	{
		// Listen on the given address without any authentication mechanism.
		m_builder.AddListeningPort(hostNamePort, grpc::InsecureServerCredentials());
		// Register "service_" as the instance through which we'll communicate with
		// clients. In this case it corresponds to an *asynchronous* service.
		m_builder.RegisterService(&service_);
		// Get hold of the completion queue used for the asynchronous communication
		// with the gRPC runtime.
		cq_ = m_builder.AddCompletionQueue();
		// Finally assemble the server.
		server_ = m_builder.BuildAndStart();
		std::cout << "grpcPlugin Bullet Physics GRPC server listening on " << hostNamePort << std::endl;

		//Start the thread to gather the requests.
		m_requestThreadCancelled = false;
		m_requestThread = new std::thread(&ServerImpl::GatherRequests, this);

		// Proceed to the server's main loop.
		InitRpcs(comProc);
	}

	// This can be run in multiple threads if needed.
	bool HandleSingleRpc()
	{
		CallData::CallStatus status = CallData::CallStatus::CREATE;
		std::lock_guard<std::mutex> guard(m_queueMutex);
    if (!m_requestQueue.empty()) {
      void* tag = m_requestQueue.front();
      m_requestQueue.pop_front();
      status = static_cast<CallData*>(tag)->Proceed();
    }

		return status == CallData::CallStatus::TERMINATE;
	}

private:
	// Class encompasing the state and logic needed to serve a request.
	class CallData
	{
	public:
		// Take in the "service" instance (in this case representing an asynchronous
		// server) and the completion queue "cq" used for asynchronous communication
		// with the gRPC runtime.
		CallData(PyBulletAPI::AsyncService* service, ServerCompletionQueue* cq, PhysicsCommandProcessorInterface* comProc)
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

						double timeOutInSeconds = 10;
						b3Clock clock;
						double startTimeSeconds = clock.getTimeInSeconds();
						double curTimeSeconds = clock.getTimeInSeconds();

						while ((!hasStatus) && ((curTimeSeconds - startTimeSeconds) < timeOutInSeconds))
						{
							hasStatus = m_comProc->receiveStatus(serverStatus, &buffer[0], buffer.size());
							curTimeSeconds = clock.getTimeInSeconds();
						}

						if (gVerboseNetworkMessagesServer4)
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
				CallData::CallStatus tmpStatus = status_;
				delete this;
				return tmpStatus;
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

		PhysicsCommandProcessorInterface* m_comProc;  //physics server command processor
	};

	// This can be run in multiple threads if needed.
	void InitRpcs(PhysicsCommandProcessorInterface* comProc)
	{
		// Spawn a new CallData instance to serve new clients.
		new CallData(&service_, cq_.get(), comProc);
	}

	ServerBuilder m_builder;
	std::unique_ptr<ServerCompletionQueue> cq_;
	PyBulletAPI::AsyncService service_;
	std::unique_ptr<Server> server_;

	// Mutex to protect access to the request queue variables (m_requestQueue,
	// m_requestThread, m_requestThreadCancelled).
	std::mutex m_queueMutex;

	// List of outstanding request tags.
	std::list<void*> m_requestQueue;

	// Whether or not the gathering thread is cancelled.
	bool m_requestThreadCancelled;

	// Thread to gather requests from the completion queue.
	std::thread* m_requestThread;

	void GatherRequests() {
		void* tag;  // uniquely identifies a request.
		bool ok;

		while(!m_requestThreadCancelled) {
			// Block waiting to read the next event from the completion queue. The
			// event is uniquely identified by its tag, which in this case is the
			// memory address of a CallData instance.
			// The return value of Next should always be checked. This return value
			// tells us whether there is any kind of event or cq_ is shutting down.
			grpc::CompletionQueue::NextStatus nextStatus = cq_->AsyncNext(&tag, &ok, gpr_now(GPR_CLOCK_MONOTONIC));
			if (nextStatus == grpc::CompletionQueue::NextStatus::GOT_EVENT)
			{
				GPR_ASSERT(ok);
				std::lock_guard<std::mutex> guard(m_queueMutex);
				m_requestQueue.push_back(tag);
			}
		}
	}
};

struct grpcMyClass
{
	int m_testData;

	ServerImpl m_grpcServer;
	bool m_grpcInitialized;
	bool m_grpcTerminated;

	grpcMyClass()
		: m_testData(42),
		  m_grpcInitialized(false),
		  m_grpcTerminated(false)
	{
	}
	virtual ~grpcMyClass()
	{
	}
};

B3_SHARED_API int initPlugin_grpcPlugin(struct b3PluginContext* context)
{
	grpcMyClass* obj = new grpcMyClass();
	context->m_userPointer = obj;

	return SHARED_MEMORY_MAGIC_NUMBER;
}

B3_SHARED_API int preTickPluginCallback_grpcPlugin(struct b3PluginContext* context)
{
	//process grpc server messages
	return 0;
}

B3_SHARED_API int processClientCommands_grpcPlugin(struct b3PluginContext* context)
{
	grpcMyClass* obj = (grpcMyClass*)context->m_userPointer;

	if (obj->m_grpcInitialized && !obj->m_grpcTerminated)
	{
		obj->m_grpcTerminated = obj->m_grpcServer.HandleSingleRpc();
	}

	obj->m_testData++;
	return 0;
}

B3_SHARED_API int postTickPluginCallback_grpcPlugin(struct b3PluginContext* context)
{
	grpcMyClass* obj = (grpcMyClass*)context->m_userPointer;
	obj->m_testData++;
	return 0;
}

B3_SHARED_API int executePluginCommand_grpcPlugin(struct b3PluginContext* context, const struct b3PluginArguments* arguments)
{
	///3 cases:
	/// 1: send a non-empty string to start the GRPC server
	/// 2: send some integer n, to call n times to HandleSingleRpc
	/// 3: send nothing to terminate the GRPC server

	grpcMyClass* obj = (grpcMyClass*)context->m_userPointer;

	if (strlen(arguments->m_text))
	{
		if (!obj->m_grpcInitialized && context->m_rpcCommandProcessorInterface)
		{
			obj->m_grpcServer.Init(context->m_rpcCommandProcessorInterface, arguments->m_text);
		}
		obj->m_grpcInitialized = true;
	}
	else
	{
		if (arguments->m_numInts > 0)
		{
			for (int i = 0; i < arguments->m_ints[0]; i++)
			{
				if (obj->m_grpcInitialized && !obj->m_grpcTerminated)
				{
					obj->m_grpcTerminated = obj->m_grpcServer.HandleSingleRpc();
				}
			}
		}
		else
		{
			obj->m_grpcServer.Exit();
			obj->m_grpcInitialized = false;
		}
	}

	return 0;
}

B3_SHARED_API void exitPlugin_grpcPlugin(struct b3PluginContext* context)
{
	grpcMyClass* obj = (grpcMyClass*)context->m_userPointer;
	obj->m_grpcServer.Exit();
	delete obj;
	context->m_userPointer = 0;
}
