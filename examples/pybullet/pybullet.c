//#include "D:/dev/visual leak detector/include/vld.h"

#include "../SharedMemory/PhysicsClientC_API.h"
#include "../SharedMemory/PhysicsDirectC_API.h"
#include "../SharedMemory/SharedMemoryInProcessPhysicsC_API.h"
#ifdef BT_ENABLE_ENET
#include "../SharedMemory/PhysicsClientUDP_C_API.h"
#endif  //BT_ENABLE_ENET
#define PYBULLET_PI (3.1415926535897932384626433832795029)

#ifdef BT_ENABLE_DART
#include "../SharedMemory/dart/DARTPhysicsC_API.h"
#endif

#ifdef BT_ENABLE_PHYSX
#include "../SharedMemory/physx/PhysXC_API.h"
#endif

#ifdef BT_ENABLE_MUJOCO
#include "../SharedMemory/mujoco/MuJoCoPhysicsC_API.h"
#endif

#ifdef BT_ENABLE_GRPC
#include "../SharedMemory/PhysicsClientGRPC_C_API.h"
#endif
#ifdef BT_ENABLE_CLSOCKET
#include "../SharedMemory/PhysicsClientTCP_C_API.h"
#endif  //BT_ENABLE_CLSOCKET

#if defined(__APPLE__) && (!defined(B3_NO_PYTHON_FRAMEWORK))
#include <Python/Python.h>
#else
#ifdef _WIN32
	#ifdef _DEBUG
		#define BT_REMOVED_DEBUG
		//always use the release build of Python
		#undef _DEBUG
	#endif //_DEBUG
#endif
#include <Python.h>
#endif
#ifdef BT_REMOVED_DEBUG
	#define _DEBUG
#endif
#include "../Importers/ImportURDFDemo/urdfStringSplit.h"

#ifdef B3_DUMP_PYTHON_VERSION
#define B3_VALUE_TO_STRING(x) #x
#define B3_VALUE(x) B3_VALUE_TO_STRING(x)
#define B3_VAR_NAME_VALUE(var) #var "=" B3_VALUE(var)
#pragma message(B3_VAR_NAME_VALUE(PY_MAJOR_VERSION))
#pragma message(B3_VAR_NAME_VALUE(PY_MINOR_VERSION))
#endif

#ifdef PYBULLET_USE_NUMPY
#include <numpy/arrayobject.h>
#endif

#if PY_MAJOR_VERSION >= 3
#define PyInt_FromLong PyLong_FromLong
#define PyString_FromString PyBytes_FromString
#endif

static PyObject* SpamError;
#define B3_MAX_NUM_END_EFFECTORS 128
#define MAX_PHYSICS_CLIENTS 1024
static b3PhysicsClientHandle sPhysicsClients1[MAX_PHYSICS_CLIENTS] = {0};
static int sPhysicsClientsGUI[MAX_PHYSICS_CLIENTS] = {0};
static int sNumPhysicsClients = 0;

b3PhysicsClientHandle getPhysicsClient(int physicsClientId)
{
	b3PhysicsClientHandle sm;
	if ((physicsClientId < 0) || (physicsClientId >= MAX_PHYSICS_CLIENTS) || (0 == sPhysicsClients1[physicsClientId]))
	{
		return 0;
	}
	sm = sPhysicsClients1[physicsClientId];
	if (sm)
	{
		if (b3CanSubmitCommand(sm))
		{
			return sm;
		}
		//broken connection?
		b3DisconnectSharedMemory(sm);
		sPhysicsClients1[physicsClientId] = 0;
		sPhysicsClientsGUI[physicsClientId] = 0;

		sNumPhysicsClients--;
	}
	return 0;
}

static double pybullet_internalGetFloatFromSequence(PyObject* seq, int index)
{
	double v = 0.0;
	PyObject* item;

	if (PyList_Check(seq))
	{
		item = PyList_GET_ITEM(seq, index);
		v = PyFloat_AsDouble(item);
	}
	else
	{
		item = PyTuple_GET_ITEM(seq, index);
		v = PyFloat_AsDouble(item);
	}
	return v;
}

static int pybullet_internalGetIntFromSequence(PyObject* seq, int index)
{
	int v = 0;
	PyObject* item;

	if (PyList_Check(seq))
	{
		item = PyList_GET_ITEM(seq, index);
		v = PyLong_AsLong(item);
	}
	else
	{
		item = PyTuple_GET_ITEM(seq, index);
		v = PyLong_AsLong(item);
	}
	return v;
}

static const char* pybullet_internalGetCStringFromSequence(PyObject* seq, int index)
{
	const char* v = 0;
	PyObject* item;

	if (PyList_Check(seq))
	{
		item = PyList_GET_ITEM(seq, index);
		v = PyUnicode_AsUTF8(item);
	}
	else
	{
		item = PyTuple_GET_ITEM(seq, index);
		v = PyUnicode_AsUTF8(item);
	}
	return v;
}

// internal function to set a float matrix[16]
// used to initialize camera position with
// a view and projection matrix in renderImage()
//
// // Args:
//  matrix - float[16] which will be set by values from objMat
static int pybullet_internalSetMatrix(PyObject* objMat, float matrix[16])
{
	int i, len;
	PyObject* seq;

	if (objMat == NULL)
		return 0;

	seq = PySequence_Fast(objMat, "expected a sequence");
	if (seq)
	{
		len = PySequence_Size(objMat);
		if (len == 16)
		{
			for (i = 0; i < len; i++)
			{
				matrix[i] = pybullet_internalGetFloatFromSequence(seq, i);
			}
			Py_DECREF(seq);
			return 1;
		}
		Py_DECREF(seq);
	}
	PyErr_Clear();
	return 0;
}

// internal function to set a float vector[3]
// used to initialize camera position with
// a view and projection matrix in renderImage()
//
// // Args:
//  vector - float[3] which will be set by values from objMat
static int pybullet_internalSetVector(PyObject* objVec, float vector[3])
{
	int i, len;
	PyObject* seq = 0;

	if (objVec == NULL)
		return 0;

	seq = PySequence_Fast(objVec, "expected a sequence");
	if (seq)
	{
		len = PySequence_Size(objVec);
		assert(len == 3);
		if (len == 3)
		{
			for (i = 0; i < len; i++)
			{
				vector[i] = pybullet_internalGetFloatFromSequence(seq, i);
			}
			Py_DECREF(seq);
			return 1;
		}
		Py_DECREF(seq);
	}
	PyErr_Clear();
	return 0;
}

//  vector - double[2] which will be set by values from obVec
static int pybullet_internalSetVector2d(PyObject* obVec, double vector[2])
{
	int i, len;
	PyObject* seq;
	if (obVec == NULL)
		return 0;

	seq = PySequence_Fast(obVec, "expected a sequence");
	if (seq)
	{
		len = PySequence_Size(obVec);
		assert(len == 2);
		if (len == 2)
		{
			for (i = 0; i < len; i++)
			{
				vector[i] = pybullet_internalGetFloatFromSequence(seq, i);
			}
			Py_DECREF(seq);
			return 1;
		}
		Py_DECREF(seq);
	}
	PyErr_Clear();
	return 0;
}

//  vector - double[3] which will be set by values from obVec
static int pybullet_internalSetVectord(PyObject* obVec, double vector[3])
{
	int i, len;
	PyObject* seq;
	if (obVec == NULL)
		return 0;

	seq = PySequence_Fast(obVec, "expected a sequence");
	if (seq)
	{
		len = PySequence_Size(obVec);
		assert(len == 3);
		if (len == 3)
		{
			for (i = 0; i < len; i++)
			{
				vector[i] = pybullet_internalGetFloatFromSequence(seq, i);
			}
			Py_DECREF(seq);
			return 1;
		}
		Py_DECREF(seq);
	}
	PyErr_Clear();
	return 0;
}

//  vector - double[4] which will be set by values from obVec
static int pybullet_internalSetVector4d(PyObject* obVec, double vector[4])
{
	int i, len;
	PyObject* seq;
	if (obVec == NULL)
		return 0;

	seq = PySequence_Fast(obVec, "expected a sequence");
	if (seq)
	{
		len = PySequence_Size(obVec);
		if (len == 4)
		{
			for (i = 0; i < len; i++)
			{
				vector[i] = pybullet_internalGetFloatFromSequence(seq, i);
			}
			Py_DECREF(seq);
			return 1;
		}
		Py_DECREF(seq);
	}
	PyErr_Clear();
	return 0;
}

static int pybullet_internalGetVector3FromSequence(PyObject* seq, int index, double vec[3])
{
	int v = 0;
	PyObject* item;

	if (PyList_Check(seq))
	{
		item = PyList_GET_ITEM(seq, index);
		pybullet_internalSetVectord(item, vec);
	}
	else
	{
		item = PyTuple_GET_ITEM(seq, index);
		pybullet_internalSetVectord(item, vec);
	}
	return v;
}

static int pybullet_internalGetVector4FromSequence(PyObject* seq, int index, double vec[4])
{
	int v = 0;
	PyObject* item;

	if (PyList_Check(seq))
	{
		item = PyList_GET_ITEM(seq, index);
		pybullet_internalSetVector4d(item, vec);
	}
	else
	{
		item = PyTuple_GET_ITEM(seq, index);
		pybullet_internalSetVector4d(item, vec);
	}
	return v;
}

// Step through one timestep of the simulation
static PyObject* pybullet_stepSimulation(PyObject* self, PyObject* args, PyObject* keywds)
{
	int physicsClientId = 0;
	static char* kwlist[] = {"physicsClientId", NULL};
	b3PhysicsClientHandle sm = 0;

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|i", kwlist, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	{
		b3SharedMemoryStatusHandle statusHandle;
		int statusType;

		if (b3CanSubmitCommand(sm))
		{
			statusHandle = b3SubmitClientCommandAndWaitStatus(
				sm, b3InitStepSimulationCommand(sm));
			statusType = b3GetStatusType(statusHandle);

			if (statusType == CMD_STEP_FORWARD_SIMULATION_COMPLETED)
			{
				struct b3ForwardDynamicsAnalyticsArgs analyticsData;
				int numIslands = 0;
				int i;
				PyObject* val = 0;
				PyObject* pyAnalyticsData;
                                
				numIslands = b3GetStatusForwardDynamicsAnalyticsData(statusHandle, &analyticsData);
				pyAnalyticsData = PyTuple_New(numIslands);
                                
				for (i=0;i<numIslands;i++)
				{
					val = Py_BuildValue("{s:i, s:i, s:i, s:d}",
					"islandId", analyticsData.m_islandData[i].m_islandId,
					"numBodies", analyticsData.m_islandData[i].m_numBodies,
					"numIterationsUsed", analyticsData.m_islandData[i].m_numIterationsUsed,
					"remainingResidual", analyticsData.m_islandData[i].m_remainingLeastSquaresResidual);
					PyTuple_SetItem(pyAnalyticsData, i, val);
				}

				return pyAnalyticsData;
			}
		}
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_connectPhysicsServer(PyObject* self, PyObject* args, PyObject* keywds)
{
	int freeIndex = -1;
	int method = eCONNECT_GUI;
	int i;
	char* options = 0;

	b3PhysicsClientHandle sm = 0;

	if (sNumPhysicsClients >= MAX_PHYSICS_CLIENTS)
	{
		PyErr_SetString(SpamError,
						"Exceeding maximum number of physics connections.");
		return NULL;
	}

	{
		int key = SHARED_MEMORY_KEY;
		int udpPort = 1234;
		int tcpPort = 6667;
		int grpcPort = -1;
		int argc = 0;
		char** argv = 0;

		const char* hostName = "localhost";

		static char* kwlist1[] = {"method", "key", "options", NULL};
		static char* kwlist2[] = {"method", "hostName", "port", "options", NULL};

		if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|is", kwlist1, &method, &key, &options))
		{
			PyErr_Clear();
			int port = -1;
			if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|sis", kwlist2, &method, &hostName, &port, &options))
			{
				return NULL;
			}
			else
			{
				PyErr_Clear();
				if (port >= 0)
				{
					udpPort = port;
					tcpPort = port;
					grpcPort = port;
				}
			}
		}

		//Only one local in-process GUI connection allowed.
		if (method == eCONNECT_GUI)
		{
			int i;
			for (i = 0; i < MAX_PHYSICS_CLIENTS; i++)
			{
				if ((sPhysicsClientsGUI[i] == eCONNECT_GUI) || (sPhysicsClientsGUI[i] == eCONNECT_GUI_SERVER))
				{
					PyErr_SetString(SpamError,
									"Only one local in-process GUI/GUI_SERVER connection allowed. Use DIRECT connection mode or start a separate GUI physics server (ExampleBrowser, App_SharedMemoryPhysics_GUI, App_SharedMemoryPhysics_VR) and connect over SHARED_MEMORY, UDP or TCP instead.");
					return NULL;
				}
			}
		}

		if (options)
		{
			int i;
			argv = urdfStrSplit(options, " ");
			argc = urdfStrArrayLen(argv);
			for (i = 0; i < argc; i++)
			{
				printf("argv[%d]=%s\n", i, argv[i]);
			}
		}
		switch (method)
		{
	
			case eCONNECT_GUI:
			{
#ifdef __APPLE__
				sm = b3CreateInProcessPhysicsServerAndConnectMainThread(argc, argv);
#else
				sm = b3CreateInProcessPhysicsServerAndConnect(argc, argv);
#endif
				break;
			}
			case eCONNECT_GUI_MAIN_THREAD:
			{
				sm = b3CreateInProcessPhysicsServerAndConnectMainThread(argc, argv);
				break;
			}
			case eCONNECT_GUI_SERVER:
			{
#ifdef __APPLE__
				sm = b3CreateInProcessPhysicsServerAndConnectMainThreadSharedMemory(argc, argv);
#else
				sm = b3CreateInProcessPhysicsServerAndConnectSharedMemory(argc, argv);
#endif
				break;
			}
			case eCONNECT_GRAPHICS_SERVER_MAIN_THREAD:
			{
				sm = b3CreateInProcessGraphicsServerAndConnectMainThreadSharedMemory(tcpPort);
				break;
			}
			case eCONNECT_GRAPHICS_SERVER:
			{
#ifdef __APPLE__
				sm = b3CreateInProcessGraphicsServerAndConnectMainThreadSharedMemory(tcpPort);
#else
				sm = b3CreateInProcessGraphicsServerAndConnectSharedMemory(tcpPort);
#endif
				break;
			}
			case eCONNECT_SHARED_MEMORY_SERVER:
			{
				sm = b3CreateInProcessPhysicsServerFromExistingExampleBrowserAndConnect3(0, key);
				break;
			}

			case eCONNECT_SHARED_MEMORY_GUI:
			{
				sm = b3CreateInProcessPhysicsServerFromExistingExampleBrowserAndConnect4(0, key);
				break;
			}

			case eCONNECT_GRAPHICS_SERVER_TCP:
			{
#ifdef BT_ENABLE_CLSOCKET
				sm = b3CreateInProcessPhysicsServerFromExistingExampleBrowserAndConnectTCP(hostName, tcpPort);
#else
				PyErr_SetString(SpamError, "TCP is not enabled in this pybullet build");
				return NULL;
#endif//BT_ENABLE_CLSOCKET
				break;
			}

			
			case eCONNECT_DIRECT:
			{
				sm = b3ConnectPhysicsDirect();
				break;
			}
#ifdef BT_ENABLE_DART
			case eCONNECT_DART:
			{
				sm = b3ConnectPhysicsDART();
				break;
			}
#endif
#ifdef BT_ENABLE_PHYSX
			case eCONNECT_PHYSX:
			{
				sm = b3ConnectPhysX(argc, argv);
				break;
			}
#endif

#ifdef BT_ENABLE_MUJOCO
			case eCONNECT_MUJOCO:
			{
				sm = b3ConnectPhysicsMuJoCo();
				break;
			}
#endif
			case eCONNECT_GRPC:
			{
#ifdef BT_ENABLE_GRPC
				sm = b3ConnectPhysicsGRPC(hostName, grpcPort);
#else
				PyErr_SetString(SpamError, "GRPC is not enabled in this pybullet build");
#endif
				break;
			}
			case eCONNECT_SHARED_MEMORY:
			{
				sm = b3ConnectSharedMemory(key);
				break;
			}
			case eCONNECT_UDP:
			{
#ifdef BT_ENABLE_ENET

				sm = b3ConnectPhysicsUDP(hostName, udpPort);
#else
				PyErr_SetString(SpamError, "UDP is not enabled in this pybullet build");
				return NULL;
#endif  //BT_ENABLE_ENET

				break;
			}
			case eCONNECT_TCP:
			{
#ifdef BT_ENABLE_CLSOCKET

				sm = b3ConnectPhysicsTCP(hostName, tcpPort);
#else
				PyErr_SetString(SpamError, "TCP is not enabled in this pybullet build");
				return NULL;
#endif  //BT_ENABLE_CLSOCKET

				break;
			}

			default:
			{
				PyErr_SetString(SpamError, "connectPhysicsServer unexpected argument");
				return NULL;
			}
		};

		if (options)
		{
			urdfStrArrayFree(argv);
		}
	}

	if (sm)
	{
		if (b3CanSubmitCommand(sm))
		{
			for (i = 0; i < MAX_PHYSICS_CLIENTS; i++)
			{
				if (sPhysicsClients1[i] == 0)
				{
					freeIndex = i;
					break;
				}
			}

			if (freeIndex >= 0)
			{
				b3SharedMemoryCommandHandle command;
				b3SharedMemoryStatusHandle statusHandle;
				int statusType;

				sPhysicsClients1[freeIndex] = sm;
				sPhysicsClientsGUI[freeIndex] = method;
				sNumPhysicsClients++;

				if (method != eCONNECT_GRAPHICS_SERVER && method != eCONNECT_GRAPHICS_SERVER_MAIN_THREAD)
				{
					command = b3InitSyncBodyInfoCommand(sm);
					statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
					statusType = b3GetStatusType(statusHandle);

					if (statusType != CMD_SYNC_BODY_INFO_COMPLETED)
					{
						printf("Connection terminated, couldn't get body info\n");
						b3DisconnectSharedMemory(sm);
						sm = 0;
						sPhysicsClients1[freeIndex] = 0;
						sPhysicsClientsGUI[freeIndex] = 0;
						sNumPhysicsClients++;
						return PyInt_FromLong(-1);
					}

					command = b3InitSyncUserDataCommand(sm);
					statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
					statusType = b3GetStatusType(statusHandle);

					if (statusType != CMD_SYNC_USER_DATA_COMPLETED)
					{
						printf("Connection terminated, couldn't get user data\n");
						b3DisconnectSharedMemory(sm);
						sm = 0;
						sPhysicsClients1[freeIndex] = 0;
						sPhysicsClientsGUI[freeIndex] = 0;
						sNumPhysicsClients++;
						return PyInt_FromLong(-1);
					}
				}
			}
		}
		else
		{
			b3DisconnectSharedMemory(sm);
		}
	}
	return PyInt_FromLong(freeIndex);
}

static PyObject* pybullet_disconnectPhysicsServer(PyObject* self,
												  PyObject* args,
												  PyObject* keywds)
{
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	static char* kwlist[] = {"physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|i", kwlist, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	{
		b3DisconnectSharedMemory(sm);
		sm = 0;
	}

	sPhysicsClients1[physicsClientId] = 0;
	sPhysicsClientsGUI[physicsClientId] = 0;
	sNumPhysicsClients--;

	Py_INCREF(Py_None);
	return Py_None;
}

///to avoid memory leaks, disconnect all physics servers explicitly
void b3pybulletExitFunc(void)
{
	int i;
	for (i = 0; i < MAX_PHYSICS_CLIENTS; i++)
	{
		if (sPhysicsClients1[i])
		{
			b3DisconnectSharedMemory(sPhysicsClients1[i]);
			sPhysicsClients1[i] = 0;
			sNumPhysicsClients--;
		}
	}
}

static PyObject* pybullet_isConnected(PyObject* self, PyObject* args, PyObject* keywds)
{
	int physicsClientId = 0;
	int isConnected = 0;
	int method = 0;
	PyObject* pylist = 0;
	PyObject* val = 0;
	b3PhysicsClientHandle sm = 0;
	static char* kwlist[] = {"physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|i", kwlist, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm != 0)
	{
		if (b3CanSubmitCommand(sm))
		{
			isConnected = 1;
			method = sPhysicsClientsGUI[physicsClientId];
		}
	}

	return PyLong_FromLong(isConnected);
}

static PyObject* pybullet_getConnectionInfo(PyObject* self, PyObject* args, PyObject* keywds)
{
	int physicsClientId = 0;
	int isConnected = 0;
	int method = 0;
	PyObject* pylist = 0;
	PyObject* val = 0;
	b3PhysicsClientHandle sm = 0;
	static char* kwlist[] = {"physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|i", kwlist, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm != 0)
	{
		if (b3CanSubmitCommand(sm))
		{
			isConnected = 1;
			method = sPhysicsClientsGUI[physicsClientId];
		}
	}

	val = Py_BuildValue("{s:i,s:i}", "isConnected", isConnected, "connectionMethod", method);
	return val;
}

static PyObject* pybullet_syncBodyInfo(PyObject* self, PyObject* args, PyObject* keywds)
{
	b3PhysicsClientHandle sm = 0;
	int physicsClientId = 0;
	static char* kwlist[] = {"physicsClientId", NULL};
	b3SharedMemoryCommandHandle command;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|i", kwlist, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	command = b3InitSyncBodyInfoCommand(sm);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusType(statusHandle);

	if (statusType != CMD_SYNC_BODY_INFO_COMPLETED)
	{
		PyErr_SetString(SpamError, "Error in syncBodyzInfo command.");
		return NULL;
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_syncUserData(PyObject* self, PyObject* args, PyObject* keywds)
{
	b3PhysicsClientHandle sm = 0;
	int physicsClientId = 0;
	static char* kwlistSingleBody[] = {"bodyUniqueId", "physicsClientId", NULL};
	static char* kwlistMultipleBodies[] = {"bodyUniqueIds", "physicsClientId", NULL};
	b3SharedMemoryCommandHandle command;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	PyObject* bodyUniqueIdsObj = 0;
	int requestedBodyUniqueId = -1;
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|ii", kwlistSingleBody, &requestedBodyUniqueId, &physicsClientId)) {
		PyErr_Clear();
		if (!PyArg_ParseTupleAndKeywords(args, keywds, "|Oi", kwlistMultipleBodies, &bodyUniqueIdsObj, &physicsClientId))
		{
			return NULL;
		}
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	command = b3InitSyncUserDataCommand(sm);
	if (bodyUniqueIdsObj)
	{
		PyObject *seq = PySequence_Fast(bodyUniqueIdsObj, "expected a sequence");
		int len = PySequence_Size(bodyUniqueIdsObj);
		int i;
		for (i=0; i < len; ++i)
		{
			b3AddBodyToSyncUserDataRequest(command, pybullet_internalGetIntFromSequence(seq, i));
		}
	}
	else if (requestedBodyUniqueId != -1)
	{
		b3AddBodyToSyncUserDataRequest(command, requestedBodyUniqueId);

	}
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusType(statusHandle);

	if (statusType != CMD_SYNC_USER_DATA_COMPLETED)
	{
		PyErr_SetString(SpamError, "Error in syncUserInfo command.");
		return NULL;
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_addUserData(PyObject* self, PyObject* args, PyObject* keywds)
{
	b3PhysicsClientHandle sm = 0;
	int physicsClientId = 0;
	int bodyUniqueId = -1;
	int linkIndex = -1;
	int visualShapeIndex = -1;
	const char* key = "";
	const char* value = "";  // TODO: Change this to a PyObject and detect the type dynamically.

	static char* kwlist[] = {"bodyUniqueId", "key", "value", "linkIndex", "visualShapeIndex", "physicsClientId", NULL};
	b3SharedMemoryCommandHandle command;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	int userDataId;
	int valueLen = -1;

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "iss|iii", kwlist, &bodyUniqueId, &key, &value, &linkIndex, &visualShapeIndex, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	valueLen = strlen(value) + 1;
	command = b3InitAddUserDataCommand(sm, bodyUniqueId, linkIndex, visualShapeIndex, key, USER_DATA_VALUE_TYPE_STRING, valueLen, value);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusType(statusHandle);

	if (statusType != CMD_ADD_USER_DATA_COMPLETED)
	{
		PyErr_SetString(SpamError, "Error in addUserData command.");
		return NULL;
	}

	userDataId = b3GetUserDataIdFromStatus(statusHandle);
	return PyInt_FromLong(userDataId);
}

static PyObject* pybullet_removeUserData(PyObject* self, PyObject* args, PyObject* keywds)
{
	b3PhysicsClientHandle sm = 0;
	int physicsClientId = 0;
	int userDataId = -1;

	static char* kwlist[] = {"userDataId", "physicsClientId", NULL};
	b3SharedMemoryCommandHandle command;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist, &userDataId, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	command = b3InitRemoveUserDataCommand(sm, userDataId);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusType(statusHandle);

	if (statusType != CMD_REMOVE_USER_DATA_COMPLETED)
	{
		PyErr_SetString(SpamError, "Error in removeUserData command.");
		return NULL;
	}
	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_getUserDataId(PyObject* self, PyObject* args, PyObject* keywds)
{
	b3PhysicsClientHandle sm = 0;
	int physicsClientId = 0;
	int bodyUniqueId = -1;
	int linkIndex = -1;
	int visualShapeIndex = -1;
	const char* key = "";
	int userDataId;

	static char* kwlist[] = {"bodyUniqueId", "key", "linkIndex", "visualShapeIndex", "physicsClientId", NULL};

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "is|iii", kwlist, &bodyUniqueId, &key, &linkIndex, &visualShapeIndex, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	userDataId = b3GetUserDataId(sm, bodyUniqueId, linkIndex, visualShapeIndex, key);
	return PyInt_FromLong(userDataId);
}

static PyObject* pybullet_getUserData(PyObject* self, PyObject* args, PyObject* keywds)
{
	b3PhysicsClientHandle sm = 0;
	int physicsClientId = 0;
	int userDataId = -1;

	static char* kwlist[] = {"userDataId", "physicsClientId", NULL};

	struct b3UserDataValue value;
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist, &userDataId, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	if (!b3GetUserData(sm, userDataId, &value))
	{
		Py_INCREF(Py_None);
		return Py_None;
	}
	if (value.m_type != USER_DATA_VALUE_TYPE_STRING)
	{
		PyErr_SetString(SpamError, "User data value has unknown type");
		return NULL;
	}

	return PyString_FromString((const char*)value.m_data1);
}

static PyObject* pybullet_getNumUserData(PyObject* self, PyObject* args, PyObject* keywds)
{
	b3PhysicsClientHandle sm = 0;
	int physicsClientId = 0;
	int bodyUniqueId = -1;

	static char* kwlist[] = {"bodyUniqueId", "physicsClientId", NULL};

	int numUserData;

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist, &bodyUniqueId, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	numUserData = b3GetNumUserData(sm, bodyUniqueId);
	return PyInt_FromLong(numUserData);
}

static PyObject* pybullet_getUserDataInfo(PyObject* self, PyObject* args, PyObject* keywds)
{
	b3PhysicsClientHandle sm = 0;
	int physicsClientId = 0;
	int bodyUniqueId = -1;
	int userDataIndex = -1;
	int linkIndex = -1;
	int visualShapeIndex = -1;

	static char* kwlist[] = {"bodyUniqueId", "userDataIndex", "physicsClientId", NULL};

	const char* key = 0;
	int userDataId = -1;

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "ii|i", kwlist, &bodyUniqueId, &userDataIndex, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	b3GetUserDataInfo(sm, bodyUniqueId, userDataIndex, &key, &userDataId, &linkIndex, &visualShapeIndex);
	if (key == 0 || userDataId == -1)
	{
		PyErr_SetString(SpamError, "Could not get user data info.");
		return NULL;
	}

	{
		PyObject* userDataInfoTuple = PyTuple_New(5);
		PyTuple_SetItem(userDataInfoTuple, 0, PyInt_FromLong(userDataId));
		PyTuple_SetItem(userDataInfoTuple, 1, PyString_FromString(key));
		PyTuple_SetItem(userDataInfoTuple, 2, PyInt_FromLong(bodyUniqueId));
		PyTuple_SetItem(userDataInfoTuple, 3, PyInt_FromLong(linkIndex));
		PyTuple_SetItem(userDataInfoTuple, 4, PyInt_FromLong(visualShapeIndex));
		return userDataInfoTuple;
	}
}

static PyObject* pybullet_saveWorld(PyObject* self, PyObject* args, PyObject* keywds)
{
	const char* worldFileName = "";

	b3PhysicsClientHandle sm = 0;
	int physicsClientId = 0;
	static char* kwlist[] = {"worldFileName", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "s|i", kwlist, &worldFileName, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	{
		b3SharedMemoryCommandHandle command;
		b3SharedMemoryStatusHandle statusHandle;
		int statusType;

		command = b3SaveWorldCommandInit(sm, worldFileName);
		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
		statusType = b3GetStatusType(statusHandle);
		if (statusType != CMD_SAVE_WORLD_COMPLETED)
		{
			PyErr_SetString(SpamError, "saveWorld command execution failed.");
			return NULL;
		}
		Py_INCREF(Py_None);
		return Py_None;
	}

	PyErr_SetString(SpamError, "Cannot execute saveWorld command.");
	return NULL;
}

static PyObject* pybullet_loadBullet(PyObject* self, PyObject* args, PyObject* keywds)
{
	const char* bulletFileName = "";
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	b3SharedMemoryCommandHandle command;
	int i, numBodies;
	int bodyIndicesOut[MAX_SDF_BODIES];
	PyObject* pylist = 0;
	b3PhysicsClientHandle sm = 0;

	int physicsClientId = 0;
	static char* kwlist[] = {"bulletFileName", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "s|i", kwlist, &bulletFileName, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	command = b3LoadBulletCommandInit(sm, bulletFileName);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusType(statusHandle);
	if (statusType != CMD_BULLET_LOADING_COMPLETED)
	{
		PyErr_SetString(SpamError, "Couldn't load .bullet file.");
		return NULL;
	}

	numBodies =
		b3GetStatusBodyIndices(statusHandle, bodyIndicesOut, MAX_SDF_BODIES);
	if (numBodies > MAX_SDF_BODIES)
	{
		PyErr_SetString(SpamError, "loadBullet exceeds body capacity");
		return NULL;
	}

	pylist = PyTuple_New(numBodies);

	if (numBodies > 0 && numBodies <= MAX_SDF_BODIES)
	{
		for (i = 0; i < numBodies; i++)
		{
			PyTuple_SetItem(pylist, i, PyInt_FromLong(bodyIndicesOut[i]));
		}
	}
	return pylist;
}

static PyObject* pybullet_saveBullet(PyObject* self, PyObject* args, PyObject* keywds)
{
	const char* bulletFileName = "";
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	b3SharedMemoryCommandHandle command;
	b3PhysicsClientHandle sm = 0;

	int physicsClientId = 0;
	static char* kwlist[] = {"bulletFileName", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "s|i", kwlist, &bulletFileName, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	command = b3SaveBulletCommandInit(sm, bulletFileName);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusType(statusHandle);
	if (statusType != CMD_BULLET_SAVING_COMPLETED)
	{
		PyErr_SetString(SpamError, "Couldn't save .bullet file.");
		return NULL;
	}
	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_restoreState(PyObject* self, PyObject* args, PyObject* keywds)
{
	const char* fileName = "";
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	int stateId = -1;
	b3SharedMemoryCommandHandle command;

	PyObject* pylist = 0;
	b3PhysicsClientHandle sm = 0;

	int physicsClientId = 0;
	static char* kwlist[] = {"stateId", "fileName", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|isi", kwlist, &stateId, &fileName, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	command = b3LoadStateCommandInit(sm);
	if (stateId >= 0)
	{
		b3LoadStateSetStateId(command, stateId);
	}
	if (fileName)
	{
		b3LoadStateSetFileName(command, fileName);
	}
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusType(statusHandle);
	if (statusType != CMD_RESTORE_STATE_COMPLETED)
	{
		PyErr_SetString(SpamError, "Couldn't restore state.");
		return NULL;
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_saveState(PyObject* self, PyObject* args, PyObject* keywds)
{
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	b3SharedMemoryCommandHandle command;
	b3PhysicsClientHandle sm = 0;
	int stateId = -1;

	int physicsClientId = 0;
	static char* kwlist[] = {"physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|i", kwlist, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	command = b3SaveStateCommandInit(sm);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusType(statusHandle);

	if (statusType != CMD_SAVE_STATE_COMPLETED)
	{
		PyErr_SetString(SpamError, "Couldn't save state");
		return NULL;
	}

	stateId = b3GetStatusGetStateId(statusHandle);
	return PyInt_FromLong(stateId);
}

static PyObject* pybullet_removeState(PyObject* self, PyObject* args, PyObject* keywds)
{
	{
		int stateUniqueId = -1;
		b3PhysicsClientHandle sm = 0;

		int physicsClientId = 0;
		static char* kwlist[] = { "stateUniqueId", "physicsClientId", NULL };
		if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist, &stateUniqueId, &physicsClientId))
		{
			return NULL;
		}
		sm = getPhysicsClient(physicsClientId);
		if (sm == 0)
		{
			PyErr_SetString(SpamError, "Not connected to physics server.");
			return NULL;
		}
		if (stateUniqueId >= 0)
		{
			b3SharedMemoryStatusHandle statusHandle;
			int statusType;
			if (b3CanSubmitCommand(sm))
			{
				statusHandle = b3SubmitClientCommandAndWaitStatus(sm, b3InitRemoveStateCommand(sm, stateUniqueId));
				statusType = b3GetStatusType(statusHandle);
			}
		}
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_loadMJCF(PyObject* self, PyObject* args, PyObject* keywds)
{
	const char* mjcfFileName = "";
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	b3SharedMemoryCommandHandle command;
	b3PhysicsClientHandle sm = 0;
	int numBodies = 0;
	int i;
	int bodyIndicesOut[MAX_SDF_BODIES];
	PyObject* pylist = 0;
	int physicsClientId = 0;
	int flags = -1;

	static char* kwlist[] = {"mjcfFileName", "flags", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "s|ii", kwlist, &mjcfFileName, &flags, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	command = b3LoadMJCFCommandInit(sm, mjcfFileName);
	if (flags >= 0)
	{
		b3LoadMJCFCommandSetFlags(command, flags);
	}
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusType(statusHandle);
	if (statusType != CMD_MJCF_LOADING_COMPLETED)
	{
		PyErr_SetString(SpamError, "Couldn't load .mjcf file.");
		return NULL;
	}

	numBodies =
		b3GetStatusBodyIndices(statusHandle, bodyIndicesOut, MAX_SDF_BODIES);
	if (numBodies > MAX_SDF_BODIES)
	{
		char str[1024];
		sprintf(str, "SDF exceeds body capacity: %d > %d", numBodies, MAX_SDF_BODIES);
		PyErr_SetString(SpamError, str);
		return NULL;
	}

	pylist = PyTuple_New(numBodies);

	if (numBodies > 0 && numBodies <= MAX_SDF_BODIES)
	{
		for (i = 0; i < numBodies; i++)
		{
			PyTuple_SetItem(pylist, i, PyInt_FromLong(bodyIndicesOut[i]));
		}
	}
	return pylist;
}

static PyObject* pybullet_changeDynamicsInfo(PyObject* self, PyObject* args, PyObject* keywds)
{
	int bodyUniqueId = -1;
	int linkIndex = -2;
	double mass = -1;
	double lateralFriction = -1;
	double spinningFriction = -1;
	double rollingFriction = -1;
	double restitution = -1;
	double linearDamping = -1;
	double angularDamping = -1;

	double contactStiffness = -1;
	double contactDamping = -1;
	double ccdSweptSphereRadius = -1;
	double collisionMargin = -1;
	int frictionAnchor = -1;
	double contactProcessingThreshold = -1;
	int activationState = -1;
	double jointDamping = -1;
	PyObject* localInertiaDiagonalObj = 0;
	PyObject* anisotropicFrictionObj = 0;
	double maxJointVelocity = -1;
	
	double jointLowerLimit = 1;
	double jointUpperLimit = -1;
	double jointLimitForce = -1;

	b3PhysicsClientHandle sm = 0;

	int physicsClientId = 0;
	static char* kwlist[] = {"bodyUniqueId", "linkIndex", "mass", "lateralFriction", "spinningFriction", "rollingFriction", "restitution", "linearDamping", "angularDamping", "contactStiffness", "contactDamping", "frictionAnchor", "localInertiaDiagonal", "ccdSweptSphereRadius", "contactProcessingThreshold", "activationState", "jointDamping", "anisotropicFriction", "maxJointVelocity",  "collisionMargin", "jointLowerLimit","jointUpperLimit", "jointLimitForce",  "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "ii|dddddddddiOddidOdddddi", kwlist, &bodyUniqueId, &linkIndex, &mass, &lateralFriction, &spinningFriction, &rollingFriction, &restitution, &linearDamping, &angularDamping, &contactStiffness, &contactDamping, &frictionAnchor, &localInertiaDiagonalObj, &ccdSweptSphereRadius, &contactProcessingThreshold, &activationState, &jointDamping, &anisotropicFrictionObj, &maxJointVelocity, &collisionMargin , &jointLowerLimit , &jointUpperLimit , &jointLimitForce , &physicsClientId))
	{
		return NULL;
	}

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	if ((contactStiffness >= 0 && contactDamping < 0) || (contactStiffness < 0 && contactDamping >= 0))
	{
		PyErr_SetString(SpamError, "Both contactStiffness and contactDamping needs to be set together.");
		return NULL;
	}

	{
		b3SharedMemoryCommandHandle command = b3InitChangeDynamicsInfo(sm);
		b3SharedMemoryStatusHandle statusHandle;

		if (jointLimitForce >= 0)
		{
			b3ChangeDynamicsInfoSetJointLimitForce(command, bodyUniqueId, linkIndex, jointLimitForce);
		}

		if (jointLowerLimit <= jointUpperLimit)
		{
			b3ChangeDynamicsInfoSetJointLimit(command, bodyUniqueId, linkIndex, jointLowerLimit, jointUpperLimit);
		}

		if (mass >= 0)
		{
			b3ChangeDynamicsInfoSetMass(command, bodyUniqueId, linkIndex, mass);
		}
		if (anisotropicFrictionObj)
		{
			double anisotropicFriction[3];
			pybullet_internalSetVectord(anisotropicFrictionObj, anisotropicFriction);
			b3ChangeDynamicsInfoSetAnisotropicFriction(command, bodyUniqueId, linkIndex, anisotropicFriction);
		}
		if (localInertiaDiagonalObj)
		{
			double localInertiaDiagonal[3];
			pybullet_internalSetVectord(localInertiaDiagonalObj, localInertiaDiagonal);
			b3ChangeDynamicsInfoSetLocalInertiaDiagonal(command, bodyUniqueId, linkIndex, localInertiaDiagonal);
		}
		if (lateralFriction >= 0)
		{
			b3ChangeDynamicsInfoSetLateralFriction(command, bodyUniqueId, linkIndex, lateralFriction);
		}
		if (spinningFriction >= 0)
		{
			b3ChangeDynamicsInfoSetSpinningFriction(command, bodyUniqueId, linkIndex, spinningFriction);
		}
		if (rollingFriction >= 0)
		{
			b3ChangeDynamicsInfoSetRollingFriction(command, bodyUniqueId, linkIndex, rollingFriction);
		}

		if (linearDamping >= 0)
		{
			b3ChangeDynamicsInfoSetLinearDamping(command, bodyUniqueId, linearDamping);
		}
		if (angularDamping >= 0)
		{
			b3ChangeDynamicsInfoSetAngularDamping(command, bodyUniqueId, angularDamping);
		}

		if (jointDamping >= 0)
		{
			b3ChangeDynamicsInfoSetJointDamping(command, bodyUniqueId, linkIndex, jointDamping);
		}
		if (restitution >= 0)
		{
			b3ChangeDynamicsInfoSetRestitution(command, bodyUniqueId, linkIndex, restitution);
		}
		if (contactStiffness >= 0 && contactDamping >= 0)
		{
			b3ChangeDynamicsInfoSetContactStiffnessAndDamping(command, bodyUniqueId, linkIndex, contactStiffness, contactDamping);
		}
		if (frictionAnchor >= 0)
		{
			b3ChangeDynamicsInfoSetFrictionAnchor(command, bodyUniqueId, linkIndex, frictionAnchor);
		}
		if (ccdSweptSphereRadius >= 0)
		{
			b3ChangeDynamicsInfoSetCcdSweptSphereRadius(command, bodyUniqueId, linkIndex, ccdSweptSphereRadius);
		}
		if (activationState >= 0)
		{
			b3ChangeDynamicsInfoSetActivationState(command, bodyUniqueId, activationState);
		}
		if (contactProcessingThreshold >= 0)
		{
			b3ChangeDynamicsInfoSetContactProcessingThreshold(command, bodyUniqueId, linkIndex, contactProcessingThreshold);
		}
		if (maxJointVelocity >= 0)
		{
			b3ChangeDynamicsInfoSetMaxJointVelocity(command, bodyUniqueId, maxJointVelocity);
		}

		if (collisionMargin >= 0)
		{
			b3ChangeDynamicsInfoSetCollisionMargin(command, bodyUniqueId, collisionMargin);
		}
				
		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_getDynamicsInfo(PyObject* self, PyObject* args, PyObject* keywds)
{
	{
		int bodyUniqueId = -1;
		int linkIndex = -2;

		b3PhysicsClientHandle sm = 0;

		int physicsClientId = 0;
		static char* kwlist[] = {"bodyUniqueId", "linkIndex", "physicsClientId", NULL};
		if (!PyArg_ParseTupleAndKeywords(args, keywds, "ii|i", kwlist, &bodyUniqueId, &linkIndex, &physicsClientId))
		{
			return NULL;
		}
		sm = getPhysicsClient(physicsClientId);
		if (sm == 0)
		{
			PyErr_SetString(SpamError, "Not connected to physics server.");
			return NULL;
		}
		{
			int status_type = 0;
			b3SharedMemoryCommandHandle cmd_handle;
			b3SharedMemoryStatusHandle status_handle;
			struct b3DynamicsInfo info;

			if (bodyUniqueId < 0)
			{
				PyErr_SetString(SpamError, "getDynamicsInfo failed; invalid bodyUniqueId");
				return NULL;
			}
			if (linkIndex < -1)
			{
				PyErr_SetString(SpamError, "getDynamicsInfo failed; invalid linkIndex");
				return NULL;
			}
			cmd_handle = b3GetDynamicsInfoCommandInit(sm, bodyUniqueId, linkIndex);
			status_handle = b3SubmitClientCommandAndWaitStatus(sm, cmd_handle);
			status_type = b3GetStatusType(status_handle);
			if (status_type != CMD_GET_DYNAMICS_INFO_COMPLETED)
			{
				PyErr_SetString(SpamError, "getDynamicsInfo failed; invalid return status");
				return NULL;
			}

			if (b3GetDynamicsInfo(status_handle, &info))
			{
				int numFields = 12;
				PyObject* pyDynamicsInfo = PyTuple_New(numFields);
				PyTuple_SetItem(pyDynamicsInfo, 0, PyFloat_FromDouble(info.m_mass));
				PyTuple_SetItem(pyDynamicsInfo, 1, PyFloat_FromDouble(info.m_lateralFrictionCoeff));

				{
					PyObject* pyInertiaDiag = PyTuple_New(3);
					PyTuple_SetItem(pyInertiaDiag, 0, PyFloat_FromDouble(info.m_localInertialDiagonal[0]));
					PyTuple_SetItem(pyInertiaDiag, 1, PyFloat_FromDouble(info.m_localInertialDiagonal[1]));
					PyTuple_SetItem(pyInertiaDiag, 2, PyFloat_FromDouble(info.m_localInertialDiagonal[2]));
					PyTuple_SetItem(pyDynamicsInfo, 2, pyInertiaDiag);
				}
				{
					PyObject* pyInertiaPos = PyTuple_New(3);
					PyTuple_SetItem(pyInertiaPos, 0, PyFloat_FromDouble(info.m_localInertialFrame[0]));
					PyTuple_SetItem(pyInertiaPos, 1, PyFloat_FromDouble(info.m_localInertialFrame[1]));
					PyTuple_SetItem(pyInertiaPos, 2, PyFloat_FromDouble(info.m_localInertialFrame[2]));
					PyTuple_SetItem(pyDynamicsInfo, 3, pyInertiaPos);
				}
				{
					PyObject* pyInertiaOrn = PyTuple_New(4);
					PyTuple_SetItem(pyInertiaOrn, 0, PyFloat_FromDouble(info.m_localInertialFrame[3]));
					PyTuple_SetItem(pyInertiaOrn, 1, PyFloat_FromDouble(info.m_localInertialFrame[4]));
					PyTuple_SetItem(pyInertiaOrn, 2, PyFloat_FromDouble(info.m_localInertialFrame[5]));
					PyTuple_SetItem(pyInertiaOrn, 3, PyFloat_FromDouble(info.m_localInertialFrame[6]));
					PyTuple_SetItem(pyDynamicsInfo, 4, pyInertiaOrn);
				}
				PyTuple_SetItem(pyDynamicsInfo, 5, PyFloat_FromDouble(info.m_restitution));
				PyTuple_SetItem(pyDynamicsInfo, 6, PyFloat_FromDouble(info.m_rollingFrictionCoeff));
				PyTuple_SetItem(pyDynamicsInfo, 7, PyFloat_FromDouble(info.m_spinningFrictionCoeff));
				PyTuple_SetItem(pyDynamicsInfo, 8, PyFloat_FromDouble(info.m_contactDamping));
				PyTuple_SetItem(pyDynamicsInfo, 9, PyFloat_FromDouble(info.m_contactStiffness));
				PyTuple_SetItem(pyDynamicsInfo, 10, PyInt_FromLong(info.m_bodyType));
				PyTuple_SetItem(pyDynamicsInfo, 11, PyFloat_FromDouble(info.m_collisionMargin));
				return pyDynamicsInfo;
			}
		}
	}
	PyErr_SetString(SpamError, "Couldn't get dynamics info");
	return NULL;
}

static PyObject* pybullet_getPhysicsEngineParameters(PyObject* self, PyObject* args, PyObject* keywds)
{
	b3PhysicsClientHandle sm = 0;
	PyObject* val = 0;
	int physicsClientId = 0;
	static char* kwlist[] = {"physicsClientId", NULL};

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|i", kwlist, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	{
		b3SharedMemoryCommandHandle command = b3InitRequestPhysicsParamCommand(sm);
		b3SharedMemoryStatusHandle statusHandle;
		struct b3PhysicsSimulationParameters params;
		int statusType;

		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
		statusType = b3GetStatusType(statusHandle);
		if (statusType != CMD_REQUEST_PHYSICS_SIMULATION_PARAMETERS_COMPLETED)
		{
			PyErr_SetString(SpamError, "Couldn't get physics simulation parameters.");
			return NULL;
		}
		b3GetStatusPhysicsSimulationParameters(statusHandle, &params);

		//for now, return a subset, expose more/all on request
		val = Py_BuildValue("{s:d,s:i,s:i,s:i,s:d,s:d,s:d, s:i}",
							"fixedTimeStep", params.m_deltaTime,
							"numSubSteps", params.m_numSimulationSubSteps,
							"numSolverIterations", params.m_numSolverIterations,
							"useRealTimeSimulation", params.m_useRealTimeSimulation,
							"gravityAccelerationX", params.m_gravityAcceleration[0],
							"gravityAccelerationY", params.m_gravityAcceleration[1],
							"gravityAccelerationZ", params.m_gravityAcceleration[2],
							"numNonContactInnerIterations", params.m_numNonContactInnerIterations);
		return val;
	}
	//"fixedTimeStep", "numSolverIterations", "useSplitImpulse", "splitImpulsePenetrationThreshold", "numSubSteps", "collisionFilterMode", "contactBreakingThreshold", "maxNumCmdPer1ms", "enableFileCaching","restitutionVelocityThreshold", "erp", "contactERP", "frictionERP",
	//val = Py_BuildValue("{s:i,s:i}","isConnected", isConnected, "connectionMethod", method);
}

static PyObject* pybullet_setPhysicsEngineParameter(PyObject* self, PyObject* args, PyObject* keywds)
{
	double fixedTimeStep = -1;
	int numSolverIterations = -1;
	int useSplitImpulse = -1;
	double splitImpulsePenetrationThreshold = -1;
	int numSubSteps = -1;
	int collisionFilterMode = -1;
	double contactBreakingThreshold = -1;
	int maxNumCmdPer1ms = -2;
	int enableFileCaching = -1;
	double restitutionVelocityThreshold = -1;
	double erp = -1;
	double contactERP = -1;
	double frictionERP = -1;
	double allowedCcdPenetration = -1;

	int enableConeFriction = -1;
	b3PhysicsClientHandle sm = 0;
	int deterministicOverlappingPairs = -1;
	int jointFeedbackMode = -1;
	double solverResidualThreshold = -1;
	double contactSlop = -1;
	int enableSAT = -1;
	int constraintSolverType = -1;
	double globalCFM = -1;

	int minimumSolverIslandSize = -1;
	int reportSolverAnalytics = -1;

	double warmStartingFactor = -1;
	double sparseSdfVoxelSize = -1;
	int numNonContactInnerIterations = -1;

	int physicsClientId = 0;

	static char* kwlist[] = {"fixedTimeStep",
							 "numSolverIterations",
							 "useSplitImpulse",
							 "splitImpulsePenetrationThreshold",
							 "numSubSteps",
							 "collisionFilterMode",
							 "contactBreakingThreshold",
							 "maxNumCmdPer1ms",
							 "enableFileCaching",
							 "restitutionVelocityThreshold",
							 "erp",
							 "contactERP",
							 "frictionERP",
							 "enableConeFriction",
							 "deterministicOverlappingPairs",
							 "allowedCcdPenetration",
							 "jointFeedbackMode",
							 "solverResidualThreshold",
							 "contactSlop",
							 "enableSAT",
							 "constraintSolverType",
							 "globalCFM",
							 "minimumSolverIslandSize",
							 "reportSolverAnalytics",
							 "warmStartingFactor",
							 "sparseSdfVoxelSize",
							 "numNonContactInnerIterations",
							 "physicsClientId", NULL};

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|diidiidiiddddiididdiidiiddii", kwlist, &fixedTimeStep, &numSolverIterations, &useSplitImpulse, &splitImpulsePenetrationThreshold, &numSubSteps,
									 &collisionFilterMode, &contactBreakingThreshold, &maxNumCmdPer1ms, &enableFileCaching, &restitutionVelocityThreshold, &erp, &contactERP, &frictionERP, &enableConeFriction, &deterministicOverlappingPairs, &allowedCcdPenetration, &jointFeedbackMode, &solverResidualThreshold, &contactSlop, &enableSAT, &constraintSolverType, &globalCFM, &minimumSolverIslandSize, 
									&reportSolverAnalytics, &warmStartingFactor, &sparseSdfVoxelSize, &numNonContactInnerIterations, &physicsClientId))
	{
		return NULL;
	}

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	{
		b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(sm);
		b3SharedMemoryStatusHandle statusHandle;

		if (numSolverIterations >= 0)
		{
			b3PhysicsParamSetNumSolverIterations(command, numSolverIterations);
		}

		if (minimumSolverIslandSize >= 0)
		{
			b3PhysicsParameterSetMinimumSolverIslandSize(command, minimumSolverIslandSize);
		}

		if (solverResidualThreshold >= 0)
		{
			b3PhysicsParamSetSolverResidualThreshold(command, solverResidualThreshold);
		}

		if (collisionFilterMode >= 0)
		{
			b3PhysicsParamSetCollisionFilterMode(command, collisionFilterMode);
		}
		if (numSubSteps >= 0)
		{
			b3PhysicsParamSetNumSubSteps(command, numSubSteps);
		}
		if (fixedTimeStep >= 0)
		{
			b3PhysicsParamSetTimeStep(command, fixedTimeStep);
		}
		if (useSplitImpulse >= 0)
		{
			b3PhysicsParamSetUseSplitImpulse(command, useSplitImpulse);
		}
		if (splitImpulsePenetrationThreshold >= 0)
		{
			b3PhysicsParamSetSplitImpulsePenetrationThreshold(command, splitImpulsePenetrationThreshold);
		}
		if (contactBreakingThreshold >= 0)
		{
			b3PhysicsParamSetContactBreakingThreshold(command, contactBreakingThreshold);
		}
		if (contactSlop >= 0)
		{
			b3PhysicsParamSetContactSlop(command, contactSlop);
		}

		//-1 is disables the maxNumCmdPer1ms feature, allow it
		if (maxNumCmdPer1ms >= -1)
		{
			b3PhysicsParamSetMaxNumCommandsPer1ms(command, maxNumCmdPer1ms);
		}

		if (restitutionVelocityThreshold >= 0)
		{
			b3PhysicsParamSetRestitutionVelocityThreshold(command, restitutionVelocityThreshold);
		}
		if (enableFileCaching >= 0)
		{
			b3PhysicsParamSetEnableFileCaching(command, enableFileCaching);
		}

		if (erp >= 0)
		{
			b3PhysicsParamSetDefaultNonContactERP(command, erp);
		}
		if (contactERP >= 0)
		{
			b3PhysicsParamSetDefaultContactERP(command, contactERP);
		}
		if (frictionERP >= 0)
		{
			b3PhysicsParamSetDefaultFrictionERP(command, frictionERP);
		}
		if (enableConeFriction >= 0)
		{
			b3PhysicsParamSetEnableConeFriction(command, enableConeFriction);
		}
		if (deterministicOverlappingPairs >= 0)
		{
			b3PhysicsParameterSetDeterministicOverlappingPairs(command, deterministicOverlappingPairs);
		}

		if (allowedCcdPenetration >= 0)
		{
			b3PhysicsParameterSetAllowedCcdPenetration(command, allowedCcdPenetration);
		}
		if (jointFeedbackMode >= 0)
		{
			b3PhysicsParameterSetJointFeedbackMode(command, jointFeedbackMode);
		}

		if (enableSAT >= 0)
		{
			b3PhysicsParameterSetEnableSAT(command, enableSAT);
		}
		if (constraintSolverType >= 0)
		{
			b3PhysicsParameterSetConstraintSolverType(command, constraintSolverType);
		}
		if (globalCFM >= 0)
		{
			b3PhysicsParamSetDefaultGlobalCFM(command, globalCFM);
		}
		if (reportSolverAnalytics >= 0)
		{
			b3PhysicsParamSetSolverAnalytics(command, reportSolverAnalytics);
		}
		if (warmStartingFactor >= 0)
		{
			b3PhysicsParamSetWarmStartingFactor(command, warmStartingFactor);
		}
		if (sparseSdfVoxelSize >= 0)
		{
			b3PhysicsParameterSetSparseSdfVoxelSize(command, sparseSdfVoxelSize);
		}
		if (numNonContactInnerIterations >= 1)
		{
			b3PhysicsParamSetNumNonContactInnerIterations(command, numNonContactInnerIterations);
		}
		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	}

	Py_INCREF(Py_None);
	return Py_None;
}

// Load a robot from a URDF file (universal robot description format)
// function can be called without arguments and will default
// to position (0,0,1) with orientation(0,0,0,1)
// els(x,y,z) or
// loadURDF(pos_x, pos_y, pos_z, orn_x, orn_y, orn_z, orn_w)
static PyObject* pybullet_loadURDF(PyObject* self, PyObject* args, PyObject* keywds)
{
	int physicsClientId = 0;
	int flags = 0;

	static char* kwlist[] = {"fileName", "basePosition", "baseOrientation", "useMaximalCoordinates", "useFixedBase", "flags", "globalScaling", "physicsClientId", NULL};

	static char* kwlistBackwardCompatible4[] = {"fileName", "startPosX", "startPosY", "startPosZ", NULL};
	static char* kwlistBackwardCompatible8[] = {"fileName", "startPosX", "startPosY", "startPosZ", "startOrnX", "startOrnY", "startOrnZ", "startOrnW", NULL};

	int bodyUniqueId = -1;
	const char* urdfFileName = "";
	double globalScaling = -1;
	double startPosX = 0.0;
	double startPosY = 0.0;
	double startPosZ = 0.0;
	double startOrnX = 0.0;
	double startOrnY = 0.0;
	double startOrnZ = 0.0;
	double startOrnW = 1.0;
	int useMaximalCoordinates = -1;
	int useFixedBase = 0;

	int backwardsCompatibilityArgs = 0;
	b3PhysicsClientHandle sm = 0;
	if (PyArg_ParseTupleAndKeywords(args, keywds, "sddd", kwlistBackwardCompatible4, &urdfFileName, &startPosX,
									&startPosY, &startPosZ))
	{
		backwardsCompatibilityArgs = 1;
	}
	else
	{
		PyErr_Clear();
	}

	if (PyArg_ParseTupleAndKeywords(args, keywds, "sddddddd", kwlistBackwardCompatible8, &urdfFileName, &startPosX,
									&startPosY, &startPosZ, &startOrnX, &startOrnY, &startOrnZ, &startOrnW))
	{
		backwardsCompatibilityArgs = 1;
	}
	else
	{
		PyErr_Clear();
	}

	if (!backwardsCompatibilityArgs)
	{
		PyObject* basePosObj = 0;
		PyObject* baseOrnObj = 0;
		double basePos[3];
		double baseOrn[4];

		if (!PyArg_ParseTupleAndKeywords(args, keywds, "s|OOiiidi", kwlist, &urdfFileName, &basePosObj, &baseOrnObj, &useMaximalCoordinates, &useFixedBase, &flags, &globalScaling, &physicsClientId))
		{
			return NULL;
		}
		else
		{
			if (basePosObj)
			{
				if (!pybullet_internalSetVectord(basePosObj, basePos))
				{
					PyErr_SetString(SpamError, "Cannot convert basePosition.");
					return NULL;
				}
				startPosX = basePos[0];
				startPosY = basePos[1];
				startPosZ = basePos[2];
			}
			if (baseOrnObj)
			{
				if (!pybullet_internalSetVector4d(baseOrnObj, baseOrn))
				{
					PyErr_SetString(SpamError, "Cannot convert baseOrientation.");
					return NULL;
				}
				startOrnX = baseOrn[0];
				startOrnY = baseOrn[1];
				startOrnZ = baseOrn[2];
				startOrnW = baseOrn[3];
			}
		}
	}

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	if (strlen(urdfFileName))
	{
		// printf("(%f, %f, %f) (%f, %f, %f, %f)\n",
		// startPosX,startPosY,startPosZ,startOrnX, startOrnY,startOrnZ, startOrnW);

		b3SharedMemoryStatusHandle statusHandle;
		int statusType;
		b3SharedMemoryCommandHandle command =
			b3LoadUrdfCommandInit(sm, urdfFileName);

		b3LoadUrdfCommandSetFlags(command, flags);

		// setting the initial position, orientation and other arguments are
		// optional
		b3LoadUrdfCommandSetStartPosition(command, startPosX, startPosY, startPosZ);
		b3LoadUrdfCommandSetStartOrientation(command, startOrnX, startOrnY,
											 startOrnZ, startOrnW);
		if (useMaximalCoordinates >= 0)
		{
			b3LoadUrdfCommandSetUseMultiBody(command, useMaximalCoordinates == 0);
		}
		if (useFixedBase)
		{
			b3LoadUrdfCommandSetUseFixedBase(command, 1);
		}
		if (globalScaling > 0)
		{
			b3LoadUrdfCommandSetGlobalScaling(command, globalScaling);
		}
		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
		statusType = b3GetStatusType(statusHandle);
		if (statusType != CMD_URDF_LOADING_COMPLETED)
		{
			PyErr_SetString(SpamError, "Cannot load URDF file.");
			return NULL;
		}
		bodyUniqueId = b3GetStatusBodyIndex(statusHandle);
	}
	else
	{
		PyErr_SetString(SpamError,
						"Empty filename, method expects 1, 4 or 8 arguments.");
		return NULL;
	}
	return PyLong_FromLong(bodyUniqueId);
}

static PyObject* pybullet_loadSDF(PyObject* self, PyObject* args, PyObject* keywds)
{
	const char* sdfFileName = "";
	int numBodies = 0;
	int i;
	int bodyIndicesOut[MAX_SDF_BODIES];
	int useMaximalCoordinates = -1;
	PyObject* pylist = 0;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	b3SharedMemoryCommandHandle commandHandle;
	b3PhysicsClientHandle sm = 0;
	double globalScaling = -1;

	int physicsClientId = 0;
	static char* kwlist[] = {"sdfFileName", "useMaximalCoordinates", "globalScaling", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "s|idi", kwlist, &sdfFileName, &useMaximalCoordinates, &globalScaling, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	commandHandle = b3LoadSdfCommandInit(sm, sdfFileName);
	if (useMaximalCoordinates > 0)
	{
		b3LoadSdfCommandSetUseMultiBody(commandHandle, 0);
	}
	if (globalScaling > 0)
	{
		b3LoadSdfCommandSetUseGlobalScaling(commandHandle, globalScaling);
	}
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	if (statusType != CMD_SDF_LOADING_COMPLETED)
	{
		PyErr_SetString(SpamError, "Cannot load SDF file.");
		return NULL;
	}

	numBodies =
		b3GetStatusBodyIndices(statusHandle, bodyIndicesOut, MAX_SDF_BODIES);
	if (numBodies > MAX_SDF_BODIES)
	{
		char str[1024];
		sprintf(str, "SDF exceeds body capacity: %d > %d", numBodies, MAX_SDF_BODIES);
		PyErr_SetString(SpamError, str);
		return NULL;
	}

	pylist = PyTuple_New(numBodies);

	if (numBodies > 0 && numBodies <= MAX_SDF_BODIES)
	{
		for (i = 0; i < numBodies; i++)
		{
			PyTuple_SetItem(pylist, i, PyInt_FromLong(bodyIndicesOut[i]));
		}
	}
	return pylist;
}

#ifndef SKIP_SOFT_BODY_MULTI_BODY_DYNAMICS_WORLD




// Load a softbody from an obj file
static PyObject* pybullet_loadSoftBody(PyObject* self, PyObject* args, PyObject* keywds)
{
	int physicsClientId = 0;
	int flags = 0;

	static char* kwlist[] = {"fileName", "basePosition", "baseOrientation", "scale", "mass", "collisionMargin", "useMassSpring", "useBendingSprings", "useNeoHookean", "springElasticStiffness", "springDampingStiffness", "springDampingAllDirections", "springBendingStiffness", "NeoHookeanMu", "NeoHookeanLambda", "NeoHookeanDamping", "frictionCoeff", "useFaceContact", "useSelfCollision", "repulsionStiffness", "simFileName", "physicsClientId", NULL};

	int bodyUniqueId = -1;
	const char* fileName = "";
	double scale = -1;
	double mass = -1;
	double collisionMargin = -1;
	int useMassSpring = 0;
	int useBendingSprings = 0;
	int useNeoHookean = 0;
	double springElasticStiffness = 1;
	double springDampingStiffness = 0.1;
	int springDampingAllDirections = 0;
	double springBendingStiffness = 0.1;
	double NeoHookeanMu = 1;
	double NeoHookeanLambda = 1;
	double NeoHookeanDamping = 0.1;
	double frictionCoeff = 0;
	int useFaceContact = 0;
	int useSelfCollision = 0;
	double repulsionStiffness = 0.5;
	const char* simFileName = "";


	b3PhysicsClientHandle sm = 0;

	double startPos[3] = {0.0, 0.0, 0.0};
	double startOrn[4] = {0.0, 0.0, 0.0, 1.0};


	PyObject* basePosObj = 0;
	PyObject* baseOrnObj = 0;

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "s|OOdddiiiddidddddiidsi", kwlist, 
		&fileName, 
		&basePosObj, 
		&baseOrnObj, 
		&scale, 
		&mass, 
		&collisionMargin, 
		&useMassSpring, 
		&useBendingSprings, 
		&useNeoHookean, 
		&springElasticStiffness, 
		&springDampingStiffness, 
		&springDampingAllDirections, 
		&springBendingStiffness, 
		&NeoHookeanMu, 
		&NeoHookeanLambda, 
		&NeoHookeanDamping, 
		&frictionCoeff, 
		&useFaceContact, 
		&useSelfCollision, 
		&repulsionStiffness,
		&simFileName,
		&physicsClientId))
	{
		return NULL;
	}
	else
	{
		if (basePosObj)
		{
			if (!pybullet_internalSetVectord(basePosObj, startPos))
			{
				PyErr_SetString(SpamError, "Cannot convert basePosition.");
				return NULL;
			}
		}
		if (baseOrnObj)
		{
			if (!pybullet_internalSetVector4d(baseOrnObj, startOrn))
			{
				PyErr_SetString(SpamError, "Cannot convert baseOrientation.");
				return NULL;
			}
		}
	}

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	if (strlen(fileName))
	{
		b3SharedMemoryStatusHandle statusHandle;
		int statusType;
		b3SharedMemoryCommandHandle command =
			b3LoadSoftBodyCommandInit(sm, fileName);

		b3LoadSoftBodySetStartPosition(command, startPos[0], startPos[1], startPos[2]);
		b3LoadSoftBodySetStartOrientation(command, startOrn[0], startOrn[1], startOrn[2], startOrn[3]);
		if (strlen(simFileName))
		{
			b3LoadSoftBodyUpdateSimMesh(command, simFileName);
		}
		if (scale > 0)
		{
			b3LoadSoftBodySetScale(command, scale);
		}
		if (mass > 0)
		{
			b3LoadSoftBodySetMass(command, mass);
		}
		if (collisionMargin > 0)
		{
			b3LoadSoftBodySetCollisionMargin(command, collisionMargin);
		}
		if (useMassSpring)
		{
			b3LoadSoftBodyAddMassSpringForce(command, springElasticStiffness, springDampingStiffness);
			b3LoadSoftBodyUseBendingSprings(command, useBendingSprings, springBendingStiffness);
                        b3LoadSoftBodyUseAllDirectionDampingSprings(command, springDampingAllDirections);
		}
		if (useNeoHookean)
		{
			b3LoadSoftBodyAddNeoHookeanForce(command, NeoHookeanMu, NeoHookeanLambda, NeoHookeanDamping);
		}
		if (useSelfCollision)
		{
			b3LoadSoftBodySetSelfCollision(command, useSelfCollision);
		}
                if (repulsionStiffness > 0)
                {
                        b3LoadSoftBodySetRepulsionStiffness(command, repulsionStiffness);
                }
		b3LoadSoftBodySetFrictionCoefficient(command, frictionCoeff);
		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
		statusType = b3GetStatusType(statusHandle);
		if (statusType != CMD_LOAD_SOFT_BODY_COMPLETED)
		{
			PyErr_SetString(SpamError, "Cannot load soft body.");
			return NULL;
		}
		bodyUniqueId = b3GetStatusBodyIndex(statusHandle);
	}
	return PyLong_FromLong(bodyUniqueId);
}

static PyObject* pybullet_createSoftBodyAnchor(PyObject* self, PyObject* args, PyObject* keywds)
{
	b3SharedMemoryCommandHandle commandHandle;
	int softBodyUniqueId = -1;
	int nodeIndex  = -1;
	int bodyUniqueId = -1;
	int linkIndex = -1;
	PyObject* bodyFramePositionObj = 0;
	double bodyFramePosition[3] = {0, 0, 0};
	struct b3JointInfo jointInfo;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	static char* kwlist[] = {"softBodyBodyUniqueId", "nodeIndex",
							 "bodyUniqueId", "linkIndex", "bodyFramePosition",
							 "physicsClientId",
							 NULL};

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "ii|iiOi", kwlist, &softBodyUniqueId, &nodeIndex,
									 &bodyUniqueId, &linkIndex,&bodyFramePositionObj,&physicsClientId))
	{
		return NULL;
	}

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	pybullet_internalSetVectord(bodyFramePositionObj, bodyFramePosition);

	commandHandle = b3InitCreateSoftBodyAnchorConstraintCommand(sm, softBodyUniqueId, nodeIndex, bodyUniqueId, linkIndex, bodyFramePosition);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	if (statusType == CMD_USER_CONSTRAINT_COMPLETED)
	{
		int userConstraintUid = b3GetStatusUserConstraintUniqueId(statusHandle);
		PyObject* ob = PyLong_FromLong(userConstraintUid);
		return ob;
	}

	PyErr_SetString(SpamError, "createSoftBodyAnchor failed.");
	return NULL;
}


#endif

// Reset the simulation to remove all loaded objects
static PyObject* pybullet_resetSimulation(PyObject* self, PyObject* args, PyObject* keywds)
{
	int physicsClientId = 0;
	int flags = 0;
	static char* kwlist[] = {"flags", "physicsClientId", NULL};
	b3PhysicsClientHandle sm = 0;

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|ii", kwlist, &flags, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	{
		b3SharedMemoryCommandHandle commandHandle;
		b3SharedMemoryStatusHandle statusHandle;
		commandHandle = b3InitResetSimulationCommand(sm);
		b3InitResetSimulationSetFlags(commandHandle, flags);
		statusHandle = b3SubmitClientCommandAndWaitStatus(
			sm, commandHandle);
	}
	Py_INCREF(Py_None);
	return Py_None;
}

//this method is obsolete, use pybullet_setJointMotorControl2 instead
static PyObject* pybullet_setJointMotorControl(PyObject* self, PyObject* args)
{
	int size;
	int bodyUniqueId, jointIndex, controlMode;

	double targetPosition = 0.0;
	double targetVelocity = 0.0;
	double maxForce = 100000.0;
	double appliedForce = 0.0;
	double kp = 0.1;
	double kd = 1.0;
	int valid = 0;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm;
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	size = PySequence_Size(args);
	if (size == 4)
	{
		double targetValue = 0.0;
		// see switch statement below for convertsions dependent on controlMode
		if (!PyArg_ParseTuple(args, "iiid", &bodyUniqueId, &jointIndex, &controlMode,
							  &targetValue))
		{
			PyErr_SetString(SpamError, "Error parsing arguments");
			return NULL;
		}
		valid = 1;
		switch (controlMode)
		{
			case CONTROL_MODE_POSITION_VELOCITY_PD:
			{
				targetPosition = targetValue;
				break;
			}
			case CONTROL_MODE_VELOCITY:
			{
				targetVelocity = targetValue;
				break;
			}
			case CONTROL_MODE_TORQUE:
			{
				appliedForce = targetValue;
				break;
			}
			default:
			{
				valid = 0;
			}
		}
	}
	if (size == 5)
	{
		double targetValue = 0.0;
		// See switch statement for conversions
		if (!PyArg_ParseTuple(args, "iiidd", &bodyUniqueId, &jointIndex, &controlMode,
							  &targetValue, &maxForce))
		{
			PyErr_SetString(SpamError, "Error parsing arguments");
			return NULL;
		}
		valid = 1;

		switch (controlMode)
		{
			case CONTROL_MODE_POSITION_VELOCITY_PD:
			{
				targetPosition = targetValue;
				break;
			}
			case CONTROL_MODE_VELOCITY:
			{
				targetVelocity = targetValue;
				break;
			}
			case CONTROL_MODE_TORQUE:
			{
				valid = 0;
				break;
			}
			default:
			{
				valid = 0;
			}
		}
	}
	if (size == 6)
	{
		double gain = 0.0;
		double targetValue = 0.0;
		if (!PyArg_ParseTuple(args, "iiiddd", &bodyUniqueId, &jointIndex, &controlMode,
							  &targetValue, &maxForce, &gain))
		{
			PyErr_SetString(SpamError, "Error parsing arguments");
			return NULL;
		}
		valid = 1;

		switch (controlMode)
		{
			case CONTROL_MODE_POSITION_VELOCITY_PD:
			{
				targetPosition = targetValue;
				kp = gain;
				break;
			}
			case CONTROL_MODE_VELOCITY:
			{
				targetVelocity = targetValue;
				kd = gain;
				break;
			}
			case CONTROL_MODE_TORQUE:
			{
				valid = 0;
				break;
			}
			default:
			{
				valid = 0;
			}
		}
	}
	if (size == 8)
	{
		// only applicable for CONTROL_MODE_POSITION_VELOCITY_PD.
		if (!PyArg_ParseTuple(args, "iiiddddd", &bodyUniqueId, &jointIndex,
							  &controlMode, &targetPosition, &targetVelocity,
							  &maxForce, &kp, &kd))
		{
			PyErr_SetString(SpamError, "Error parsing arguments");
			return NULL;
		}
		valid = 1;
	}

	if (valid)
	{
		int numJoints;
		b3SharedMemoryCommandHandle commandHandle;
		b3SharedMemoryStatusHandle statusHandle;
		struct b3JointInfo info;

		numJoints = b3GetNumJoints(sm, bodyUniqueId);
		if ((jointIndex >= numJoints) || (jointIndex < 0))
		{
			PyErr_SetString(SpamError, "Joint index out-of-range.");
			return NULL;
		}

		if ((controlMode != CONTROL_MODE_VELOCITY) &&
			(controlMode != CONTROL_MODE_TORQUE) &&
			(controlMode != CONTROL_MODE_POSITION_VELOCITY_PD))
		{
			PyErr_SetString(SpamError, "Illegal control mode.");
			return NULL;
		}

		commandHandle = b3JointControlCommandInit2(sm, bodyUniqueId, controlMode);

		b3GetJointInfo(sm, bodyUniqueId, jointIndex, &info);

		switch (controlMode)
		{
			case CONTROL_MODE_VELOCITY:
			{
				b3JointControlSetDesiredVelocity(commandHandle, info.m_uIndex,
												 targetVelocity);
				b3JointControlSetKd(commandHandle, info.m_uIndex, kd);
				b3JointControlSetMaximumForce(commandHandle, info.m_uIndex, maxForce);
				break;
			}

			case CONTROL_MODE_TORQUE:
			{
				b3JointControlSetDesiredForceTorque(commandHandle, info.m_uIndex,
													appliedForce);
				break;
			}

			case CONTROL_MODE_POSITION_VELOCITY_PD:
			{
				b3JointControlSetDesiredPosition(commandHandle, info.m_qIndex,
												 targetPosition);
				b3JointControlSetKp(commandHandle, info.m_uIndex, kp);
				b3JointControlSetDesiredVelocity(commandHandle, info.m_uIndex,
												 targetVelocity);
				b3JointControlSetKd(commandHandle, info.m_uIndex, kd);
				b3JointControlSetMaximumForce(commandHandle, info.m_uIndex, maxForce);
				break;
			}
			default:
			{
			}
		};

		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);

		Py_INCREF(Py_None);
		return Py_None;
	}
	PyErr_SetString(SpamError, "Error parsing arguments in setJointControl.");
	return NULL;
}

static PyObject* pybullet_setJointMotorControlArray(PyObject* self, PyObject* args, PyObject* keywds)
{
	int bodyUniqueId, controlMode;
	PyObject* jointIndicesObj = 0;
	PyObject* targetPositionsObj = 0;
	PyObject* targetVelocitiesObj = 0;
	PyObject* forcesObj = 0;
	PyObject* kpsObj = 0;
	PyObject* kdsObj = 0;

	b3PhysicsClientHandle sm = 0;

	int physicsClientId = 0;
	static char* kwlist[] = {"bodyUniqueId", "jointIndices", "controlMode", "targetPositions", "targetVelocities", "forces", "positionGains", "velocityGains", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "iOi|OOOOOi", kwlist, &bodyUniqueId, &jointIndicesObj, &controlMode,
									 &targetPositionsObj, &targetVelocitiesObj, &forcesObj, &kpsObj, &kdsObj, &physicsClientId))
	{
		static char* kwlist2[] = {"bodyIndex", "jointIndices", "controlMode", "targetPositions", "targetVelocities", "forces", "positionGains", "velocityGains", "physicsClientId", NULL};
		PyErr_Clear();
		if (!PyArg_ParseTupleAndKeywords(args, keywds, "iOi|OOOOOi", kwlist2, &bodyUniqueId, &jointIndicesObj, &controlMode,
										 &targetPositionsObj, &targetVelocitiesObj, &forcesObj, &kpsObj, &kdsObj, &physicsClientId))
		{
			return NULL;
		}
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	{
		int numJoints;
		int i;
		b3SharedMemoryCommandHandle commandHandle;
		b3SharedMemoryStatusHandle statusHandle;
		struct b3JointInfo info;
		int numControlledDofs = 0;
		PyObject* jointIndicesSeq = 0;
		PyObject* targetVelocitiesSeq = 0;
		PyObject* targetPositionsSeq = 0;
		PyObject* forcesSeq = 0;
		PyObject* kpsSeq = 0;
		PyObject* kdsSeq = 0;

		numJoints = b3GetNumJoints(sm, bodyUniqueId);

		if ((controlMode != CONTROL_MODE_VELOCITY) &&
			(controlMode != CONTROL_MODE_TORQUE) &&
			(controlMode != CONTROL_MODE_POSITION_VELOCITY_PD) &&
			(controlMode != CONTROL_MODE_PD))
		{
			PyErr_SetString(SpamError, "Illegal control mode.");
			return NULL;
		}

		jointIndicesSeq = PySequence_Fast(jointIndicesObj, "expected a sequence of joint indices");

		if (jointIndicesSeq == 0)
		{
			PyErr_SetString(SpamError, "expected a sequence of joint indices");
			return NULL;
		}

		numControlledDofs = PySequence_Size(jointIndicesObj);
		if (numControlledDofs == 0)
		{
			Py_DECREF(jointIndicesSeq);
			Py_INCREF(Py_None);
			return Py_None;
		}

		{
			int i;
			for (i = 0; i < numControlledDofs; i++)
			{
				int jointIndex = pybullet_internalGetIntFromSequence(jointIndicesSeq, i);
				if ((jointIndex >= numJoints) || (jointIndex < 0))
				{
					Py_DECREF(jointIndicesSeq);
					PyErr_SetString(SpamError, "Joint index out-of-range.");
					return NULL;
				}
			}
		}

		if (targetVelocitiesObj)
		{
			int num = PySequence_Size(targetVelocitiesObj);
			if (num != numControlledDofs)
			{
				Py_DECREF(jointIndicesSeq);
				PyErr_SetString(SpamError, "number of target velocies should match the number of joint indices");
				return NULL;
			}
			targetVelocitiesSeq = PySequence_Fast(targetVelocitiesObj, "expected a sequence of target velocities");
		}

		if (targetPositionsObj)
		{
			int num = PySequence_Size(targetPositionsObj);
			if (num != numControlledDofs)
			{
				Py_DECREF(jointIndicesSeq);
				if (targetVelocitiesSeq)
				{
					Py_DECREF(targetVelocitiesSeq);
				}
				PyErr_SetString(SpamError, "number of target positions should match the number of joint indices");
				return NULL;
			}

			targetPositionsSeq = PySequence_Fast(targetPositionsObj, "expected a sequence of target positions");
		}

		if (forcesObj)
		{
			int num = PySequence_Size(forcesObj);
			if (num != numControlledDofs)
			{
				Py_DECREF(jointIndicesSeq);
				if (targetVelocitiesSeq)
				{
					Py_DECREF(targetVelocitiesSeq);
				}
				if (targetPositionsSeq)
				{
					Py_DECREF(targetPositionsSeq);
				}

				PyErr_SetString(SpamError, "number of forces should match the joint indices");
				return NULL;
			}

			forcesSeq = PySequence_Fast(forcesObj, "expected a sequence of forces");
		}

		if (kpsObj)
		{
			int num = PySequence_Size(kpsObj);
			if (num != numControlledDofs)
			{
				Py_DECREF(jointIndicesSeq);
				if (targetVelocitiesSeq)
				{
					Py_DECREF(targetVelocitiesSeq);
				}
				if (targetPositionsSeq)
				{
					Py_DECREF(targetPositionsSeq);
				}
				if (forcesSeq)
				{
					Py_DECREF(forcesSeq);
				}

				PyErr_SetString(SpamError, "number of kps should match the joint indices");
				return NULL;
			}

			kpsSeq = PySequence_Fast(kpsObj, "expected a sequence of kps");
		}

		if (kdsObj)
		{
			int num = PySequence_Size(kdsObj);
			if (num != numControlledDofs)
			{
				Py_DECREF(jointIndicesSeq);
				if (targetVelocitiesSeq)
				{
					Py_DECREF(targetVelocitiesSeq);
				}
				if (targetPositionsSeq)
				{
					Py_DECREF(targetPositionsSeq);
				}
				if (forcesSeq)
				{
					Py_DECREF(forcesSeq);
				}
				if (kpsSeq)
				{
					Py_DECREF(kpsSeq);
				}

				PyErr_SetString(SpamError, "number of kds should match the number of joint indices");
				return NULL;
			}

			kdsSeq = PySequence_Fast(kdsObj, "expected a sequence of kds");
		}

		commandHandle = b3JointControlCommandInit2(sm, bodyUniqueId, controlMode);

		for (i = 0; i < numControlledDofs; i++)
		{
			double targetVelocity = 0.0;
			double targetPosition = 0.0;
			double force = 100000.0;
			double kp = 0.1;
			double kd = 1.0;
			int jointIndex;

			if (targetVelocitiesSeq)
			{
				targetVelocity = pybullet_internalGetFloatFromSequence(targetVelocitiesSeq, i);
			}

			if (targetPositionsSeq)
			{
				targetPosition = pybullet_internalGetFloatFromSequence(targetPositionsSeq, i);
			}

			if (forcesSeq)
			{
				force = pybullet_internalGetFloatFromSequence(forcesSeq, i);
			}

			if (kpsSeq)
			{
				kp = pybullet_internalGetFloatFromSequence(kpsSeq, i);
			}

			if (kdsSeq)
			{
				kd = pybullet_internalGetFloatFromSequence(kdsSeq, i);
			}

			jointIndex = pybullet_internalGetFloatFromSequence(jointIndicesSeq, i);
			b3GetJointInfo(sm, bodyUniqueId, jointIndex, &info);

			switch (controlMode)
			{
				case CONTROL_MODE_VELOCITY:
				{
					b3JointControlSetDesiredVelocity(commandHandle, info.m_uIndex,
													 targetVelocity);
					b3JointControlSetKd(commandHandle, info.m_uIndex, kd);
					b3JointControlSetMaximumForce(commandHandle, info.m_uIndex, force);
					break;
				}

				case CONTROL_MODE_TORQUE:
				{
					b3JointControlSetDesiredForceTorque(commandHandle, info.m_uIndex,
														force);
					break;
				}

				default:
				{
					b3JointControlSetDesiredPosition(commandHandle, info.m_qIndex,
													 targetPosition);
					b3JointControlSetKp(commandHandle, info.m_uIndex, kp);
					b3JointControlSetDesiredVelocity(commandHandle, info.m_uIndex,
													 targetVelocity);
					b3JointControlSetKd(commandHandle, info.m_uIndex, kd);
					b3JointControlSetMaximumForce(commandHandle, info.m_uIndex, force);
					break;
				}
			};
		}

		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);

		if (targetVelocitiesSeq)
		{
			Py_DECREF(targetVelocitiesSeq);
		}
		if (targetPositionsSeq)
		{
			Py_DECREF(targetPositionsSeq);
		}
		if (forcesSeq)
		{
			Py_DECREF(forcesSeq);
		}
		if (kpsSeq)
		{
			Py_DECREF(kpsSeq);
		}

		if (kdsSeq)
		{
			Py_DECREF(kdsSeq);
		}

		Py_DECREF(jointIndicesSeq);
		Py_INCREF(Py_None);
		return Py_None;
	}
	//  PyErr_SetString(SpamError, "Error parsing arguments in setJointControl.");
	//  return NULL;
}



static PyObject* pybullet_setJointMotorControlMultiDofArray(PyObject* self, PyObject* args, PyObject* keywds)
{
	int bodyUniqueId, controlMode;


	PyObject* jointIndicesObj = 0;
	PyObject* targetPositionsObj = 0;
	PyObject* targetVelocitiesObj = 0;
	PyObject* forcesObj = 0;
	PyObject* kpsObj = 0;
	PyObject* kdsObj = 0;
	PyObject* maxVelocitiesObj = 0;

	b3PhysicsClientHandle sm = 0;

	int physicsClientId = 0;
	static char* kwlist[] = { "bodyUniqueId", "jointIndices", "controlMode", "targetPositions", "targetVelocities", "forces", "positionGains", "velocityGains", "maxVelocities", "physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "iOi|OOOOOOi", kwlist, &bodyUniqueId, &jointIndicesObj, &controlMode,
		&targetPositionsObj, &targetVelocitiesObj, &forcesObj, &kpsObj, &kdsObj, &maxVelocitiesObj, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	{
		b3SharedMemoryStatusHandle statusHandle;
		int numJoints = 0;
		int j = 0;
		int numControlledDofs = 0;
		int numKps = 0;
		int numKds = 0;
		int numTargetPositionObjs = 0;
		int numTargetVelocityobjs = 0;
		int numForceObj = 0;

		PyObject* jointIndicesSeq = 0;
		PyObject* targetPositionsSeq = 0;
		PyObject* targetVelocitiesSeq = 0;
		PyObject* forcesSeq = 0;
		PyObject* kpsSeq = 0;
		PyObject* kdsSeq = 0;

		b3SharedMemoryCommandHandle commandHandle;
		commandHandle = b3JointControlCommandInit2(sm, bodyUniqueId, controlMode);

		numJoints = b3GetNumJoints(sm, bodyUniqueId);

		if ((controlMode != CONTROL_MODE_TORQUE) &&
			(controlMode != CONTROL_MODE_PD) &&
			(controlMode != CONTROL_MODE_STABLE_PD) &&
			(controlMode != CONTROL_MODE_POSITION_VELOCITY_PD))
		{
			PyErr_SetString(SpamError, "Illegal control mode.");
			return NULL;
		}

		jointIndicesSeq = PySequence_Fast(jointIndicesObj, "expected a sequence of joint indices");

		if (jointIndicesSeq == 0)
		{
			PyErr_SetString(SpamError, "expected a sequence of joint indices");
			return NULL;
		}

		numControlledDofs = jointIndicesObj ? PySequence_Size(jointIndicesObj) : 0;
		numKps = kpsObj? PySequence_Size(kpsObj) : 0;
		numKds = kdsObj ? PySequence_Size(kdsObj):0;
		numTargetPositionObjs = targetPositionsObj?PySequence_Size(targetPositionsObj):0;
		numTargetVelocityobjs = targetVelocitiesObj?PySequence_Size(targetVelocitiesObj):0;
		numForceObj = forcesObj?PySequence_Size(forcesObj):0;

		if ((numControlledDofs == 0) ||
			((numKps>0) && (numControlledDofs != numKps)) ||
			((numKds>0) && (numControlledDofs != numKds)) ||
			((numTargetPositionObjs>0) && (numControlledDofs != numTargetPositionObjs)) ||
			((numTargetVelocityobjs>0) && (numControlledDofs != numTargetVelocityobjs)) ||
			((numForceObj>0) && (numControlledDofs != numForceObj))
			)
		{
			Py_DECREF(jointIndicesSeq);
			Py_INCREF(Py_None);
			return Py_None;
		}

		if (targetPositionsObj)
		{
			targetPositionsSeq = PySequence_Fast(targetPositionsObj, "expected a targetPositions sequence");
		}
		if (targetVelocitiesObj)
		{
			targetVelocitiesSeq = PySequence_Fast(targetVelocitiesObj, "expected a targetVelocities sequence");
		}
		if (forcesObj)
		{
			forcesSeq = PySequence_Fast(forcesObj, "expected a forces sequence");
		}
		if (kpsObj)
		{
			kpsSeq = PySequence_Fast(kpsObj, "expected a kps sequence");
		}
		if (kdsObj)
		{
			kdsSeq = PySequence_Fast(kdsObj, "expected a kds sequence");
		}
		


		for (j = 0; j < numControlledDofs; j++)
		{
			double targetPositionArray[4] = { 0, 0, 0, 1 };
			double targetVelocityArray[4] = { 0, 0, 0 , 0};
			double targetForceArray[4] = { 100000.0, 100000.0, 100000.0 ,0};
			int targetPositionSize = 0;
			int targetVelocitySize = 0;
			int targetForceSize = 0;
			PyObject* targetPositionObj = 0;
			PyObject* targetVelocityObj = 0;
			PyObject* targetForceObj = 0;

			double kp = 0.1;
			double kd = 1.0;
			double maxVelocity = -1;
			int numTargetPositions = -1;
			int jointIndex = pybullet_internalGetIntFromSequence(jointIndicesSeq, j); 
			if ((jointIndex >= numJoints) || (jointIndex < 0))
			{
				Py_DECREF(jointIndicesSeq);
				PyErr_SetString(SpamError, "Joint index out-of-range.");
				return NULL;
			}

			if (numTargetPositionObjs > 0)
			{
				targetPositionObj = PyList_GET_ITEM(targetPositionsSeq, j);
			}
			if (numTargetVelocityobjs > 0)
			{
				targetVelocityObj = PyList_GET_ITEM(targetVelocitiesSeq, j);
			}
			if (numForceObj > 0)
			{
				targetForceObj = PyList_GET_ITEM(forcesSeq, j);
			}
			if (numKps > 0)
			{
				kp = pybullet_internalGetFloatFromSequence(kpsSeq, j);
			}
			if (numKds>0)
			{
				kd = pybullet_internalGetFloatFromSequence(kdsSeq, j);
			}
			
			if (targetPositionObj)
			{
				PyObject* targetPositionSeq = 0;
				int i = 0;
				targetPositionSeq = PySequence_Fast(targetPositionObj, "expected a targetPosition sequence");
				targetPositionSize = PySequence_Size(targetPositionObj);

				if (targetPositionSize < 0)
				{
					targetPositionSize = 0;
				}
				if (targetPositionSize > 4)
				{
					targetPositionSize = 4;
				}
				if (targetPositionSeq)
				{
					for (i = 0; i < targetPositionSize; i++)
					{
						targetPositionArray[i] = pybullet_internalGetFloatFromSequence(targetPositionSeq, i);
					}
					Py_DECREF(targetPositionSeq);
					targetPositionSeq = 0;
				}
			}

			
			if (targetVelocityObj)
			{
				int i = 0;
				PyObject* targetVelocitySeq = 0;
				targetVelocitySeq = PySequence_Fast(targetVelocityObj, "expected a targetVelocity sequence");
				targetVelocitySize = PySequence_Size(targetVelocityObj);

				if (targetVelocitySize < 0)
				{
					targetVelocitySize = 0;
				}
				if (targetVelocitySize > 3)
				{
					targetVelocitySize = 3;
				}
				if (targetVelocitySeq)
				{
					for (i = 0; i < targetVelocitySize; i++)
					{
						targetVelocityArray[i] = pybullet_internalGetFloatFromSequence(targetVelocitySeq, i);
					}
					Py_DECREF(targetVelocitySeq);
					targetVelocitySeq = 0;
				}
			}

			

			if (targetForceObj)
			{
				int i = 0;
				PyObject* targetForceSeq = 0;
				targetForceSeq = PySequence_Fast(targetForceObj, "expected a force sequence");
				targetForceSize = PySequence_Size(targetForceObj);

				if (targetForceSize < 0)
				{
					targetForceSize = 0;
				}
				if (targetForceSize > 3)
				{
					targetForceSize = 3;
				}
				if (targetForceSeq)
				{
					for (i = 0; i < targetForceSize; i++)
					{
						targetForceArray[i] = pybullet_internalGetFloatFromSequence(targetForceSeq, i);
					}
					Py_DECREF(targetForceSeq);
					targetForceSeq = 0;
				}
			}

			//if (targetPositionSize == 0 && targetVelocitySize == 0)
			//{

			{
				
				struct b3JointInfo info;


				b3GetJointInfo(sm, bodyUniqueId, jointIndex, &info);

				switch (controlMode)
				{
				case CONTROL_MODE_TORQUE:
				{
					if (info.m_uSize == targetForceSize)
					{
						b3JointControlSetDesiredForceTorqueMultiDof(commandHandle, info.m_uIndex,
							targetForceArray, targetForceSize);
					}
					break;
				}
				case CONTROL_MODE_STABLE_PD:
				case CONTROL_MODE_POSITION_VELOCITY_PD:
				case CONTROL_MODE_PD:
				{
					//make sure size == info.m_qSize

					if (maxVelocity > 0)
					{
						b3JointControlSetMaximumVelocity(commandHandle, info.m_uIndex, maxVelocity);
					}

					if (info.m_qSize == targetPositionSize)
					{
						b3JointControlSetDesiredPositionMultiDof(commandHandle, info.m_qIndex,
							targetPositionArray, targetPositionSize);
					}
					else
					{
						//printf("Warning: targetPosition array size doesn't match joint position size  (got %d, expected %d).",targetPositionSize, info.m_qSize);
					}

					if (controlMode == CONTROL_MODE_STABLE_PD)
					{
						if (targetVelocitySize == 0)
						{
							targetVelocitySize = info.m_uSize;
							targetVelocityArray[0] = 0;
							targetVelocityArray[1] = 0;
							targetVelocityArray[2] = 0;
							targetVelocityArray[3] = 0;
						}
						if (info.m_uSize == 3)
						{
							b3JointControlSetDesiredVelocityMultiDof(commandHandle, info.m_qIndex,
								targetVelocityArray, targetVelocitySize + 1);
						}
						else
						{
							b3JointControlSetDesiredVelocityMultiDof(commandHandle, info.m_qIndex,
								targetVelocityArray, targetVelocitySize);
						}
					}
					else
					{
						if (info.m_uSize == targetVelocitySize)
						{
							b3JointControlSetDesiredVelocityMultiDof(commandHandle, info.m_uIndex,
								targetVelocityArray, targetVelocitySize);
						}
					}
					
				

					if (controlMode == CONTROL_MODE_STABLE_PD)
					{
						if (info.m_uSize == 3)
						{
							b3JointControlSetKp(commandHandle, info.m_qIndex + 0, kp);
							b3JointControlSetKp(commandHandle, info.m_qIndex + 1, kp);
							b3JointControlSetKp(commandHandle, info.m_qIndex + 2, kp);
							b3JointControlSetKp(commandHandle, info.m_qIndex + 3, kp);

							b3JointControlSetKd(commandHandle, info.m_qIndex + 0, kd);
							b3JointControlSetKd(commandHandle, info.m_qIndex + 1, kd);
							b3JointControlSetKd(commandHandle, info.m_qIndex + 2, kd);
							b3JointControlSetKd(commandHandle, info.m_qIndex + 3, kd);

							b3JointControlSetDesiredForceTorqueMultiDof(commandHandle, info.m_qIndex,
								targetForceArray, targetForceSize+1);
						}
						else
						{
							b3JointControlSetKp(commandHandle, info.m_qIndex, kp);
							b3JointControlSetKd(commandHandle, info.m_qIndex, kd);
							b3JointControlSetDesiredForceTorqueMultiDof(commandHandle, info.m_qIndex,
								targetForceArray, targetForceSize);
						}
					}
					else
					{
						b3JointControlSetKp(commandHandle, info.m_uIndex, kp);
						b3JointControlSetKd(commandHandle, info.m_uIndex, kd);
						if (info.m_uSize == targetForceSize || targetForceSize == 1)
						{
							b3JointControlSetDesiredForceTorqueMultiDof(commandHandle, info.m_uIndex,
								targetForceArray, targetForceSize);
						}
					}
					
					break;
				}
				default:
				{
				}
				};
			}
		}
		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);

		if (jointIndicesSeq)
		{
			Py_DECREF(jointIndicesSeq);
		}
		if (targetPositionsSeq)
		{
			Py_DECREF(targetPositionsSeq);
		}
		if (targetVelocitiesSeq)
		{
			Py_DECREF(targetVelocitiesSeq);
		}
		if (forcesSeq)
		{
			Py_DECREF(forcesSeq);
		}
		if (kpsSeq)
		{
			Py_DECREF(kpsSeq);
		}
		if (kdsSeq)
		{
			Py_DECREF(kdsSeq);
		}

		Py_INCREF(Py_None);
		return Py_None;
	}
	//  PyErr_SetString(SpamError, "Error parsing arguments in setJointControl.");
	//  return NULL;
}


static PyObject* pybullet_setJointMotorControlMultiDof(PyObject* self, PyObject* args, PyObject* keywds)
{
	int bodyUniqueId, jointIndex, controlMode;

	double targetPositionArray[4] = {0, 0, 0, 1};
	double targetVelocityArray[3] = {0, 0, 0};
	double targetForceArray[3] = {100000.0, 100000.0, 100000.0};
	int targetPositionSize = 0;
	int targetVelocitySize = 0;
	int targetForceSize = 0;
	PyObject* targetPositionObj = 0;
	PyObject* targetVelocityObj = 0;
	PyObject* targetForceObj = 0;

	double kp = 0.1;
	double kd = 1.0;
	double maxVelocity = -1;
	b3PhysicsClientHandle sm = 0;

	int physicsClientId = 0;
	static char* kwlist[] = {"bodyUniqueId", "jointIndex", "controlMode", "targetPosition", "targetVelocity", "force", "positionGain", "velocityGain", "maxVelocity", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "iii|OOOdddi", kwlist, &bodyUniqueId, &jointIndex, &controlMode,
									 &targetPositionObj, &targetVelocityObj, &targetForceObj, &kp, &kd, &maxVelocity, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	if (targetPositionObj)
	{
		PyObject* targetPositionSeq = 0;
		int i = 0;
		targetPositionSeq = PySequence_Fast(targetPositionObj, "expected a targetPosition sequence");
		targetPositionSize = PySequence_Size(targetPositionObj);

		if (targetPositionSize < 0)
		{
			targetPositionSize = 0;
		}
		if (targetPositionSize > 4)
		{
			targetPositionSize = 4;
		}
		if (targetPositionSeq)
		{
			for (i = 0; i < targetPositionSize; i++)
			{
				targetPositionArray[i] = pybullet_internalGetFloatFromSequence(targetPositionSeq, i);
			}
			Py_DECREF(targetPositionSeq);
		}
	}

	if (targetVelocityObj)
	{
		int i = 0;
		PyObject* targetVelocitySeq = 0;
		targetVelocitySeq = PySequence_Fast(targetVelocityObj, "expected a targetVelocity sequence");
		targetVelocitySize = PySequence_Size(targetVelocityObj);

		if (targetVelocitySize < 0)
		{
			targetVelocitySize = 0;
		}
		if (targetVelocitySize > 3)
		{
			targetVelocitySize = 3;
		}
		if (targetVelocitySeq)
		{
			for (i = 0; i < targetVelocitySize; i++)
			{
				targetVelocityArray[i] = pybullet_internalGetFloatFromSequence(targetVelocitySeq, i);
			}
			Py_DECREF(targetVelocitySeq);
		}
	}

	if (targetForceObj)
	{
		int i = 0;
		PyObject* targetForceSeq = 0;
		targetForceSeq = PySequence_Fast(targetForceObj, "expected a force sequence");
		targetForceSize = PySequence_Size(targetForceObj);

		if (targetForceSize < 0)
		{
			targetForceSize = 0;
		}
		if (targetForceSize > 3)
		{
			targetForceSize = 3;
		}
		if (targetForceSeq)
		{
			for (i = 0; i < targetForceSize; i++)
			{
				targetForceArray[i] = pybullet_internalGetFloatFromSequence(targetForceSeq, i);
			}
			Py_DECREF(targetForceSeq);
		}
	}

	//if (targetPositionSize == 0 && targetVelocitySize == 0)
	//{

	{
		int numJoints;
		b3SharedMemoryCommandHandle commandHandle;
		b3SharedMemoryStatusHandle statusHandle;
		struct b3JointInfo info;

		numJoints = b3GetNumJoints(sm, bodyUniqueId);
		if ((jointIndex >= numJoints) || (jointIndex < 0))
		{
			PyErr_SetString(SpamError, "Joint index out-of-range.");
			return NULL;
		}

		if (  //(controlMode != CONTROL_MODE_VELOCITY)&&
			(controlMode != CONTROL_MODE_TORQUE) &&
			(controlMode != CONTROL_MODE_POSITION_VELOCITY_PD)  //&&
			//(controlMode != CONTROL_MODE_PD)
		)
		{
			PyErr_SetString(SpamError, "Illegal control mode.");
			return NULL;
		}

		commandHandle = b3JointControlCommandInit2(sm, bodyUniqueId, controlMode);

		b3GetJointInfo(sm, bodyUniqueId, jointIndex, &info);

		switch (controlMode)
		{
#if 0
		case CONTROL_MODE_VELOCITY:
		{
			b3JointControlSetDesiredVelocity(commandHandle, info.m_uIndex,
				targetVelocity);
			b3JointControlSetKd(commandHandle, info.m_uIndex, kd);
			b3JointControlSetMaximumForce(commandHandle, info.m_uIndex, force);
			break;
		}

#endif
			case CONTROL_MODE_TORQUE:
			{
				if (info.m_uSize == targetForceSize)
				{
					b3JointControlSetDesiredForceTorqueMultiDof(commandHandle, info.m_uIndex,
																targetForceArray, targetForceSize);
				}
				break;
			}
			case CONTROL_MODE_POSITION_VELOCITY_PD:
			case CONTROL_MODE_PD:
			{
				//make sure size == info.m_qSize

				if (maxVelocity > 0)
				{
					b3JointControlSetMaximumVelocity(commandHandle, info.m_uIndex, maxVelocity);
				}

				if (info.m_qSize == targetPositionSize)
				{
					b3JointControlSetDesiredPositionMultiDof(commandHandle, info.m_qIndex,
															 targetPositionArray, targetPositionSize);
				}
				else
				{
					//printf("Warning: targetPosition array size doesn't match joint position size  (got %d, expected %d).",targetPositionSize, info.m_qSize);
				}

				b3JointControlSetKp(commandHandle, info.m_uIndex, kp);
				if (info.m_uSize == targetVelocitySize)
				{
					b3JointControlSetDesiredVelocityMultiDof(commandHandle, info.m_uIndex,
															 targetVelocityArray, targetVelocitySize);
				}
				else
				{
					//printf("Warning: targetVelocity array size doesn't match joint dimentions (got %d, expected %d).", targetVelocitySize, info.m_uSize);
				}
				b3JointControlSetKd(commandHandle, info.m_uIndex, kd);
				if (info.m_uSize == targetForceSize || targetForceSize == 1)
				{
					b3JointControlSetDesiredForceTorqueMultiDof(commandHandle, info.m_uIndex,
																targetForceArray, targetForceSize);
				}
				break;
			}
			default:
			{
			}
		};

		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);

		Py_INCREF(Py_None);
		return Py_None;
	}
	//  PyErr_SetString(SpamError, "Error parsing arguments in setJointControl.");
	//  return NULL;
}


static PyObject* pybullet_setJointMotorControl2(PyObject* self, PyObject* args, PyObject* keywds)
{
	int bodyUniqueId, jointIndex, controlMode;

	double targetPosition = 0.0;
	double targetVelocity = 0.0;
	double force = 100000.0;
	double kp = 0.1;
	double kd = 1.0;
	double maxVelocity = -1;
	b3PhysicsClientHandle sm = 0;

	int physicsClientId = 0;
	static char* kwlist[] = {"bodyUniqueId", "jointIndex", "controlMode", "targetPosition", "targetVelocity", "force", "positionGain", "velocityGain", "maxVelocity", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "iii|ddddddi", kwlist, &bodyUniqueId, &jointIndex, &controlMode,
									 &targetPosition, &targetVelocity, &force, &kp, &kd, &maxVelocity, &physicsClientId))
	{
		//backward compatibility, bodyIndex -> bodyUniqueId, don't need to update this function: people have to migrate to bodyUniqueId
		static char* kwlist2[] = {"bodyIndex", "jointIndex", "controlMode", "targetPosition", "targetVelocity", "force", "positionGain", "velocityGain", "maxVelocity", "physicsClientId", NULL};
		PyErr_Clear();
		if (!PyArg_ParseTupleAndKeywords(args, keywds, "iii|ddddddi", kwlist2, &bodyUniqueId, &jointIndex, &controlMode,
										 &targetPosition, &targetVelocity, &force, &kp, &kd, &maxVelocity, &physicsClientId))
		{
			return NULL;
		}
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	{
		int numJoints;
		b3SharedMemoryCommandHandle commandHandle;
		b3SharedMemoryStatusHandle statusHandle;
		struct b3JointInfo info;

		numJoints = b3GetNumJoints(sm, bodyUniqueId);
		if ((jointIndex >= numJoints) || (jointIndex < 0))
		{
			PyErr_SetString(SpamError, "Joint index out-of-range.");
			return NULL;
		}

		if ((controlMode != CONTROL_MODE_VELOCITY) &&
			(controlMode != CONTROL_MODE_TORQUE) &&
			(controlMode != CONTROL_MODE_POSITION_VELOCITY_PD) &&
			(controlMode != CONTROL_MODE_PD))
		{
			PyErr_SetString(SpamError, "Illegal control mode.");
			return NULL;
		}

		commandHandle = b3JointControlCommandInit2(sm, bodyUniqueId, controlMode);

		b3GetJointInfo(sm, bodyUniqueId, jointIndex, &info);

		switch (controlMode)
		{
			case CONTROL_MODE_VELOCITY:
			{
				b3JointControlSetDesiredVelocity(commandHandle, info.m_uIndex,
												 targetVelocity);
				b3JointControlSetKd(commandHandle, info.m_uIndex, kd);
				b3JointControlSetMaximumForce(commandHandle, info.m_uIndex, force);
				break;
			}

			case CONTROL_MODE_TORQUE:
			{
				b3JointControlSetDesiredForceTorque(commandHandle, info.m_uIndex,
													force);
				break;
			}

			case CONTROL_MODE_POSITION_VELOCITY_PD:
			case CONTROL_MODE_PD:
			{
				if (maxVelocity > 0)
				{
					b3JointControlSetMaximumVelocity(commandHandle, info.m_uIndex, maxVelocity);
				}
				b3JointControlSetDesiredPosition(commandHandle, info.m_qIndex,
												 targetPosition);
				b3JointControlSetKp(commandHandle, info.m_uIndex, kp);
				b3JointControlSetDesiredVelocity(commandHandle, info.m_uIndex,
												 targetVelocity);
				b3JointControlSetKd(commandHandle, info.m_uIndex, kd);
				b3JointControlSetMaximumForce(commandHandle, info.m_uIndex, force);
				break;
			}
			default:
			{
			}
		};

		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);

		Py_INCREF(Py_None);
		return Py_None;
	}
	//  PyErr_SetString(SpamError, "Error parsing arguments in setJointControl.");
	//  return NULL;
}

static PyObject* pybullet_setRealTimeSimulation(PyObject* self,
												PyObject* args,
												PyObject* keywds)
{
	int enableRealTimeSimulation = 0;
	int ret;
	b3PhysicsClientHandle sm = 0;

	int physicsClientId = 0;
	static char* kwlist[] = {"enableRealTimeSimulation", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist, &enableRealTimeSimulation, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	{
		b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(sm);
		b3SharedMemoryStatusHandle statusHandle;

		ret =
			b3PhysicsParamSetRealTimeSimulation(command, enableRealTimeSimulation);

		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
		// ASSERT_EQ(b3GetStatusType(statusHandle), CMD_CLIENT_COMMAND_COMPLETED);
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_setInternalSimFlags(PyObject* self,
											  PyObject* args, PyObject* keywds)
{
	int flags = 0;
	int ret;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;

	static char* kwlist[] = {"flags", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist, &flags, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	{
		b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(sm);
		b3SharedMemoryStatusHandle statusHandle;

		ret =
			b3PhysicsParamSetInternalSimFlags(command, flags);

		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
		// ASSERT_EQ(b3GetStatusType(statusHandle), CMD_CLIENT_COMMAND_COMPLETED);
	}

	Py_INCREF(Py_None);
	return Py_None;
}

// Set the gravity of the world with (x, y, z) arguments
static PyObject* pybullet_setGravity(PyObject* self, PyObject* args, PyObject* keywds)
{
	{
		double gravX = 0.0;
		double gravY = 0.0;
		double gravZ = -10.0;
		int ret;
		b3PhysicsClientHandle sm = 0;
		b3SharedMemoryCommandHandle command;
		b3SharedMemoryStatusHandle statusHandle;

		int physicsClientId = 0;
		static char* kwlist[] = {"gravX", "gravY", "gravZ", "physicsClientId", NULL};
		if (!PyArg_ParseTupleAndKeywords(args, keywds, "ddd|i", kwlist, &gravX, &gravY, &gravZ, &physicsClientId))
		{
			return NULL;
		}
		sm = getPhysicsClient(physicsClientId);
		if (sm == 0)
		{
			PyErr_SetString(SpamError, "Not connected to physics server.");
			return NULL;
		}

		command = b3InitPhysicsParamCommand(sm);

		ret = b3PhysicsParamSetGravity(command, gravX, gravY, gravZ);
		// ret = b3PhysicsParamSetTimeStep(command,  timeStep);
		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
		// ASSERT_EQ(b3GetStatusType(statusHandle), CMD_CLIENT_COMMAND_COMPLETED);
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_setTimeStep(PyObject* self, PyObject* args, PyObject* keywds)
{
	double timeStep = 0.001;
	int ret;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;

	static char* kwlist[] = {"timeStep", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "d|i", kwlist, &timeStep, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
	{
		b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(sm);
		b3SharedMemoryStatusHandle statusHandle;

		ret = b3PhysicsParamSetTimeStep(command, timeStep);
		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);

		Py_INCREF(Py_None);
		return Py_None;
	}
}

static PyObject*
pybullet_setDefaultContactERP(PyObject* self, PyObject* args, PyObject* keywds)
{
	double defaultContactERP = 0.005;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;

	static char* kwlist[] = {"defaultContactERP", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "d|i", kwlist, &defaultContactERP, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
	{
		int ret;

		b3SharedMemoryStatusHandle statusHandle;

		b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(sm);
		ret = b3PhysicsParamSetDefaultContactERP(command, defaultContactERP);

		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static int pybullet_internalGetBaseVelocity(
	int bodyUniqueId, double baseLinearVelocity[3], double baseAngularVelocity[3], b3PhysicsClientHandle sm)
{
	baseLinearVelocity[0] = 0.;
	baseLinearVelocity[1] = 0.;
	baseLinearVelocity[2] = 0.;

	baseAngularVelocity[0] = 0.;
	baseAngularVelocity[1] = 0.;
	baseAngularVelocity[2] = 0.;

	if (0 == sm)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return 0;
	}

	{
		{
			b3SharedMemoryCommandHandle cmd_handle =
				b3RequestActualStateCommandInit(sm, bodyUniqueId);
			b3SharedMemoryStatusHandle status_handle =
				b3SubmitClientCommandAndWaitStatus(sm, cmd_handle);

			const int status_type = b3GetStatusType(status_handle);
			const double* actualStateQdot;
			// const double* jointReactionForces[];

			if (status_type != CMD_ACTUAL_STATE_UPDATE_COMPLETED)
			{
				PyErr_SetString(SpamError, "getBaseVelocity failed.");
				return 0;
			}

			b3GetStatusActualState(
				status_handle, 0 /* body_unique_id */,
				0 /* num_degree_of_freedom_q */, 0 /* num_degree_of_freedom_u */,
				0 /*root_local_inertial_frame*/, 0,
				&actualStateQdot, 0 /* joint_reaction_forces */);

			// printf("joint reaction forces=");
			// for (i=0; i < (sizeof(jointReactionForces)/sizeof(double)); i++) {
			//   printf("%f ", jointReactionForces[i]);
			// }
			// now, position x,y,z = actualStateQ[0],actualStateQ[1],actualStateQ[2]
			// and orientation x,y,z,w =
			// actualStateQ[3],actualStateQ[4],actualStateQ[5],actualStateQ[6]
			baseLinearVelocity[0] = actualStateQdot[0];
			baseLinearVelocity[1] = actualStateQdot[1];
			baseLinearVelocity[2] = actualStateQdot[2];

			baseAngularVelocity[0] = actualStateQdot[3];
			baseAngularVelocity[1] = actualStateQdot[4];
			baseAngularVelocity[2] = actualStateQdot[5];
		}
	}
	return 1;
}

// Internal function used to get the base position and orientation
// Orientation is returned in quaternions
static int pybullet_internalGetBasePositionAndOrientation(
	int bodyUniqueId, double basePosition[3], double baseOrientation[4], b3PhysicsClientHandle sm)
{
	basePosition[0] = 0.;
	basePosition[1] = 0.;
	basePosition[2] = 0.;

	baseOrientation[0] = 0.;
	baseOrientation[1] = 0.;
	baseOrientation[2] = 0.;
	baseOrientation[3] = 1.;

	if (0 == sm)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return 0;
	}

	{
		{
			b3SharedMemoryCommandHandle cmd_handle =
				b3RequestActualStateCommandInit(sm, bodyUniqueId);
			b3SharedMemoryStatusHandle status_handle =
				b3SubmitClientCommandAndWaitStatus(sm, cmd_handle);

			const int status_type = b3GetStatusType(status_handle);
			const double* actualStateQ;
			// const double* jointReactionForces[];

			if (status_type != CMD_ACTUAL_STATE_UPDATE_COMPLETED)
			{
				PyErr_SetString(SpamError, "getBasePositionAndOrientation failed.");
				return 0;
			}

			b3GetStatusActualState(
				status_handle, 0 /* body_unique_id */,
				0 /* num_degree_of_freedom_q */, 0 /* num_degree_of_freedom_u */,
				0 /*root_local_inertial_frame*/, &actualStateQ,
				0 /* actual_state_q_dot */, 0 /* joint_reaction_forces */);

			// printf("joint reaction forces=");
			// for (i=0; i < (sizeof(jointReactionForces)/sizeof(double)); i++) {
			//   printf("%f ", jointReactionForces[i]);
			// }
			// now, position x,y,z = actualStateQ[0],actualStateQ[1],actualStateQ[2]
			// and orientation x,y,z,w =
			// actualStateQ[3],actualStateQ[4],actualStateQ[5],actualStateQ[6]
			basePosition[0] = actualStateQ[0];
			basePosition[1] = actualStateQ[1];
			basePosition[2] = actualStateQ[2];

			baseOrientation[0] = actualStateQ[3];
			baseOrientation[1] = actualStateQ[4];
			baseOrientation[2] = actualStateQ[5];
			baseOrientation[3] = actualStateQ[6];
		}
	}
	return 1;
}

static PyObject* pybullet_getAABB(PyObject* self, PyObject* args, PyObject* keywds)
{
	int bodyUniqueId = -1;
	int linkIndex = -1;

	b3PhysicsClientHandle sm = 0;
	int physicsClientId = 0;
	static char* kwlist[] = {"bodyUniqueId", "linkIndex", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|ii", kwlist, &bodyUniqueId, &linkIndex, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	{
		int status_type = 0;
		b3SharedMemoryCommandHandle cmd_handle;
		b3SharedMemoryStatusHandle status_handle;

		if (bodyUniqueId < 0)
		{
			PyErr_SetString(SpamError, "getAABB failed; invalid bodyUniqueId");
			return NULL;
		}

		if (linkIndex < -1)
		{
			PyErr_SetString(SpamError, "getAABB failed; invalid linkIndex");
			return NULL;
		}

		cmd_handle =
			b3RequestCollisionInfoCommandInit(sm, bodyUniqueId);
		status_handle =
			b3SubmitClientCommandAndWaitStatus(sm, cmd_handle);

		status_type = b3GetStatusType(status_handle);
		if (status_type != CMD_REQUEST_COLLISION_INFO_COMPLETED)
		{
			PyErr_SetString(SpamError, "getAABB failed.");
			return NULL;
		}

		{
			PyObject* pyListAabb = 0;
			PyObject* pyListAabbMin = 0;
			PyObject* pyListAabbMax = 0;
			double aabbMin[3];
			double aabbMax[3];
			int i = 0;
			if (b3GetStatusAABB(status_handle, linkIndex, aabbMin, aabbMax))
			{
				pyListAabb = PyTuple_New(2);
				pyListAabbMin = PyTuple_New(3);
				pyListAabbMax = PyTuple_New(3);

				for (i = 0; i < 3; i++)
				{
					PyTuple_SetItem(pyListAabbMin, i, PyFloat_FromDouble(aabbMin[i]));
					PyTuple_SetItem(pyListAabbMax, i, PyFloat_FromDouble(aabbMax[i]));
				}

				PyTuple_SetItem(pyListAabb, 0, pyListAabbMin);
				PyTuple_SetItem(pyListAabb, 1, pyListAabbMax);

				//PyFloat_FromDouble(basePosition[i]);

				return pyListAabb;
			}
		}
	}

	PyErr_SetString(SpamError, "getAABB failed.");
	return NULL;
}

// Get the positions (x,y,z) and orientation (x,y,z,w) in quaternion
// values for the base link of your object
// Object is retrieved based on body index, which is the order
// the object was loaded into the simulation (0-based)
static PyObject* pybullet_getBasePositionAndOrientation(PyObject* self,
														PyObject* args, PyObject* keywds)
{
	int bodyUniqueId = -1;
	double basePosition[3];
	double baseOrientation[4];
	PyObject* pylistPos;
	PyObject* pylistOrientation;
	b3PhysicsClientHandle sm = 0;

	int physicsClientId = 0;
	static char* kwlist[] = {"bodyUniqueId", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist, &bodyUniqueId, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	if (0 == pybullet_internalGetBasePositionAndOrientation(
				 bodyUniqueId, basePosition, baseOrientation, sm))
	{
		PyErr_SetString(SpamError,
						"GetBasePositionAndOrientation failed.");
		return NULL;
	}

	{
		PyObject* item;
		int i;
		int num = 3;
		pylistPos = PyTuple_New(num);
		for (i = 0; i < num; i++)
		{
			item = PyFloat_FromDouble(basePosition[i]);
			PyTuple_SetItem(pylistPos, i, item);
		}
	}

	{
		PyObject* item;
		int i;
		int num = 4;
		pylistOrientation = PyTuple_New(num);
		for (i = 0; i < num; i++)
		{
			item = PyFloat_FromDouble(baseOrientation[i]);
			PyTuple_SetItem(pylistOrientation, i, item);
		}
	}

	{
		PyObject* pylist;
		pylist = PyTuple_New(2);
		PyTuple_SetItem(pylist, 0, pylistPos);
		PyTuple_SetItem(pylist, 1, pylistOrientation);
		return pylist;
	}
}

static PyObject* pybullet_getBaseVelocity(PyObject* self,
										  PyObject* args, PyObject* keywds)
{
	int bodyUniqueId = -1;
	double baseLinearVelocity[3];
	double baseAngularVelocity[3];
	PyObject* pylistLinVel = 0;
	PyObject* pylistAngVel = 0;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;

	static char* kwlist[] = {"bodyUniqueId", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist, &bodyUniqueId, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	if (0 == pybullet_internalGetBaseVelocity(
				 bodyUniqueId, baseLinearVelocity, baseAngularVelocity, sm))
	{
		PyErr_SetString(SpamError,
						"getBaseVelocity failed.");
		return NULL;
	}

	{
		PyObject* item;
		int i;
		int num = 3;
		pylistLinVel = PyTuple_New(num);
		for (i = 0; i < num; i++)
		{
			item = PyFloat_FromDouble(baseLinearVelocity[i]);
			PyTuple_SetItem(pylistLinVel, i, item);
		}
	}

	{
		PyObject* item;
		int i;
		int num = 3;
		pylistAngVel = PyTuple_New(num);
		for (i = 0; i < num; i++)
		{
			item = PyFloat_FromDouble(baseAngularVelocity[i]);
			PyTuple_SetItem(pylistAngVel, i, item);
		}
	}

	{
		PyObject* pylist;
		pylist = PyTuple_New(2);
		PyTuple_SetItem(pylist, 0, pylistLinVel);
		PyTuple_SetItem(pylist, 1, pylistAngVel);
		return pylist;
	}
}
static PyObject* pybullet_getNumBodies(PyObject* self, PyObject* args, PyObject* keywds)
{
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;

	static char* kwlist[] = {"physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|i", kwlist, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	{
		int numBodies = b3GetNumBodies(sm);

#if PY_MAJOR_VERSION >= 3
		return PyLong_FromLong(numBodies);
#else
		return PyInt_FromLong(numBodies);
#endif
	}
}


static PyObject* pybullet_computeDofCount(PyObject* self, PyObject* args, PyObject* keywds)
{
	int physicsClientId = 0;
	int bodyUniqueId = -1;
	b3PhysicsClientHandle sm = 0;

	static char* kwlist[] = { "bodyUniqueId", "physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist, &bodyUniqueId, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	{
		int dofCount = b3ComputeDofCount(sm, bodyUniqueId);
		
#if PY_MAJOR_VERSION >= 3
		return PyLong_FromLong(dofCount);
#else
		return PyInt_FromLong(dofCount);
#endif
	}
}


static PyObject* pybullet_getBodyUniqueId(PyObject* self, PyObject* args, PyObject* keywds)
{
	int physicsClientId = 0;
	int serialIndex = -1;
	b3PhysicsClientHandle sm = 0;

	static char* kwlist[] = {"serialIndex", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist, &serialIndex, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	{
		int bodyUniqueId = -1;
		bodyUniqueId = b3GetBodyUniqueId(sm, serialIndex);

#if PY_MAJOR_VERSION >= 3
		return PyLong_FromLong(bodyUniqueId);
#else
		return PyInt_FromLong(bodyUniqueId);
#endif
	}
}

static PyObject* pybullet_removeCollisionShape(PyObject* self, PyObject* args, PyObject* keywds)
{
	{
		int collisionShapeId = -1;
		b3PhysicsClientHandle sm = 0;

		int physicsClientId = 0;
		static char* kwlist[] = {"collisionShapeId", "physicsClientId", NULL};
		if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist, &collisionShapeId, &physicsClientId))
		{
			return NULL;
		}
		sm = getPhysicsClient(physicsClientId);
		if (sm == 0)
		{
			PyErr_SetString(SpamError, "Not connected to physics server.");
			return NULL;
		}
		if (collisionShapeId >= 0)
		{
			b3SharedMemoryStatusHandle statusHandle;
			int statusType;
			if (b3CanSubmitCommand(sm))
			{
				statusHandle = b3SubmitClientCommandAndWaitStatus(sm, b3InitRemoveCollisionShapeCommand(sm, collisionShapeId));
				statusType = b3GetStatusType(statusHandle);
			}
		}
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_removeBody(PyObject* self, PyObject* args, PyObject* keywds)
{
	{
		int bodyUniqueId = -1;
		b3PhysicsClientHandle sm = 0;

		int physicsClientId = 0;
		static char* kwlist[] = {"bodyUniqueId", "physicsClientId", NULL};
		if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist, &bodyUniqueId, &physicsClientId))
		{
			return NULL;
		}
		sm = getPhysicsClient(physicsClientId);
		if (sm == 0)
		{
			PyErr_SetString(SpamError, "Not connected to physics server.");
			return NULL;
		}
		if (bodyUniqueId >= 0)
		{
			b3SharedMemoryStatusHandle statusHandle;
			int statusType;
			if (b3CanSubmitCommand(sm))
			{
				statusHandle = b3SubmitClientCommandAndWaitStatus(sm, b3InitRemoveBodyCommand(sm, bodyUniqueId));
				statusType = b3GetStatusType(statusHandle);
			}
		}
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_getBodyInfo(PyObject* self, PyObject* args, PyObject* keywds)
{
	{
		int bodyUniqueId = -1;
		b3PhysicsClientHandle sm = 0;

		int physicsClientId = 0;
		static char* kwlist[] = {"bodyUniqueId", "physicsClientId", NULL};
		if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist, &bodyUniqueId, &physicsClientId))
		{
			return NULL;
		}
		sm = getPhysicsClient(physicsClientId);
		if (sm == 0)
		{
			PyErr_SetString(SpamError, "Not connected to physics server.");
			return NULL;
		}

		{
			struct b3BodyInfo info;
			if (b3GetBodyInfo(sm, bodyUniqueId, &info))
			{
				PyObject* pyListJointInfo = PyTuple_New(2);
				PyTuple_SetItem(pyListJointInfo, 0, PyString_FromString(info.m_baseName));
				PyTuple_SetItem(pyListJointInfo, 1, PyString_FromString(info.m_bodyName));
				return pyListJointInfo;
			}
		}
	}
	PyErr_SetString(SpamError, "Couldn't get body info");
	return NULL;
}

static PyObject* pybullet_getConstraintInfo(PyObject* self, PyObject* args, PyObject* keywds)
{
	{
		int constraintUniqueId = -1;
		b3PhysicsClientHandle sm = 0;

		int physicsClientId = 0;
		static char* kwlist[] = {"constraintUniqueId", "physicsClientId", NULL};
		if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist, &constraintUniqueId, &physicsClientId))
		{
			return NULL;
		}
		sm = getPhysicsClient(physicsClientId);
		if (sm == 0)
		{
			PyErr_SetString(SpamError, "Not connected to physics server.");
			return NULL;
		}

		{
			struct b3UserConstraint constraintInfo;

			if (b3GetUserConstraintInfo(sm, constraintUniqueId, &constraintInfo))
			{
				PyObject* pyListConstraintInfo = PyTuple_New(15);

				PyTuple_SetItem(pyListConstraintInfo, 0, PyLong_FromLong(constraintInfo.m_parentBodyIndex));
				PyTuple_SetItem(pyListConstraintInfo, 1, PyLong_FromLong(constraintInfo.m_parentJointIndex));
				PyTuple_SetItem(pyListConstraintInfo, 2, PyLong_FromLong(constraintInfo.m_childBodyIndex));
				PyTuple_SetItem(pyListConstraintInfo, 3, PyLong_FromLong(constraintInfo.m_childJointIndex));
				PyTuple_SetItem(pyListConstraintInfo, 4, PyLong_FromLong(constraintInfo.m_jointType));

				{
					PyObject* axisObj = PyTuple_New(3);
					PyTuple_SetItem(axisObj, 0, PyFloat_FromDouble(constraintInfo.m_jointAxis[0]));
					PyTuple_SetItem(axisObj, 1, PyFloat_FromDouble(constraintInfo.m_jointAxis[1]));
					PyTuple_SetItem(axisObj, 2, PyFloat_FromDouble(constraintInfo.m_jointAxis[2]));
					PyTuple_SetItem(pyListConstraintInfo, 5, axisObj);
				}
				{
					PyObject* parentFramePositionObj = PyTuple_New(3);
					PyTuple_SetItem(parentFramePositionObj, 0, PyFloat_FromDouble(constraintInfo.m_parentFrame[0]));
					PyTuple_SetItem(parentFramePositionObj, 1, PyFloat_FromDouble(constraintInfo.m_parentFrame[1]));
					PyTuple_SetItem(parentFramePositionObj, 2, PyFloat_FromDouble(constraintInfo.m_parentFrame[2]));
					PyTuple_SetItem(pyListConstraintInfo, 6, parentFramePositionObj);
				}
				{
					PyObject* childFramePositionObj = PyTuple_New(3);
					PyTuple_SetItem(childFramePositionObj, 0, PyFloat_FromDouble(constraintInfo.m_childFrame[0]));
					PyTuple_SetItem(childFramePositionObj, 1, PyFloat_FromDouble(constraintInfo.m_childFrame[1]));
					PyTuple_SetItem(childFramePositionObj, 2, PyFloat_FromDouble(constraintInfo.m_childFrame[2]));
					PyTuple_SetItem(pyListConstraintInfo, 7, childFramePositionObj);
				}
				{
					PyObject* parentFrameOrientationObj = PyTuple_New(4);
					PyTuple_SetItem(parentFrameOrientationObj, 0, PyFloat_FromDouble(constraintInfo.m_parentFrame[3]));
					PyTuple_SetItem(parentFrameOrientationObj, 1, PyFloat_FromDouble(constraintInfo.m_parentFrame[4]));
					PyTuple_SetItem(parentFrameOrientationObj, 2, PyFloat_FromDouble(constraintInfo.m_parentFrame[5]));
					PyTuple_SetItem(parentFrameOrientationObj, 3, PyFloat_FromDouble(constraintInfo.m_parentFrame[6]));
					PyTuple_SetItem(pyListConstraintInfo, 8, parentFrameOrientationObj);
				}
				{
					PyObject* childFrameOrientation = PyTuple_New(4);
					PyTuple_SetItem(childFrameOrientation, 0, PyFloat_FromDouble(constraintInfo.m_childFrame[3]));
					PyTuple_SetItem(childFrameOrientation, 1, PyFloat_FromDouble(constraintInfo.m_childFrame[4]));
					PyTuple_SetItem(childFrameOrientation, 2, PyFloat_FromDouble(constraintInfo.m_childFrame[5]));
					PyTuple_SetItem(childFrameOrientation, 3, PyFloat_FromDouble(constraintInfo.m_childFrame[6]));
					PyTuple_SetItem(pyListConstraintInfo, 9, childFrameOrientation);
				}
				PyTuple_SetItem(pyListConstraintInfo, 10, PyFloat_FromDouble(constraintInfo.m_maxAppliedForce));
				PyTuple_SetItem(pyListConstraintInfo, 11, PyFloat_FromDouble(constraintInfo.m_gearRatio));
				PyTuple_SetItem(pyListConstraintInfo, 12, PyLong_FromLong(constraintInfo.m_gearAuxLink));
				PyTuple_SetItem(pyListConstraintInfo, 13, PyFloat_FromDouble(constraintInfo.m_relativePositionTarget));
				PyTuple_SetItem(pyListConstraintInfo, 14, PyFloat_FromDouble(constraintInfo.m_erp));

				return pyListConstraintInfo;
			}
		}
	}

	PyErr_SetString(SpamError, "Couldn't get user constraint info");
	return NULL;
}

static PyObject* pybullet_getConstraintState(PyObject* self, PyObject* args, PyObject* keywds)
{
	{
		int constraintUniqueId = -1;
		b3PhysicsClientHandle sm = 0;

		int physicsClientId = 0;
		static char* kwlist[] = {"constraintUniqueId", "physicsClientId", NULL};
		if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist, &constraintUniqueId, &physicsClientId))
		{
			return NULL;
		}
		sm = getPhysicsClient(physicsClientId);
		if (sm == 0)
		{
			PyErr_SetString(SpamError, "Not connected to physics server.");
			return NULL;
		}

		{
			b3SharedMemoryCommandHandle cmd_handle;
			b3SharedMemoryStatusHandle statusHandle;
			int statusType;
			if (b3CanSubmitCommand(sm))
			{
				struct b3UserConstraintState constraintState;
				cmd_handle = b3InitGetUserConstraintStateCommand(sm, constraintUniqueId);
				statusHandle = b3SubmitClientCommandAndWaitStatus(sm, cmd_handle);
				statusType = b3GetStatusType(statusHandle);

				if (b3GetStatusUserConstraintState(statusHandle, &constraintState))
				{
					if (constraintState.m_numDofs)
					{
						PyObject* appliedConstraintForces = PyTuple_New(constraintState.m_numDofs);
						int i = 0;
						for (i = 0; i < constraintState.m_numDofs; i++)
						{
							PyTuple_SetItem(appliedConstraintForces, i, PyFloat_FromDouble(constraintState.m_appliedConstraintForces[i]));
						}
						return appliedConstraintForces;
					}
				}
			}
		}
	}
	PyErr_SetString(SpamError, "Couldn't getConstraintState.");
	return NULL;
}

static PyObject* pybullet_getConstraintUniqueId(PyObject* self, PyObject* args, PyObject* keywds)
{
	int physicsClientId = 0;
	int serialIndex = -1;
	b3PhysicsClientHandle sm = 0;

	static char* kwlist[] = {"serialIndex", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist, &serialIndex, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	{
		int userConstraintId = -1;
		userConstraintId = b3GetUserConstraintId(sm, serialIndex);

#if PY_MAJOR_VERSION >= 3
		return PyLong_FromLong(userConstraintId);
#else
		return PyInt_FromLong(userConstraintId);
#endif
	}
}

static PyObject* pybullet_getNumConstraints(PyObject* self, PyObject* args, PyObject* keywds)
{
	int numConstraints = 0;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	static char* kwlist[] = {"physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|i", kwlist, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	numConstraints = b3GetNumUserConstraints(sm);

#if PY_MAJOR_VERSION >= 3
	return PyLong_FromLong(numConstraints);
#else
	return PyInt_FromLong(numConstraints);
#endif
}

// Return the number of joints in an object based on
// body index; body index is based on order of sequence
// the object is loaded into simulation
static PyObject* pybullet_getAPIVersion(PyObject* self, PyObject* args, PyObject* keywds)
{
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	static char* kwlist[] = {"physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|i", kwlist, &physicsClientId))
	{
		return NULL;
	}

#if PY_MAJOR_VERSION >= 3
	return PyLong_FromLong(SHARED_MEMORY_MAGIC_NUMBER);
#else
	return PyInt_FromLong(SHARED_MEMORY_MAGIC_NUMBER);
#endif
}

// Return the number of joints in an object based on
// body index; body index is based on order of sequence
// the object is loaded into simulation
static PyObject* pybullet_getNumJoints(PyObject* self, PyObject* args, PyObject* keywds)
{
	int bodyUniqueId = -1;
	int numJoints = 0;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	static char* kwlist[] = {"bodyUniqueId", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist, &bodyUniqueId, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	numJoints = b3GetNumJoints(sm, bodyUniqueId);

#if PY_MAJOR_VERSION >= 3
	return PyLong_FromLong(numJoints);
#else
	return PyInt_FromLong(numJoints);
#endif
}

// Initalize all joint positions given a list of values
static PyObject* pybullet_resetJointState(PyObject* self, PyObject* args, PyObject* keywds)
{
	{
		int bodyUniqueId;
		int jointIndex;
		double targetValue;
		double targetVelocity = 0;

		b3PhysicsClientHandle sm = 0;

		int physicsClientId = 0;
		static char* kwlist[] = {"bodyUniqueId", "jointIndex", "targetValue", "targetVelocity", "physicsClientId", NULL};
		if (!PyArg_ParseTupleAndKeywords(args, keywds, "iid|di", kwlist, &bodyUniqueId, &jointIndex, &targetValue, &targetVelocity, &physicsClientId))
		{
			return NULL;
		}
		sm = getPhysicsClient(physicsClientId);
		if (sm == 0)
		{
			PyErr_SetString(SpamError, "Not connected to physics server.");
			return NULL;
		}

		{
			b3SharedMemoryCommandHandle commandHandle;
			b3SharedMemoryStatusHandle statusHandle;
			int numJoints;

			numJoints = b3GetNumJoints(sm, bodyUniqueId);
			if ((jointIndex >= numJoints) || (jointIndex < 0))
			{
				PyErr_SetString(SpamError, "Joint index out-of-range.");
				return NULL;
			}

			commandHandle = b3CreatePoseCommandInit(sm, bodyUniqueId);

			b3CreatePoseCommandSetJointPosition(sm, commandHandle, jointIndex,
												targetValue);

			b3CreatePoseCommandSetJointVelocity(sm, commandHandle, jointIndex,
												targetVelocity);

			statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
		}
	}
	Py_INCREF(Py_None);
	return Py_None;
}


// Initalize all joint positions given a list of values
static PyObject* pybullet_resetJointStatesMultiDof(PyObject* self, PyObject* args, PyObject* keywds)
{
	{
		b3PhysicsClientHandle sm = 0;
		int bodyUniqueId;
		PyObject* jointIndicesObj = 0;
		PyObject* targetPositionsObj = 0;
		PyObject* targetVelocitiesObj = 0;
		PyObject* jointIndicesSeq = 0;
		int numIndices = 0;

		int physicsClientId = 0;
		static char* kwlist[] = { "bodyUniqueId", "jointIndices", "targetValues", "targetVelocities", "physicsClientId", NULL };
		if (!PyArg_ParseTupleAndKeywords(args, keywds, "iOO|Oi", kwlist, &bodyUniqueId, &jointIndicesObj, &targetPositionsObj, &targetVelocitiesObj, &physicsClientId))
		{
			return NULL;
		}
		sm = getPhysicsClient(physicsClientId);
		if (sm == 0)
		{
			PyErr_SetString(SpamError, "Not connected to physics server.");
			return NULL;
		}

		numIndices = PySequence_Size(jointIndicesObj);
		if (numIndices == 0)
		{
			Py_INCREF(Py_None);
			return Py_None;
		}

		jointIndicesSeq = PySequence_Fast(jointIndicesObj, "expected a sequence of joint indices");

		if (jointIndicesSeq == 0)
		{
			PyErr_SetString(SpamError, "expected a sequence of joint indices");
			return NULL;
		}

		{
			int i;
			int numJoints;
			int numTargetPositionObjs, numTargetVelocityobjs;
			PyObject* targetPositionsSeq = 0;
			PyObject* targetVelocitiesSeq = 0;
			
			b3SharedMemoryCommandHandle commandHandle;
			b3SharedMemoryStatusHandle statusHandle;
			commandHandle = b3CreatePoseCommandInit(sm, bodyUniqueId);
			numJoints = b3GetNumJoints(sm, bodyUniqueId);

			numTargetPositionObjs = targetPositionsObj ? PySequence_Size(targetPositionsObj) : 0;
			numTargetVelocityobjs = targetVelocitiesObj ? PySequence_Size(targetVelocitiesObj) : 0;
			

			if (
				((numTargetPositionObjs > 0) && (numIndices != numTargetPositionObjs)) ||
				((numTargetVelocityobjs > 0) && (numIndices != numTargetVelocityobjs)) 
				)
			{
				Py_DECREF(jointIndicesSeq);
				PyErr_SetString(SpamError, "Number of targetValues and targetVelocities needs to match number of indices.");
				return NULL;
			}
			
			
			targetPositionsSeq = PySequence_Fast(targetPositionsObj, "expected a sequence of target positions");
			if (targetVelocitiesObj)
				targetVelocitiesSeq = PySequence_Fast(targetVelocitiesObj, "expected a sequence of target positions");
			for (i = 0; i < numIndices; i++)
			{
				double targetPositionArray[4] = { 0, 0, 0, 1 };
				double targetVelocityArray[3] = { 0, 0, 0 };
				int targetPositionSize = 0;
				int targetVelocitySize = 0;
				PyObject* targetPositionObj = 0;
				PyObject* targetVelocityObj = 0;
				
				int jointIndex = pybullet_internalGetIntFromSequence(jointIndicesSeq, i);
				if ((jointIndex >= numJoints) || (jointIndex < 0))
				{
					if (targetPositionsSeq)
						Py_DECREF(targetPositionsSeq);
					if (targetVelocitiesSeq)
						Py_DECREF(targetVelocitiesSeq);
					Py_DECREF(jointIndicesSeq);
					PyErr_SetString(SpamError, "Joint index out-of-range.");
					return NULL;
				}


				
				

				if (numTargetPositionObjs > 0)
				{
					targetPositionObj = PyList_GET_ITEM(targetPositionsSeq, i);
				}
				if (numTargetVelocityobjs > 0)
				{
					targetVelocityObj = PyList_GET_ITEM(targetVelocitiesSeq, i);
				}


				if (targetPositionObj)
				{
					PyObject* targetPositionSeq = 0;
					int i = 0;
					targetPositionSeq = PySequence_Fast(targetPositionObj, "expected a targetPosition sequence");
					targetPositionSize = PySequence_Size(targetPositionObj);

					if (targetPositionSize < 0)
					{
						targetPositionSize = 0;
					}
					if (targetPositionSize > 4)
					{
						targetPositionSize = 4;
					}
					if (targetPositionSeq)
					{
						for (i = 0; i < targetPositionSize; i++)
						{
							targetPositionArray[i] = pybullet_internalGetFloatFromSequence(targetPositionSeq, i);
						}
						Py_DECREF(targetPositionSeq);
					}
				}

				if (targetVelocityObj)
				{
					int i = 0;
					PyObject* targetVelocitySeq = 0;
					targetVelocitySeq = PySequence_Fast(targetVelocityObj, "expected a targetVelocity sequence");
					targetVelocitySize = PySequence_Size(targetVelocityObj);

					if (targetVelocitySize < 0)
					{
						targetVelocitySize = 0;
					}
					if (targetVelocitySize > 3)
					{
						targetVelocitySize = 3;
					}
					if (targetVelocitySeq)
					{
						for (i = 0; i < targetVelocitySize; i++)
						{
							targetVelocityArray[i] = pybullet_internalGetFloatFromSequence(targetVelocitySeq, i);
						}
						Py_DECREF(targetVelocitySeq);
					}
				}

				if (targetPositionSize == 0 && targetVelocitySize == 0)
				{
					if (targetPositionsSeq)
						Py_DECREF(targetPositionsSeq);
					if (targetVelocitiesSeq)
						Py_DECREF(targetVelocitiesSeq);
					Py_DECREF(jointIndicesSeq);
					PyErr_SetString(SpamError, "Expected an position and/or velocity list.");
					return NULL;
				}
				{


					if (targetPositionSize)
					{
						b3CreatePoseCommandSetJointPositionMultiDof(sm, commandHandle, jointIndex, targetPositionArray, targetPositionSize);
					}
					if (targetVelocitySize)
					{
						b3CreatePoseCommandSetJointVelocityMultiDof(sm, commandHandle, jointIndex, targetVelocityArray, targetVelocitySize);
					}
				}
			}

			if (targetPositionsSeq)
				Py_DECREF(targetPositionsSeq);
			if (targetVelocitiesSeq)
				Py_DECREF(targetVelocitiesSeq);
			Py_DECREF(jointIndicesSeq);
			statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
			
		}
	}
	Py_INCREF(Py_None);
	return Py_None;
}

// Initalize all joint positions given a list of values
static PyObject* pybullet_resetJointStateMultiDof(PyObject* self, PyObject* args, PyObject* keywds)
{
	{
		b3PhysicsClientHandle sm = 0;
		int bodyUniqueId;
		int jointIndex;
		double targetPositionArray[4] = {0, 0, 0, 1};
		double targetVelocityArray[3] = {0, 0, 0};
		int targetPositionSize = 0;
		int targetVelocitySize = 0;
		PyObject* targetPositionObj = 0;
		PyObject* targetVelocityObj = 0;

		int physicsClientId = 0;
		static char* kwlist[] = {"bodyUniqueId", "jointIndex", "targetValue", "targetVelocity", "physicsClientId", NULL};
		if (!PyArg_ParseTupleAndKeywords(args, keywds, "iiO|Oi", kwlist, &bodyUniqueId, &jointIndex, &targetPositionObj, &targetVelocityObj, &physicsClientId))
		{
			return NULL;
		}
		sm = getPhysicsClient(physicsClientId);
		if (sm == 0)
		{
			PyErr_SetString(SpamError, "Not connected to physics server.");
			return NULL;
		}

		if (targetPositionObj)
		{
			PyObject* targetPositionSeq = 0;
			int i = 0;
			targetPositionSeq = PySequence_Fast(targetPositionObj, "expected a targetPosition sequence");
			targetPositionSize = PySequence_Size(targetPositionObj);

			if (targetPositionSize < 0)
			{
				targetPositionSize = 0;
			}
			if (targetPositionSize > 4)
			{
				targetPositionSize = 4;
			}
			if (targetPositionSeq)
			{
				for (i = 0; i < targetPositionSize; i++)
				{
					targetPositionArray[i] = pybullet_internalGetFloatFromSequence(targetPositionSeq, i);
				}
				Py_DECREF(targetPositionSeq);
			}
		}

		if (targetVelocityObj)
		{
			int i = 0;
			PyObject* targetVelocitySeq = 0;
			targetVelocitySeq = PySequence_Fast(targetVelocityObj, "expected a targetVelocity sequence");
			targetVelocitySize = PySequence_Size(targetVelocityObj);

			if (targetVelocitySize < 0)
			{
				targetVelocitySize = 0;
			}
			if (targetVelocitySize > 3)
			{
				targetVelocitySize = 3;
			}
			if (targetVelocitySeq)
			{
				for (i = 0; i < targetVelocitySize; i++)
				{
					targetVelocityArray[i] = pybullet_internalGetFloatFromSequence(targetVelocitySeq, i);
				}
				Py_DECREF(targetVelocitySeq);
			}
		}

		if (targetPositionSize == 0 && targetVelocitySize == 0)
		{
			PyErr_SetString(SpamError, "Expected an position and/or velocity list.");
			return NULL;
		}
		{
			b3SharedMemoryCommandHandle commandHandle;
			b3SharedMemoryStatusHandle statusHandle;
			int numJoints;

			numJoints = b3GetNumJoints(sm, bodyUniqueId);
			if ((jointIndex >= numJoints) || (jointIndex < 0))
			{
				PyErr_SetString(SpamError, "Joint index out-of-range.");
				return NULL;
			}

			commandHandle = b3CreatePoseCommandInit(sm, bodyUniqueId);

			if (targetPositionSize)
			{
				b3CreatePoseCommandSetJointPositionMultiDof(sm, commandHandle, jointIndex, targetPositionArray, targetPositionSize);
			}
			if (targetVelocitySize)
			{
				b3CreatePoseCommandSetJointVelocityMultiDof(sm, commandHandle, jointIndex, targetVelocityArray, targetVelocitySize);
			}

			statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
		}
	}
	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_resetBaseVelocity(PyObject* self, PyObject* args, PyObject* keywds)
{
	static char* kwlist[] = {"objectUniqueId", "linearVelocity", "angularVelocity", "physicsClientId", NULL};

	{
		int bodyUniqueId = 0;
		PyObject* linVelObj = 0;
		PyObject* angVelObj = 0;
		double linVel[3] = {0, 0, 0};
		double angVel[3] = {0, 0, 0};
		int physicsClientId = 0;
		b3PhysicsClientHandle sm = 0;

		if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|OOi", kwlist, &bodyUniqueId, &linVelObj, &angVelObj, &physicsClientId))
		{
			return NULL;
		}

		sm = getPhysicsClient(physicsClientId);
		if (sm == 0)
		{
			PyErr_SetString(SpamError, "Not connected to physics server.");
			return NULL;
		}

		if (linVelObj || angVelObj)
		{
			b3SharedMemoryCommandHandle commandHandle;
			b3SharedMemoryStatusHandle statusHandle;

			commandHandle = b3CreatePoseCommandInit(sm, bodyUniqueId);

			if (linVelObj)
			{
				pybullet_internalSetVectord(linVelObj, linVel);
				b3CreatePoseCommandSetBaseLinearVelocity(commandHandle, linVel);
			}

			if (angVelObj)
			{
				pybullet_internalSetVectord(angVelObj, angVel);
				b3CreatePoseCommandSetBaseAngularVelocity(commandHandle, angVel);
			}

			statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
			Py_INCREF(Py_None);
			return Py_None;
		}
		else
		{
			PyErr_SetString(SpamError, "expected at least linearVelocity and/or angularVelocity.");
			return NULL;
		}
	}
	PyErr_SetString(SpamError, "error in resetJointState.");
	return NULL;
}

// Reset the position and orientation of the base/root link, position [x,y,z]
// and orientation quaternion [x,y,z,w]
static PyObject* pybullet_resetBasePositionAndOrientation(PyObject* self,
														  PyObject* args, PyObject* keywds)
{
	{
		int bodyUniqueId;
		PyObject* posObj;
		PyObject* ornObj;
		double pos[3];
		double orn[4];  // as a quaternion
		b3PhysicsClientHandle sm = 0;

		int physicsClientId = 0;
		static char* kwlist[] = {"bodyUniqueId", "posObj", "ornObj", "physicsClientId", NULL};
		if (!PyArg_ParseTupleAndKeywords(args, keywds, "iOO|i", kwlist, &bodyUniqueId, &posObj, &ornObj, &physicsClientId))
		{
			return NULL;
		}
		sm = getPhysicsClient(physicsClientId);
		if (sm == 0)
		{
			PyErr_SetString(SpamError, "Not connected to physics server.");
			return NULL;
		}

		{
			b3SharedMemoryCommandHandle commandHandle;
			b3SharedMemoryStatusHandle statusHandle;

			{
				PyObject* seq;
				int len, i;
				seq = PySequence_Fast(posObj, "expected a sequence");
				len = PySequence_Size(posObj);
				if (len == 3)
				{
					for (i = 0; i < 3; i++)
					{
						pos[i] = pybullet_internalGetFloatFromSequence(seq, i);
					}
				}
				else
				{
					PyErr_SetString(SpamError, "position needs a 3 coordinates [x,y,z].");
					Py_DECREF(seq);
					return NULL;
				}
				Py_DECREF(seq);
			}

			{
				PyObject* seq;
				int len, i;
				seq = PySequence_Fast(ornObj, "expected a sequence");
				len = PySequence_Size(ornObj);
				if (len == 4)
				{
					for (i = 0; i < 4; i++)
					{
						orn[i] = pybullet_internalGetFloatFromSequence(seq, i);
					}
				}
				else
				{
					PyErr_SetString(
						SpamError,
						"orientation needs a 4 coordinates, quaternion [x,y,z,w].");
					Py_DECREF(seq);
					return NULL;
				}
				Py_DECREF(seq);
			}

			commandHandle = b3CreatePoseCommandInit(sm, bodyUniqueId);

			b3CreatePoseCommandSetBasePosition(commandHandle, pos[0], pos[1], pos[2]);
			b3CreatePoseCommandSetBaseOrientation(commandHandle, orn[0], orn[1],
												  orn[2], orn[3]);

			statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
		}
	}
	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_changeScaling(PyObject* self,
	PyObject* args, PyObject* keywds)
{
	{
		int bodyUniqueId;
		PyObject* scalingObj;
		double scaling[3];
		
		b3PhysicsClientHandle sm = 0;

		int physicsClientId = 0;
		static char* kwlist[] = { "bodyUniqueId", "scaling", "physicsClientId", NULL };
		if (!PyArg_ParseTupleAndKeywords(args, keywds, "iO|i", kwlist, &bodyUniqueId, &scalingObj, &physicsClientId))
		{
			return NULL;
		}
		sm = getPhysicsClient(physicsClientId);
		if (sm == 0)
		{
			PyErr_SetString(SpamError, "Not connected to physics server.");
			return NULL;
		}

		{
			b3SharedMemoryCommandHandle commandHandle;
			b3SharedMemoryStatusHandle statusHandle;

			{
				PyObject* seq;
				int len, i;
				seq = PySequence_Fast(scalingObj, "expected a sequence");
				len = PySequence_Size(scalingObj);
				if (len == 3)
				{
					for (i = 0; i < 3; i++)
					{
						scaling[i] = pybullet_internalGetFloatFromSequence(seq, i);
					}
				}
				else
				{
					PyErr_SetString(SpamError, "scaling needs a 3 coordinates [x,y,z].");
					Py_DECREF(seq);
					return NULL;
				}
				Py_DECREF(seq);
			}



			commandHandle = b3CreatePoseCommandInit(sm, bodyUniqueId);
			b3CreatePoseCommandSetBaseScaling(commandHandle, scaling);

			statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
		}
	}
	Py_INCREF(Py_None);
	return Py_None;
}


// Get the a single joint info for a specific bodyUniqueId
//
// Args:
//  bodyUniqueId - integer indicating body in simulation
//  jointIndex - integer indicating joint for a specific body
//
// Joint information includes:
//  index, name, type, q-index, u-index,
//  flags, joint damping, joint friction
//
// The format of the returned list is
// [int, str, int, int, int, int, float, float]
//
// TODO(hellojas): get joint positions for a body
static PyObject* pybullet_getJointInfo(PyObject* self, PyObject* args, PyObject* keywds)
{
	PyObject* pyListJointInfo;

	struct b3JointInfo info;

	int bodyUniqueId = -1;
	int jointIndex = -1;
	int jointInfoSize = 17;  // size of struct b3JointInfo
	b3PhysicsClientHandle sm = 0;
	int physicsClientId = 0;
	static char* kwlist[] = {"bodyUniqueId", "jointIndex", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "ii|i", kwlist, &bodyUniqueId, &jointIndex, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	{
		{
			// printf("body index = %d, joint index =%d\n", bodyUniqueId, jointIndex);

			pyListJointInfo = PyTuple_New(jointInfoSize);

			if (b3GetJointInfo(sm, bodyUniqueId, jointIndex, &info))
			{
				//  printf("Joint%d %s, type %d, at q-index %d and u-index %d\n",
				//          info.m_jointIndex,
				//          info.m_jointName,
				//          info.m_jointType,
				//          info.m_qIndex,
				//          info.m_uIndex);
				//  printf("  flags=%d jointDamping=%f jointFriction=%f\n",
				//          info.m_flags,
				//          info.m_jointDamping,
				//          info.m_jointFriction);
				PyTuple_SetItem(pyListJointInfo, 0, PyInt_FromLong(info.m_jointIndex));

				if (info.m_jointName[0])
				{
					PyTuple_SetItem(pyListJointInfo, 1,
									PyString_FromString(info.m_jointName));
				}
				else
				{
					PyTuple_SetItem(pyListJointInfo, 1,
									PyString_FromString("not available"));
				}

				PyTuple_SetItem(pyListJointInfo, 2, PyInt_FromLong(info.m_jointType));
				PyTuple_SetItem(pyListJointInfo, 3, PyInt_FromLong(info.m_qIndex));
				PyTuple_SetItem(pyListJointInfo, 4, PyInt_FromLong(info.m_uIndex));
				PyTuple_SetItem(pyListJointInfo, 5, PyInt_FromLong(info.m_flags));
				PyTuple_SetItem(pyListJointInfo, 6,
								PyFloat_FromDouble(info.m_jointDamping));
				PyTuple_SetItem(pyListJointInfo, 7,
								PyFloat_FromDouble(info.m_jointFriction));
				PyTuple_SetItem(pyListJointInfo, 8,
								PyFloat_FromDouble(info.m_jointLowerLimit));
				PyTuple_SetItem(pyListJointInfo, 9,
								PyFloat_FromDouble(info.m_jointUpperLimit));
				PyTuple_SetItem(pyListJointInfo, 10,
								PyFloat_FromDouble(info.m_jointMaxForce));
				PyTuple_SetItem(pyListJointInfo, 11,
								PyFloat_FromDouble(info.m_jointMaxVelocity));
				if (info.m_linkName[0])
				{
					PyTuple_SetItem(pyListJointInfo, 12,
									PyString_FromString(info.m_linkName));
				}
				else
				{
					PyTuple_SetItem(pyListJointInfo, 12,
									PyString_FromString("not available"));
				}
				{
					PyObject* axis = PyTuple_New(3);
					PyTuple_SetItem(axis, 0, PyFloat_FromDouble(info.m_jointAxis[0]));
					PyTuple_SetItem(axis, 1, PyFloat_FromDouble(info.m_jointAxis[1]));
					PyTuple_SetItem(axis, 2, PyFloat_FromDouble(info.m_jointAxis[2]));
					PyTuple_SetItem(pyListJointInfo, 13, axis);
				}
				{
					PyObject* pos = PyTuple_New(3);
					PyTuple_SetItem(pos, 0, PyFloat_FromDouble(info.m_parentFrame[0]));
					PyTuple_SetItem(pos, 1, PyFloat_FromDouble(info.m_parentFrame[1]));
					PyTuple_SetItem(pos, 2, PyFloat_FromDouble(info.m_parentFrame[2]));
					PyTuple_SetItem(pyListJointInfo, 14, pos);
				}
				{
					PyObject* orn = PyTuple_New(4);
					PyTuple_SetItem(orn, 0, PyFloat_FromDouble(info.m_parentFrame[3]));
					PyTuple_SetItem(orn, 1, PyFloat_FromDouble(info.m_parentFrame[4]));
					PyTuple_SetItem(orn, 2, PyFloat_FromDouble(info.m_parentFrame[5]));
					PyTuple_SetItem(orn, 3, PyFloat_FromDouble(info.m_parentFrame[6]));
					PyTuple_SetItem(pyListJointInfo, 15, orn);
				}
				PyTuple_SetItem(pyListJointInfo, 16, PyInt_FromLong(info.m_parentIndex));

				return pyListJointInfo;
			}
			else
			{
				PyErr_SetString(SpamError, "GetJointInfo failed.");
				return NULL;
			}
		}
	}

	Py_INCREF(Py_None);
	return Py_None;
}

// Returns the state of a specific joint in a given bodyUniqueId
//
// Args:
//  bodyUniqueId - integer indicating body in simulation
//  jointIndex - integer indicating joint for a specific body
//
// The state of a joint includes the following:
//  position, velocity, force torque (6 values), and motor torque
// The returned pylist is an array of [float, float, float[6], float]

// TODO(hellojas): check accuracy of position and velocity
// TODO(hellojas): check force torque values

static PyObject* pybullet_getJointState(PyObject* self, PyObject* args, PyObject* keywds)
{
	PyObject* pyListJointForceTorque;
	PyObject* pyListJointState;

	struct b3JointSensorState sensorState;

	int bodyUniqueId = -1;
	int jointIndex = -1;
	int sensorStateSize = 4;  // size of struct b3JointSensorState
	int forceTorqueSize = 6;  // size of force torque list from b3JointSensorState
	int j;

	b3PhysicsClientHandle sm = 0;
	int physicsClientId = 0;
	static char* kwlist[] = {"bodyUniqueId", "jointIndex", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "ii|i", kwlist, &bodyUniqueId, &jointIndex, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	{
		{
			int status_type = 0;
			b3SharedMemoryCommandHandle cmd_handle;
			b3SharedMemoryStatusHandle status_handle;

			if (bodyUniqueId < 0)
			{
				PyErr_SetString(SpamError, "getJointState failed; invalid bodyUniqueId");
				return NULL;
			}
			if (jointIndex < 0)
			{
				PyErr_SetString(SpamError, "getJointState failed; invalid jointIndex");
				return NULL;
			}

			cmd_handle =
				b3RequestActualStateCommandInit(sm, bodyUniqueId);
			status_handle =
				b3SubmitClientCommandAndWaitStatus(sm, cmd_handle);

			status_type = b3GetStatusType(status_handle);
			if (status_type != CMD_ACTUAL_STATE_UPDATE_COMPLETED)
			{
				PyErr_SetString(SpamError, "getJointState failed.");
				return NULL;
			}

			pyListJointState = PyTuple_New(sensorStateSize);
			pyListJointForceTorque = PyTuple_New(forceTorqueSize);

			if (b3GetJointState(sm, status_handle, jointIndex, &sensorState))
			{
				PyTuple_SetItem(pyListJointState, 0,
								PyFloat_FromDouble(sensorState.m_jointPosition));
				PyTuple_SetItem(pyListJointState, 1,
								PyFloat_FromDouble(sensorState.m_jointVelocity));

				for (j = 0; j < forceTorqueSize; j++)
				{
					PyTuple_SetItem(pyListJointForceTorque, j,
									PyFloat_FromDouble(sensorState.m_jointForceTorque[j]));
				}

				PyTuple_SetItem(pyListJointState, 2, pyListJointForceTorque);

				PyTuple_SetItem(pyListJointState, 3,
								PyFloat_FromDouble(sensorState.m_jointMotorTorque));

				return pyListJointState;
			}
			else
			{
				PyErr_SetString(SpamError, "getJointState failed (2).");
				return NULL;
			}
		}
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_getJointStateMultiDof(PyObject* self, PyObject* args, PyObject* keywds)
{
	PyObject* pyListJointForceTorque;
	PyObject* pyListJointState;
	PyObject* pyListPosition;
	PyObject* pyListVelocity;
	PyObject* pyListJointMotorTorque;

	struct b3JointSensorState2 sensorState;

	int bodyUniqueId = -1;
	int jointIndex = -1;
	int sensorStateSize = 4;  // size of struct b3JointSensorState
	int forceTorqueSize = 6;  // size of force torque list from b3JointSensorState
	int j;

	b3PhysicsClientHandle sm = 0;
	int physicsClientId = 0;
	static char* kwlist[] = {"bodyUniqueId", "jointIndex", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "ii|i", kwlist, &bodyUniqueId, &jointIndex, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	{
		{
			int status_type = 0;
			b3SharedMemoryCommandHandle cmd_handle;
			b3SharedMemoryStatusHandle status_handle;

			if (bodyUniqueId < 0)
			{
				PyErr_SetString(SpamError, "getJointState failed; invalid bodyUniqueId");
				return NULL;
			}
			if (jointIndex < 0)
			{
				PyErr_SetString(SpamError, "getJointState failed; invalid jointIndex");
				return NULL;
			}

			cmd_handle =
				b3RequestActualStateCommandInit(sm, bodyUniqueId);
			status_handle =
				b3SubmitClientCommandAndWaitStatus(sm, cmd_handle);

			status_type = b3GetStatusType(status_handle);
			if (status_type != CMD_ACTUAL_STATE_UPDATE_COMPLETED)
			{
				PyErr_SetString(SpamError, "getJointState failed.");
				return NULL;
			}

			pyListJointState = PyTuple_New(sensorStateSize);
			pyListJointForceTorque = PyTuple_New(forceTorqueSize);

			if (b3GetJointStateMultiDof(sm, status_handle, jointIndex, &sensorState))
			{
				int i = 0;
				pyListPosition = PyTuple_New(sensorState.m_qDofSize);
				pyListVelocity = PyTuple_New(sensorState.m_uDofSize);
				pyListJointMotorTorque = PyTuple_New(sensorState.m_uDofSize);
				PyTuple_SetItem(pyListJointState, 0, pyListPosition);
				PyTuple_SetItem(pyListJointState, 1, pyListVelocity);

				for (i = 0; i < sensorState.m_qDofSize; i++)
				{
					PyTuple_SetItem(pyListPosition, i,
									PyFloat_FromDouble(sensorState.m_jointPosition[i]));
				}

				for (i = 0; i < sensorState.m_uDofSize; i++)
				{
					PyTuple_SetItem(pyListVelocity, i,
									PyFloat_FromDouble(sensorState.m_jointVelocity[i]));

					PyTuple_SetItem(pyListJointMotorTorque, i,
									PyFloat_FromDouble(sensorState.m_jointMotorTorqueMultiDof[i]));
				}

				for (j = 0; j < forceTorqueSize; j++)
				{
					PyTuple_SetItem(pyListJointForceTorque, j,
									PyFloat_FromDouble(sensorState.m_jointReactionForceTorque[j]));
				}

				PyTuple_SetItem(pyListJointState, 2, pyListJointForceTorque);

				PyTuple_SetItem(pyListJointState, 3, pyListJointMotorTorque);

				return pyListJointState;
			}
			else
			{
				PyErr_SetString(SpamError, "getJointState failed (2).");
				return NULL;
			}
		}
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_getJointStatesMultiDof(PyObject* self, PyObject* args, PyObject* keywds)
{
	PyObject* pyListPosition;
	PyObject* pyListVelocity;
	PyObject* pyListJointMotorTorque;

	struct b3JointSensorState2 sensorState;

	int bodyUniqueId = -1;
	
	PyObject* jointIndicesObj = 0;
	int sensorStateSize = 4;  // size of struct b3JointSensorState
	int forceTorqueSize = 6;  // size of force torque list from b3JointSensorState
	int j;

	b3PhysicsClientHandle sm = 0;
	int physicsClientId = 0;
	static char* kwlist[] = { "bodyUniqueId", "jointIndex", "physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "iO|i", kwlist, &bodyUniqueId, &jointIndicesObj, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
	{
		{
			int status_type = 0;
			b3SharedMemoryCommandHandle cmd_handle;
			b3SharedMemoryStatusHandle status_handle;

			if (bodyUniqueId < 0)
			{
				PyErr_SetString(SpamError, "getJointState failed; invalid bodyUniqueId");
				
				return NULL;
			}
			
			cmd_handle =
				b3RequestActualStateCommandInit(sm, bodyUniqueId);
			status_handle =
				b3SubmitClientCommandAndWaitStatus(sm, cmd_handle);

			status_type = b3GetStatusType(status_handle);
			if (status_type != CMD_ACTUAL_STATE_UPDATE_COMPLETED)
			{
				PyErr_SetString(SpamError, "getJointState failed.");
				
				return NULL;
			}

			{
				int numJoints, numRequestedJoints;
				int jnt;
				PyObject* jointIndicesSeq = 0;
				PyObject* resultListJointState = 0;

				numJoints = b3GetNumJoints(sm, bodyUniqueId);
				jointIndicesSeq = PySequence_Fast(jointIndicesObj, "expected a sequence of joint indices");

				if (jointIndicesSeq == 0)
				{
					PyErr_SetString(SpamError, "expected a sequence of joint indices");
					return NULL;
				}

				numRequestedJoints = PySequence_Size(jointIndicesObj);
				if (numRequestedJoints == 0)
				{
					Py_DECREF(jointIndicesSeq);
					Py_INCREF(Py_None);
					return Py_None;
				}

				resultListJointState = PyTuple_New(numRequestedJoints);

				for (jnt = 0; jnt < numRequestedJoints; jnt++)
				{
					PyObject* pyListJointForceTorque;
					PyObject* pyListJointState;
					int jointIndex = pybullet_internalGetFloatFromSequence(jointIndicesSeq, jnt);

					pyListJointState = PyTuple_New(sensorStateSize);
					pyListJointForceTorque = PyTuple_New(forceTorqueSize);

					if (b3GetJointStateMultiDof(sm, status_handle, jointIndex, &sensorState))
					{
						int i = 0;
						pyListPosition = PyTuple_New(sensorState.m_qDofSize);
						pyListVelocity = PyTuple_New(sensorState.m_uDofSize);
						pyListJointMotorTorque = PyTuple_New(sensorState.m_uDofSize);
						PyTuple_SetItem(pyListJointState, 0, pyListPosition);
						PyTuple_SetItem(pyListJointState, 1, pyListVelocity);

						for (i = 0; i < sensorState.m_qDofSize; i++)
						{
							PyTuple_SetItem(pyListPosition, i,
								PyFloat_FromDouble(sensorState.m_jointPosition[i]));
						}

						for (i = 0; i < sensorState.m_uDofSize; i++)
						{
							PyTuple_SetItem(pyListVelocity, i,
								PyFloat_FromDouble(sensorState.m_jointVelocity[i]));

							PyTuple_SetItem(pyListJointMotorTorque, i,
								PyFloat_FromDouble(sensorState.m_jointMotorTorqueMultiDof[i]));
						}

						for (j = 0; j < forceTorqueSize; j++)
						{
							PyTuple_SetItem(pyListJointForceTorque, j,
								PyFloat_FromDouble(sensorState.m_jointReactionForceTorque[j]));
						}

						PyTuple_SetItem(pyListJointState, 2, pyListJointForceTorque);

						PyTuple_SetItem(pyListJointState, 3, pyListJointMotorTorque);

						PyTuple_SetItem(resultListJointState, jnt, pyListJointState);
					}
					else
					{
						PyErr_SetString(SpamError, "getJointState failed (2).");
						Py_DECREF(jointIndicesSeq);
						return NULL;
					}
				}
				Py_DECREF(jointIndicesSeq);
				return resultListJointState;
			}
		}
	}

	Py_INCREF(Py_None);
	return Py_None;
}


static PyObject* pybullet_getJointStates(PyObject* self, PyObject* args, PyObject* keywds)
{
	PyObject* pyListJointForceTorque;
	PyObject* pyListJointState;
	PyObject* jointIndicesObj = 0;

	struct b3JointSensorState sensorState;

	int bodyUniqueId = -1;
	int sensorStateSize = 4;  // size of struct b3JointSensorState
	int forceTorqueSize = 6;  // size of force torque list from b3JointSensorState
	int j;

	b3PhysicsClientHandle sm = 0;
	int physicsClientId = 0;
	static char* kwlist[] = {"bodyUniqueId", "jointIndices", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "iO|i", kwlist, &bodyUniqueId, &jointIndicesObj, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	{
		{
			int i;
			int status_type = 0;
			int numRequestedJoints = 0;
			PyObject* jointIndicesSeq = 0;
			int numJoints = 0;
			PyObject* resultListJointState = 0;
			b3SharedMemoryCommandHandle cmd_handle;
			b3SharedMemoryStatusHandle status_handle;

			if (bodyUniqueId < 0)
			{
				PyErr_SetString(SpamError, "getJointState failed; invalid bodyUniqueId");
				return NULL;
			}
			numJoints = b3GetNumJoints(sm, bodyUniqueId);
			jointIndicesSeq = PySequence_Fast(jointIndicesObj, "expected a sequence of joint indices");

			if (jointIndicesSeq == 0)
			{
				PyErr_SetString(SpamError, "expected a sequence of joint indices");
				return NULL;
			}

			numRequestedJoints = PySequence_Size(jointIndicesObj);
			if (numRequestedJoints == 0)
			{
				Py_DECREF(jointIndicesSeq);
				Py_INCREF(Py_None);
				return Py_None;
			}

			cmd_handle =
				b3RequestActualStateCommandInit(sm, bodyUniqueId);
			status_handle =
				b3SubmitClientCommandAndWaitStatus(sm, cmd_handle);

			status_type = b3GetStatusType(status_handle);
			if (status_type != CMD_ACTUAL_STATE_UPDATE_COMPLETED)
			{
				PyErr_SetString(SpamError, "getJointState failed.");
				return NULL;
			}

			resultListJointState = PyTuple_New(numRequestedJoints);

			for (i = 0; i < numRequestedJoints; i++)
			{
				int jointIndex = pybullet_internalGetFloatFromSequence(jointIndicesSeq, i);
				if ((jointIndex >= numJoints) || (jointIndex < 0))
				{
					Py_DECREF(jointIndicesSeq);
					PyErr_SetString(SpamError, "Joint index out-of-range.");
					return NULL;
				}

				pyListJointState = PyTuple_New(sensorStateSize);
				pyListJointForceTorque = PyTuple_New(forceTorqueSize);

				if (b3GetJointState(sm, status_handle, jointIndex, &sensorState))
				{
					PyTuple_SetItem(pyListJointState, 0,
									PyFloat_FromDouble(sensorState.m_jointPosition));
					PyTuple_SetItem(pyListJointState, 1,
									PyFloat_FromDouble(sensorState.m_jointVelocity));

					for (j = 0; j < forceTorqueSize; j++)
					{
						PyTuple_SetItem(pyListJointForceTorque, j,
										PyFloat_FromDouble(sensorState.m_jointForceTorque[j]));
					}

					PyTuple_SetItem(pyListJointState, 2, pyListJointForceTorque);

					PyTuple_SetItem(pyListJointState, 3,
									PyFloat_FromDouble(sensorState.m_jointMotorTorque));

					PyTuple_SetItem(resultListJointState, i, pyListJointState);
				}
				else
				{
					PyErr_SetString(SpamError, "getJointState failed (2).");
					return NULL;
				}
			}
			Py_DECREF(jointIndicesSeq);
			return resultListJointState;
		}
	}

	Py_INCREF(Py_None);
	return Py_None;
}
static PyObject* pybullet_getLinkState(PyObject* self, PyObject* args, PyObject* keywds)
{
	PyObject* pyLinkState;
	PyObject* pyLinkStateWorldPosition;
	PyObject* pyLinkStateWorldOrientation;
	PyObject* pyLinkStateLocalInertialPosition;
	PyObject* pyLinkStateLocalInertialOrientation;
	PyObject* pyLinkStateWorldLinkFramePosition;
	PyObject* pyLinkStateWorldLinkFrameOrientation;
	PyObject* pyLinkStateWorldLinkLinearVelocity;
	PyObject* pyLinkStateWorldLinkAngularVelocity;

	struct b3LinkState linkState;

	int bodyUniqueId = -1;
	int linkIndex = -1;
	int computeLinkVelocity = 0;
	int computeForwardKinematics = 0;

	int i;
	b3PhysicsClientHandle sm = 0;

	int physicsClientId = 0;
	static char* kwlist[] = {"bodyUniqueId", "linkIndex", "computeLinkVelocity", "computeForwardKinematics", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "ii|iii", kwlist, &bodyUniqueId, &linkIndex, &computeLinkVelocity, &computeForwardKinematics, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	{
		{
			int status_type = 0;
			b3SharedMemoryCommandHandle cmd_handle;
			b3SharedMemoryStatusHandle status_handle;

			if (bodyUniqueId < 0)
			{
				PyErr_SetString(SpamError, "getLinkState failed; invalid bodyUniqueId");
				return NULL;
			}
			if (linkIndex < 0)
			{
				PyErr_SetString(SpamError, "getLinkState failed; invalid linkIndex");
				return NULL;
			}

			cmd_handle =
				b3RequestActualStateCommandInit(sm, bodyUniqueId);

			if (computeLinkVelocity)
			{
				b3RequestActualStateCommandComputeLinkVelocity(cmd_handle, computeLinkVelocity);
			}

			if (computeForwardKinematics)
			{
				b3RequestActualStateCommandComputeForwardKinematics(cmd_handle, computeForwardKinematics);
			}

			status_handle =
				b3SubmitClientCommandAndWaitStatus(sm, cmd_handle);

			status_type = b3GetStatusType(status_handle);
			if (status_type != CMD_ACTUAL_STATE_UPDATE_COMPLETED)
			{
				PyErr_SetString(SpamError, "getLinkState failed.");
				return NULL;
			}

			if (b3GetLinkState(sm, status_handle, linkIndex, &linkState))
			{
				pyLinkStateWorldPosition = PyTuple_New(3);
				for (i = 0; i < 3; ++i)
				{
					PyTuple_SetItem(pyLinkStateWorldPosition, i,
									PyFloat_FromDouble(linkState.m_worldPosition[i]));
				}

				pyLinkStateWorldOrientation = PyTuple_New(4);
				for (i = 0; i < 4; ++i)
				{
					PyTuple_SetItem(pyLinkStateWorldOrientation, i,
									PyFloat_FromDouble(linkState.m_worldOrientation[i]));
				}

				pyLinkStateLocalInertialPosition = PyTuple_New(3);
				for (i = 0; i < 3; ++i)
				{
					PyTuple_SetItem(pyLinkStateLocalInertialPosition, i,
									PyFloat_FromDouble(linkState.m_localInertialPosition[i]));
				}

				pyLinkStateLocalInertialOrientation = PyTuple_New(4);
				for (i = 0; i < 4; ++i)
				{
					PyTuple_SetItem(pyLinkStateLocalInertialOrientation, i,
									PyFloat_FromDouble(linkState.m_localInertialOrientation[i]));
				}

				pyLinkStateWorldLinkFramePosition = PyTuple_New(3);
				for (i = 0; i < 3; ++i)
				{
					PyTuple_SetItem(pyLinkStateWorldLinkFramePosition, i,
									PyFloat_FromDouble(linkState.m_worldLinkFramePosition[i]));
				}

				pyLinkStateWorldLinkFrameOrientation = PyTuple_New(4);
				for (i = 0; i < 4; ++i)
				{
					PyTuple_SetItem(pyLinkStateWorldLinkFrameOrientation, i,
									PyFloat_FromDouble(linkState.m_worldLinkFrameOrientation[i]));
				}

				if (computeLinkVelocity)
				{
					pyLinkState = PyTuple_New(8);
				}
				else
				{
					pyLinkState = PyTuple_New(6);
				}

				PyTuple_SetItem(pyLinkState, 0, pyLinkStateWorldPosition);
				PyTuple_SetItem(pyLinkState, 1, pyLinkStateWorldOrientation);
				PyTuple_SetItem(pyLinkState, 2, pyLinkStateLocalInertialPosition);
				PyTuple_SetItem(pyLinkState, 3, pyLinkStateLocalInertialOrientation);
				PyTuple_SetItem(pyLinkState, 4, pyLinkStateWorldLinkFramePosition);
				PyTuple_SetItem(pyLinkState, 5, pyLinkStateWorldLinkFrameOrientation);

				if (computeLinkVelocity)
				{
					pyLinkStateWorldLinkLinearVelocity = PyTuple_New(3);
					pyLinkStateWorldLinkAngularVelocity = PyTuple_New(3);
					for (i = 0; i < 3; ++i)
					{
						PyTuple_SetItem(pyLinkStateWorldLinkLinearVelocity, i,
										PyFloat_FromDouble(linkState.m_worldLinearVelocity[i]));
						PyTuple_SetItem(pyLinkStateWorldLinkAngularVelocity, i,
										PyFloat_FromDouble(linkState.m_worldAngularVelocity[i]));
					}
					PyTuple_SetItem(pyLinkState, 6, pyLinkStateWorldLinkLinearVelocity);
					PyTuple_SetItem(pyLinkState, 7, pyLinkStateWorldLinkAngularVelocity);
				}
				return pyLinkState;
			}
		}
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_getLinkStates(PyObject* self, PyObject* args, PyObject* keywds)
{
	PyObject* pyLinkState;
	PyObject* pyLinkStateWorldPosition;
	PyObject* pyLinkStateWorldOrientation;
	PyObject* pyLinkStateLocalInertialPosition;
	PyObject* pyLinkStateLocalInertialOrientation;
	PyObject* pyLinkStateWorldLinkFramePosition;
	PyObject* pyLinkStateWorldLinkFrameOrientation;
	PyObject* pyLinkStateWorldLinkLinearVelocity;
	PyObject* pyLinkStateWorldLinkAngularVelocity;
	PyObject* linkIndicesObj = 0;

	struct b3LinkState linkState;

	int bodyUniqueId = -1;
	
	int computeLinkVelocity = 0;
	int computeForwardKinematics = 0;

	
	b3PhysicsClientHandle sm = 0;

	int physicsClientId = 0;
	static char* kwlist[] = { "bodyUniqueId", "linkIndices", "computeLinkVelocity", "computeForwardKinematics", "physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "iO|iii", kwlist, &bodyUniqueId, &linkIndicesObj, &computeLinkVelocity, &computeForwardKinematics, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	{
		{
			int status_type = 0;
			b3SharedMemoryCommandHandle cmd_handle;
			b3SharedMemoryStatusHandle status_handle;
			PyObject* linkIndicesSeq = 0;
			int numRequestedLinks = -1;
			int numJoints = 0;
			int link = -1;
			PyObject* resultListLinkState = 0;
			if (bodyUniqueId < 0)
			{
				PyErr_SetString(SpamError, "getLinkState failed; invalid bodyUniqueId");
				return NULL;
			}
		
			cmd_handle =
				b3RequestActualStateCommandInit(sm, bodyUniqueId);

			if (computeLinkVelocity)
			{
				b3RequestActualStateCommandComputeLinkVelocity(cmd_handle, computeLinkVelocity);
			}

			if (computeForwardKinematics)
			{
				b3RequestActualStateCommandComputeForwardKinematics(cmd_handle, computeForwardKinematics);
			}

			status_handle =
				b3SubmitClientCommandAndWaitStatus(sm, cmd_handle);

			status_type = b3GetStatusType(status_handle);
			if (status_type != CMD_ACTUAL_STATE_UPDATE_COMPLETED)
			{
				PyErr_SetString(SpamError, "getLinkState failed.");
				return NULL;
			}

			linkIndicesSeq = PySequence_Fast(linkIndicesObj, "expected a sequence of link indices");

			if (linkIndicesSeq == 0)
			{
				PyErr_SetString(SpamError, "expected a sequence of joint indices");
				return NULL;
			}

			numRequestedLinks = PySequence_Size(linkIndicesObj);
			numJoints = b3GetNumJoints(sm, bodyUniqueId);
			resultListLinkState = PyTuple_New(numRequestedLinks);
			for (link=0;link<numRequestedLinks;link++)
			{
				int linkIndex = pybullet_internalGetIntFromSequence(linkIndicesSeq, link);
				if ((linkIndex < numJoints) || (linkIndex >= 0))
				{
					if (b3GetLinkState(sm, status_handle, linkIndex, &linkState))
					{
						int i;
						pyLinkStateWorldPosition = PyTuple_New(3);
						for (i = 0; i < 3; ++i)
						{
							PyTuple_SetItem(pyLinkStateWorldPosition, i,
								PyFloat_FromDouble(linkState.m_worldPosition[i]));
						}

						pyLinkStateWorldOrientation = PyTuple_New(4);
						for (i = 0; i < 4; ++i)
						{
							PyTuple_SetItem(pyLinkStateWorldOrientation, i,
								PyFloat_FromDouble(linkState.m_worldOrientation[i]));
						}

						pyLinkStateLocalInertialPosition = PyTuple_New(3);
						for (i = 0; i < 3; ++i)
						{
							PyTuple_SetItem(pyLinkStateLocalInertialPosition, i,
								PyFloat_FromDouble(linkState.m_localInertialPosition[i]));
						}

						pyLinkStateLocalInertialOrientation = PyTuple_New(4);
						for (i = 0; i < 4; ++i)
						{
							PyTuple_SetItem(pyLinkStateLocalInertialOrientation, i,
								PyFloat_FromDouble(linkState.m_localInertialOrientation[i]));
						}

						pyLinkStateWorldLinkFramePosition = PyTuple_New(3);
						for (i = 0; i < 3; ++i)
						{
							PyTuple_SetItem(pyLinkStateWorldLinkFramePosition, i,
								PyFloat_FromDouble(linkState.m_worldLinkFramePosition[i]));
						}

						pyLinkStateWorldLinkFrameOrientation = PyTuple_New(4);
						for (i = 0; i < 4; ++i)
						{
							PyTuple_SetItem(pyLinkStateWorldLinkFrameOrientation, i,
								PyFloat_FromDouble(linkState.m_worldLinkFrameOrientation[i]));
						}

						if (computeLinkVelocity)
						{
							pyLinkState = PyTuple_New(8);
						}
						else
						{
							pyLinkState = PyTuple_New(6);
						}

						PyTuple_SetItem(pyLinkState, 0, pyLinkStateWorldPosition);
						PyTuple_SetItem(pyLinkState, 1, pyLinkStateWorldOrientation);
						PyTuple_SetItem(pyLinkState, 2, pyLinkStateLocalInertialPosition);
						PyTuple_SetItem(pyLinkState, 3, pyLinkStateLocalInertialOrientation);
						PyTuple_SetItem(pyLinkState, 4, pyLinkStateWorldLinkFramePosition);
						PyTuple_SetItem(pyLinkState, 5, pyLinkStateWorldLinkFrameOrientation);

						if (computeLinkVelocity)
						{
							pyLinkStateWorldLinkLinearVelocity = PyTuple_New(3);
							pyLinkStateWorldLinkAngularVelocity = PyTuple_New(3);
							for (i = 0; i < 3; ++i)
							{
								PyTuple_SetItem(pyLinkStateWorldLinkLinearVelocity, i,
									PyFloat_FromDouble(linkState.m_worldLinearVelocity[i]));
								PyTuple_SetItem(pyLinkStateWorldLinkAngularVelocity, i,
									PyFloat_FromDouble(linkState.m_worldAngularVelocity[i]));
							}
							PyTuple_SetItem(pyLinkState, 6, pyLinkStateWorldLinkLinearVelocity);
							PyTuple_SetItem(pyLinkState, 7, pyLinkStateWorldLinkAngularVelocity);
						}
						PyTuple_SetItem(resultListLinkState, link, pyLinkState);
						
					}
				}
				else
				{
					//invalid link, add a -1 result
					PyTuple_SetItem(resultListLinkState, link, PyFloat_FromDouble(-1));
				}
			}
			Py_DECREF(linkIndicesSeq);
			return resultListLinkState;
		}
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_readUserDebugParameter(PyObject* self, PyObject* args, PyObject* keywds)
{
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	int itemUniqueId;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;

	static char* kwlist[] = {"itemUniqueId", "physicsClientId", NULL};

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist, &itemUniqueId, &physicsClientId))
	{
		return NULL;
	}

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	commandHandle = b3InitUserDebugReadParameter(sm, itemUniqueId);

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	if (statusType == CMD_USER_DEBUG_DRAW_PARAMETER_COMPLETED)
	{
		double paramValue = 0.f;
		int ok = b3GetStatusDebugParameterValue(statusHandle, &paramValue);
		if (ok)
		{
			PyObject* item = PyFloat_FromDouble(paramValue);
			return item;
		}
	}

	PyErr_SetString(SpamError, "Failed to read parameter.");
	return NULL;
}

static PyObject* pybullet_addUserDebugParameter(PyObject* self, PyObject* args, PyObject* keywds)
{
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	char* text;

	double rangeMin = 0.f;
	double rangeMax = 1.f;
	double startValue = 0.f;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	static char* kwlist[] = {"paramName", "rangeMin", "rangeMax", "startValue", "physicsClientId", NULL};

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "s|dddi", kwlist, &text, &rangeMin, &rangeMax, &startValue, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	commandHandle = b3InitUserDebugAddParameter(sm, text, rangeMin, rangeMax, startValue);

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);

	if (statusType == CMD_USER_DEBUG_DRAW_COMPLETED)
	{
		int debugItemUniqueId = b3GetDebugItemUniqueId(statusHandle);
		PyObject* item = PyInt_FromLong(debugItemUniqueId);
		return item;
	}

	PyErr_SetString(SpamError, "Error in addUserDebugParameter.");
	return NULL;
}

static PyObject* pybullet_addUserDebugText(PyObject* self, PyObject* args, PyObject* keywds)
{
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	int res = 0;

	char* text;
	double posXYZ[3];
	double colorRGB[3] = {1, 1, 1};

	PyObject* textPositionObj = 0;
	PyObject* textColorRGBObj = 0;
	PyObject* textOrientationObj = 0;
	double textOrientation[4];
	int parentObjectUniqueId = -1;
	int parentLinkIndex = -1;

	double textSize = 1.f;
	double lifeTime = 0.f;
	int physicsClientId = 0;
	int debugItemUniqueId = -1;
	int replaceItemUniqueId = -1;

	b3PhysicsClientHandle sm = 0;
	static char* kwlist[] = {"text", "textPosition", "textColorRGB", "textSize", "lifeTime", "textOrientation", "parentObjectUniqueId", "parentLinkIndex", "replaceItemUniqueId", "physicsClientId", NULL};

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "sO|OddOiiii", kwlist, &text, &textPositionObj, &textColorRGBObj, &textSize, &lifeTime, &textOrientationObj, &parentObjectUniqueId, &parentLinkIndex, &replaceItemUniqueId, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	res = pybullet_internalSetVectord(textPositionObj, posXYZ);
	if (!res)
	{
		PyErr_SetString(SpamError, "Error converting textPositionObj[3]");
		return NULL;
	}

	if (textColorRGBObj)
	{
		res = pybullet_internalSetVectord(textColorRGBObj, colorRGB);
		if (!res)
		{
			PyErr_SetString(SpamError, "Error converting textColorRGBObj[3]");
			return NULL;
		}
	}

	commandHandle = b3InitUserDebugDrawAddText3D(sm, text, posXYZ, colorRGB, textSize, lifeTime);

	if (parentObjectUniqueId >= 0)
	{
		b3UserDebugItemSetParentObject(commandHandle, parentObjectUniqueId, parentLinkIndex);
	}
	if (textOrientationObj)
	{
		res = pybullet_internalSetVector4d(textOrientationObj, textOrientation);
		if (!res)
		{
			PyErr_SetString(SpamError, "Error converting textOrientation[4]");
			return NULL;
		}
		else
		{
			b3UserDebugTextSetOrientation(commandHandle, textOrientation);
		}
	}

	if (replaceItemUniqueId >= 0)
	{
		b3UserDebugItemSetReplaceItemUniqueId(commandHandle, replaceItemUniqueId);
	}

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);

	if (statusType == CMD_USER_DEBUG_DRAW_COMPLETED)
	{
		debugItemUniqueId = b3GetDebugItemUniqueId(statusHandle);
	}

	{
		PyObject* item = PyInt_FromLong(debugItemUniqueId);
		return item;
	}
}

static PyObject* pybullet_addUserDebugLine(PyObject* self, PyObject* args, PyObject* keywds)
{
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	int res = 0;

	double fromXYZ[3];
	double toXYZ[3];
	double colorRGB[3] = {1, 1, 1};
	int parentObjectUniqueId = -1;
	int parentLinkIndex = -1;

	PyObject* lineFromObj = 0;
	PyObject* lineToObj = 0;
	PyObject* lineColorRGBObj = 0;
	double lineWidth = 1.f;
	double lifeTime = 0.f;
	int physicsClientId = 0;
	int debugItemUniqueId = -1;
	int replaceItemUniqueId = -1;
	b3PhysicsClientHandle sm = 0;
	static char* kwlist[] = {"lineFromXYZ", "lineToXYZ", "lineColorRGB", "lineWidth", "lifeTime", "parentObjectUniqueId", "parentLinkIndex", "replaceItemUniqueId", "physicsClientId", NULL};

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "OO|Oddiiii", kwlist, &lineFromObj, &lineToObj, &lineColorRGBObj, &lineWidth, &lifeTime, &parentObjectUniqueId, &parentLinkIndex, &replaceItemUniqueId, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	res = pybullet_internalSetVectord(lineFromObj, fromXYZ);
	if (!res)
	{
		PyErr_SetString(SpamError, "Error converting lineFrom[3]");
		return NULL;
	}

	res = pybullet_internalSetVectord(lineToObj, toXYZ);
	if (!res)
	{
		PyErr_SetString(SpamError, "Error converting lineTo[3]");
		return NULL;
	}
	if (lineColorRGBObj)
	{
		res = pybullet_internalSetVectord(lineColorRGBObj, colorRGB);
	}

	commandHandle = b3InitUserDebugDrawAddLine3D(sm, fromXYZ, toXYZ, colorRGB, lineWidth, lifeTime);

	if (parentObjectUniqueId >= 0)
	{
		b3UserDebugItemSetParentObject(commandHandle, parentObjectUniqueId, parentLinkIndex);
	}

	if (replaceItemUniqueId >= 0)
	{
		b3UserDebugItemSetReplaceItemUniqueId(commandHandle, replaceItemUniqueId);
	}

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	if (statusType == CMD_USER_DEBUG_DRAW_COMPLETED)
	{
		debugItemUniqueId = b3GetDebugItemUniqueId(statusHandle);
	}
	{
		PyObject* item = PyInt_FromLong(debugItemUniqueId);
		return item;
	}
}

static PyObject* pybullet_removeUserDebugItem(PyObject* self, PyObject* args, PyObject* keywds)
{
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	int itemUniqueId;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;

	static char* kwlist[] = {"itemUniqueId", "physicsClientId", NULL};

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist, &itemUniqueId, &physicsClientId))
	{
		return NULL;
	}

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	commandHandle = b3InitUserDebugDrawRemove(sm, itemUniqueId);

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_removeAllUserDebugItems(PyObject* self, PyObject* args, PyObject* keywds)
{
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	static char* kwlist[] = {"physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|i", kwlist,  &physicsClientId))
	{
		return NULL;
	}

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	commandHandle = b3InitUserDebugDrawRemoveAll(sm);

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);

	Py_INCREF(Py_None);
	return Py_None;
}


static PyObject* pybullet_removeAllUserParameters(PyObject* self, PyObject* args, PyObject* keywds)
{
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	static char* kwlist[] = {  "physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|i", kwlist, &physicsClientId))
	{
		return NULL;
	}

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	commandHandle = b3InitUserRemoveAllParameters(sm);

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_startStateLogging(PyObject* self, PyObject* args, PyObject* keywds)
{
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	b3PhysicsClientHandle sm = 0;
	int loggingType = -1;
	char* fileName = 0;
	PyObject* objectUniqueIdsObj = 0;
	int maxLogDof = -1;
	int bodyUniqueIdA = -1;
	int bodyUniqueIdB = -1;
	int linkIndexA = -2;
	int linkIndexB = -2;
	int deviceTypeFilter = -1;
	int logFlags = -1;

	static char* kwlist[] = {"loggingType", "fileName", "objectUniqueIds", "maxLogDof", "bodyUniqueIdA", "bodyUniqueIdB", "linkIndexA", "linkIndexB", "deviceTypeFilter", "logFlags", "physicsClientId", NULL};
	int physicsClientId = 0;

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "is|Oiiiiiiii", kwlist,
									 &loggingType, &fileName, &objectUniqueIdsObj, &maxLogDof, &bodyUniqueIdA, &bodyUniqueIdB, &linkIndexA, &linkIndexB, &deviceTypeFilter, &logFlags, &physicsClientId))
		return NULL;

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
	{
		b3SharedMemoryCommandHandle commandHandle;
		commandHandle = b3StateLoggingCommandInit(sm);

		b3StateLoggingStart(commandHandle, loggingType, fileName);

		if (objectUniqueIdsObj)
		{
			PyObject* seq = PySequence_Fast(objectUniqueIdsObj, "expected a sequence of object unique ids");
			if (seq)
			{
				int len = PySequence_Size(objectUniqueIdsObj);
				int i;
				for (i = 0; i < len; i++)
				{
					int objectUid = pybullet_internalGetFloatFromSequence(seq, i);
					b3StateLoggingAddLoggingObjectUniqueId(commandHandle, objectUid);
				}
				Py_DECREF(seq);
			}
		}

		if (maxLogDof > 0)
		{
			b3StateLoggingSetMaxLogDof(commandHandle, maxLogDof);
		}

		if (bodyUniqueIdA > -1)
		{
			b3StateLoggingSetBodyAUniqueId(commandHandle, bodyUniqueIdA);
		}
		if (bodyUniqueIdB > -1)
		{
			b3StateLoggingSetBodyBUniqueId(commandHandle, bodyUniqueIdB);
		}
		if (linkIndexA > -2)
		{
			b3StateLoggingSetLinkIndexA(commandHandle, linkIndexA);
		}
		if (linkIndexB > -2)
		{
			b3StateLoggingSetLinkIndexB(commandHandle, linkIndexB);
		}

		if (deviceTypeFilter >= 0)
		{
			b3StateLoggingSetDeviceTypeFilter(commandHandle, deviceTypeFilter);
		}

		if (logFlags > 0)
		{
			b3StateLoggingSetLogFlags(commandHandle, logFlags);
		}

		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
		statusType = b3GetStatusType(statusHandle);
		if (statusType == CMD_STATE_LOGGING_START_COMPLETED)
		{
			int loggingUniqueId = b3GetStatusLoggingUniqueId(statusHandle);
			PyObject* loggingUidObj = PyInt_FromLong(loggingUniqueId);
			return loggingUidObj;
		}
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_submitProfileTiming(PyObject* self, PyObject* args, PyObject* keywds)
{
	//	b3SharedMemoryStatusHandle statusHandle;
	//	int statusType;
	char* eventName = 0;
	b3PhysicsClientHandle sm = 0;
	static char* kwlist[] = {"eventName ", "physicsClientId", NULL};
	int physicsClientId = 0;
	b3SharedMemoryCommandHandle commandHandle;

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|si", kwlist,
									 &eventName, &physicsClientId))
		return NULL;

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
	
	commandHandle = b3ProfileTimingCommandInit(sm, eventName);

	if (eventName)
	{
		b3SetProfileTimingType(commandHandle, 0);
	}
	else
	{
		b3SetProfileTimingType(commandHandle, 1);
	}
	b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_stopStateLogging(PyObject* self, PyObject* args, PyObject* keywds)
{
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	int loggingId = -1;

	b3PhysicsClientHandle sm = 0;
	static char* kwlist[] = {"loggingId", "physicsClientId", NULL};
	int physicsClientId = 0;

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist,
									 &loggingId, &physicsClientId))
		return NULL;

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
	if (loggingId >= 0)
	{
		b3SharedMemoryCommandHandle commandHandle;
		commandHandle = b3StateLoggingCommandInit(sm);
		b3StateLoggingStop(commandHandle, loggingId);
		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
		statusType = b3GetStatusType(statusHandle);
	}
	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_setAdditionalSearchPath(PyObject* self, PyObject* args, PyObject* keywds)
{
	static char* kwlist[] = {"path", "physicsClientId", NULL};
	char* path = 0;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "s|i", kwlist,
									 &path, &physicsClientId))
		return NULL;
	if (path)
	{
		b3SharedMemoryCommandHandle commandHandle;
		b3SharedMemoryStatusHandle statusHandle;

		sm = getPhysicsClient(physicsClientId);
		if (sm == 0)
		{
			PyErr_SetString(SpamError, "Not connected to physics server.");
			return NULL;
		}
		commandHandle = b3SetAdditionalSearchPath(sm, path);
		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	}

	Py_INCREF(Py_None);
	return Py_None;
}

#ifdef BT_ENABLE_VHACD
static PyObject* pybullet_vhacd(PyObject* self, PyObject* args, PyObject* keywds)
{
	char* fileNameIn = 0;
	char* fileNameOut = 0;
	char* fileNameLogging = 0;
	double concavity = -1;
	double alpha = -1;
	double beta = -1;
	double gamma = -1;
	double minVolumePerCH = -1;
	int resolution = -1;
	int maxNumVerticesPerCH = -1;
	int depth = -1;
	int planeDownsampling = -1;
	int convexhullDownsampling = -1;
	int pca = -1;
	int mode = -1;
	int convexhullApproximation = -1;

	static char* kwlist[] = {"fileNameIn", "fileNameOut", "fileNameLogging", 
		"concavity", "alpha","beta","gamma","minVolumePerCH",
		"resolution","maxNumVerticesPerCH","depth","planeDownsampling",
		"convexhullDownsampling","pca","mode","convexhullApproximation",
		"physicsClientId", NULL};
	double timeOutInSeconds = -1;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "sss|dddddiiiiiiiii", kwlist,
									 &fileNameIn , &fileNameOut, &fileNameLogging , 
		&concavity, &alpha,&beta, &gamma, &minVolumePerCH,
		&resolution, &maxNumVerticesPerCH, &depth, &planeDownsampling,
		&convexhullDownsampling, &pca, &mode, &convexhullApproximation,
		&physicsClientId))
		return NULL;
	if (fileNameIn && fileNameOut)
	{
		b3VHACD(fileNameIn, fileNameOut, fileNameLogging,
			concavity, alpha, beta, gamma, minVolumePerCH,
			resolution, maxNumVerticesPerCH, depth, planeDownsampling,
			convexhullDownsampling, pca, mode, convexhullApproximation);
	}
	Py_INCREF(Py_None);
	return Py_None;
}
#endif//BT_ENABLE_VHACD


static PyObject* pybullet_setTimeOut(PyObject* self, PyObject* args, PyObject* keywds)
{
	static char* kwlist[] = {"timeOutInSeconds", "physicsClientId", NULL};
	double timeOutInSeconds = -1;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "d|i", kwlist,
									 &timeOutInSeconds, &physicsClientId))
		return NULL;
	if (timeOutInSeconds >= 0)
	{
		sm = getPhysicsClient(physicsClientId);
		if (sm == 0)
		{
			PyErr_SetString(SpamError, "Not connected to physics server.");
			return NULL;
		}
		b3SetTimeOut(sm, timeOutInSeconds);
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_rayTestObsolete(PyObject* self, PyObject* args, PyObject* keywds)
{
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	PyObject* rayFromObj = 0;
	PyObject* rayToObj = 0;
	double from[3];
	double to[3];
	b3PhysicsClientHandle sm = 0;
	int reportHitNumber = -1;
	static char* kwlist[] = {"rayFromPosition", "rayToPosition", "collisionFilterMask", "reportHitNumber", "physicsClientId", NULL};
	int physicsClientId = 0;
	int collisionFilterMask = -1;

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "OO|iii", kwlist,
									 &rayFromObj, &rayToObj, &collisionFilterMask, &reportHitNumber, &physicsClientId))
		return NULL;

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	pybullet_internalSetVectord(rayFromObj, from);
	pybullet_internalSetVectord(rayToObj, to);

	
	commandHandle = b3CreateRaycastCommandInit(sm, from[0], from[1], from[2],
											   to[0], to[1], to[2]);


	b3RaycastBatchSetCollisionFilterMask(commandHandle, collisionFilterMask);
	
	if (reportHitNumber >= 0)
	{
		b3RaycastBatchSetReportHitNumber(commandHandle, reportHitNumber);
	}
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	if (statusType == CMD_REQUEST_RAY_CAST_INTERSECTIONS_COMPLETED)
	{
		struct b3RaycastInformation raycastInfo;
		PyObject* rayHitsObj = 0;
		int i;
		b3GetRaycastInformation(sm, &raycastInfo);

		rayHitsObj = PyTuple_New(raycastInfo.m_numRayHits);
		for (i = 0; i < raycastInfo.m_numRayHits; i++)
		{
			PyObject* singleHitObj = PyTuple_New(5);
			{
				PyObject* ob = PyInt_FromLong(raycastInfo.m_rayHits[i].m_hitObjectUniqueId);
				PyTuple_SetItem(singleHitObj, 0, ob);
			}
			{
				PyObject* ob = PyInt_FromLong(raycastInfo.m_rayHits[i].m_hitObjectLinkIndex);
				PyTuple_SetItem(singleHitObj, 1, ob);
			}
			{
				PyObject* ob = PyFloat_FromDouble(raycastInfo.m_rayHits[i].m_hitFraction);
				PyTuple_SetItem(singleHitObj, 2, ob);
			}
			{
				PyObject* posObj = PyTuple_New(3);
				int p;
				for (p = 0; p < 3; p++)
				{
					PyObject* ob = PyFloat_FromDouble(raycastInfo.m_rayHits[i].m_hitPositionWorld[p]);
					PyTuple_SetItem(posObj, p, ob);
				}
				PyTuple_SetItem(singleHitObj, 3, posObj);
			}
			{
				PyObject* normalObj = PyTuple_New(3);
				int p;
				for (p = 0; p < 3; p++)
				{
					PyObject* ob = PyFloat_FromDouble(raycastInfo.m_rayHits[i].m_hitNormalWorld[p]);
					PyTuple_SetItem(normalObj, p, ob);
				}
				PyTuple_SetItem(singleHitObj, 4, normalObj);
			}
			PyTuple_SetItem(rayHitsObj, i, singleHitObj);
		}
		return rayHitsObj;
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_rayTestBatch(PyObject* self, PyObject* args, PyObject* keywds)
{
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	PyObject* rayFromObjList = 0;
	PyObject* rayToObjList = 0;
	int numThreads = 1;
	int reportHitNumber = -1;
	b3PhysicsClientHandle sm = 0;
	int sizeFrom = 0;
	int sizeTo = 0;
	int parentObjectUniqueId = -1;
	int parentLinkIndex = -1;
	int collisionFilterMask = -1;
	double fractionEpsilon = -1;

	static char* kwlist[] = {"rayFromPositions", "rayToPositions", "numThreads", "parentObjectUniqueId", "parentLinkIndex", "reportHitNumber", "collisionFilterMask","fractionEpsilon","physicsClientId", NULL};
	int physicsClientId = 0;

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "OO|iiiiidi", kwlist,
									 &rayFromObjList, &rayToObjList, &numThreads, &parentObjectUniqueId, &parentLinkIndex, &reportHitNumber, &collisionFilterMask , &fractionEpsilon, &physicsClientId))
		return NULL;

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	if (!rayFromObjList || !rayToObjList)
	{
		PyErr_SetString(SpamError, "rayFromPositions and rayToPositions must be not None.");
		return NULL;
	}

	commandHandle = b3CreateRaycastBatchCommandInit(sm);
	b3RaycastBatchSetNumThreads(commandHandle, numThreads);


	int raysAdded = 0;
#ifdef PYBULLET_USE_NUMPY
	// Faster approach if both inputs can be converted into ndarray.
	if (PyArray_Check(rayFromObjList) && PyArray_Check(rayToObjList)) {
		b3PushProfileTiming(sm, "extractPythonFromToNumpy");
		PyArrayObject* rayFromPyArrayObj = (PyArrayObject*)PyArray_FROMANY(rayFromObjList, NPY_DOUBLE, 1, 2, NPY_ARRAY_CARRAY_RO);
		PyArrayObject* rayToPyArrayObj = (PyArrayObject*)PyArray_FROMANY(rayToObjList, NPY_DOUBLE, 1, 2, NPY_ARRAY_CARRAY_RO);

		// If there is error, this will fall back to default method and error messages will be reported there.
		if (rayFromPyArrayObj && rayToPyArrayObj
			&& PyArray_SAMESHAPE(rayFromPyArrayObj, rayToPyArrayObj)
			&& PyArray_DIMS(rayFromPyArrayObj)[PyArray_NDIM(rayFromPyArrayObj) - 1] == 3)
		{
			int len = (PyArray_NDIM(rayFromPyArrayObj) == 2) ? PyArray_DIMS(rayFromPyArrayObj)[0] : 1;
			if (len <= MAX_RAY_INTERSECTION_BATCH_SIZE_STREAMING)
			{
				b3RaycastBatchAddRays(sm, commandHandle, PyArray_DATA(rayFromPyArrayObj), PyArray_DATA(rayToPyArrayObj), len);
				raysAdded = 1;
			}
		}
		if (rayFromPyArrayObj) Py_DECREF(rayFromPyArrayObj);
		if (rayToPyArrayObj) Py_DECREF(rayToPyArrayObj);
		b3PopProfileTiming(sm);
	}
#endif
	if (!raysAdded)
	{
		// go back to default method.
		PyObject* seqRayFromObj = PySequence_Fast(rayFromObjList, "expected a sequence of rayFrom positions");
		PyObject* seqRayToObj = PySequence_Fast(rayToObjList, "expected a sequence of 'rayTo' positions");

		if (seqRayFromObj && seqRayToObj)
		{
			int lenFrom = PySequence_Size(rayFromObjList);
			int lenTo = PySequence_Size(seqRayToObj);
			if (lenFrom != lenTo)
			{
				PyErr_SetString(SpamError, "Size of from_positions need to be equal to size of to_positions.");
				Py_DECREF(seqRayFromObj);
				Py_DECREF(seqRayToObj);
				return NULL;
			}
			else
			{
				int i;

				if (lenFrom > MAX_RAY_INTERSECTION_BATCH_SIZE_STREAMING)
				{
					PyErr_SetString(SpamError, "Number of rays exceed the maximum batch size.");
					Py_DECREF(seqRayFromObj);
					Py_DECREF(seqRayToObj);
					return NULL;
				}
				b3PushProfileTiming(sm, "extractPythonFromToSequenceToC");
				for (i = 0; i < lenFrom; i++)
				{
					PyObject* rayFromObj = PySequence_GetItem(rayFromObjList, i);
					PyObject* rayToObj = PySequence_GetItem(seqRayToObj, i);
					double rayFromWorld[3];
					double rayToWorld[3];

					if ((pybullet_internalSetVectord(rayFromObj, rayFromWorld)) &&
						(pybullet_internalSetVectord(rayToObj, rayToWorld)))
					{
						//todo: better to upload all rays at once
						//b3RaycastBatchAddRay(commandHandle, rayFromWorld, rayToWorld);
						b3RaycastBatchAddRays(sm, commandHandle, rayFromWorld, rayToWorld, 1);
					}
					else
					{
						PyErr_SetString(SpamError, "Items in the from/to positions need to be an [x,y,z] list of 3 floats/doubles");
						Py_DECREF(seqRayFromObj);
						Py_DECREF(seqRayToObj);
						Py_DECREF(rayFromObj);
						Py_DECREF(rayToObj);
						b3PopProfileTiming(sm);
						return NULL;
					}
					Py_DECREF(rayFromObj);
					Py_DECREF(rayToObj);
				}
				b3PopProfileTiming(sm);
			}
		}
		else
		{
		}
		if (seqRayFromObj)
		{
			Py_DECREF(seqRayFromObj);
		}
		if (seqRayToObj)
		{
			Py_DECREF(seqRayToObj);
		}
	}

	if (parentObjectUniqueId >= 0)
	{
		b3RaycastBatchSetParentObject(commandHandle, parentObjectUniqueId, parentLinkIndex);
	}
	if (reportHitNumber >= 0)
	{
		b3RaycastBatchSetReportHitNumber(commandHandle, reportHitNumber);
	}
	b3RaycastBatchSetCollisionFilterMask(commandHandle, collisionFilterMask);

	if (fractionEpsilon >= 0)
	{
		b3RaycastBatchSetFractionEpsilon(commandHandle, fractionEpsilon);
		
	}
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	if (statusType == CMD_REQUEST_RAY_CAST_INTERSECTIONS_COMPLETED)
	{
		struct b3RaycastInformation raycastInfo;
		PyObject* rayHitsObj = 0;
		int i;
		b3PushProfileTiming(sm, "convertRaycastInformationToPython");
		b3GetRaycastInformation(sm, &raycastInfo);

		rayHitsObj = PyTuple_New(raycastInfo.m_numRayHits);
		for (i = 0; i < raycastInfo.m_numRayHits; i++)
		{
			PyObject* singleHitObj = PyTuple_New(5);
			{
				PyObject* ob = PyInt_FromLong(raycastInfo.m_rayHits[i].m_hitObjectUniqueId);
				PyTuple_SetItem(singleHitObj, 0, ob);
			}
			{
				PyObject* ob = PyInt_FromLong(raycastInfo.m_rayHits[i].m_hitObjectLinkIndex);
				PyTuple_SetItem(singleHitObj, 1, ob);
			}
			{
				PyObject* ob = PyFloat_FromDouble(raycastInfo.m_rayHits[i].m_hitFraction);
				PyTuple_SetItem(singleHitObj, 2, ob);
			}
			{
				PyObject* posObj = PyTuple_New(3);
				int p;
				for (p = 0; p < 3; p++)
				{
					PyObject* ob = PyFloat_FromDouble(raycastInfo.m_rayHits[i].m_hitPositionWorld[p]);
					PyTuple_SetItem(posObj, p, ob);
				}
				PyTuple_SetItem(singleHitObj, 3, posObj);
			}
			{
				PyObject* normalObj = PyTuple_New(3);
				int p;
				for (p = 0; p < 3; p++)
				{
					PyObject* ob = PyFloat_FromDouble(raycastInfo.m_rayHits[i].m_hitNormalWorld[p]);
					PyTuple_SetItem(normalObj, p, ob);
				}
				PyTuple_SetItem(singleHitObj, 4, normalObj);
			}
			PyTuple_SetItem(rayHitsObj, i, singleHitObj);
		}
		b3PopProfileTiming(sm);
		return rayHitsObj;
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_getMatrixFromQuaternion(PyObject* self, PyObject* args, PyObject* keywds)
{
	PyObject* quatObj;
	double quat[4];
	int physicsClientId = 0;
	static char* kwlist[] = {"quaternion", "physicsClientId", NULL};

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "O|i", kwlist, &quatObj, &physicsClientId))
	{
		return NULL;
	}

	if (quatObj)
	{
		if (pybullet_internalSetVector4d(quatObj, quat))
		{
			///see btMatrix3x3::setRotation
			int i;
			double d = quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3];
			double s = 2.0 / d;
			double xs = quat[0] * s, ys = quat[1] * s, zs = quat[2] * s;
			double wx = quat[3] * xs, wy = quat[3] * ys, wz = quat[3] * zs;
			double xx = quat[0] * xs, xy = quat[0] * ys, xz = quat[0] * zs;
			double yy = quat[1] * ys, yz = quat[1] * zs, zz = quat[2] * zs;
			double mat3x3[9] = {
				1.0 - (yy + zz), xy - wz, xz + wy,
				xy + wz, 1.0 - (xx + zz), yz - wx,
				xz - wy, yz + wx, 1.0 - (xx + yy)};
			PyObject* matObj = PyTuple_New(9);
			for (i = 0; i < 9; i++)
			{
				PyTuple_SetItem(matObj, i, PyFloat_FromDouble(mat3x3[i]));
			}
			return matObj;
		}
	}
	PyErr_SetString(SpamError, "Couldn't convert quaternion [x,y,z,w].");
	return NULL;
};

static PyObject* pybullet_setVRCameraState(PyObject* self, PyObject* args, PyObject* keywds)
{
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	PyObject* rootPosObj = 0;
	PyObject* rootOrnObj = 0;
	int trackObjectUid = -2;
	int trackObjectFlag = -1;
	double rootPos[3];
	double rootOrn[4];

	static char* kwlist[] = {"rootPosition", "rootOrientation", "trackObject", "trackObjectFlag", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|OOiii", kwlist, &rootPosObj, &rootOrnObj, &trackObjectUid, &trackObjectFlag, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	commandHandle = b3SetVRCameraStateCommandInit(sm);

	if (pybullet_internalSetVectord(rootPosObj, rootPos))
	{
		b3SetVRCameraRootPosition(commandHandle, rootPos);
	}
	if (pybullet_internalSetVector4d(rootOrnObj, rootOrn))
	{
		b3SetVRCameraRootOrientation(commandHandle, rootOrn);
	}

	if (trackObjectUid >= -1)
	{
		b3SetVRCameraTrackingObject(commandHandle, trackObjectUid);
	}

	if (trackObjectFlag >= -1)
	{
		b3SetVRCameraTrackingObjectFlag(commandHandle, trackObjectFlag);
	}

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_getKeyboardEvents(PyObject* self, PyObject* args, PyObject* keywds)
{
	b3SharedMemoryCommandHandle commandHandle;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	struct b3KeyboardEventsData keyboardEventsData;
	PyObject* keyEventsObj = 0;
	int i = 0;

	static char* kwlist[] = {"physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|i", kwlist, &physicsClientId))
	{
		return NULL;
	}

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	commandHandle = b3RequestKeyboardEventsCommandInit(sm);
	b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	b3GetKeyboardEventsData(sm, &keyboardEventsData);

	keyEventsObj = PyDict_New();

	for (i = 0; i < keyboardEventsData.m_numKeyboardEvents; i++)
	{
		PyObject* keyObj = PyLong_FromLong(keyboardEventsData.m_keyboardEvents[i].m_keyCode);
		PyObject* valObj = PyLong_FromLong(keyboardEventsData.m_keyboardEvents[i].m_keyState);
		PyDict_SetItem(keyEventsObj, keyObj, valObj);
	}
	return keyEventsObj;
}

static PyObject* pybullet_getMouseEvents(PyObject* self, PyObject* args, PyObject* keywds)
{
	b3SharedMemoryCommandHandle commandHandle;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	struct b3MouseEventsData mouseEventsData;
	PyObject* mouseEventsObj = 0;
	int i = 0;

	static char* kwlist[] = {"physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|i", kwlist, &physicsClientId))
	{
		return NULL;
	}

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	commandHandle = b3RequestMouseEventsCommandInit(sm);
	b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	b3GetMouseEventsData(sm, &mouseEventsData);

	mouseEventsObj = PyTuple_New(mouseEventsData.m_numMouseEvents);

	for (i = 0; i < mouseEventsData.m_numMouseEvents; i++)
	{
		PyObject* mouseEventObj = PyTuple_New(5);
		PyTuple_SetItem(mouseEventObj, 0, PyInt_FromLong(mouseEventsData.m_mouseEvents[i].m_eventType));
		PyTuple_SetItem(mouseEventObj, 1, PyFloat_FromDouble(mouseEventsData.m_mouseEvents[i].m_mousePosX));
		PyTuple_SetItem(mouseEventObj, 2, PyFloat_FromDouble(mouseEventsData.m_mouseEvents[i].m_mousePosY));
		PyTuple_SetItem(mouseEventObj, 3, PyInt_FromLong(mouseEventsData.m_mouseEvents[i].m_buttonIndex));
		PyTuple_SetItem(mouseEventObj, 4, PyInt_FromLong(mouseEventsData.m_mouseEvents[i].m_buttonState));
		PyTuple_SetItem(mouseEventsObj, i, mouseEventObj);
	}
	return mouseEventsObj;
}

static PyObject* pybullet_getVREvents(PyObject* self, PyObject* args, PyObject* keywds)
{
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	int deviceTypeFilter = VR_DEVICE_CONTROLLER;
	int physicsClientId = 0;
	int allAnalogAxes = 0;
	b3PhysicsClientHandle sm = 0;
	static char* kwlist[] = {"deviceTypeFilter", "allAnalogAxes", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|iii", kwlist, &deviceTypeFilter, &allAnalogAxes, &physicsClientId))
	{
		return NULL;
	}

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	commandHandle = b3RequestVREventsCommandInit(sm);

	b3VREventsSetDeviceTypeFilter(commandHandle, deviceTypeFilter);

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	if (statusType == CMD_REQUEST_VR_EVENTS_DATA_COMPLETED)
	{
		struct b3VREventsData vrEvents;
		PyObject* vrEventsObj;
		int i = 0;
		b3GetVREventsData(sm, &vrEvents);

		vrEventsObj = PyTuple_New(vrEvents.m_numControllerEvents);
		for (i = 0; i < vrEvents.m_numControllerEvents; i++)
		{
			int numFields = allAnalogAxes ? 9 : 8;
			PyObject* vrEventObj = PyTuple_New(numFields);

			PyTuple_SetItem(vrEventObj, 0, PyInt_FromLong(vrEvents.m_controllerEvents[i].m_controllerId));
			{
				PyObject* posObj = PyTuple_New(3);
				PyTuple_SetItem(posObj, 0, PyFloat_FromDouble(vrEvents.m_controllerEvents[i].m_pos[0]));
				PyTuple_SetItem(posObj, 1, PyFloat_FromDouble(vrEvents.m_controllerEvents[i].m_pos[1]));
				PyTuple_SetItem(posObj, 2, PyFloat_FromDouble(vrEvents.m_controllerEvents[i].m_pos[2]));
				PyTuple_SetItem(vrEventObj, 1, posObj);
			}
			{
				PyObject* ornObj = PyTuple_New(4);
				PyTuple_SetItem(ornObj, 0, PyFloat_FromDouble(vrEvents.m_controllerEvents[i].m_orn[0]));
				PyTuple_SetItem(ornObj, 1, PyFloat_FromDouble(vrEvents.m_controllerEvents[i].m_orn[1]));
				PyTuple_SetItem(ornObj, 2, PyFloat_FromDouble(vrEvents.m_controllerEvents[i].m_orn[2]));
				PyTuple_SetItem(ornObj, 3, PyFloat_FromDouble(vrEvents.m_controllerEvents[i].m_orn[3]));
				PyTuple_SetItem(vrEventObj, 2, ornObj);
			}

			PyTuple_SetItem(vrEventObj, 3, PyFloat_FromDouble(vrEvents.m_controllerEvents[i].m_analogAxis));
			PyTuple_SetItem(vrEventObj, 4, PyInt_FromLong(vrEvents.m_controllerEvents[i].m_numButtonEvents));
			PyTuple_SetItem(vrEventObj, 5, PyInt_FromLong(vrEvents.m_controllerEvents[i].m_numMoveEvents));
			{
				PyObject* buttonsObj = PyTuple_New(MAX_VR_BUTTONS);
				int b;
				for (b = 0; b < MAX_VR_BUTTONS; b++)
				{
					PyObject* button = PyInt_FromLong(vrEvents.m_controllerEvents[i].m_buttons[b]);
					PyTuple_SetItem(buttonsObj, b, button);
				}
				PyTuple_SetItem(vrEventObj, 6, buttonsObj);
			}
			PyTuple_SetItem(vrEventObj, 7, PyInt_FromLong(vrEvents.m_controllerEvents[i].m_deviceType));

			if (allAnalogAxes)
			{
				PyObject* buttonsObj = PyTuple_New(MAX_VR_ANALOG_AXIS * 2);
				int b;
				for (b = 0; b < MAX_VR_ANALOG_AXIS * 2; b++)
				{
					PyObject* axisVal = PyFloat_FromDouble(vrEvents.m_controllerEvents[i].m_auxAnalogAxis[b]);
					PyTuple_SetItem(buttonsObj, b, axisVal);
				}
				PyTuple_SetItem(vrEventObj, 8, buttonsObj);
			}

			PyTuple_SetItem(vrEventsObj, i, vrEventObj);
		}
		return vrEventsObj;
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_getDebugVisualizerCamera(PyObject* self, PyObject* args, PyObject* keywds)
{
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	static char* kwlist[] = {"physicsClientId", NULL};
	b3SharedMemoryCommandHandle commandHandle;
	int hasCamInfo;
	b3SharedMemoryStatusHandle statusHandle;
	struct b3OpenGLVisualizerCameraInfo camera;
        int i;
        camera.m_width=0;
        camera.m_height=0;
        camera.m_dist=0;
        camera.m_yaw=0;
        camera.m_pitch=0;

        for (i=0;i<16;i++)
        {
          camera.m_viewMatrix[i]=0;
          camera.m_projectionMatrix[i]=0;
        }
        for (i=0;i<3;i++)
        {
          camera.m_camUp[i]=0;
          camera.m_camForward[i]=0;
          camera.m_horizontal[i]=0;
          camera.m_vertical[i]=0;
          camera.m_target[i]=0;
        }
        PyObject* pyCameraList = 0;

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	commandHandle = b3InitRequestOpenGLVisualizerCameraCommand(sm);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);

	hasCamInfo = b3GetStatusOpenGLVisualizerCamera(statusHandle, &camera);
	if (1)
	{
		PyObject* item = 0;
		pyCameraList = PyTuple_New(12);
		item = PyInt_FromLong(camera.m_width);
		PyTuple_SetItem(pyCameraList, 0, item);
		item = PyInt_FromLong(camera.m_height);
		PyTuple_SetItem(pyCameraList, 1, item);
		{
			PyObject* viewMat16 = PyTuple_New(16);
			PyObject* projMat16 = PyTuple_New(16);
			int i;

			for (i = 0; i < 16; i++)
			{
				item = PyFloat_FromDouble(camera.m_viewMatrix[i]);
				PyTuple_SetItem(viewMat16, i, item);
				item = PyFloat_FromDouble(camera.m_projectionMatrix[i]);
				PyTuple_SetItem(projMat16, i, item);
			}
			PyTuple_SetItem(pyCameraList, 2, viewMat16);
			PyTuple_SetItem(pyCameraList, 3, projMat16);
		}

		{
			PyObject* item = 0;
			int i;
			PyObject* camUp = PyTuple_New(3);
			PyObject* camFwd = PyTuple_New(3);
			PyObject* hor = PyTuple_New(3);
			PyObject* vert = PyTuple_New(3);
			for (i = 0; i < 3; i++)
			{
				item = PyFloat_FromDouble(camera.m_camUp[i]);
				PyTuple_SetItem(camUp, i, item);
				item = PyFloat_FromDouble(camera.m_camForward[i]);
				PyTuple_SetItem(camFwd, i, item);
				item = PyFloat_FromDouble(camera.m_horizontal[i]);
				PyTuple_SetItem(hor, i, item);
				item = PyFloat_FromDouble(camera.m_vertical[i]);
				PyTuple_SetItem(vert, i, item);
			}
			PyTuple_SetItem(pyCameraList, 4, camUp);
			PyTuple_SetItem(pyCameraList, 5, camFwd);
			PyTuple_SetItem(pyCameraList, 6, hor);
			PyTuple_SetItem(pyCameraList, 7, vert);
		}
		item = PyFloat_FromDouble(camera.m_yaw);
		PyTuple_SetItem(pyCameraList, 8, item);
		item = PyFloat_FromDouble(camera.m_pitch);
		PyTuple_SetItem(pyCameraList, 9, item);
		item = PyFloat_FromDouble(camera.m_dist);
		PyTuple_SetItem(pyCameraList, 10, item);
		{
			PyObject* item = 0;
			int i;
			PyObject* camTarget = PyTuple_New(3);
			for (i = 0; i < 3; i++)
			{
				item = PyFloat_FromDouble(camera.m_target[i]);
				PyTuple_SetItem(camTarget, i, item);
			}
			PyTuple_SetItem(pyCameraList, 11, camTarget);
		}
		return pyCameraList;
	}

	PyErr_SetString(SpamError, "Cannot get OpenGL visualizer camera info.");
	return NULL;
}

static PyObject* pybullet_configureDebugVisualizer(PyObject* self, PyObject* args, PyObject* keywds)
{
	int flag = -1;
	int enable = -1;
	int shadowMapResolution = -1;
	double shadowMapIntensity = -1;
	int shadowMapWorldSize = -1;
	int physicsClientId = 0;
	double remoteSyncTransformInterval = -1;
	PyObject* pyLightPosition = 0;
	b3PhysicsClientHandle sm = 0;
	static char* kwlist[] = {"flag", "enable", "lightPosition", "shadowMapResolution", "shadowMapWorldSize", "remoteSyncTransformInterval", "shadowMapIntensity", "physicsClientId", NULL};

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|iiOiiddi", kwlist,
									 &flag, &enable, &pyLightPosition, &shadowMapResolution, &shadowMapWorldSize, &remoteSyncTransformInterval, &shadowMapIntensity,  &physicsClientId))
		return NULL;

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	{
		b3SharedMemoryCommandHandle commandHandle = b3InitConfigureOpenGLVisualizer(sm);
		if (flag >= 0)
		{
			b3ConfigureOpenGLVisualizerSetVisualizationFlags(commandHandle, flag, enable);
		}
		if (pyLightPosition)
		{
			float lightPosition[3];
			if (pybullet_internalSetVector(pyLightPosition, lightPosition))
			{
				b3ConfigureOpenGLVisualizerSetLightPosition(commandHandle, lightPosition);
			}
		}
		if (shadowMapIntensity >= 0)
		{
			b3ConfigureOpenGLVisualizerSetShadowMapIntensity(commandHandle, shadowMapIntensity);
		}


		if (shadowMapResolution > 0)
		{
			b3ConfigureOpenGLVisualizerSetShadowMapResolution(commandHandle, shadowMapResolution);
		}
		if (shadowMapWorldSize > 0)
		{
			b3ConfigureOpenGLVisualizerSetShadowMapWorldSize(commandHandle, shadowMapWorldSize);
		}
		if (remoteSyncTransformInterval >= 0)
		{
			b3ConfigureOpenGLVisualizerSetRemoteSyncTransformInterval(commandHandle, remoteSyncTransformInterval);
		}

		b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	}
	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_resetDebugVisualizerCamera(PyObject* self, PyObject* args, PyObject* keywds)
{
	float cameraDistance = -1;
	float cameraYaw = 35;
	float cameraPitch = 50;
	PyObject* cameraTargetPosObj = 0;

	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	static char* kwlist[] = {"cameraDistance", "cameraYaw", "cameraPitch", "cameraTargetPosition", "physicsClientId", NULL};

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "fffO|i", kwlist,
									 &cameraDistance, &cameraYaw, &cameraPitch, &cameraTargetPosObj, &physicsClientId))
		return NULL;

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	{
		b3SharedMemoryCommandHandle commandHandle = b3InitConfigureOpenGLVisualizer(sm);
		if ((cameraDistance >= 0))
		{
			float cameraTargetPosition[3];
			if (pybullet_internalSetVector(cameraTargetPosObj, cameraTargetPosition))
			{
				b3ConfigureOpenGLVisualizerSetViewMatrix(commandHandle, cameraDistance, cameraPitch, cameraYaw, cameraTargetPosition);
			}
		}
		b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	}
	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_setDebugObjectColor(PyObject* self, PyObject* args, PyObject* keywds)
{
	PyObject* objectColorRGBObj = 0;
	double objectColorRGB[3];

	int objectUniqueId = -1;
	int linkIndex = -2;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	static char* kwlist[] = {"objectUniqueId", "linkIndex", "objectDebugColorRGB", "physicsClientId", NULL};

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "ii|Oi", kwlist,
									 &objectUniqueId, &linkIndex, &objectColorRGBObj, &physicsClientId))
		return NULL;

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	if (objectColorRGBObj)
	{
		if (pybullet_internalSetVectord(objectColorRGBObj, objectColorRGB))
		{
			b3SharedMemoryCommandHandle commandHandle = b3InitDebugDrawingCommand(sm);
			b3SetDebugObjectColor(commandHandle, objectUniqueId, linkIndex, objectColorRGB);
			b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
		}
	}
	else
	{
		b3SharedMemoryCommandHandle commandHandle = b3InitDebugDrawingCommand(sm);
		b3RemoveDebugObjectColor(commandHandle, objectUniqueId, linkIndex);
		b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	}
	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_getCollisionShapeData(PyObject* self, PyObject* args, PyObject* keywds)
{
	int objectUniqueId = -1;
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	struct b3CollisionShapeInformation collisionShapeInfo;
	int statusType;
	int i;
	int linkIndex;
	PyObject* pyResultList = 0;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	static char* kwlist[] = {"objectUniqueId", "linkIndex", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "ii|i", kwlist, &objectUniqueId, &linkIndex, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	{
		commandHandle = b3InitRequestCollisionShapeInformation(sm, objectUniqueId, linkIndex);
		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
		statusType = b3GetStatusType(statusHandle);
		if (statusType == CMD_COLLISION_SHAPE_INFO_COMPLETED)
		{
			b3GetCollisionShapeInformation(sm, &collisionShapeInfo);
			pyResultList = PyTuple_New(collisionShapeInfo.m_numCollisionShapes);
			for (i = 0; i < collisionShapeInfo.m_numCollisionShapes; i++)
			{
				PyObject* collisionShapeObList = PyTuple_New(7);
				PyObject* item;

				item = PyInt_FromLong(collisionShapeInfo.m_collisionShapeData[i].m_objectUniqueId);
				PyTuple_SetItem(collisionShapeObList, 0, item);

				item = PyInt_FromLong(collisionShapeInfo.m_collisionShapeData[i].m_linkIndex);
				PyTuple_SetItem(collisionShapeObList, 1, item);

				item = PyInt_FromLong(collisionShapeInfo.m_collisionShapeData[i].m_collisionGeometryType);
				PyTuple_SetItem(collisionShapeObList, 2, item);

				{
					PyObject* vec = PyTuple_New(3);
					item = PyFloat_FromDouble(collisionShapeInfo.m_collisionShapeData[i].m_dimensions[0]);
					PyTuple_SetItem(vec, 0, item);
					item = PyFloat_FromDouble(collisionShapeInfo.m_collisionShapeData[i].m_dimensions[1]);
					PyTuple_SetItem(vec, 1, item);
					item = PyFloat_FromDouble(collisionShapeInfo.m_collisionShapeData[i].m_dimensions[2]);
					PyTuple_SetItem(vec, 2, item);
					PyTuple_SetItem(collisionShapeObList, 3, vec);
				}

				item = PyString_FromString(collisionShapeInfo.m_collisionShapeData[i].m_meshAssetFileName);
				PyTuple_SetItem(collisionShapeObList, 4, item);

				{
					PyObject* vec = PyTuple_New(3);
					item = PyFloat_FromDouble(collisionShapeInfo.m_collisionShapeData[i].m_localCollisionFrame[0]);
					PyTuple_SetItem(vec, 0, item);
					item = PyFloat_FromDouble(collisionShapeInfo.m_collisionShapeData[i].m_localCollisionFrame[1]);
					PyTuple_SetItem(vec, 1, item);
					item = PyFloat_FromDouble(collisionShapeInfo.m_collisionShapeData[i].m_localCollisionFrame[2]);
					PyTuple_SetItem(vec, 2, item);
					PyTuple_SetItem(collisionShapeObList, 5, vec);
				}

				{
					PyObject* vec = PyTuple_New(4);
					item = PyFloat_FromDouble(collisionShapeInfo.m_collisionShapeData[i].m_localCollisionFrame[3]);
					PyTuple_SetItem(vec, 0, item);
					item = PyFloat_FromDouble(collisionShapeInfo.m_collisionShapeData[i].m_localCollisionFrame[4]);
					PyTuple_SetItem(vec, 1, item);
					item = PyFloat_FromDouble(collisionShapeInfo.m_collisionShapeData[i].m_localCollisionFrame[5]);
					PyTuple_SetItem(vec, 2, item);
					item = PyFloat_FromDouble(collisionShapeInfo.m_collisionShapeData[i].m_localCollisionFrame[6]);
					PyTuple_SetItem(vec, 3, item);
					PyTuple_SetItem(collisionShapeObList, 6, vec);
				}

				PyTuple_SetItem(pyResultList, i, collisionShapeObList);
			}
			return pyResultList;
		}
		else
		{
			PyErr_SetString(SpamError, "Error receiving collision shape info");
			return NULL;
		}
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_getVisualShapeData(PyObject* self, PyObject* args, PyObject* keywds)
{
	int objectUniqueId = -1;
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	struct b3VisualShapeInformation visualShapeInfo;
	int statusType;
	int i;
	PyObject* pyResultList = 0;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	int flags = 0;

	static char* kwlist[] = {"objectUniqueId", "flags", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|ii", kwlist, &objectUniqueId, &flags, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	{
		commandHandle = b3InitRequestVisualShapeInformation(sm, objectUniqueId);
		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
		statusType = b3GetStatusType(statusHandle);
		if (statusType == CMD_VISUAL_SHAPE_INFO_COMPLETED)
		{
			b3GetVisualShapeInformation(sm, &visualShapeInfo);
			pyResultList = PyTuple_New(visualShapeInfo.m_numVisualShapes);
			for (i = 0; i < visualShapeInfo.m_numVisualShapes; i++)
			{
				int numFields = flags & eVISUAL_SHAPE_DATA_TEXTURE_UNIQUE_IDS ? 9 : 8;

				PyObject* visualShapeObList = PyTuple_New(numFields);
				PyObject* item;
				item = PyInt_FromLong(visualShapeInfo.m_visualShapeData[i].m_objectUniqueId);
				PyTuple_SetItem(visualShapeObList, 0, item);

				item = PyInt_FromLong(visualShapeInfo.m_visualShapeData[i].m_linkIndex);
				PyTuple_SetItem(visualShapeObList, 1, item);

				item = PyInt_FromLong(visualShapeInfo.m_visualShapeData[i].m_visualGeometryType);
				PyTuple_SetItem(visualShapeObList, 2, item);

				{
					PyObject* vec = PyTuple_New(3);
					item = PyFloat_FromDouble(visualShapeInfo.m_visualShapeData[i].m_dimensions[0]);
					PyTuple_SetItem(vec, 0, item);
					item = PyFloat_FromDouble(visualShapeInfo.m_visualShapeData[i].m_dimensions[1]);
					PyTuple_SetItem(vec, 1, item);
					item = PyFloat_FromDouble(visualShapeInfo.m_visualShapeData[i].m_dimensions[2]);
					PyTuple_SetItem(vec, 2, item);
					PyTuple_SetItem(visualShapeObList, 3, vec);
				}

				item = PyString_FromString(visualShapeInfo.m_visualShapeData[i].m_meshAssetFileName);
				PyTuple_SetItem(visualShapeObList, 4, item);

				{
					PyObject* vec = PyTuple_New(3);
					item = PyFloat_FromDouble(visualShapeInfo.m_visualShapeData[i].m_localVisualFrame[0]);
					PyTuple_SetItem(vec, 0, item);
					item = PyFloat_FromDouble(visualShapeInfo.m_visualShapeData[i].m_localVisualFrame[1]);
					PyTuple_SetItem(vec, 1, item);
					item = PyFloat_FromDouble(visualShapeInfo.m_visualShapeData[i].m_localVisualFrame[2]);
					PyTuple_SetItem(vec, 2, item);
					PyTuple_SetItem(visualShapeObList, 5, vec);
				}

				{
					PyObject* vec = PyTuple_New(4);
					item = PyFloat_FromDouble(visualShapeInfo.m_visualShapeData[i].m_localVisualFrame[3]);
					PyTuple_SetItem(vec, 0, item);
					item = PyFloat_FromDouble(visualShapeInfo.m_visualShapeData[i].m_localVisualFrame[4]);
					PyTuple_SetItem(vec, 1, item);
					item = PyFloat_FromDouble(visualShapeInfo.m_visualShapeData[i].m_localVisualFrame[5]);
					PyTuple_SetItem(vec, 2, item);
					item = PyFloat_FromDouble(visualShapeInfo.m_visualShapeData[i].m_localVisualFrame[6]);
					PyTuple_SetItem(vec, 3, item);
					PyTuple_SetItem(visualShapeObList, 6, vec);
				}

				{
					PyObject* rgba = PyTuple_New(4);
					item = PyFloat_FromDouble(visualShapeInfo.m_visualShapeData[i].m_rgbaColor[0]);
					PyTuple_SetItem(rgba, 0, item);
					item = PyFloat_FromDouble(visualShapeInfo.m_visualShapeData[i].m_rgbaColor[1]);
					PyTuple_SetItem(rgba, 1, item);
					item = PyFloat_FromDouble(visualShapeInfo.m_visualShapeData[i].m_rgbaColor[2]);
					PyTuple_SetItem(rgba, 2, item);
					item = PyFloat_FromDouble(visualShapeInfo.m_visualShapeData[i].m_rgbaColor[3]);
					PyTuple_SetItem(rgba, 3, item);
					PyTuple_SetItem(visualShapeObList, 7, rgba);
				}
				if (flags & eVISUAL_SHAPE_DATA_TEXTURE_UNIQUE_IDS)
				{
					item = PyInt_FromLong(visualShapeInfo.m_visualShapeData[i].m_textureUniqueId);
					PyTuple_SetItem(visualShapeObList, 8, item);
				}

				PyTuple_SetItem(pyResultList, i, visualShapeObList);
			}
			return pyResultList;
		}
		else
		{
			PyErr_SetString(SpamError, "Error receiving visual shape info");
			return NULL;
		}
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_changeVisualShape(PyObject* self, PyObject* args, PyObject* keywds)
{
	int objectUniqueId = -1;
	int jointIndex = -1;
	int shapeIndex = -1;
	int textureUniqueId = -2;
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	int physicsClientId = 0;
	PyObject* rgbaColorObj = 0;
	PyObject* specularColorObj = 0;
	int flags = -1;
	b3PhysicsClientHandle sm = 0;
	static char* kwlist[] = {"objectUniqueId", "linkIndex", "shapeIndex", "textureUniqueId", "rgbaColor", "specularColor", "flags", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "ii|iiOOii", kwlist, &objectUniqueId, &jointIndex, &shapeIndex, &textureUniqueId, &rgbaColorObj, &specularColorObj, &flags , &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	{
		commandHandle = b3InitUpdateVisualShape2(sm, objectUniqueId, jointIndex, shapeIndex);

		if (textureUniqueId >= -1)
		{
			b3UpdateVisualShapeTexture(commandHandle, textureUniqueId);
		}

		if (specularColorObj)
		{
			double specularColor[3] = {1, 1, 1};
			pybullet_internalSetVectord(specularColorObj, specularColor);
			b3UpdateVisualShapeSpecularColor(commandHandle, specularColor);
		}

		if (rgbaColorObj)
		{
			double rgbaColor[4] = {1, 1, 1, 1};
			pybullet_internalSetVector4d(rgbaColorObj, rgbaColor);
			b3UpdateVisualShapeRGBAColor(commandHandle, rgbaColor);
		}
		if (flags >= 0)
		{
			b3UpdateVisualShapeFlags(commandHandle, flags);
		}

		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
		statusType = b3GetStatusType(statusHandle);
		if (statusType == CMD_VISUAL_SHAPE_UPDATE_COMPLETED)
		{
		}
		else
		{
			PyErr_SetString(SpamError, "Error resetting visual shape info");
			return NULL;
		}
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_changeTexture(PyObject* self, PyObject* args, PyObject* keywds)
{
	b3SharedMemoryCommandHandle commandHandle = 0;
	b3SharedMemoryStatusHandle statusHandle = 0;
	int statusType = -1;
	int textureUniqueId = -1;
	int physicsClientId = 0;
	int width = -1;
	int height = -1;

	PyObject* pixelsObj = 0;

	b3PhysicsClientHandle sm = 0;
	static char* kwlist[] = {"textureUniqueId", "pixels", "width", "height", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "iOii|i", kwlist, &textureUniqueId, &pixelsObj, &width, &height, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	if (textureUniqueId >= 0 && width >= 0 && height >= 0 && pixelsObj)
	{
		PyObject* seqPixels = PySequence_Fast(pixelsObj, "expected a sequence");
		PyObject* item;
		int i;
		int numPixels = width * height;
		unsigned char* pixelBuffer = (unsigned char*)malloc(numPixels * 3);
		if (PyList_Check(seqPixels))
		{
			for (i = 0; i < numPixels * 3; i++)
			{
				item = PyList_GET_ITEM(seqPixels, i);
				pixelBuffer[i] = PyLong_AsLong(item);
			}
		}
		else
		{
			for (i = 0; i < numPixels * 3; i++)
			{
				item = PyTuple_GET_ITEM(seqPixels, i);
				pixelBuffer[i] = PyLong_AsLong(item);
			}
		}

		Py_DECREF(seqPixels);
		commandHandle = b3CreateChangeTextureCommandInit(sm, textureUniqueId, width, height, (const char*)pixelBuffer);
		free(pixelBuffer);
		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
		statusType = b3GetStatusType(statusHandle);
		if (statusType == CMD_CLIENT_COMMAND_COMPLETED)
		{
			Py_INCREF(Py_None);
			return Py_None;
		}
		else
		{
			PyErr_SetString(SpamError, "Error processing changeTexture.");
			return NULL;
		}
	}

	PyErr_SetString(SpamError, "Error: invalid arguments in changeTexture.");
	return NULL;
}

static PyObject* pybullet_loadTexture(PyObject* self, PyObject* args, PyObject* keywds)
{
	const char* filename = 0;
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	static char* kwlist[] = {"textureFilename", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "s|i", kwlist, &filename, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	{
		commandHandle = b3InitLoadTexture(sm, filename);
		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
		statusType = b3GetStatusType(statusHandle);
		if (statusType == CMD_LOAD_TEXTURE_COMPLETED)
		{
			PyObject* item;
			item = PyInt_FromLong(b3GetStatusTextureUniqueId(statusHandle));
			return item;
		}
	}

	PyErr_SetString(SpamError, "Error loading texture");
	return NULL;
}

static PyObject* MyConvertContactPoint(struct b3ContactInformation* contactPointPtr)
{
	/*
	 0     int m_contactFlags;
	 1     int m_bodyUniqueIdA;
	 2     int m_bodyUniqueIdB;
	 3     int m_linkIndexA;
	 4     int m_linkIndexB;
	 5		double m_positionOnAInWS[3];//contact point location on object A,
	 in world space coordinates
	 6		double m_positionOnBInWS[3];//contact point location on object
	 A, in world space coordinates
	 7		double m_contactNormalOnBInWS[3];//the separating contact
	 normal, pointing from object B towards object A
	 8		double m_contactDistance;//negative number is penetration, positive
	 is distance.
	 9		double m_normalForce;
	 10     double m_linearFrictionForce1;
	 11     double double m_linearFrictionDirection1[3];
	 12     double m_linearFrictionForce2;
	 13     double double m_linearFrictionDirection2[3];
	 */

	int i;

	PyObject* pyResultList = PyTuple_New(contactPointPtr->m_numContactPoints);
	for (i = 0; i < contactPointPtr->m_numContactPoints; i++)
	{
		PyObject* contactObList = PyTuple_New(14);  // see above 10 fields
		PyObject* item;
		item =
			PyInt_FromLong(contactPointPtr->m_contactPointData[i].m_contactFlags);
		PyTuple_SetItem(contactObList, 0, item);
		item = PyInt_FromLong(
			contactPointPtr->m_contactPointData[i].m_bodyUniqueIdA);
		PyTuple_SetItem(contactObList, 1, item);
		item = PyInt_FromLong(
			contactPointPtr->m_contactPointData[i].m_bodyUniqueIdB);
		PyTuple_SetItem(contactObList, 2, item);
		item =
			PyInt_FromLong(contactPointPtr->m_contactPointData[i].m_linkIndexA);
		PyTuple_SetItem(contactObList, 3, item);
		item =
			PyInt_FromLong(contactPointPtr->m_contactPointData[i].m_linkIndexB);
		PyTuple_SetItem(contactObList, 4, item);

		{
			PyObject* posAObj = PyTuple_New(3);

			item = PyFloat_FromDouble(
				contactPointPtr->m_contactPointData[i].m_positionOnAInWS[0]);
			PyTuple_SetItem(posAObj, 0, item);
			item = PyFloat_FromDouble(
				contactPointPtr->m_contactPointData[i].m_positionOnAInWS[1]);
			PyTuple_SetItem(posAObj, 1, item);
			item = PyFloat_FromDouble(
				contactPointPtr->m_contactPointData[i].m_positionOnAInWS[2]);
			PyTuple_SetItem(posAObj, 2, item);

			PyTuple_SetItem(contactObList, 5, posAObj);
		}

		{
			PyObject* posBObj = PyTuple_New(3);

			item = PyFloat_FromDouble(
				contactPointPtr->m_contactPointData[i].m_positionOnBInWS[0]);
			PyTuple_SetItem(posBObj, 0, item);
			item = PyFloat_FromDouble(
				contactPointPtr->m_contactPointData[i].m_positionOnBInWS[1]);
			PyTuple_SetItem(posBObj, 1, item);
			item = PyFloat_FromDouble(
				contactPointPtr->m_contactPointData[i].m_positionOnBInWS[2]);
			PyTuple_SetItem(posBObj, 2, item);

			PyTuple_SetItem(contactObList, 6, posBObj);
		}

		{
			PyObject* normalOnB = PyTuple_New(3);
			item = PyFloat_FromDouble(
				contactPointPtr->m_contactPointData[i].m_contactNormalOnBInWS[0]);
			PyTuple_SetItem(normalOnB, 0, item);
			item = PyFloat_FromDouble(
				contactPointPtr->m_contactPointData[i].m_contactNormalOnBInWS[1]);
			PyTuple_SetItem(normalOnB, 1, item);
			item = PyFloat_FromDouble(
				contactPointPtr->m_contactPointData[i].m_contactNormalOnBInWS[2]);
			PyTuple_SetItem(normalOnB, 2, item);
			PyTuple_SetItem(contactObList, 7, normalOnB);
		}

		item = PyFloat_FromDouble(
			contactPointPtr->m_contactPointData[i].m_contactDistance);
		PyTuple_SetItem(contactObList, 8, item);
		item = PyFloat_FromDouble(
			contactPointPtr->m_contactPointData[i].m_normalForce);
		PyTuple_SetItem(contactObList, 9, item);

		item = PyFloat_FromDouble(
			contactPointPtr->m_contactPointData[i].m_linearFrictionForce1);
		PyTuple_SetItem(contactObList, 10, item);

		{
			PyObject* posAObj = PyTuple_New(3);

			item = PyFloat_FromDouble(
				contactPointPtr->m_contactPointData[i].m_linearFrictionDirection1[0]);
			PyTuple_SetItem(posAObj, 0, item);
			item = PyFloat_FromDouble(
				contactPointPtr->m_contactPointData[i].m_linearFrictionDirection1[1]);
			PyTuple_SetItem(posAObj, 1, item);
			item = PyFloat_FromDouble(
				contactPointPtr->m_contactPointData[i].m_linearFrictionDirection1[2]);
			PyTuple_SetItem(posAObj, 2, item);
			PyTuple_SetItem(contactObList, 11, posAObj);
		}

		item = PyFloat_FromDouble(
			contactPointPtr->m_contactPointData[i].m_linearFrictionForce2);
		PyTuple_SetItem(contactObList, 12, item);

		{
			PyObject* posAObj = PyTuple_New(3);

			item = PyFloat_FromDouble(
				contactPointPtr->m_contactPointData[i].m_linearFrictionDirection2[0]);
			PyTuple_SetItem(posAObj, 0, item);
			item = PyFloat_FromDouble(
				contactPointPtr->m_contactPointData[i].m_linearFrictionDirection2[1]);
			PyTuple_SetItem(posAObj, 1, item);
			item = PyFloat_FromDouble(
				contactPointPtr->m_contactPointData[i].m_linearFrictionDirection2[2]);
			PyTuple_SetItem(posAObj, 2, item);
			PyTuple_SetItem(contactObList, 13, posAObj);
		}

		PyTuple_SetItem(pyResultList, i, contactObList);
	}
	return pyResultList;
}

static PyObject* pybullet_setCollisionFilterGroupMask(PyObject* self, PyObject* args, PyObject* keywds)
{
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	int bodyUniqueIdA = -1;
	int linkIndexA = -2;

	int collisionFilterGroup = -1;
	int collisionFilterMask = -1;

	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	static char* kwlist[] = {"bodyUniqueId", "linkIndexA", "collisionFilterGroup", "collisionFilterMask", "physicsClientId", NULL};

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "iiii|i", kwlist,
									 &bodyUniqueIdA, &linkIndexA, &collisionFilterGroup, &collisionFilterMask, &physicsClientId))
		return NULL;

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	commandHandle = b3CollisionFilterCommandInit(sm);
	b3SetCollisionFilterGroupMask(commandHandle, bodyUniqueIdA, linkIndexA, collisionFilterGroup, collisionFilterMask);

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_setCollisionFilterPair(PyObject* self, PyObject* args, PyObject* keywds)
{
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	int bodyUniqueIdA = -1;
	int bodyUniqueIdB = -1;
	int linkIndexA = -2;
	int linkIndexB = -2;
	int enableCollision = -1;
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	static char* kwlist[] = {"bodyUniqueIdA", "bodyUniqueIdB", "linkIndexA", "linkIndexB", "enableCollision", "physicsClientId", NULL};

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "iiiii|i", kwlist,
									 &bodyUniqueIdA, &bodyUniqueIdB, &linkIndexA, &linkIndexB, &enableCollision, &physicsClientId))
		return NULL;

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	commandHandle = b3CollisionFilterCommandInit(sm);
	b3SetCollisionFilterPair(commandHandle, bodyUniqueIdA, bodyUniqueIdB, linkIndexA, linkIndexB, enableCollision);

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_getOverlappingObjects(PyObject* self, PyObject* args, PyObject* keywds)
{
	PyObject *aabbMinOb = 0, *aabbMaxOb = 0;
	double aabbMin[3];
	double aabbMax[3];
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	struct b3AABBOverlapData overlapData;
	int i;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	static char* kwlist[] = {"aabbMin", "aabbMax", "physicsClientId", NULL};

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "OO|i", kwlist,
									 &aabbMinOb, &aabbMaxOb, &physicsClientId))
		return NULL;

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	pybullet_internalSetVectord(aabbMinOb, aabbMin);
	pybullet_internalSetVectord(aabbMaxOb, aabbMax);

	commandHandle = b3InitAABBOverlapQuery(sm, aabbMin, aabbMax);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	b3GetAABBOverlapResults(sm, &overlapData);

	if (overlapData.m_numOverlappingObjects)
	{
		PyObject* pyResultList = PyTuple_New(overlapData.m_numOverlappingObjects);
		//For huge amount of overlap, we could use numpy instead (see camera pixel data)
		//What would Python do with huge amount of data? Pass it onto TensorFlow!

		for (i = 0; i < overlapData.m_numOverlappingObjects; i++)
		{
			PyObject* overlap = PyTuple_New(2);  //body unique id and link index

			PyObject* item;
			item =
				PyInt_FromLong(overlapData.m_overlappingObjects[i].m_objectUniqueId);
			PyTuple_SetItem(overlap, 0, item);
			item =
				PyInt_FromLong(overlapData.m_overlappingObjects[i].m_linkIndex);
			PyTuple_SetItem(overlap, 1, item);
			PyTuple_SetItem(pyResultList, i, overlap);
		}

		return pyResultList;
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_getClosestPointData(PyObject* self, PyObject* args, PyObject* keywds)
{
	int bodyUniqueIdA = -1;
	int bodyUniqueIdB = -1;
	int linkIndexA = -2;
	int linkIndexB = -2;
	int collisionShapeA = -1;
	int collisionShapeB = -1;

	PyObject* collisionShapePositionAObj = 0;
	PyObject* collisionShapeOrientationAObj = 0;
	double collisionShapePositionA[3] = {0, 0, 0};
	double collisionShapeOrientationA[4] = {0, 0, 0, 1};
	PyObject* collisionShapePositionBObj = 0;
	PyObject* collisionShapeOrientationBObj = 0;
	double collisionShapePositionB[3] = {0, 0, 0};
	double collisionShapeOrientationB[4] = {0, 0, 0, 1};

	double distanceThreshold = 0.f;

	b3SharedMemoryCommandHandle commandHandle;
	struct b3ContactInformation contactPointData;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	static char* kwlist[] = {"bodyA", "bodyB", "distance", "linkIndexA", "linkIndexB", "collisionShapeA", "collisionShapeB", "collisionShapePositionA", "collisionShapePositionB", "collisionShapeOrientationA", "collisionShapeOrientationB", "physicsClientId", NULL};

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "iid|iiiiOOOOi", kwlist,
									 &bodyUniqueIdA, &bodyUniqueIdB, &distanceThreshold, &linkIndexA, &linkIndexB,
									 &collisionShapeA, &collisionShapeB,
									 &collisionShapePositionAObj, &collisionShapePositionBObj,
									 &collisionShapeOrientationAObj, &collisionShapeOrientationBObj,
									 &physicsClientId))
		return NULL;

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	commandHandle = b3InitClosestDistanceQuery(sm);
	if (bodyUniqueIdA >= 0)
	{
		b3SetClosestDistanceFilterBodyA(commandHandle, bodyUniqueIdA);
	}
	if (bodyUniqueIdB >= 0)
	{
		b3SetClosestDistanceFilterBodyB(commandHandle, bodyUniqueIdB);
	}
	b3SetClosestDistanceThreshold(commandHandle, distanceThreshold);
	if (linkIndexA >= -1)
	{
		b3SetClosestDistanceFilterLinkA(commandHandle, linkIndexA);
	}
	if (linkIndexB >= -1)
	{
		b3SetClosestDistanceFilterLinkB(commandHandle, linkIndexB);
	}
	if (collisionShapeA >= 0)
	{
		b3SetClosestDistanceFilterCollisionShapeA(commandHandle, collisionShapeA);
	}
	if (collisionShapeB >= 0)
	{
		b3SetClosestDistanceFilterCollisionShapeB(commandHandle, collisionShapeB);
	}
	if (collisionShapePositionAObj)
	{
		pybullet_internalSetVectord(collisionShapePositionAObj, collisionShapePositionA);
		b3SetClosestDistanceFilterCollisionShapePositionA(commandHandle, collisionShapePositionA);
	}
	if (collisionShapePositionBObj)
	{
		pybullet_internalSetVectord(collisionShapePositionBObj, collisionShapePositionB);
		b3SetClosestDistanceFilterCollisionShapePositionB(commandHandle, collisionShapePositionB);
	}
	if (collisionShapeOrientationAObj)
	{
		pybullet_internalSetVector4d(collisionShapeOrientationAObj, collisionShapeOrientationA);
		b3SetClosestDistanceFilterCollisionShapeOrientationA(commandHandle, collisionShapeOrientationA);
	}
	if (collisionShapeOrientationBObj)
	{
		pybullet_internalSetVector4d(collisionShapeOrientationBObj, collisionShapeOrientationB);
		b3SetClosestDistanceFilterCollisionShapeOrientationB(commandHandle, collisionShapeOrientationB);
	}

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	if (statusType == CMD_CONTACT_POINT_INFORMATION_COMPLETED)
	{
		b3GetContactPointInformation(sm, &contactPointData);

		return MyConvertContactPoint(&contactPointData);
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_changeUserConstraint(PyObject* self, PyObject* args, PyObject* keywds)
{
	static char* kwlist[] = {"userConstraintUniqueId", "jointChildPivot", "jointChildFrameOrientation", "maxForce", "gearRatio", "gearAuxLink", "relativePositionTarget", "erp", "physicsClientId", NULL};
	int userConstraintUniqueId = -1;
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	int gearAuxLink = -1;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	PyObject* jointChildPivotObj = 0;
	PyObject* jointChildFrameOrnObj = 0;
	double jointChildPivot[3];
	double jointChildFrameOrn[4];
	double maxForce = -1;
	double gearRatio = 0;
	double relativePositionTarget = 1e32;
	double erp = -1;
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|OOddiddi", kwlist, &userConstraintUniqueId, &jointChildPivotObj, &jointChildFrameOrnObj, &maxForce, &gearRatio, &gearAuxLink, &relativePositionTarget, &erp, &physicsClientId))
	{
		return NULL;
	}

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	commandHandle = b3InitChangeUserConstraintCommand(sm, userConstraintUniqueId);

	if (pybullet_internalSetVectord(jointChildPivotObj, jointChildPivot))
	{
		b3InitChangeUserConstraintSetPivotInB(commandHandle, jointChildPivot);
	}

	if (pybullet_internalSetVector4d(jointChildFrameOrnObj, jointChildFrameOrn))
	{
		b3InitChangeUserConstraintSetFrameInB(commandHandle, jointChildFrameOrn);
	}

	if (relativePositionTarget < 1e10)
	{
		b3InitChangeUserConstraintSetRelativePositionTarget(commandHandle, relativePositionTarget);
	}
	if (erp >= 0)
	{
		b3InitChangeUserConstraintSetERP(commandHandle, erp);
	}

	if (maxForce >= 0)
	{
		b3InitChangeUserConstraintSetMaxForce(commandHandle, maxForce);
	}
	if (gearRatio != 0)
	{
		b3InitChangeUserConstraintSetGearRatio(commandHandle, gearRatio);
	}
	if (gearAuxLink >= 0)
	{
		b3InitChangeUserConstraintSetGearAuxLink(commandHandle, gearAuxLink);
	}
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	Py_INCREF(Py_None);
	return Py_None;
};

static PyObject* pybullet_removeUserConstraint(PyObject* self, PyObject* args, PyObject* keywds)
{
	static char* kwlist[] = {"userConstraintUniqueId", "physicsClientId", NULL};
	int userConstraintUniqueId = -1;
	b3SharedMemoryCommandHandle commandHandle;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist, &userConstraintUniqueId, &physicsClientId))
	{
		return NULL;
	}

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	commandHandle = b3InitRemoveUserConstraintCommand(sm, userConstraintUniqueId);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	Py_INCREF(Py_None);
	return Py_None;
};

/*
static PyObject* pybullet_updateUserConstraint(PyObject* self, PyObject* args, PyObject *keywds)
{
	return NULL;
}
*/

static PyObject* pybullet_enableJointForceTorqueSensor(PyObject* self, PyObject* args, PyObject* keywds)
{
	int bodyUniqueId = -1;
	int jointIndex = -1;
	int enableSensor = 1;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	int numJoints = -1;

	static char* kwlist[] = {"bodyUniqueId", "jointIndex", "enableSensor", "physicsClientId", NULL};

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "ii|ii", kwlist, &bodyUniqueId, &jointIndex, &enableSensor, &physicsClientId))
	{
		return NULL;
	}

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	if (bodyUniqueId < 0)
	{
		PyErr_SetString(SpamError, "Error: invalid bodyUniqueId");
		return NULL;
	}
	numJoints = b3GetNumJoints(sm, bodyUniqueId);
	if ((jointIndex < 0) || (jointIndex >= numJoints))
	{
		PyErr_SetString(SpamError, "Error: invalid jointIndex.");
		return NULL;
	}

	{
		b3SharedMemoryCommandHandle commandHandle;
		b3SharedMemoryStatusHandle statusHandle;
		int statusType;

		commandHandle = b3CreateSensorCommandInit(sm, bodyUniqueId);
		b3CreateSensorEnable6DofJointForceTorqueSensor(commandHandle, jointIndex, enableSensor);
		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
		statusType = b3GetStatusType(statusHandle);
		if (statusType == CMD_CLIENT_COMMAND_COMPLETED)
		{
			Py_INCREF(Py_None);
			return Py_None;
		}
	}

	PyErr_SetString(SpamError, "Error creating sensor.");
	return NULL;
}

static int extractVertices(PyObject* verticesObj, double* vertices, int maxNumVertices)
{
	int numVerticesOut = 0;

	if (verticesObj)
	{
		PyObject* seqVerticesObj = PySequence_Fast(verticesObj, "expected a sequence of vertex positions");
		if (seqVerticesObj)
		{
			int numVerticesSrc = PySequence_Size(seqVerticesObj);
			{
				int i;

				if (numVerticesSrc > B3_MAX_NUM_VERTICES)
				{
					PyErr_SetString(SpamError, "Number of vertices exceeds the maximum.");
					Py_DECREF(seqVerticesObj);
					return 0;
				}
				for (i = 0; i < numVerticesSrc; i++)
				{
					PyObject* vertexObj = PySequence_GetItem(seqVerticesObj, i);
					double vertex[3];
					if (pybullet_internalSetVectord(vertexObj, vertex))
					{
						if (vertices)
						{
							vertices[numVerticesOut * 3 + 0] = vertex[0];
							vertices[numVerticesOut * 3 + 1] = vertex[1];
							vertices[numVerticesOut * 3 + 2] = vertex[2];
						}
						numVerticesOut++;
					}
				}
			}
		}
	}
	return numVerticesOut;
}

static int extractUVs(PyObject* uvsObj, double* uvs, int maxNumVertices)
{
	int numUVOut = 0;

	if (uvsObj)
	{
		PyObject* seqVerticesObj = PySequence_Fast(uvsObj, "expected a sequence of uvs");
		if (seqVerticesObj)
		{
			int numVerticesSrc = PySequence_Size(seqVerticesObj);
			{
				int i;

				if (numVerticesSrc > B3_MAX_NUM_VERTICES)
				{
					PyErr_SetString(SpamError, "Number of uvs exceeds the maximum.");
					Py_DECREF(seqVerticesObj);
					return 0;
				}
				for (i = 0; i < numVerticesSrc; i++)
				{
					PyObject* vertexObj = PySequence_GetItem(seqVerticesObj, i);
					double uv[2];
					if (pybullet_internalSetVector2d(vertexObj, uv))
					{
						if (uvs)
						{
							uvs[numUVOut * 2 + 0] = uv[0];
							uvs[numUVOut * 2 + 1] = uv[1];
						}
						numUVOut++;
					}
				}
			}
		}
	}
	return numUVOut;
}
static int extractIndices(PyObject* indicesObj, int* indices, int maxNumIndices)
{
	int numIndicesOut = 0;

	if (indicesObj)
	{
		PyObject* seqIndicesObj = PySequence_Fast(indicesObj, "expected a sequence of indices");
		if (seqIndicesObj)
		{
			int numIndicesSrc = PySequence_Size(seqIndicesObj);
			{
				int i;

				if (numIndicesSrc > B3_MAX_NUM_INDICES)
				{
					PyErr_SetString(SpamError, "Number of indices exceeds the maximum.");
					Py_DECREF(seqIndicesObj);
					return 0;
				}
				for (i = 0; i < numIndicesSrc; i++)
				{
					int index = pybullet_internalGetIntFromSequence(seqIndicesObj, i);
					if (indices)
					{
						indices[numIndicesOut] = index;
					}
					numIndicesOut++;
				}
			}
		}
	}
	return numIndicesOut;
}


static PyObject* pybullet_createCollisionShape(PyObject* self, PyObject* args, PyObject* keywds)
{
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	int shapeType = -1;
	double radius = 0.5;
	double height = 1;
	PyObject* meshScaleObj = 0;
	double meshScale[3] = {1, 1, 1};
	PyObject* planeNormalObj = 0;
	double planeNormal[3] = {0, 0, 1};
	PyObject* collisionFramePositionObj = 0;
	double collisionFramePosition[3] = {0, 0, 0};
	PyObject* collisionFrameOrientationObj = 0;
	double collisionFrameOrientation[4] = {0, 0, 0, 1};
	char* fileName = 0;
	int flags = 0;
	double heightfieldTextureScaling = 1;
	PyObject* halfExtentsObj = 0;
	PyObject* verticesObj = 0;
	PyObject* indicesObj = 0;
	PyObject* heightfieldDataObj = 0;
	int numHeightfieldRows = -1;
	int numHeightfieldColumns = -1;
	int replaceHeightfieldIndex = -1;
	static char* kwlist[] = {"shapeType",
							 "radius",
							 "halfExtents",
							 "height",
							 "fileName",
							 "meshScale",
							 "planeNormal",
							 "flags",
							 "collisionFramePosition",
							 "collisionFrameOrientation",
							 "vertices",
							 "indices",
							 "heightfieldTextureScaling",
							 "heightfieldData",
							 "numHeightfieldRows",
							 "numHeightfieldColumns",
							 "replaceHeightfieldIndex",
							 "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|dOdsOOiOOOOdOiiii", kwlist,
									 &shapeType, &radius, &halfExtentsObj, &height, &fileName, &meshScaleObj, &planeNormalObj, &flags, &collisionFramePositionObj, &collisionFrameOrientationObj, &verticesObj, &indicesObj, &heightfieldTextureScaling, &heightfieldDataObj, &numHeightfieldRows, &numHeightfieldColumns, &replaceHeightfieldIndex, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	if (shapeType >= GEOM_SPHERE)
	{
		b3SharedMemoryStatusHandle statusHandle;
		int statusType;
		int shapeIndex = -1;

		b3SharedMemoryCommandHandle commandHandle = b3CreateCollisionShapeCommandInit(sm);
		if (shapeType == GEOM_SPHERE && radius > 0)
		{
			shapeIndex = b3CreateCollisionShapeAddSphere(commandHandle, radius);
		}
		if (shapeType == GEOM_BOX && halfExtentsObj)
		{
			double halfExtents[3] = {1, 1, 1};
			pybullet_internalSetVectord(halfExtentsObj, halfExtents);
			shapeIndex = b3CreateCollisionShapeAddBox(commandHandle, halfExtents);
		}
		if (shapeType == GEOM_CAPSULE && radius > 0 && height >= 0)
		{
			shapeIndex = b3CreateCollisionShapeAddCapsule(commandHandle, radius, height);
		}
		if (shapeType == GEOM_CYLINDER && radius > 0 && height >= 0)
		{
			shapeIndex = b3CreateCollisionShapeAddCylinder(commandHandle, radius, height);
		}
		if (shapeType == GEOM_HEIGHTFIELD && fileName)
		{
			if (meshScaleObj)
			{
				pybullet_internalSetVectord(meshScaleObj, meshScale);
			}
			shapeIndex = b3CreateCollisionShapeAddHeightfield(commandHandle, fileName, meshScale, heightfieldTextureScaling);

		}
		if (shapeType == GEOM_HEIGHTFIELD && fileName==0 && heightfieldDataObj && numHeightfieldColumns>0 && numHeightfieldRows > 0)
		{
			PyObject* seqPoints=0;
			int numHeightfieldPoints;
			if (meshScaleObj)
			{
				pybullet_internalSetVectord(meshScaleObj, meshScale);
			}
			seqPoints = PySequence_Fast(heightfieldDataObj, "expected a sequence");
			numHeightfieldPoints = PySequence_Size(heightfieldDataObj);
			if (numHeightfieldPoints != numHeightfieldColumns*numHeightfieldRows)
			{
				PyErr_SetString(SpamError, "Size of heightfieldData doesn't match numHeightfieldColumns*numHeightfieldRows");
				return NULL;
			}
			{
				PyObject* item;
				int i;
				float* pointBuffer = (float*)malloc(numHeightfieldPoints*sizeof(float));
				if (PyList_Check(seqPoints))
				{
					for (i = 0; i < numHeightfieldPoints; i++)
					{
						item = PyList_GET_ITEM(seqPoints, i);
						pointBuffer[i] = (float)PyFloat_AsDouble(item);
					}
				}
				else
				{
					for (i = 0; i < numHeightfieldPoints; i++)
					{
						item = PyTuple_GET_ITEM(seqPoints, i);
						pointBuffer[i] = (float)PyFloat_AsDouble(item);
					}
				}
				shapeIndex = b3CreateCollisionShapeAddHeightfield2(sm, commandHandle, meshScale, heightfieldTextureScaling, pointBuffer, numHeightfieldRows, numHeightfieldColumns, replaceHeightfieldIndex);
			
				free(pointBuffer);
				if (seqPoints)
					Py_DECREF(seqPoints);
			}
		}
		if (shapeType == GEOM_MESH && fileName)
		{
			pybullet_internalSetVectord(meshScaleObj, meshScale);
			shapeIndex = b3CreateCollisionShapeAddMesh(commandHandle, fileName, meshScale);
		}
		if (shapeType == GEOM_MESH && verticesObj)
		{
			int numVertices = extractVertices(verticesObj, 0, B3_MAX_NUM_VERTICES);
			int numIndices = extractIndices(indicesObj, 0, B3_MAX_NUM_INDICES);
			double* vertices = numVertices ? malloc(numVertices * 3 * sizeof(double)) : 0;
			int* indices = numIndices ? malloc(numIndices * sizeof(int)) : 0;

			numVertices = extractVertices(verticesObj, vertices, B3_MAX_NUM_VERTICES);
			pybullet_internalSetVectord(meshScaleObj, meshScale);

			if (indicesObj)
			{
				numIndices = extractIndices(indicesObj, indices, B3_MAX_NUM_INDICES);
			}

			if (numIndices)
			{
				shapeIndex = b3CreateCollisionShapeAddConcaveMesh(sm, commandHandle, meshScale, vertices, numVertices, indices, numIndices);
			}
			else
			{
				shapeIndex = b3CreateCollisionShapeAddConvexMesh(sm, commandHandle, meshScale, vertices, numVertices);
			}
			free(vertices);
			free(indices);
		}

		if (shapeType == GEOM_PLANE)
		{
			double planeConstant = 0;
			pybullet_internalSetVectord(planeNormalObj, planeNormal);
			shapeIndex = b3CreateCollisionShapeAddPlane(commandHandle, planeNormal, planeConstant);
		}
		if (shapeIndex >= 0 && flags)
		{
			b3CreateCollisionSetFlag(commandHandle, shapeIndex, flags);
		}
		if (shapeIndex >= 0)
		{
			if (collisionFramePositionObj)
			{
				pybullet_internalSetVectord(collisionFramePositionObj, collisionFramePosition);
			}

			if (collisionFrameOrientationObj)
			{
				pybullet_internalSetVector4d(collisionFrameOrientationObj, collisionFrameOrientation);
			}
			if (collisionFramePositionObj || collisionFrameOrientationObj)
			{
				b3CreateCollisionShapeSetChildTransform(commandHandle, shapeIndex, collisionFramePosition, collisionFrameOrientation);
			}
		}
		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
		statusType = b3GetStatusType(statusHandle);
		if (statusType == CMD_CREATE_COLLISION_SHAPE_COMPLETED)
		{
			int uid = b3GetStatusCollisionShapeUniqueId(statusHandle);
			PyObject* ob = PyLong_FromLong(uid);
			return ob;
		}
	}
	PyErr_SetString(SpamError, "createCollisionShape failed.");
	return NULL;
}

static PyObject* pybullet_createCollisionShapeArray(PyObject* self, PyObject* args, PyObject* keywds)
{
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	PyObject* shapeTypeArray = 0;
	PyObject* radiusArray = 0;
	PyObject* halfExtentsObjArray = 0;
	PyObject* lengthArray = 0;
	PyObject* fileNameArray = 0;
	PyObject* meshScaleObjArray = 0;
	PyObject* planeNormalObjArray = 0;
	PyObject* flagsArray = 0;
	PyObject* collisionFramePositionObjArray = 0;
	PyObject* collisionFrameOrientationObjArray = 0;

	static char* kwlist[] = {"shapeTypes", "radii", "halfExtents", "lengths", "fileNames", "meshScales", "planeNormals",
							 "flags", "collisionFramePositions", "collisionFrameOrientations", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "O|OOOOOOOOOi", kwlist,
									 &shapeTypeArray, &radiusArray, &halfExtentsObjArray, &lengthArray, &fileNameArray, &meshScaleObjArray, &planeNormalObjArray, &flagsArray, &collisionFramePositionObjArray, &collisionFrameOrientationObjArray, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	{
		b3SharedMemoryCommandHandle commandHandle = b3CreateCollisionShapeCommandInit(sm);
		int numShapeTypes = 0;
		int numRadius = 0;
		int numHalfExtents = 0;
		int numLengths = 0;
		int numFileNames = 0;
		int numMeshScales = 0;
		int numPlaneNormals = 0;
		int numFlags = 0;
		int numPositions = 0;
		int numOrientations = 0;

		int s;
		PyObject* shapeTypeArraySeq = shapeTypeArray ? PySequence_Fast(shapeTypeArray, "expected a sequence of shape types") : 0;
		PyObject* radiusArraySeq = radiusArray ? PySequence_Fast(radiusArray, "expected a sequence of radii") : 0;
		PyObject* halfExtentsArraySeq = halfExtentsObjArray ? PySequence_Fast(halfExtentsObjArray, "expected a sequence of half extents") : 0;
		PyObject* lengthArraySeq = lengthArray ? PySequence_Fast(lengthArray, "expected a sequence of lengths") : 0;
		PyObject* fileNameArraySeq = fileNameArray ? PySequence_Fast(fileNameArray, "expected a sequence of filename") : 0;
		PyObject* meshScaleArraySeq = meshScaleObjArray ? PySequence_Fast(meshScaleObjArray, "expected a sequence of mesh scale") : 0;
		PyObject* planeNormalArraySeq = planeNormalObjArray ? PySequence_Fast(planeNormalObjArray, "expected a sequence of plane normal") : 0;
		PyObject* flagsArraySeq = flagsArray ? PySequence_Fast(flagsArray, "expected a sequence of flags") : 0;
		PyObject* positionArraySeq = collisionFramePositionObjArray ? PySequence_Fast(collisionFramePositionObjArray, "expected a sequence of collision frame positions") : 0;
		PyObject* orientationArraySeq = collisionFrameOrientationObjArray ? PySequence_Fast(collisionFrameOrientationObjArray, "expected a sequence of collision frame orientations") : 0;

		if (shapeTypeArraySeq == 0)
		{
			PyErr_SetString(SpamError, "expected a sequence of shape types");
			return NULL;
		}

		numShapeTypes = shapeTypeArray ? PySequence_Size(shapeTypeArray) : 0;
		numRadius = radiusArraySeq ? PySequence_Size(radiusArraySeq) : 0;
		numHalfExtents = halfExtentsArraySeq ? PySequence_Size(halfExtentsArraySeq) : 0;
		numLengths = lengthArraySeq ? PySequence_Size(lengthArraySeq) : 0;
		numFileNames = fileNameArraySeq ? PySequence_Size(fileNameArraySeq) : 0;
		numMeshScales = meshScaleArraySeq ? PySequence_Size(meshScaleArraySeq) : 0;
		numPlaneNormals = planeNormalArraySeq ? PySequence_Size(planeNormalArraySeq) : 0;

		for (s = 0; s < numShapeTypes; s++)
		{
			int shapeType = pybullet_internalGetIntFromSequence(shapeTypeArraySeq, s);
			if (shapeType >= GEOM_SPHERE)
			{
				int shapeIndex = -1;

				if (shapeType == GEOM_SPHERE && s <= numRadius)
				{
					double radius = pybullet_internalGetFloatFromSequence(radiusArraySeq, s);
					if (radius > 0)
					{
						shapeIndex = b3CreateCollisionShapeAddSphere(commandHandle, radius);
					}
				}
				if (shapeType == GEOM_BOX)
				{
					PyObject* halfExtentsObj = 0;
					double halfExtents[3] = {1, 1, 1};

					if (halfExtentsArraySeq && s <= numHalfExtents)
					{
						if (PyList_Check(halfExtentsArraySeq))
						{
							halfExtentsObj = PyList_GET_ITEM(halfExtentsArraySeq, s);
						}
						else
						{
							halfExtentsObj = PyTuple_GET_ITEM(halfExtentsArraySeq, s);
						}
					}
					pybullet_internalSetVectord(halfExtentsObj, halfExtents);
					shapeIndex = b3CreateCollisionShapeAddBox(commandHandle, halfExtents);
				}
				if (shapeType == GEOM_CAPSULE && s <= numRadius)
				{
					double radius = pybullet_internalGetFloatFromSequence(radiusArraySeq, s);
					double height = pybullet_internalGetFloatFromSequence(lengthArraySeq, s);
					if (radius > 0 && height >= 0)
					{
						shapeIndex = b3CreateCollisionShapeAddCapsule(commandHandle, radius, height);
					}
				}
				if (shapeType == GEOM_CYLINDER && s <= numRadius && s < numLengths)
				{
					double radius = pybullet_internalGetFloatFromSequence(radiusArraySeq, s);
					double height = pybullet_internalGetFloatFromSequence(lengthArraySeq, s);
					if (radius > 0 && height >= 0)
					{
						shapeIndex = b3CreateCollisionShapeAddCylinder(commandHandle, radius, height);
					}
				}
				if (shapeType == GEOM_MESH)
				{
					double meshScale[3] = {1, 1, 1};

					PyObject* meshScaleObj = meshScaleArraySeq ? PyList_GET_ITEM(meshScaleArraySeq, s) : 0;
					PyObject* fileNameObj = fileNameArraySeq ? PyList_GET_ITEM(fileNameArraySeq, s) : 0;
					const char* fileName = 0;

					if (fileNameObj)
					{
#if PY_MAJOR_VERSION >= 3
						PyObject* ob = PyUnicode_AsASCIIString(fileNameObj);
						fileName = PyBytes_AS_STRING(ob);
#else
						fileName = PyString_AsString(fileNameObj);
#endif
					}
					if (meshScaleObj)
					{
						pybullet_internalSetVectord(meshScaleObj, meshScale);
					}
					if (fileName)
					{
						shapeIndex = b3CreateCollisionShapeAddMesh(commandHandle, fileName, meshScale);
					}
				}
				if (shapeType == GEOM_PLANE)
				{
					PyObject* planeNormalObj = planeNormalArraySeq ? PyList_GET_ITEM(planeNormalArraySeq, s) : 0;
					double planeNormal[3];
					double planeConstant = 0;
					pybullet_internalSetVectord(planeNormalObj, planeNormal);
					shapeIndex = b3CreateCollisionShapeAddPlane(commandHandle, planeNormal, planeConstant);
				}
				if (flagsArraySeq)
				{
					int flags = pybullet_internalGetIntFromSequence(flagsArraySeq, s);
					b3CreateCollisionSetFlag(commandHandle, shapeIndex, flags);
				}
				if (positionArraySeq || orientationArraySeq)
				{
					PyObject* collisionFramePositionObj = positionArraySeq ? PyList_GET_ITEM(positionArraySeq, s) : 0;
					PyObject* collisionFrameOrientationObj = orientationArraySeq ? PyList_GET_ITEM(orientationArraySeq, s) : 0;
					double collisionFramePosition[3] = {0, 0, 0};
					double collisionFrameOrientation[4] = {0, 0, 0, 1};
					if (collisionFramePositionObj)
					{
						pybullet_internalSetVectord(collisionFramePositionObj, collisionFramePosition);
					}

					if (collisionFrameOrientationObj)
					{
						pybullet_internalSetVector4d(collisionFrameOrientationObj, collisionFrameOrientation);
					}
					if (shapeIndex >= 0)
					{
						b3CreateCollisionShapeSetChildTransform(commandHandle, shapeIndex, collisionFramePosition, collisionFrameOrientation);
					}
				}
			}
		}

		if (shapeTypeArraySeq)
			Py_DECREF(shapeTypeArraySeq);
		if (radiusArraySeq)
			Py_DECREF(radiusArraySeq);
		if (halfExtentsArraySeq)
			Py_DECREF(halfExtentsArraySeq);
		if (lengthArraySeq)
			Py_DECREF(lengthArraySeq);
		if (fileNameArraySeq)
			Py_DECREF(fileNameArraySeq);
		if (meshScaleArraySeq)
			Py_DECREF(meshScaleArraySeq);
		if (planeNormalArraySeq)
			Py_DECREF(planeNormalArraySeq);
		if (flagsArraySeq)
			Py_DECREF(flagsArraySeq);
		if (positionArraySeq)
			Py_DECREF(positionArraySeq);
		if (orientationArraySeq)
			Py_DECREF(orientationArraySeq);

		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
		statusType = b3GetStatusType(statusHandle);
		if (statusType == CMD_CREATE_COLLISION_SHAPE_COMPLETED)
		{
			int uid = b3GetStatusCollisionShapeUniqueId(statusHandle);
			PyObject* ob = PyLong_FromLong(uid);
			return ob;
		}
	}

	PyErr_SetString(SpamError, "createCollisionShapeArray failed.");
	return NULL;
}

static PyObject* pybullet_getMeshData(PyObject* self, PyObject* args, PyObject* keywds)
{
	int bodyUniqueId = -1;
	int linkIndex = -1;
	int collisionShapeIndex = -1;
	b3PhysicsClientHandle sm = 0;
	b3SharedMemoryCommandHandle command;
	b3SharedMemoryStatusHandle statusHandle;
	struct b3MeshData meshData;
	int statusType;
	int flags = -1;

	int physicsClientId = 0;
	static char* kwlist[] = {"bodyUniqueId", "linkIndex", "collisionShapeIndex", "flags", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|iiii", kwlist, &bodyUniqueId, &linkIndex,&collisionShapeIndex, &flags , &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
	command = b3GetMeshDataCommandInit(sm, bodyUniqueId, linkIndex);
	if (collisionShapeIndex >= 0)
	{
		b3GetMeshDataSetCollisionShapeIndex(command, collisionShapeIndex);
	}
	if (flags >= 0)
	{
		b3GetMeshDataSetFlags(command, flags);
	}
	
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusType(statusHandle);
	if (statusType == CMD_REQUEST_MESH_DATA_COMPLETED)
	{
		int i;
		PyObject* pyVertexData;
		PyObject* pyListMeshData = PyTuple_New(2);
		b3GetMeshData(sm, &meshData);
		PyTuple_SetItem(pyListMeshData, 0, PyInt_FromLong(meshData.m_numVertices));
		pyVertexData = PyTuple_New(meshData.m_numVertices);
		PyTuple_SetItem(pyListMeshData, 1, pyVertexData);
		
		for (i = 0; i < meshData.m_numVertices; i++)
		{
			PyObject* pyListVertex = PyTuple_New(3);
			PyTuple_SetItem(pyListVertex, 0, PyFloat_FromDouble(meshData.m_vertices[i].x));
			PyTuple_SetItem(pyListVertex, 1, PyFloat_FromDouble(meshData.m_vertices[i].y));
			PyTuple_SetItem(pyListVertex, 2, PyFloat_FromDouble(meshData.m_vertices[i].z));
			PyTuple_SetItem(pyVertexData, i, pyListVertex);
		}
		
		return pyListMeshData;
	}
	
	PyErr_SetString(SpamError, "getMeshData failed");
	return NULL;
}

static PyObject* pybullet_createVisualShape(PyObject* self, PyObject* args, PyObject* keywds)
{
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;

	int shapeType = -1;
	double radius = 0.5;
	double length = 1;
	PyObject* meshScaleObj = 0;
	double meshScale[3] = {1, 1, 1};
	PyObject* planeNormalObj = 0;
	double planeNormal[3] = {0, 0, 1};

	PyObject* rgbaColorObj = 0;
	double rgbaColor[4] = {1, 1, 1, 1};

	PyObject* specularColorObj = 0;
	double specularColor[3] = {1, 1, 1};

	char* fileName = 0;
	int flags = 0;

	PyObject* visualFramePositionObj = 0;
	double visualFramePosition[3] = {0, 0, 0};
	PyObject* visualFrameOrientationObj = 0;
	double visualFrameOrientation[4] = {0, 0, 0, 1};

	PyObject* halfExtentsObj = 0;

	PyObject* verticesObj = 0;
	PyObject* indicesObj = 0;
	PyObject* normalsObj = 0;
	PyObject* uvsObj = 0;

	static char* kwlist[] = {"shapeType", "radius", "halfExtents", "length", "fileName", "meshScale", "planeNormal", "flags", "rgbaColor", "specularColor", "visualFramePosition", "visualFrameOrientation", "vertices", "indices", "normals", "uvs", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|dOdsOOiOOOOOOOOi", kwlist,
									 &shapeType, &radius, &halfExtentsObj, &length, &fileName, &meshScaleObj, &planeNormalObj, &flags, &rgbaColorObj, &specularColorObj, &visualFramePositionObj, &visualFrameOrientationObj, &verticesObj, &indicesObj, &normalsObj, &uvsObj, &physicsClientId))
	{
		return NULL;
	}

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	if (shapeType >= GEOM_SPHERE)
	{
		b3SharedMemoryStatusHandle statusHandle;
		int statusType;
		b3SharedMemoryCommandHandle commandHandle = b3CreateVisualShapeCommandInit(sm);
		int shapeIndex = -1;

		if (shapeType == GEOM_SPHERE && radius > 0)
		{
			shapeIndex = b3CreateVisualShapeAddSphere(commandHandle, radius);
		}
		if (shapeType == GEOM_BOX && halfExtentsObj)
		{
			double halfExtents[3] = {1, 1, 1};
			pybullet_internalSetVectord(halfExtentsObj, halfExtents);
			shapeIndex = b3CreateVisualShapeAddBox(commandHandle, halfExtents);
		}

		if (shapeType == GEOM_CAPSULE && radius > 0 && length >= 0)
		{
			shapeIndex = b3CreateVisualShapeAddCapsule(commandHandle, radius, length);
		}
		if (shapeType == GEOM_CYLINDER && radius > 0 && length >= 0)
		{
			shapeIndex = b3CreateVisualShapeAddCylinder(commandHandle, radius, length);
		}
		if (shapeType == GEOM_MESH && fileName)
		{
			pybullet_internalSetVectord(meshScaleObj, meshScale);
			shapeIndex = b3CreateVisualShapeAddMesh(commandHandle, fileName, meshScale);
		}

		if (shapeType == GEOM_MESH && verticesObj && indicesObj)
		{
			int numVertices = extractVertices(verticesObj, 0, B3_MAX_NUM_VERTICES);
			int numIndices = extractIndices(indicesObj, 0, B3_MAX_NUM_INDICES);
			int numNormals = extractVertices(normalsObj, 0, B3_MAX_NUM_VERTICES);
			int numUVs = extractUVs(uvsObj, 0, B3_MAX_NUM_VERTICES);

			double* vertices = numVertices ? malloc(numVertices * 3 * sizeof(double)) : 0;
			int* indices = numIndices ? malloc(numIndices * sizeof(int)) : 0;
			double* normals = numNormals ? malloc(numNormals * 3 * sizeof(double)) : 0;
			double* uvs = numUVs ? malloc(numUVs * 2 * sizeof(double)) : 0;

			numVertices = extractVertices(verticesObj, vertices, B3_MAX_NUM_VERTICES);
			pybullet_internalSetVectord(meshScaleObj, meshScale);

			if (indicesObj)
			{
				numIndices = extractIndices(indicesObj, indices, B3_MAX_NUM_INDICES);
			}
			if (numNormals)
			{
				extractVertices(normalsObj, normals, numNormals);
			}
			if (numUVs)
			{
				extractUVs(uvsObj, uvs, numUVs);
			}

			if (numIndices)
			{
				shapeIndex = b3CreateVisualShapeAddMesh2(sm, commandHandle, meshScale, vertices, numVertices, indices, numIndices, normals, numNormals, uvs, numUVs);
			}
			free(uvs);
			free(normals);
			free(vertices);
			free(indices);
		}

		if (shapeType == GEOM_PLANE)
		{
			double planeConstant = 0;
			pybullet_internalSetVectord(planeNormalObj, planeNormal);
			shapeIndex = b3CreateVisualShapeAddPlane(commandHandle, planeNormal, planeConstant);
		}
		if (shapeIndex >= 0 && flags)
		{
			b3CreateVisualSetFlag(commandHandle, shapeIndex, flags);
		}

		if (shapeIndex >= 0)
		{
			double rgbaColor[4] = {1, 1, 1, 1};
			double specularColor[3] = {1, 1, 1};
			if (rgbaColorObj)
			{
				pybullet_internalSetVector4d(rgbaColorObj, rgbaColor);
			}
			b3CreateVisualShapeSetRGBAColor(commandHandle, shapeIndex, rgbaColor);

			if (specularColorObj)
			{
				pybullet_internalSetVectord(specularColorObj, specularColor);
			}
			b3CreateVisualShapeSetSpecularColor(commandHandle, shapeIndex, specularColor);

			if (visualFramePositionObj)
			{
				pybullet_internalSetVectord(visualFramePositionObj, visualFramePosition);
			}

			if (visualFrameOrientationObj)
			{
				pybullet_internalSetVector4d(visualFrameOrientationObj, visualFrameOrientation);
			}
			b3CreateVisualShapeSetChildTransform(commandHandle, shapeIndex, visualFramePosition, visualFrameOrientation);
		}

		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
		statusType = b3GetStatusType(statusHandle);
		if (statusType == CMD_CREATE_VISUAL_SHAPE_COMPLETED)
		{
			int uid = b3GetStatusVisualShapeUniqueId(statusHandle);
			PyObject* ob = PyLong_FromLong(uid);
			return ob;
		}
	}
	PyErr_SetString(SpamError, "createVisualShape failed.");
	return NULL;
}

static PyObject* pybullet_createVisualShapeArray(PyObject* self, PyObject* args, PyObject* keywds)
{
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	PyObject* shapeTypeArray = 0;
	PyObject* radiusArray = 0;
	PyObject* halfExtentsObjArray = 0;
	PyObject* lengthArray = 0;
	PyObject* fileNameArray = 0;
	PyObject* meshScaleObjArray = 0;
	PyObject* planeNormalObjArray = 0;
	PyObject* rgbaColorArray = 0;
	PyObject* flagsArray = 0;
	PyObject* visualFramePositionObjArray = 0;
	PyObject* visualFrameOrientationObjArray = 0;

	static char* kwlist[] = {"shapeTypes", "radii", "halfExtents", "lengths", "fileNames", "meshScales", "planeNormals",
							 "flags", "rgbaColors", "visualFramePositions", "visualFrameOrientations", "physicsClientId", NULL};

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "O|OOOOOOOOOOi", kwlist,
									 &shapeTypeArray, &radiusArray, &halfExtentsObjArray, &lengthArray, &fileNameArray, &meshScaleObjArray, &planeNormalObjArray, &flagsArray, &rgbaColorArray, &visualFramePositionObjArray, &visualFrameOrientationObjArray, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	{
		b3SharedMemoryCommandHandle commandHandle = b3CreateVisualShapeCommandInit(sm);
		int numShapeTypes = 0;
		int numRadius = 0;
		int numHalfExtents = 0;
		int numLengths = 0;
		int numFileNames = 0;
		int numMeshScales = 0;
		int numPlaneNormals = 0;
		int numRGBAColors = 0;
		int numFlags = 0;
		int numPositions = 0;
		int numOrientations = 0;

		int s;
		PyObject* shapeTypeArraySeq = shapeTypeArray ? PySequence_Fast(shapeTypeArray, "expected a sequence of shape types") : 0;
		PyObject* radiusArraySeq = radiusArray ? PySequence_Fast(radiusArray, "expected a sequence of radii") : 0;
		PyObject* halfExtentsArraySeq = halfExtentsObjArray ? PySequence_Fast(halfExtentsObjArray, "expected a sequence of half extents") : 0;
		PyObject* lengthArraySeq = lengthArray ? PySequence_Fast(lengthArray, "expected a sequence of lengths") : 0;
		PyObject* fileNameArraySeq = fileNameArray ? PySequence_Fast(fileNameArray, "expected a sequence of filename") : 0;
		PyObject* meshScaleArraySeq = meshScaleObjArray ? PySequence_Fast(meshScaleObjArray, "expected a sequence of mesh scale") : 0;
		PyObject* planeNormalArraySeq = planeNormalObjArray ? PySequence_Fast(planeNormalObjArray, "expected a sequence of plane normal") : 0;
		PyObject* rgbaColorArraySeq = rgbaColorArray ? PySequence_Fast(rgbaColorArray, "expected a sequence of rgba color") : 0;
		PyObject* flagsArraySeq = flagsArray ? PySequence_Fast(flagsArray, "expected a sequence of flags") : 0;
		PyObject* positionArraySeq = visualFramePositionObjArray ? PySequence_Fast(visualFramePositionObjArray, "expected a sequence of visual frame positions") : 0;
		PyObject* orientationArraySeq = visualFrameOrientationObjArray ? PySequence_Fast(visualFrameOrientationObjArray, "expected a sequence of visual frame orientations") : 0;

		if (shapeTypeArraySeq == 0)
		{
			PyErr_SetString(SpamError, "expected a sequence of shape types");
			return NULL;
		}

		numShapeTypes = shapeTypeArray ? PySequence_Size(shapeTypeArray) : 0;
		numRadius = radiusArraySeq ? PySequence_Size(radiusArraySeq) : 0;
		numHalfExtents = halfExtentsArraySeq ? PySequence_Size(halfExtentsArraySeq) : 0;
		numLengths = lengthArraySeq ? PySequence_Size(lengthArraySeq) : 0;
		numFileNames = fileNameArraySeq ? PySequence_Size(fileNameArraySeq) : 0;
		numMeshScales = meshScaleArraySeq ? PySequence_Size(meshScaleArraySeq) : 0;
		numPlaneNormals = planeNormalArraySeq ? PySequence_Size(planeNormalArraySeq) : 0;
		numRGBAColors = rgbaColorArraySeq ? PySequence_Size(rgbaColorArraySeq) : 0;

		for (s = 0; s < numShapeTypes; s++)
		{
			int shapeType = pybullet_internalGetIntFromSequence(shapeTypeArraySeq, s);
			if (shapeType >= GEOM_SPHERE)
			{
				int shapeIndex = -1;

				if (shapeType == GEOM_SPHERE && s <= numRadius)
				{
					double radius = pybullet_internalGetFloatFromSequence(radiusArraySeq, s);
					if (radius > 0)
					{
						shapeIndex = b3CreateVisualShapeAddSphere(commandHandle, radius);
					}
				}
				if (shapeType == GEOM_BOX)
				{
					PyObject* halfExtentsObj = 0;
					double halfExtents[3] = {1, 1, 1};

					if (halfExtentsArraySeq && s <= numHalfExtents)
					{
						if (PyList_Check(halfExtentsArraySeq))
						{
							halfExtentsObj = PyList_GET_ITEM(halfExtentsArraySeq, s);
						}
						else
						{
							halfExtentsObj = PyTuple_GET_ITEM(halfExtentsArraySeq, s);
						}
					}
					pybullet_internalSetVectord(halfExtentsObj, halfExtents);
					shapeIndex = b3CreateVisualShapeAddBox(commandHandle, halfExtents);
				}
				if (shapeType == GEOM_CAPSULE && s <= numRadius)
				{
					double radius = pybullet_internalGetFloatFromSequence(radiusArraySeq, s);
					double height = pybullet_internalGetFloatFromSequence(lengthArraySeq, s);
					if (radius > 0 && height >= 0)
					{
						shapeIndex = b3CreateVisualShapeAddCapsule(commandHandle, radius, height);
					}
				}
				if (shapeType == GEOM_CYLINDER && s <= numRadius && s < numLengths)
				{
					double radius = pybullet_internalGetFloatFromSequence(radiusArraySeq, s);
					double height = pybullet_internalGetFloatFromSequence(lengthArraySeq, s);
					if (radius > 0 && height >= 0)
					{
						shapeIndex = b3CreateVisualShapeAddCylinder(commandHandle, radius, height);
					}
				}
				if (shapeType == GEOM_MESH)
				{
					double meshScale[3] = {1, 1, 1};

					PyObject* meshScaleObj = meshScaleArraySeq ? PyList_GET_ITEM(meshScaleArraySeq, s) : 0;
					PyObject* fileNameObj = fileNameArraySeq ? PyList_GET_ITEM(fileNameArraySeq, s) : 0;
					const char* fileName = 0;

					if (fileNameObj)
					{
#if PY_MAJOR_VERSION >= 3
						PyObject* ob = PyUnicode_AsASCIIString(fileNameObj);
						fileName = PyBytes_AS_STRING(ob);
#else
						fileName = PyString_AsString(fileNameObj);
#endif
					}
					if (meshScaleObj)
					{
						pybullet_internalSetVectord(meshScaleObj, meshScale);
					}
					if (fileName)
					{
						shapeIndex = b3CreateVisualShapeAddMesh(commandHandle, fileName, meshScale);
					}
				}
				if (shapeType == GEOM_PLANE)
				{
					PyObject* planeNormalObj = planeNormalArraySeq ? PyList_GET_ITEM(planeNormalArraySeq, s) : 0;
					double planeNormal[3];
					double planeConstant = 0;
					pybullet_internalSetVectord(planeNormalObj, planeNormal);
					shapeIndex = b3CreateVisualShapeAddPlane(commandHandle, planeNormal, planeConstant);
				}
				if (flagsArraySeq)
				{
					int flags = pybullet_internalGetIntFromSequence(flagsArraySeq, s);
					b3CreateVisualSetFlag(commandHandle, shapeIndex, flags);
				}
				if (rgbaColorArraySeq)
				{
					PyObject* rgbaColorObj = rgbaColorArraySeq ? PyList_GET_ITEM(rgbaColorArraySeq, s) : 0;
					double rgbaColor[4] = {1, 1, 1, 1};
					if (rgbaColorObj)
					{
						pybullet_internalSetVector4d(rgbaColorObj, rgbaColor);
					}
					b3CreateVisualShapeSetRGBAColor(commandHandle, shapeIndex, rgbaColor);
				}
				if (positionArraySeq || orientationArraySeq)
				{
					PyObject* visualFramePositionObj = positionArraySeq ? PyList_GET_ITEM(positionArraySeq, s) : 0;
					PyObject* visualFrameOrientationObj = orientationArraySeq ? PyList_GET_ITEM(orientationArraySeq, s) : 0;
					double visualFramePosition[3] = {0, 0, 0};
					double visualFrameOrientation[4] = {0, 0, 0, 1};
					if (visualFramePositionObj)
					{
						pybullet_internalSetVectord(visualFramePositionObj, visualFramePosition);
					}

					if (visualFrameOrientationObj)
					{
						pybullet_internalSetVector4d(visualFrameOrientationObj, visualFrameOrientation);
					}
					if (shapeIndex >= 0)
					{
						b3CreateVisualShapeSetChildTransform(commandHandle, shapeIndex, visualFramePosition, visualFrameOrientation);
					}
				}
			}
		}

		if (shapeTypeArraySeq)
			Py_DECREF(shapeTypeArraySeq);
		if (radiusArraySeq)
			Py_DECREF(radiusArraySeq);
		if (halfExtentsArraySeq)
			Py_DECREF(halfExtentsArraySeq);
		if (lengthArraySeq)
			Py_DECREF(lengthArraySeq);
		if (fileNameArraySeq)
			Py_DECREF(fileNameArraySeq);
		if (meshScaleArraySeq)
			Py_DECREF(meshScaleArraySeq);
		if (planeNormalArraySeq)
			Py_DECREF(planeNormalArraySeq);
		if (rgbaColorArraySeq)
			Py_DECREF(rgbaColorArraySeq);
		if (flagsArraySeq)
			Py_DECREF(flagsArraySeq);
		if (positionArraySeq)
			Py_DECREF(positionArraySeq);
		if (orientationArraySeq)
			Py_DECREF(orientationArraySeq);

		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
		statusType = b3GetStatusType(statusHandle);
		if (statusType == CMD_CREATE_VISUAL_SHAPE_COMPLETED)
		{
			int uid = b3GetStatusVisualShapeUniqueId(statusHandle);
			PyObject* ob = PyLong_FromLong(uid);
			return ob;
		}
	}

	PyErr_SetString(SpamError, "createVisualShapeArray failed.");
	return NULL;
}
static PyObject* pybullet_createMultiBody(PyObject* self, PyObject* args, PyObject* keywds)
{
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	double baseMass = 0;
	int baseCollisionShapeIndex = -1;
	int baseVisualShapeIndex = -1;
	int useMaximalCoordinates = 0;
	int flags = -1;
	PyObject* basePosObj = 0;
	PyObject* baseOrnObj = 0;
	PyObject* baseInertialFramePositionObj = 0;
	PyObject* baseInertialFrameOrientationObj = 0;

	PyObject* linkMassesObj = 0;
	PyObject* linkCollisionShapeIndicesObj = 0;
	PyObject* linkVisualShapeIndicesObj = 0;
	PyObject* linkPositionsObj = 0;
	PyObject* linkOrientationsObj = 0;
	PyObject* linkParentIndicesObj = 0;
	PyObject* linkJointTypesObj = 0;
	PyObject* linkJointAxisObj = 0;
	PyObject* linkInertialFramePositionObj = 0;
	PyObject* linkInertialFrameOrientationObj = 0;
	PyObject* objBatchPositions = 0;
	PyObject* linkNamesObj = 0;

	static char* kwlist[] = {
		"baseMass", "baseCollisionShapeIndex", "baseVisualShapeIndex", "basePosition", "baseOrientation",
		"baseInertialFramePosition", "baseInertialFrameOrientation", "linkMasses", "linkCollisionShapeIndices",
		"linkVisualShapeIndices", "linkPositions", "linkOrientations", "linkInertialFramePositions", "linkInertialFrameOrientations", "linkParentIndices",
		"linkJointTypes", "linkJointAxis", "useMaximalCoordinates", "flags", "batchPositions", "linkNames", "physicsClientId", NULL};

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|diiOOOOOOOOOOOOOOiiOOi", kwlist,
									 &baseMass, &baseCollisionShapeIndex, &baseVisualShapeIndex, &basePosObj, &baseOrnObj,
									 &baseInertialFramePositionObj, &baseInertialFrameOrientationObj, &linkMassesObj, &linkCollisionShapeIndicesObj,
									 &linkVisualShapeIndicesObj, &linkPositionsObj, &linkOrientationsObj, &linkInertialFramePositionObj, &linkInertialFrameOrientationObj, &linkParentIndicesObj,
									 &linkJointTypesObj, &linkJointAxisObj, &useMaximalCoordinates, &flags, &objBatchPositions, &linkNamesObj, &physicsClientId))
	{
		return NULL;
	}

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	{
		int numLinkMasses = linkMassesObj ? PySequence_Size(linkMassesObj) : 0;
		int numLinkCollisionShapes = linkCollisionShapeIndicesObj ? PySequence_Size(linkCollisionShapeIndicesObj) : 0;
		int numLinkVisualShapes = linkVisualShapeIndicesObj ? PySequence_Size(linkVisualShapeIndicesObj) : 0;
		int numLinkPositions = linkPositionsObj ? PySequence_Size(linkPositionsObj) : 0;
		int numLinkOrientations = linkOrientationsObj ? PySequence_Size(linkOrientationsObj) : 0;
		int numLinkParentIndices = linkParentIndicesObj ? PySequence_Size(linkParentIndicesObj) : 0;
		int numLinkJointTypes = linkJointTypesObj ? PySequence_Size(linkJointTypesObj) : 0;
		int numLinkJoinAxis = linkJointAxisObj ? PySequence_Size(linkJointAxisObj) : 0;
		int numLinkInertialFramePositions = linkInertialFramePositionObj ? PySequence_Size(linkInertialFramePositionObj) : 0;
		int numLinkInertialFrameOrientations = linkInertialFrameOrientationObj ? PySequence_Size(linkInertialFrameOrientationObj) : 0;
		int numBatchPositions = objBatchPositions ? PySequence_Size(objBatchPositions) : 0;

		PyObject* seqLinkMasses = linkMassesObj ? PySequence_Fast(linkMassesObj, "expected a sequence") : 0;
		PyObject* seqLinkCollisionShapes = linkCollisionShapeIndicesObj ? PySequence_Fast(linkCollisionShapeIndicesObj, "expected a sequence") : 0;
		PyObject* seqLinkVisualShapes = linkVisualShapeIndicesObj ? PySequence_Fast(linkVisualShapeIndicesObj, "expected a sequence") : 0;
		PyObject* seqLinkPositions = linkPositionsObj ? PySequence_Fast(linkPositionsObj, "expected a sequence") : 0;
		PyObject* seqLinkOrientations = linkOrientationsObj ? PySequence_Fast(linkOrientationsObj, "expected a sequence") : 0;
		PyObject* seqLinkParentIndices = linkParentIndicesObj ? PySequence_Fast(linkParentIndicesObj, "expected a sequence") : 0;
		PyObject* seqLinkJointTypes = linkJointTypesObj ? PySequence_Fast(linkJointTypesObj, "expected a sequence") : 0;
		PyObject* seqLinkJoinAxis = linkJointAxisObj ? PySequence_Fast(linkJointAxisObj, "expected a sequence") : 0;
		PyObject* seqLinkInertialFramePositions = linkInertialFramePositionObj ? PySequence_Fast(linkInertialFramePositionObj, "expected a sequence") : 0;
		PyObject* seqLinkInertialFrameOrientations = linkInertialFrameOrientationObj ? PySequence_Fast(linkInertialFrameOrientationObj, "expected a sequence") : 0;

		PyObject* seqBatchPositions = objBatchPositions ? PySequence_Fast(objBatchPositions, "expected a sequence") : 0;
		PyObject* seqLinkNames = linkNamesObj ? PySequence_Fast(linkNamesObj, "expected a sequence") : 0;

		if ((numLinkMasses == numLinkCollisionShapes) &&
			(numLinkMasses == numLinkVisualShapes) &&
			(numLinkMasses == numLinkPositions) &&
			(numLinkMasses == numLinkOrientations) &&
			(numLinkMasses == numLinkParentIndices) &&
			(numLinkMasses == numLinkJointTypes) &&
			(numLinkMasses == numLinkJoinAxis) &&
			(numLinkMasses == numLinkInertialFramePositions) &&
			(numLinkMasses == numLinkInertialFrameOrientations))
		{
			b3SharedMemoryStatusHandle statusHandle;
			int statusType;
			int i;
			b3SharedMemoryCommandHandle commandHandle = b3CreateMultiBodyCommandInit(sm);
			double basePosition[3] = {0, 0, 0};
			double baseOrientation[4] = {0, 0, 0, 1};
			double baseInertialFramePosition[3] = {0, 0, 0};
			double baseInertialFrameOrientation[4] = {0, 0, 0, 1};
			int baseIndex;
			pybullet_internalSetVectord(basePosObj, basePosition);
			pybullet_internalSetVector4d(baseOrnObj, baseOrientation);
			pybullet_internalSetVectord(baseInertialFramePositionObj, baseInertialFramePosition);
			pybullet_internalSetVector4d(baseInertialFrameOrientationObj, baseInertialFrameOrientation);

			baseIndex = b3CreateMultiBodyBase(commandHandle, baseMass, baseCollisionShapeIndex, baseVisualShapeIndex, basePosition, baseOrientation, baseInertialFramePosition, baseInertialFrameOrientation);

			if (numBatchPositions > 0)
			{
				double* batchPositions = malloc(sizeof(double) * 3 * numBatchPositions);
				for (i = 0; i < numBatchPositions; i++)
				{
					pybullet_internalGetVector3FromSequence(seqBatchPositions, i, &batchPositions[3 * i]);
				}
				b3CreateMultiBodySetBatchPositions(sm, commandHandle, batchPositions, numBatchPositions);
				free(batchPositions);
			}

			for (i = 0; i < numLinkMasses; i++)
			{
				double linkMass = pybullet_internalGetFloatFromSequence(seqLinkMasses, i);
				int linkCollisionShapeIndex = pybullet_internalGetIntFromSequence(seqLinkCollisionShapes, i);
				int linkVisualShapeIndex = pybullet_internalGetIntFromSequence(seqLinkVisualShapes, i);
				double linkPosition[3];
				double linkOrientation[4];
				double linkJointAxis[3];
				double linkInertialFramePosition[3];
				double linkInertialFrameOrientation[4];
				int linkParentIndex;
				int linkJointType;
				const char* linkName;

				pybullet_internalGetVector3FromSequence(seqLinkInertialFramePositions, i, linkInertialFramePosition);
				pybullet_internalGetVector4FromSequence(seqLinkInertialFrameOrientations, i, linkInertialFrameOrientation);
				pybullet_internalGetVector3FromSequence(seqLinkPositions, i, linkPosition);
				pybullet_internalGetVector4FromSequence(seqLinkOrientations, i, linkOrientation);
				pybullet_internalGetVector3FromSequence(seqLinkJoinAxis, i, linkJointAxis);
				linkParentIndex = pybullet_internalGetIntFromSequence(seqLinkParentIndices, i);
				linkJointType = pybullet_internalGetIntFromSequence(seqLinkJointTypes, i);
				linkName = seqLinkNames? pybullet_internalGetCStringFromSequence(seqLinkNames, i) : 0;

				b3CreateMultiBodyLink(commandHandle,
									  linkMass,
									  linkCollisionShapeIndex,
									  linkVisualShapeIndex,
									  linkPosition,
									  linkOrientation,
									  linkInertialFramePosition,
									  linkInertialFrameOrientation,
									  linkParentIndex,
									  linkJointType,
									  linkJointAxis,
									  linkName);
			}

			if (seqLinkMasses)
				Py_DECREF(seqLinkMasses);
			if (seqLinkCollisionShapes)
				Py_DECREF(seqLinkCollisionShapes);
			if (seqLinkVisualShapes)
				Py_DECREF(seqLinkVisualShapes);
			if (seqLinkPositions)
				Py_DECREF(seqLinkPositions);
			if (seqLinkOrientations)
				Py_DECREF(seqLinkOrientations);
			if (seqLinkParentIndices)
				Py_DECREF(seqLinkParentIndices);
			if (seqLinkJointTypes)
				Py_DECREF(seqLinkJointTypes);
			if (seqLinkJoinAxis)
				Py_DECREF(seqLinkJoinAxis);
			if (seqLinkInertialFramePositions)
				Py_DECREF(seqLinkInertialFramePositions);
			if (seqLinkInertialFrameOrientations)
				Py_DECREF(seqLinkInertialFrameOrientations);
			if (seqBatchPositions)
				Py_DECREF(seqBatchPositions);

			if (useMaximalCoordinates > 0)
			{
				b3CreateMultiBodyUseMaximalCoordinates(commandHandle);
			}
			if (flags > 0)
			{
				b3CreateMultiBodySetFlags(commandHandle, flags);
			}
			statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
			statusType = b3GetStatusType(statusHandle);
			if (statusType == CMD_CREATE_MULTI_BODY_COMPLETED)
			{
				int uid = b3GetStatusBodyIndex(statusHandle);
				if (numBatchPositions > 0)
				{
					PyObject* pyResultList = PyTuple_New(numBatchPositions );
					for (i = 0; i < numBatchPositions; i++)
					{
						PyTuple_SetItem(pyResultList, i, PyLong_FromLong(uid - numBatchPositions + i + 1));
					}
					return pyResultList;
					
				}
				else
				{
					PyObject* ob = PyLong_FromLong(uid);
					return ob;
				}
			}
		}
		else
		{
			if (seqLinkMasses)
				Py_DECREF(seqLinkMasses);
			if (seqLinkCollisionShapes)
				Py_DECREF(seqLinkCollisionShapes);
			if (seqLinkVisualShapes)
				Py_DECREF(seqLinkVisualShapes);
			if (seqLinkPositions)
				Py_DECREF(seqLinkPositions);
			if (seqLinkOrientations)
				Py_DECREF(seqLinkOrientations);
			if (seqLinkParentIndices)
				Py_DECREF(seqLinkParentIndices);
			if (seqLinkJointTypes)
				Py_DECREF(seqLinkJointTypes);
			if (seqLinkJoinAxis)
				Py_DECREF(seqLinkJoinAxis);
			if (seqLinkInertialFramePositions)
				Py_DECREF(seqLinkInertialFramePositions);
			if (seqLinkInertialFrameOrientations)
				Py_DECREF(seqLinkInertialFrameOrientations);
			if (seqBatchPositions)
				Py_DECREF(seqBatchPositions);
			PyErr_SetString(SpamError, "All link arrays need to be same size.");
			return NULL;
		}

#if 0
		PyObject* seq;
		seq = PySequence_Fast(objMat, "expected a sequence");
		if (seq)
		{
			len = PySequence_Size(objMat);
			if (len == 16)
			{
				for (i = 0; i < len; i++)
				{
					matrix[i] = pybullet_internalGetFloatFromSequence(seq, i);
				}
				Py_DECREF(seq);
				return 1;
			}
			Py_DECREF(seq);
		}
#endif
	}
	PyErr_SetString(SpamError, "createMultiBody failed.");
	return NULL;
}

static PyObject* pybullet_createUserConstraint(PyObject* self, PyObject* args, PyObject* keywds)
{
	b3SharedMemoryCommandHandle commandHandle;
	int parentBodyUniqueId = -1;
	int parentLinkIndex = -1;
	int childBodyUniqueId = -1;
	int childLinkIndex = -1;
	int jointType = ePoint2PointType;
	PyObject* jointAxisObj = 0;
	double jointAxis[3] = {0, 0, 0};
	PyObject* parentFramePositionObj = 0;
	double parentFramePosition[3] = {0, 0, 0};
	PyObject* childFramePositionObj = 0;
	double childFramePosition[3] = {0, 0, 0};
	PyObject* parentFrameOrientationObj = 0;
	double parentFrameOrientation[4] = {0, 0, 0, 1};
	PyObject* childFrameOrientationObj = 0;
	double childFrameOrientation[4] = {0, 0, 0, 1};

	struct b3JointInfo jointInfo;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	static char* kwlist[] = {"parentBodyUniqueId", "parentLinkIndex",
							 "childBodyUniqueId", "childLinkIndex",
							 "jointType",
							 "jointAxis",
							 "parentFramePosition",
							 "childFramePosition",
							 "parentFrameOrientation",
							 "childFrameOrientation",
							 "physicsClientId",
							 NULL};

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "iiiiiOOO|OOi", kwlist, &parentBodyUniqueId, &parentLinkIndex,
									 &childBodyUniqueId, &childLinkIndex,
									 &jointType, &jointAxisObj,
									 &parentFramePositionObj,
									 &childFramePositionObj,
									 &parentFrameOrientationObj,
									 &childFrameOrientationObj,
									 &physicsClientId))
	{
		return NULL;
	}

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	pybullet_internalSetVectord(jointAxisObj, jointAxis);
	pybullet_internalSetVectord(parentFramePositionObj, parentFramePosition);
	pybullet_internalSetVectord(childFramePositionObj, childFramePosition);
	pybullet_internalSetVector4d(parentFrameOrientationObj, parentFrameOrientation);
	pybullet_internalSetVector4d(childFrameOrientationObj, childFrameOrientation);

	jointInfo.m_jointType = jointType;
	jointInfo.m_parentFrame[0] = parentFramePosition[0];
	jointInfo.m_parentFrame[1] = parentFramePosition[1];
	jointInfo.m_parentFrame[2] = parentFramePosition[2];
	jointInfo.m_parentFrame[3] = parentFrameOrientation[0];
	jointInfo.m_parentFrame[4] = parentFrameOrientation[1];
	jointInfo.m_parentFrame[5] = parentFrameOrientation[2];
	jointInfo.m_parentFrame[6] = parentFrameOrientation[3];

	jointInfo.m_childFrame[0] = childFramePosition[0];
	jointInfo.m_childFrame[1] = childFramePosition[1];
	jointInfo.m_childFrame[2] = childFramePosition[2];
	jointInfo.m_childFrame[3] = childFrameOrientation[0];
	jointInfo.m_childFrame[4] = childFrameOrientation[1];
	jointInfo.m_childFrame[5] = childFrameOrientation[2];
	jointInfo.m_childFrame[6] = childFrameOrientation[3];

	jointInfo.m_jointAxis[0] = jointAxis[0];
	jointInfo.m_jointAxis[1] = jointAxis[1];
	jointInfo.m_jointAxis[2] = jointAxis[2];

	commandHandle = b3InitCreateUserConstraintCommand(sm, parentBodyUniqueId, parentLinkIndex, childBodyUniqueId, childLinkIndex, &jointInfo);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	if (statusType == CMD_USER_CONSTRAINT_COMPLETED)
	{
		int userConstraintUid = b3GetStatusUserConstraintUniqueId(statusHandle);
		PyObject* ob = PyLong_FromLong(userConstraintUid);
		return ob;
	}

	PyErr_SetString(SpamError, "createConstraint failed.");
	return NULL;
}

static PyObject* pybullet_getContactPointData(PyObject* self, PyObject* args, PyObject* keywds)
{
	int bodyUniqueIdA = -1;
	int bodyUniqueIdB = -1;
	int linkIndexA = -2;
	int linkIndexB = -2;

	b3SharedMemoryCommandHandle commandHandle;
	struct b3ContactInformation contactPointData;
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;

	static char* kwlist[] = {"bodyA", "bodyB", "linkIndexA", "linkIndexB", "physicsClientId", NULL};

	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|iiiii", kwlist,
									 &bodyUniqueIdA, &bodyUniqueIdB, &linkIndexA, &linkIndexB, &physicsClientId))
		return NULL;

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	commandHandle = b3InitRequestContactPointInformation(sm);
	if (bodyUniqueIdA >= 0)
	{
		b3SetContactFilterBodyA(commandHandle, bodyUniqueIdA);
	}
	if (bodyUniqueIdB >= 0)
	{
		b3SetContactFilterBodyB(commandHandle, bodyUniqueIdB);
	}

	if (linkIndexA >= -1)
	{
		b3SetContactFilterLinkA(commandHandle, linkIndexA);
	}
	if (linkIndexB >= -1)
	{
		b3SetContactFilterLinkB(commandHandle, linkIndexB);
	}

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
	statusType = b3GetStatusType(statusHandle);
	if (statusType == CMD_CONTACT_POINT_INFORMATION_COMPLETED)
	{
		b3GetContactPointInformation(sm, &contactPointData);

		return MyConvertContactPoint(&contactPointData);
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_isNumpyEnabled(PyObject* self, PyObject* args, PyObject* keywds)
{
	int physicsClientId = 0;
	int isNumpyEnabled = 0;
	int method = 0;
	PyObject* pylist = 0;
	PyObject* val = 0;
	b3PhysicsClientHandle sm = 0;
	static char* kwlist[] = {"physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "|i", kwlist, &physicsClientId))
	{
		return NULL;
	}

#ifdef PYBULLET_USE_NUMPY
	isNumpyEnabled = 1;
#endif
	return PyLong_FromLong(isNumpyEnabled);
}

/// Render an image from the current timestep of the simulation, width, height are required, other args are optional
// getCameraImage(w, h, view[16], projection[16], lightDir[3], lightColor[3], lightDist, hasShadow, lightAmbientCoeff, lightDiffuseCoeff, lightSpecularCoeff, renderer)
static PyObject* pybullet_getCameraImage(PyObject* self, PyObject* args, PyObject* keywds)
{
	/// request an image from a simulated camera, using software or hardware renderer.
	struct b3CameraImageData imageData;
	PyObject *objViewMat = 0, *objProjMat = 0, *lightDirObj = 0, *lightColorObj = 0, *objProjectiveTextureView = 0, *objProjectiveTextureProj = 0;
	int width, height;
	float viewMatrix[16];
	float projectionMatrix[16];
	float projectiveTextureView[16];
	float projectiveTextureProj[16];
	float lightDir[3];
	float lightColor[3];
	float lightDist = -1;
	int hasShadow = -1;
	float lightAmbientCoeff = -1;
	float lightDiffuseCoeff = -1;
	float lightSpecularCoeff = -1;
	int flags = -1;
	int renderer = -1;
	// inialize cmd
	b3SharedMemoryCommandHandle command;
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	// set camera resolution, optionally view, projection matrix, light direction, light color, light distance, shadow
	static char* kwlist[] = {"width", "height", "viewMatrix", "projectionMatrix", "lightDirection", "lightColor", "lightDistance", "shadow", "lightAmbientCoeff", "lightDiffuseCoeff", "lightSpecularCoeff", "renderer", "flags", "projectiveTextureView", "projectiveTextureProj", "physicsClientId", NULL};

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "ii|OOOOfifffiiOOi", kwlist, &width, &height, &objViewMat, &objProjMat, &lightDirObj, &lightColorObj, &lightDist, &hasShadow, &lightAmbientCoeff, &lightDiffuseCoeff, &lightSpecularCoeff, &renderer, &flags, &objProjectiveTextureView, &objProjectiveTextureProj, &physicsClientId))
	{
		return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	command = b3InitRequestCameraImage(sm);
	b3RequestCameraImageSetPixelResolution(command, width, height);

	// set camera matrices only if set matrix function succeeds
	if (objViewMat && objProjMat && pybullet_internalSetMatrix(objViewMat, viewMatrix) && (pybullet_internalSetMatrix(objProjMat, projectionMatrix)))
	{
		b3RequestCameraImageSetCameraMatrices(command, viewMatrix, projectionMatrix);
	}
	//set light direction only if function succeeds
	if (lightDirObj && pybullet_internalSetVector(lightDirObj, lightDir))
	{
		b3RequestCameraImageSetLightDirection(command, lightDir);
	}
	//set light color only if function succeeds
	if (pybullet_internalSetVector(lightColorObj, lightColor))
	{
		b3RequestCameraImageSetLightColor(command, lightColor);
	}
	if (lightDist >= 0)
	{
		b3RequestCameraImageSetLightDistance(command, lightDist);
	}

	if (hasShadow >= 0)
	{
		b3RequestCameraImageSetShadow(command, hasShadow);
	}
	if (lightAmbientCoeff >= 0)
	{
		b3RequestCameraImageSetLightAmbientCoeff(command, lightAmbientCoeff);
	}
	if (lightDiffuseCoeff >= 0)
	{
		b3RequestCameraImageSetLightDiffuseCoeff(command, lightDiffuseCoeff);
	}

	if (lightSpecularCoeff >= 0)
	{
		b3RequestCameraImageSetLightSpecularCoeff(command, lightSpecularCoeff);
	}

	if (flags >= 0)
	{
		b3RequestCameraImageSetFlags(command, flags);
	}
	if (objProjectiveTextureView && objProjectiveTextureProj && pybullet_internalSetMatrix(objProjectiveTextureView, projectiveTextureView) && (pybullet_internalSetMatrix(objProjectiveTextureProj, projectiveTextureProj)))
	{
		b3RequestCameraImageSetProjectiveTextureMatrices(command, projectiveTextureView, projectiveTextureProj);
	}
	if (renderer >= 0)
	{
		b3RequestCameraImageSelectRenderer(command, renderer);  //renderer could be ER_BULLET_HARDWARE_OPENGL
	}
	//PyErr_Clear();

	if (b3CanSubmitCommand(sm))
	{
		b3SharedMemoryStatusHandle statusHandle;
		int statusType;

		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
		statusType = b3GetStatusType(statusHandle);
		if (statusType == CMD_CAMERA_IMAGE_COMPLETED)
		{
			PyObject* pyResultList;  // store 4 elements in this result: width,
									 // height, rgbData, depth

#ifdef PYBULLET_USE_NUMPY
			PyObject* pyRGB;
			PyObject* pyDep;
			PyObject* pySeg;

			int bytesPerPixel = 4;  // Red, Green, Blue, and Alpha each 8 bit values

			b3GetCameraImageData(sm, &imageData);
			// TODO(hellojas): error handling if image size is 0
			{
				npy_intp rgb_dims[3] = {imageData.m_pixelHeight, imageData.m_pixelWidth,
										bytesPerPixel};
				npy_intp dep_dims[2] = {imageData.m_pixelHeight, imageData.m_pixelWidth};
				npy_intp seg_dims[2] = {imageData.m_pixelHeight, imageData.m_pixelWidth};

				pyResultList = PyTuple_New(5);

				PyTuple_SetItem(pyResultList, 0, PyInt_FromLong(imageData.m_pixelWidth));
				PyTuple_SetItem(pyResultList, 1, PyInt_FromLong(imageData.m_pixelHeight));

				pyRGB = PyArray_SimpleNew(3, rgb_dims, NPY_UINT8);
				pyDep = PyArray_SimpleNew(2, dep_dims, NPY_FLOAT32);
				pySeg = PyArray_SimpleNew(2, seg_dims, NPY_INT32);

				memcpy(PyArray_DATA(pyRGB), imageData.m_rgbColorData,
					   imageData.m_pixelHeight * imageData.m_pixelWidth * bytesPerPixel);
				memcpy(PyArray_DATA(pyDep), imageData.m_depthValues,
					   imageData.m_pixelHeight * imageData.m_pixelWidth * sizeof(float));
				memcpy(PyArray_DATA(pySeg), imageData.m_segmentationMaskValues,
					   imageData.m_pixelHeight * imageData.m_pixelWidth * sizeof(int));

				PyTuple_SetItem(pyResultList, 2, pyRGB);
				PyTuple_SetItem(pyResultList, 3, pyDep);
				PyTuple_SetItem(pyResultList, 4, pySeg);
			}
#else   //PYBULLET_USE_NUMPY
			PyObject* item2;
			PyObject* pylistRGB;
			PyObject* pylistDep;
			PyObject* pylistSeg;

			int i, j, p;

			b3GetCameraImageData(sm, &imageData);
			// TODO(hellojas): error handling if image size is 0
			pyResultList = PyTuple_New(5);
			PyTuple_SetItem(pyResultList, 0, PyInt_FromLong(imageData.m_pixelWidth));
			PyTuple_SetItem(pyResultList, 1, PyInt_FromLong(imageData.m_pixelHeight));

			{
				PyObject* item;
				int bytesPerPixel = 4;  // Red, Green, Blue, and Alpha each 8 bit values
				int num =
					bytesPerPixel * imageData.m_pixelWidth * imageData.m_pixelHeight;
				pylistRGB = PyTuple_New(num);
				pylistDep =
					PyTuple_New(imageData.m_pixelWidth * imageData.m_pixelHeight);
				pylistSeg =
					PyTuple_New(imageData.m_pixelWidth * imageData.m_pixelHeight);
				for (i = 0; i < imageData.m_pixelWidth; i++)
				{
					for (j = 0; j < imageData.m_pixelHeight; j++)
					{
						// TODO(hellojas): validate depth values make sense
						int depIndex = i + j * imageData.m_pixelWidth;
						{
							item = PyFloat_FromDouble(imageData.m_depthValues[depIndex]);
							PyTuple_SetItem(pylistDep, depIndex, item);
						}
						{
							item2 =
								PyLong_FromLong(imageData.m_segmentationMaskValues[depIndex]);
							PyTuple_SetItem(pylistSeg, depIndex, item2);
						}

						for (p = 0; p < bytesPerPixel; p++)
						{
							int pixelIndex =
								bytesPerPixel * (i + j * imageData.m_pixelWidth) + p;
							item = PyInt_FromLong(imageData.m_rgbColorData[pixelIndex]);
							PyTuple_SetItem(pylistRGB, pixelIndex, item);
						}
					}
				}
			}

			PyTuple_SetItem(pyResultList, 2, pylistRGB);
			PyTuple_SetItem(pyResultList, 3, pylistDep);
			PyTuple_SetItem(pyResultList, 4, pylistSeg);
			return pyResultList;
#endif  //PYBULLET_USE_NUMPY

			return pyResultList;
		}
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_computeViewMatrix(PyObject* self, PyObject* args, PyObject* keywds)
{
	PyObject* camEyeObj = 0;
	PyObject* camTargetPositionObj = 0;
	PyObject* camUpVectorObj = 0;
	float camEye[3];
	float camTargetPosition[3];
	float camUpVector[3];
	int physicsClientId = 0;
	// set camera resolution, optionally view, projection matrix, light position
	static char* kwlist[] = {"cameraEyePosition", "cameraTargetPosition", "cameraUpVector", "physicsClientId", NULL};

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "OOO|i", kwlist, &camEyeObj, &camTargetPositionObj, &camUpVectorObj, &physicsClientId))
	{
		return NULL;
	}

	if (pybullet_internalSetVector(camEyeObj, camEye) &&
		pybullet_internalSetVector(camTargetPositionObj, camTargetPosition) &&
		pybullet_internalSetVector(camUpVectorObj, camUpVector))
	{
		float viewMatrix[16];
		PyObject* pyResultList = 0;
		int i;
		b3ComputeViewMatrixFromPositions(camEye, camTargetPosition, camUpVector, viewMatrix);

		pyResultList = PyTuple_New(16);
		for (i = 0; i < 16; i++)
		{
			PyObject* item = PyFloat_FromDouble(viewMatrix[i]);
			PyTuple_SetItem(pyResultList, i, item);
		}
		return pyResultList;
	}

	PyErr_SetString(SpamError, "Error in computeViewMatrix.");
	return NULL;
}

///compute a view matrix, helper function for b3RequestCameraImageSetCameraMatrices
static PyObject* pybullet_computeViewMatrixFromYawPitchRoll(PyObject* self, PyObject* args, PyObject* keywds)
{
	PyObject* cameraTargetPositionObj = 0;
	float cameraTargetPosition[3];
	float distance, yaw, pitch, roll;
	int upAxisIndex;
	float viewMatrix[16];
	PyObject* pyResultList = 0;
	int i;
	int physicsClientId = 0;

	// set camera resolution, optionally view, projection matrix, light position
	static char* kwlist[] = {"cameraTargetPosition", "distance", "yaw", "pitch", "roll", "upAxisIndex", "physicsClientId", NULL};

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "Offffi|i", kwlist, &cameraTargetPositionObj, &distance, &yaw, &pitch, &roll, &upAxisIndex, &physicsClientId))
	{
		return NULL;
	}

	if (!pybullet_internalSetVector(cameraTargetPositionObj, cameraTargetPosition))
	{
		PyErr_SetString(SpamError, "Cannot convert cameraTargetPosition.");
		return NULL;
	}

	b3ComputeViewMatrixFromYawPitchRoll(cameraTargetPosition, distance, yaw, pitch, roll, upAxisIndex, viewMatrix);

	pyResultList = PyTuple_New(16);
	for (i = 0; i < 16; i++)
	{
		PyObject* item = PyFloat_FromDouble(viewMatrix[i]);
		PyTuple_SetItem(pyResultList, i, item);
	}
	return pyResultList;
}

///compute a projection matrix, helper function for b3RequestCameraImageSetCameraMatrices
static PyObject* pybullet_computeProjectionMatrix(PyObject* self, PyObject* args, PyObject* keywds)
{
	PyObject* pyResultList = 0;
	float left;
	float right;
	float bottom;
	float top;
	float nearVal;
	float farVal;
	float projectionMatrix[16];
	int i;
	int physicsClientId;

	// set camera resolution, optionally view, projection matrix, light position
	static char* kwlist[] = {"left", "right", "bottom", "top", "nearVal", "farVal", "physicsClientId", NULL};

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "ffffff|i", kwlist, &left, &right, &bottom, &top, &nearVal, &farVal, &physicsClientId))
	{
		return NULL;
	}

	b3ComputeProjectionMatrix(left, right, bottom, top, nearVal, farVal, projectionMatrix);

	pyResultList = PyTuple_New(16);
	for (i = 0; i < 16; i++)
	{
		PyObject* item = PyFloat_FromDouble(projectionMatrix[i]);
		PyTuple_SetItem(pyResultList, i, item);
	}
	return pyResultList;
}

static PyObject* pybullet_computeProjectionMatrixFOV(PyObject* self, PyObject* args, PyObject* keywds)
{
	float fov, aspect, nearVal, farVal;
	PyObject* pyResultList = 0;
	float projectionMatrix[16];
	int i;
	int physicsClientId = 0;

	static char* kwlist[] = {"fov", "aspect", "nearVal", "farVal", "physicsClientId", NULL};

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "ffff|i", kwlist, &fov, &aspect, &nearVal, &farVal, &physicsClientId))
	{
		return NULL;
	}

	b3ComputeProjectionMatrixFOV(fov, aspect, nearVal, farVal, projectionMatrix);

	pyResultList = PyTuple_New(16);
	for (i = 0; i < 16; i++)
	{
		PyObject* item = PyFloat_FromDouble(projectionMatrix[i]);
		PyTuple_SetItem(pyResultList, i, item);
	}
	return pyResultList;
}

// Render an image from the current timestep of the simulation
//
// Examples:
//  renderImage() - default image resolution and camera position
//  renderImage(w, h) - image resolution of (w,h), default camera
//  renderImage(w, h, view[16], projection[16]) - set both resolution
//    and initialize camera to the view and projection values
//  renderImage(w, h, cameraPos, targetPos, cameraUp, nearVal, farVal) - set
//    resolution and initialize camera based on camera position, target
//    position, camera up and fulstrum near/far values.
//  renderImage(w, h, cameraPos, targetPos, cameraUp, nearVal, farVal, fov) -
//    set resolution and initialize camera based on camera position, target
//    position, camera up, fulstrum near/far values and camera field of view.
//  renderImage(w, h, targetPos, distance, yaw, pitch, upAxisIndex, nearVal,
//  farVal, fov)

//
// Note if the (w,h) is too small, the objects may not appear based on
// where the camera has been set
//
// TODO(hellojas): fix image is cut off at head
// TODO(hellojas): should we add check to give minimum image resolution
//  to see object based on camera position?
static PyObject* pybullet_renderImageObsolete(PyObject* self, PyObject* args)
{
	/// request an image from a simulated camera, using a software renderer.
	struct b3CameraImageData imageData;
	PyObject *objViewMat, *objProjMat;
	PyObject *objCameraPos, *objTargetPos, *objCameraUp;

	int width, height;
	int size = PySequence_Size(args);
	float viewMatrix[16];
	float projectionMatrix[16];

	float cameraPos[3];
	float targetPos[3];
	float cameraUp[3];

	float left, right, bottom, top, aspect;
	float nearVal, farVal;
	float fov;

	// inialize cmd
	b3SharedMemoryCommandHandle command;
	b3PhysicsClientHandle sm;
	int physicsClientId = 0;

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
	command = b3InitRequestCameraImage(sm);

	if (size == 2)  // only set camera resolution
	{
		if (PyArg_ParseTuple(args, "ii", &width, &height))
		{
			b3RequestCameraImageSetPixelResolution(command, width, height);
		}
	}
	else if (size == 4)  // set camera resolution and view and projection matrix
	{
		if (PyArg_ParseTuple(args, "iiOO", &width, &height, &objViewMat,
							 &objProjMat))
		{
			b3RequestCameraImageSetPixelResolution(command, width, height);

			// set camera matrices only if set matrix function succeeds
			if (pybullet_internalSetMatrix(objViewMat, viewMatrix) &&
				(pybullet_internalSetMatrix(objProjMat, projectionMatrix)))
			{
				b3RequestCameraImageSetCameraMatrices(command, viewMatrix,
													  projectionMatrix);
			}
			else
			{
				PyErr_SetString(SpamError, "Error parsing view or projection matrix.");
				return NULL;
			}
		}
	}
	else if (size == 7)  // set camera resolution, camera positions and
						 // calculate projection using near/far values.
	{
		if (PyArg_ParseTuple(args, "iiOOOff", &width, &height, &objCameraPos,
							 &objTargetPos, &objCameraUp, &nearVal, &farVal))
		{
			b3RequestCameraImageSetPixelResolution(command, width, height);
			if (pybullet_internalSetVector(objCameraPos, cameraPos) &&
				pybullet_internalSetVector(objTargetPos, targetPos) &&
				pybullet_internalSetVector(objCameraUp, cameraUp))
			{
				b3RequestCameraImageSetViewMatrix(command, cameraPos, targetPos,
												  cameraUp);
			}
			else
			{
				PyErr_SetString(SpamError,
								"Error parsing camera position, target or up.");
				return NULL;
			}

			aspect = width / height;
			left = -aspect * nearVal;
			right = aspect * nearVal;
			bottom = -nearVal;
			top = nearVal;
			b3RequestCameraImageSetProjectionMatrix(command, left, right, bottom, top,
													nearVal, farVal);
		}
	}
	else if (size == 8)  // set camera resolution, camera positions and
						 // calculate projection using near/far values & field
						 // of view
	{
		if (PyArg_ParseTuple(args, "iiOOOfff", &width, &height, &objCameraPos,
							 &objTargetPos, &objCameraUp, &nearVal, &farVal,
							 &fov))
		{
			b3RequestCameraImageSetPixelResolution(command, width, height);
			if (pybullet_internalSetVector(objCameraPos, cameraPos) &&
				pybullet_internalSetVector(objTargetPos, targetPos) &&
				pybullet_internalSetVector(objCameraUp, cameraUp))
			{
				b3RequestCameraImageSetViewMatrix(command, cameraPos, targetPos,
												  cameraUp);
			}
			else
			{
				PyErr_SetString(SpamError,
								"Error parsing camera position, target or up.");
				return NULL;
			}

			aspect = width / height;
			b3RequestCameraImageSetFOVProjectionMatrix(command, fov, aspect, nearVal,
													   farVal);
		}
	}
	else if (size == 11)
	{
		int upAxisIndex = 1;
		float camDistance, yaw, pitch, roll;

		// sometimes more arguments are better :-)
		if (PyArg_ParseTuple(args, "iiOffffifff", &width, &height, &objTargetPos,
							 &camDistance, &yaw, &pitch, &roll, &upAxisIndex,
							 &nearVal, &farVal, &fov))
		{
			b3RequestCameraImageSetPixelResolution(command, width, height);
			if (pybullet_internalSetVector(objTargetPos, targetPos))
			{
				// printf("width = %d, height = %d, targetPos = %f,%f,%f, distance = %f,
				// yaw = %f, pitch = %f,   upAxisIndex = %d, near=%f, far=%f,
				// fov=%f\n",width,height,targetPos[0],targetPos[1],targetPos[2],camDistance,yaw,pitch,upAxisIndex,nearVal,farVal,fov);

				b3RequestCameraImageSetViewMatrix2(command, targetPos, camDistance, yaw,
												   pitch, roll, upAxisIndex);
				aspect = width / height;
				b3RequestCameraImageSetFOVProjectionMatrix(command, fov, aspect,
														   nearVal, farVal);
			}
			else
			{
				PyErr_SetString(SpamError, "Error parsing camera target pos");
			}
		}
		else
		{
			PyErr_SetString(SpamError, "Error parsing arguments");
		}
	}
	else
	{
		PyErr_SetString(SpamError, "Invalid number of args passed to renderImage.");
		return NULL;
	}

	if (b3CanSubmitCommand(sm))
	{
		b3SharedMemoryStatusHandle statusHandle;
		int statusType;

		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
		statusType = b3GetStatusType(statusHandle);
		if (statusType == CMD_CAMERA_IMAGE_COMPLETED)
		{
			PyObject* pyResultList;  // store 4 elements in this result: width,
									 // height, rgbData, depth

#ifdef PYBULLET_USE_NUMPY
			PyObject* pyRGB;
			PyObject* pyDep;
			PyObject* pySeg;

			int bytesPerPixel = 4;  // Red, Green, Blue, and Alpha each 8 bit values

			b3GetCameraImageData(sm, &imageData);
			// TODO(hellojas): error handling if image size is 0
			pyResultList = PyTuple_New(5);
			PyTuple_SetItem(pyResultList, 0, PyInt_FromLong(imageData.m_pixelWidth));
			PyTuple_SetItem(pyResultList, 1, PyInt_FromLong(imageData.m_pixelHeight));
			{
				npy_intp rgb_dims[3] = {imageData.m_pixelHeight, imageData.m_pixelWidth,
										bytesPerPixel};
				npy_intp dep_dims[2] = {imageData.m_pixelHeight, imageData.m_pixelWidth};
				npy_intp seg_dims[2] = {imageData.m_pixelHeight, imageData.m_pixelWidth};

				pyRGB = PyArray_SimpleNew(3, rgb_dims, NPY_UINT8);
				pyDep = PyArray_SimpleNew(2, dep_dims, NPY_FLOAT32);
				pySeg = PyArray_SimpleNew(2, seg_dims, NPY_INT32);

				memcpy(PyArray_DATA(pyRGB), imageData.m_rgbColorData,
					   imageData.m_pixelHeight * imageData.m_pixelWidth * bytesPerPixel);
				memcpy(PyArray_DATA(pyDep), imageData.m_depthValues,
					   imageData.m_pixelHeight * imageData.m_pixelWidth);
				memcpy(PyArray_DATA(pySeg), imageData.m_segmentationMaskValues,
					   imageData.m_pixelHeight * imageData.m_pixelWidth);

				PyTuple_SetItem(pyResultList, 2, pyRGB);
				PyTuple_SetItem(pyResultList, 3, pyDep);
				PyTuple_SetItem(pyResultList, 4, pySeg);
			}
#else   //PYBULLET_USE_NUMPY
			PyObject* item2;
			PyObject* pylistRGB;
			PyObject* pylistDep;
			PyObject* pylistSeg;

			int i, j, p;

			b3GetCameraImageData(sm, &imageData);
			// TODO(hellojas): error handling if image size is 0
			pyResultList = PyTuple_New(5);
			PyTuple_SetItem(pyResultList, 0, PyInt_FromLong(imageData.m_pixelWidth));
			PyTuple_SetItem(pyResultList, 1, PyInt_FromLong(imageData.m_pixelHeight));

			{
				PyObject* item;
				int bytesPerPixel = 4;  // Red, Green, Blue, and Alpha each 8 bit values
				int num =
					bytesPerPixel * imageData.m_pixelWidth * imageData.m_pixelHeight;
				pylistRGB = PyTuple_New(num);
				pylistDep =
					PyTuple_New(imageData.m_pixelWidth * imageData.m_pixelHeight);
				pylistSeg =
					PyTuple_New(imageData.m_pixelWidth * imageData.m_pixelHeight);
				for (i = 0; i < imageData.m_pixelWidth; i++)
				{
					for (j = 0; j < imageData.m_pixelHeight; j++)
					{
						// TODO(hellojas): validate depth values make sense
						int depIndex = i + j * imageData.m_pixelWidth;
						{
							item = PyFloat_FromDouble(imageData.m_depthValues[depIndex]);
							PyTuple_SetItem(pylistDep, depIndex, item);
						}
						{
							item2 =
								PyLong_FromLong(imageData.m_segmentationMaskValues[depIndex]);
							PyTuple_SetItem(pylistSeg, depIndex, item2);
						}

						for (p = 0; p < bytesPerPixel; p++)
						{
							int pixelIndex =
								bytesPerPixel * (i + j * imageData.m_pixelWidth) + p;
							item = PyInt_FromLong(imageData.m_rgbColorData[pixelIndex]);
							PyTuple_SetItem(pylistRGB, pixelIndex, item);
						}
					}
				}
			}

			PyTuple_SetItem(pyResultList, 2, pylistRGB);
			PyTuple_SetItem(pyResultList, 3, pylistDep);
			PyTuple_SetItem(pyResultList, 4, pylistSeg);
			return pyResultList;
#endif  //PYBULLET_USE_NUMPY

			return pyResultList;
		}
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_applyExternalForce(PyObject* self, PyObject* args, PyObject* keywds)
{
	{
		int objectUniqueId = -1, linkIndex = -1, flags;
		double force[3];
		double position[3] = {0.0, 0.0, 0.0};
		PyObject *forceObj = 0, *posObj = 0;

		b3SharedMemoryCommandHandle command;
		b3SharedMemoryStatusHandle statusHandle;
		int physicsClientId = 0;
		b3PhysicsClientHandle sm = 0;
		static char* kwlist[] = {"objectUniqueId", "linkIndex",
								 "forceObj", "posObj", "flags", "physicsClientId", NULL};

		if (!PyArg_ParseTupleAndKeywords(args, keywds, "iiOOi|i", kwlist, &objectUniqueId, &linkIndex,
										 &forceObj, &posObj, &flags, &physicsClientId))
		{
			return NULL;
		}
		sm = getPhysicsClient(physicsClientId);
		if (sm == 0)
		{
			PyErr_SetString(SpamError, "Not connected to physics server.");
			return NULL;
		}
		{
			PyObject* seq;
			int len, i;
			seq = PySequence_Fast(forceObj, "expected a sequence");
			len = PySequence_Size(forceObj);
			if (len == 3)
			{
				for (i = 0; i < 3; i++)
				{
					force[i] = pybullet_internalGetFloatFromSequence(seq, i);
				}
			}
			else
			{
				PyErr_SetString(SpamError, "force needs a 3 coordinates [x,y,z].");
				Py_DECREF(seq);
				return NULL;
			}
			Py_DECREF(seq);
		}
		{
			PyObject* seq;
			int len, i;
			seq = PySequence_Fast(posObj, "expected a sequence");
			len = PySequence_Size(posObj);
			if (len == 3)
			{
				for (i = 0; i < 3; i++)
				{
					position[i] = pybullet_internalGetFloatFromSequence(seq, i);
				}
			}
			else
			{
				PyErr_SetString(SpamError, "position needs a 3 coordinates [x,y,z].");
				Py_DECREF(seq);
				return NULL;
			}
			Py_DECREF(seq);
		}
		if ((flags != EF_WORLD_FRAME) && (flags != EF_LINK_FRAME))
		{
			PyErr_SetString(SpamError,
							"flag has to be either WORLD_FRAME or LINK_FRAME");
			return NULL;
		}
		command = b3ApplyExternalForceCommandInit(sm);
		b3ApplyExternalForce(command, objectUniqueId, linkIndex, force, position,
							 flags);
		statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	}
	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_applyExternalTorque(PyObject* self, PyObject* args, PyObject* keywds)
{
	{
		int objectUniqueId, linkIndex, flags;
		double torque[3];
		PyObject* torqueObj;
		int physicsClientId = 0;
		b3PhysicsClientHandle sm = 0;
		static char* kwlist[] = {"objectUniqueId", "linkIndex", "torqueObj",
								 "flags", "physicsClientId", NULL};

		if (!PyArg_ParseTupleAndKeywords(args, keywds, "iiOi|i", kwlist, &objectUniqueId, &linkIndex, &torqueObj,
										 &flags, &physicsClientId))
		{
			return NULL;
		}
		sm = getPhysicsClient(physicsClientId);
		if (sm == 0)
		{
			PyErr_SetString(SpamError, "Not connected to physics server.");
			return NULL;
		}

		{
			PyObject* seq;
			int len, i;
			seq = PySequence_Fast(torqueObj, "expected a sequence");
			len = PySequence_Size(torqueObj);
			if (len == 3)
			{
				for (i = 0; i < 3; i++)
				{
					torque[i] = pybullet_internalGetFloatFromSequence(seq, i);
				}
			}
			else
			{
				PyErr_SetString(SpamError, "torque needs a 3 coordinates [x,y,z].");
				Py_DECREF(seq);
				return NULL;
			}
			Py_DECREF(seq);

			if (linkIndex < -1)
			{
				PyErr_SetString(SpamError,
								"Invalid link index, has to be -1 or larger");
				return NULL;
			}
			if ((flags != EF_WORLD_FRAME) && (flags != EF_LINK_FRAME))
			{
				PyErr_SetString(SpamError,
								"flag has to be either WORLD_FRAME or LINK_FRAME");
				return NULL;
			}
			{
				b3SharedMemoryStatusHandle statusHandle;
				b3SharedMemoryCommandHandle command =
					b3ApplyExternalForceCommandInit(sm);
				b3ApplyExternalTorque(command, objectUniqueId, linkIndex, torque, flags);
				statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
			}
		}
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_getQuaternionFromEuler(PyObject* self,
												 PyObject* args, PyObject* keywds)
{
	double rpy[3];

	PyObject* eulerObj;
	int physicsClientId = 0;

	static char* kwlist[] = {"eulerAngles", "physicsClientId", NULL};

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "O|i", kwlist, &eulerObj, &physicsClientId))
	{
		return NULL;
	}

	if (eulerObj)
	{
		PyObject* seq;
		int len, i;
		seq = PySequence_Fast(eulerObj, "expected a sequence");
		len = PySequence_Size(eulerObj);
		if (len == 3)
		{
			for (i = 0; i < 3; i++)
			{
				rpy[i] = pybullet_internalGetFloatFromSequence(seq, i);
			}
		}
		else
		{
			PyErr_SetString(SpamError,
							"Euler angles need a 3 coordinates [roll, pitch, yaw].");
			Py_DECREF(seq);
			return NULL;
		}
		Py_DECREF(seq);
	}
	else
	{
		PyErr_SetString(SpamError,
						"Euler angles need a 3 coordinates [roll, pitch, yaw].");
		return NULL;
	}

	{
		double phi, the, psi;
		double roll = rpy[0];
		double pitch = rpy[1];
		double yaw = rpy[2];
		phi = roll / 2.0;
		the = pitch / 2.0;
		psi = yaw / 2.0;
		{
			double quat[4] = {
				sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi),
				cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi),
				cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi),
				cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi)};

			// normalize the quaternion
			double len = sqrt(quat[0] * quat[0] + quat[1] * quat[1] +
							  quat[2] * quat[2] + quat[3] * quat[3]);
			quat[0] /= len;
			quat[1] /= len;
			quat[2] /= len;
			quat[3] /= len;
			{
				PyObject* pylist;
				int i;
				pylist = PyTuple_New(4);
				for (i = 0; i < 4; i++)
					PyTuple_SetItem(pylist, i, PyFloat_FromDouble(quat[i]));
				return pylist;
			}
		}
	}
	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_multiplyTransforms(PyObject* self,
											 PyObject* args, PyObject* keywds)
{
	PyObject* posAObj = 0;
	PyObject* ornAObj = 0;
	PyObject* posBObj = 0;
	PyObject* ornBObj = 0;

	int hasPosA = 0;
	int hasOrnA = 0;
	int hasPosB = 0;
	int hasOrnB = 0;

	double posA[3];
	double ornA[4] = {0, 0, 0, 1};
	double posB[3];
	double ornB[4] = {0, 0, 0, 1};
	int physicsClientId = 0;

	static char* kwlist[] = {"positionA", "orientationA", "positionB", "orientationB", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "OOOO|i", kwlist, &posAObj, &ornAObj, &posBObj, &ornBObj, &physicsClientId))
	{
		return NULL;
	}

	hasPosA = pybullet_internalSetVectord(posAObj, posA);
	hasOrnA = pybullet_internalSetVector4d(ornAObj, ornA);
	hasPosB = pybullet_internalSetVectord(posBObj, posB);
	hasOrnB = pybullet_internalSetVector4d(ornBObj, ornB);

	if (hasPosA && hasOrnA && hasPosB && hasOrnB)
	{
		double outPos[3];
		double outOrn[4];
		int i;
		PyObject* pyListOutObj = 0;
		PyObject* pyPosOutObj = 0;
		PyObject* pyOrnOutObj = 0;

		b3MultiplyTransforms(posA, ornA, posB, ornB, outPos, outOrn);

		pyListOutObj = PyTuple_New(2);
		pyPosOutObj = PyTuple_New(3);
		pyOrnOutObj = PyTuple_New(4);
		for (i = 0; i < 3; i++)
			PyTuple_SetItem(pyPosOutObj, i, PyFloat_FromDouble(outPos[i]));
		for (i = 0; i < 4; i++)
			PyTuple_SetItem(pyOrnOutObj, i, PyFloat_FromDouble(outOrn[i]));

		PyTuple_SetItem(pyListOutObj, 0, pyPosOutObj);
		PyTuple_SetItem(pyListOutObj, 1, pyOrnOutObj);

		return pyListOutObj;
	}
	PyErr_SetString(SpamError, "Invalid input: expected positionA [x,y,z], orientationA [x,y,z,w], positionB, orientationB.");
	return NULL;
}

static PyObject* pybullet_invertTransform(PyObject* self,
										  PyObject* args, PyObject* keywds)
{
	PyObject* posObj = 0;
	PyObject* ornObj = 0;
	double pos[3];
	double orn[4] = {0, 0, 0, 1};
	int hasPos = 0;
	int hasOrn = 0;
	int physicsClientId = 0;

	static char* kwlist[] = {"position", "orientation", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "OO|i", kwlist, &posObj, &ornObj, &physicsClientId))
	{
		return NULL;
	}

	hasPos = pybullet_internalSetVectord(posObj, pos);
	hasOrn = pybullet_internalSetVector4d(ornObj, orn);

	if (hasPos && hasOrn)
	{
		double outPos[3];
		double outOrn[4];
		int i;
		PyObject* pyListOutObj = 0;
		PyObject* pyPosOutObj = 0;
		PyObject* pyOrnOutObj = 0;

		b3InvertTransform(pos, orn, outPos, outOrn);

		pyListOutObj = PyTuple_New(2);
		pyPosOutObj = PyTuple_New(3);
		pyOrnOutObj = PyTuple_New(4);
		for (i = 0; i < 3; i++)
			PyTuple_SetItem(pyPosOutObj, i, PyFloat_FromDouble(outPos[i]));
		for (i = 0; i < 4; i++)
			PyTuple_SetItem(pyOrnOutObj, i, PyFloat_FromDouble(outOrn[i]));

		PyTuple_SetItem(pyListOutObj, 0, pyPosOutObj);
		PyTuple_SetItem(pyListOutObj, 1, pyOrnOutObj);

		return pyListOutObj;
	}

	PyErr_SetString(SpamError, "Invalid input: expected position [x,y,z] and orientation [x,y,z,w].");
	return NULL;
}

static PyObject* pybullet_rotateVector(PyObject* self, PyObject* args, PyObject* keywds)
{
	PyObject* quatObj;
	PyObject* vectorObj;
	double quat[4];
	double vec[3];
	int physicsClientId = 0;
	int hasQuat = 0;
	int hasVec = 0;

	static char* kwlist[] = {"quaternion", "vector", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "OO|i", kwlist, &quatObj, &vectorObj, &physicsClientId))
	{
		return NULL;
	}

	if (quatObj)
	{
		hasQuat = pybullet_internalSetVector4d(quatObj, quat);
	}

	if (vectorObj)
	{
		hasVec = pybullet_internalSetVectord(vectorObj, vec);
	}
	if (hasQuat && hasVec)
	{
		double vecOut[3];
		b3RotateVector(quat, vec, vecOut);
		{
			PyObject* pylist;
			int i;
			pylist = PyTuple_New(3);
			for (i = 0; i < 3; i++)
				PyTuple_SetItem(pylist, i, PyFloat_FromDouble(vecOut[i]));
			return pylist;
		}
	}
	else
	{
		PyErr_SetString(SpamError, "Require quaternion with 4 components [x,y,z,w] and a vector [x,y,z].");
		return NULL;
	}
	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_calculateVelocityQuaternion(PyObject* self, PyObject* args, PyObject* keywds)
{
	PyObject* quatStartObj;
	PyObject* quatEndObj;
	double quatStart[4];
	double quatEnd[4];
	double deltaTime;
	int physicsClientId = 0;
	int hasQuatStart = 0;
	int hasQuatEnd = 0;

	static char* kwlist[] = {"quaternionStart", "quaternionEnd", "deltaTime", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "OOd|i", kwlist, &quatStartObj, &quatEndObj, &deltaTime, &physicsClientId))
	{
		return NULL;
	}

	if (quatStartObj)
	{
		hasQuatStart = pybullet_internalSetVector4d(quatStartObj, quatStart);
	}

	if (quatEndObj)
	{
		hasQuatEnd = pybullet_internalSetVector4d(quatEndObj, quatEnd);
	}
	if (hasQuatStart && hasQuatEnd)
	{
		double angVelOut[3];
		b3CalculateVelocityQuaternion(quatStart, quatEnd, deltaTime, angVelOut);
		{
			PyObject* pylist;
			int i;
			pylist = PyTuple_New(3);
			for (i = 0; i < 3; i++)
				PyTuple_SetItem(pylist, i, PyFloat_FromDouble(angVelOut[i]));
			return pylist;
		}
	}
	else
	{
		PyErr_SetString(SpamError, "Require start and end quaternion, each with 4 components [x,y,z,w].");
		return NULL;
	}
	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_getQuaternionSlerp(PyObject* self, PyObject* args, PyObject* keywds)
{
	PyObject* quatStartObj;
	PyObject* quatEndObj;
	double quatStart[4];
	double quatEnd[4];
	double interpolationFraction;
	int physicsClientId = 0;
	int hasQuatStart = 0;
	int hasQuatEnd = 0;

	static char* kwlist[] = {"quaternionStart", "quaternionEnd", "interpolationFraction", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "OOd|i", kwlist, &quatStartObj, &quatEndObj, &interpolationFraction, &physicsClientId))
	{
		return NULL;
	}

	if (quatStartObj)
	{
		hasQuatStart = pybullet_internalSetVector4d(quatStartObj, quatStart);
	}

	if (quatEndObj)
	{
		hasQuatEnd = pybullet_internalSetVector4d(quatEndObj, quatEnd);
	}
	if (hasQuatStart && hasQuatEnd)
	{
		double quatOut[4];
		b3QuaternionSlerp(quatStart, quatEnd, interpolationFraction, quatOut);
		{
			PyObject* pylist;
			int i;
			pylist = PyTuple_New(4);
			for (i = 0; i < 4; i++)
				PyTuple_SetItem(pylist, i, PyFloat_FromDouble(quatOut[i]));
			return pylist;
		}
	}
	else
	{
		PyErr_SetString(SpamError, "Require start and end quaternion, each with 4 components [x,y,z,w].");
		return NULL;
	}
	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_getAxisAngleFromQuaternion(PyObject* self, PyObject* args, PyObject* keywds)
{
	int physicsClientId = 0;
	PyObject* quatObj;
	double quat[4];
	int hasQuat = 0;

	static char* kwlist[] = {"quaternion", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "O|i", kwlist, &quatObj, &physicsClientId))
	{
		return NULL;
	}

	if (quatObj)
	{
		hasQuat = pybullet_internalSetVector4d(quatObj, quat);
	}

	if (hasQuat)
	{
		double axis[3];
		double angle;
		b3GetAxisAngleFromQuaternion(quat, axis, &angle);
		{
			PyObject* pylist2 = PyTuple_New(2);
			{
				PyObject* axislist;
				int i;
				axislist = PyTuple_New(3);
				for (i = 0; i < 3; i++)
					PyTuple_SetItem(axislist, i, PyFloat_FromDouble(axis[i]));
				PyTuple_SetItem(pylist2, 0, axislist);
			}
			PyTuple_SetItem(pylist2, 1, PyFloat_FromDouble(angle));

			return pylist2;
		}
	}
	else
	{
		PyErr_SetString(SpamError, "Require a quaternion with 4 components [x,y,z,w].");
		return NULL;
	}
	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_getQuaternionFromAxisAngle(PyObject* self, PyObject* args, PyObject* keywds)
{
	PyObject* axisObj;
	double axis[3];
	double angle;
	int physicsClientId = 0;
	int hasAxis = 0;

	static char* kwlist[] = {"axis", "angle", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "Od|i", kwlist, &axisObj, &angle, &physicsClientId))
	{
		return NULL;
	}

	if (axisObj)
	{
		hasAxis = pybullet_internalSetVectord(axisObj, axis);
	}

	if (hasAxis)
	{
		double quatOut[4];
		b3GetQuaternionFromAxisAngle(axis, angle, quatOut);
		{
			PyObject* pylist;
			int i;
			pylist = PyTuple_New(4);
			for (i = 0; i < 4; i++)
				PyTuple_SetItem(pylist, i, PyFloat_FromDouble(quatOut[i]));
			return pylist;
		}
	}
	else
	{
		PyErr_SetString(SpamError, "Require axis [x,y,z] and angle.");
		return NULL;
	}
	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_getAxisDifferenceQuaternion(PyObject* self, PyObject* args, PyObject* keywds)
{
	PyObject* quatStartObj;
	PyObject* quatEndObj;
	double quatStart[4];
	double quatEnd[4];
	int physicsClientId = 0;
	int hasQuatStart = 0;
	int hasQuatEnd = 0;

	static char* kwlist[] = {"quaternionStart", "quaternionEnd", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "OO|i", kwlist, &quatStartObj, &quatEndObj, &physicsClientId))
	{
		return NULL;
	}

	if (quatStartObj)
	{
		hasQuatStart = pybullet_internalSetVector4d(quatStartObj, quatStart);
	}

	if (quatEndObj)
	{
		hasQuatEnd = pybullet_internalSetVector4d(quatEndObj, quatEnd);
	}
	if (hasQuatStart && hasQuatEnd)
	{
		double axisOut[3];
		b3GetAxisDifferenceQuaternion(quatStart, quatEnd, axisOut);
		{
			PyObject* pylist;
			int i;
			pylist = PyTuple_New(3);
			for (i = 0; i < 3; i++)
				PyTuple_SetItem(pylist, i, PyFloat_FromDouble(axisOut[i]));
			return pylist;
		}
	}
	else
	{
		PyErr_SetString(SpamError, "Require start and end quaternion, each with 4 components [x,y,z,w].");
		return NULL;
	}
	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_getDifferenceQuaternion(PyObject* self, PyObject* args, PyObject* keywds)
{
	PyObject* quatStartObj;
	PyObject* quatEndObj;
	double quatStart[4];
	double quatEnd[4];
	int physicsClientId = 0;
	int hasQuatStart = 0;
	int hasQuatEnd = 0;

	static char* kwlist[] = {"quaternionStart", "quaternionEnd", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "OO|i", kwlist, &quatStartObj, &quatEndObj, &physicsClientId))
	{
		return NULL;
	}

	if (quatStartObj)
	{
		hasQuatStart = pybullet_internalSetVector4d(quatStartObj, quatStart);
	}

	if (quatEndObj)
	{
		hasQuatEnd = pybullet_internalSetVector4d(quatEndObj, quatEnd);
	}
	if (hasQuatStart && hasQuatEnd)
	{
		double quatOut[4];
		b3GetQuaternionDifference(quatStart, quatEnd, quatOut);
		{
			PyObject* pylist;
			int i;
			pylist = PyTuple_New(4);
			for (i = 0; i < 4; i++)
				PyTuple_SetItem(pylist, i, PyFloat_FromDouble(quatOut[i]));
			return pylist;
		}
	}
	else
	{
		PyErr_SetString(SpamError, "Require start and end quaternion, each with 4 components [x,y,z,w].");
		return NULL;
	}
	Py_INCREF(Py_None);
	return Py_None;
}

/// quaternion <-> euler yaw/pitch/roll convention from URDF/SDF, see Gazebo
/// https://github.com/arpg/Gazebo/blob/master/gazebo/math/Quaternion.cc
static PyObject* pybullet_getEulerFromQuaternion(PyObject* self,
												 PyObject* args, PyObject* keywds)
{
	double squ;
	double sqx;
	double sqy;
	double sqz;

	double quat[4];

	PyObject* quatObj;

	int physicsClientId = 0;

	static char* kwlist[] = {"quaternion", "physicsClientId", NULL};

	if (!PyArg_ParseTupleAndKeywords(args, keywds, "O|i", kwlist, &quatObj, &physicsClientId))
	{
		return NULL;
	}

	if (quatObj)
	{
		PyObject* seq;
		int len, i;
		seq = PySequence_Fast(quatObj, "expected a sequence");
		len = PySequence_Size(quatObj);
		if (len == 4)
		{
			for (i = 0; i < 4; i++)
			{
				quat[i] = pybullet_internalGetFloatFromSequence(seq, i);
			}
		}
		else
		{
			PyErr_SetString(SpamError, "Quaternion need a 4 components [x,y,z,w].");
			Py_DECREF(seq);
			return NULL;
		}
		Py_DECREF(seq);
	}
	else
	{
		PyErr_SetString(SpamError, "Quaternion need a 4 components [x,y,z,w].");
		return NULL;
	}

	{
		double rpy[3];
		double sarg;
		sqx = quat[0] * quat[0];
		sqy = quat[1] * quat[1];
		sqz = quat[2] * quat[2];
		squ = quat[3] * quat[3];
		sarg = -2 * (quat[0] * quat[2] - quat[3] * quat[1]);

		// If the pitch angle is PI/2 or -PI/2, we can only compute
		// the sum roll + yaw.  However, any combination that gives
		// the right sum will produce the correct orientation, so we
		// set rollX = 0 and compute yawZ.
		if (sarg <= -0.99999)
		{
			rpy[0] = 0;
			rpy[1] = -0.5 * PYBULLET_PI;
			rpy[2] = 2 * atan2(quat[0], -quat[1]);
		}
		else if (sarg >= 0.99999)
		{
			rpy[0] = 0;
			rpy[1] = 0.5 * PYBULLET_PI;
			rpy[2] = 2 * atan2(-quat[0], quat[1]);
		}
		else
		{
			rpy[0] = atan2(2 * (quat[1] * quat[2] + quat[3] * quat[0]), squ - sqx - sqy + sqz);
			rpy[1] = asin(sarg);
			rpy[2] = atan2(2 * (quat[0] * quat[1] + quat[3] * quat[2]), squ + sqx - sqy - sqz);
		}
		{
			PyObject* pylist;
			int i;
			pylist = PyTuple_New(3);
			for (i = 0; i < 3; i++)
				PyTuple_SetItem(pylist, i, PyFloat_FromDouble(rpy[i]));
			return pylist;
		}
	}
	Py_INCREF(Py_None);
	return Py_None;
}

static PyObject* pybullet_loadPlugin(PyObject* self,
									 PyObject* args, PyObject* keywds)
{
	int physicsClientId = 0;

	char* pluginPath = 0;
	char* postFix = 0;

	b3SharedMemoryCommandHandle command = 0;
	b3SharedMemoryStatusHandle statusHandle = 0;
	int statusType = -1;

	b3PhysicsClientHandle sm = 0;
	static char* kwlist[] = {"pluginPath", "postFix", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "s|si", kwlist, &pluginPath, &postFix, &physicsClientId))
	{
		return NULL;
	}

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	command = b3CreateCustomCommand(sm);
	b3CustomCommandLoadPlugin(command, pluginPath);
	if (postFix)
	{
		b3CustomCommandLoadPluginSetPostFix(command, postFix);
	}
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusPluginUniqueId(statusHandle);
	return PyInt_FromLong(statusType);
}

static PyObject* pybullet_unloadPlugin(PyObject* self,
									   PyObject* args, PyObject* keywds)
{
	int physicsClientId = 0;
	int pluginUniqueId = -1;

	b3SharedMemoryCommandHandle command = 0;
	b3SharedMemoryStatusHandle statusHandle = 0;
	int statusType = -1;

	b3PhysicsClientHandle sm = 0;
	static char* kwlist[] = {"pluginUniqueId", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|i", kwlist, &pluginUniqueId, &physicsClientId))
	{
		return NULL;
	}

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	command = b3CreateCustomCommand(sm);
	b3CustomCommandUnloadPlugin(command, pluginUniqueId);
	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);

	Py_INCREF(Py_None);
	return Py_None;
	;
}

//createCustomCommand for executing commands implemented in a plugin system
static PyObject* pybullet_executePluginCommand(PyObject* self,
											   PyObject* args, PyObject* keywds)
{
	int physicsClientId = 0;
	int pluginUniqueId = -1;
	char* textArgument = 0;
	b3SharedMemoryCommandHandle command = 0;
	b3SharedMemoryStatusHandle statusHandle = 0;
	int statusType = -1;
	int statusResult = -1;
	PyObject* intArgs = 0;
	PyObject* floatArgs = 0;

	b3PhysicsClientHandle sm = 0;
	static char* kwlist[] = {"pluginUniqueId", "textArgument", "intArgs", "floatArgs", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "i|sOOi", kwlist, &pluginUniqueId, &textArgument, &intArgs, &floatArgs, &physicsClientId))
	{
		return NULL;
	}

	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}

	command = b3CreateCustomCommand(sm);
	b3CustomCommandExecutePluginCommand(command, pluginUniqueId, textArgument);

	{
		PyObject* seqIntArgs = intArgs ? PySequence_Fast(intArgs, "expected a sequence") : 0;
		PyObject* seqFloatArgs = floatArgs ? PySequence_Fast(floatArgs, "expected a sequence") : 0;
		int numIntArgs = seqIntArgs ? PySequence_Size(intArgs) : 0;
		int numFloatArgs = seqFloatArgs ? PySequence_Size(floatArgs) : 0;
		int i;
		for (i = 0; i < numIntArgs; i++)
		{
			int val = pybullet_internalGetIntFromSequence(seqIntArgs, i);
			b3CustomCommandExecuteAddIntArgument(command, val);
		}

		for (i = 0; i < numFloatArgs; i++)
		{
			float val = pybullet_internalGetFloatFromSequence(seqFloatArgs, i);
			b3CustomCommandExecuteAddFloatArgument(command, val);
		}
		if (seqFloatArgs)
		{
			Py_DECREF(seqFloatArgs);
		}
		if (seqIntArgs)
		{
			Py_DECREF(seqIntArgs);
		}
	}

	statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);
	statusType = b3GetStatusType(statusHandle);
	if (statusType == CMD_CUSTOM_COMMAND_COMPLETED)
	{
		statusResult = b3GetStatusPluginCommandResult(statusHandle);
		struct b3UserDataValue dv;
		if (b3GetStatusPluginCommandReturnData(sm, &dv))
		{
			assert(dv.m_length>0);
			PyObject* pylist;
			PyObject* pydata;
			int i;
			//return type
			//user data type
			//bytes
			pylist = PyTuple_New(3);
			PyTuple_SetItem(pylist, 0, PyInt_FromLong(statusResult));
			PyTuple_SetItem(pylist, 1, PyInt_FromLong(dv.m_type));
			pydata = PyTuple_New(dv.m_length);
			for (i = 0; i < dv.m_length; i++)
			{
				PyTuple_SetItem(pydata, i, PyInt_FromLong(dv.m_data1[i]));
			}
			PyTuple_SetItem(pylist, 2, pydata);
			return pylist;
		}
		return PyInt_FromLong(statusResult);
	}
	return PyInt_FromLong(-1);
}



///Inverse Kinematics binding
static PyObject* pybullet_calculateInverseKinematics(PyObject* self,
													 PyObject* args, PyObject* keywds)

{
	int bodyUniqueId;
	int endEffectorLinkIndex;

	PyObject* targetPosObj = 0;
	PyObject* targetOrnObj = 0;

	int solver = 0;  // the default IK solver is DLS
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	PyObject* lowerLimitsObj = 0;
	PyObject* upperLimitsObj = 0;
	PyObject* jointRangesObj = 0;
	PyObject* restPosesObj = 0;
	PyObject* jointDampingObj = 0;
	PyObject* currentPositionsObj = 0;
	int maxNumIterations = -1;
	double residualThreshold = -1;

	static char* kwlist[] = {"bodyUniqueId", "endEffectorLinkIndex", "targetPosition", "targetOrientation", "lowerLimits", "upperLimits", "jointRanges", "restPoses", "jointDamping", "solver", "currentPositions", "maxNumIterations", "residualThreshold", "physicsClientId", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "iiO|OOOOOOiOidi", kwlist, &bodyUniqueId, &endEffectorLinkIndex, &targetPosObj, &targetOrnObj, &lowerLimitsObj, &upperLimitsObj, &jointRangesObj, &restPosesObj, &jointDampingObj, &solver, &currentPositionsObj, &maxNumIterations, &residualThreshold, &physicsClientId))
	{
		//backward compatibility bodyIndex -> bodyUniqueId. don't update keywords, people need to migrate to bodyUniqueId version
		static char* kwlist2[] = {"bodyIndex", "endEffectorLinkIndex", "targetPosition", "targetOrientation", "lowerLimits", "upperLimits", "jointRanges", "restPoses", "jointDamping", "physicsClientId", NULL};
		PyErr_Clear();
		if (!PyArg_ParseTupleAndKeywords(args, keywds, "iiO|OOOOOOi", kwlist2, &bodyUniqueId, &endEffectorLinkIndex, &targetPosObj, &targetOrnObj, &lowerLimitsObj, &upperLimitsObj, &jointRangesObj, &restPosesObj, &jointDampingObj, &physicsClientId))
		{
			return NULL;
		}
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
	{
		double pos[3];
		double ori[4] = {0, 0, 0, 1};
		int hasPos = pybullet_internalSetVectord(targetPosObj, pos);
		int hasOrn = pybullet_internalSetVector4d(targetOrnObj, ori);

		int szLowerLimits = lowerLimitsObj ? PySequence_Size(lowerLimitsObj) : 0;
		int szUpperLimits = upperLimitsObj ? PySequence_Size(upperLimitsObj) : 0;
		int szJointRanges = jointRangesObj ? PySequence_Size(jointRangesObj) : 0;
		int szRestPoses = restPosesObj ? PySequence_Size(restPosesObj) : 0;
		int szJointDamping = jointDampingObj ? PySequence_Size(jointDampingObj) : 0;

		int szCurrentPositions = currentPositionsObj ? PySequence_Size(currentPositionsObj) : 0;

		int numJoints = b3GetNumJoints(sm, bodyUniqueId);
		int dofCount = b3ComputeDofCount(sm, bodyUniqueId);

		int hasNullSpace = 0;
		int hasJointDamping = 0;
		int hasCurrentPositions = 0;
		double* lowerLimits = 0;
		double* upperLimits = 0;
		double* jointRanges = 0;
		double* restPoses = 0;
		double* jointDamping = 0;
		double* currentPositions = 0;

		if (dofCount && (szLowerLimits == dofCount) && (szUpperLimits == dofCount) &&
			(szJointRanges == dofCount) && (szRestPoses == dofCount))
		{
			int szInBytes = sizeof(double) * dofCount;
			int i;
			lowerLimits = (double*)malloc(szInBytes);
			upperLimits = (double*)malloc(szInBytes);
			jointRanges = (double*)malloc(szInBytes);
			restPoses = (double*)malloc(szInBytes);

			for (i = 0; i < dofCount; i++)
			{
				lowerLimits[i] = pybullet_internalGetFloatFromSequence(lowerLimitsObj, i);
				upperLimits[i] = pybullet_internalGetFloatFromSequence(upperLimitsObj, i);
				jointRanges[i] = pybullet_internalGetFloatFromSequence(jointRangesObj, i);
				restPoses[i] = pybullet_internalGetFloatFromSequence(restPosesObj, i);
			}
			hasNullSpace = 1;
		}

		if (szCurrentPositions > 0)
		{
			if (szCurrentPositions != dofCount)
			{
				PyErr_SetString(SpamError,
								"calculateInverseKinematics the size of input current positions needs to be equal to the number of degrees of freedom.");
				free(lowerLimits); 
                        	free(upperLimits); 
                        	free(jointRanges); 
                        	free(restPoses); 
				return NULL;
			}
			else
			{
				int szInBytes = sizeof(double) * szCurrentPositions;
				int i;
				currentPositions = (double*)malloc(szInBytes);
				for (i = 0; i < szCurrentPositions; i++)
				{
					currentPositions[i] = pybullet_internalGetFloatFromSequence(currentPositionsObj, i);
				}
				hasCurrentPositions = 1;
			}
		}

		if (szJointDamping > 0)
		{
			if (szJointDamping < dofCount)
			{
				printf("calculateInverseKinematics: the size of input joint damping values should be equal to the number of degrees of freedom, not using joint damping.");
			}
			else
			{
				int szInBytes = sizeof(double) * szJointDamping;
				int i;
				//if (szJointDamping != dofCount)
				//{
				//	printf("calculateInverseKinematics: the size of input joint damping values should be equal to the number of degrees of freedom, ignoring the additonal values.");
				//}
				jointDamping = (double*)malloc(szInBytes);
				for (i = 0; i < szJointDamping; i++)
				{
					jointDamping[i] = pybullet_internalGetFloatFromSequence(jointDampingObj, i);
				}
				hasJointDamping = 1;
			}
		}

		if (hasPos)
		{
			b3SharedMemoryStatusHandle statusHandle;
			int numPos = 0;
			int resultBodyIndex;
			int result;

			b3SharedMemoryCommandHandle command = b3CalculateInverseKinematicsCommandInit(sm, bodyUniqueId);
			b3CalculateInverseKinematicsSelectSolver(command, solver);

			if (hasCurrentPositions)
			{
				b3CalculateInverseKinematicsSetCurrentPositions(command, dofCount, currentPositions);
			}
			if (maxNumIterations > 0)
			{
				b3CalculateInverseKinematicsSetMaxNumIterations(command, maxNumIterations);
			}
			if (residualThreshold >= 0)
			{
				b3CalculateInverseKinematicsSetResidualThreshold(command, residualThreshold);
			}

			if (hasNullSpace)
			{
				if (hasOrn)
				{
					b3CalculateInverseKinematicsPosOrnWithNullSpaceVel(command, dofCount, endEffectorLinkIndex, pos, ori, lowerLimits, upperLimits, jointRanges, restPoses);
				}
				else
				{
					b3CalculateInverseKinematicsPosWithNullSpaceVel(command, dofCount, endEffectorLinkIndex, pos, lowerLimits, upperLimits, jointRanges, restPoses);
				}
			}
			else
			{
				if (hasOrn)
				{
					b3CalculateInverseKinematicsAddTargetPositionWithOrientation(command, endEffectorLinkIndex, pos, ori);
				}
				else
				{
					b3CalculateInverseKinematicsAddTargetPurePosition(command, endEffectorLinkIndex, pos);
				}
			}

			if (hasJointDamping)
			{
				b3CalculateInverseKinematicsSetJointDamping(command, dofCount, jointDamping);
			}
			free(currentPositions);
			free(jointDamping);

			free(lowerLimits);
                        free(upperLimits);
                        free(jointRanges);
                        free(restPoses);

			statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);

			result = b3GetStatusInverseKinematicsJointPositions(statusHandle,
																&resultBodyIndex,
																&numPos,
																0);
			if (result && numPos)
			{
				int i;
				PyObject* pylist;
				double* ikOutPutJointPos = (double*)malloc(numPos * sizeof(double));
				result = b3GetStatusInverseKinematicsJointPositions(statusHandle,
																	&resultBodyIndex,
																	&numPos,
																	ikOutPutJointPos);
				pylist = PyTuple_New(numPos);
				for (i = 0; i < numPos; i++)
				{
					PyTuple_SetItem(pylist, i,
									PyFloat_FromDouble(ikOutPutJointPos[i]));
				}

				free(ikOutPutJointPos);
				return pylist;
			}
			else
			{
				PyErr_SetString(SpamError,
								"Error in calculateInverseKinematics");
				return NULL;
			}
		}
		else
		{
			PyErr_SetString(SpamError,
							"calculateInverseKinematics couldn't extract position vector3");
			return NULL;
		}
	}

	Py_INCREF(Py_None);
	return Py_None;
}


///Inverse Kinematics binding
static PyObject* pybullet_calculateInverseKinematics2(PyObject* self,
	PyObject* args, PyObject* keywds)

{
	int bodyUniqueId;
	int endEffectorLinkIndex=-1;

	PyObject* targetPosObj = 0;
	//PyObject* targetOrnObj = 0;

	int solver = 0;  // the default IK solver is DLS
	int physicsClientId = 0;
	b3PhysicsClientHandle sm = 0;
	PyObject* endEffectorLinkIndicesObj = 0;
	PyObject* lowerLimitsObj = 0;
	PyObject* upperLimitsObj = 0;
	PyObject* jointRangesObj = 0;
	PyObject* restPosesObj = 0;
	PyObject* jointDampingObj = 0;
	PyObject* currentPositionsObj = 0;
	int maxNumIterations = -1;
	double residualThreshold = -1;

	static char* kwlist[] = { "bodyUniqueId", "endEffectorLinkIndices", "targetPositions",  "lowerLimits", "upperLimits", "jointRanges", "restPoses", "jointDamping", "solver", "currentPositions", "maxNumIterations", "residualThreshold", "physicsClientId", NULL };
	if (!PyArg_ParseTupleAndKeywords(args, keywds, "iOO|OOOOOiOidi", kwlist, &bodyUniqueId, &endEffectorLinkIndicesObj, &targetPosObj, &lowerLimitsObj, &upperLimitsObj, &jointRangesObj, &restPosesObj, &jointDampingObj, &solver, &currentPositionsObj, &maxNumIterations, &residualThreshold, &physicsClientId))
	{
			return NULL;
	}
	sm = getPhysicsClient(physicsClientId);
	if (sm == 0)
	{
		PyErr_SetString(SpamError, "Not connected to physics server.");
		return NULL;
	}
	{
		int numEndEffectorPositions = extractVertices(targetPosObj, 0, B3_MAX_NUM_END_EFFECTORS);
		    
		int numIndices = extractIndices(endEffectorLinkIndicesObj, 0, B3_MAX_NUM_END_EFFECTORS);
		double* positions = numEndEffectorPositions ? malloc(numEndEffectorPositions * 3 * sizeof(double)) : 0;
		int* indices = numIndices ? malloc(numIndices * sizeof(int)) : 0;


		numEndEffectorPositions = extractVertices(targetPosObj, positions, B3_MAX_NUM_VERTICES);
		
		if (endEffectorLinkIndicesObj)
		{
			numIndices = extractIndices(endEffectorLinkIndicesObj, indices, B3_MAX_NUM_INDICES);
		}
		
		{
			double pos[3] = { 0, 0, 0 };
			double ori[4] = { 0, 0, 0, 1 };
			int hasPos = numEndEffectorPositions > 0;
			int hasOrn = 0;// pybullet_internalSetVector4d(targetOrnObj, ori);

			int szLowerLimits = lowerLimitsObj ? PySequence_Size(lowerLimitsObj) : 0;
			int szUpperLimits = upperLimitsObj ? PySequence_Size(upperLimitsObj) : 0;
			int szJointRanges = jointRangesObj ? PySequence_Size(jointRangesObj) : 0;
			int szRestPoses = restPosesObj ? PySequence_Size(restPosesObj) : 0;
			int szJointDamping = jointDampingObj ? PySequence_Size(jointDampingObj) : 0;

			int szCurrentPositions = currentPositionsObj ? PySequence_Size(currentPositionsObj) : 0;

			int numJoints = b3GetNumJoints(sm, bodyUniqueId);
			int dofCount = b3ComputeDofCount(sm, bodyUniqueId);

			int hasNullSpace = 0;
			int hasJointDamping = 0;
			int hasCurrentPositions = 0;
			double* lowerLimits = 0;
			double* upperLimits = 0;
			double* jointRanges = 0;
			double* restPoses = 0;
			double* jointDamping = 0;
			double* currentPositions = 0;

			if (dofCount && (szLowerLimits == dofCount) && (szUpperLimits == dofCount) &&
				(szJointRanges == dofCount) && (szRestPoses == dofCount))
			{
				int szInBytes = sizeof(double) * dofCount;
				int i;
				lowerLimits = (double*)malloc(szInBytes);
				upperLimits = (double*)malloc(szInBytes);
				jointRanges = (double*)malloc(szInBytes);
				restPoses = (double*)malloc(szInBytes);

				for (i = 0; i < dofCount; i++)
				{
					lowerLimits[i] = pybullet_internalGetFloatFromSequence(lowerLimitsObj, i);
					upperLimits[i] = pybullet_internalGetFloatFromSequence(upperLimitsObj, i);
					jointRanges[i] = pybullet_internalGetFloatFromSequence(jointRangesObj, i);
					restPoses[i] = pybullet_internalGetFloatFromSequence(restPosesObj, i);
				}
				hasNullSpace = 1;
			}

			if (szCurrentPositions > 0)
			{
				if (szCurrentPositions != dofCount)
				{
					PyErr_SetString(SpamError,
						"calculateInverseKinematics the size of input current positions needs to be equal to the number of degrees of freedom.");
					free(lowerLimits);
					free(upperLimits);
					free(jointRanges);
					free(restPoses);
					free(positions);
					free(indices);
					return NULL;
				}
				else
				{
					int szInBytes = sizeof(double) * szCurrentPositions;
					int i;
					currentPositions = (double*)malloc(szInBytes);
					for (i = 0; i < szCurrentPositions; i++)
					{
						currentPositions[i] = pybullet_internalGetFloatFromSequence(currentPositionsObj, i);
					}
					hasCurrentPositions = 1;
				}
			}

			if (szJointDamping > 0)
			{
				if (szJointDamping < dofCount)
				{
					printf("calculateInverseKinematics: the size of input joint damping values should be equal to the number of degrees of freedom, not using joint damping.");
				}
				else
				{
					int szInBytes = sizeof(double) * szJointDamping;
					int i;
					//if (szJointDamping != dofCount)
					//{
					//	printf("calculateInverseKinematics: the size of input joint damping values should be equal to the number of degrees of freedom, ignoring the additonal values.");
					//}
					jointDamping = (double*)malloc(szInBytes);
					for (i = 0; i < szJointDamping; i++)
					{
						jointDamping[i] = pybullet_internalGetFloatFromSequence(jointDampingObj, i);
					}
					hasJointDamping = 1;
				}
			}

			if (hasPos)
			{
				b3SharedMemoryStatusHandle statusHandle;
				int numPos = 0;
				int resultBodyIndex;
				int result;

				b3SharedMemoryCommandHandle command = b3CalculateInverseKinematicsCommandInit(sm, bodyUniqueId);
				b3CalculateInverseKinematicsSelectSolver(command, solver);

				if (hasCurrentPositions)
				{
					b3CalculateInverseKinematicsSetCurrentPositions(command, dofCount, currentPositions);
				}
				if (maxNumIterations > 0)
				{
					b3CalculateInverseKinematicsSetMaxNumIterations(command, maxNumIterations);
				}
				if (residualThreshold >= 0)
				{
					b3CalculateInverseKinematicsSetResidualThreshold(command, residualThreshold);
				}

				if (hasNullSpace)
				{
					if (hasOrn)
					{
						b3CalculateInverseKinematicsPosOrnWithNullSpaceVel(command, dofCount, endEffectorLinkIndex, pos, ori, lowerLimits, upperLimits, jointRanges, restPoses);
					}
					else
					{
						b3CalculateInverseKinematicsPosWithNullSpaceVel(command, dofCount, endEffectorLinkIndex, pos, lowerLimits, upperLimits, jointRanges, restPoses);
					}
				}
				else
				{
					if (hasOrn)
					{
						b3CalculateInverseKinematicsAddTargetPositionWithOrientation(command, endEffectorLinkIndex, pos, ori);
					}
					else
					{
						//b3CalculateInverseKinematicsAddTargetPurePosition(command, endEffectorLinkIndex, pos);
						b3CalculateInverseKinematicsAddTargetsPurePosition(command, numEndEffectorPositions, indices, positions);
					}
				}

				if (hasJointDamping)
				{
					b3CalculateInverseKinematicsSetJointDamping(command, dofCount, jointDamping);
				}
				free(currentPositions);
				free(jointDamping);

				free(lowerLimits);
				free(upperLimits);
				free(jointRanges);
				free(restPoses);

				statusHandle = b3SubmitClientCommandAndWaitStatus(sm, command);

				result = b3GetStatusInverseKinematicsJointPositions(statusHandle,
					&resultBodyIndex,
					&numPos,
					0);
				if (result && numPos)
				{
					int i;
					PyObject* pylist;
					double* ikOutPutJointPos = (double*)malloc(numPos * sizeof(double));
					result = b3GetStatusInverseKinematicsJointPositions(statusHandle,
						&resultBodyIndex,
						&numPos,
						ikOutPutJointPos);
					pylist = PyTuple_New(numPos);
					for (i = 0; i < numPos; i++)
					{
						PyTuple_SetItem(pylist, i,
							PyFloat_FromDouble(ikOutPutJointPos[i]));
					}

					free(ikOutPutJointPos);
					free(positions);
					free(indices);
					return pylist;
				}
				else
				{
					PyErr_SetString(SpamError,
						"Error in calculateInverseKinematics");
					free(positions);
					free(indices);
					return NULL;
				}
			}
			else
			{
				PyErr_SetString(SpamError,
					"calculateInverseKinematics couldn't extract position vector3");
				free(positions);
				free(indices);
				return NULL;
			}

			free(positions);
			free(indices);
		}
	}

	Py_INCREF(Py_None);
	return Py_None;
}


/// Given an object id, joint positions, joint velocities and joint
/// accelerations,
/// compute the joint forces using Inverse Dynamics
static PyObject* pybullet_calculateInverseDynamics(PyObject* self, PyObject* args, PyObject* keywds)
{
	{
		int bodyUniqueId;
		PyObject* objPositionsQ;
		PyObject* objVelocitiesQdot;
		PyObject* objAccelerations;
		int physicsClientId = 0;
		int flags = 0;
		b3PhysicsClientHandle sm = 0;
		static char* kwlist[] = {"bodyUniqueId", "objPositions",
								 "objVelocities", "objAccelerations",
								 "flags",
								 "physicsClientId", NULL};
		if (!PyArg_ParseTupleAndKeywords(args, keywds, "iOOO|ii", kwlist,
										 &bodyUniqueId, &objPositionsQ,
										 &objVelocitiesQdot, &objAccelerations,
										 &flags,
										 &physicsClientId))
		{
			static char* kwlist2[] = {"bodyIndex", "objPositions",
									  "objVelocities", "objAccelerations",
									  "physicsClientId", NULL};
			PyErr_Clear();
			if (!PyArg_ParseTupleAndKeywords(args, keywds, "iOOO|i", kwlist2,
											 &bodyUniqueId, &objPositionsQ, &objVelocitiesQdot,
											 &objAccelerations, &physicsClientId))
			{
				return NULL;
			}
		}
		sm = getPhysicsClient(physicsClientId);
		if (sm == 0)
		{
			PyErr_SetString(SpamError, "Not connected to physics server.");
			return NULL;
		}

		{
			int szObPos = PySequence_Size(objPositionsQ);
			int szObVel = PySequence_Size(objVelocitiesQdot);
			int szObAcc = PySequence_Size(objAccelerations);

			if (szObVel == szObAcc)
			{
				int szInBytesQ = sizeof(double) * szObPos;
				int szInBytesQdot = sizeof(double) * szObVel;
				int i;
				PyObject* pylist = 0;
				double* jointPositionsQ = (double*)malloc(szInBytesQ);
				double* jointVelocitiesQdot = (double*)malloc(szInBytesQdot);
				double* jointAccelerations = (double*)malloc(szInBytesQdot);

				for (i = 0; i < szObPos; i++)
				{
					jointPositionsQ[i] =
						pybullet_internalGetFloatFromSequence(objPositionsQ, i);
				}
				for (i = 0; i < szObVel; i++)
				{
					jointVelocitiesQdot[i] =
						pybullet_internalGetFloatFromSequence(objVelocitiesQdot, i);
					jointAccelerations[i] =
						pybullet_internalGetFloatFromSequence(objAccelerations, i);
				}

				{
					b3SharedMemoryStatusHandle statusHandle;
					int statusType;

					b3SharedMemoryCommandHandle commandHandle =
						b3CalculateInverseDynamicsCommandInit2(
							sm, bodyUniqueId, jointPositionsQ, szObPos, jointVelocitiesQdot,
							jointAccelerations, szObVel);
					b3CalculateInverseDynamicsSetFlags(commandHandle, flags);
					statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);

					statusType = b3GetStatusType(statusHandle);

					if (statusType == CMD_CALCULATED_INVERSE_DYNAMICS_COMPLETED)
					{
						int bodyUniqueId;
						int dofCount;
						b3GetStatusInverseDynamicsJointForces(statusHandle, &bodyUniqueId, &dofCount, 0);

						if (dofCount)
						{
							double* jointForcesOutput = (double*)malloc(sizeof(double) * dofCount);

							b3GetStatusInverseDynamicsJointForces(statusHandle, 0, 0, jointForcesOutput);
							{
								{
									int i;
									pylist = PyTuple_New(dofCount);
									for (i = 0; i < dofCount; i++)
										PyTuple_SetItem(pylist, i,
														PyFloat_FromDouble(jointForcesOutput[i]));
								}
							}
							free(jointForcesOutput);
						}
					}
					else
					{
						PyErr_SetString(SpamError,
										"Error in calculateInverseDynamics, please check arguments.");
					}
				}
				free(jointPositionsQ);
				free(jointVelocitiesQdot);
				free(jointAccelerations);

				if (pylist) return pylist;
			}
			else
			{
				PyErr_SetString(SpamError,
								"calculateInverseDynamics numDofs needs to be "
								"positive and [joint velocities] and"
								"[joint accelerations] need to be equal and match the number of "
								"degrees of freedom.");
				return NULL;
			}
		}
	}
	Py_INCREF(Py_None);
	return Py_None;
}

/// Given an object id, joint positions, joint velocities and joint
/// accelerations, compute the Jacobian
static PyObject* pybullet_calculateJacobian(PyObject* self, PyObject* args, PyObject* keywds)
{
	{
		int bodyUniqueId;
		int linkIndex;
		PyObject* localPosition;
		PyObject* objPositions;
		PyObject* objVelocities;
		PyObject* objAccelerations;
		int physicsClientId = 0;
		b3PhysicsClientHandle sm = 0;
		static char* kwlist[] = {"bodyUniqueId", "linkIndex", "localPosition",
								 "objPositions", "objVelocities",
								 "objAccelerations", "physicsClientId", NULL};
		if (!PyArg_ParseTupleAndKeywords(args, keywds, "iiOOOO|i", kwlist,
										 &bodyUniqueId, &linkIndex, &localPosition, &objPositions,
										 &objVelocities, &objAccelerations, &physicsClientId))
		{
			return NULL;
		}
		sm = getPhysicsClient(physicsClientId);
		if (sm == 0)
		{
			PyErr_SetString(SpamError, "Not connected to physics server.");
			return NULL;
		}

		{
			int szLoPos = PySequence_Size(localPosition);
			int szObPos = PySequence_Size(objPositions);
			int szObVel = PySequence_Size(objVelocities);
			int szObAcc = PySequence_Size(objAccelerations);
			int numJoints = b3GetNumJoints(sm, bodyUniqueId);

			int j = 0;
			int dofCountOrg = 0;
			for (j = 0; j < numJoints; j++)
			{
				struct b3JointInfo info;
				b3GetJointInfo(sm, bodyUniqueId, j, &info);
				switch (info.m_jointType)
				{
					case eRevoluteType:
					{
						dofCountOrg += 1;
						break;
					}
					case ePrismaticType:
					{
						dofCountOrg += 1;
						break;
					}
					case eSphericalType:
					{
						PyErr_SetString(SpamError,
										"Spherirical joints are not supported in the pybullet binding");
						return NULL;
					}
					case ePlanarType:
					{
						PyErr_SetString(SpamError,
										"Planar joints are not supported in the pybullet binding");
						return NULL;
					}
					default:
					{
						//fixed joint has 0-dof and at the moment, we don't deal with planar, spherical etc
					}
				}
			}

			if (dofCountOrg && (szLoPos == 3) && (szObPos == dofCountOrg) &&
				(szObVel == dofCountOrg) && (szObAcc == dofCountOrg))
			{
				int byteSizeJoints = sizeof(double) * dofCountOrg;
				int byteSizeVec3 = sizeof(double) * 3;
				int i;
				PyObject* pyResultList = PyTuple_New(2);
				double* localPoint = (double*)malloc(byteSizeVec3);
				double* jointPositions = (double*)malloc(byteSizeJoints);
				double* jointVelocities = (double*)malloc(byteSizeJoints);
				double* jointAccelerations = (double*)malloc(byteSizeJoints);
				double* linearJacobian = NULL;
				double* angularJacobian = NULL;

				pybullet_internalSetVectord(localPosition, localPoint);
				for (i = 0; i < dofCountOrg; i++)
				{
					jointPositions[i] =
						pybullet_internalGetFloatFromSequence(objPositions, i);
					jointVelocities[i] =
						pybullet_internalGetFloatFromSequence(objVelocities, i);
					jointAccelerations[i] =
						pybullet_internalGetFloatFromSequence(objAccelerations, i);
				}
				{
					b3SharedMemoryStatusHandle statusHandle;
					int statusType;
					b3SharedMemoryCommandHandle commandHandle =
						b3CalculateJacobianCommandInit(sm, bodyUniqueId,
													   linkIndex, localPoint, jointPositions,
													   jointVelocities, jointAccelerations);
					statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
					statusType = b3GetStatusType(statusHandle);
					if (statusType == CMD_CALCULATED_JACOBIAN_COMPLETED)
					{
						int dofCount;
						b3GetStatusJacobian(statusHandle, &dofCount, NULL, NULL);
						if (dofCount)
						{
							int byteSizeDofCount = sizeof(double) * dofCount;
							linearJacobian = (double*)malloc(3 * byteSizeDofCount);
							angularJacobian = (double*)malloc(3 * byteSizeDofCount);
							b3GetStatusJacobian(statusHandle,
												NULL,
												linearJacobian,
												angularJacobian);
							if (linearJacobian)
							{
								int r;
								PyObject* pymat = PyTuple_New(3);
								for (r = 0; r < 3; ++r)
								{
									int c;
									PyObject* pyrow = PyTuple_New(dofCount);
									for (c = 0; c < dofCount; ++c)
									{
										int element = r * dofCount + c;
										PyTuple_SetItem(pyrow, c,
														PyFloat_FromDouble(linearJacobian[element]));
									}
									PyTuple_SetItem(pymat, r, pyrow);
								}
								PyTuple_SetItem(pyResultList, 0, pymat);
							}
							if (angularJacobian)
							{
								int r;
								PyObject* pymat = PyTuple_New(3);
								for (r = 0; r < 3; ++r)
								{
									int c;
									PyObject* pyrow = PyTuple_New(dofCount);
									for (c = 0; c < dofCount; ++c)
									{
										int element = r * dofCount + c;
										PyTuple_SetItem(pyrow, c,
														PyFloat_FromDouble(angularJacobian[element]));
									}
									PyTuple_SetItem(pymat, r, pyrow);
								}
								PyTuple_SetItem(pyResultList, 1, pymat);
							}
						}
					}
					else
					{
						PyErr_SetString(SpamError,
										"Internal error in calculateJacobian");
					}
				}
				free(localPoint);
				free(jointPositions);
				free(jointVelocities);
				free(jointAccelerations);
				free(linearJacobian);
				free(angularJacobian);
				if (pyResultList) return pyResultList;
			}
			else
			{
				PyErr_SetString(SpamError,
								"calculateJacobian [numDof] needs to be "
								"positive, [local position] needs to be of "
								"size 3 and [joint positions], "
								"[joint velocities], [joint accelerations] "
								"need to match the number of DoF.");
				return NULL;
			}
		}
	}
	Py_INCREF(Py_None);
	return Py_None;
}

/// Given an object id, joint positions, joint velocities and joint
/// accelerations, compute the Jacobian
static PyObject* pybullet_calculateMassMatrix(PyObject* self, PyObject* args, PyObject* keywds)
{
	{
		int bodyUniqueId;
		PyObject* objPositions;
		int physicsClientId = 0;
		b3PhysicsClientHandle sm = 0;
		int flags = 0;
		static char* kwlist[] = {"bodyUniqueId", "objPositions", "flags", "physicsClientId", NULL};
		if (!PyArg_ParseTupleAndKeywords(args, keywds, "iO|ii", kwlist,
										 &bodyUniqueId, &objPositions, &flags, &physicsClientId))
		{
			return NULL;
		}
		sm = getPhysicsClient(physicsClientId);
		if (sm == 0)
		{
			PyErr_SetString(SpamError, "Not connected to physics server.");
			return NULL;
		}

		{
			int szObPos = PySequence_Size(objPositions);
			///int dofCountQ = b3GetNumJoints(sm, bodyUniqueId);

			if (szObPos >= 0)  //(szObPos == dofCountQ))
			{
				int byteSizeJoints = sizeof(double) * szObPos;
				PyObject* pyResultList = NULL;
				double* jointPositions = (double*)malloc(byteSizeJoints);
				double* massMatrix = NULL;
				int i;
				for (i = 0; i < szObPos; i++)
				{
					jointPositions[i] =
						pybullet_internalGetFloatFromSequence(objPositions, i);
				}
				{
					b3SharedMemoryStatusHandle statusHandle;
					int statusType;
					b3SharedMemoryCommandHandle commandHandle =
						b3CalculateMassMatrixCommandInit(sm, bodyUniqueId, jointPositions, szObPos);
					b3CalculateMassMatrixSetFlags(commandHandle, flags);
					statusHandle = b3SubmitClientCommandAndWaitStatus(sm, commandHandle);
					statusType = b3GetStatusType(statusHandle);
					if (statusType == CMD_CALCULATED_MASS_MATRIX_COMPLETED)
					{
						int dofCount;
						b3GetStatusMassMatrix(sm, statusHandle, &dofCount, NULL);
						pyResultList = PyTuple_New(dofCount);
						if (dofCount)
						{
							int byteSizeDofCount = sizeof(double) * dofCount;

							massMatrix = (double*)malloc(dofCount * byteSizeDofCount);
							b3GetStatusMassMatrix(sm, statusHandle, NULL, massMatrix);
							if (massMatrix)
							{
								int r;
								for (r = 0; r < dofCount; ++r)
								{
									int c;
									PyObject* pyrow = PyTuple_New(dofCount);
									for (c = 0; c < dofCount; ++c)
									{
										int element = r * dofCount + c;
										PyTuple_SetItem(pyrow, c,
														PyFloat_FromDouble(massMatrix[element]));
									}
									PyTuple_SetItem(pyResultList, r, pyrow);
								}
							}
						}
					}
					else
					{
						PyErr_SetString(SpamError,
										"Internal error in calculateMassMatrix");
					}
				}
				free(jointPositions);
				free(massMatrix);
				if (pyResultList) return pyResultList;
			}
			else
			{
				PyErr_SetString(SpamError,
								"calculateMassMatrix [numJoints] needs to be "
								"positive and [joint positions] "
								"need to match the number of joints.");
				return NULL;
			}
		}
	}
	Py_INCREF(Py_None);
	return Py_None;
}



static PyMethodDef SpamMethods[] = {

	{"connect", (PyCFunction)pybullet_connectPhysicsServer, METH_VARARGS | METH_KEYWORDS,
	 "connect(method, key=SHARED_MEMORY_KEY, options='')\n"
	 "connect(method, hostname='localhost', port=1234, options='')\n"
	 "Connect to an existing physics server (using shared memory by default)."},

	{"disconnect", (PyCFunction)pybullet_disconnectPhysicsServer, METH_VARARGS | METH_KEYWORDS,
	 "disconnect(physicsClientId=0)\n"
	 "Disconnect from the physics server."},

	{"getConnectionInfo", (PyCFunction)pybullet_getConnectionInfo, METH_VARARGS | METH_KEYWORDS,
	 "getConnectionInfo(physicsClientId=0)\n"
	 "Return if a given client id is connected, and using what method."},

	{"isConnected", (PyCFunction)pybullet_isConnected, METH_VARARGS | METH_KEYWORDS,
	 "isConnected(physicsClientId=0)\n"
	 "Return if a given client id is connected."},

	{"resetSimulation", (PyCFunction)pybullet_resetSimulation, METH_VARARGS | METH_KEYWORDS,
	 "resetSimulation(physicsClientId=0)\n"
	 "Reset the simulation: remove all objects and start from an empty world."},
	
	{"stepSimulation", (PyCFunction)pybullet_stepSimulation, METH_VARARGS | METH_KEYWORDS,
	 "stepSimulation(physicsClientId=0)\n"
	 "Step the simulation using forward dynamics."},

	{"setGravity", (PyCFunction)pybullet_setGravity, METH_VARARGS | METH_KEYWORDS,
	 "setGravity(gravX, gravY, gravZ, physicsClientId=0)\n"
	 "Set the gravity acceleration (x,y,z)."},

	{"setTimeStep", (PyCFunction)pybullet_setTimeStep, METH_VARARGS | METH_KEYWORDS,
	 "setTimeStep(timestep, physicsClientId=0)\n"
	 "Set the amount of time to proceed at each call to stepSimulation. (unit "
	 "is seconds, typically range is 0.01 or 0.001)"},

	{"setDefaultContactERP", (PyCFunction)pybullet_setDefaultContactERP, METH_VARARGS | METH_KEYWORDS,
	 "setDefaultContactERP(defaultContactERP, physicsClientId=0)\n"
	 "Set the amount of contact penetration Error Recovery Paramater "
	 "(ERP) in each time step. \
		This is an tuning parameter to control resting contact stability. "
	 "This value depends on the time step."},

	{"setRealTimeSimulation", (PyCFunction)pybullet_setRealTimeSimulation, METH_VARARGS | METH_KEYWORDS,
	 "setRealTimeSimulation(enableRealTimeSimulation, physicsClientId=0)\n"
	 "Enable or disable real time simulation (using the real time clock,"
	 " RTC) in the physics server. Expects one integer argument, 0 or 1"},

	{"setPhysicsEngineParameter", (PyCFunction)pybullet_setPhysicsEngineParameter, METH_VARARGS | METH_KEYWORDS,
	 "Set some internal physics engine parameter, such as cfm or erp etc."},

	{"getPhysicsEngineParameters", (PyCFunction)pybullet_getPhysicsEngineParameters, METH_VARARGS | METH_KEYWORDS,
	 "Get the current values of internal physics engine parameters"},

	{"setInternalSimFlags", (PyCFunction)pybullet_setInternalSimFlags, METH_VARARGS | METH_KEYWORDS,
	 "This is for experimental purposes, use at own risk, magic may or not happen"},

	{"loadURDF", (PyCFunction)pybullet_loadURDF, METH_VARARGS | METH_KEYWORDS,
	 "bodyUniqueId = loadURDF(fileName, basePosition=[0.,0.,0.], baseOrientation=[0.,0.,0.,1.], "
	 "useMaximalCoordinates=0, useFixedBase=0, flags=0, globalScaling=1.0, physicsClientId=0)\n"
	 "Create a multibody by loading a URDF file."},

	{"loadSDF", (PyCFunction)pybullet_loadSDF, METH_VARARGS | METH_KEYWORDS,
	 "Load multibodies from an SDF file."},
#ifndef SKIP_SOFT_BODY_MULTI_BODY_DYNAMICS_WORLD
	{"loadSoftBody", (PyCFunction)pybullet_loadSoftBody, METH_VARARGS | METH_KEYWORDS,
	 "Load a softbody from an obj file."},

	{"createSoftBodyAnchor", (PyCFunction)pybullet_createSoftBodyAnchor, METH_VARARGS | METH_KEYWORDS,
	 "Create an anchor (attachment) between a soft body and a rigid or multi body."},

#endif
	{"loadBullet", (PyCFunction)pybullet_loadBullet, METH_VARARGS | METH_KEYWORDS,
	 "Load a world from a .bullet file."},

	{"saveBullet", (PyCFunction)pybullet_saveBullet, METH_VARARGS | METH_KEYWORDS,
	 "Save the full state of the world to a .bullet file."},

	{"restoreState", (PyCFunction)pybullet_restoreState, METH_VARARGS | METH_KEYWORDS,
	 "Restore the full state of an existing world."},

	{"saveState", (PyCFunction)pybullet_saveState, METH_VARARGS | METH_KEYWORDS,
	 "Save the full state of the world to memory."},

	 { "removeState", (PyCFunction)pybullet_removeState, METH_VARARGS | METH_KEYWORDS,
	"Remove a state created using saveState by its state unique id." },

	{"loadMJCF", (PyCFunction)pybullet_loadMJCF, METH_VARARGS | METH_KEYWORDS,
	 "Load multibodies from an MJCF file."},

	{"createCollisionShape", (PyCFunction)pybullet_createCollisionShape, METH_VARARGS | METH_KEYWORDS,
	 "Create a collision shape. Returns a non-negative (int) unique id, if successfull, negative otherwise."},

	{"createCollisionShapeArray", (PyCFunction)pybullet_createCollisionShapeArray, METH_VARARGS | METH_KEYWORDS,
	 "Create collision shapes. Returns a non-negative (int) unique id, if successfull, negative otherwise."},

	{"removeCollisionShape", (PyCFunction)pybullet_removeCollisionShape, METH_VARARGS | METH_KEYWORDS,
	 "Remove a collision shape. Only useful when the collision shape is not used in a body (to perform a getClosestPoint query)."},

     {"getMeshData", (PyCFunction)pybullet_getMeshData, METH_VARARGS | METH_KEYWORDS,
	 "Get mesh data. Returns vertices etc from the mesh."},

	{"createVisualShape", (PyCFunction)pybullet_createVisualShape, METH_VARARGS | METH_KEYWORDS,
	 "Create a visual shape. Returns a non-negative (int) unique id, if successfull, negative otherwise."},

	{"createVisualShapeArray", (PyCFunction)pybullet_createVisualShapeArray, METH_VARARGS | METH_KEYWORDS,
	 "Create visual shapes. Returns a non-negative (int) unique id, if successfull, negative otherwise."},

	{"createMultiBody", (PyCFunction)pybullet_createMultiBody, METH_VARARGS | METH_KEYWORDS,
	 "Create a multi body. Returns a non-negative (int) unique id, if successfull, negative otherwise."},

	{"createConstraint", (PyCFunction)pybullet_createUserConstraint, METH_VARARGS | METH_KEYWORDS,
	 "Create a constraint between two bodies. Returns a (int) unique id, if successfull."},

	{"changeConstraint", (PyCFunction)pybullet_changeUserConstraint, METH_VARARGS | METH_KEYWORDS,
	 "Change some parameters of an existing constraint, such as the child pivot or child frame orientation, using its unique id."},

	{"removeConstraint", (PyCFunction)pybullet_removeUserConstraint, METH_VARARGS | METH_KEYWORDS,
	 "Remove a constraint using its unique id."},

	{"enableJointForceTorqueSensor", (PyCFunction)pybullet_enableJointForceTorqueSensor, METH_VARARGS | METH_KEYWORDS,
	 "Enable or disable a joint force/torque sensor measuring the joint reaction forces."},

	{"saveWorld", (PyCFunction)pybullet_saveWorld, METH_VARARGS | METH_KEYWORDS,
	 "Save a approximate Python file to reproduce the current state of the world: saveWorld"
	 "(filename). (very preliminary and approximately)"},

	{"getNumBodies", (PyCFunction)pybullet_getNumBodies, METH_VARARGS | METH_KEYWORDS,
	 "Get the number of bodies in the simulation."},

	{"getBodyUniqueId", (PyCFunction)pybullet_getBodyUniqueId, METH_VARARGS | METH_KEYWORDS,
	 "getBodyUniqueId is used after connecting to server with existing bodies."
	 "Get the unique id of the body, given a integer range [0.. number of bodies)."},

	{"getBodyInfo", (PyCFunction)pybullet_getBodyInfo, METH_VARARGS | METH_KEYWORDS,
	 "Get the body info, given a body unique id."},

	{ "computeDofCount", (PyCFunction)pybullet_computeDofCount, METH_VARARGS | METH_KEYWORDS,
	"computeDofCount returns the number of degrees of freedom, including 7 degrees of freedom for the base in case of floating base" },

	{"syncBodyInfo", (PyCFunction)pybullet_syncBodyInfo, METH_VARARGS | METH_KEYWORDS,
	 "syncBodyInfo(physicsClientId=0)\n"
	 "Update body and constraint/joint information, in case other clients made changes."},

	{"syncUserData", (PyCFunction)pybullet_syncUserData, METH_VARARGS | METH_KEYWORDS,
	 "syncUserData(bodyUniqueIds=[], physicsClientId=0)\n"
	 "Update user data, in case other clients made changes."},

	{"addUserData", (PyCFunction)pybullet_addUserData, METH_VARARGS | METH_KEYWORDS,
	 "addUserData(bodyUniqueId, key, value, linkIndex=-1, visualShapeIndex=-1, physicsClientId=0)\n"
	 "Adds or updates a user data entry. Returns user data identifier."},

	{"getUserData", (PyCFunction)pybullet_getUserData, METH_VARARGS | METH_KEYWORDS,
	 "getUserData(userDataId, physicsClientId=0)\n"
	 "Returns the user data value."},

	{"removeUserData", (PyCFunction)pybullet_removeUserData, METH_VARARGS | METH_KEYWORDS,
	 "removeUserData(userDataId, physicsClientId=0)\n"
	 "Removes a user data entry."},

	{"getUserDataId", (PyCFunction)pybullet_getUserDataId, METH_VARARGS | METH_KEYWORDS,
	 "getUserDataId(bodyUniqueId, key, linkIndex=-1, visualShapeIndex=-1, physicsClientId=0)\n"
	 "Retrieves the userDataId given the key and optionally link and visual shape index."},

	{"getNumUserData", (PyCFunction)pybullet_getNumUserData, METH_VARARGS | METH_KEYWORDS,
	 "getNumUserData(bodyUniqueId physicsClientId=0)\n"
	 "Retrieves the number of user data entries in a body."},

	{"getUserDataInfo", (PyCFunction)pybullet_getUserDataInfo, METH_VARARGS | METH_KEYWORDS,
	 "getUserDataInfo(bodyUniqueId, userDataIndex, physicsClientId=0)\n"
	 "Retrieves the key and the identifier of a user data as (userDataId, key, bodyUniqueId, linkIndex, visualShapeIndex)."},

	{"removeBody", (PyCFunction)pybullet_removeBody, METH_VARARGS | METH_KEYWORDS,
	 "Remove a body by its body unique id."},

	{"getNumConstraints", (PyCFunction)pybullet_getNumConstraints, METH_VARARGS | METH_KEYWORDS,
	 "Get the number of user-created constraints in the simulation."},

	{"getConstraintInfo", (PyCFunction)pybullet_getConstraintInfo, METH_VARARGS | METH_KEYWORDS,
	 "Get the user-created constraint info, given a constraint unique id."},

	{"getConstraintState", (PyCFunction)pybullet_getConstraintState, METH_VARARGS | METH_KEYWORDS,
	 "Get the user-created constraint state (applied forces), given a constraint unique id."},

	{"getConstraintUniqueId", (PyCFunction)pybullet_getConstraintUniqueId, METH_VARARGS | METH_KEYWORDS,
	 "Get the unique id of the constraint, given a integer index in range [0.. number of constraints)."},

	{"getBasePositionAndOrientation", (PyCFunction)pybullet_getBasePositionAndOrientation,
	 METH_VARARGS | METH_KEYWORDS,
	 "Get the world position and orientation of the base of the object. "
	 "(x,y,z) position vector and (x,y,z,w) quaternion orientation."},

	{"getAABB", (PyCFunction)pybullet_getAABB,
	 METH_VARARGS | METH_KEYWORDS,
	 "Get the axis aligned bound box min and max coordinates in world space."},

	{"resetBasePositionAndOrientation",
	 (PyCFunction)pybullet_resetBasePositionAndOrientation, METH_VARARGS | METH_KEYWORDS,
	 "Reset the world position and orientation of the base of the object "
	 "instantaneously, not through physics simulation. (x,y,z) position vector "
	 "and (x,y,z,w) quaternion orientation."},
	
	{ "unsupportedChangeScaling",
	 (PyCFunction)pybullet_changeScaling, METH_VARARGS | METH_KEYWORDS,
	 "Change the scaling of the base of an object."	 
	 "Warning: unsupported rudimentary feature that has many limitations."
	 },

	{"getBaseVelocity", (PyCFunction)pybullet_getBaseVelocity,
	 METH_VARARGS | METH_KEYWORDS,
	 "Get the linear and angular velocity of the base of the object "
	 " in world space coordinates. "
	 "(x,y,z) linear velocity vector and (x,y,z) angular velocity vector."},

	{"resetBaseVelocity", (PyCFunction)pybullet_resetBaseVelocity, METH_VARARGS | METH_KEYWORDS,
	 "Reset the linear and/or angular velocity of the base of the object "
	 " in world space coordinates. "
	 "linearVelocity (x,y,z) and angularVelocity (x,y,z)."},

	{"getNumJoints", (PyCFunction)pybullet_getNumJoints, METH_VARARGS | METH_KEYWORDS,
	 "Get the number of joints for an object."},

	{"getJointInfo", (PyCFunction)pybullet_getJointInfo, METH_VARARGS | METH_KEYWORDS,
	 "Get the name and type info for a joint on a body."},

	{"getJointState", (PyCFunction)pybullet_getJointState, METH_VARARGS | METH_KEYWORDS,
	 "Get the state (position, velocity etc) for a joint on a body."},

	{"getJointStates", (PyCFunction)pybullet_getJointStates, METH_VARARGS | METH_KEYWORDS,
	 "Get the state (position, velocity etc) for multiple joints on a body."},

	 { "getJointStateMultiDof", (PyCFunction)pybullet_getJointStateMultiDof, METH_VARARGS | METH_KEYWORDS,
		"Get the state (position, velocity etc) for a joint on a body. (supports planar and spherical joints)" },

	{ "getJointStatesMultiDof", (PyCFunction)pybullet_getJointStatesMultiDof, METH_VARARGS | METH_KEYWORDS,
		 "Get the states (position, velocity etc) for multiple joint on a body. (supports planar and spherical joints)" },


	{"getLinkState", (PyCFunction)pybullet_getLinkState, METH_VARARGS | METH_KEYWORDS,
	 "position_linkcom_world, world_rotation_linkcom,\n"
	 "position_linkcom_frame, frame_rotation_linkcom,\n"
	 "position_frame_world, world_rotation_frame,\n"
	 "linearVelocity_linkcom_world, angularVelocity_linkcom_world\n"
	 "  = getLinkState(objectUniqueId, linkIndex, computeLinkVelocity=0,\n"
	 "                 computeForwardKinematics=0, physicsClientId=0)\n"
	 "Provides extra information such as the Cartesian world coordinates"
	 " center of mass (COM) of the link, relative to the world reference"
	 " frame."},

	 { "getLinkStates", (PyCFunction)pybullet_getLinkStates, METH_VARARGS | METH_KEYWORDS,
	"same as getLinkState except it takes a list of linkIndices" },

	{"resetJointState", (PyCFunction)pybullet_resetJointState, METH_VARARGS | METH_KEYWORDS,
	 "resetJointState(objectUniqueId, jointIndex, targetValue, targetVelocity=0, physicsClientId=0)\n"
	 "Reset the state (position, velocity etc) for a joint on a body "
	 "instantaneously, not through physics simulation."},

	{"resetJointStateMultiDof", (PyCFunction)pybullet_resetJointStateMultiDof, METH_VARARGS | METH_KEYWORDS,
	 "resetJointStateMultiDof(objectUniqueId, jointIndex, targetValue, targetVelocity=0, physicsClientId=0)\n"
	 "Reset the state (position, velocity etc) for a joint on a body "
	 "instantaneously, not through physics simulation."},
	 { "resetJointStatesMultiDof", (PyCFunction)pybullet_resetJointStatesMultiDof, METH_VARARGS | METH_KEYWORDS,
		"resetJointStatesMultiDof(objectUniqueId, jointIndices, targetValues, targetVelocities=0, physicsClientId=0)\n"
		"Reset the states (position, velocity etc) for multiple joints on a body "
		"instantaneously, not through physics simulation." },

	{"changeDynamics", (PyCFunction)pybullet_changeDynamicsInfo, METH_VARARGS | METH_KEYWORDS,
	 "change dynamics information such as mass, lateral friction coefficient."},

	{"getDynamicsInfo", (PyCFunction)pybullet_getDynamicsInfo, METH_VARARGS | METH_KEYWORDS,
	 "Get dynamics information such as mass, lateral friction coefficient."},

	{"setJointMotorControl", (PyCFunction)pybullet_setJointMotorControl, METH_VARARGS,
	 "This (obsolete) method cannot select non-zero physicsClientId, use setJointMotorControl2 instead."
	 "Set a single joint motor control mode and desired target value. There is "
	 "no immediate state change, stepSimulation will process the motors."},

	{"setJointMotorControl2", (PyCFunction)pybullet_setJointMotorControl2, METH_VARARGS | METH_KEYWORDS,
	 "Set a single joint motor control mode and desired target value. There is "
	 "no immediate state change, stepSimulation will process the motors."},

	{"setJointMotorControlMultiDof", (PyCFunction)pybullet_setJointMotorControlMultiDof, METH_VARARGS | METH_KEYWORDS,
	 "Set a single joint motor control mode and desired target value. There is "
	 "no immediate state change, stepSimulation will process the motors."
	 "This method sets multi-degree-of-freedom motor such as the spherical joint motor."},

	 { "setJointMotorControlMultiDofArray", (PyCFunction)pybullet_setJointMotorControlMultiDofArray, METH_VARARGS | METH_KEYWORDS,
		 "Set control mode and desired target values for multiple motors. There is "
		 "no immediate state change, stepSimulation will process the motors."
		 "This method sets multi-degree-of-freedom motor such as the spherical joint motor." },

		 


	{"setJointMotorControlArray", (PyCFunction)pybullet_setJointMotorControlArray, METH_VARARGS | METH_KEYWORDS,
	 "Set an array of motors control mode and desired target value. There is "
	 "no immediate state change, stepSimulation will process the motors."
	 "This is similar to setJointMotorControl2, with jointIndices as a list, and optional targetPositions, "
	 "targetVelocities, forces, kds and kps as lists"
	 "Using setJointMotorControlArray has the benefit of lower calling overhead."},

	{"applyExternalForce", (PyCFunction)pybullet_applyExternalForce, METH_VARARGS | METH_KEYWORDS,
	 "for objectUniqueId, linkIndex (-1 for base/root link), apply a force "
	 "[x,y,z] at the a position [x,y,z], flag to select FORCE_IN_LINK_FRAME or "
	 "WORLD_FRAME coordinates"},

	{"applyExternalTorque", (PyCFunction)pybullet_applyExternalTorque, METH_VARARGS | METH_KEYWORDS,
	 "for objectUniqueId, linkIndex (-1 for base/root link) apply a torque "
	 "[x,y,z] in Cartesian coordinates, flag to select TORQUE_IN_LINK_FRAME or "
	 "WORLD_FRAME coordinates"},

	{"renderImage", pybullet_renderImageObsolete, METH_VARARGS,
	 "obsolete, please use getCameraImage and getViewProjectionMatrices instead"},

	{"getCameraImage", (PyCFunction)pybullet_getCameraImage, METH_VARARGS | METH_KEYWORDS,
	 "Render an image (given the pixel resolution width, height, camera viewMatrix "
	 ", projectionMatrix, lightDirection, lightColor, lightDistance, shadow, lightAmbientCoeff, lightDiffuseCoeff, lightSpecularCoeff, and renderer), and return the "
	 "8-8-8bit RGB pixel data and floating point depth values"
#ifdef PYBULLET_USE_NUMPY
	 " as NumPy arrays"
#endif
	},

	{"isNumpyEnabled", (PyCFunction)pybullet_isNumpyEnabled, METH_VARARGS | METH_KEYWORDS,
	 "return True if PyBullet was compiled with NUMPY support. This makes the getCameraImage API faster"},

	{"computeViewMatrix", (PyCFunction)pybullet_computeViewMatrix, METH_VARARGS | METH_KEYWORDS,
	 "Compute a camera viewmatrix from camera eye,  target position and up vector "},

	{"computeViewMatrixFromYawPitchRoll", (PyCFunction)pybullet_computeViewMatrixFromYawPitchRoll, METH_VARARGS | METH_KEYWORDS,
	 "Compute a camera viewmatrix from camera eye,  target position and up vector "},

	{"computeProjectionMatrix", (PyCFunction)pybullet_computeProjectionMatrix, METH_VARARGS | METH_KEYWORDS,
	 "Compute a camera projection matrix from screen left/right/bottom/top/near/far values"},

	{"computeProjectionMatrixFOV", (PyCFunction)pybullet_computeProjectionMatrixFOV, METH_VARARGS | METH_KEYWORDS,
	 "Compute a camera projection matrix from fov, aspect ratio, near, far values"},

	{"getContactPoints", (PyCFunction)pybullet_getContactPointData, METH_VARARGS | METH_KEYWORDS,
	 "Return existing contact points after the stepSimulation command. "
	 "Optional arguments one or two object unique "
	 "ids, that need to be involved in the contact."},

	{"getClosestPoints", (PyCFunction)pybullet_getClosestPointData, METH_VARARGS | METH_KEYWORDS,
	 "Compute the closest points between two objects, if the distance is below a given threshold."
	 "Input is two objects unique ids and distance threshold."},

	{"getOverlappingObjects", (PyCFunction)pybullet_getOverlappingObjects, METH_VARARGS | METH_KEYWORDS,
	 "Return all the objects that have overlap with a given "
	 "axis-aligned bounding box volume (AABB)."
	 "Input are two vectors defining the AABB in world space [min_x,min_y,min_z],[max_x,max_y,max_z]."},

	{"setCollisionFilterPair", (PyCFunction)pybullet_setCollisionFilterPair, METH_VARARGS | METH_KEYWORDS,
	 "Enable or disable collision detection between two object links."
	 "Input are two object unique ids and two link indices and an enum"
	 "to enable or disable collisions."},

	{"setCollisionFilterGroupMask", (PyCFunction)pybullet_setCollisionFilterGroupMask, METH_VARARGS | METH_KEYWORDS,
	 "Set the collision filter group and the mask for a body."},

	{"addUserDebugLine", (PyCFunction)pybullet_addUserDebugLine, METH_VARARGS | METH_KEYWORDS,
	 "Add a user debug draw line with lineFrom[3], lineTo[3], lineColorRGB[3], lineWidth, lifeTime. "
	 "A lifeTime of 0 means permanent until removed. Returns a unique id for the user debug item."},

	{"addUserDebugText", (PyCFunction)pybullet_addUserDebugText, METH_VARARGS | METH_KEYWORDS,
	 "Add a user debug draw line with text, textPosition[3], textSize and lifeTime in seconds "
	 "A lifeTime of 0 means permanent until removed. Returns a unique id for the user debug item."},

	{"addUserDebugParameter", (PyCFunction)pybullet_addUserDebugParameter, METH_VARARGS | METH_KEYWORDS,
	 "Add a user debug parameter, such as a slider, that can be controlled using a GUI."},
	{"readUserDebugParameter", (PyCFunction)pybullet_readUserDebugParameter, METH_VARARGS | METH_KEYWORDS,
	 "Read the current value of a user debug parameter, given the user debug item unique id."},

	{"removeUserDebugItem", (PyCFunction)pybullet_removeUserDebugItem, METH_VARARGS | METH_KEYWORDS,
	 "remove a user debug draw item, giving its unique id"},

	{"removeAllUserDebugItems", (PyCFunction)pybullet_removeAllUserDebugItems, METH_VARARGS | METH_KEYWORDS,
	 "remove all user debug draw items"},

	 { "removeAllUserParameters", (PyCFunction)pybullet_removeAllUserParameters, METH_VARARGS | METH_KEYWORDS,
		 "remove all user debug parameters (sliders, buttons)" },

	{"setDebugObjectColor", (PyCFunction)pybullet_setDebugObjectColor, METH_VARARGS | METH_KEYWORDS,
	 "Override the wireframe debug drawing color for a particular object unique id / link index."
	 "If you ommit the color, the custom color will be removed."},

	{"getDebugVisualizerCamera", (PyCFunction)pybullet_getDebugVisualizerCamera, METH_VARARGS | METH_KEYWORDS,
	 "Get information about the 3D visualizer camera, such as width, height, view matrix, projection matrix etc."},

	{"configureDebugVisualizer", (PyCFunction)pybullet_configureDebugVisualizer, METH_VARARGS | METH_KEYWORDS,
	 "For the 3D OpenGL Visualizer, enable/disable GUI, shadows."},

	{"resetDebugVisualizerCamera", (PyCFunction)pybullet_resetDebugVisualizerCamera, METH_VARARGS | METH_KEYWORDS,
	 "For the 3D OpenGL Visualizer, set the camera distance, yaw, pitch and target position."},

	{"getVisualShapeData", (PyCFunction)pybullet_getVisualShapeData, METH_VARARGS | METH_KEYWORDS,
	 "Return the visual shape information for one object."},

	{"getCollisionShapeData", (PyCFunction)pybullet_getCollisionShapeData, METH_VARARGS | METH_KEYWORDS,
	 "Return the collision shape information for one object."},

	{"changeVisualShape", (PyCFunction)pybullet_changeVisualShape, METH_VARARGS | METH_KEYWORDS,
	 "Change part of the visual shape information for one object."},

	{"resetVisualShapeData", (PyCFunction)pybullet_changeVisualShape, METH_VARARGS | METH_KEYWORDS,
	 "Obsolete method, kept for backward compatibility, use changeVisualShapeData instead."},

	{"loadTexture", (PyCFunction)pybullet_loadTexture, METH_VARARGS | METH_KEYWORDS,
	 "Load texture file."},

	{"changeTexture", (PyCFunction)pybullet_changeTexture, METH_VARARGS | METH_KEYWORDS,
	 "Change a texture file."},

	{"getQuaternionFromEuler", (PyCFunction)pybullet_getQuaternionFromEuler, METH_VARARGS | METH_KEYWORDS,
	 "Convert Euler [roll, pitch, yaw] as in URDF/SDF convention, to "
	 "quaternion [x,y,z,w]"},

	{"getEulerFromQuaternion", (PyCFunction)pybullet_getEulerFromQuaternion, METH_VARARGS | METH_KEYWORDS,
	 "Convert quaternion [x,y,z,w] to Euler [roll, pitch, yaw] as in URDF/SDF "
	 "convention"},

	{"multiplyTransforms", (PyCFunction)pybullet_multiplyTransforms, METH_VARARGS | METH_KEYWORDS,
	 "Multiply two transform, provided as [position], [quaternion]."},

	{"invertTransform", (PyCFunction)pybullet_invertTransform, METH_VARARGS | METH_KEYWORDS,
	 "Invert a transform, provided as [position], [quaternion]."},

	{"getMatrixFromQuaternion", (PyCFunction)pybullet_getMatrixFromQuaternion, METH_VARARGS | METH_KEYWORDS,
	 "Compute the 3x3 matrix from a quaternion, as a list of 9 values (row-major)"},

	{"getQuaternionSlerp", (PyCFunction)pybullet_getQuaternionSlerp, METH_VARARGS | METH_KEYWORDS,
	 "Compute the spherical interpolation given a start and end quaternion and an interpolation value in range [0..1]"},

	{"getQuaternionFromAxisAngle", (PyCFunction)pybullet_getQuaternionFromAxisAngle, METH_VARARGS | METH_KEYWORDS,
	 "Compute the quaternion from axis and angle representation."},

	{"getAxisAngleFromQuaternion", (PyCFunction)pybullet_getAxisAngleFromQuaternion, METH_VARARGS | METH_KEYWORDS,
	 "Compute the quaternion from axis and angle representation."},

	{"getDifferenceQuaternion", (PyCFunction)pybullet_getDifferenceQuaternion, METH_VARARGS | METH_KEYWORDS,
	 "Compute the quaternion difference from two quaternions."},

	{"getAxisDifferenceQuaternion", (PyCFunction)pybullet_getAxisDifferenceQuaternion, METH_VARARGS | METH_KEYWORDS,
	 "Compute the velocity axis difference from two quaternions."},

	{"calculateVelocityQuaternion", (PyCFunction)pybullet_calculateVelocityQuaternion, METH_VARARGS | METH_KEYWORDS,
	 "Compute the angular velocity given start and end quaternion and delta time."},

	{"rotateVector", (PyCFunction)pybullet_rotateVector, METH_VARARGS | METH_KEYWORDS,
	 "Rotate a vector using a quaternion."},

	{"calculateInverseDynamics", (PyCFunction)pybullet_calculateInverseDynamics, METH_VARARGS | METH_KEYWORDS,
	 "Given an object id, joint positions, joint velocities and joint "
	 "accelerations, compute the joint forces using Inverse Dynamics"},

	{"calculateJacobian", (PyCFunction)pybullet_calculateJacobian, METH_VARARGS | METH_KEYWORDS,
	 "linearJacobian, angularJacobian = calculateJacobian(bodyUniqueId, "
	 "linkIndex, localPosition, objPositions, objVelocities, objAccelerations, physicsClientId=0)\n"
	 "Compute the jacobian for a specified local position on a body and its kinematics.\n"
	 "Args:\n"
	 "  bodyIndex - a scalar defining the unique object id.\n"
	 "  linkIndex - a scalar identifying the link containing the local point.\n"
	 "  localPosition - a list of [x, y, z] of the coordinates defined in the link frame.\n"
	 "  objPositions - a list of the joint positions.\n"
	 "  objVelocities - a list of the joint velocities.\n"
	 "  objAccelerations - a list of the joint accelerations.\n"
	 "Returns:\n"
	 "  linearJacobian - a list of the partial linear velocities of the jacobian.\n"
	 "  angularJacobian - a list of the partial angular velocities of the jacobian.\n"},

	{"calculateMassMatrix", (PyCFunction)pybullet_calculateMassMatrix, METH_VARARGS | METH_KEYWORDS,
	 "massMatrix = calculateMassMatrix(bodyUniqueId, objPositions, physicsClientId=0)\n"
	 "Compute the mass matrix for an object and its chain of bodies.\n"
	 "Args:\n"
	 "  bodyIndex - a scalar defining the unique object id.\n"
	 "  objPositions - a list of the joint positions.\n"
	 "Returns:\n"
	 "  massMatrix - a list of lists of the mass matrix components.\n"},

		
	{"calculateInverseKinematics", (PyCFunction)pybullet_calculateInverseKinematics,
	 METH_VARARGS | METH_KEYWORDS,
	 "Inverse Kinematics bindings: Given an object id, "
	 "current joint positions and target position"
	 " for the end effector,"
	 "compute the inverse kinematics and return the new joint state"},
	 { "calculateInverseKinematics2", (PyCFunction)pybullet_calculateInverseKinematics2,
		METH_VARARGS | METH_KEYWORDS,
		"Inverse Kinematics bindings: Given an object id, "
		"current joint positions and target positions"
		" for the end effectors,"
		"compute the inverse kinematics and return the new joint state" 
	 },
	
	{"getVREvents", (PyCFunction)pybullet_getVREvents, METH_VARARGS | METH_KEYWORDS,
	 "Get Virtual Reality events, for example to track VR controllers position/buttons"},
	{"setVRCameraState", (PyCFunction)pybullet_setVRCameraState, METH_VARARGS | METH_KEYWORDS,
	 "Set properties of the VR Camera such as its root transform "
	 "for teleporting or to track objects (camera inside a vehicle for example)."},
	{"getKeyboardEvents", (PyCFunction)pybullet_getKeyboardEvents, METH_VARARGS | METH_KEYWORDS,
	 "Get keyboard events, keycode and state (KEY_IS_DOWN, KEY_WAS_TRIGGERED, KEY_WAS_RELEASED)"},

	{"getMouseEvents", (PyCFunction)pybullet_getMouseEvents, METH_VARARGS | METH_KEYWORDS,
	 "Get mouse events, event type and button state (KEY_IS_DOWN, KEY_WAS_TRIGGERED, KEY_WAS_RELEASED)"},

	{"startStateLogging", (PyCFunction)pybullet_startStateLogging, METH_VARARGS | METH_KEYWORDS,
	 "Start logging of state, such as robot base position, orientation, joint positions etc. "
	 "Specify loggingType (STATE_LOGGING_MINITAUR, STATE_LOGGING_GENERIC_ROBOT, STATE_LOGGING_VR_CONTROLLERS, STATE_LOGGING_CONTACT_POINTS, etc), "
	 "fileName, optional objectUniqueId, maxLogDof, bodyUniqueIdA, bodyUniqueIdB, linkIndexA, linkIndexB. Function returns int loggingUniqueId"},
	{"stopStateLogging", (PyCFunction)pybullet_stopStateLogging, METH_VARARGS | METH_KEYWORDS,
	 "Stop logging of robot state, given a loggingUniqueId."},
	{"rayTest", (PyCFunction)pybullet_rayTestObsolete, METH_VARARGS | METH_KEYWORDS,
	 "Cast a ray and return the first object hit, if any. "
	 "Takes two arguments (from_position [x,y,z] and to_position [x,y,z] in Cartesian world coordinates"},

	{"rayTestBatch", (PyCFunction)pybullet_rayTestBatch, METH_VARARGS | METH_KEYWORDS,
	 "Cast a batch of rays and return the result for each of the rays (first object hit, if any. or -1) "
	 "Takes two required arguments (list of from_positions [x,y,z] and a list of to_positions [x,y,z] in Cartesian world coordinates) "
	 "and one optional argument numThreads to specify the number of threads to use to compute the ray intersections for the batch. "
	 "Specify 0 to let Bullet decide, 1 (default) for single core execution, 2 or more to select the number of threads to use."},

	{"loadPlugin", (PyCFunction)pybullet_loadPlugin, METH_VARARGS | METH_KEYWORDS,
	 "Load a plugin, could implement custom commands etc."},

	{"unloadPlugin", (PyCFunction)pybullet_unloadPlugin, METH_VARARGS | METH_KEYWORDS,
	 "Unload a plugin, given the pluginUniqueId."},

	{"executePluginCommand", (PyCFunction)pybullet_executePluginCommand, METH_VARARGS | METH_KEYWORDS,
	 "Execute a command, implemented in a plugin."},

	{"submitProfileTiming", (PyCFunction)pybullet_submitProfileTiming, METH_VARARGS | METH_KEYWORDS,
	 "Add a custom profile timing that will be visible in performance profile recordings on the physics server."
	 "On the physics server (in GUI and VR mode) you can press 'p' to start and/or stop profile recordings"},

	{"setTimeOut", (PyCFunction)pybullet_setTimeOut, METH_VARARGS | METH_KEYWORDS,
	 "Set the timeOut in seconds, used for most of the API calls."},

#ifdef BT_ENABLE_VHACD
	{"vhacd", (PyCFunction)pybullet_vhacd, METH_VARARGS | METH_KEYWORDS,
	 "Compute volume hierarchical convex decomposition of an OBJ file."},
#endif //BT_ENABLE_VHACD
	{"setAdditionalSearchPath", (PyCFunction)pybullet_setAdditionalSearchPath,
	 METH_VARARGS | METH_KEYWORDS,
	 "Set an additional search path, used to load URDF/SDF files."},

	{"getAPIVersion", (PyCFunction)pybullet_getAPIVersion,
	 METH_VARARGS | METH_KEYWORDS,
	 "Get version of the API. Compatibility exists for connections using the same API version. Make sure both client and server use the same number of bits (32-bit or 64bit)."},

	// todo(erwincoumans)
	// saveSnapshot
	// loadSnapshot
	// raycast info
	// object names

	{NULL, NULL, 0, NULL} /* Sentinel */
};

///copied from CommonCallbacks.h
enum
{
	B3G_ESCAPE = 27,
	B3G_SPACE = 32,
	B3G_F1 = 0xff00,
	B3G_F2,
	B3G_F3,
	B3G_F4,
	B3G_F5,
	B3G_F6,
	B3G_F7,
	B3G_F8,
	B3G_F9,
	B3G_F10,
	B3G_F11,
	B3G_F12,
	B3G_F13,
	B3G_F14,
	B3G_F15,
	B3G_LEFT_ARROW,
	B3G_RIGHT_ARROW,
	B3G_UP_ARROW,
	B3G_DOWN_ARROW,
	B3G_PAGE_UP,
	B3G_PAGE_DOWN,
	B3G_END,
	B3G_HOME,
	B3G_INSERT,
	B3G_DELETE,
	B3G_BACKSPACE,
	B3G_SHIFT,
	B3G_CONTROL,
	B3G_ALT,
	B3G_RETURN,
};

#if PY_MAJOR_VERSION >= 3
static struct PyModuleDef moduledef = {
	PyModuleDef_HEAD_INIT, "pybullet", /* m_name */
	"Python bindings for Bullet Physics Robotics API (also known as Shared "
	"Memory API)", /* m_doc */
	-1,            /* m_size */
	SpamMethods,   /* m_methods */
	NULL,          /* m_reload */
	NULL,          /* m_traverse */
	NULL,          /* m_clear */
	NULL,          /* m_free */
};
#endif

#if __GNUC__ >= 4
__attribute__((visibility ("default")))
#endif
PyMODINIT_FUNC
#if PY_MAJOR_VERSION >= 3
PyInit_pybullet(void)
#else
initpybullet(void)
#endif
{
	PyObject* m;
#if PY_MAJOR_VERSION >= 3
	m = PyModule_Create(&moduledef);
#else
	m = Py_InitModule3("pybullet", SpamMethods, "Python bindings for Bullet");
#endif

#if PY_MAJOR_VERSION >= 3
	if (m == NULL) return m;
#else
	if (m == NULL) return;
#endif

	PyModule_AddIntConstant(m, "SHARED_MEMORY",
							eCONNECT_SHARED_MEMORY);                                    // user read
	PyModule_AddIntConstant(m, "DIRECT", eCONNECT_DIRECT);                              // user read
	PyModule_AddIntConstant(m, "GUI", eCONNECT_GUI);                                    // user read
	PyModule_AddIntConstant(m, "UDP", eCONNECT_UDP);                                    // user read
	PyModule_AddIntConstant(m, "TCP", eCONNECT_TCP);                                    // user read
	PyModule_AddIntConstant(m, "GUI_SERVER", eCONNECT_GUI_SERVER);                      // user read
	PyModule_AddIntConstant(m, "GUI_MAIN_THREAD", eCONNECT_GUI_MAIN_THREAD);            // user read
	PyModule_AddIntConstant(m, "SHARED_MEMORY_SERVER", eCONNECT_SHARED_MEMORY_SERVER);  // user read
	PyModule_AddIntConstant(m, "SHARED_MEMORY_GUI", eCONNECT_SHARED_MEMORY_GUI);  // user read
	PyModule_AddIntConstant(m, "GRAPHICS_CLIENT", eCONNECT_SHARED_MEMORY_GUI);  // user read
	PyModule_AddIntConstant(m, "GRAPHICS_SERVER", eCONNECT_GRAPHICS_SERVER);  // user read
	PyModule_AddIntConstant(m, "GRAPHICS_SERVER_TCP", eCONNECT_GRAPHICS_SERVER_TCP);  // user read
	PyModule_AddIntConstant(m, "GRAPHICS_SERVER_MAIN_THREAD", eCONNECT_GRAPHICS_SERVER_MAIN_THREAD);  // user read
	
	

	
#ifdef BT_ENABLE_DART
	PyModule_AddIntConstant(m, "DART", eCONNECT_DART);  // user read
#endif

#ifdef BT_ENABLE_PHYSX
	PyModule_AddIntConstant(m, "PhysX", eCONNECT_PHYSX);
#endif

#ifdef BT_ENABLE_MUJOCO
	PyModule_AddIntConstant(m, "MuJoCo", eCONNECT_MUJOCO);  // user read
#endif
#ifdef BT_ENABLE_GRPC
	PyModule_AddIntConstant(m, "GRPC", eCONNECT_GRPC);  // user read
#endif

	PyModule_AddIntConstant(m, "SHARED_MEMORY_KEY", SHARED_MEMORY_KEY);
	PyModule_AddIntConstant(m, "SHARED_MEMORY_KEY2", SHARED_MEMORY_KEY + 1);

	PyModule_AddIntConstant(m, "JOINT_REVOLUTE", eRevoluteType);        // user read
	PyModule_AddIntConstant(m, "JOINT_PRISMATIC", ePrismaticType);      // user read
	PyModule_AddIntConstant(m, "JOINT_SPHERICAL", eSphericalType);      // user read
	PyModule_AddIntConstant(m, "JOINT_PLANAR", ePlanarType);            // user read
	PyModule_AddIntConstant(m, "JOINT_FIXED", eFixedType);              // user read
	PyModule_AddIntConstant(m, "JOINT_POINT2POINT", ePoint2PointType);  // user read
	PyModule_AddIntConstant(m, "JOINT_GEAR", eGearType);                // user read

	PyModule_AddIntConstant(m, "SENSOR_FORCE_TORQUE", eSensorForceTorqueType);  // user read

	PyModule_AddIntConstant(m, "JOINT_FEEDBACK_IN_WORLD_SPACE", JOINT_FEEDBACK_IN_WORLD_SPACE);  // user read
	PyModule_AddIntConstant(m, "JOINT_FEEDBACK_IN_JOINT_FRAME", JOINT_FEEDBACK_IN_JOINT_FRAME);  // user read

	PyModule_AddIntConstant(m, "TORQUE_CONTROL", CONTROL_MODE_TORQUE);
	PyModule_AddIntConstant(m, "VELOCITY_CONTROL",
							CONTROL_MODE_VELOCITY);  // user read
	PyModule_AddIntConstant(m, "POSITION_CONTROL",
							CONTROL_MODE_POSITION_VELOCITY_PD);  // user read
	PyModule_AddIntConstant(m, "PD_CONTROL",
							CONTROL_MODE_PD);  // user read
	PyModule_AddIntConstant(m, "STABLE_PD_CONTROL",CONTROL_MODE_STABLE_PD);
	
	PyModule_AddIntConstant(m, "LINK_FRAME", EF_LINK_FRAME);
	PyModule_AddIntConstant(m, "WORLD_FRAME", EF_WORLD_FRAME);

	PyModule_AddIntConstant(m, "CONTACT_REPORT_EXISTING", CONTACT_QUERY_MODE_REPORT_EXISTING_CONTACT_POINTS);
	PyModule_AddIntConstant(m, "CONTACT_RECOMPUTE_CLOSEST", CONTACT_QUERY_MODE_COMPUTE_CLOSEST_POINTS);

	PyModule_AddIntConstant(m, "CONSTRAINT_SOLVER_LCP_SI", eConstraintSolverLCP_SI);
	PyModule_AddIntConstant(m, "CONSTRAINT_SOLVER_LCP_PGS", eConstraintSolverLCP_PGS);
	PyModule_AddIntConstant(m, "CONSTRAINT_SOLVER_LCP_DANTZIG", eConstraintSolverLCP_DANTZIG);
	//PyModule_AddIntConstant(m, "CONSTRAINT_SOLVER_LCP_LEMKE",eConstraintSolverLCP_LEMKE);
	//PyModule_AddIntConstant(m, "CONSTRAINT_SOLVER_LCP_NNCF",eConstraintSolverLCP_NNCG);
	//PyModule_AddIntConstant(m, "CONSTRAINT_SOLVER_LCP_BLOCK",eConstraintSolverLCP_BLOCK_PGS);

	PyModule_AddIntConstant(m, "RESET_USE_DEFORMABLE_WORLD", RESET_USE_DEFORMABLE_WORLD);
	PyModule_AddIntConstant(m, "RESET_USE_DISCRETE_DYNAMICS_WORLD", RESET_USE_DISCRETE_DYNAMICS_WORLD);
	PyModule_AddIntConstant(m, "RESET_USE_SIMPLE_BROADPHASE", RESET_USE_SIMPLE_BROADPHASE);

	PyModule_AddIntConstant(m, "VR_BUTTON_IS_DOWN", eButtonIsDown);
	PyModule_AddIntConstant(m, "VR_BUTTON_WAS_TRIGGERED", eButtonTriggered);
	PyModule_AddIntConstant(m, "VR_BUTTON_WAS_RELEASED", eButtonReleased);

	PyModule_AddIntConstant(m, "VR_MAX_CONTROLLERS", MAX_VR_CONTROLLERS);
	PyModule_AddIntConstant(m, "VR_MAX_BUTTONS", MAX_VR_BUTTONS);

	PyModule_AddIntConstant(m, "VR_DEVICE_CONTROLLER", VR_DEVICE_CONTROLLER);
	PyModule_AddIntConstant(m, "VR_DEVICE_HMD", VR_DEVICE_HMD);
	PyModule_AddIntConstant(m, "VR_DEVICE_GENERIC_TRACKER", VR_DEVICE_GENERIC_TRACKER);

	PyModule_AddIntConstant(m, "VR_CAMERA_TRACK_OBJECT_ORIENTATION", VR_CAMERA_TRACK_OBJECT_ORIENTATION);

	PyModule_AddIntConstant(m, "KEY_IS_DOWN", eButtonIsDown);
	PyModule_AddIntConstant(m, "KEY_WAS_TRIGGERED", eButtonTriggered);
	PyModule_AddIntConstant(m, "KEY_WAS_RELEASED", eButtonReleased);

	PyModule_AddIntConstant(m, "STATE_LOGGING_MINITAUR", STATE_LOGGING_MINITAUR);
	PyModule_AddIntConstant(m, "STATE_LOGGING_GENERIC_ROBOT", STATE_LOGGING_GENERIC_ROBOT);
	PyModule_AddIntConstant(m, "STATE_LOGGING_VR_CONTROLLERS", STATE_LOGGING_VR_CONTROLLERS);
	PyModule_AddIntConstant(m, "STATE_LOGGING_VIDEO_MP4", STATE_LOGGING_VIDEO_MP4);
	PyModule_AddIntConstant(m, "STATE_LOGGING_CONTACT_POINTS", STATE_LOGGING_CONTACT_POINTS);
	PyModule_AddIntConstant(m, "STATE_LOGGING_PROFILE_TIMINGS", STATE_LOGGING_PROFILE_TIMINGS);
	PyModule_AddIntConstant(m, "STATE_LOGGING_ALL_COMMANDS", STATE_LOGGING_ALL_COMMANDS);
	PyModule_AddIntConstant(m, "STATE_REPLAY_ALL_COMMANDS", STATE_REPLAY_ALL_COMMANDS);
	PyModule_AddIntConstant(m, "STATE_LOGGING_CUSTOM_TIMER", STATE_LOGGING_CUSTOM_TIMER);

	PyModule_AddIntConstant(m, "COV_ENABLE_GUI", COV_ENABLE_GUI);
	PyModule_AddIntConstant(m, "COV_ENABLE_SHADOWS", COV_ENABLE_SHADOWS);
	PyModule_AddIntConstant(m, "COV_ENABLE_WIREFRAME", COV_ENABLE_WIREFRAME);
	PyModule_AddIntConstant(m, "COV_ENABLE_VR_PICKING", COV_ENABLE_VR_PICKING);
	PyModule_AddIntConstant(m, "COV_ENABLE_VR_TELEPORTING", COV_ENABLE_VR_TELEPORTING);
	PyModule_AddIntConstant(m, "COV_ENABLE_RENDERING", COV_ENABLE_RENDERING);
	PyModule_AddIntConstant(m, "COV_ENABLE_TINY_RENDERER", COV_ENABLE_TINY_RENDERER);
	PyModule_AddIntConstant(m, "COV_ENABLE_Y_AXIS_UP", COV_ENABLE_Y_AXIS_UP);

	PyModule_AddIntConstant(m, "COV_ENABLE_VR_RENDER_CONTROLLERS", COV_ENABLE_VR_RENDER_CONTROLLERS);
	PyModule_AddIntConstant(m, "COV_ENABLE_KEYBOARD_SHORTCUTS", COV_ENABLE_KEYBOARD_SHORTCUTS);
	PyModule_AddIntConstant(m, "COV_ENABLE_MOUSE_PICKING", COV_ENABLE_MOUSE_PICKING);

	PyModule_AddIntConstant(m, "COV_ENABLE_RGB_BUFFER_PREVIEW", COV_ENABLE_RGB_BUFFER_PREVIEW);
	PyModule_AddIntConstant(m, "COV_ENABLE_DEPTH_BUFFER_PREVIEW", COV_ENABLE_DEPTH_BUFFER_PREVIEW);
	PyModule_AddIntConstant(m, "COV_ENABLE_SEGMENTATION_MARK_PREVIEW", COV_ENABLE_SEGMENTATION_MARK_PREVIEW);
	PyModule_AddIntConstant(m, "COV_ENABLE_PLANAR_REFLECTION", COV_ENABLE_PLANAR_REFLECTION);
	PyModule_AddIntConstant(m, "COV_ENABLE_SINGLE_STEP_RENDERING", COV_ENABLE_SINGLE_STEP_RENDERING);

	PyModule_AddIntConstant(m, "ER_TINY_RENDERER", ER_TINY_RENDERER);
	PyModule_AddIntConstant(m, "ER_BULLET_HARDWARE_OPENGL", ER_BULLET_HARDWARE_OPENGL);
	PyModule_AddIntConstant(m, "ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX", ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX);
	PyModule_AddIntConstant(m, "ER_NO_SEGMENTATION_MASK", ER_NO_SEGMENTATION_MASK);
	PyModule_AddIntConstant(m, "ER_USE_PROJECTIVE_TEXTURE", ER_USE_PROJECTIVE_TEXTURE);

	PyModule_AddIntConstant(m, "IK_DLS", IK_DLS);
	PyModule_AddIntConstant(m, "IK_SDLS", IK_SDLS);
	PyModule_AddIntConstant(m, "IK_HAS_TARGET_POSITION", IK_HAS_TARGET_POSITION);
	PyModule_AddIntConstant(m, "IK_HAS_TARGET_ORIENTATION", IK_HAS_TARGET_ORIENTATION);
	PyModule_AddIntConstant(m, "IK_HAS_NULL_SPACE_VELOCITY", IK_HAS_NULL_SPACE_VELOCITY);
	PyModule_AddIntConstant(m, "IK_HAS_JOINT_DAMPING", IK_HAS_JOINT_DAMPING);

	PyModule_AddIntConstant(m, "URDF_USE_INERTIA_FROM_FILE", URDF_USE_INERTIA_FROM_FILE);
	PyModule_AddIntConstant(m, "URDF_USE_IMPLICIT_CYLINDER", URDF_USE_IMPLICIT_CYLINDER);
	PyModule_AddIntConstant(m, "URDF_GLOBAL_VELOCITIES_MB", URDF_GLOBAL_VELOCITIES_MB);
	PyModule_AddIntConstant(m, "MJCF_COLORS_FROM_FILE", MJCF_COLORS_FROM_FILE);
	PyModule_AddIntConstant(m, "URDF_ENABLE_CACHED_GRAPHICS_SHAPES", URDF_ENABLE_CACHED_GRAPHICS_SHAPES);
	PyModule_AddIntConstant(m, "URDF_ENABLE_SLEEPING", URDF_ENABLE_SLEEPING);
	PyModule_AddIntConstant(m, "URDF_INITIALIZE_SAT_FEATURES", URDF_INITIALIZE_SAT_FEATURES);
	PyModule_AddIntConstant(m, "URDF_USE_MATERIAL_COLORS_FROM_MTL", URDF_USE_MATERIAL_COLORS_FROM_MTL);
	PyModule_AddIntConstant(m, "URDF_USE_MATERIAL_TRANSPARANCY_FROM_MTL", URDF_USE_MATERIAL_TRANSPARANCY_FROM_MTL);
	PyModule_AddIntConstant(m, "URDF_MAINTAIN_LINK_ORDER", URDF_MAINTAIN_LINK_ORDER);
	PyModule_AddIntConstant(m, "URDF_ENABLE_WAKEUP", URDF_ENABLE_WAKEUP);
	PyModule_AddIntConstant(m, "URDF_MERGE_FIXED_LINKS", URDF_MERGE_FIXED_LINKS);
	PyModule_AddIntConstant(m, "URDF_IGNORE_VISUAL_SHAPES", URDF_IGNORE_VISUAL_SHAPES);
	PyModule_AddIntConstant(m, "URDF_IGNORE_COLLISION_SHAPES",URDF_IGNORE_COLLISION_SHAPES);
	PyModule_AddIntConstant(m, "URDF_PRINT_URDF_INFO", URDF_PRINT_URDF_INFO);
	PyModule_AddIntConstant(m, "URDF_GOOGLEY_UNDEFINED_COLORS", URDF_GOOGLEY_UNDEFINED_COLORS);

	PyModule_AddIntConstant(m, "ACTIVATION_STATE_ENABLE_SLEEPING", eActivationStateEnableSleeping);
	PyModule_AddIntConstant(m, "ACTIVATION_STATE_DISABLE_SLEEPING", eActivationStateDisableSleeping);
	PyModule_AddIntConstant(m, "ACTIVATION_STATE_WAKE_UP", eActivationStateWakeUp);
	PyModule_AddIntConstant(m, "ACTIVATION_STATE_SLEEP", eActivationStateSleep);
	PyModule_AddIntConstant(m, "ACTIVATION_STATE_ENABLE_WAKEUP", eActivationStateEnableWakeup);
	PyModule_AddIntConstant(m, "ACTIVATION_STATE_DISABLE_WAKEUP", eActivationStateDisableWakeup);

	PyModule_AddIntConstant(m, "URDF_USE_SELF_COLLISION", URDF_USE_SELF_COLLISION);
	PyModule_AddIntConstant(m, "URDF_USE_SELF_COLLISION_EXCLUDE_PARENT", URDF_USE_SELF_COLLISION_EXCLUDE_PARENT);
	PyModule_AddIntConstant(m, "URDF_USE_SELF_COLLISION_INCLUDE_PARENT", URDF_USE_SELF_COLLISION_INCLUDE_PARENT);

	PyModule_AddIntConstant(m, "URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS", URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS);

	PyModule_AddIntConstant(m, "VISUAL_SHAPE_DATA_TEXTURE_UNIQUE_IDS", eVISUAL_SHAPE_DATA_TEXTURE_UNIQUE_IDS);
	PyModule_AddIntConstant(m, "VISUAL_SHAPE_DOUBLE_SIDED", eVISUAL_SHAPE_DOUBLE_SIDED);

	PyModule_AddIntConstant(m, "MAX_RAY_INTERSECTION_BATCH_SIZE", MAX_RAY_INTERSECTION_BATCH_SIZE_STREAMING);

	PyModule_AddIntConstant(m, "B3G_F1", B3G_F1);
	PyModule_AddIntConstant(m, "B3G_F2", B3G_F2);
	PyModule_AddIntConstant(m, "B3G_F3", B3G_F3);
	PyModule_AddIntConstant(m, "B3G_F4", B3G_F4);
	PyModule_AddIntConstant(m, "B3G_F5", B3G_F5);
	PyModule_AddIntConstant(m, "B3G_F6", B3G_F6);
	PyModule_AddIntConstant(m, "B3G_F7", B3G_F7);
	PyModule_AddIntConstant(m, "B3G_F8", B3G_F8);
	PyModule_AddIntConstant(m, "B3G_F9", B3G_F9);
	PyModule_AddIntConstant(m, "B3G_F10", B3G_F10);
	PyModule_AddIntConstant(m, "B3G_F11", B3G_F11);
	PyModule_AddIntConstant(m, "B3G_F12", B3G_F12);
	PyModule_AddIntConstant(m, "B3G_F13", B3G_F13);
	PyModule_AddIntConstant(m, "B3G_F14", B3G_F14);
	PyModule_AddIntConstant(m, "B3G_F15", B3G_F15);
	PyModule_AddIntConstant(m, "B3G_LEFT_ARROW", B3G_LEFT_ARROW);
	PyModule_AddIntConstant(m, "B3G_RIGHT_ARROW", B3G_RIGHT_ARROW);
	PyModule_AddIntConstant(m, "B3G_UP_ARROW", B3G_UP_ARROW);
	PyModule_AddIntConstant(m, "B3G_DOWN_ARROW", B3G_DOWN_ARROW);
	PyModule_AddIntConstant(m, "B3G_PAGE_UP", B3G_PAGE_UP);
	PyModule_AddIntConstant(m, "B3G_PAGE_DOWN", B3G_PAGE_DOWN);
	PyModule_AddIntConstant(m, "B3G_END", B3G_END);
	PyModule_AddIntConstant(m, "B3G_HOME", B3G_HOME);
	PyModule_AddIntConstant(m, "B3G_INSERT", B3G_INSERT);
	PyModule_AddIntConstant(m, "B3G_DELETE", B3G_DELETE);
	PyModule_AddIntConstant(m, "B3G_BACKSPACE", B3G_BACKSPACE);
	PyModule_AddIntConstant(m, "B3G_SHIFT", B3G_SHIFT);
	PyModule_AddIntConstant(m, "B3G_CONTROL", B3G_CONTROL);
	PyModule_AddIntConstant(m, "B3G_ALT", B3G_ALT);
	PyModule_AddIntConstant(m, "B3G_RETURN", B3G_RETURN);
	PyModule_AddIntConstant(m, "B3G_SPACE", B3G_SPACE);

	PyModule_AddIntConstant(m, "GEOM_SPHERE", GEOM_SPHERE);
	PyModule_AddIntConstant(m, "GEOM_BOX", GEOM_BOX);
	PyModule_AddIntConstant(m, "GEOM_CYLINDER", GEOM_CYLINDER);
	PyModule_AddIntConstant(m, "GEOM_MESH", GEOM_MESH);
	PyModule_AddIntConstant(m, "GEOM_PLANE", GEOM_PLANE);
	PyModule_AddIntConstant(m, "GEOM_CAPSULE", GEOM_CAPSULE);
	PyModule_AddIntConstant(m, "GEOM_HEIGHTFIELD", GEOM_HEIGHTFIELD);
	

	PyModule_AddIntConstant(m, "GEOM_FORCE_CONCAVE_TRIMESH", GEOM_FORCE_CONCAVE_TRIMESH);
	PyModule_AddIntConstant(m, "GEOM_CONCAVE_INTERNAL_EDGE", GEOM_CONCAVE_INTERNAL_EDGE);

	PyModule_AddIntConstant(m, "STATE_LOG_JOINT_MOTOR_TORQUES", STATE_LOG_JOINT_MOTOR_TORQUES);
	PyModule_AddIntConstant(m, "STATE_LOG_JOINT_USER_TORQUES", STATE_LOG_JOINT_USER_TORQUES);
	PyModule_AddIntConstant(m, "STATE_LOG_JOINT_TORQUES", STATE_LOG_JOINT_USER_TORQUES + STATE_LOG_JOINT_MOTOR_TORQUES);

	PyModule_AddIntConstant(m, "MESH_DATA_SIMULATION_MESH", B3_MESH_DATA_SIMULATION_MESH);

	PyModule_AddIntConstant(m, "AddFileIOAction", eAddFileIOAction);
	PyModule_AddIntConstant(m, "RemoveFileIOAction", eRemoveFileIOAction);

	PyModule_AddIntConstant(m, "PosixFileIO", ePosixFileIO);
	PyModule_AddIntConstant(m, "ZipFileIO", eZipFileIO);
	PyModule_AddIntConstant(m, "CNSFileIO", eCNSFileIO);

	SpamError = PyErr_NewException("pybullet.error", NULL, NULL);
	Py_INCREF(SpamError);
	PyModule_AddObject(m, "error", SpamError);
	fprintf(stderr, "pybullet build time: %s %s\n", __DATE__, __TIME__);

	Py_AtExit(b3pybulletExitFunc);

#ifdef PYBULLET_USE_NUMPY
	// Initialize numpy array.
	import_array();
#endif  //PYBULLET_USE_NUMPY

#if PY_MAJOR_VERSION >= 3
	return m;
#endif
}
