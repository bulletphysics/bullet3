#ifndef GRAPHICS_SHARED_MEMORY_COMMANDS_H
#define GRAPHICS_SHARED_MEMORY_COMMANDS_H

//this is a very experimental draft of commands. We will iterate on this API (commands, arguments etc)

#include "GraphicsSharedMemoryPublic.h"

#ifdef __GNUC__
#include <stdint.h>
typedef int32_t smInt32a_t;
typedef int64_t smInt64a_t;
typedef uint32_t smUint32a_t;
typedef uint64_t smUint64a_t;
#elif defined(_MSC_VER)
typedef __int32 smInt32a_t;
typedef __int64 smInt64a_t;
typedef unsigned __int32 smUint32a_t;
typedef unsigned __int64 smUint64a_t;
#else
typedef int smInt32a_t;
typedef long long int smInt64a_t;
typedef unsigned int smUint32a_t;
typedef unsigned long long int smUint64a_t;
#endif

#ifdef __APPLE__
#define GRAPHICS_SHARED_MEMORY_MAX_STREAM_CHUNK_SIZE (512 * 1024)
#else
 #define GRAPHICS_SHARED_MEMORY_MAX_STREAM_CHUNK_SIZE (4 * 1024 * 1024)
#endif

struct GraphicsCommand0
{
	int bla;
};

struct GraphicsUpAxisCommand
{
	int m_enableUpAxisY;
};

struct GraphicsStatus0
{
	int bla;
};

struct GraphicsVisualizerFlagCommand
{
	int m_visualizerFlag;
	int m_enable;
};

struct GraphicsUploadDataCommand
{
	int m_numBytes;
	int m_dataOffset;
	int m_dataSlot;
};

struct GraphicRegisterTextureCommand
{
	int m_width;
	int m_height;
};

struct GraphicsRegisterTextureStatus
{
	int m_textureId;
};

struct GraphicsRegisterGraphicsShapeCommand
{
	int m_numVertices;
	int m_numIndices;
	int m_primitiveType;
	int m_textureId;
};

struct GraphicsRegisterGraphicsShapeStatus
{
	int m_shapeId;
};

struct GraphicsRegisterGraphicsInstanceCommand
{
	
	int m_shapeIndex;
	float m_position[4];
	float m_quaternion[4];
	float m_color[4];
	float m_scaling[4];
};

struct GraphicsRegisterGraphicsInstanceStatus
{
	int m_graphicsInstanceId;
};

struct GraphicsSyncTransformsCommand
{
	int m_numPositions;
};

struct GraphicsRemoveInstanceCommand
{
	int m_graphicsUid;
};

struct GraphicsChangeRGBAColorCommand
{
	int m_graphicsUid;
	double m_rgbaColor[4];
};

struct GraphicsChangeScalingCommand
{
	int m_graphicsUid;
	double m_scaling[3];
};



struct GraphicsGetCameraInfoStatus
{
	int width;
	int height;
	float viewMatrix[16];
	float projectionMatrix[16];
	float camUp[3];
	float camForward[3];
	float hor[3];
	float vert[3];
	float yaw;
	float pitch;
	float camDist;
	float camTarget[3];
};


struct GraphicsSharedMemoryCommand
{
	int m_type;
	smUint64a_t m_timeStamp;
	int m_sequenceNumber;

	//m_updateFlags is a bit fields to tell which parameters need updating
	//for example m_updateFlags = SIM_PARAM_UPDATE_DELTA_TIME | SIM_PARAM_UPDATE_NUM_SOLVER_ITERATIONS;
	int m_updateFlags;

	union {
		struct GraphicsCommand0 m_graphicsCommand0;
		struct GraphicsUpAxisCommand m_upAxisYCommand;
		struct GraphicsVisualizerFlagCommand m_visualizerFlagCommand;
		struct GraphicsUploadDataCommand m_uploadDataCommand;
		struct GraphicRegisterTextureCommand m_registerTextureCommand;
		struct GraphicsRegisterGraphicsShapeCommand m_registerGraphicsShapeCommand;
		struct GraphicsRegisterGraphicsInstanceCommand m_registerGraphicsInstanceCommand;
		struct GraphicsSyncTransformsCommand m_syncTransformsCommand;
		struct GraphicsRemoveInstanceCommand m_removeGraphicsInstanceCommand;
		struct GraphicsChangeRGBAColorCommand m_changeRGBAColorCommand;
		struct GraphicsChangeScalingCommand m_changeScalingCommand;
	};
};

struct GraphicsSharedMemoryStatus
{
	int m_type;

	smUint64a_t m_timeStamp;
	int m_sequenceNumber;

	//m_streamBytes is only for internal purposes
	int m_numDataStreamBytes;
	char* m_dataStream;

	//m_updateFlags is a bit fields to tell which parameters were updated,
	//m_updateFlags is ignored for most status messages
	int m_updateFlags;

	union {
		
		struct GraphicsStatus0 m_graphicsStatus0;
		struct GraphicsRegisterTextureStatus m_registerTextureStatus;
		struct GraphicsRegisterGraphicsShapeStatus m_registerGraphicsShapeStatus;
		struct GraphicsRegisterGraphicsInstanceStatus m_registerGraphicsInstanceStatus;
		struct GraphicsGetCameraInfoStatus m_getCameraInfoStatus;
	};
};

typedef struct GraphicsSharedMemoryStatus GraphicsSharedMemoryStatus_t;

#endif  //GRAPHICS_SHARED_MEMORY_COMMANDS_H
