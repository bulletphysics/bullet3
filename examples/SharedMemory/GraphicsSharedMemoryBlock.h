#ifndef GRAPHICS_SHARED_MEMORY_BLOCK_H
#define GRAPHICS_SHARED_MEMORY_BLOCK_H

#define GRAPHICS_SHARED_MEMORY_MAX_COMMANDS 1

#include "GraphicsSharedMemoryCommands.h"

struct GraphicsSharedMemoryBlock
{
	int m_magicId;
	struct GraphicsSharedMemoryCommand m_clientCommands[GRAPHICS_SHARED_MEMORY_MAX_COMMANDS];
	struct GraphicsSharedMemoryStatus m_serverCommands[GRAPHICS_SHARED_MEMORY_MAX_COMMANDS];

	int m_numClientCommands;
	int m_numProcessedClientCommands;

	int m_numServerCommands;
	int m_numProcessedServerCommands;

	char m_bulletStreamData[GRAPHICS_SHARED_MEMORY_MAX_STREAM_CHUNK_SIZE];
};

//http://stackoverflow.com/questions/24736304/unable-to-use-inline-in-declaration-get-error-c2054
#ifdef _WIN32
__inline
#else
inline
#endif
	void
	InitSharedMemoryBlock(struct GraphicsSharedMemoryBlock* sharedMemoryBlock)
{
	sharedMemoryBlock->m_numClientCommands = 0;
	sharedMemoryBlock->m_numServerCommands = 0;
	sharedMemoryBlock->m_numProcessedClientCommands = 0;
	sharedMemoryBlock->m_numProcessedServerCommands = 0;
	sharedMemoryBlock->m_magicId = GRAPHICS_SHARED_MEMORY_MAGIC_NUMBER;
}

#define GRAPHICS_SHARED_MEMORY_SIZE sizeof(GraphicsSharedMemoryBlock)

#endif  //GRAPHICS_SHARED_MEMORY_BLOCK_H
