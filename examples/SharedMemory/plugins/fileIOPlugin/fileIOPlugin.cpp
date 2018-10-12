

#include "fileIOPlugin.h"
#include "../../SharedMemoryPublic.h"
#include "../b3PluginContext.h"
#include <stdio.h>
#include "../../../CommonInterfaces/CommonFileIOInterface.h"
#include "../../../Utils/b3ResourcePath.h"


#ifndef B3_EXCLUDE_DEFAULT_FILEIO
#include "../../../Utils/b3BulletDefaultFileIO.h"
#endif //B3_EXCLUDE_DEFAULT_FILEIO


#ifdef B3_USE_ZIPFILE_FILEIO
#include "zipFileIO.h"
#endif //B3_USE_ZIPFILE_FILEIO


#ifdef B3_USE_CNS_FILEIO
#include "CNSFileIO.h"
#endif //B3_USE_CNS_FILEIO

#define B3_MAX_FILEIO_INTERFACES 1024

struct WrapperFileHandle
{
	CommonFileIOInterface* childFileIO;
	int m_childFileHandle;
};

struct WrapperFileIO : public CommonFileIOInterface
{
	CommonFileIOInterface* m_availableFileIOInterfaces[B3_MAX_FILEIO_INTERFACES];
	int m_numWrapperInterfaces;

	WrapperFileHandle m_wrapperFileHandles[B3_MAX_FILEIO_INTERFACES];
	

	WrapperFileIO()
		:m_numWrapperInterfaces(0)
	{
		for (int i=0;i<B3_MAX_FILEIO_INTERFACES;i++)
		{
			m_availableFileIOInterfaces[i]=0;
			m_wrapperFileHandles[i].childFileIO=0;
			m_wrapperFileHandles[i].m_childFileHandle=0;
		}
	}

	virtual ~WrapperFileIO()
	{
		for (int i=0;i<B3_MAX_FILEIO_INTERFACES;i++)
		{
			removeFileIOInterface(i);
		}
	}

	int addFileIOInterface(CommonFileIOInterface* fileIO)
	{
		int result = -1;
		for (int i=0;i<B3_MAX_FILEIO_INTERFACES;i++)
		{
			if (m_availableFileIOInterfaces[i]==0)
			{
				m_availableFileIOInterfaces[i]=fileIO;
				result = i;
				break;
			}
		}
		return result;
	}

	void removeFileIOInterface(int fileIOIndex)
	{
		if (fileIOIndex>=0 && fileIOIndex<B3_MAX_FILEIO_INTERFACES)
		{
			if (m_availableFileIOInterfaces[fileIOIndex])
			{
				delete m_availableFileIOInterfaces[fileIOIndex];
				m_availableFileIOInterfaces[fileIOIndex]=0;
			}
		}
	}

	virtual int fileOpen(const char* fileName, const char* mode)
	{
		//find an available wrapperFileHandle slot
		int wrapperFileHandle=-1;
		int slot = -1;
		for (int i=0;i<B3_MAX_FILEIO_INTERFACES;i++)
		{
			if (m_wrapperFileHandles[i].childFileIO==0)
			{
				slot=i;
				break;
			}
		}
		if (slot>=0)
		{
			//figure out what wrapper interface to use
			//use the first one that can open the file
			for (int i=0;i<B3_MAX_FILEIO_INTERFACES;i++)
			{
				CommonFileIOInterface* childFileIO=m_availableFileIOInterfaces[i];
				if (childFileIO)
				{
					int childHandle = childFileIO->fileOpen(fileName, mode);
					if (childHandle>=0)
					{
						wrapperFileHandle = slot;
						m_wrapperFileHandles[slot].childFileIO = childFileIO;
						m_wrapperFileHandles[slot].m_childFileHandle = childHandle;
						break;
					}
				}
			}
		}
		return wrapperFileHandle;
	}

	virtual int fileRead(int fileHandle, char* destBuffer, int numBytes)
	{
		int fileReadResult=-1;
		if (fileHandle>=0 && fileHandle<B3_MAX_FILEIO_INTERFACES)
		{
			if (m_wrapperFileHandles[fileHandle].childFileIO)
			{
				fileReadResult = m_wrapperFileHandles[fileHandle].childFileIO->fileRead(
					m_wrapperFileHandles[fileHandle].m_childFileHandle, destBuffer, numBytes);
			}
		}
		return fileReadResult;
	}

	virtual int fileWrite(int fileHandle,const char* sourceBuffer, int numBytes)
	{
		//todo
		return -1;
	}
	virtual void fileClose(int fileHandle)
	{
		int fileReadResult=-1;
		if (fileHandle>=0 && fileHandle<B3_MAX_FILEIO_INTERFACES)
		{
			if (m_wrapperFileHandles[fileHandle].childFileIO)
			{
				m_wrapperFileHandles[fileHandle].childFileIO->fileClose(
					m_wrapperFileHandles[fileHandle].m_childFileHandle);
				m_wrapperFileHandles[fileHandle].childFileIO = 0;
				m_wrapperFileHandles[fileHandle].m_childFileHandle = -1;
			}
		}
	}
	virtual bool findResourcePath(const char* fileName,  char* resourcePathOut, int resourcePathMaxNumBytes)
	{
		bool found = false;
		for (int i=0;i<B3_MAX_FILEIO_INTERFACES;i++)
		{
			if (m_availableFileIOInterfaces[i])
			{
				found = m_availableFileIOInterfaces[i]->findResourcePath(fileName, resourcePathOut, resourcePathMaxNumBytes);
			}
			if (found)
				break;
		}
		return found;
	}
	virtual char* readLine(int fileHandle, char* destBuffer, int numBytes)
	{
		char* result = 0;

		int fileReadResult=-1;
		if (fileHandle>=0 && fileHandle<B3_MAX_FILEIO_INTERFACES)
		{
			if (m_wrapperFileHandles[fileHandle].childFileIO)
			{
				result = m_wrapperFileHandles[fileHandle].childFileIO->readLine(
					m_wrapperFileHandles[fileHandle].m_childFileHandle,
					destBuffer, numBytes);
			}
		}
		return result;
	}
	virtual int getFileSize(int fileHandle)
	{
		int numBytes = 0;

		int fileReadResult=-1;
		if (fileHandle>=0 && fileHandle<B3_MAX_FILEIO_INTERFACES)
		{
			if (m_wrapperFileHandles[fileHandle].childFileIO)
			{
				numBytes = m_wrapperFileHandles[fileHandle].childFileIO->getFileSize(
					m_wrapperFileHandles[fileHandle].m_childFileHandle);
			}
		}
		return numBytes;
	}

};


struct FileIOClass
{
	int m_testData;

	WrapperFileIO m_fileIO;

	FileIOClass()
		: m_testData(42),
		m_fileIO()
	{
	}
	virtual ~FileIOClass()
	{
	}
};

B3_SHARED_API int initPlugin_fileIOPlugin(struct b3PluginContext* context)
{
	FileIOClass* obj = new FileIOClass();
	context->m_userPointer = obj;
	
#ifndef B3_EXCLUDE_DEFAULT_FILEIO
	obj->m_fileIO.addFileIOInterface(new b3BulletDefaultFileIO());
#endif //B3_EXCLUDE_DEFAULT_FILEIO
	

	return SHARED_MEMORY_MAGIC_NUMBER;
}


B3_SHARED_API int executePluginCommand_fileIOPlugin(struct b3PluginContext* context, const struct b3PluginArguments* arguments)
{
	int result=-1;

	FileIOClass* obj = (FileIOClass*)context->m_userPointer;

	printf("text argument:%s\n", arguments->m_text);
	printf("int args: [");
	
	//remove a fileIO type
	if (arguments->m_numInts==1)
	{
		int fileIOIndex = arguments->m_ints[0];
		obj->m_fileIO.removeFileIOInterface(fileIOIndex);
	}

	if (arguments->m_numInts==2)
	{
		int action = arguments->m_ints[0];
		switch (action)
		{
			case eAddFileIOAction:
			{
				//create new fileIO interface
				int fileIOType = arguments->m_ints[1];
				switch (fileIOType)
				{
					case ePosixFileIO:
					{
#ifdef B3_EXCLUDE_DEFAULT_FILEIO
						printf("ePosixFileIO is not enabled in this build.\n");
#else
						obj->m_fileIO.addFileIOInterface(new b3BulletDefaultFileIO());
#endif
						break;
					}
					case eZipFileIO:
					{
#ifdef B3_USE_ZIPFILE_FILEIO
						if (arguments->m_text)
						{
							obj->m_fileIO.addFileIOInterface(new ZipFileIO(arguments->m_text));
						}
#else
						printf("eZipFileIO is not enabled in this build.\n");
#endif
						break;
					}
					case eCNSFileIO:
					{
#ifdef B3_USE_CNS_FILEIO
						B3_USE_ZIPFILE_FILEIO
						if (arguments->m_text)
						{
							obj->m_fileIO.addFileIOInterface(new CNSFileIO(arguments->m_text));
						}
#else//B3_USE_CNS_FILEIO
						printf("CNSFileIO is not enabled in this build.\n");
#endif //B3_USE_CNS_FILEIO
						break;
					}
					default:
					{
					}
				}
				break;
			}
			case eRemoveFileIOAction:

			{
				//remove fileIO interface
				int fileIOIndex = arguments->m_ints[1];
				obj->m_fileIO.removeFileIOInterface(fileIOIndex);
				break;
			}
			default:
			{
				printf("executePluginCommand_fileIOPlugin: unknown action\n");
			}
		}
	}
	return result;
}

B3_SHARED_API struct CommonFileIOInterface* getFileIOFunc_fileIOPlugin(struct b3PluginContext* context)
{
	FileIOClass* obj = (FileIOClass*)context->m_userPointer;
	return &obj->m_fileIO;
}

B3_SHARED_API void exitPlugin_fileIOPlugin(struct b3PluginContext* context)
{
	FileIOClass* obj = (FileIOClass*)context->m_userPointer;
	delete obj;
	context->m_userPointer = 0;
}
