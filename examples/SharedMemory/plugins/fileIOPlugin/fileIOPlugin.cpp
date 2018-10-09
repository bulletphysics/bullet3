

#include "fileIOPlugin.h"
#include "../../SharedMemoryPublic.h"
#include "../b3PluginContext.h"
#include <stdio.h>
#include "../../../CommonInterfaces/CommonFileIOInterface.h"
#include "../../../Utils/b3ResourcePath.h"
#include "../../../Utils/b3BulletDefaultFileIO.h"


//#define B3_USE_ZLIB
#ifdef B3_USE_ZLIB

#include "minizip/unzip.h"

struct MyFileIO : public CommonFileIOInterface
{
	std::string m_zipfileName;

	unzFile	m_fileHandles[FILEIO_MAX_FILES];
	int m_numFileHandles;

	MyFileIO(const char* zipfileName)
		:m_zipfileName(zipfileName),
		m_numFileHandles(0)
	{
		for (int i=0;i<FILEIO_MAX_FILES;i++)
		{
			m_fileHandles[i]=0;
		}
	}

		
	static bool FileIOPluginFindFile(void* userPtr, const char* orgFileName, char* relativeFileName, int maxRelativeFileNameMaxLen)
	{
		MyFileIO* fileIo = (MyFileIO*) userPtr;
		return fileIo->findFile(orgFileName, relativeFileName, maxRelativeFileNameMaxLen);
	}

	virtual ~MyFileIO()
	{
	}
	virtual int fileOpen(const char* fileName, const char* mode)
	{
		//search a free slot
		int slot = -1;
		for (int i=0;i<FILEIO_MAX_FILES;i++)
		{
			if (m_fileHandles[i]==0)
			{
				slot=i;
				break;
			}
		}
		if (slot>=0)
		{
			unzFile zipfile;
			unz_global_info m_global_info;
			zipfile = unzOpen(m_zipfileName.c_str());
			if (zipfile == NULL)
			{
				printf("%s: not found\n", m_zipfileName.c_str());
				slot = -1;
			} else
			{
				int result = 0;
				result = unzGetGlobalInfo(zipfile, &m_global_info );
				if (result != UNZ_OK)
				{
					printf("could not read file global info from %s\n", m_zipfileName.c_str());
					unzClose(zipfile);
					zipfile = 0;
					slot = -1;
				} else
				{
					m_fileHandles[slot] = zipfile;
				}
			}
			if (slot >=0)
			{
				int result = unzLocateFile(zipfile, fileName, 0);
				if (result == UNZ_OK)
				{
					unz_file_info info;
					result = unzGetCurrentFileInfo(zipfile, &info, NULL, 0, NULL, 0, NULL, 0);
					if (result != UNZ_OK)
					{
						printf("unzGetCurrentFileInfo() != UNZ_OK (%d)\n", result);
						slot=-1;
					}
					else
					{
						result = unzOpenCurrentFile(zipfile);
						if (result == UNZ_OK)
						{
						} else
						{
							slot=-1;
						}
					}
				}
			}
		}
		return slot;
	}
	virtual int fileRead(int fileHandle, char* destBuffer, int numBytes)
	{
		int result = -1;
		if (fileHandle>=0 && fileHandle < FILEIO_MAX_FILES)
		{
			unzFile f = m_fileHandles[fileHandle];
			if (f)
			{
				result = unzReadCurrentFile(f, destBuffer,numBytes);
				//::fread(destBuffer, 1, numBytes, f);
			}
		}
		return result;
			
	}
	virtual int fileWrite(int fileHandle,const char* sourceBuffer, int numBytes)
	{
#if 0
		if (fileHandle>=0 && fileHandle < FILEIO_MAX_FILES)
		{
			FILE* f = m_fileHandles[fileHandle];
			if (f)
			{
				return ::fwrite(sourceBuffer, 1, numBytes,m_fileHandles[fileHandle]);
			}
		}
#endif
		return -1;
	}
	virtual void fileClose(int fileHandle)
	{
		if (fileHandle>=0 && fileHandle < FILEIO_MAX_FILES)
		{
			unzFile f = m_fileHandles[fileHandle];
			if (f)
			{
				unzClose(f);
				m_fileHandles[fileHandle]=0;
			}
		}
	}

	virtual bool findResourcePath(const char* fileName, char* relativeFileName, int relativeFileNameSizeInBytes)
	{
		return b3ResourcePath::findResourcePath(fileName, relativeFileName, relativeFileNameSizeInBytes, MyFileIO::FileIOPluginFindFile, this);
	}


	virtual bool findFile(const char* orgFileName, char* relativeFileName, int maxRelativeFileNameMaxLen)
	{
		int fileHandle = -1;
		fileHandle = fileOpen(orgFileName, "rb");
		if (fileHandle>=0)
		{
			//printf("original file found: [%s]\n", orgFileName);
			sprintf(relativeFileName, "%s", orgFileName);
			fileClose(fileHandle);
			return true;
		}

		//printf("Trying various directories, relative to current working directory\n");
		const char* prefix[] = {"./", "./data/", "../data/", "../../data/", "../../../data/", "../../../../data/"};
		int numPrefixes = sizeof(prefix) / sizeof(const char*);

		int f = 0;
		bool fileFound = false;

		for (int i = 0; !f && i < numPrefixes; i++)
		{
#ifdef _MSC_VER
			sprintf_s(relativeFileName, maxRelativeFileNameMaxLen, "%s%s", prefix[i], orgFileName);
#else
			sprintf(relativeFileName, "%s%s", prefix[i], orgFileName);
#endif
			f = fileOpen(relativeFileName, "rb");
			if (f>=0)
			{
				fileFound = true;
				break;
			}
		}
		if (f>=0)
		{
			fileClose(f);
		}

		return fileFound;
	}
	virtual char* readLine(int fileHandle, char* destBuffer, int numBytes)
	{
		int numRead = 0;
				
		if (fileHandle>=0 && fileHandle < FILEIO_MAX_FILES)
		{
			unzFile f = m_fileHandles[fileHandle];
			if (f)
			{
				//return ::fgets(destBuffer, numBytes, m_fileHandles[fileHandle]);
				char c = 0;
				do
				{
					fileRead(fileHandle,&c,1);
					if (c && c!='\n')
					{
						if (c!=13)
						{
							destBuffer[numRead++]=c;
						} else
						{
							destBuffer[numRead++]=0;
						}
					}
				} while (c != 0 && c != '\n' && numRead<(numBytes-1));
			}
		}
		if (numRead<numBytes && numRead>0)
		{
			destBuffer[numRead]=0;
			return &destBuffer[0];
		}
		return 0;
	}
	virtual int getFileSize(int fileHandle)
	{
		int size=0;

		if (fileHandle>=0 && fileHandle < FILEIO_MAX_FILES)
		{
			unzFile f = m_fileHandles[fileHandle];
			if (f)
			{
				unz_file_info info;
				int result = unzGetCurrentFileInfo(f, &info, NULL, 0, NULL, 0, NULL, 0);
				if (result == UNZ_OK)
				{
					size = info.uncompressed_size;
				}
			}
		}
		return size;
	}
	
};



#else
typedef b3BulletDefaultFileIO MyFileIO;
#endif

struct FileIOClass
{
	int m_testData;

	MyFileIO m_fileIO;

	FileIOClass()
		: m_testData(42),
		m_fileIO()//"e:/develop/bullet3/data/plane.zip")
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
	return SHARED_MEMORY_MAGIC_NUMBER;
}


B3_SHARED_API int executePluginCommand_fileIOPlugin(struct b3PluginContext* context, const struct b3PluginArguments* arguments)
{
	printf("text argument:%s\n", arguments->m_text);
	printf("int args: [");
	for (int i = 0; i < arguments->m_numInts; i++)
	{
		printf("%d", arguments->m_ints[i]);
		if ((i + 1) < arguments->m_numInts)
		{
			printf(",");
		}
	}
	printf("]\nfloat args: [");
	for (int i = 0; i < arguments->m_numFloats; i++)
	{
		printf("%f", arguments->m_floats[i]);
		if ((i + 1) < arguments->m_numFloats)
		{
			printf(",");
		}
	}
	printf("]\n");

	FileIOClass* obj = (FileIOClass*)context->m_userPointer;

	b3SharedMemoryStatusHandle statusHandle;
	int statusType = -1;
	int bodyUniqueId = -1;

	b3SharedMemoryCommandHandle command =
		b3LoadUrdfCommandInit(context->m_physClient, arguments->m_text);

	statusHandle = b3SubmitClientCommandAndWaitStatus(context->m_physClient, command);
	statusType = b3GetStatusType(statusHandle);
	if (statusType == CMD_URDF_LOADING_COMPLETED)
	{
		bodyUniqueId = b3GetStatusBodyIndex(statusHandle);
	}
	return bodyUniqueId;
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
