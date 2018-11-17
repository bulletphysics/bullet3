#include "fileIOPlugin.h"
#include "../../SharedMemoryPublic.h"
#include "../b3PluginContext.h"
#include <stdio.h>
#include "../../../CommonInterfaces/CommonFileIOInterface.h"
#include "../../../Utils/b3ResourcePath.h"
#include "Bullet3Common/b3HashMap.h"
#include <string.h> //memcpy/strlen
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

struct InMemoryFile
{
	char* m_buffer;
	int m_fileSize;
};

struct InMemoryFileAccessor
{
	InMemoryFile* m_file;
	int m_curPos;
};

struct InMemoryFileIO : public CommonFileIOInterface
{
	b3HashMap<b3HashString,InMemoryFile*> m_fileCache;
	InMemoryFileAccessor m_fileHandles[B3_MAX_FILEIO_INTERFACES];
	int m_numAllocs;
	int m_numFrees;

	InMemoryFileIO()
		:CommonFileIOInterface(eInMemoryFileIO,0)
	{
		m_numAllocs=0;
		m_numFrees=0;

		for (int i=0;i<B3_FILEIO_MAX_FILES;i++)
		{
			m_fileHandles[i].m_curPos = 0;
			m_fileHandles[i].m_file = 0;
		}
	}

	virtual ~InMemoryFileIO()
	{
		clearCache();
		if (m_numAllocs != m_numFrees)
		{
			printf("Error: InMemoryFile::~InMemoryFileIO (numAllocs %d numFrees %d\n", m_numAllocs, m_numFrees);
		}
	}
	void clearCache()
	{
		for (int i=0;i<m_fileCache.size();i++)
		{
			InMemoryFile** memPtr = m_fileCache.getAtIndex(i);
			if (memPtr && *memPtr)
			{
				InMemoryFile* mem = *memPtr;
				freeBuffer(mem->m_buffer);
				m_numFrees++;
				delete (mem);
				m_numFrees++;
			}
		}
	}

	char* allocateBuffer(int len)
	{
		char* buffer = 0;
		if (len)
		{
			m_numAllocs++;
			buffer = new char[len];
		}
		return buffer;
	}

	void freeBuffer(char* buffer)
	{
		delete[] buffer;
	}

	virtual int registerFile(const char* fileName, char* buffer, int len)
	{
		m_numAllocs++;
		InMemoryFile* f = new InMemoryFile();
		f->m_buffer = buffer;
		f->m_fileSize = len;
		b3HashString key(fileName);
		m_fileCache.insert(key,f);
		return 0;
	}

	void removeFileFromCache(const char* fileName)
	{
		InMemoryFile* f = getInMemoryFile(fileName);
		if (f)
		{
			m_fileCache.remove(fileName);
			freeBuffer(f->m_buffer);
			delete (f);
		}
	}

	InMemoryFile* getInMemoryFile(const char* fileName)
	{
		InMemoryFile** fPtr = m_fileCache[fileName];
		if (fPtr && *fPtr)
		{
			return *fPtr;
		}
		return 0;
	}

	virtual int fileOpen(const char* fileName, const char* mode)
	{
		//search a free slot
		int slot = -1;
		for (int i=0;i<B3_FILEIO_MAX_FILES;i++)
		{
			if (m_fileHandles[i].m_file==0)
			{
				slot=i;
				break;
			}
		}
		if (slot>=0)
		{
			InMemoryFile* f = getInMemoryFile(fileName);
			if (f)
			{
				m_fileHandles[slot].m_curPos = 0;
				m_fileHandles[slot].m_file = f;
			} else
			{
				slot=-1;
			}
		}
		//printf("InMemoryFileIO fileOpen %s, %d\n", fileName, slot);
		return slot;
	}
	virtual int fileRead(int fileHandle, char* destBuffer, int numBytes)
	{
		if (fileHandle>=0 && fileHandle < B3_FILEIO_MAX_FILES)
		{
			InMemoryFileAccessor& f = m_fileHandles[fileHandle];
			if (f.m_file)
			{
				//if (numBytes>1)
				//	printf("curPos = %d\n", f.m_curPos);
				if (f.m_curPos+numBytes <= f.m_file->m_fileSize)
				{
					memcpy(destBuffer,f.m_file->m_buffer+f.m_curPos,numBytes);
					f.m_curPos+=numBytes;
					//if (numBytes>1)
					//	printf("read %d bytes, now curPos = %d\n", numBytes, f.m_curPos);
					return numBytes;
				} else
				{
					if (numBytes!=1)
					{
						printf("InMemoryFileIO::fileRead Attempt to read beyond end of file\n");
					}
				}
				
			}
		}
		return 0;
	}

	virtual int fileWrite(int fileHandle,const char* sourceBuffer, int numBytes)
	{
		return 0;
	}
	virtual void fileClose(int fileHandle)
	{
		if (fileHandle>=0 && fileHandle < B3_FILEIO_MAX_FILES)
		{
			InMemoryFileAccessor& f = m_fileHandles[fileHandle];
			if (f.m_file)
			{
				m_fileHandles[fileHandle].m_file = 0;
				m_fileHandles[fileHandle].m_curPos = 0;
				//printf("InMemoryFileIO fileClose %d\n", fileHandle);
			}
		}
	}
	virtual bool findResourcePath(const char* fileName,  char* resourcePathOut, int resourcePathMaxNumBytes)
	{
		InMemoryFile* f = getInMemoryFile(fileName);
		int fileNameLen = strlen(fileName);
		if (f && fileNameLen<(resourcePathMaxNumBytes-1))
		{
			memcpy(resourcePathOut, fileName, fileNameLen);
			resourcePathOut[fileNameLen]=0;
			return true;
		} 
		return false;
	}
	virtual char* readLine(int fileHandle, char* destBuffer, int numBytes)
	{
		int numRead = 0;
		int endOfFile = 0;
		if (fileHandle>=0 && fileHandle < B3_FILEIO_MAX_FILES )
		{
			InMemoryFileAccessor& f = m_fileHandles[fileHandle];
			if (f.m_file)
			{
				//return ::fgets(destBuffer, numBytes, m_fileHandles[fileHandle]);
				char c = 0;
				do
				{
					int bytesRead = fileRead(fileHandle,&c,1);
					if (bytesRead != 1)
					{
						endOfFile = 1;
						c=0;
					}
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
		if (numRead==0 && endOfFile)
		{
			return 0;
		}

		if (numRead<numBytes)
		{
			if (numRead >=0)
			{
				destBuffer[numRead]=0;
			}
			return &destBuffer[0];
		} else
		{
			if (endOfFile==0)
			{
				printf("InMemoryFileIO::readLine readLine warning: numRead=%d, numBytes=%d\n", numRead, numBytes);
			}
		}
		return 0;
	}
	virtual int getFileSize(int fileHandle)
	{
		if (fileHandle>=0 && fileHandle < B3_FILEIO_MAX_FILES )
		{
			
			InMemoryFileAccessor& f = m_fileHandles[fileHandle];
			if (f.m_file)
			{
				return f.m_file->m_fileSize;
			}
		}
		return 0;
	}

	virtual void enableFileCaching(bool enable)
	{
		(void)enable;
	}
};

struct WrapperFileIO : public CommonFileIOInterface
{
	CommonFileIOInterface* m_availableFileIOInterfaces[B3_MAX_FILEIO_INTERFACES];
	int m_numWrapperInterfaces;

	WrapperFileHandle m_wrapperFileHandles[B3_MAX_FILEIO_INTERFACES];
	InMemoryFileIO m_cachedFiles;
	bool m_enableFileCaching;

	WrapperFileIO()
		:CommonFileIOInterface(0,0),
		m_numWrapperInterfaces(0),
		m_enableFileCaching(true)
	{
		for (int i=0;i<B3_MAX_FILEIO_INTERFACES;i++)
		{
			m_availableFileIOInterfaces[i]=0;
			m_wrapperFileHandles[i].childFileIO=0;
			m_wrapperFileHandles[i].m_childFileHandle=0;
		}
		//addFileIOInterface(&m_cachedFiles);
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

	CommonFileIOInterface* getFileIOInterface(int fileIOIndex)
	{
		if (fileIOIndex>=0 && fileIOIndex<B3_MAX_FILEIO_INTERFACES)
		{
			return m_availableFileIOInterfaces[fileIOIndex];
		}
		return 0;
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
			//first check the cache
			int cacheHandle = m_cachedFiles.fileOpen(fileName, mode);
			if (cacheHandle>=0)
			{
				m_cachedFiles.fileClose(cacheHandle);
			}
			if (cacheHandle<0)
			{
				for (int i=0;i<B3_MAX_FILEIO_INTERFACES;i++)
				{
					CommonFileIOInterface* childFileIO=m_availableFileIOInterfaces[i];
					if (childFileIO)
					{
						int childHandle = childFileIO->fileOpen(fileName, mode);
						if (childHandle>=0)
						{
							int fileSize = childFileIO->getFileSize(childHandle);
							char* buffer = 0;
							if (fileSize)
							{
								buffer = m_cachedFiles.allocateBuffer(fileSize);
								if (buffer)
								{
									int readBytes = childFileIO->fileRead(childHandle, buffer, fileSize);
									if (readBytes!=fileSize)
									{
										if (readBytes<fileSize)
										{
											fileSize = readBytes;
										} else
										{
											printf("WrapperFileIO error: reading more bytes (%d) then reported file size (%d) of file %s.\n", readBytes, fileSize, fileName);
										}
									}
								} else
								{
									fileSize=0;
								}
							}

							//potentially register a zero byte file, or files that only can be read partially
							if (m_enableFileCaching)
							{
								m_cachedFiles.registerFile(fileName, buffer, fileSize);
							}
							childFileIO->fileClose(childHandle);
							break;
						}
					}
				}
			}
			
			{
				int childHandle = m_cachedFiles.fileOpen(fileName, mode);
				if (childHandle>=0)
				{
					wrapperFileHandle = slot;
					m_wrapperFileHandles[slot].childFileIO = &m_cachedFiles;
					m_wrapperFileHandles[slot].m_childFileHandle = childHandle;
				} else
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
		if (m_cachedFiles.findResourcePath(fileName, resourcePathOut, resourcePathMaxNumBytes))
			return true;

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

	virtual void enableFileCaching(bool enable)
	{
		m_enableFileCaching = enable;
		if (!enable)
		{
			m_cachedFiles.clearCache();
		}
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
				//if the fileIO already exists, skip this action
				int fileIOType = arguments->m_ints[1];
				bool alreadyExists = false;

				for (int i=0;i<B3_MAX_FILEIO_INTERFACES;i++)
				{
					CommonFileIOInterface* fileIO = obj->m_fileIO.getFileIOInterface(i);
					if (fileIO)
					{
						if (fileIO->m_fileIOType == fileIOType)
						{
							if (fileIO->m_pathPrefix && strcmp(fileIO->m_pathPrefix,arguments->m_text)==0)
							{
								result = i;
								alreadyExists = true;
								break;
							}
						}
					}
				}
				

				//create new fileIO interface
				if (!alreadyExists)
				{
					switch (fileIOType)
					{
						case ePosixFileIO:
						{
	#ifdef B3_EXCLUDE_DEFAULT_FILEIO
							printf("ePosixFileIO is not enabled in this build.\n");
	#else
							result = obj->m_fileIO.addFileIOInterface(new b3BulletDefaultFileIO(ePosixFileIO, arguments->m_text));
	#endif
							break;
						}
						case eZipFileIO:
						{
	#ifdef B3_USE_ZIPFILE_FILEIO
							if (arguments->m_text)
							{
								result = obj->m_fileIO.addFileIOInterface(new ZipFileIO(eZipFileIO, arguments->m_text, &obj->m_fileIO));
							}
	#else
							printf("eZipFileIO is not enabled in this build.\n");
	#endif
							break;
						}
						case eCNSFileIO:
						{
	#ifdef B3_USE_CNS_FILEIO
							result = obj->m_fileIO.addFileIOInterface(new CNSFileIO(eCNSFileIO, arguments->m_text));
	#else//B3_USE_CNS_FILEIO
							printf("CNSFileIO is not enabled in this build.\n");
	#endif //B3_USE_CNS_FILEIO
							break;
						}
						default:
						{
						}
					}//switch (fileIOType)
				}//if (!alreadyExists)
				break;
			}
			case eRemoveFileIOAction:

			{
				//remove fileIO interface
				int fileIOIndex = arguments->m_ints[1];
				obj->m_fileIO.removeFileIOInterface(fileIOIndex);
				result = fileIOIndex;
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
