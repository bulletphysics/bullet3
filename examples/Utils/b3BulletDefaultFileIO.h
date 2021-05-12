
#ifndef B3_BULLET_DEFAULT_FILE_IO_H
#define B3_BULLET_DEFAULT_FILE_IO_H

#include "../CommonInterfaces/CommonFileIOInterface.h"
#include "b3ResourcePath.h"

#include <stdio.h>
#include <string.h>

#define B3_FILEIO_MAX_FILES 1024

struct b3BulletDefaultFileIO : public CommonFileIOInterface
{
	static bool FileIOPluginFindFile(void* userPtr, const char* orgFileName, char* relativeFileName, int maxRelativeFileNameMaxLen)
	{
		b3BulletDefaultFileIO* fileIo = (b3BulletDefaultFileIO*) userPtr;
		return fileIo->findFile(orgFileName, relativeFileName, maxRelativeFileNameMaxLen);
	}

	char m_prefix[1024];
	FILE* m_fileHandles[B3_FILEIO_MAX_FILES];
	int m_numFileHandles;

	b3BulletDefaultFileIO(int fileIOType=0, const char* pathPrefix=0)
		:CommonFileIOInterface(fileIOType, m_prefix),
		m_numFileHandles(0)
	{
		m_prefix[0] = 0;
		if (pathPrefix)
		{
			sprintf(m_prefix,"%s", pathPrefix);
		}
		for (int i=0;i<B3_FILEIO_MAX_FILES;i++)
		{
			m_fileHandles[i]=0;
		}
	}

	virtual ~b3BulletDefaultFileIO()
	{
	}
	virtual int fileOpen(const char* fileName, const char* mode)
	{
		//search a free slot
		int slot = -1;
		for (int i=0;i<B3_FILEIO_MAX_FILES;i++)
		{
			if (m_fileHandles[i]==0)
			{
				slot=i;
				break;
			}
		}
		if (slot>=0)
		{
			FILE*f = ::fopen(fileName, mode);
			if (f)
			{
				m_fileHandles[slot]=f;
			} else
			{
				slot=-1;
			}
		}
		return slot;
	}
	virtual int fileRead(int fileHandle, char* destBuffer, int numBytes)
	{
		if (fileHandle>=0 && fileHandle < B3_FILEIO_MAX_FILES)
		{
			FILE* f = m_fileHandles[fileHandle];
			if (f)
			{
				int readBytes = ::fread(destBuffer, 1, numBytes, f);
				return readBytes;
			}
		}
		return -1;
			
	}
	virtual int fileWrite(int fileHandle,const char* sourceBuffer, int numBytes)
	{
		if (fileHandle>=0 && fileHandle < B3_FILEIO_MAX_FILES)
		{
			FILE* f = m_fileHandles[fileHandle];
			if (f)
			{
				return ::fwrite(sourceBuffer, 1, numBytes,m_fileHandles[fileHandle]);
			}
		}
		return -1;
	}
	virtual void fileClose(int fileHandle)
	{
		if (fileHandle>=0 && fileHandle < B3_FILEIO_MAX_FILES)
		{
			FILE* f = m_fileHandles[fileHandle];
			if (f)
			{
				::fclose(f);
				m_fileHandles[fileHandle]=0;
			}
		}
	}

	virtual bool findResourcePath(const char* fileName, char* relativeFileName, int relativeFileNameSizeInBytes)
	{
		return b3ResourcePath::findResourcePath(fileName, relativeFileName, relativeFileNameSizeInBytes, b3BulletDefaultFileIO::FileIOPluginFindFile, this)>0;
	}


	virtual bool findFile(const char* orgFileName, char* relativeFileName, int maxRelativeFileNameMaxLen)
	{
		FILE* f = 0;
		f = fopen(orgFileName, "rb");
		if (f)
		{
			//printf("original file found: [%s]\n", orgFileName);
			sprintf(relativeFileName, "%s", orgFileName);
			fclose(f);
			return true;
		}

		//printf("Trying various directories, relative to current working directory\n");
		const char* prefix[] = {m_prefix, "./", "./data/", "../data/", "../../data/", "../../../data/", "../../../../data/"};
		int numPrefixes = sizeof(prefix) / sizeof(const char*);

		f = 0;
		bool fileFound = false;

		for (int i = 0; !f && i < numPrefixes; i++)
		{
#ifdef _MSC_VER
			sprintf_s(relativeFileName, maxRelativeFileNameMaxLen, "%s%s", prefix[i], orgFileName);
#else
			sprintf(relativeFileName, "%s%s", prefix[i], orgFileName);
#endif
			f = fopen(relativeFileName, "rb");
			if (f)
			{
				fileFound = true;
				break;
			}
		}
		if (f)
		{
			fclose(f);
		}

		return fileFound;
	}
	virtual char* readLine(int fileHandle, char* destBuffer, int numBytes)
	{
		if (fileHandle>=0 && fileHandle < B3_FILEIO_MAX_FILES)
		{
			FILE* f = m_fileHandles[fileHandle];
			if (f)
			{
                                memset(destBuffer, 0, numBytes);
				char* txt = ::fgets(destBuffer, numBytes, m_fileHandles[fileHandle]);
				for (int i=0;i<numBytes;i++)
				{
					if (destBuffer[i]=='\r'||destBuffer[i]=='\n' || destBuffer[i]==0)
					{
						destBuffer[i] = 0;
						break;
					}
				}
				return txt;
			}
		}
		return 0;
	}
	virtual int getFileSize(int fileHandle)
	{
		int size = 0;
		if (fileHandle>=0 && fileHandle < B3_FILEIO_MAX_FILES)
		{
			FILE* f = m_fileHandles[fileHandle];
			if (f)
			{
				
				if (fseek(f, 0, SEEK_END) || (size = ftell(f)) == EOF || fseek(f, 0, SEEK_SET))
				{
					printf("Error: Cannot access file to determine size\n");
				}
			}
		}
		return size;
	}

	virtual void enableFileCaching(bool enable)
	{
		(void) enable;
	}
};

#endif //B3_BULLET_DEFAULT_FILE_IO_H
