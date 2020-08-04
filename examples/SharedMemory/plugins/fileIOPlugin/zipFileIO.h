
#include "minizip/unzip.h"

#define B3_ZIP_FILEIO_MAX_FILES   1024

struct ZipFileIO : public CommonFileIOInterface
{
	std::string m_zipfileName;

	unzFile	m_fileHandles[B3_ZIP_FILEIO_MAX_FILES ];
	int m_numFileHandles;
	unzFile m_zipfile;
	voidpf m_stream;
	unz_global_info m_global_info;
	bool m_memoryFile;
	b3AlignedObjectArray<char> m_buffer;
	
	ZipFileIO(int fileIOType, const char* zipfileName, CommonFileIOInterface* wrapperFileIO)
		:CommonFileIOInterface(fileIOType,0),
		m_zipfileName(zipfileName),
		m_numFileHandles(0),
		m_stream(0),
		m_memoryFile(false)
	{
		m_pathPrefix = m_zipfileName.c_str();
		for (int i=0;i<B3_ZIP_FILEIO_MAX_FILES ;i++)
		{
			m_fileHandles[i]=0;
		}
		m_zipfile = unzOpen(m_zipfileName.c_str());

		if (m_zipfile == 0)
		{
			int fileIO = wrapperFileIO->fileOpen(m_zipfileName.c_str(), "rb");
			if (fileIO >= 0)
			{
				int stream_size = wrapperFileIO->getFileSize(fileIO);
				m_buffer.resize(stream_size);
				int read_bytes = wrapperFileIO->fileRead(fileIO, &m_buffer[0], stream_size);
				b3Assert(read_bytes == stream_size);
				if (read_bytes != stream_size)
				{
					printf("Error: mismatch reading file %s, expected %d bytes, read %d\n", m_zipfileName.c_str(), stream_size, read_bytes);
				}
				zlib_filefunc_def api;  // callbacks for in-mem file
				
				m_stream = mem_simple_create_file(&api, &m_buffer[0], stream_size);

				m_zipfile = unzAttach(m_stream, &api);
				m_memoryFile = true;
				wrapperFileIO->fileClose(fileIO);
			}
		}
	}

	
		
	static bool FileIOPluginFindFile(void* userPtr, const char* orgFileName, char* relativeFileName, int maxRelativeFileNameMaxLen)
	{
		ZipFileIO* fileIo = (ZipFileIO*) userPtr;
		return fileIo->findFile(orgFileName, relativeFileName, maxRelativeFileNameMaxLen);
	}

	void closeZipFile()
	{
		if (m_zipfile)
		{
			if (m_memoryFile)
			{
				unzDetach(&m_zipfile);
			}
			else
			{
				unzClose(m_zipfile);
			}
		}
		m_zipfile = 0;
	}

	virtual ~ZipFileIO()
	{
		for (int i=0;i<B3_ZIP_FILEIO_MAX_FILES;i++)
		{
			fileClose(i);
		}
		closeZipFile();
		if (m_stream)
		{
			mem_simple_destroy_file(m_stream);
		}
	}

	
	virtual int fileOpen(const char* fileName, const char* mode)
	{
		
		//search a free slot
		int slot = -1;
		for (int i=0;i<B3_ZIP_FILEIO_MAX_FILES ;i++)
		{
			if (m_fileHandles[i]==0)
			{
				slot=i;
				break;
			}
		}
		if (slot>=0)
		{
			
			if (m_zipfile == NULL)
			{
				printf("%s: not found\n", m_zipfileName.c_str());
				slot = -1;
			} else
			{
				
				int result = 0;
				result = unzGetGlobalInfo(m_zipfile, &m_global_info );
				if (result != UNZ_OK)
				{
					printf("could not read file global info from %s\n", m_zipfileName.c_str());
					slot = -1;
				}
			}
			if (slot >=0)
			{
				int result = unzLocateFile(m_zipfile, fileName, 0);
				if (result == UNZ_OK)
				{
					unz_file_info info;
					result = unzGetCurrentFileInfo(m_zipfile, &info, NULL, 0, NULL, 0, NULL, 0);
					if (result != UNZ_OK)
					{
						printf("unzGetCurrentFileInfo() != UNZ_OK (%d)\n", result);
						slot=-1;
					}
					else
					{
						result = unzOpenCurrentFile(m_zipfile);
						if (result == UNZ_OK)
						{
							printf("zipFile::fileOpen %s in mode %s in fileHandle %d\n", fileName, mode, slot);
							m_fileHandles[slot] = m_zipfile;
						} else
						{
							slot=-1;
						}
					}
				} else
				{
					slot=-1;
					
				}
			}
		}
		return slot;
	}
	virtual int fileRead(int fileHandle, char* destBuffer, int numBytes)
	{
		int result = -1;
		if (fileHandle>=0 && fileHandle < B3_ZIP_FILEIO_MAX_FILES )
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
		if (fileHandle>=0 && fileHandle < B3_ZIP_FILEIO_MAX_FILES )
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
		if (fileHandle>=0 && fileHandle < B3_ZIP_FILEIO_MAX_FILES )
		{
			unzFile f = m_fileHandles[fileHandle];
			if (f)
			{
				printf("zipFile::fileClose slot %d\n", fileHandle);
				m_fileHandles[fileHandle]=0;
			}
		}
	}

	virtual bool findResourcePath(const char* fileName, char* relativeFileName, int relativeFileNameSizeInBytes)
	{
		return b3ResourcePath::findResourcePath(fileName, relativeFileName, relativeFileNameSizeInBytes, ZipFileIO::FileIOPluginFindFile, this);
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
				
		if (fileHandle>=0 && fileHandle < B3_ZIP_FILEIO_MAX_FILES )
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

		if (fileHandle>=0 && fileHandle < B3_ZIP_FILEIO_MAX_FILES )
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

	virtual void enableFileCaching(bool enable)
	{
		(void)enable;
	}

};
