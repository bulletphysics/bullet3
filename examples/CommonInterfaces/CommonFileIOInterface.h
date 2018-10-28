#ifndef COMMON_FILE_IO_INTERFACE_H
#define COMMON_FILE_IO_INTERFACE_H

struct CommonFileIOInterface
{
	virtual ~CommonFileIOInterface()
	{
	}
	virtual int fileOpen(const char* fileName, const char* mode)=0;
	virtual int fileRead(int fileHandle, char* destBuffer, int numBytes)=0;
	virtual int fileWrite(int fileHandle,const char* sourceBuffer, int numBytes)=0;
	virtual void fileClose(int fileHandle)=0;
	virtual bool findResourcePath(const char* fileName,  char* resourcePathOut, int resourcePathMaxNumBytes)=0;
	virtual char* readLine(int fileHandle, char* destBuffer, int numBytes)=0;
	virtual int getFileSize(int fileHandle)=0;

};

#endif //COMMON_FILE_IO_INTERFACE_H