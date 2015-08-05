#ifdef _WIN32
#include "Win32SharedMemory.h"
#include "Bullet3Common/b3Logging.h"
#include "Bullet3Common/b3Scalar.h"

#include <windows.h>

//see also https://msdn.microsoft.com/en-us/library/windows/desktop/aa366551%28v=vs.85%29.aspx

//TCHAR szName[]=TEXT("Global\\MyFileMappingObject2");
TCHAR szName[]=TEXT("MyFileMappingObject2");

struct Win32SharedMemoryInteralData
{
	HANDLE m_hMapFile;

	void*	m_buf;

	Win32SharedMemoryInteralData()
		:m_hMapFile(0),
		m_buf(0)
	{
	}
};

Win32SharedMemory::Win32SharedMemory()
{
	m_internalData = new Win32SharedMemoryInteralData;
}
Win32SharedMemory::~Win32SharedMemory()
{
	delete m_internalData;
}

void*   Win32SharedMemory::allocateSharedMemory(int key, int size, bool allowCreation)
{
	b3Assert(m_internalData->m_buf==0);

	m_internalData->m_hMapFile = OpenFileMapping(
                   FILE_MAP_ALL_ACCESS,   // read/write access
                   FALSE,                 // do not inherit the name
                   szName);               // name of mapping object

	if (m_internalData->m_hMapFile==NULL)
	{
		 if (allowCreation)
		 {
			  m_internalData->m_hMapFile = CreateFileMapping(
                 INVALID_HANDLE_VALUE,    // use paging file
                 NULL,                    // default security
                 PAGE_READWRITE,          // read/write access
                 0,                       // maximum object size (high-order DWORD)
                 size,						// maximum object size (low-order DWORD)
                 szName);                 // name of mapping object
		 } else
		 {
			   b3Error("Could not create file mapping object (%d).\n",GetLastError());
			 return 0;
		 }

	}

   m_internalData->m_buf =  MapViewOfFile(m_internalData->m_hMapFile,   // handle to map object
                        FILE_MAP_ALL_ACCESS, // read/write permission
                        0,
                        0,
                        size);

   if (m_internalData->m_buf == NULL)
	{
		b3Error("Could not map view of file (%d).\n",GetLastError());
		CloseHandle(m_internalData->m_hMapFile);
		return 0;
   }

   return m_internalData->m_buf;
}
void Win32SharedMemory::releaseSharedMemory(int key, int size)
{

	if (m_internalData->m_buf)
	{

		UnmapViewOfFile(m_internalData->m_buf);
		m_internalData->m_buf=0;
	}

	if (m_internalData->m_hMapFile)
	{
		CloseHandle(m_internalData->m_hMapFile);
		m_internalData->m_hMapFile = 0;
	}

}

Win32SharedMemoryServer::Win32SharedMemoryServer()
{
}
Win32SharedMemoryServer::~Win32SharedMemoryServer()
{
}

Win32SharedMemoryClient::Win32SharedMemoryClient()
{
}
Win32SharedMemoryClient:: ~Win32SharedMemoryClient()
{
}

#endif //_WIN32
