/*
Copyright (c) 2012 Advanced Micro Devices, Inc.  

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Takahiro Harada

#ifdef _WIN32
	#include <windows.h>
#else
	#include <sys/time.h>
#endif

namespace adl
{

class StopwatchHost : public StopwatchBase
{
	public:
		__inline
		StopwatchHost();
		__inline
		void init( const Device* deviceData );
		__inline
		void start();
		__inline
		void split();
		__inline
		void stop();
		__inline
		float getMs(int index=0);
		__inline
		void getMs( float* times, int capacity );

	private:
#ifdef _WIN32
		LARGE_INTEGER m_frequency;
		LARGE_INTEGER m_t[CAPACITY];
#else
		struct timeval mStartTime;
		timeval m_t[CAPACITY];
#endif
};

__inline
StopwatchHost::StopwatchHost()
 : StopwatchBase()
{
}

__inline
void StopwatchHost::init( const Device* deviceData )
{
	m_device = deviceData;
#ifdef _WIN32
	QueryPerformanceFrequency( &m_frequency );
#else
	gettimeofday(&mStartTime, 0);
#endif
}

__inline
void StopwatchHost::start()
{
	m_idx = 0;
#ifdef _WIN32
	QueryPerformanceCounter(&m_t[m_idx++]);
#else
	gettimeofday(&m_t[m_idx++], 0);
#endif
}

__inline
void StopwatchHost::split()
{
#ifdef _WIN32
	QueryPerformanceCounter(&m_t[m_idx++]);
#else
	gettimeofday(&m_t[m_idx++], 0);
#endif
}

__inline
void StopwatchHost::stop()
{
	split();
}

__inline
float StopwatchHost::getMs(int index)
{
#ifdef _WIN32
	return (float)(1000*(m_t[index+1].QuadPart - m_t[index].QuadPart))/m_frequency.QuadPart;
#else
		return (m_t[index+1].tv_sec - m_t[index].tv_sec) * 1000 + 
			(m_t[index+1].tv_usec - m_t[index].tv_usec) / 1000;
#endif
}

__inline
void StopwatchHost::getMs(float* times, int capacity)
{
	for(int i=0; i<capacity; i++) times[i] = 0.f;

	for(int i=0; i<min(capacity, m_idx-1); i++)
	{
		times[i] = getMs(i);
	}
}

};