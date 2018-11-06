
#include "ChromeTraceUtil.h"
#include "b3Clock.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "Bullet3Common/b3Logging.h"
#include <stdio.h>

struct btTiming
{
	const char* m_name;
	int m_threadId;
	unsigned long long int m_usStartTime;
	unsigned long long int m_usEndTime;
};

FILE* gTimingFile = 0;
#ifndef __STDC_FORMAT_MACROS
#define __STDC_FORMAT_MACROS
#endif  //__STDC_FORMAT_MACROS

//see http://stackoverflow.com/questions/18107426/printf-format-for-unsigned-int64-on-windows
#ifndef _WIN32
#include <inttypes.h>
#endif

#define BT_TIMING_CAPACITY 16 * 65536
static bool m_firstTiming = true;

struct btTimings
{
	btTimings()
		: m_numTimings(0),
		  m_activeBuffer(0)
	{
	}
	void flush()
	{
		for (int i = 0; i < m_numTimings; i++)
		{
			const char* name = m_timings[m_activeBuffer][i].m_name;
			int threadId = m_timings[m_activeBuffer][i].m_threadId;
			unsigned long long int startTime = m_timings[m_activeBuffer][i].m_usStartTime;
			unsigned long long int endTime = m_timings[m_activeBuffer][i].m_usEndTime;

			if (!m_firstTiming)
			{
				fprintf(gTimingFile, ",\n");
			}

			m_firstTiming = false;

			unsigned long long int startTimeDiv1000 = startTime / 1000;
			unsigned long long int endTimeDiv1000 = endTime / 1000;

#if 0

			fprintf(gTimingFile, "{\"cat\":\"timing\",\"pid\":1,\"tid\":%d,\"ts\":%" PRIu64 ".123 ,\"ph\":\"B\",\"name\":\"%s\",\"args\":{}},\n",
				threadId, startTimeDiv1000, name);
			fprintf(gTimingFile, "{\"cat\":\"timing\",\"pid\":1,\"tid\":%d,\"ts\":%" PRIu64 ".234 ,\"ph\":\"E\",\"name\":\"%s\",\"args\":{}}",
				threadId, endTimeDiv1000, name);

#else

			if (startTime > endTime)
			{
				endTime = startTime;
			}
			unsigned int startTimeRem1000 = startTime % 1000;
			unsigned int endTimeRem1000 = endTime % 1000;

			char startTimeRem1000Str[16];
			char endTimeRem1000Str[16];

			if (startTimeRem1000 < 10)
			{
				sprintf(startTimeRem1000Str, "00%d", startTimeRem1000);
			}
			else
			{
				if (startTimeRem1000 < 100)
				{
					sprintf(startTimeRem1000Str, "0%d", startTimeRem1000);
				}
				else
				{
					sprintf(startTimeRem1000Str, "%d", startTimeRem1000);
				}
			}

			if (endTimeRem1000 < 10)
			{
				sprintf(endTimeRem1000Str, "00%d", endTimeRem1000);
			}
			else
			{
				if (endTimeRem1000 < 100)
				{
					sprintf(endTimeRem1000Str, "0%d", endTimeRem1000);
				}
				else
				{
					sprintf(endTimeRem1000Str, "%d", endTimeRem1000);
				}
			}

			char newname[1024];
			static int counter2 = 0;
			sprintf(newname, "%s%d", name, counter2++);

#ifdef _WIN32

			fprintf(gTimingFile, "{\"cat\":\"timing\",\"pid\":1,\"tid\":%d,\"ts\":%I64d.%s ,\"ph\":\"B\",\"name\":\"%s\",\"args\":{}},\n",
					threadId, startTimeDiv1000, startTimeRem1000Str, newname);
			fprintf(gTimingFile, "{\"cat\":\"timing\",\"pid\":1,\"tid\":%d,\"ts\":%I64d.%s ,\"ph\":\"E\",\"name\":\"%s\",\"args\":{}}",
					threadId, endTimeDiv1000, endTimeRem1000Str, newname);

#else
			fprintf(gTimingFile, "{\"cat\":\"timing\",\"pid\":1,\"tid\":%d,\"ts\":%" PRIu64 ".%s ,\"ph\":\"B\",\"name\":\"%s\",\"args\":{}},\n",
					threadId, startTimeDiv1000, startTimeRem1000Str, newname);
			fprintf(gTimingFile, "{\"cat\":\"timing\",\"pid\":1,\"tid\":%d,\"ts\":%" PRIu64 ".%s ,\"ph\":\"E\",\"name\":\"%s\",\"args\":{}}",
					threadId, endTimeDiv1000, endTimeRem1000Str, newname);
#endif
#endif
		}
		m_numTimings = 0;
	}

	void addTiming(const char* name, int threadId, unsigned long long int startTime, unsigned long long int endTime)
	{
		if (m_numTimings >= BT_TIMING_CAPACITY)
		{
			return;
		}

		if (m_timings[0].size() == 0)
		{
			m_timings[0].resize(BT_TIMING_CAPACITY);
		}

		int slot = m_numTimings++;

		m_timings[m_activeBuffer][slot].m_name = name;
		m_timings[m_activeBuffer][slot].m_threadId = threadId;
		m_timings[m_activeBuffer][slot].m_usStartTime = startTime;
		m_timings[m_activeBuffer][slot].m_usEndTime = endTime;
	}

	int m_numTimings;
	int m_activeBuffer;
	btAlignedObjectArray<btTiming> m_timings[1];
};
//#ifndef BT_NO_PROFILE
btTimings gTimings[BT_QUICKPROF_MAX_THREAD_COUNT];
#define MAX_NESTING 1024
int gStackDepths[BT_QUICKPROF_MAX_THREAD_COUNT] = {0};
const char* gFuncNames[BT_QUICKPROF_MAX_THREAD_COUNT][MAX_NESTING];
unsigned long long int gStartTimes[BT_QUICKPROF_MAX_THREAD_COUNT][MAX_NESTING];
//#endif

btClock clk;

bool gProfileDisabled = true;

void MyDummyEnterProfileZoneFunc(const char* msg)
{
}

void MyDummyLeaveProfileZoneFunc()
{
}

void MyEnterProfileZoneFunc(const char* msg)
{
	if (gProfileDisabled)
		return;

	int threadId = btQuickprofGetCurrentThreadIndex2();
	if (threadId < 0 || threadId >= BT_QUICKPROF_MAX_THREAD_COUNT)
		return;

	if (gStackDepths[threadId] >= MAX_NESTING)
	{
		btAssert(0);
		return;
	}
	gFuncNames[threadId][gStackDepths[threadId]] = msg;
	gStartTimes[threadId][gStackDepths[threadId]] = clk.getTimeNanoseconds();
	if (gStartTimes[threadId][gStackDepths[threadId]] <= gStartTimes[threadId][gStackDepths[threadId] - 1])
	{
		gStartTimes[threadId][gStackDepths[threadId]] = 1 + gStartTimes[threadId][gStackDepths[threadId] - 1];
	}
	gStackDepths[threadId]++;

}
void MyLeaveProfileZoneFunc()
{
	if (gProfileDisabled)
		return;

	int threadId = btQuickprofGetCurrentThreadIndex2();
	if (threadId < 0 || threadId >= BT_QUICKPROF_MAX_THREAD_COUNT)
		return;

	if (gStackDepths[threadId] <= 0)
	{
		return;
	}

	gStackDepths[threadId]--;

	const char* name = gFuncNames[threadId][gStackDepths[threadId]];
	unsigned long long int startTime = gStartTimes[threadId][gStackDepths[threadId]];

	unsigned long long int endTime = clk.getTimeNanoseconds();
	gTimings[threadId].addTiming(name, threadId, startTime, endTime);

}

void b3ChromeUtilsStartTimings()
{
	m_firstTiming = true;
	gProfileDisabled = false;  //true;
	b3SetCustomEnterProfileZoneFunc(MyEnterProfileZoneFunc);
	b3SetCustomLeaveProfileZoneFunc(MyLeaveProfileZoneFunc);

	//also for Bullet 2.x API
	btSetCustomEnterProfileZoneFunc(MyEnterProfileZoneFunc);
	btSetCustomLeaveProfileZoneFunc(MyLeaveProfileZoneFunc);
}

void b3ChromeUtilsStopTimingsAndWriteJsonFile(const char* fileNamePrefix)
{
	b3SetCustomEnterProfileZoneFunc(MyDummyEnterProfileZoneFunc);
	b3SetCustomLeaveProfileZoneFunc(MyDummyLeaveProfileZoneFunc);
	//also for Bullet 2.x API
	btSetCustomEnterProfileZoneFunc(MyDummyEnterProfileZoneFunc);
	btSetCustomLeaveProfileZoneFunc(MyDummyLeaveProfileZoneFunc);
	char fileName[1024];
	static int fileCounter = 0;
	sprintf(fileName, "%s_%d.json", fileNamePrefix, fileCounter++);
	gTimingFile = fopen(fileName, "w");
	if (gTimingFile)
	{
		fprintf(gTimingFile, "{\"traceEvents\":[\n");
		//dump the content to file
		for (int i = 0; i < BT_QUICKPROF_MAX_THREAD_COUNT; i++)
		{
			if (gTimings[i].m_numTimings)
			{
				printf("Writing %d timings for thread %d\n", gTimings[i].m_numTimings, i);
				gTimings[i].flush();
			}
		}
		fprintf(gTimingFile, "\n],\n\"displayTimeUnit\": \"ns\"}");
		fclose(gTimingFile);
	}
	else
	{
		b3Printf("Error opening file");
		b3Printf(fileName);
	}
	gTimingFile = 0;
}

void b3ChromeUtilsEnableProfiling()
{
	gProfileDisabled = false;
}
