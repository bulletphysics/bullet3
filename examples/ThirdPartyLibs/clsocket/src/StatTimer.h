/*----------------------------------------------------------------------------*/
/*                                                                            */
/* StatTimer.h: interface for the CStatTimer class.                           */
/*                                                                            */
/* Author: Mark Carrier (mark@carrierlabs.com)                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2006 CarrierLabs, LLC.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * 4. The name "CarrierLabs" must not be used to
 *    endorse or promote products derived from this software without
 *    prior written permission. For written permission, please contact
 *    mark@carrierlabs.com.
 *
 * THIS SOFTWARE IS PROVIDED BY MARK CARRIER ``AS IS'' AND ANY
 * EXPRESSED OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL MARK CARRIER OR
 * ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *----------------------------------------------------------------------------*/
#ifndef __CSTATTIMER_H__
#define __CSTATTIMER_H__

#include <string.h>

#if WIN32
#include <winsock2.h>
#include <time.h>
#endif

#ifdef _LINUX
#include <stdio.h>
#include <sys/time.h>
#endif

#include "Host.h"

#if defined(WIN32)
#define GET_CLOCK_COUNT(x) QueryPerformanceCounter((LARGE_INTEGER *)x)
#else
#define GET_CLOCK_COUNT(x) gettimeofday(x, NULL)
#endif

#define MILLISECONDS_CONVERSION 1000
#define MICROSECONDS_CONVERSION 1000000

/// Class to abstract socket communications in a cross platform manner.
/// This class is designed
class CStatTimer
{
public:
	CStatTimer(){};

	~CStatTimer(){};

	void Initialize()
	{
		memset(&m_startTime, 0, sizeof(struct timeval));
		memset(&m_endTime, 0, sizeof(struct timeval));
	};

	struct timeval GetStartTime() { return m_startTime; };
	void SetStartTime() { GET_CLOCK_COUNT(&m_startTime); };

	struct timeval GetEndTime() { return m_endTime; };
	void SetEndTime() { GET_CLOCK_COUNT(&m_endTime); };

	uint32 GetMilliSeconds() { return (CalcTotalUSec() / MILLISECONDS_CONVERSION); };
	uint32 GetMicroSeconds() { return (CalcTotalUSec()); };
	uint32 GetSeconds() { return (CalcTotalUSec() / MICROSECONDS_CONVERSION); };

	uint32 GetCurrentTime()
	{
		struct timeval tmpTime;
		GET_CLOCK_COUNT(&tmpTime);
		return ((tmpTime.tv_sec * MICROSECONDS_CONVERSION) + tmpTime.tv_usec);
	};

private:
	uint32 CalcTotalUSec() { return (((m_endTime.tv_sec - m_startTime.tv_sec) * MICROSECONDS_CONVERSION) +
									 (m_endTime.tv_usec - m_startTime.tv_usec)); };

private:
	struct timeval m_startTime;
	struct timeval m_endTime;
};

#endif  // __CSTATTIMER_H__
