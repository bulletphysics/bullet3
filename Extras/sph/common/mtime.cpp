/*
  FLUIDS v.1 - SPH Fluid Simulator for CPU and GPU
  Copyright (C) 2009. Rama Hoetzlein, http://www.rchoetzlein.com

  ZLib license
  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

#ifdef _MSC_VER
	#include <windows.h>
#else
	#include <sys/time.h>
#endif 

#include <stdio.h>
#include <time.h>
#include <math.h>

#include "mtime.h"
#include "mdebug.h"

#ifdef _MSC_VER
	#define VS2005
	#pragma comment ( lib, "winmm.lib" )
	LARGE_INTEGER	m_BaseCount;
	LARGE_INTEGER	m_BaseFreq;
#endif

using namespace mint;

const int Time::m_DaysInMonth[13] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
bool Time::m_Started = false;
sjtime			m_BaseTime;
sjtime			m_BaseTicks;

void mint::start_timing ( sjtime base )
{	
	m_BaseTime = base;

	#ifdef _MSC_VER
		m_BaseTicks = timeGetTime();
		QueryPerformanceCounter ( &m_BaseCount );
		QueryPerformanceFrequency ( &m_BaseFreq );
	#else
		struct timeval tv;
		gettimeofday(&tv, NULL);
		m_BaseTicks = ((sjtime) tv.tv_sec * 1000000LL) + (sjtime) tv.tv_usec;		
	#endif
}

void Time::SetSystemTime ( int accuracy )
{
	switch ( accuracy ) {
	case ACC_SEC:			// 1 second accuracy
		SetSystemTime ();
		break;	
	case ACC_MSEC: {			// 1 millisecond accuracy
		#ifdef _MSC_VER
			m_CurrTime = m_BaseTime + sjtime(timeGetTime() - m_BaseTicks)*MSEC_SCALAR;
		#else
			struct timeval tv;
			gettimeofday(&tv, NULL);			
			sjtime t = ((sjtime) tv.tv_sec * 1000000LL) + (sjtime) tv.tv_usec;	
			m_CurrTime = m_BaseTime + ( t - m_BaseTicks) * 1000LL;			// 1000LL - converts microseconds to milliseconds
		#endif
		} break;
	case ACC_NSEC: {		// 1 nanosecond accuracy
		#ifdef _MSC_VER
			LARGE_INTEGER currCount;
			QueryPerformanceCounter ( &currCount );
			m_CurrTime = m_BaseTime + sjtime( (double(currCount.QuadPart-m_BaseCount.QuadPart) / m_BaseFreq.QuadPart) * SEC_SCALAR);
		#else
			debug.Printf ( "mtime", "ERROR: ACC_NSEC NOT IMPLEMENTED for Time::SetSystemTime\n" );
		#endif
		} break;	
	}
}


Time::Time ()
{	
	if ( !m_Started ) {
		m_Started = true;			
		SetSystemTime ();				// Get base time from wall clock
		start_timing ( m_CurrTime );	// Start timing from base time
	}
	m_CurrTime = 0;
}

// Note regarding hours:
//  0 <= hr <= 23
//  hr = 0 is midnight (12 am)
//  hr = 1 is 1 am
//  hr = 12 is noon 
//  hr = 13 is 1 pm (subtract 12)
//  hr = 23 is 11 pm (subtact 12)

// GetScaledJulianTime
// Returns -1.0 if the time specified is invalid.
sjtime Time::GetScaledJulianTime ( int hr, int min, int m, int d, int y, int s, int ms, int ns )
{
	double MJD;				// Modified Julian Date (JD - 2400000.5)
	sjtime SJT;				// Scaled Julian Time SJT = MJD * 86400000 + UT

	// Check if date/time is valid
	if (m <=0 || m > 12) return (sjtime) -1;	
	if ( y % 4 == 0 && m == 2) {	// leap year in february
		if (d <=0 || d > m_DaysInMonth[m]+1) return (sjtime) -1;
	} else {
		if (d <=0 || d > m_DaysInMonth[m]) return (sjtime) -1;		
	}
	if (hr < 0 || hr > 23) return (sjtime) -1;
	if (min < 0 || min > 59) return  (sjtime) -1;

	// Compute Modified Julian Date
	MJD = 367 * y - int ( 7 * (y + int (( m + 9)/12)) / 4 );
	MJD -= int ( 3 * (int((y + (m - 9)/7)/100) + 1) / 4);
	MJD += int ( 275 * m / 9 ) + d + 1721028.5 - 1.0;
	MJD -= 2400000.5;
	// Compute Scaled Julian Time
	SJT = sjtime(MJD) * sjtime( DAY_SCALAR );	
	SJT += hr * HR_SCALAR + min * MIN_SCALAR + s * SEC_SCALAR + ms * MSEC_SCALAR + ns * NSEC_SCALAR;
	return SJT;
}

sjtime Time::GetScaledJulianTime ( int hr, int min, int m, int d, int y )
{
	return GetScaledJulianTime ( hr, min, m, d, y, 0, 0, 0 );
}

void Time::GetTime ( sjtime SJT, int& hr, int& min, int& m, int& d, int& y)
{
	int s = 0, ms = 0, ns = 0;
	GetTime ( SJT, hr, min, m, d, y, s, ms, ns );
}

void Time::GetTime ( sjtime SJT, int& hr, int& min, int& m, int& d, int& y, int& s, int &ms, int& ns)
{	
	// Compute Universal Time from SJT
	sjtime UT = sjtime( SJT % sjtime( DAY_SCALAR ) );

	// Compute Modified Julian Date from SJT
	double MJD = double(SJT / DAY_SCALAR);

	// Use MJD to get Month, Day, Year
	double z = floor ( MJD + 1 + 2400000.5 - 1721118.5);
	double g = z - 0.25;
	double a = floor ( g / 36524.25 );
	double b = a - floor  ( a / 4.0 );
	y = int( floor (( b + g ) / 365.25 ) );
	double c = b + z - floor  ( 365.25 * y );
	m = int (( 5 * c + 456) / 153 );
	d = int( c - int (( 153 * m - 457) / 5) );
	if (m > 12) {
		y++;
		m -= 12;
	}
	// Use UT to get Hrs, Mins, Secs, Msecs
	hr = int( UT / HR_SCALAR );
	UT -= hr * HR_SCALAR;
	min = int( UT / MIN_SCALAR );
	UT -= min * MIN_SCALAR;
	s = int ( UT / SEC_SCALAR );
	UT -= s * SEC_SCALAR;
	ms = int ( UT / MSEC_SCALAR );	
	UT -= ms * MSEC_SCALAR;
	ns = int ( UT / NSEC_SCALAR );

	// UT Example:
	//      MSEC_SCALAR =         1
	//      SEC_SCALAR =      1,000
	//      MIN_SCALAR =     60,000
	//		HR_SCALAR =   3,600,000
	//      DAY_SCALAR = 86,400,000
	//
	//   7:14:03, 32 msec 	
	//   UT = 7*3,600,000 + 14*60,000 + 3*1,000 + 32 = 26,043,032
	//
	//   26,043,032 / 3,600,000 = 7			26,043,032 - (7 * 3,600,000) = 843,032
	//      843,032 /    60,000 = 14		   843,032 - (14 * 60,000) = 3,032
	//        3,032 /     1,000 = 3              3,032 - (3 * 1,000) = 32
	//           32 /         1 = 32	
}

void Time::GetTime (int& s, int& ms, int& ns )
{
	int hr, min, m, d, y;
	GetTime ( m_CurrTime, hr, min, m, d, y, s, ms, ns );
}


void Time::GetTime (int& hr, int& min, int& m, int& d, int& y)
{
	GetTime ( m_CurrTime, hr, min, m, d, y);
}

void Time::GetTime (int& hr, int& min, int& m, int& d, int& y, int& s, int& ms, int& ns)
{
	GetTime ( m_CurrTime, hr, min, m, d, y, s, ms, ns);
}

bool Time::SetTime ( int sec )
{
	int hr, min, m, d, y;
	GetTime ( m_CurrTime, hr, min, m, d, y );
	m_CurrTime = GetScaledJulianTime ( hr, min, m, d, y, sec, 0, 0 );
	return true;
}

bool Time::SetTime ( int sec, int msec )
{
	int hr, min, m, d, y;
	GetTime ( m_CurrTime, hr, min, m, d, y );
	m_CurrTime = GetScaledJulianTime ( hr, min, m, d, y, sec, msec, 0 );
	return true;
}

bool Time::SetTime (int hr, int min, int m, int d, int y)
{
	int s, ms, ns;
	GetTime ( s, ms, ns );
	m_CurrTime = GetScaledJulianTime ( hr, min, m, d, y, s, ms, ns );
	if (m_CurrTime == -1.0) return false;
	return true;
}

bool Time::SetTime (int hr, int min, int m, int d, int y, int s, int ms, int ns)
{
	m_CurrTime = GetScaledJulianTime ( hr, min, m, d, y, s, ms, ns );
	if (m_CurrTime == -1.0) return false;
	return true;
}

bool Time::SetTime ( std::string line )
{
	int hr, min, m, d, y;
	std::string dat;
	if ( line.substr ( 0, 1 ) == " " ) 
		dat = line.substr ( 1, line.length()-1 ).c_str();
	else 
		dat = line;

	hr = atoi ( dat.substr ( 0, 2).c_str() );
	min = atoi ( dat.substr ( 3, 2).c_str() );
	m = atoi ( dat.substr ( 6, 2).c_str () );
	d = atoi ( dat.substr ( 9, 2).c_str () );
	y = atoi ( dat.substr ( 12, 4).c_str () );
	return SetTime ( hr, min, m, d, y);
}

bool Time::SetDate ( std::string line )
{
	int hr, min, m, d, y;
	std::string dat;
	if ( line.substr ( 0, 1 ) == " " ) 
		dat = line.substr ( 1, line.length()-1 ).c_str();
	else 
		dat = line;

	hr = 0;
	min = 0;
	m = atoi ( dat.substr ( 0, 2).c_str () );
	d = atoi ( dat.substr ( 3, 2).c_str () );
	y = atoi ( dat.substr ( 6, 4).c_str () );
	return SetTime ( hr, min, m, d, y);
}

std::string Time::GetDayOfWeekName ()
{
	switch (GetDayOfWeek()) {
	case 1:		return "Sunday";	break;
	case 2:		return "Monday";	break;
	case 3:		return "Tuesday";	break;
	case 4:		return "Wednesday";	break;
	case 5:		return "Thursday";	break;
	case 6:		return "Friday";	break;
	case 7:		return "Saturday";	break;
	}
	return "day error";
}

int Time::GetDayOfWeek ()
{
	// Compute Modified Julian Date
	double MJD = (double) m_CurrTime / sjtime( DAY_SCALAR );

	// Compute Julian Date
	double JD = floor ( MJD + 1 + 2400000.5 );
	int dow = (int(JD - 0.5) % 7) + 4;
	if (dow > 7) dow -= 7;

	// day of week (1 = sunday, 7 = saturday)
	return dow ;
}

int Time::GetWeekOfYear ()
{
	int hr, min, m, d, y;
	GetTime ( hr, min, m, d, y );
	double mjd_start = (double) GetScaledJulianTime ( 0, 0, 1, 1, y ) / DAY_SCALAR; // mjt for jan 1st of year
	double mjd_curr = (double) GetScaledJulianTime ( 0, 0, m, d, y ) / DAY_SCALAR; // mjt for specified day in year
	double JD = floor ( mjd_start + 1 + 2400000.5 );
	int dow = (int ( JD - 0.5 ) % 7) + 4;  // day of week for jan 1st of year.
	if (dow > 7) dow -= 7;
	
	// week of year (first week in january = week 0)
	return int((mjd_curr - mjd_start + dow -1 ) / 7 );
}

int Time::GetElapsedDays ( Time& base )
{
	return int( sjtime(m_CurrTime - base.GetSJT() ) / sjtime( DAY_SCALAR ) );
}

int Time::GetElapsedWeeks ( Time& base )
{
	return GetElapsedDays(base) / 7;
}

int Time::GetElapsedMonths ( Time& base)
{
	return int ( double(GetElapsedDays(base)) / 30.416 );
}

int Time::GetElapsedYears ( Time& base )
{
	// It is much easier to compute this in m/d/y format rather
	// than using julian dates.
	int bhr, bmin, bm, bd, by;
	int ehr, emin, em, ed, ey;
	GetTime ( base.GetSJT(), bhr, bmin, bm, bd, by );
	GetTime ( m_CurrTime, ehr, emin, em, ed, ey );
	if ( em < bm) {
		// earlier month
		return ey - by - 1;
	} else if ( em > bm) {
		// later month
		return ey - by;
	} else {
		// same month
		if ( ed < bd ) {
			// earlier day
			return ey - by - 1;
		} else if ( ed >= bd ) {
			// later or same day
			return ey - by;
		}
	}
	return -1;
}

int Time::GetFracDay ( Time& base )
{
	// Resolution = 5-mins
	return int( sjtime(m_CurrTime - base.GetSJT() ) % sjtime(DAY_SCALAR) ) / (MIN_SCALAR*5);
}

int Time::GetFracWeek ( Time& base )
{
	// Resolution = 1 hr
	int day = GetElapsedDays(base) % 7;		// day in week
	int hrs = int( sjtime(m_CurrTime - base.GetSJT() ) % sjtime(DAY_SCALAR) ) / (HR_SCALAR);
	return day * 24 + hrs;
}

int Time::GetFracMonth ( Time& base )
{
	// Resolution = 4 hrs
	int day = (int) fmod ( double(GetElapsedDays(base)), 30.416 );	// day in month
	int hrs = int( sjtime(m_CurrTime - base.GetSJT() ) % sjtime(DAY_SCALAR) ) / (HR_SCALAR*4);
	return day * (24 / 4) + hrs;
}

int Time::GetFracYear ( Time& base )
{
	// It is much easier to compute this in m/d/y format rather
	// than using julian dates.
	int bhr, bmin, bm, bd, by;
	int ehr, emin, em, ed, ey;
	sjtime LastFullYear;
	GetTime ( base.GetSJT() , bhr, bmin, bm, bd, by );
	GetTime ( m_CurrTime, ehr, emin, em, ed, ey );
	if ( em < bm) {
		// earlier month
		LastFullYear = GetScaledJulianTime ( ehr, emin, bm, bd, ey - 1);		
		return int( sjtime(m_CurrTime - LastFullYear) / sjtime(DAY_SCALAR) );		
	} else if ( em > bm) {
		// later month
		LastFullYear = GetScaledJulianTime ( ehr, emin, bm, bd, ey);
		return int( sjtime(m_CurrTime - LastFullYear) / sjtime(DAY_SCALAR) );				
	} else {
		// same month
		if ( ed < bd ) {
			// earlier day
			LastFullYear = GetScaledJulianTime ( ehr, emin, bm, bd, ey - 1);		
			return int( sjtime(m_CurrTime - LastFullYear) / sjtime(DAY_SCALAR) );		
		} else if ( ed > bd ) {
			// later day
			LastFullYear = GetScaledJulianTime ( ehr, emin, bm, bd, ey);
			return int( sjtime(m_CurrTime - LastFullYear) / sjtime(DAY_SCALAR) );
		} else {
			return 0;	// same day
		}
	}	
}

std::string Time::GetReadableDate ()
{
	char buf[200];
	std::string line;
	int hr, min, m, d, y;

	GetTime ( hr, min, m, d, y );
	sprintf ( buf, "%02d:%02d %02d-%02d-%04d", hr, min, m, d, y);	
	return std::string ( buf );
}

std::string Time::GetReadableTime ()
{
	char buf[200];
	std::string line;
	int hr, min, m, d, y, s, ms, ns;

	GetTime ( hr, min, m, d, y, s, ms, ns );	
	sprintf ( buf, "%02d:%02d:%02d %03d.%06d %02d-%02d-%04d", hr, min, s, ms, ns, m, d, y);
	return std::string ( buf );
}

std::string Time::GetReadableSJT ()
{
	char buf[200];	
	sprintf ( buf, "%I64d", m_CurrTime );
	return std::string ( buf );
}

std::string Time::GetReadableTime ( int fmt )
{
	char buf[200];	
	int hr, min, m, d, y, s, ms, ns;
	GetTime ( hr, min, m, d, y, s, ms, ns );	

	switch (fmt) {
	case 0: sprintf ( buf, "%02d %03d.%06d", s, ms, ns);
	}
	return std::string ( buf );
}

void Time::SetSystemTime ()
{
	int hr, mn, sec, m, d, y;
	char timebuf[100];
	char datebuf[100];
	std::string line;

	#ifdef _MSC_VER
		#ifdef VS2005
		_strtime_s ( timebuf, 100 );
		_strdate_s ( datebuf, 100 );
		#else
		_strtime ( timebuf );
		_strdate ( datebuf );
		#endif
	#endif
	#if (defined(__linux__) || defined(__CYGWIN__))
		time_t tt; 
		struct tm tim;
		tt = time(NULL);	
		localtime_r(&tt, &tim);	
		sprintf( timebuf, "%02i:%02i:%02i", tim.tm_hour, tim.tm_min, tim.tm_sec);
		sprintf( datebuf, "%02i:%02i:%02i", tim.tm_mon, tim.tm_mday, tim.tm_year % 100);
	#endif

	line = timebuf;
	hr = atoi ( line.substr ( 0, 2).c_str() );
	mn = atoi ( line.substr ( 3, 2).c_str() );
	sec = atoi ( line.substr ( 6, 2).c_str() );
	line = datebuf;
	m = atoi ( line.substr ( 0, 2).c_str() );
	d = atoi ( line.substr ( 3, 2).c_str() );
	y = atoi ( line.substr ( 6, 2).c_str() );
	
	// NOTE: This only works from 1930 to 2030
	if ( y > 30) y += 1900;
	else y += 2000;
	
	SetTime ( hr, mn, m, d, y, sec, 0, 0);
}
   

double Time::GetSec ()
{
	return ((double) m_CurrTime / (double) SEC_SCALAR );
}

int Time::GetMSec ()
{
	return ((double) m_CurrTime / (double) MSEC_SCALAR );

	/*int s, ms, ns;
	GetTime ( s, ms, ns );
	return ms;*/
}

void Time::Advance ( Time& t )
{
	m_CurrTime += t.GetSJT ();
}

void Time::AdvanceMinutes ( int n)
{
	m_CurrTime += (sjtime) MIN_SCALAR * n;
}

void Time::AdvanceHours ( int n )
{
	m_CurrTime += (sjtime) HR_SCALAR * n;	
}

void Time::AdvanceDays ( int n )
{
	m_CurrTime += (sjtime) DAY_SCALAR * n;	
}

void Time::AdvanceSec ( int n )
{
	m_CurrTime += (sjtime) SEC_SCALAR * n;	
}

void Time::AdvanceMins ( int n)
{
	m_CurrTime += (sjtime) MIN_SCALAR * n;
}	

void Time::AdvanceMSec ( int n )
{
	m_CurrTime += (sjtime) MSEC_SCALAR * n;	
}

Time& Time::operator= ( const Time& op )	{ m_CurrTime = op.m_CurrTime; return *this; }
Time& Time::operator= ( Time& op )			{ m_CurrTime = op.m_CurrTime; return *this; }
bool Time::operator< ( const Time& op )		{ return (m_CurrTime < op.m_CurrTime); }
bool Time::operator> ( const Time& op )		{ return (m_CurrTime > op.m_CurrTime); }
bool Time::operator< ( Time& op )			{ return (m_CurrTime < op.m_CurrTime); }
bool Time::operator> ( Time& op )			{ return (m_CurrTime > op.m_CurrTime); }

bool Time::operator<= ( const Time& op )		{ return (m_CurrTime <= op.m_CurrTime); }
bool Time::operator>= ( const Time& op )		{ return (m_CurrTime >= op.m_CurrTime); }
bool Time::operator<= ( Time& op )			{ return (m_CurrTime <= op.m_CurrTime); }
bool Time::operator>= ( Time& op )			{ return (m_CurrTime >= op.m_CurrTime); }

Time Time::operator- ( Time& op )
{
	return Time( m_CurrTime - op.GetSJT() );
}
Time Time::operator+ ( Time& op )
{
	return Time( m_CurrTime + op.GetSJT() );
}

bool Time::operator== ( const Time& op )
{
	return (m_CurrTime == op.m_CurrTime);
}
bool Time::operator!= ( Time& op )
{
	return (m_CurrTime != op.m_CurrTime);
}
	
void Time::RegressionTest ()
{
	// This code verifies the Julian Date calculations are correct for all
	// minutes over a range of years. Useful to debug type issues when
	// compiling on different platforms.
	//
	int m, d, y, hr, min;
	int cm, cd, cy, chr, cmin;

	for (y=2000; y < 2080; y++) {
		for (m=1; m <= 12; m++) {
			for (d=1; d <= 31; d++) {
				for (hr=0; hr<=23; hr++) {
					for (min=0; min<=59; min++) {
						if ( SetTime ( hr, min, m, d, y, 0, 0, 0 ) ) {
							GetTime ( chr, cmin, cm, cd, cy );
							if ( hr!=chr || min!=cmin || m!=cm || d!=cd || y!=cy) {
//								debug.Printf (" time", "Error: %d, %d, %d, %d, %d = %I64d\n", hr, min, m, d, y, GetSJT());
//								debug.Printf (" time", "-----: %d, %d, %d, %d, %d\n", chr, cmin, cm, cd, cy);
							}
						}
					}
				}				
			}			
		}
//		debug.Printf (" time", "Verified: %d\n", y);
	}
}


