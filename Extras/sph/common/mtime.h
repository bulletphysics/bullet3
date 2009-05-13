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


#ifndef TIME_HPP
	#define TIME_HPP

	#include <string>
	
	#ifdef _MSC_VER
		#include <windows.h>
		typedef __int64		mstime;		// microsecond time = 8 byte integer
		typedef __int64		sjtime;		// scaled julian times = 8 byte integer
    
		#define MSEC_SCALAR		1000000i64
		#define SEC_SCALAR		1000000000i64
		#define MIN_SCALAR		60000000000i64
		#define HR_SCALAR		3600000000000i64
		#define	DAY_SCALAR		86400000000000i64

		#pragma warning ( disable : 4522 )
		#pragma warning ( disable : 4996 )		// sprintf warning
	#endif

	#ifdef __linux__
        #include <linux/types.h>
        typedef __s64       mstime;
        typedef __s64       sjtime;

		#define MSEC_SCALAR		1000000LL
		#define SEC_SCALAR		1000000000LL
		#define MIN_SCALAR		60000000000LL
		#define HR_SCALAR		3600000000000LL
		#define	DAY_SCALAR		86400000000000LL
	#endif

	#ifdef __CYGWIN__	
		#include <largeint.h>			// found in \cygwin\usr\include\w32api
        typedef __int64       mstime;
        typedef __int64       sjtime;

		#define MSEC_SCALAR		1000000LL
		#define SEC_SCALAR		1000000000LL
		#define MIN_SCALAR		60000000000LL
		#define HR_SCALAR		3600000000000LL
		#define	DAY_SCALAR		86400000000000LL
	#endif	

	#define ACC_SEC			0
	#define ACC_MSEC		1
	#define ACC_NSEC		2

	#define NSEC_SCALAR		1

    // Time Class
	// R. Hoetzlein
	//
	// Overview:
	//  There is a need in many systems to represent both very small (nanoseconds) and 
	//  very large (millenia) timescales accurately. Modified Julian Date accurate represents 
	//  individual days over +/- about 30,000 yrs. However, MJD represents fractions of a day
	//  as a floating point fraction. This is inaccurate for any timing-critical applications.	
	//  The Time class here uses an 8-byte (64 bit) integer called SJT, Scaled Julian Time.
	//      SJT = MJD * DAY_SCALAR + UT (nanoseconds).
	//  SJT is the Modified Julian Date scaled by a integer factor, and added to Universal Time
	//  represented in nanoseconds.
	//
	// Features:
	// - Accurately represents individual nanoseconds over +/- 30,000 yrs.
	// - Correct rollover of tiny time scales on month, day, year boundaries.
	//     e.g. Set date/time to 11:59:59.9999, on Feb 28th, 
	// - Accurately gives day of the week for any date
	// - Accurately compares two dates (days elapsed) even across leap-years.
	// - Adjust sec/nsec independently from month/day/year (work at scale you desire)	
	//
	// Implementation Notes:
	// JD = Julian Day is the number of days elapsed since Jan 1, 4713 BC in the proleptic Julian calendar.
	//     http://en.wikipedia.org/wiki/Julian_day
	// MJD = Modified Julian Date. Most modern dates, after 19th c., have Julian Date which are greater
	//       than 2400000.5. MJD is an offset. MJD = JD - 2400000.5
	//       It shifts the epoch date (start date) to Nov 17, 1858.
	// UT = Universal Time. This is the time of day in hours as measured from Greenwich England.
	//       For non-astronomic uses, this is: UT = Local Time + Time Zone.
	// SJT = Scaled Julian Time = MJD * DAY_SCALAR + UT (in nanoseconds).
	//
	// Julian Dates (and their MJD and SJT equivalents)
	// ------------
	// Jan 1,  4713 BC = JD 0			= MJD -2400000	= SJT 
	// Jan 1,  1500 AD = JD 2268933.5	= MJD -131067	= SJT 
	// Nov 16, 1858 AD = JD 2400000.5	= MJD 0			= SJT 0
	// Jan 1,  1960 AD = JD 2436935.5	= MJD 36935		= SJT 3,191,184,000,000
	// Jan 1,  2005 AD = JD 2453372.5	= MJD 53372		= SJT 4,611,340,800,000
	// Jan 1,  2100 AD = JD 2488070.5	= MJD 88070		= SJT 7,609,248,000,000
	//
	//
	// 
	// 32/64-Bit Integer Ranges
	//    32-bit Integer Min:              –2,147,483,648   ( 4 bytes )
	//    32-bit Integer Max:               2,147,483,647 
	//    SJT 2005:                     4,611,340,800,000
	//    64-bit Integer Min:  –9,223,372,036,854,775,808 
	//    64-bit Integer Max:   9,223,372,036,854,775,807	( 8 bytes )
	//
	// SJT Range
	// ---------
	//   * USING DAY_SCALAR = 86,400,000 (millisec accuracy)
	//   SJT Range = (+/-9,223,372,036,854,775,807 SJT / 86,400,000 DAY_SCALAR)
	//   SJT Range (in Julian Days) = +2400000.5 + (+/-106,751,991,167 MJD) 	
	//   SJT Range (in Julian Days) = +/- 292278883 years, with 1 millisecond accuracy.
	//
	//   * USING DAY_SCALAR = 86,400,000,000,000 (nanosec accuracy)	
	//   SJT Range = (+/-9,223,372,036,854,775,807 SJT / 86,400,000,000,000 DAY_SCALAR)	
	//   SJT Range (in Julian Days) = +2400000.5 + (+/-106,751 MJD)	
	//   SJT Range (in Julian Days) = 1566 AD to 2151 AD, with 1 nanosecond accuracy.

	namespace mint {

	class Time {
	public:
		Time ();
		Time ( sjtime t )			{ m_CurrTime = t; }
		Time ( int sec, int msec )	{ m_CurrTime = 0; SetTime ( sec, msec ); }
		
		// Set time
		bool SetTime ( int sec );									// Set seconds
		bool SetTime ( int sec, int msec );							// Set seconds, msecs	
		bool SetTime ( int hr, int min, int m, int d, int y);		// Set hr/min, month, day, year	
		bool SetTime ( int hr, int min, int m, int d, int y, int s, int ms, int ns);	// Set hr/min, month, day, year, sec, ms, ns
		bool SetTime ( Time& t )	{ m_CurrTime = t.GetSJT(); return true;} // Set to another Time object				
		bool SetTime ( std::string line );							// Set time from string (hr,min,sec)
		bool SetDate ( std::string line );							// Set date from string (mo,day,yr)		
		void SetSystemTime ();										// Set date/time to system clock		
		void SetSystemTime ( int accuracy );						// Set date/time to system clock
		void SetSJT ( sjtime t )	{ m_CurrTime = t ;}		// Set Scaled Julian Time directly
		
		// Get time		
		void GetTime (int& sec, int& msec, int& nsec );
		void GetTime (int& hr, int& min, int& m, int& d, int& y);				
		void GetTime (int& hr, int& min, int& m, int& d, int& y, int& s, int& ms, int& ns);
		double GetSec ();
		int GetMSec ();
		std::string GetReadableDate ();
		std::string GetReadableTime ();		
		std::string GetReadableTime ( int fmt );
		std::string GetReadableSJT ();
		std::string GetDayOfWeekName ();
		sjtime GetSJT ()			{ return m_CurrTime; } 			

		// Advance Time
		void Advance ( Time& t );
		void AdvanceMinutes ( int n);
		void AdvanceHours ( int n );
		void AdvanceDays ( int n );
		void AdvanceSec ( int n );
		void AdvanceMins ( int n);
		void AdvanceMSec ( int n );
		
		// Utility functions 
		// (these do the actual work, but should not be private as they may be useful to user)
		sjtime GetScaledJulianTime ( int hr, int min, int m, int d, int y );
		sjtime GetScaledJulianTime ( int hr, int min, int m, int d, int y, int s, int ms, int ns );
		void GetTime ( sjtime t, int& hr, int& min, int& m, int& d, int& y);
		void GetTime ( sjtime t, int& hr, int& min, int& m, int& d, int& y, int& s, int& ms, int& ns);
		
		// Get/Set Julian Date and Modified Julain Date
		void SetJD ( double jd );
		void SetMJD ( int jd );
		double GetJD ();
		int GetMJD ();

		// Time operators
		Time& operator= ( const Time& op );
		Time& operator= ( Time& op );
		bool operator< ( const Time& op );
		bool operator< ( Time& op );
		bool operator> ( const Time& op );
		bool operator> ( Time& op );		
		bool operator<= ( const Time& op );
		bool operator<= ( Time& op );
		bool operator>= ( const Time& op );
		bool operator>= ( Time& op );	
		bool operator== ( const Time& op );
		bool operator!= ( Time& op );
		Time operator- ( Time& op );
		Time operator+ ( Time& op );

		// Elapsed Times
		int GetElapsedDays ( Time& base );		
		int GetElapsedWeeks ( Time& base );
		int GetElapsedMonths ( Time& base );
		int GetElapsedYears ( Time& base );
		int GetFracDay ( Time& base );			// Return Unit = 5 mins
		int GetFracWeek ( Time& base );			// Return Unit = 1 hr
		int GetFracMonth ( Time& base );		// Return Unit = 4 hrs
		int GetFracYear ( Time& base );			// Return Unit = 1 day
		int GetDayOfWeek ();
		int GetWeekOfYear ();

		void RegressionTest ();

	private:		
		static const int	m_DaysInMonth[13];
		static bool			m_Started;

		sjtime				m_CurrTime;		
	};

	// Used for precise system time (Win32)
	void start_timing ( sjtime base );
	
	}
#endif



