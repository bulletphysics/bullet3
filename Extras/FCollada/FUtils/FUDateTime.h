/*
	Copyright (C) 2006 Feeling Software Inc
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

/**
	@file FUDateTime.h
	This file contains the FUDateTime class.
*/

#ifndef _FU_DATETIME_H_
#define _FU_DATETIME_H_

/**
	A common date-time.
	Encapsulates the OS-dependant timing functions. Use the static member function: GetNow()
	to sample the current time. The day and month values are 1-indexed, to be user-friendly.

	@ingroup FUtils
*/
class FCOLLADA_EXPORT FUDateTime
{
private:
	// To be more friendly, 'day' and 'month' are 1-indexed
	uint32 seconds;
	uint32 minutes;
	uint32 hour;
	uint32 day;
	uint32 month;
	uint32 year;

public:
	/** Default constructor. The default date-time is set to 01/01/1900 at 00:00:00. */
	FUDateTime();
	/** Copy constructor. Creates an identical clone of the given date-time structure.
		@param time The date-time structure to copy. */
	FUDateTime(const FUDateTime& time);
	/** Destructor. */
	~FUDateTime();

	/** Retrieves the seconds component of the date-time structure.
		@returns The seconds component. The seconds component is always in the range [0, 60[. */
	inline uint32 GetSeconds() const { return seconds; }
	/** Retrieves the minutes component of the date-time structure.
		@returns The minutes component. The minutes component is always in the range [0, 60[. */
	inline uint32 GetMinutes() const { return minutes; }
	/** Retrieves the hour component of the date-time structure.
		@returns The hour component. The hour component is always in the range [0, 24[. */
	inline uint32 GetHour() const { return hour; }
	/** Retrieves the day component of the date-time structure.
		@returns The day component. The day component is 1-indexed and has an upper range
			that depends on the month component. The valid range for the day component is [1, 31].*/
	inline uint32 GetDay() const { return day; }
	/** Retrieves the month component of the date-time structure.
		@returns The month component. The month component is 1-indexed and is always in the range [1, 12].*/
	inline uint32 GetMonth() const { return month; }
	/** Retrieves the year component of the date-time structure.
		@returns The year component. The year component represents the full year value,
			where a value of 2000 is returned for the year 2000.*/
	inline uint32 GetYear() const { return year; }

	/** Sets the seconds component of the date-time structure.
		@param _seconds The new seconds value. No verification is made
			to verify that the new value is within the valid range. */
	inline void SetSeconds(uint32 _seconds) { seconds = _seconds; }
	/** Sets the minutes component of the date-time structure.
		@param _minutes The new seconds value. No verification is made
			to verify that the new value is within the valid range. */
	inline void SetMinutes(uint32 _minutes) { minutes = _minutes; }
	/** Sets the hour component of the date-time structure.
		@param _hour The new seconds value. No verification is made
			to verify that the new value is within the valid range. */
	inline void SetHour(uint32 _hour) { hour = _hour; }
	/** Sets the day component of the date-time structure.
		@param _day The new seconds value. No verification is made
			to verify that the new value is within the valid range. */
	inline void SetDay(uint32 _day) { day = _day; }
	/** Sets the month component of the date-time structure.
		@param _month The new seconds value. No verification is made
			to verify that the new value is within the valid range. */
	inline void SetMonth(uint32 _month) { month = _month; }
	/** Sets the year component of the date-time structure.
		@param _year The new seconds value. No verification is made
			to verify that the new value is within the valid range. */
	inline void SetYear(uint32 _year) { year = _year; }

	/** Creates a date-time structure to represent the current time.
		Encapsulates the OS-dependant time() function.
		@return The current date-time. */
	static FUDateTime GetNow();
};

#endif // _FU_DATETIME_H_
