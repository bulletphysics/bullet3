/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FUtils/FUDateTime.h"

#include <time.h>

FUDateTime::FUDateTime(const FUDateTime& time)
{
	seconds = time.seconds;
	minutes = time.minutes;
	hour = time.hour;
	day = time.day;
	month = time.month;
	year = time.year;
}

FUDateTime::FUDateTime()
{
	seconds = minutes = hour = 0;
	day = month = 1;
	year = 1900;
}

FUDateTime::~FUDateTime()
{
}

FUDateTime FUDateTime::GetNow()
{
	FUDateTime dateTime;

	// Get current UTC time
	time_t currentTime;

	time(&currentTime);
	tm* utcTime = gmtime(&currentTime);	

	// Convert to our own data structure
	dateTime.SetSeconds(utcTime->tm_sec);
	dateTime.SetMinutes(utcTime->tm_min);
	dateTime.SetHour(utcTime->tm_hour);
	dateTime.SetDay(utcTime->tm_mday);
	dateTime.SetMonth(utcTime->tm_mon + 1);
	dateTime.SetYear(utcTime->tm_year + 1900);

	return dateTime;
}
