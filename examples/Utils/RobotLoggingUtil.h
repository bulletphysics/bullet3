#ifndef ROBOT_LOGGING_UTIL_H
#define ROBOT_LOGGING_UTIL_H

#include "LinearMath/btAlignedObjectArray.h"
#include <string>

struct MinitaurLogValue
{
	MinitaurLogValue()
		: m_intVal(0xcdcdcdcd)
	{
	}
	MinitaurLogValue(int iv)
		: m_intVal(iv)
	{
	}
	MinitaurLogValue(float fv)
		: m_floatVal(fv)
	{
	}
	MinitaurLogValue(char fv)
		: m_charVal(fv)
	{
	}

	union {
		char m_charVal;
		int m_intVal;
		float m_floatVal;
	};
};

struct MinitaurLogRecord
{
	btAlignedObjectArray<MinitaurLogValue> m_values;
};

enum MINITAUR_LOG_ERROR
{
	eMinitaurFileNotFound = -1,
	eCorruptHeader = -2,
	eUnknownType = -3,
	eCorruptValue = -4,
	eInvalidAABBAlignCheck = -5,
};

int readMinitaurLogFile(const char* fileName, btAlignedObjectArray<std::string>& structNames, std::string& structTypes, btAlignedObjectArray<MinitaurLogRecord>& logRecords, bool verbose);

FILE* createMinitaurLogFile(const char* fileName, btAlignedObjectArray<std::string>& structNames, std::string& structTypes);
void appendMinitaurLogData(FILE* f, std::string& structTypes, const MinitaurLogRecord& logData);
void closeMinitaurLogFile(FILE* f);

#endif  //ROBOT_LOGGING_UTIL_H
