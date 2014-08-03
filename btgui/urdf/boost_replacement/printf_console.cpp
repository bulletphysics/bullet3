#include "printf_console.h"
#include <stdio.h>


void logError(const char* msg, const char* arg0, const char* arg1, const char* arg2)
{
	printf("%s %s %s %s\n", msg,arg0,arg1,arg2);

}
	
void logDebug(const char* msg, float v0, float v1)
{
	printf("%s %f %f\n", msg, v0, v1);
};
void logDebug(const char* msg, const char* msg1, const char* arg1)
{
	printf("%s %s %s\n", msg, msg1, arg1);

}

void logInform(const char* msg, const char* arg0)
{
	printf("%s %s\n", msg, arg0);
}
void logWarn(const char* msg,int id, const char* arg0)
{
	printf("%s %d %s\n", msg,id,arg0);
}
