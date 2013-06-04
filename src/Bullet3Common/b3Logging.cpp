#include "b3Logging.h"

#include <stdio.h>
#include <stdarg.h>


void b3PrintfFuncDefault(const char* msg)
{
	printf("%s",msg);
}

void b3WarningMessageFuncDefault(const char* msg)
{
	printf("%s",msg);
}

void b3ErrorMessageFuncDefault(const char* msg)
{
	printf("%s",msg);
}

static b3PrintfFunc* b3s_printfFunc = b3PrintfFuncDefault;
static b3WarningMessageFunc* b3s_warningMessageFunc = b3WarningMessageFuncDefault;
static b3ErrorMessageFunc* b3s_errorMessageFunc = b3ErrorMessageFuncDefault;


///The developer can route b3Printf output using their own implementation
void b3SetCustomPrintfFunc(b3PrintfFunc* printfFunc)
{
	b3s_printfFunc = printfFunc;
}
void b3SetCustomWarningMessageFunc(b3PrintfFunc* warningMessageFunc)
{
	b3s_warningMessageFunc = warningMessageFunc;
}
void b3SetCustomErrorMessageFunc(b3PrintfFunc* errorMessageFunc)
{
	b3s_errorMessageFunc = errorMessageFunc;
}

#define B3_MAX_DEBUG_STRING_LENGTH 2048


void b3OutputPrintfVarArgsInternal(const char *str, ...)
{
    char strDebug[B3_MAX_DEBUG_STRING_LENGTH]={0};
    va_list argList;
    va_start(argList, str);
#ifdef _WIN32
    vsprintf_s(strDebug,B3_MAX_DEBUG_STRING_LENGTH,str,argList);
#else
    vsnprintf(strDebug,B3_MAX_DEBUG_STRING_LENGTH,str,argList);
#endif
        (b3s_printfFunc)(strDebug);
    va_end(argList);    
}
void b3OutputWarningMessageVarArgsInternal(const char *str, ...)
{
    char strDebug[B3_MAX_DEBUG_STRING_LENGTH]={0};
    va_list argList;
    va_start(argList, str);
#ifdef _WIN32
    vsprintf_s(strDebug,B3_MAX_DEBUG_STRING_LENGTH,str,argList);
#else
    vsnprintf(strDebug,B3_MAX_DEBUG_STRING_LENGTH,str,argList);
#endif
        (b3s_warningMessageFunc)(strDebug);
    va_end(argList);    
}
void b3OutputErrorMessageVarArgsInternal(const char *str, ...)
{
	
    char strDebug[B3_MAX_DEBUG_STRING_LENGTH]={0};
    va_list argList;
    va_start(argList, str);
#ifdef _WIN32
    vsprintf_s(strDebug,B3_MAX_DEBUG_STRING_LENGTH,str,argList);
#else
    vsnprintf(strDebug,B3_MAX_DEBUG_STRING_LENGTH,str,argList);
#endif
        (b3s_errorMessageFunc)(strDebug);
    va_end(argList);    

}
#ifndef _WIN32
#undef vsprintf_s
#endif

