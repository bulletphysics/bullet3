#ifndef PRINTF_CONSOLE_H
#define PRINTF_CONSOLE_H


void logError(const char* msg="", const char* arg0="", const char* arg1="", const char* arg2="");
void logDebug(const char* msg, float v0, float v1);
void logDebug(const char* msg, const char* msg1="", const char* arg1="");
void logInform(const char* msg, const char* arg0="");
void logWarn(const char* msg,int id, const char* arg0="");

#endif


