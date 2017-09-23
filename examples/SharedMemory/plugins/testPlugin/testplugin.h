#ifndef TEST_PLUGIN_H
#define TEST_PLUGIN_H

#include "../b3PluginAPI.h"

#ifdef __cplusplus
extern "C" 
{
#endif

B3_SHARED_API int initPlugin();
B3_SHARED_API void exitPlugin();
B3_SHARED_API int executePluginCommand(const char* arguments);

#ifdef __cplusplus
};
#endif

#endif//#define TEST_PLUGIN_H
