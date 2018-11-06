
#ifndef B3_CHROME_TRACE_UTIL_H
#define B3_CHROME_TRACE_UTIL_H

void b3ChromeUtilsStartTimings();
void b3ChromeUtilsStopTimingsAndWriteJsonFile(const char* fileNamePrefix);
void b3ChromeUtilsEnableProfiling();

#endif  //B3_CHROME_TRACE_UTIL_H