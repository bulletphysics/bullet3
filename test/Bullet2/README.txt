1) Add a .cpp and .h file for your test function.  The function should conform to:

    #ifdef __cplusplus
        extern "C" {
    #endif

        #include "Utils.h"
        #include "main.h"
        #include "vector.h"

        // Your test function
        int MyTestFunc(void);

    #ifdef __cplusplus
        }
    #endif

    The rest of the program doesn't care or know what you do in MyTestFunc, except that MyTestFunc should return non-zero in case of failure in MyTestFunc. There are some handy functions in Utils.h that you might want to use. Please use vlog instead of printf to print stuff, and random_number32/64() in place of rand(), so I can multithread later if it comes to that.  There are some read-only globals that you may wish to respond to, declared in Utils.h:

        gReportAverageTimes	if you do timing, report times as averages instead of best times if non-zero
        gExitOnError		if non-zero, return non-zero immediately if you encounter an error
        gAppName			(const char*) the name of the application

    As a convenience, vector.h has some cross platform vector types declared and will correctly include various vector headers according to compiler flag.


2) Add an entry to gTestList in TestList.cpp for your test function, so the rest of the app knows to call it

