
this is the public C interface to the ODE library.

all these files should be includable from C, i.e. they should not use any
C++ features. everything should be protected with

	#ifdef __cplusplus
	extern "C" {
	#endif

	...

	#ifdef __cplusplus
	}
	#endif

the only exceptions are the odecpp.h and odecpp_collisioh.h files, which define a C++ wrapper for
the C interface. remember to keep this in sync!
