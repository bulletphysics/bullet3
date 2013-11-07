#ifndef B3_PLATFORM_DEFINITIONS_H
#define B3_PLATFORM_DEFINITIONS_H

struct MyTest
{
	int bla;
};

#ifdef __cplusplus
#define b3AtomicInc(a) ((*a)++)
#define __global 
#else
#define b3AtomicInc atomic_inc
#define b3Fabs fabs
#endif

#endif
