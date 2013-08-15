#ifndef B3_PLATFORM_DEFINITIONS_H
#define B3_PLATFORM_DEFINITIONS_H

struct MyTest
{
	int bla;
};

#ifdef __cplusplus
#define b3AtomicInc(a) ((*a)++)
#else
#define b3AtomicInc atomic_inc

#endif

#endif
