#ifndef __CLSTUFF_HDR__
#define __CLSTUFF_HDR__




// OpenCL initialization.
// Takes an optional GL context which, if passed, will create an interop-enabled CL context.
void initCL( void* glContext = 0, void* glDC = 0 );

#endif //__CLSTUFF_HDR__