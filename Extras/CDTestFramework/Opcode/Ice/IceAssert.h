///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Contains custom assertion code.
 *	\file		IceAssert.h
 *	\author		Pierre Terdiman
 *	\date		January, 14, 2001
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef ICEASSERT_H
#define ICEASSERT_H

// Leave the {} so that you can write this kind of things safely in release mode:
//	if(condition)	ASSERT()

#ifndef ASSERT
	#if defined( _DEBUG )
		FUNCTION ICECORE_API bool CustomAssertFunction(int, char*, int, char*, bool&);

		//! Custom ASSERT function. Various usages:
		//! ASSERT(condition)
		//! ASSERT(!"Not implemented")
		//! ASSERT(condition && "error text")
		#define ASSERT(exp)																		\
		{																						\
			static bool IgnoreAlways = false;													\
			if(!IgnoreAlways)																	\
			{																					\
				if(CustomAssertFunction((int)(exp), #exp, __LINE__, __FILE__, IgnoreAlways))	\
				{																				\
					_asm { int 3 }																\
				}																				\
			}																					\
		}
	#else
		#define ASSERT(exp)	{}
	#endif
#endif

#ifndef assert
	#define assert	ASSERT
#endif

	#define ICE_COMPILE_TIME_ASSERT(exp)	extern char ICE_Dummy[ (exp) ? 1 : -1 ]

#endif // ICEASSERT_H
