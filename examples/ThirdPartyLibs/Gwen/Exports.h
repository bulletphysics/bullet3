/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_EXPORTS_H
#define GWEN_EXPORTS_H



#define GWEN_EXPORT


#ifdef _MSC_VER

	#define GWEN_FINLINE __forceinline
	#define GWEN_PURE_INTERFACE __declspec(novtable)

#elif defined(__GNUC__)

	#define GWEN_FINLINE __attribute__((always_inline)) inline
	#define GWEN_PUREINTERFACE 

#else

	#define GWEN_FINLINE inline
	#define GWEN_PUREINTERFACE 

#endif
#endif
