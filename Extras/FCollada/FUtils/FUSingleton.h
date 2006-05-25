/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/
/*
	Taken off the Protect project in 2005.
*/

/**
	@file FUSingleton.h
	This file contains macros to easily implement singletons.
	A singleton is a class which has only one object of this class.
	The advantage of a singleton over a static class is that the
	application controls when and how the singleton is created and
	destroyed. The disadvantage of a singleton is that you have one
	extra memory lookup to do.
*/

#ifndef _FU_SINGLETON_H_
#define _FU_SINGLETON_H_

/** Declares a singleton.
	Use this macros within the class declaration.
	@param className The name of the class. */
#define DECLARE_SINGLETON_CLASS(className) \
private: \
	static className* m_pSingleton; \
public: \
	static bool CreateSingleton(); \
	static void DestroySingleton(); \
	friend className* Get##className();

/**	Declares a singleton.
	Use this macros within the class declaration.
	@param className The name of the class.
	@param createArgs An argument for the constructor of the singleton. */
#define DECLARE_SINGLETON_CLASS_WITH_ARGS(className, createArgs) \
private: \
	static className* m_pSingleton; \
public: \
	static bool CreateSingleton(createArgs); \
	static void DestroySingleton(); \
	friend className* Get##className();

/**	Declares a singleton.
	Use this macros within the class declaration.
	@param className The name of the class.
	@param createArgs1 A first argument for the constructor of the singleton.
	@param createArgs2 A second argument for the constructor of the singleton. */
#define DECLARE_SINGLETON_CLASS_WITH_ARGS2(className, createArgs1, createArgs2) \
private: \
	static className* m_pSingleton; \
public: \
	static bool CreateSingleton(createArgs1, createArgs2); \
	static void DestroySingleton(); \
	friend className* Get##className();

/**	Implements the singleton.
	Use this macros within the class implementation.
	@param className The name of the class. */
#define IMPLEMENT_SINGLETON(className) \
	IMPLEMENT_CREATE_SINGLETON(className) \
	IMPLEMENT_DESTROY_SINGLETON(className)

/**	Implements the singleton.
	Use this macros within the class implementation.
	@param className The name of the class.
	@param createArg1 The argument for the constructor of the singleton. */
#define IMPLEMENT_SINGLETON_WITH_ARGS(className, createArg1) \
	IMPLEMENT_CREATE_SINGLETON_WITH_ARGS(className, createArg1) \
	IMPLEMENT_DESTROY_SINGLETON(className)

/**	Implements the singleton.
	Use this macros within the class implementation.
	@param className The name of the class.
	@param createArg1 A first argument for the constructor of the singleton.
	@param createArg2 A second argument for the constructor of the singleton. */
#define IMPLEMENT_SINGLETON_WITH_ARGS2(className, createArg1, createArg2) \
	IMPLEMENT_CREATE_SINGLETON_WITH_ARGS2(className, createArg1, createArg2) \
	IMPLEMENT_DESTROY_SINGLETON(className)

/**	Implements the construction of a singleton.
	Use this macros within the class implementation.
	@param className The name of the class. */
#define IMPLEMENT_CREATE_SINGLETON(className) \
	className* className::m_pSingleton; \
	className* Get##className() { return className::m_pSingleton; } \
	bool className::CreateSingleton() { \
		m_pSingleton = new className(); \
		return true; }

/** Implements the construction of a singleton.
	Use this macros within the class implementation.
	@param className The name of the class.
	@param createArg1 The argument for the constructor of the singleton. */
#define IMPLEMENT_CREATE_SINGLETON_WITH_ARGS(className, createArg1) \
	className* className::m_pSingleton; \
	className* Get##className() { return className::m_pSingleton; } \
	bool className::CreateSingleton(createArg1 argument1) { \
		m_pSingleton = new className(argument1); }

/** Implements the construction of a singleton.
	Use this macros within the class implementation.
	@param className The name of the class.
	@param createArg1 A first argument for the constructor of the singleton.
	@param createArg2 A second argument for the constructor of the singleton. */
#define IMPLEMENT_CREATE_SINGLETON_WITH_ARGS2(className, createArg1, createArg2) \
	className* className::m_pSingleton; \
	className* Get##className() { return className::m_pSingleton; } \
	bool className::CreateSingleton(createArg1 argument1, createArg2 argument2) { \
		m_pSingleton = new className(argument1, argument2); }

/** Implements the destruction of a singleton.
	Use this macros within the class implementation.
	@param className The name of the class. */
#define IMPLEMENT_DESTROY_SINGLETON(className) \
	void className::DestroySingleton() \
	{ SAFE_DELETE(m_pSingleton); }

#endif // _FU_SINGLETON_H_
