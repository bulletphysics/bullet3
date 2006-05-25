/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/
/*
* The idea is simple: the object registers itself in the constructor, with the container.
* It keeps a pointer to this container, so that the parent can easily access it
* In the destructor, the object unregisters itself with the container.
*/

#include "StdAfx.h"
#include "FUtils/FUObject.h"

#ifdef _DEBUG
FUObject::FUObject(FUObjectContainer* _container, const char* _className)
#else 
FUObject::FUObject(FUObjectContainer* _container)
#endif
{
	container = _container;
#ifdef _DEBUG
	className = _className;
#endif
	if (container != NULL)
	{
		container->RegisterObject(this);
	}
}

FUObject::~FUObject()
{
	if (container != NULL)
	{
		container->UnregisterObject(this);
		container = NULL;
	}
}

FUObjectContainer::~FUObjectContainer()
{
#ifdef _DEBUG
	while (!objects.empty()) 
	{ 
		FUObject* o = objects.back();
		SAFE_DELETE(o); //FIXME: this might cause a crash, but it's good for finding bugs or memory leaks.
	}
#endif
}
void FUObjectContainer::RegisterObject(FUObject* object)
{
#if defined(_DEBUG) && defined(_WIN32)
	// If the check below fails, you are registering an object twice with this container
	FUObjectList::iterator it = std::find(objects.begin(), objects.end(), object);
	if (it != objects.end()) { __asm int 3 };
#endif
	objects.push_back(object);
}

void FUObjectContainer::UnregisterObject(FUObject* object)
{
	FUObjectList::iterator it = std::find(objects.begin(), objects.end(), object);
	if (it != objects.end()) objects.erase(it);
#if defined(_DEBUG) && defined(_WIN32)
	else { __asm int 3 };
#endif
}

