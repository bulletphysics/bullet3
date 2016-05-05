/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/


#include "Gwen/Hook.h"

#ifdef GWEN_HOOKSYSTEM

using namespace Gwen;
using namespace Gwen::Hook;

std::list<BaseHook*>	g_HookList;

void Gwen::Hook::AddHook( BaseHook* pHook )
{ 
	g_HookList.push_back( pHook ); 
}

void Gwen::Hook::RemoveHook( BaseHook* pHook )
{ 
	g_HookList.remove( pHook ); 
}

HookList& Gwen::Hook::GetHookList()
{
	return g_HookList;
}

#endif