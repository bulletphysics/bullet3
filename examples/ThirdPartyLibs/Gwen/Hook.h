/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_HOOK_H
#define GWEN_HOOK_H

#include "Gwen/Gwen.h"
#include <list>

#ifdef GWEN_HOOKSYSTEM

namespace Gwen
{
	namespace Hook
	{
		class GWEN_EXPORT BaseHook
		{
			public:

				virtual bool OnControlClicked( Gwen::Controls::Base*, int /*iMouseX*/, int /*iMouseY*/ ){ return false; };
		};

		typedef std::list<BaseHook*> HookList;

		GWEN_EXPORT HookList& GetHookList();

		GWEN_EXPORT void AddHook( BaseHook* pHook );
		GWEN_EXPORT void RemoveHook( BaseHook* pHook );

		template< typename fnc >
		bool  CallHook( fnc f )
		{
			for ( HookList::iterator it = GetHookList().begin(); it != GetHookList().end(); ++it )
			{
				if ( ((*it)->*f)() ) return true;
			}

			return false;
		}

		template< typename fnc, typename AA >
		bool CallHook( fnc f, AA a )
		{
			for ( HookList::iterator it = GetHookList().begin(); it != GetHookList().end(); ++it )
			{
				if ( ((*it)->*f)( a ) ) return true;
			}

			return false;
		}

		template< typename fnc, typename AA, typename AB >
		bool CallHook( fnc f, AA a, AB b )
		{
			for ( HookList::iterator it = GetHookList().begin(); it != GetHookList().end(); ++it )
			{
				if ( ((*it)->*f)( a, b ) ) return true;
			}

			return false;
		}

		template< typename fnc, typename AA, typename AB, typename AC >
		bool CallHook( fnc f, AA a, AB b, AC c )
		{
			for ( HookList::iterator it = GetHookList().begin(); it != GetHookList().end(); ++it )
			{
				if ( ((*it)->*f)( a, b, c ) ) return true;
			}

			return false;
		}
	}

}

#endif
#endif
