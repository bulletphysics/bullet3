/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_EVENTS_H
#define GWEN_EVENTS_H

#include <list>
#include "Gwen/Exports.h"
#include "Gwen/Structures.h"

// TODO: REMOVE THIS - IT SUCKS. Template the function instead.
#define GWEN_MCALL(fnc) this, (Gwen::Event::Handler::Function)&fnc

namespace Gwen
{
namespace Controls
{
class Base;
}

namespace Event
{
class Caller;

// A class must be derived from this
class GWEN_EXPORT Handler
{
public:
	Handler();
	virtual ~Handler();

	void RegisterCaller(Caller*);
	void UnRegisterCaller(Caller*);

protected:
	void CleanLinks();
	std::list<Caller*> m_Callers;

public:
	typedef void (Handler::*Function)(Gwen::Controls::Base* pFromPanel);
	typedef void (Handler::*FunctionStr)(const Gwen::String& string);
};

//
//
//
class GWEN_EXPORT Caller
{
public:
	Caller();
	~Caller();

	void Call(Controls::Base* pThis);

	template <typename T>
	void Add(Event::Handler* ob, T f)
	{
		AddInternal(ob, static_cast<Handler::Function>(f));
	}

	void RemoveHandler(Event::Handler* pObject);

protected:
	void CleanLinks();
	void AddInternal(Event::Handler* pObject, Handler::Function pFunction);

	struct handler
	{
		Handler::Function fnFunction;
		Event::Handler* pObject;
	};

	std::list<handler> m_Handlers;
};

}  // namespace Event

}  // namespace Gwen
#endif
