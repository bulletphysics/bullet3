/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#include "Gwen/Events.h"

using namespace Gwen;
using namespace Gwen::Event;

Handler::Handler()
{
}

Handler::~Handler()
{
	CleanLinks();
}

void Handler::CleanLinks()
{
	// Tell all the callers that we're dead
	std::list<Caller*>::iterator iter = m_Callers.begin();
	while (iter != m_Callers.end())
	{
		Caller* pCaller = *iter;
		UnRegisterCaller(pCaller);
		pCaller->RemoveHandler(this);
		iter = m_Callers.begin();
	}
}

void Handler::RegisterCaller(Caller* pCaller)
{
	m_Callers.push_back(pCaller);
}

void Handler::UnRegisterCaller(Caller* pCaller)
{
	m_Callers.remove(pCaller);
}

Caller::Caller()
{
}

Caller::~Caller()
{
	CleanLinks();
}

void Caller::CleanLinks()
{
	std::list<handler>::iterator iter;
	for (iter = m_Handlers.begin(); iter != m_Handlers.end(); ++iter)
	{
		handler& h = *iter;
		h.pObject->UnRegisterCaller(this);
	}

	m_Handlers.clear();
}

void Caller::Call(Controls::Base* pThis)
{
	std::list<handler>::iterator iter;
	for (iter = m_Handlers.begin(); iter != m_Handlers.end(); ++iter)
	{
		handler& h = *iter;
		(h.pObject->*h.fnFunction)(pThis);
	}
}

void Caller::AddInternal(Event::Handler* pObject, Event::Handler::Function pFunction)
{
	handler h;
	h.fnFunction = pFunction;
	h.pObject = pObject;

	m_Handlers.push_back(h);

	pObject->RegisterCaller(this);
}

void Caller::RemoveHandler(Event::Handler* pObject)
{
	pObject->UnRegisterCaller(this);

	std::list<handler>::iterator iter = m_Handlers.begin();

	while (iter != m_Handlers.end())
	{
		handler& h = *iter;

		if (h.pObject == pObject)
		{
			iter = m_Handlers.erase(iter);
		}
		else
		{
			++iter;
		}
	}
}