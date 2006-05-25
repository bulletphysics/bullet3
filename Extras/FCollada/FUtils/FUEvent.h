/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/
/*
	This file was taken off the Protect project on 26-09-2005
*/


#ifndef _EVENT_H_
#define _EVENT_H_

// Zero argument Event
class FUEvent0
{
private:
	typedef IFunctor0<void> THandler;
	typedef vector<THandler*> THandlerList;
	THandlerList m_xHandlers;

public:
	FUEvent0() {}
	~FUEvent0()
	{
		CLEAR_POINTER_VECTOR(m_xHandlers);
	}

	size_t GetHandlerCount() { return m_xHandlers.size(); }

	void InsertHandler(THandler* pFunctor)
	{
		m_xHandlers.push_back(pFunctor);
	}

	void RemoveHandler(void* pObject, void* pFunction)
	{
		THandlerList::iterator it;
		for (it = m_xHandlers.begin(); it != m_xHandlers.end(); ++it)
		{
			if ((*it)->Compare(pObject, pFunction))
			{
				delete (*it);
				m_xHandlers.erase(it);
                break;
			}
		}
	}

	void operator()()
	{
		THandlerList::iterator it;
		for (it = m_xHandlers.begin(); it != m_xHandlers.end(); ++it)
		{
			(*(*it))();
		}
	}
};

// One argument Event
template <typename Arg1>
class FUEvent1
{
private:
	typedef IFunctor1<Arg1, void> THandler;
	typedef vector<THandler> THandlerList;
	THandlerList m_xHandlers;

public:
	FUEvent1() {}

	~FUEvent1()
	{
		assert(m_xHandlers.empty());
	}

	size_t GetHandlerCount() { return m_xHandlers.size(); }

	void InsertHandler(THandler* pFunctor)
	{
		m_xHandlers.push_back(pFunctor);
	}

	void RemoveHandler(void* pObject, void* pFunction)
	{
		typename THandlerList::iterator it;
		for (it = m_xHandlers.begin(); it != m_xHandlers.end(); ++it)
		{
			if ((*it)->Compare(pObject, pFunction))
			{
				delete (*it);
				m_xHandlers.erase(it);
                break;
			}
		}
	}

	void operator()(Arg1 sArgument1)
	{
		typename THandlerList::iterator it;
		for (it = m_xHandlers.begin(); it != m_xHandlers.end(); ++it)
		{
			(*(*it))(sArgument1);
		}
	}
};

// Two arguments Event
template <typename Arg1, typename Arg2>
class FUEvent2
{
private:
	typedef IFunctor2<Arg1, Arg2, void> THandler;
	typedef vector<THandler*> THandlerList;
	THandlerList m_xHandlers;

public:
	FUEvent2() {}

	~FUEvent2()
	{
		assert(m_xHandlers.empty());
	}

	size_t GetHandlerCount() { return m_xHandlers.size(); }

	void InsertHandler(THandler* pFunctor)
	{
		m_xHandlers.push_back(pFunctor);
	}

	void RemoveHandler(void* pObject, void* pFunction)
	{
		typename THandlerList::iterator it;
		for (it = m_xHandlers.begin(); it != m_xHandlers.end(); ++it)
		{
			if ((*it)->Compare(pObject, pFunction))
			{
				delete (*it);
				m_xHandlers.erase(it);
				break;
			}
		}
	}

	void operator()(Arg1 sArgument1, Arg2 sArgument2)
	{
		typename THandlerList::iterator it;
		for (it = m_xHandlers.begin(); it != m_xHandlers.end(); ++it)
		{
			(*(*it))(sArgument1, sArgument2);
		}
	}
};

// Macro for member function pointer type bypass
template <typename Class>
class FURemoveHandler0
{
public:
	void operator() (FUEvent0* event, Class* pObject, void (Class::*pFunction)(void))
	{
		void* pVoid = *(void**)&pFunction;
		event->RemoveHandler(pObject, pVoid);
	}
};

#endif // _EVENT_H_
