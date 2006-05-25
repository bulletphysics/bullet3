/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/
/*
	This file was taken off the Protect project on 26-09-2005
*/

#ifndef _FUNCTOR_H_
#define _FUNCTOR_H_

// Functor with no argument.
template<typename ReturnType>
class IFunctor0
{
public:
	virtual ~IFunctor0() {}
	virtual ReturnType operator()() const = 0;
	virtual bool Compare(void* pObject, void* pFunction) const = 0;
};

template<typename Class, typename ReturnType>
class FUFunctor0 : public IFunctor0<ReturnType>
{
private:
	Class* m_pObject;
	ReturnType (Class::*m_pFunction) ();

public:
	FUFunctor0(Class* pObject, ReturnType (Class::*pFunction) ()) { m_pObject = pObject; m_pFunction = pFunction; }

	virtual ReturnType operator()() const
	{ return ((*m_pObject).*m_pFunction)(); }

	virtual bool Compare(void* pObject, void* pFunction) const
	{ return pObject == m_pObject && (unsigned long)pFunction == *(unsigned long*)&m_pFunction; }
};

template<typename ReturnType>
class FUStaticFunctor0 : public IFunctor0<ReturnType>
{
private:
	ReturnType (*m_pFunction) ();

public:
	FUStaticFunctor0(ReturnType (*pFunction) ()) { m_pFunction = pFunction; }

	virtual ReturnType operator()() const
	{ return (*m_pFunction)(); }

	virtual bool Compare(void*, void* pFunction) const
	{ return (unsigned long)pFunction == *(unsigned long*)&m_pFunction; }
};

// Functor with one argument.
template<typename Arg1, typename ReturnType>
class IFunctor1
{
public:
	virtual ~IFunctor1() {}
	virtual ReturnType operator()(Arg1 sArgument1) const = 0;
	virtual bool Compare(void* pObject, void* pFunction) const = 0;
};

template<typename Class, typename Arg1, typename ReturnType>
class FUFunctor1 : public IFunctor1<Arg1, ReturnType>
{
private:
	Class* m_pObject;
	ReturnType (Class::*m_pFunction) (Arg1);

public:
	FUFunctor1(Class* pObject, ReturnType (Class::*pFunction) (Arg1)) { m_pObject = pObject; m_pFunction = pFunction; }

	virtual ReturnType operator()(Arg1 sArgument1) const
	{ return ((*m_pObject).*m_pFunction)(sArgument1); }

	virtual bool Compare(void* pObject, void* pFunction) const
	{ return pObject == m_pObject && (unsigned long)pFunction == *(unsigned long*)&m_pFunction; }
};

template<typename Arg1, typename ReturnType>
class FUStaticFunctor1 : public IFunctor1<Arg1, ReturnType>
{
private:
	ReturnType (*m_pFunction) (Arg1);

public:
	FUStaticFunctor1(ReturnType (*pFunction) (Arg1)) { m_pFunction = pFunction; }

	virtual ReturnType operator()(Arg1 sArgument1) const
	{ return (*m_pFunction)(sArgument1); }

	virtual bool Compare(void*, void* pFunction) const
	{ return (unsigned long)pFunction == *(unsigned long*)&m_pFunction; }
};

// Functor with two argument.
template<typename Arg1, typename Arg2, typename ReturnType>
class IFunctor2
{
public:
	virtual ~IFunctor2() {}
	virtual ReturnType operator()(Arg1 sArgument1, Arg2 sArgument2) const = 0;
	virtual bool Compare(void* pObject, void* pFunction) const = 0;
};

template<typename Class, typename Arg1, typename Arg2, typename ReturnType>
class FUFunctor2 : public IFunctor2<Arg1, Arg2, ReturnType>
{
private:
	Class* m_pObject;
	ReturnType (Class::*m_pFunction) (Arg1, Arg2);

public:
	FUFunctor2(Class* pObject, ReturnType (Class::*pFunction) (Arg1, Arg2)) { m_pObject = pObject; m_pFunction = pFunction; }

	virtual ReturnType operator()(Arg1 sArgument1, Arg2 sArgument2) const
	{ return ((*m_pObject).*m_pFunction)(sArgument1, sArgument2); }

	virtual bool Compare(void* pObject, void* pFunction) const
	{ return pObject == m_pObject && (unsigned long)pFunction == *(unsigned long*)&m_pFunction; }
};

template<typename Arg1, typename Arg2, typename ReturnType>
class FUStaticFunctor2 : public IFunctor2<Arg1, Arg2, ReturnType>
{
private:
	ReturnType (*m_pFunction) (Arg1, Arg2);

public:
	FUStaticFunctor2(ReturnType (*pFunction) (Arg1, Arg2)) { m_pFunction = pFunction; }

	virtual ReturnType operator()(Arg1 sArgument1, Arg2 sArgument2) const
	{ return (*m_pFunction)(sArgument1, sArgument2); }

	virtual bool Compare(void*, void* pFunction) const
	{ return (unsigned long)pFunction == *(unsigned long*)&m_pFunction; }
};

// Macro for member function pointer type bypass
#define FUNCTOR_COMPARE(sFunctor, pObject, pFunction, bRet) \
{ \
	void* pVoid; \
	__asm { __asm push eax __asm mov eax, pFunction __asm mov pVoid, eax __asm pop eax } \
	bRet = sFunctor.Compare(pObject, pVoid); \
}

#endif //_FUNCTOR_H_
