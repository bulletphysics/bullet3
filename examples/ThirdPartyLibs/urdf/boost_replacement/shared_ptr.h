/*
Bullet Continuous Collision Detection and Physics Library Maya Plugin
Copyright (c) 2008 Walt Disney Studios
 
This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising
from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:
 
1. The origin of this software must not be misrepresented; you must
not claim that you wrote the original software. If you use this
software in a product, an acknowledgment in the product documentation
would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must
not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 
Written by: Nicola Candussi <nicola@fluidinteractive.com>

Modified by Francisco Gochez
Dec 2011 - Added deferencing operator
*/

//my_shared_ptr

#ifndef DYN_SHARED_PTR_H
#define DYN_SHARED_PTR_H

#define DYN_SHARED_PTR_THREAD_SAFE


#include <assert.h>

#ifdef _WIN32
#include <windows.h>

class my_shared_count {
public:
    my_shared_count(): m_count(1) {   }
    ~my_shared_count() {    }

    long increment()
    {
#ifdef DYN_SHARED_PTR_THREAD_SAFE
        return InterlockedIncrement(&m_count);
#else
        return ++m_count;
#endif
    }

    long decrement() {
#ifdef DYN_SHARED_PTR_THREAD_SAFE
        return InterlockedDecrement(&m_count);
#else
        return ++m_count;
#endif
    }

    long use_count() { return m_count;  }

private:
    long m_count;
};
#else //ifdef WIN32


#include <pthread.h>

class my_shared_count {
public:
    my_shared_count(): m_count(1) {
#ifdef DYN_SHARED_PTR_THREAD_SAFE
        pthread_mutex_init(&m_mutex, 0);
#endif
    }
    ~my_shared_count() {
#ifdef DYN_SHARED_PTR_THREAD_SAFE
        pthread_mutex_destroy(&m_mutex);
#endif
    }

    long increment()
    {
#ifdef DYN_SHARED_PTR_THREAD_SAFE
        pthread_mutex_lock(&m_mutex);
#endif
        long c = ++m_count;  
#ifdef DYN_SHARED_PTR_THREAD_SAFE
        pthread_mutex_unlock(&m_mutex);
#endif
        return c;
     }

    long decrement() {
#ifdef DYN_SHARED_PTR_THREAD_SAFE
        pthread_mutex_lock(&m_mutex);
#endif
        long c = --m_count;  
#ifdef DYN_SHARED_PTR_THREAD_SAFE
        pthread_mutex_unlock(&m_mutex);
#endif
        return c;
    }

    long use_count() { return m_count;  }

private:
    long m_count;
    mutable pthread_mutex_t m_mutex;
};

#endif


template<typename T>
class my_shared_ptr
{
public:
    my_shared_ptr(): m_ptr(NULL), m_count(NULL) { }
    my_shared_ptr(my_shared_ptr<T> const& other):
            m_ptr(other.m_ptr),
            m_count(other.m_count)
    {
        if(other.m_count != NULL) other.m_count->increment();
    }

    template<typename U>
    my_shared_ptr(my_shared_ptr<U> const& other):
            m_ptr(other.m_ptr),
            m_count(other.m_count)
    {
        if(other.m_count != NULL)  other.m_count->increment();
    }

    my_shared_ptr(T const* other): m_ptr(const_cast<T*>(other)), m_count(NULL)
    {
        if(other != NULL) m_count = new my_shared_count;
    }

    ~my_shared_ptr() 
    {
        giveup_ownership();
    }

    void reset(T const* other) 
    {
        if(m_ptr == other) return;
        giveup_ownership();
        m_ptr = const_cast<T*>(other);
        if(other != NULL) m_count = new my_shared_count;
        else m_count = NULL;
    }

    T* get() { return m_ptr; }
    T const* get() const { return m_ptr; }
    T* operator->() {   return m_ptr; }
    T const* operator->() const { return m_ptr; }
    operator bool() const { return m_ptr != NULL; }
	T& operator*() const
	{ 
		assert(m_ptr != 0);
		return *m_ptr;
	}

    bool operator<(my_shared_ptr<T> const& rhs) const { return m_ptr < rhs.m_ptr; }

    my_shared_ptr<T>& operator=(my_shared_ptr<T> const& other) {
        if(m_ptr == other.m_ptr) return *this;
        giveup_ownership();
        m_ptr = other.m_ptr;
        m_count = other.m_count;
        if(other.m_count != NULL) m_count->increment();
        return *this;
    }

    template<typename U> 
    my_shared_ptr<T>& operator=(my_shared_ptr<U>& other) {
        if(m_ptr == other.m_ptr) return *this;
        giveup_ownership();
        m_ptr = other.m_ptr;
        m_count = other.m_count;
        if(other.m_count != NULL) m_count->increment();
        return *this;
    }

protected:

    template<typename U> friend class my_shared_ptr;
    void giveup_ownership()
    {
        if(m_count != NULL) {
            if( m_count->decrement() == 0) {
                delete m_ptr;
                m_ptr = NULL;
                delete m_count;
                m_count = NULL;
            }
        }
    }

protected:
    T               *m_ptr;
    my_shared_count    *m_count;

};


#endif
