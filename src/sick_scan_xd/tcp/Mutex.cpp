/**
 * \file Mutex.cpp
 */

#include "Mutex.hpp"
#include "errorhandler.hpp"

// ****************************************************************************
//  ScopedLock
// ****************************************************************************

// Konstruktor.
// Lock sperren.
ScopedLock::ScopedLock(Mutex* mutexPtr) : m_mutexPtr(mutexPtr)
{
	
	if (m_mutexPtr != nullptr)
	{
		m_mutexPtr->lock();
	}
}

// Destruktor.
// Lock wieder freigeben.
ScopedLock::~ScopedLock()
{
	if (m_mutexPtr != nullptr)
	{
		m_mutexPtr->unlock();
	}
}



// ****************************************************************************
//  Mutex
// ****************************************************************************
Mutex::Mutex()
{
	//pthread_mutex_init (&m_mutex, NULL);
}

Mutex::~Mutex()
= default;

void Mutex::lock()
{
	m_mutex.lock();
	//pthread_mutex_lock(&m_mutex);
}

void Mutex::unlock()
{
	m_mutex.unlock();
	// pthread_mutex_unlock(&m_mutex);
}
