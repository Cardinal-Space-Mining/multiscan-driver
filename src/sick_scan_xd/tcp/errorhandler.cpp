/**
 * ErrorHandler.
 *
 */
#include <cstdio>      // for printf() and fprintf()
#include <cstdlib>     // for atoi() and exit()
// #include <string.h>     // for memset()
//#include <backward/iostream.h>	// fuer cout()
// #include "pthread.h"
#include <mutex>

#include "errorhandler.hpp"
#include "Time.hpp"


// Print mutex to print thread-safe
// pthread_mutex_t m_printMutex = PTHREAD_MUTEX_INITIALIZER;
std::mutex m_printMutex;

/**
    * Der Name ist Programm...
    */
void doNothing()
{
}


/**
    * Fehler-"behandlung": Schreibe die Fehlermeldung und beende das Programm.
    */
void dieWithError(const std::string& errorMessage)
{
    {
    std::lock_guard<std::mutex> printGuard(m_printMutex);

    // Mutex setzen
    // pthread_mutex_lock(&m_printMutex);
    
    // Nachricht schreiben
    printError(errorMessage);
    
    // Mutex wieder freigeben
    // pthread_mutex_unlock(&m_printMutex);
    }

    // Programm abbrechen
    exit(1);
}

/**
    * Info-Text auf die Ausgabe schreiben.
    */
void infoMessage(const std::string& message, bool print)
{
    if (print)
    {
#ifndef ROSSIMU
        Time t = Time::now();
#endif	
        std::lock_guard<std::mutex> printGuard(m_printMutex);
        // Mutex setzen
        // pthread_mutex_lock(&m_printMutex);
        
        // Nachricht schreiben
#ifndef ROSSIMU
        printf("%s ", t.toString().c_str());
#endif	
        printf ("Info: %s\n", message.c_str());
        fflush(nullptr);

        // Mutex wieder freigeben
        // pthread_mutex_unlock(&m_printMutex);
    }
}



//
// Warnungs-Text auf die Ausgabe schreiben.
//
void printWarning(const std::string& message)
{
#ifndef ROSSIMU
    Time t = Time::now();
#endif	
    std::lock_guard<std::mutex> printGuard(m_printMutex);
    // Mutex setzen
    // pthread_mutex_lock(&m_printMutex);
        
#ifndef ROSSIMU
    printf ("%s ", t.toString().c_str());
#endif
    printf ("Warning: %s\n", message.c_str());
    fflush(nullptr);
        
    // Mutex wieder freigeben
    // pthread_mutex_unlock(&m_printMutex);
}

//
// Fehler-Text auf die Ausgabe schreiben.
//
void printError(const std::string& message)
{
#ifndef ROSSIMU
    Time t = Time::now();
#endif	
    
    std::lock_guard<std::mutex> printGuard(m_printMutex);
    // Mutex setzen
    // pthread_mutex_lock(&m_printMutex);
        
#ifndef ROSSIMU
    printf("%s ", t.toString().c_str());
#endif
    printf ("ERROR: %s\n", message.c_str());
    fflush(nullptr);
    
    // Mutex wieder freigeben
    // pthread_mutex_unlock(&m_printMutex);
}
