/**
 * Fehler-Handler.
 *
 */
#pragma once

#include <stdexcept>	// for throw

#include "BasicDatatypes.hpp"


#define printInfoMessage(a, b)  ((b) ? infoMessage(a, b):doNothing())

// Fehler-"behandlung": Schreibe die Fehlermeldung und beende das Programm.
void dieWithError(const std::string& errorMessage);

void infoMessage(const std::string& message, bool print = true);

void printWarning(const std::string& message);

void printError(const std::string& message);

void doNothing();
