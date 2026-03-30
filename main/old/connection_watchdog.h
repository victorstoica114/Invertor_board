# Connection Watchdog Header

#ifndef CONNECTION_WATCHDOG_H
#define CONNECTION_WATCHDOG_H

#include <stddef.h>
#include <stdbool.h>

// Function to initialize the connection watchdog
void init_connection_watchdog();

// Function to check the BMS connection
bool check_bms_connection();

// Function to signal to the orchestrator in case of disconnection
void signal_orchestrator_disconnection();

#endif // CONNECTION_WATCHDOG_H
