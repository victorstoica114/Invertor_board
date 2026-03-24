// Pylon BMS Protocol Task Header
#ifndef PYLON_BMS_TASK_H
#define PYLON_BMS_TASK_H

// Defines and function prototypes for Pylon BMS Protocol Tasks

// Initialize Pylon BMS task
void Pylon_BMS_Init();

// Function for handling BMS data
void Pylon_BMS_HandleData();

// Function for sending commands to the BMS
void Pylon_BMS_SendCommand();

#endif // PYLON_BMS_TASK_H