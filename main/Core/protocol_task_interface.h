#ifndef PROTOCOL_TASK_INTERFACE_H
#define PROTOCOL_TASK_INTERFACE_H

// Define the BMS protocol interface
typedef struct {
    // Add BMS relevant fields here
    int temperature;
    int voltage;
    int current;
} BMSProtocol;

// Define the inverter protocol interface
typedef struct {
    // Add inverter relevant fields here
    int power;
    int frequency;
    int voltage;
} InverterProtocol;

// Function prototypes
void initializeBMS(BMSProtocol *bms);
void initializeInverter(InverterProtocol *inverter);
void updateBMSData(BMSProtocol *bms);
void updateInverterData(InverterProtocol *inverter);

#endif // PROTOCOL_TASK_INTERFACE_H
