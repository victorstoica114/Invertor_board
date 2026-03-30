# Growatt BMS Protocol Task Header

#ifndef GROWATT_BMS_TASK_H
#define GROWATT_BMS_TASK_H

// Register mappings
#define GROWATT_BMS_REG_STATUS       0x00
#define GROWATT_BMS_REG_VOLTAGE      0x01
#define GROWATT_BMS_REG_CURRENT      0x02
#define GROWATT_BMS_REG_TEMP         0x03

// Task interface
void growatt_bms_init();
void growatt_bms_run();

#endif // GROWATT_BMS_TASK_H