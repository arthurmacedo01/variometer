#ifndef BLE_LK8EX1_H
#define BLE_LK8EX1_H

#include "esp_err.h"

// Function to initialize BLE
esp_err_t ble_init(void);

// Function to send LK8EX1 data
void ble_send_lk8ex1(float pressure, float altitude, float vario, float temperature, float battery);

#endif // BLE_LK8EX1_H
