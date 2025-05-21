#ifndef BLE_H
#define BLE_H

#include <stdint.h>
#include <stdbool.h>
#include "app_data_types.h"

#define BLE_MSG_ID_CONNECTED                0x01
#define BLE_MSG_ID_DISCONNECTED             0x02
#define BLE_MSG_ID_REINIT                   0x03
#define BLE_MSG_ID_DEINIT                   0x04

#define BLE_MSG_ID_EMS_CONTROL              0x20
#define BLE_MSG_ID_EMS_SESSION              0x21
#define BLE_MSG_ID_EMS_MODE                 0x22
#define BLE_MSG_ID_EMS_CHAN1_CONTROL        0x23
#define BLE_MSG_ID_EMS_CHAN2_CONTROL        0x24
#define BLE_MSG_ID_EMS_CHAN3_CONTROL        0x25
#define BLE_MSG_ID_EMS_CHAN4_CONTROL        0x26
#define BLE_MSG_ID_EMS_INTENSITY            0x27

#define BLE_MSG_ID_CHAN0_PLAYBACK_COUNT     0x28
#define BLE_MSG_ID_CHAN0_INUCT_DURATION     0x29
#define BLE_MSG_ID_CHAN0_ON_DURATION        0x30
#define BLE_MSG_ID_CHAN0_DURATION           0x31

#define BLE_MSG_ID_PWM_PERIOD               0x33
#define BLE_MSG_ID_PWM_ONTIME               0x34
#define BLE_MSG_ID_PWM_INTERVAL             0x35
#define BLE_MSG_ID_PWM_DURATION             0x36
#define BLE_MSG_ID_CHAN23_PWM_PERIOD_NS     0x37
#define BLE_MSG_ID_CHAN23_PWM_ON_TIME_NS    0x38

#define BLE_MSG_ID_CHARGER_ON_OFF           0x39
#define BLE_MSG_ID_BATT_UPDATE              0x40

extern uint8_t disconnect_reason;

int ble_adv_start();

void ble_new_adv();

extern bool pairing_on;

// Structure to store the ble ems data
struct ble_msg{
    uint8_t id;
    union{
        struct ems_def ems;
        int32_t batt_mv;
    }data;
};

#endif