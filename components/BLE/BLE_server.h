#ifndef _BLE_H
#define _BLE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#pragma once
#define PROFILE_NUM                 2
#define PROFILE_APP_IDX             0
#define PROFILE_HR_APP_ID 0
#define PROFILE_BAT_APP_ID 1

#define ESP_APP_ID                  0x55
#define DEVICE_NAME                 "ECG_BLE"
#define GATTS_TAG                   "GATTS_TABLE"
#define HR_SVC_INST_ID                  0
#define BAT_SVC_INST_ID                 1

#define TEST_MANUFACTURER_DATA_LEN  17
#define PREPARE_BUF_MAX_SIZE 1024

#define ADV_CONFIG_FLAG      (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)


typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;

    uint16_t app_id;
    uint16_t conn_id;
    
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;

    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;

    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;

    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

typedef struct indicate_notify_t {
    uint16_t conn_id;
    uint16_t attr_handle; 
    uint16_t value_len; 
    uint8_t *value; 
    uint8_t mode; // notify mode or indicate mode
    bool need_confirm;
    esp_gatt_if_t gatt_if;
} indicate_notify_t;


// this is struct hold data to communicate with other library file
typedef struct BLE_server_common_obj_t {
    bool isConnected;
    bool isBLEEnable;
}BLE_server_common_obj_t;

void BLE_init();
esp_err_t BLE_enable();
esp_err_t BLE_disable();
#endif