
#include <sys/param.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"

#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "GATTS_TABLE.h"
#include "BLE_server.h"

#define BLE_TAG "BLE"
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 500
#define PREPARE_BUF_MAX_SIZE        1024
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define BLE_NOTIFY_EN 1
#define BLE_INDICATE_EN 2
#define BLE_NOTIFY_INDICATE_DISABLE 0

static uint8_t adv_config_done = 0;
uint16_t heart_rate_handle_table[HRTotalIdx];
uint16_t battery_handle_table[BATTotalIdx];
extern BLE_server_common_obj_t BSCO ={.isBLEEnable = false, .isConnected = false};
extern indicate_notify_t   ind_nof_HR = {.mode = BLE_NOTIFY_INDICATE_DISABLE,},
                    ind_nof_BAT = {.mode = BLE_NOTIFY_INDICATE_DISABLE};
static uint8_t adv_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value for HR profile
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
    //second uuid, 32bit, [12], [13], [14], [15] is the value for BAT profile
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};
// Khai báo biến cấu hình thông tin hiển thị data advertise
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// Khai báo biến cấu hình thông tin hiển thị data scan respone
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    //.min_interval = 0x0006,
    //.max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// Khai báo biến cấu hình phương thức truyền dữ liệu giữa 2 bên
static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};



/* Service */
static const uint16_t GATTS_SERVICE_UUID_HR         = ESP_GATT_UUID_HEART_RATE_SVC;
static const uint16_t GATTS_CHAR_UUID_HR_MEASURE    = ESP_GATT_HEART_RATE_MEAS;
static const uint16_t GATTS_CHAR_UUID_CTRL_P        = ESP_GATT_HEART_RATE_CNTL_POINT;
static const uint8_t HR_measure_client_conf[2]      = {0x00, 0x00};
static const uint8_t HR_char_value[4]                 = {0x11, 0x22, 0x33, 0x44};

static const uint16_t GATTS_SERVICE_UUID_BAT         = ESP_GATT_UUID_BATTERY_SERVICE_SVC;
static const uint16_t GATTS_CHAR_UUID_BAT_LELVEL    = ESP_GATT_UUID_BATTERY_LEVEL;
static const uint8_t BAT_level_client_conf[2]      = {0x00, 0x00};
static const uint8_t BAT_char_value[4]                 = {0x11, 0x22, 0x33, 0x44};

static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read                =  ESP_GATT_CHAR_PROP_BIT_READ;
// static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE 
                                                    | ESP_GATT_CHAR_PROP_BIT_READ                     
                                                    | ESP_GATT_CHAR_PROP_BIT_NOTIFY
                                                    ;

static const esp_gatts_attr_db_t gatt_database_HR[HRTotalIdx] =
{
    // Heart Rate Service Declaration
    [HRServiceIdx]        =
    {   
        {ESP_GATT_AUTO_RSP}, 
        {
            ESP_UUID_LEN_16, 
            (uint8_t *)&primary_service_uuid, 
            ESP_GATT_PERM_READ,
            sizeof(uint16_t), 
            sizeof(GATTS_SERVICE_UUID_HR), 
            (uint8_t *)&GATTS_SERVICE_UUID_HR
        }
    },

    /* Heart Rate Characteristic Heart Rate Measurement Declaration */
    [HRCharMeasureIdx]     =
    {
        {ESP_GATT_AUTO_RSP}, 
        {
            ESP_UUID_LEN_16, 
            (uint8_t *)&character_declaration_uuid, 
            ESP_GATT_PERM_READ,
            CHAR_DECLARATION_SIZE, 
            CHAR_DECLARATION_SIZE, 
            (uint8_t *)&char_prop_read_write_notify
        }
    },

    /* Heart Rate Characteristic Heart Rate Measurement value */
    [HRCharMeasureValIdx] =
    {
        {ESP_GATT_AUTO_RSP}, 
        {
            ESP_UUID_LEN_16, 
            (uint8_t *)&GATTS_CHAR_UUID_HR_MEASURE, 
            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            GATTS_DEMO_CHAR_VAL_LEN_MAX, 
            sizeof(HR_char_value), 
            (uint8_t *)HR_char_value
        }
    },

    /* Heart Rate Characteristic Control Point Declaration */
    [HRCharCtrlPointIdx]      =
    {
        {ESP_GATT_AUTO_RSP}, 
        {
            ESP_UUID_LEN_16, 
            (uint8_t *)&character_declaration_uuid, 
            ESP_GATT_PERM_READ,
            CHAR_DECLARATION_SIZE, 
            CHAR_DECLARATION_SIZE, 
            (uint8_t *)&char_prop_read
        }
    },

    /* Heart Rate Characteristic Control Point value */
    [HRCharCtrlPointValIdx]  =
    {
        {ESP_GATT_AUTO_RSP}, 
        {
            ESP_UUID_LEN_16, 
            (uint8_t *)&GATTS_CHAR_UUID_CTRL_P, 
            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            GATTS_DEMO_CHAR_VAL_LEN_MAX, 
            sizeof(HR_char_value), 
            (uint8_t *)HR_char_value
        }
    },

    /* Heart Rate Client Characteristic Configuration Descriptor */
    [HRCCCIdx]  =
    {
        {ESP_GATT_AUTO_RSP}, 
        {
            ESP_UUID_LEN_16, 
            (uint8_t *)&character_client_config_uuid, 
            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            sizeof(uint16_t), 
            sizeof(HR_measure_client_conf), 
            (uint8_t *)HR_measure_client_conf
        }
    },
};

static const esp_gatts_attr_db_t gatt_database_BAT[BATTotalIdx] =
{
    // Battery Service Declaration
    [BATServiceIdx]        =
    {   
        {ESP_GATT_AUTO_RSP}, 
        {
            ESP_UUID_LEN_16, 
            (uint8_t *)&primary_service_uuid, 
            ESP_GATT_PERM_READ,
            sizeof(uint16_t), 
            sizeof(GATTS_SERVICE_UUID_BAT), 
            (uint8_t *)&GATTS_SERVICE_UUID_BAT
        }
    },

    /* Battery Characteristic Declaration */
    [BATCharLevelIdx]     =
    {
        {ESP_GATT_AUTO_RSP}, 
        {
            ESP_UUID_LEN_16, 
            (uint8_t *)&character_declaration_uuid, 
            ESP_GATT_PERM_READ,
            CHAR_DECLARATION_SIZE, 
            CHAR_DECLARATION_SIZE, 
            (uint8_t *)&char_prop_read_write_notify
        }
    },

    /* Battery Characteristic Battery level value */
    [BATCharLevelValIdx] =
    {
        {ESP_GATT_AUTO_RSP}, 
        {
            ESP_UUID_LEN_16, 
            (uint8_t *)&GATTS_CHAR_UUID_BAT_LELVEL, 
            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            GATTS_DEMO_CHAR_VAL_LEN_MAX, 
            sizeof(BAT_char_value), 
            (uint8_t *)BAT_char_value
        }
    },

    /* Battery Client Characteristic Configuration Descriptor */
    [BATCCCIdx]  =
    {
        {ESP_GATT_AUTO_RSP}, 
        {
            ESP_UUID_LEN_16, 
            (uint8_t *)&character_client_config_uuid, 
            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            sizeof(uint16_t), sizeof(BAT_level_client_conf), 
            (uint8_t *)BAT_level_client_conf
        }
    },
};

static prepare_type_env_t HR_prepare_write_env;
static prepare_type_env_t BAT_prepare_write_env;

// 2 Function prototype xử lý sự kiện để gatts_cb thuộc GATT Profile trỏ tới 
static void gatts_profile_HR_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_profile_BAT_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
/**
 * @brief Tạo ra 2 profile là Heart Rate Profile và Battery Level Profile
 * gatt_cb lần lượt trỏ tới các hàm xử lý sự kiện là gatts_profile_HR_event_handler 
 * và gatts_profile_BAT_event_handler
 * ban đầu chưa nhận được phương thức giao tiếp nên cho gatt_if = Interface none 
 */

static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_HR_APP_ID] = {
        .gatts_cb = gatts_profile_HR_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
    [PROFILE_BAT_APP_ID] = {
        .gatts_cb = gatts_profile_BAT_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};



void prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TAG, "prepare write, handle = %d, value len = %d", param->write.handle, param->write.len);
    esp_gatt_status_t status = ESP_GATT_OK;
    if (prepare_write_env->prepare_buf == NULL) {
        prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
        prepare_write_env->prepare_len = 0;
        if (prepare_write_env->prepare_buf == NULL) {
            ESP_LOGE(GATTS_TAG, "%s, Gatt_server prep no mem", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    } else {
        if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_OFFSET;
        } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_ATTR_LEN;
        }
    }
    /*send response when param->write.need_rsp is true */
    if (param->write.need_rsp){
        esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
        if (gatt_rsp != NULL){
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK){
               ESP_LOGE(GATTS_TAG, "Send response error");
            }
            free(gatt_rsp);
        }else{
            ESP_LOGE(GATTS_TAG, "%s, malloc failed", __func__);
        }
    }
    if (status != ESP_GATT_OK){
        return;
    }
    memcpy(prepare_write_env->prepare_buf + param->write.offset,
           param->write.value,
           param->write.len);
    prepare_write_env->prepare_len += param->write.len;

}

void exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf){
        esp_log_buffer_hex(GATTS_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }else{
        ESP_LOGI(GATTS_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_profile_HR_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event){
        // Sự kiện đăng ký app ID xảy ra, thực hiện tạo service có tên là Heart Rate service
        case ESP_GATTS_REG_EVT:{
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(DEVICE_NAME);
            if (set_dev_name_ret){
                ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }
            esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret){
                ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            //config scan response data
            ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
            if (ret){
                ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;


            esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_database_HR, gatts_if, HRTotalIdx, HR_SVC_INST_ID);
            if (create_attr_ret){
                ESP_LOGE(GATTS_TAG, "create attr table failed, error code = %x", create_attr_ret);
            }
            break;
        }

        case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
            if (param->add_attr_tab.status != ESP_GATT_OK){
                ESP_LOGE(GATTS_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != HRTotalIdx){
                ESP_LOGE(GATTS_TAG, "create attribute table abnormally, num_handle (%u) \
                        doesn't equal to HRTotalIdx(%d)", param->add_attr_tab.num_handle, HRTotalIdx);
            }
            else {
                ESP_LOGI(GATTS_TAG, "create attribute table successfully, the number handle = %u\n",param->add_attr_tab.num_handle);
                memcpy(heart_rate_handle_table, param->add_attr_tab.handles, sizeof(heart_rate_handle_table));
                esp_ble_gatts_start_service(heart_rate_handle_table[HRServiceIdx]);
            }
            break;
        }
        
        case ESP_GATTS_START_EVT:{
            ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %u", param->start.status, param->start.service_handle);
            break;
        }
        
        case ESP_GATTS_WRITE_EVT:{
            if (!param->write.is_prep){
                // the data length of gattc write  must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
                ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, handle = %u, value len = %u, value :", param->write.handle, param->write.len);
                esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
                if (heart_rate_handle_table[HRCCCIdx] == param->write.handle && param->write.len == 2){
                    uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                    if (descr_value == 0x0001){
                        ESP_LOGI(GATTS_TAG, "notify enable");
                        ind_nof_HR.gatt_if = gatts_if;
                        ind_nof_HR.mode = BLE_NOTIFY_EN;
                        ind_nof_HR.attr_handle = heart_rate_handle_table[HRCharMeasureValIdx];
                        ind_nof_HR.need_confirm = false;
                    }else if (descr_value == 0x0002){
                        ESP_LOGI(GATTS_TAG, "indicate enable");
                        ind_nof_HR.gatt_if = gatts_if;
                        ind_nof_HR.mode = BLE_NOTIFY_EN;
                        ind_nof_HR.attr_handle = heart_rate_handle_table[HRCharMeasureValIdx];
                        ind_nof_HR.need_confirm = true;
                    }
                    else if (descr_value == 0x0000){
                        ESP_LOGI(GATTS_TAG, "notify/indicate disable ");
                        ind_nof_HR.mode = BLE_NOTIFY_INDICATE_DISABLE;
                    }else{
                        ESP_LOGE(GATTS_TAG, "unknown descr value");
                        esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
                    }

                }
                /* send response when param->write.need_rsp is true*/
                if (param->write.need_rsp){
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }else{
                /* handle prepare write */
                prepare_write_event_env(gatts_if, &HR_prepare_write_env, param);
            }
            break;
        }
        
        case ESP_GATTS_EXEC_WRITE_EVT:{
            ESP_LOGI(GATTS_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            exec_write_event_env(&HR_prepare_write_env, param);
            break;
        }
        
        case ESP_GATTS_MTU_EVT:{
            ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %u", param->mtu.mtu);
            break;
        }
        
        case ESP_GATTS_READ_EVT:{
            ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %u, trans_id %lu, handle %u\n", param->read.conn_id, param->read.trans_id, param->read.handle);
            break;
        }
        
        case ESP_GATTS_DISCONNECT_EVT:{
            ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
            esp_ble_gap_start_advertising(&adv_params);
            ind_nof_HR.mode = BLE_NOTIFY_INDICATE_DISABLE;
            BSCO.isConnected = false;
            break;
         }
        
        case ESP_GATTS_CONNECT_EVT:{
            ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %u", param->connect.conn_id);
            esp_log_buffer_hex(GATTS_TAG, param->connect.remote_bda, 6);
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);
            BSCO.isConnected = true;
            break;
         }
        
        case ESP_GATTS_CONF_EVT:{
            ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONF_EVT, status %d attr_handle %u", param->conf.status, param->conf.handle);
            if (param->conf.status != ESP_GATT_OK){
                esp_log_buffer_hex(GATTS_TAG, param->conf.value, param->conf.len);
            }
            break;
        }
        
        // case ESP_GATTS_UNREG_EVT:break;
        // case ESP_GATTS_ADD_INCL_SRVC_EVT:break;
        // case ESP_GATTS_DELETE_EVT:break;
        // case ESP_GATTS_STOP_EVT:break;
        // case ESP_GATTS_OPEN_EVT:
        // case ESP_GATTS_CANCEL_OPEN_EVT:
        // case ESP_GATTS_CLOSE_EVT:
        // case ESP_GATTS_LISTEN_EVT:
        // case ESP_GATTS_CONGEST_EVT:
        default: break;
    }
}

static void gatts_profile_BAT_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event){
    // Sự kiện đăng ký app ID xảy ra, thực hiện tạo service có tên là Battery service
        case ESP_GATTS_REG_EVT:{
            esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_database_BAT, gatts_if, BATTotalIdx, BAT_SVC_INST_ID);
            if (create_attr_ret){
                ESP_LOGE(GATTS_TAG, "create attr table failed, error code = %x", create_attr_ret);
            }
            break;
        }
        
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
            if (param->add_attr_tab.status != ESP_GATT_OK){
                ESP_LOGE(GATTS_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != BATTotalIdx){
                ESP_LOGE(GATTS_TAG, "create attribute table abnormally, num_handle (%u) \
                        doesn't equal to BATTotalIdx(%d)", param->add_attr_tab.num_handle, BATTotalIdx);
            }
            else {
                ESP_LOGI(GATTS_TAG, "create attribute table successfully, the number handle = %u\n",param->add_attr_tab.num_handle);
                memcpy(battery_handle_table, param->add_attr_tab.handles, sizeof(battery_handle_table));
                esp_ble_gatts_start_service(battery_handle_table[BATServiceIdx]);
            }
            break;
        }
        
        case ESP_GATTS_START_EVT:{
            ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %u", param->start.status, param->start.service_handle);
            break;
        }
        
        case ESP_GATTS_WRITE_EVT:{
            if (!param->write.is_prep){
                // the data length of gattc write  must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
                ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
                esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
                if (battery_handle_table[BATCCCIdx] == param->write.handle && param->write.len == 2){
                    uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                    if (descr_value == 0x0001){
                        ESP_LOGI(GATTS_TAG, "notify enable");
                        ind_nof_BAT.gatt_if = gatts_if;
                        ind_nof_BAT.mode = BLE_NOTIFY_EN;
                        ind_nof_BAT.attr_handle = battery_handle_table[BATCharLevelValIdx];
                        ind_nof_BAT.need_confirm = false;
                    }else if (descr_value == 0x0002){
                        ESP_LOGI(GATTS_TAG, "indicate enable");
                        ind_nof_BAT.gatt_if = gatts_if;
                        ind_nof_BAT.mode = BLE_NOTIFY_EN;
                        ind_nof_BAT.attr_handle = battery_handle_table[BATCharLevelValIdx];
                        ind_nof_BAT.need_confirm = true;
                        
                    }
                    else if (descr_value == 0x0000){
                        ESP_LOGI(GATTS_TAG, "notify/indicate disable ");
                        ind_nof_BAT.mode = BLE_NOTIFY_INDICATE_DISABLE;
                    }else{
                        ESP_LOGE(GATTS_TAG, "unknown descr value");
                        esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
                    }

                }
                /* send response when param->write.need_rsp is true*/
                if (param->write.need_rsp){
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }else{
                /* handle prepare write */
                prepare_write_event_env(gatts_if, &BAT_prepare_write_env, param);
            }
            break;
        }
        
        case ESP_GATTS_MTU_EVT:{
            ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        }

        case ESP_GATTS_READ_EVT:{
            ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %u, trans_id %lu, handle %u\n", param->read.conn_id, param->read.trans_id, param->read.handle);
            break;
        }

        case ESP_GATTS_EXEC_WRITE_EVT:{
            ESP_LOGI(GATTS_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            exec_write_event_env(&BAT_prepare_write_env, param);
            break;
        }
        
        case ESP_GATTS_CONNECT_EVT:{
            ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %u", param->connect.conn_id);
            esp_log_buffer_hex(GATTS_TAG, param->connect.remote_bda, 6);
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);
            BSCO.isConnected = true;
            break;
         }
        
        case ESP_GATTS_DISCONNECT_EVT:{
            ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
            esp_ble_gap_start_advertising(&adv_params);
            ind_nof_BAT.mode = BLE_NOTIFY_INDICATE_DISABLE;
            BSCO.isConnected = false;
            break;
        }
        
        case ESP_GATTS_CONF_EVT:{
            ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONF_EVT, status %d attr_handle %u", param->conf.status, param->conf.handle);
            if (param->conf.status != ESP_GATT_OK){
                esp_log_buffer_hex(GATTS_TAG, param->conf.value, param->conf.len);
            }
            break;
        }
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param){
     /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        } else {
            ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d\n",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gatts_if equal to profile HR, call profile HR cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == gl_profile_tab[idx].gatts_if) {
                if (gl_profile_tab[idx].gatts_cb) {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param){
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:{
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
        }
    
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:{
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    }
    
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:{
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising start failed\n");
        }else{
                ESP_LOGI(GATTS_TAG, "advertising start successfully");
        }
        break;
    }
    
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:{
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising stop failed\n");
        } else {
            ESP_LOGI(GATTS_TAG, "Stop adv successfully\n");
        }
        break;
    }
    
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:{
         ESP_LOGI(GATTS_TAG, "update connection params status = %d, min_int = %u, max_int = %u,conn_int = %u,latency = %u, timeout = %u",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    }
    
    default:
        break;
    }
}

void BLE_create_service()
{
    esp_err_t ret;
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(BLE_TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(BLE_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(PROFILE_HR_APP_ID);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts HR register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(PROFILE_BAT_APP_ID);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts BAT register error, error code = %x", ret);
        return;
    }
    
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
}

void BLE_init() 
{
    esp_err_t ret;

    ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND){
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();

    }
    ESP_ERROR_CHECK(ret);
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(BLE_TAG, "%s init controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(BLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    BLE_create_service();
    

    BSCO.isBLEEnable = true;
}



esp_err_t BLE_enable()
{
    esp_err_t ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }
    BLE_create_service();
   BSCO.isBLEEnable = true;
   return ret;
}

esp_err_t BLE_disable()
{
    esp_err_t ret = ESP_OK;
    ret = esp_bluedroid_disable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s disable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_controller_disable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s disable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }
    BSCO.isBLEEnable = false;
    return ret;
}