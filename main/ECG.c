#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/uart.h"
#include "esp_adc_cal.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "MAX30003.h"
#include "esp_timer.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "BLE_server.h"

#  define MAX30003_HOST    SPI2_HOST
#  define PIN_NUM_MISO 2
#  define PIN_NUM_MOSI 7
#  define PIN_NUM_CLK  6
#  define PIN_NUM_CS   10
#  define PIN_INTB 0
#  define PIN_BTN 9
#  define PIN_SIGNAL 5
#  define NOT_USE 0
#  define BAT_MIN_VOLT 3400
#  define BAT_MAX_VOLT 4200
#  define BAT_PERCENT_OFFSET 1
#  define ASSERT_BITS(__VAL__,__BIT__) (((__VAL__) & (__BIT__)) == __BIT__ ? 1 : 0)
#define BLE_NOTIFY_EN 1
#define BLE_INDICATE_EN 2
#define BLE_NOTIFY_INDICATE_DISABLE 0

#define BAT_SAMPLE_TIMEOUT           9*1000 
#define BLE_WAIT_CONNECT_TIMEOUT    15*1000
#define MAX_DCLOFF_TIMEOUT          20*1000

#define SLEEP_TIME                  20*1000000

#define EVTB_TASK_INTB              (1 << 0)
#define EVTB_TASK_BTN               (1 << 1)
#define EVTB_TASK_BATTERY           (1 << 2)
#define EVTB_LIGHT_SLEEP            (1 << 3)

#define EVTB_BLE_CONNECTED          (1 << 4)
#define EVTB_BLE_ENABLE             (1 << 5) 
#define EVTB_BLE_BAT_SEND_NOTIFY    (1 << 6) 
#define EVTB_BLE_HR_SEND_NOTIFY     (1 << 7) 

#define EVTB_MAX_LONINT             (1 << 8)
#define EVTB_MAX_CHECK_TIMEOUT      (1 << 9)

static const char TAG[] = "main";

EventGroupHandle_t xEvtGr1;
static esp_adc_cal_characteristics_t adc1_chars;
MAX30003_handle_t MAX30003_handle;


extern BLE_server_common_obj_t BSCO;
extern indicate_notify_t ind_nof_HR,ind_nof_BAT;
uint8_t HR_data_test[2]={0};
uint8_t BAT_data_test[2]={0};
uint32_t t_BLE=0;
uint32_t t_BAT=0;
uint32_t t_MAX=0;
// Funtion Prototype *************************************************************
esp_err_t SPI_init_bus();
esp_err_t MAX30003_init_device(MAX30003_handle_t *handle);
esp_err_t ECG_Config_register(MAX30003_handle_t handle);


esp_err_t MAX30003Config_register_ECG_on(MAX30003_handle_t handle);
esp_err_t MAX30003Config_register_ULP(MAX30003_handle_t handle);


static void INTB_handle(void* arg);
static void BTN_handle(void* arg);
static void esp_check_light_sleep(void* arg);

void pin_init();
void light_sleep_init();

void ADC1_init();
void read_battery(void* pvParameter);

bool check_timeout_ms(uint32_t *t,uint32_t interval,char *log);
void BLE_notify_indicate_handle();

// Funtion Prototype *************************************************************
void app_main(void)
{
    ind_nof_HR.value_len = sizeof(HR_data_test);
    ind_nof_HR.value = HR_data_test;
    ind_nof_BAT.value_len = sizeof(BAT_data_test);
    ind_nof_BAT.value = BAT_data_test;
    gpio_install_isr_service(0);
    BLE_init();
    pin_init();
    SPI_init_bus();
    MAX30003_init_device(&MAX30003_handle);
    MAX30003_write(MAX30003_handle,REG_SW_RST,0);
    ADC1_init();
    xEvtGr1 = xEventGroupCreate();
    light_sleep_init();
    gpio_set_level(PIN_SIGNAL,1);
    vTaskDelay(500/portTICK_RATE_MS);
    gpio_set_level(PIN_SIGNAL,0);
    xTaskCreate(INTB_handle, "INTB_handle", 4096, NULL, 10,NULL);
    xTaskCreate(BTN_handle, "BTN_handle", 4096, NULL, 8,NULL);
    xTaskCreate(esp_check_light_sleep, "esp_check_light_sleep", 4096, NULL, 1, NULL);
    xTaskCreate(read_battery, "read_battery", 2048, NULL, 7, NULL);
    if(BSCO.isBLEEnable) xEventGroupSetBits(xEvtGr1,EVTB_BLE_ENABLE);
    while (1) {
        vTaskDelay(500/portTICK_RATE_MS);
    }
}

static void BTN_handle(void* arg)
{ 
    EventBits_t xEvtBVal;
    for(;;) {
        xEvtBVal = xEventGroupWaitBits(xEvtGr1,EVTB_TASK_BTN,pdFALSE,pdFALSE,portMAX_DELAY);
        if(ASSERT_BITS(xEvtBVal,EVTB_TASK_BTN)){
            while(gpio_get_level(PIN_BTN)==0){
                ESP_LOGI("Waiting",".");
                vTaskDelay(500/portTICK_RATE_MS);
            }
            gpio_set_level(PIN_SIGNAL,1);
            if(ASSERT_BITS(xEvtBVal,EVTB_BLE_CONNECTED)) {
                MAX30003Config_register_ULP(MAX30003_handle);
            }
            else if(!ASSERT_BITS(xEvtBVal,EVTB_BLE_ENABLE)) {
                BLE_enable();
                xEventGroupSetBits(xEvtGr1,EVTB_BLE_ENABLE);
            }
            vTaskDelay(10/portTICK_RATE_MS);
            gpio_set_level(PIN_SIGNAL,0);
            xEventGroupClearBits(xEvtGr1,EVTB_TASK_BTN);
        }
    }
}

static void INTB_handle(void* arg)
{
#define LOINT_DEBOUNCE 20
    EventBits_t xEvtBVal;
    uint8_t count = 0;
    bool enECG = false;
    for(;;) {
        
        xEvtBVal = xEventGroupWaitBits(xEvtGr1, EVTB_TASK_INTB, 
                                                pdFALSE,
                                                pdFALSE,
                                                portMAX_DELAY);
        if(ASSERT_BITS(xEvtBVal,EVTB_TASK_INTB)){
            unsigned int MAX_Val=0;
            unsigned int RR = 0;
            int16_t ECG_sample[32] = {0};
            uint8_t ECG_Idx = 0;
            do
            {
                MAX30003_read(MAX30003_handle,REG_STATUS,&MAX_Val);
                if(ASSERT_BITS(MAX_Val,STATUS_LONINT)
                && !enECG
                ){
                    count ++;
                    t_MAX = 0;
                    if(count >= LOINT_DEBOUNCE){
                        count = 0;
                        enECG = true;
                        gpio_set_level(PIN_SIGNAL,1);
                    }
                    if(enECG){
                        xEventGroupSetBits(xEvtGr1,EVTB_MAX_LONINT);
                        xEvtBVal = xEventGroupGetBits(xEvtGr1);
                        MAX30003Config_register_ECG_on(MAX30003_handle);
                    }
                }
                else if(
                    !ASSERT_BITS(MAX_Val,STATUS_LONINT)
                // &&  ASSERT_BITS(MAX_Val,STATUS_DCLOFF_INT)
                ){
                    enECG = false;  
                    t_MAX = 0;
                    gpio_set_level(PIN_SIGNAL,1);
                    xEventGroupClearBits(xEvtGr1,EVTB_MAX_LONINT);
                    if(ASSERT_BITS(MAX_Val,STATUS_EOVF)){
                        MAX30003_write(MAX30003_handle,REG_FIFO_RST,0);
                    }
                    else if(ASSERT_BITS(MAX_Val,STATUS_EINT)){
                        MAX30003_read_FIFO_normal(MAX30003_handle,ECG_sample,&ECG_Idx);
                        for(uint16_t i=0; i < ECG_Idx;i++){
                            if(ECG_sample[i] < 10000 && ECG_sample[i] > -7000) 
                            printf("%6d\n",ECG_sample[i]);
                        }
                    }
                }
                else{
                    // ESP_LOGI("INTB","check timeout");
                    MAX30003_write(MAX30003_handle,REG_SYNCH_RST,0);
                    vTaskDelay(1/portTICK_RATE_MS);
                    if(check_timeout_ms(&t_MAX,MAX_DCLOFF_TIMEOUT,"DCLOFF TIMEOUT")){
                        MAX30003_write(MAX30003_handle,REG_SW_RST,0);
                        gpio_set_level(PIN_SIGNAL,1);
                        vTaskDelay(3000/portTICK_RATE_MS);
                        gpio_set_level(PIN_SIGNAL,0);
                    }
                    break;
                }
                if(ASSERT_BITS(MAX_Val,STATUS_RRINT)){
                    if(MAX30003_read_RRHR(MAX30003_handle,&*(ind_nof_HR.value+1),&RR) == ESP_OK){
                        xEventGroupSetBits(xEvtGr1,EVTB_BLE_HR_SEND_NOTIFY);
                    }
                }
            } while (gpio_get_level(PIN_INTB));
            gpio_set_level(PIN_SIGNAL,0);
            xEventGroupClearBits(xEvtGr1,EVTB_TASK_INTB);
        }
    }
}

static void esp_check_light_sleep(void* arg)
{
EventBits_t xEvtBVal;
    for(;;) {
        // không được cho TicktoWait = 0 vì sẽ check liên tục trong hàm này dẫn tới watchdog timer
        xEvtBVal = xEventGroupWaitBits(xEvtGr1,EVTB_LIGHT_SLEEP,pdTRUE,pdFALSE,10/portTICK_RATE_MS);
        if(ASSERT_BITS(xEvtBVal,EVTB_LIGHT_SLEEP)
        ){
            gpio_set_level(PIN_SIGNAL,0);
            uart_wait_tx_idle_polling(0);
            esp_light_sleep_start();
            gpio_set_level(PIN_SIGNAL,1);
            switch (esp_sleep_get_wakeup_cause()) {
                case ESP_SLEEP_WAKEUP_TIMER:{
                    ESP_LOGW("LIGHTWAKE","Timeout");
                    // block this task right here enough time for LED to blink
                    vTaskDelay(10/portTICK_RATE_MS);
                    // then break and continue to execute below
                    break;
                }
                case ESP_SLEEP_WAKEUP_GPIO:{
                    if(!gpio_get_level(PIN_BTN)){
                        xEventGroupSetBits(xEvtGr1,EVTB_TASK_BTN);
                    }
                    // if(gpio_get_level(PIN_INTB)){
                    //     xEventGroupSetBits(xEvtGr1,EVTB_TASK_INTB);
                    // }
                    break;
                }
                default:{
                    break;
                }
            }
        }
        xEvtBVal = xEventGroupGetBits(xEvtGr1);
        if(BSCO.isBLEEnable && !BSCO.isConnected){
            xEventGroupClearBits(xEvtGr1,EVTB_BLE_CONNECTED);
            gpio_set_level(PIN_SIGNAL,1);
            vTaskDelay(100/portTICK_RATE_MS);
            gpio_set_level(PIN_SIGNAL,0);
            vTaskDelay(100/portTICK_RATE_MS);
            if(check_timeout_ms(&t_BLE,BLE_WAIT_CONNECT_TIMEOUT,"BLE_CONN_TIMEOUT")){
                gpio_set_level(PIN_SIGNAL,0);
                xEventGroupClearBits(xEvtGr1,EVTB_BLE_ENABLE);
                BLE_disable();
            }
        } else if(BSCO.isBLEEnable && BSCO.isConnected){
            t_BLE = 0;

            if(!ASSERT_BITS(xEvtBVal,EVTB_BLE_CONNECTED))
            xEventGroupSetBits(xEvtGr1,EVTB_BLE_CONNECTED);

            if(check_timeout_ms(&t_BAT,BAT_SAMPLE_TIMEOUT,"BAT_LEVEL_CHECK"))
            xEventGroupSetBits(xEvtGr1,EVTB_TASK_BATTERY);
            
            BLE_notify_indicate_handle();

            if(gpio_get_level(PIN_INTB)
            && !ASSERT_BITS(xEvtBVal,EVTB_TASK_INTB)
            && !ASSERT_BITS(xEvtBVal,EVTB_LIGHT_SLEEP)
            ){
                // ESP_LOGI("INTB",":1");
                vTaskDelay(10/portTICK_RATE_MS);
                xEventGroupSetBits(xEvtGr1,EVTB_TASK_INTB);
            }
            // khi kết nối bluetooth, không thể sleep để wakeup bằng nút nhấn nên phải dùng cách này
            if(!gpio_get_level(PIN_BTN)
            && !ASSERT_BITS(xEvtBVal,EVTB_TASK_BTN)
            && !ASSERT_BITS(xEvtBVal,EVTB_LIGHT_SLEEP)
            ){
                xEventGroupSetBits(xEvtGr1,EVTB_TASK_BTN);
                
            }
        }
        if(    !ASSERT_BITS(xEvtBVal,EVTB_TASK_BATTERY) 
            && !ASSERT_BITS(xEvtBVal,EVTB_TASK_BTN) 
            && !ASSERT_BITS(xEvtBVal,EVTB_TASK_INTB)
            && !ASSERT_BITS(xEvtBVal,EVTB_BLE_ENABLE)
                ){
                // ESP_LOGI("SLEEP","0x%x",xEvtBVal);
                xEventGroupSetBits(xEvtGr1,EVTB_LIGHT_SLEEP);
            }
    }
}

void read_battery(void* pvParameter)
{
#define NUM_OF_SAMPLES 25
#define BATT_LEVEL_OFFSET 3
    EventBits_t xEvtBVal;
    for(;;){
        int adc_raw=0;
        unsigned int voltage = 0;
        uint8_t bat_percent=0;
        xEvtBVal = xEventGroupWaitBits(xEvtGr1,EVTB_TASK_BATTERY,pdTRUE,pdFALSE,portMAX_DELAY);
        gpio_set_level(PIN_SIGNAL,1);
        if(ASSERT_BITS(xEvtBVal,EVTB_TASK_BATTERY)){
            for(uint8_t i=0;i<NUM_OF_SAMPLES;i++){
                adc_raw += adc1_get_raw(ADC1_CHANNEL_4);
            }
            voltage = esp_adc_cal_raw_to_voltage(adc_raw/NUM_OF_SAMPLES,&adc1_chars);
            bat_percent = (((float)voltage*2 - BAT_MIN_VOLT)/(BAT_MAX_VOLT-BAT_MIN_VOLT))*100;
            *(ind_nof_BAT.value) = bat_percent;
            xEventGroupSetBits(xEvtGr1,EVTB_BLE_BAT_SEND_NOTIFY);
            // ESP_LOGI("BatPercent", "%u%%", bat_percent);
        }
        gpio_set_level(PIN_SIGNAL,0);
    }
}

void BLE_notify_indicate_handle()
{
    EventBits_t xEvtBVal;
    xEvtBVal = xEventGroupWaitBits(xEvtGr1, EVTB_BLE_BAT_SEND_NOTIFY 
                                            | EVTB_BLE_HR_SEND_NOTIFY
                                            ,pdTRUE,pdFALSE,0);
    if( ind_nof_HR.mode == BLE_NOTIFY_EN
        && ASSERT_BITS(xEvtBVal,EVTB_BLE_HR_SEND_NOTIFY)
    ){
        esp_ble_gatts_send_indicate(ind_nof_HR.gatt_if,
                                        ind_nof_HR.conn_id,
                                        ind_nof_HR.attr_handle,
                                        ind_nof_HR.value_len,
                                        ind_nof_HR.value,
                                        ind_nof_HR.need_confirm);
    }

    if (ind_nof_BAT.mode == BLE_NOTIFY_EN 
        && ASSERT_BITS(xEvtBVal,EVTB_BLE_BAT_SEND_NOTIFY)
    ){
        esp_ble_gatts_send_indicate(ind_nof_BAT.gatt_if,
                                    ind_nof_BAT.conn_id,
                                    ind_nof_BAT.attr_handle,
                                    ind_nof_BAT.value_len,
                                    ind_nof_BAT.value,
                                    ind_nof_BAT.need_confirm);
    }
}

bool check_timeout_ms(uint32_t *t,uint32_t interval,char* log)
{
#define TIME_SCALE 1000
    if(!*t) *t = (uint32_t)(esp_timer_get_time()/TIME_SCALE);
    if(esp_timer_get_time()/TIME_SCALE - *t >= interval){
        // ESP_LOGW("TIMEOUT","%s after %u ms",log,(uint16_t)(esp_timer_get_time()/TIME_SCALE - *t));
        *t=0;
        return true;
    }
    return false;
}







void light_sleep_init()
{
    gpio_wakeup_enable(PIN_BTN,GPIO_INTR_LOW_LEVEL);
    // gpio_wakeup_enable(PIN_INTB,GPIO_INTR_HIGH_LEVEL);
    esp_sleep_enable_timer_wakeup(SLEEP_TIME);
    esp_sleep_enable_gpio_wakeup();
}

void pin_init()
{
    gpio_config_t io_cfg = {};
    io_cfg.pin_bit_mask = BIT(PIN_SIGNAL);
    io_cfg.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io_cfg);
    
    io_cfg.pin_bit_mask = BIT(PIN_BTN);
    io_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    io_cfg.mode = GPIO_MODE_INPUT;
    gpio_config(&io_cfg);

    io_cfg.pin_bit_mask = BIT(PIN_INTB);
    io_cfg.mode = GPIO_MODE_INPUT;
    gpio_config(&io_cfg);
}

void ADC1_init()
{
    esp_err_t ret;
    ret = esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP);
    if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "Calibration scheme not supported, skip software calibration");
    } else if (ret == ESP_ERR_INVALID_VERSION) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else if (ret == ESP_OK) {
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);
    } else {
        ESP_LOGE(TAG, "Invalid arg");
    }
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11));
}

esp_err_t MAX30003_init_device(MAX30003_handle_t *handle)
{
    esp_err_t ret;
    ESP_LOGI(TAG, "Initializing device...");
    MAX30003_config_pin_t MAX30003_config_pin = {
        .cs_io = PIN_NUM_CS,
        .host = MAX30003_HOST,
        .miso_io = PIN_NUM_MISO,
    };
    ret = MAX30003_init(&MAX30003_config_pin,handle);
    if(ret == ESP_OK) ESP_LOGI(TAG,"Init done");
    ret = MAX30003_get_info(*handle);
    
    return ret;
}

esp_err_t SPI_init_bus()
{
    esp_err_t ret;
    ESP_LOGI(TAG, "Initializing bus SPI%d...", MAX30003_HOST+1);
    spi_bus_config_t buscfg={
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    //Initialize the SPI bus
    ret = spi_bus_initialize(MAX30003_HOST, &buscfg, SPI_DMA_CH_AUTO);
    return ret;
}

esp_err_t MAX30003Config_register_ECG_on(MAX30003_handle_t handle)
{
    
    EN_INT_t enint = {
        .REG.REG_INTB = REG_ENINTB,
        .E_INT = EN_EINT,
        .E_OVF = EN_EINT,
        .E_FS = NOT_USE,
        .E_DCOFF = EN_DCLOFFINT,
        .E_LON = NOT_USE,
        .E_RR = EN_RRINT,
        .E_SAMP = NOT_USE,
        .E_PLL = NOT_USE,
        .E_INT_TYPE = EN_INTB_OD_PU,
    };

    MNGR_INT_t mngrint = {
        .REG = REG_MNGR_INT,
        .EFIT = 0b01111, // Assert EINT w/ 4 unread samples
        .CLR_RRINT = MNGR_INT_CLR_RRINT_RTOR,
    };

    MNGR_DYN_t dyn = {
        .REG = REG_MNGR_DYN,
        .FAST = MNGR_DYN_FAST_NORMAL,
    };

    gen= {
        .REG = 0x01,
        .ULP_LON = 0x02,
        .FMSTR = 0x03
        };
    GEN_t gen = {   
        .REG = REG_GEN,
        .ULP_LON = NOT_USE,
        .FMSTR = GEN_FMSTR_32768_512HZ,
        .ECG = GEN_EN_ECG,
        .DCLOFF = GEN_DCLOFF_EN,
        .IPOL = GEN_IPOL_ECGP_PU_ECGN_PD,
        .IMAG = GEN_IMAG_10nA,
        .VTH = GEN_DCLOFF_VTH_300mV,
        .EN_RBIAS = GEN_EN_RBIAS,
        .RBIASV = GEN_RBIASV_100MOHM,
        .RBIASP = GEN_RBIASP_EN,
        .RBIASN = GEN_RBIASN_EN,
    };

    EMUX_t emux = {
        .REG = REG_EMUX,
        .POL = NOT_USE,
    };

    ECG_t ecg = {
        .REG = REG_ECG,
        .RATE = ECG_RATE_128, // 128sps, FMSTR = 00
        // .RATE = ECG_RATE_0, // 256sps, FMSTR = 00
        .GAIN = ECG_GAIN_160V,
        .DHPF = ECG_DHPF_05HZ,
        .DLPF = ECG_DLPF_40HZ,
    };

    RTOR_t rtor = {
        .REG.RTOR1 = REG_RTOR1,
        .WNDW = RTOR1_WNDW_12,
        .GAIN = RTOR1_GAIN_AUTO,
        .EN = RTOR1_EN_RTOR,
        .PAVG = RTOR1_PAVG_16,
        .PTSF = 4 << RTOR1_PTSF_POS,
    };

    MAX30003_config_register_t cfgreg = {
        .EN_INT = &enint,
        .MNGR_INT = &mngrint,
        .DYN = &dyn,
        .GEN = &gen,
        .EMUX = &emux,
        .ECG = &ecg,
        .RTOR = &rtor,
    };
    MAX30003_conf_reg(handle,&cfgreg);
    return ESP_OK;
}

esp_err_t MAX30003Config_register_ULP(MAX30003_handle_t handle)
{
    EN_INT_t enint = {
        .REG.REG_INTB = REG_ENINTB,
        .E_INT = NOT_USE,
        .E_OVF = NOT_USE,
        .E_FS =NOT_USE,
        .E_DCOFF = NOT_USE,
        .E_LON = EN_LONINT,
        .E_RR =NOT_USE,
        .E_SAMP = NOT_USE,
        .E_PLL = NOT_USE,
        .E_INT_TYPE = EN_INTB_OD_PU,
    };

    GEN_t gen = {   
        .REG = REG_GEN,
        .ULP_LON = GEN_EN_ULP_LON,
        .FMSTR = GEN_FMSTR_32768_512HZ,
        .ECG = NOT_USE,
        .DCLOFF = NOT_USE,
        .IPOL = GEN_IPOL_ECGP_PU_ECGN_PD,
        .IMAG = GEN_IMAG_5nA,
        .VTH = GEN_DCLOFF_VTH_300mV,
        .EN_RBIAS = NOT_USE,
        .RBIASV = GEN_RBIASV_100MOHM,
        .RBIASP = GEN_RBIASP_EN,
        .RBIASN = GEN_RBIASN_EN,
    };

    CAL_t cal = {
        .REG = REG_CAL,
        .VCAL = CAL_EN_VCAL,
        .VMODE = CAL_VMODE_BIPOLAR,
        .VMAG = CAL_VMAG_025mV,
        .FCAL = NOT_USE,
        .FIFTY = CAL_FIFTY,
        .THIGH = NOT_USE,
    };
    
    EMUX_t emux ={
        .REG = REG_EMUX,
        .CALP = EMUX_CALP_SEL_VCALN,
        .CALN = EMUX_CALN_SEL_VCALP,
        .OPENN = EMUX_OPENN,
        .OPENP = EMUX_OPENP,
    };
    
    MAX30003_config_register_t cfgreg = {
        .EN_INT = &enint,
        .GEN = &gen,
        .CAL = &cal,
        .EMUX = &emux,
    };
    MAX30003_conf_reg(handle,&cfgreg);
    return ESP_OK;
}









esp_err_t MAX30003Config_register_ULP_malloc()
{
    ESP_LOGI("Before malloc","%d",esp_get_free_heap_size()); 
    MAX30003_config_register_t_2 *cfgreg2 = (MAX30003_config_register_t_2*)malloc(sizeof(MAX30003_config_register_t_2));
    if(!cfgreg2) return ESP_ERR_NO_MEM;
    cfgreg2->EN_INT.REG.REG_INTB = REG_ENINTB;
    cfgreg2->EN_INT.E_LON = EN_LONINT;
    cfgreg2->EN_INT.E_INT_TYPE = EN_INTB_CMOS;

    cfgreg2->GEN.REG = REG_GEN;
    cfgreg2->GEN.ULP_LON = GEN_EN_ULP_LON;
    cfgreg2->GEN.FMSTR = GEN_FMSTR_32768_512HZ;
    cfgreg2->GEN.IPOL = GEN_IPOL_ECGP_PU_ECGN_PD;
    
    cfgreg2->CAL.REG = REG_CAL;
    cfgreg2->CAL.VCAL = NOT_USE;
    cfgreg2->CAL.VMODE = CAL_VMODE_BIPOLAR;
    cfgreg2->CAL.VMAG = CAL_VMAG_050mV;
    cfgreg2->CAL.FCAL = NOT_USE;
    cfgreg2->CAL.FIFTY = CAL_FIFTY;
    cfgreg2->CAL.THIGH = NOT_USE;
    
    cfgreg2->EMUX.REG = REG_EMUX;
    cfgreg2->EMUX.OPENN = NOT_USE;
    cfgreg2->EMUX.OPENP = NOT_USE;
    cfgreg2->EMUX.CALP = EMUX_CALP_SEL_VCALN;
    cfgreg2->EMUX.CALN = EMUX_CALN_SEL_VCALP;
    
    
    ESP_LOGI("After malloc","%d",esp_get_free_heap_size()); 
    free(cfgreg2);
    ESP_LOGI("After free","%d",esp_get_free_heap_size()); 
    return ESP_OK;
}


esp_err_t MAX30003Config_register_ECG_on_malloc()
{
    ESP_LOGI("Before malloc","%d",esp_get_free_heap_size()); 
    MAX30003_config_register_t_2 *cfgreg2 = (MAX30003_config_register_t_2*)malloc(sizeof(MAX30003_config_register_t_2));
    if(!cfgreg2) return ESP_ERR_NO_MEM;
    cfgreg2->EN_INT.REG.REG_INTB = REG_ENINTB;
    cfgreg2->EN_INT.E_INT = NOT_USE;
    cfgreg2->EN_INT.E_OVF = NOT_USE;
    cfgreg2->EN_INT.E_FS =NOT_USE;
    cfgreg2->EN_INT.E_DCOFF = EN_DCLOFFINT;
    cfgreg2->EN_INT.E_LON = EN_LONINT;
    cfgreg2->EN_INT.E_RR =EN_RRINT;
    cfgreg2->EN_INT.E_SAMP = NOT_USE;
    cfgreg2->EN_INT.E_PLL = NOT_USE;
    cfgreg2->EN_INT.E_INT_TYPE = EN_INTB_CMOS;

    cfgreg2->GEN.REG = REG_GEN;
    cfgreg2->GEN.ULP_LON = GEN_EN_ULP_LON;
    cfgreg2->GEN.FMSTR = GEN_FMSTR_32768_512HZ;
    cfgreg2->GEN.ECG = GEN_EN_ECG;
    cfgreg2->GEN.DCLOFF = GEN_DCLOFF_EN;
    cfgreg2->GEN.IPOL = GEN_IPOL_ECGP_PU_ECGN_PD;
    cfgreg2->GEN.IMAG = GEN_IMAG_100nA;
    cfgreg2->GEN.VTH = GEN_DCLOFF_VTH_300mV;
    cfgreg2->GEN.EN_RBIAS = GEN_EN_RBIAS;
    cfgreg2->GEN.RBIASV = GEN_RBIASV_200MOHM;
    cfgreg2->GEN.RBIASP = GEN_RBIASP_EN;
    cfgreg2->GEN.RBIASN = GEN_RBIASN_EN;
    
    cfgreg2->CAL.REG = REG_CAL;
    cfgreg2->CAL.VCAL = NOT_USE;
    cfgreg2->CAL.VMODE = CAL_VMODE_BIPOLAR;
    cfgreg2->CAL.VMAG = CAL_VMAG_050mV;
    cfgreg2->CAL.FCAL = NOT_USE;
    cfgreg2->CAL.FIFTY = CAL_FIFTY;
    cfgreg2->CAL.THIGH = NOT_USE;
    
    cfgreg2->EMUX.REG = REG_EMUX;
    cfgreg2->EMUX.OPENN = NOT_USE;
    cfgreg2->EMUX.OPENP = NOT_USE;
    cfgreg2->EMUX.CALP = EMUX_CALP_SEL_VCALN;
    cfgreg2->EMUX.CALN = EMUX_CALN_SEL_VCALP;
    
    cfgreg2->ECG.REG = REG_ECG;
    cfgreg2->ECG.RATE = ECG_RATE_1; // 128sps; FMSTR = 00
    cfgreg2->ECG.GAIN = ECG_GAIN_160V;
    cfgreg2->ECG.DHPF = ECG_DHPF_05HZ;
    cfgreg2->ECG.DLPF = ECG_DLPF_40HZ;
    
    cfgreg2->RTOR.REG.RTOR1 = REG_RTOR1;
    cfgreg2->RTOR.WNDW = RTOR1_WNDW_12;
    cfgreg2->RTOR.GAIN = RTOR1_GAIN_AUTO;
    cfgreg2->RTOR.EN = NOT_USE;
    cfgreg2->RTOR.PAVG = 0;
    cfgreg2->RTOR.PTSF = (BIT10 | BIT9);
    ESP_LOGI("After malloc","%d",esp_get_free_heap_size()); 
    free(cfgreg2);
    ESP_LOGI("After free","%d",esp_get_free_heap_size()); 
    return ESP_OK;
}

