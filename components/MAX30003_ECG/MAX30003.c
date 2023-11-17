/**
 * @brief MADE BY TRUONG MINH KHOA
 * Spirit boi
 * 
 */
#include "MAX30003.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include <unistd.h>
#include "esp_log.h"
#include <sys/param.h>
#include "sdkconfig.h"

static const char TAG[] = "MAX30003";
/// Context (config and data) of MAX30003
struct MAX30003_context_t{
    MAX30003_config_pin_t cfg;        ///< Configuration by the caller.
    spi_device_handle_t spi;    ///< SPI device handle
    SemaphoreHandle_t INTB2Bsem; ///< Semaphore for INTB and INT2B ISR
};


typedef struct INTB2B_t INTB2B_t;
typedef struct MAX30003_context_t MAX30003_context_t;

// Workaround: The driver depends on some data in the flash and cannot be placed to DRAM easily for
// now. Using the version in LL instead.
#define gpio_set_level  gpio_set_level_patch
#include "hal/gpio_ll.h"
static inline esp_err_t gpio_set_level_patch(gpio_num_t gpio_num, unsigned int level)
{
    gpio_ll_set_level(&GPIO, gpio_num, level);
    return ESP_OK;
}

static void cs_high(spi_transaction_t* t)
{
    ESP_EARLY_LOGV(TAG, "cs high %d.", ((MAX30003_context_t*)t->user)->cfg.cs_io);
    gpio_set_level(((MAX30003_context_t*)t->user)->cfg.cs_io, 1);
}

static void cs_low(spi_transaction_t* t)
{
    gpio_set_level(((MAX30003_context_t*)t->user)->cfg.cs_io, 0);
    ESP_EARLY_LOGV(TAG, "cs low %d.", ((MAX30003_context_t*)t->user)->cfg.cs_io);
}

esp_err_t MAX30003_init(const MAX30003_config_pin_t *cfg, MAX30003_context_t** out_ctx)
{
    esp_err_t err = ESP_OK;
    // Nếu sử dụng SPI interrupt thì không thể sử dụng trên SPI1
    MAX30003_context_t* ctx = (MAX30003_context_t*)malloc(sizeof(MAX30003_context_t));
    if(!ctx) return ESP_ERR_NO_MEM;
    *ctx = (MAX30003_context_t){
        .cfg = *cfg, // đưa tham số config *cfg vào member .cfg của biến ctx vừa tạo
    };

    spi_device_interface_config_t devcfg={
        .clock_speed_hz = 1*1000*1000, // speed 1Mhz
        .mode = 0,
        .command_bits = 8,
        .spics_io_num = -1,
        .pre_cb = cs_low, // trước khi bắt đầu truyền dữ liệu, CS ở mức thấp
        .post_cb = cs_high, // sau khi truyền dữ liệu, CS đưa lên mức cao
        .queue_size = 1,
        .input_delay_ns = 500,
        .flags = SPI_DEVICE_POSITIVE_CS,
    };
    //Attach the MAX30003 to the SPI bus
    err = spi_bus_add_device(ctx->cfg.host,&devcfg,&ctx->spi);
    if  (err != ESP_OK) {
        goto cleanup;
    }
    /** cấu hình chân CS là output
     *  trở kéo lên
     */
    gpio_set_level(ctx->cfg.cs_io,0);
    gpio_config_t cs_cfg = {
        .pin_bit_mask = BIT(ctx->cfg.cs_io),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&cs_cfg);

    
    /** sau khi cấu hình xong, gửi ngược context *ctx về *out_ctx
     *  ra bên ngoài hàm để sử dụng cho mục đích khác
     */
    *out_ctx = ctx;
    return ESP_OK;
cleanup:
    /** gỡ slave khỏi bus, xóa handle dùng để xử lý slave đó 
     *  giải phóng vùng nhớ ctx
     */
    if (ctx->spi) {
        spi_bus_remove_device(ctx->spi);
        ctx->spi = NULL;
    }
    free(ctx);
    return err;
}

esp_err_t MAX30003_read(MAX30003_context_t *ctx,uint8_t reg, unsigned int *out_data)
{
    spi_transaction_t trans = {
        .cmd = (reg << 1) | 0x01,
        .user = ctx,
        .flags = SPI_TRANS_USE_RXDATA,
        .rxlength = 24,
        .length = 24,
    };
    spi_transaction_t *pTrans = &trans;
    spi_device_queue_trans(ctx->spi,&trans,portMAX_DELAY);
    
    esp_err_t err = spi_device_get_trans_result(ctx->spi,&pTrans,portMAX_DELAY);
    if(err!= ESP_OK) {
        return err;
    }
    *out_data = trans.rx_data[0] << 16 | trans.rx_data[1] << 8 | trans.rx_data[2];
    return ESP_OK;
}

esp_err_t MAX30003_write(MAX30003_context_t *ctx,uint8_t reg, unsigned int in_data)
{
    esp_err_t err;
    uint8_t DataTemp[4]= {0};
    DataTemp[3] = (in_data >> 16) & 0xff;
    DataTemp[2] = (in_data >> 8) & 0xff;
    DataTemp[1] = in_data & 0xff;
    spi_transaction_t trans = {
        .cmd = (reg << 1) | 0x00,
        .user = ctx,
        .flags = SPI_TRANS_USE_TXDATA,
        .tx_data = {DataTemp[3],DataTemp[2],DataTemp[1]},
        .length = 24,
    };
    spi_transaction_t *pTrans = &trans;
    spi_device_queue_trans(ctx->spi,&trans,portMAX_DELAY);
    err = spi_device_get_trans_result(ctx->spi,&pTrans,portMAX_DELAY);
    if(err!= ESP_OK) {
        return err;
    }
    return ESP_OK;
}

esp_err_t MAX30003_get_info(MAX30003_context_t *ctx)
{
    esp_err_t err = ESP_OK;
    unsigned int Info=0;
    err = MAX30003_read(ctx,REG_INFO,&Info);
    ESP_LOGI(TAG,"INFO: 0x%x",Info);
    return err;
}

void MAX30003_check_ETAG(unsigned int ECG_Data)
{
    unsigned int ETAG;
    ETAG = (ECG_Data >> 3) & ETAG_MASK;
    switch(ETAG)
    {
        case ETAG_EMPTY:
        ESP_LOGE("ETAG","Empty data",NULL);
        break;
        case ETAG_OVERFLOW:
        ESP_LOGE("ETAG","Overflow",NULL);
        break;
        case ETAG_VALID:
        ESP_LOGI("ETAG","Valid",NULL);
        break;
        case ETAG_VALID_EOF:
        ESP_LOGI("ETAG","Valid EOF",NULL);
        break;
        case ETAG_FAST:
        ESP_LOGW("ETAG","Fast",NULL);
        break;
        case ETAG_FAST_EOF:
        ESP_LOGW("ETAG","Fast EOF",NULL);
        break;
        default:
        {
            ESP_LOGE("ETAG","Invalid ETAG",NULL);
        }
        break;
    }
}

esp_err_t MAX30003_read_FIFO_normal(MAX30003_context_t *ctx, int16_t *ECG_Sample, uint8_t *Idx)
{
    esp_err_t ret = ESP_OK;
    unsigned int ECG_Data,ETAG[32];
    uint8_t ECG_Idx = 0;
    spi_device_acquire_bus(ctx->spi,portMAX_DELAY);
    do{
        MAX30003_read(ctx,REG_ECG_FIFO_NORMAL,&ECG_Data);
        ETAG[ECG_Idx] = (ECG_Data >> 3) & ETAG_MASK;
        ECG_Sample[ECG_Idx] = ECG_Data >> 8;
        ECG_Idx++;
    }while(ETAG[ECG_Idx -1] == ETAG_VALID || ETAG[ECG_Idx -1] == ETAG_FAST);
    spi_device_release_bus(ctx->spi);
    *Idx = ECG_Idx;
    if(ETAG[ECG_Idx-1] == ETAG_OVERFLOW){
        MAX30003_write(ctx,REG_FIFO_RST,0);
        ESP_LOGE("FIFO","Overflow");
        return ret;
    }
    return ret;
}

esp_err_t MAX30003_read_RRHR(MAX30003_context_t *ctx, uint8_t *hr, unsigned int *RR)
{
    esp_err_t ret = ESP_OK;
    unsigned int RTOR;
    ret = MAX30003_read(ctx,REG_RTOR_INTERVAL,&RTOR);
    RTOR = RTOR >> 10;
    *hr =  (uint8_t)(60 /((double)RTOR*0.0078125));
    *RR = (unsigned int)RTOR*(7.8125);
    // printf("HR:%u, RR:%u \n",hr,RR);
    // printf("HR:%u \n",*hr);
    return ret;
}

esp_err_t MAX30003_set_get_register(MAX30003_context_t *ctx,unsigned int reg,unsigned int value,char *NAME_REG)
{
    esp_err_t ret = ESP_OK;
    unsigned int Temp_val = value; 
    MAX30003_write(ctx,reg,Temp_val);
    MAX30003_read(ctx,reg,&Temp_val);
    if(Temp_val != value){
        ESP_LOGE(TAG,"REG %s mismatch, W:0x%x, R:0x%x",NAME_REG,value,Temp_val);
        ret = ESP_ERR_INVALID_RESPONSE;
    }
    // else ESP_LOGI(TAG,"Write %s:0x%x OK",NAME_REG,Temp_val);
    return ret;
}

esp_err_t MAX30003_conf_reg(MAX30003_context_t *ctx, MAX30003_config_register_t *cfgreg)
{
    unsigned int reg_val;
    char *REG_NAME;
    MAX30003_write(ctx,REG_SW_RST,0);
    //Config EN_INT,EN_INT2 register
    if(cfgreg->EN_INT != NULL){
        REG_NAME = "EN_INT";
        if( cfgreg->EN_INT->REG.REG_INTB == REG_ENINTB ||
            cfgreg->EN_INT->REG.REG_INT2B == REG_ENINT2B) 
            {
                EN_INT_t *reg = cfgreg->EN_INT;
                reg_val =   reg->E_INT      | 
                            reg->E_OVF      |
                            reg->E_FS       |
                            reg->E_DCOFF    | 
                            reg->E_LON      | 
                            reg->E_RR       |
                            reg->E_SAMP     |
                            reg->E_PLL      | 
                            reg->E_INT_TYPE ;
                if(reg->REG.REG_INTB == REG_ENINTB) 
                MAX30003_set_get_register(ctx,reg->REG.REG_INTB,reg_val,REG_NAME);
                else MAX30003_set_get_register(ctx,reg->REG.REG_INT2B,reg_val,REG_NAME);
                reg_val = 0;
            }
            else goto REG_ADDR_ERR;
    }
    
    //Config MNGR_INT register
    if(cfgreg->MNGR_INT != NULL){
        REG_NAME = "MNGR_INT";
        if(cfgreg->MNGR_INT->REG != REG_MNGR_INT) goto REG_ADDR_ERR;
        MNGR_INT_t *reg = cfgreg->MNGR_INT;
        reg_val =   (reg->EFIT << MNGR_INT_EFIT_POS)  | 
                    reg->CLR_FAST      |
                    reg->CLR_RRINT     | 
                    reg->CLR_SAMP      | 
                    reg->SAMP_IT; 
        MAX30003_set_get_register(ctx,reg->REG,reg_val,REG_NAME);
        reg_val = 0;
    }

    //Config MNGR_DYN register
    if(cfgreg->DYN != NULL){
        REG_NAME = "MNGR_DYN";
        if(cfgreg->DYN->REG != REG_MNGR_DYN) goto REG_ADDR_ERR;
        MNGR_DYN_t *reg = cfgreg->DYN;
        reg_val =   (reg->FAST_TH << MNGR_DYN_FAST_TH_POS)  | 
                    reg->FAST;
        MAX30003_set_get_register(ctx,reg->REG,reg_val,REG_NAME);
        reg_val = 0;
    }

    //Config MNGR_GEN register
    if(cfgreg->GEN != NULL){
        REG_NAME = "GEN";
        if(cfgreg->GEN->REG != REG_GEN) goto REG_ADDR_ERR;
        GEN_t *reg = cfgreg->GEN;
        reg_val =   reg->ULP_LON    |
                    reg->FMSTR      |
                    reg->ECG        |
                    reg->DCLOFF     | 
                    reg->IPOL       |
                    reg->IMAG       |
                    reg->VTH        |
                    reg->EN_RBIAS   |
                    reg->RBIASV     |
                    reg->RBIASP     |
                    reg->RBIASN     ;

        MAX30003_set_get_register(ctx,reg->REG,reg_val,REG_NAME);
        reg_val = 0;
    }

    //Config CAL register
    if(cfgreg->CAL != NULL){
        REG_NAME = "CALIB";
        if(cfgreg->CAL->REG != REG_CAL) goto REG_ADDR_ERR;
        CAL_t *reg = cfgreg->CAL;
        reg_val =   reg->VCAL   | 
                    reg->VMODE  | 
                    reg->VMAG   | 
                    reg->FCAL   | 
                    reg->FIFTY  | 
                    reg->THIGH;
        MAX30003_set_get_register(ctx,reg->REG,reg_val,REG_NAME);
        reg_val = 0;
    }  
    
    //Config EMUX register
    if(cfgreg->EMUX != NULL){
        REG_NAME = "EMUX";
        if(cfgreg->EMUX->REG != REG_EMUX) goto REG_ADDR_ERR;
        EMUX_t *reg = cfgreg->EMUX;
        reg_val =   reg->POL   | 
                    reg->OPENP | 
                    reg->OPENN | 
                    reg->CALP  | 
                    reg->CALN; 
        
        MAX30003_set_get_register(ctx,reg->REG,reg_val,REG_NAME);
        reg_val = 0;
    }
    
    //Config ECG register
    if(cfgreg->ECG != NULL){
        REG_NAME = "ECG";
        if(cfgreg->ECG->REG != REG_ECG) goto REG_ADDR_ERR;
        ECG_t *reg = cfgreg->ECG;
        reg_val =   reg->RATE | 
                    reg->GAIN | 
                    reg->DHPF | 
                    reg->DLPF ; 
        MAX30003_set_get_register(ctx,reg->REG,reg_val,REG_NAME);
        reg_val = 0;
    }
    
    //Config RTOR register
    if(cfgreg->RTOR != NULL){
        REG_NAME = "RTOR";
        RTOR_t *reg = cfgreg->RTOR;
        if(reg->REG.RTOR1 == REG_RTOR1){
            reg_val =   reg->EN   | 
                        reg->WNDW |
                        reg->PAVG | 
                        reg->PTSF | 
                        reg->GAIN ;
            MAX30003_set_get_register(ctx,reg->REG.RTOR1,reg_val,REG_NAME);
        }
        else if(reg->REG.RTOR2 == REG_RTOR2){
            reg_val =   reg->HOFF | 
                        reg->RAVG | 
                        reg->RHSF ; 
            MAX30003_set_get_register(ctx,reg->REG.RTOR2,reg_val,REG_NAME);
        }
        else goto REG_ADDR_ERR;
        reg_val = 0;
    }

    MAX30003_write(ctx,REG_SYNCH_RST,0);
    return ESP_OK;
REG_ADDR_ERR:
    ESP_LOGE("REG","%s incorrect register address",REG_NAME);
    return ESP_ERR_INVALID_ARG;
}

