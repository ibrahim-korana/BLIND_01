#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "iot_button.h"
#include "led_dim.h"
#include "storage.h"
#include "dali_global.h"
#include "dali.h"
#include "IPTool.h"

#include "comp/network.h"
#include "comp/udpserver.h"

#define RELAY_COUNT 1

ESP_EVENT_DEFINE_BASE(SYSTEM_EVENTS);

#define FATAL_MSG(a, str)                          \
    if (!(a))                                                     \
    {                                                             \
        ESP_LOGE(TAG, "%s(%d): %s", __FUNCTION__, __LINE__, str); \
        abort();                                         \
    }

ESP_EVENT_DECLARE_BASE(DIMMER_EVENTS);
ESP_EVENT_DECLARE_BASE(DALI_EVENTS);

#define OUTPUT_RL1 GPIO_NUM_13
#define OUTPUT_RL2 GPIO_NUM_15

#define DALI_TX GPIO_NUM_22
#define DALI_RX GPIO_NUM_23

#define LED0 GPIO_NUM_2
#define LED1 GPIO_NUM_0

IPAddr Addr = IPAddr();
const char *TAG ="MAIN";

LedDimmer dimmer0;

Storage disk;
Dali dali;
Network wifi = Network();
config_t GlobalConfig;
home_network_config_t NetworkConfig = {};
UdpServer udpserver = UdpServer();

bool busy = true;
bool initialize_mod = false;
bool initialize_start = false;
bool have_random=false;
bool random_start=false;
bool dali_hat_status=true;
bool dali_hat_yk=false;

uint8_t AddrOK = 0;
//uint8_t initializeID[RELAY_COUNT] ={9};
bool Net_Connect = false;
volatile bool Heard=true;

#define GLOBAL_FILE "/config/config.bin"
#define NETWORK_FILE "/config/network.bin"

void dali_callback(package_t *data, backword_t *backword);
void defreset(void);

extern "C" {
#include "bootloader_random.h"
}

enum {
   UP_DIM=0,
   DOWN_DIM,
   NULL_DIM,  
   ON_DIM,
   OFF_DIM,
   BUSY_DIM,
};

uint8_t bt1[3]={OFF_DIM,OFF_DIM,OFF_DIM}, channel = 0;
bool dim_busy=false, yon[3]={false,false,false}, task_running=false;
TaskHandle_t dimHandle = NULL;
SemaphoreHandle_t sem = xSemaphoreCreateBinary();

typedef struct {
    LedDimmer *dimmer;
    uint8_t yon;
} taskarg_t;


void Dimm_Task(void *arg)
{
    taskarg_t *par = (taskarg_t *) arg;
    bool ok=true;
    task_running = true;
    uint8_t basla=0;
    basla = par->dimmer->get_actual_level();
    while(ok){
        if(par->yon==0) {
           if(--basla==0) ok=false; 
           par->dimmer->without_fade_change_nowrite(basla);
           //islem yap
        } else {
            if(++basla==255) ok=false;
            if (basla<255) par->dimmer->without_fade_change_nowrite(basla);
        }
        //printf("%d\n",basla);
        if (xSemaphoreTake(sem, 50 / portTICK_PERIOD_MS)==pdTRUE) ok=false;       
    }
    dim_busy=false;
    task_running=false;
    vTaskDelete(NULL);
}


void dimmer_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
    uint8_t *kk = (uint8_t *)event_data;
    uint8_t zz = 99;
    if (kk!=NULL) zz=*kk;
    if (id==DIMMER_BUSY_END) {ESP_LOGW(TAG,"%d Fade STOP",zz); busy=false;gpio_set_level(LED0,0);}
    if (id==DIMMER_BUSY_START) {ESP_LOGW(TAG,"%d Fade START",zz);busy=true;gpio_set_level(LED0,1);}
}

void dali_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
    if (id==DALI_HAT_ERROR) {
        if (!Net_Connect)
            {
                ESP_LOGE(TAG,"DALI Hat KOPTU");
                dimmer0.direct_power(dimmer0.get_failure_level());
                dali_hat_status=false;
                dali_hat_yk = true;
            }
    }
    if (id==DALI_HAT_NORMAL) {
        if (!Net_Connect)
            {
                ESP_LOGW(TAG,"DALI Hat NORMAL");
                dali_hat_status=true;
                dali_hat_yk = false;
            }
        }
}

#include "comp/http.c"
#include "tool/config.cpp"
#include "tool/events.cpp"
#include "tool/udpcallback.cpp"
#include "tool/smqmdns.cpp"

void webwrite(home_network_config_t net, config_t glob)
{
    disk.write_file(NETWORK_FILE,&net,sizeof(net),0);
    disk.write_file(GLOBAL_FILE,&glob,sizeof(glob),0);
}

void defreset(void )
{
    network_default_config();
    default_config();
}

static void heard_task(void *args)
{   uint8_t cnt = 0;   
    while(Heard) {
        if (++cnt>100)
        {
            gpio_set_level(LED0,1);
            vTaskDelay(pdMS_TO_TICKS(10));
            gpio_set_level(LED0,0);
            cnt = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(100));         
        }
    vTaskDelete(NULL);    
}


gptimer_handle_t addr_timer = NULL;
static bool addr_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx);
QueueHandle_t addr_queue = xQueueCreate(10, sizeof(uint8_t));
static void error_print_task(void *args);

extern "C" void app_main()
{
    esp_log_level_set("wifi", ESP_LOG_NONE);
    esp_log_level_set("wifi_init", ESP_LOG_NONE);
    esp_log_level_set("phy_init", ESP_LOG_NONE);
    esp_log_level_set("gpio", ESP_LOG_NONE);

    bootloader_random_enable();

    ESP_LOGE(TAG, "SMARTQ 1 CHANNEL Dali DT7 Blind Controller");
    ESP_LOGI(TAG, "INITIALIZING...");

    config();

    ESP_ERROR_CHECK(esp_event_handler_instance_register(DIMMER_EVENTS, ESP_EVENT_ANY_ID, dimmer_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(DALI_EVENTS, ESP_EVENT_ANY_ID, dali_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, ip_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_handler, NULL, NULL));

   //uint8_t act=0; 
   dimmer0.set_short_address(16);
  // dimmer1.set_short_address(18);

   if (dimmer0.get_power_on_level()==0xFF) dimmer0.direct_power(dimmer0.get_actual_level()); else dimmer0.direct_power(dimmer0.get_power_on_level()); 

      ESP_LOGW(TAG, "Dimmer  ");
      ESP_LOGI(TAG, "         Short ADDR = %d", dimmer0.get_short_address()); 
      ESP_LOGI(TAG, "     Power on Level = %d", dimmer0.get_power_on_level());
      ESP_LOGI(TAG, "          Max Level = %d", dimmer0.get_max_level());
      ESP_LOGI(TAG, "          Min Level = %d", dimmer0.get_min_level());
      ESP_LOGI(TAG, "      Current Level = %d", dimmer0.get_actual_level());
      ESP_LOGI(TAG, "              Lamp is %s", (dimmer0.is_on())?"ON":"OFF");
      ESP_LOGI(TAG, "              TYPE is %s", (dimmer0.type==6)?"DT6":"DT7");
      ESP_LOGI(TAG, "              Wifi is %s", (NetworkConfig.wan_type==WAN_WIFI)?"ON":"OFF");
  

   strcpy((char *)GlobalConfig.mdnshost,generate_hostname()); 
   initialise_mdns();
   

   Net_Connect = false;  
   if (NetworkConfig.wan_type==WAN_WIFI)
			{
				//Wan haberleşmesi Wifi. Etherneti kapat. Wifi STA olarak start et.
				wifi.init(NetworkConfig);				
				if (wifi.Station_Start()!=ESP_OK) {
                    ESP_LOGE(TAG,"WIFI Station Baslatilamadi. Wifi AP olarak açılıyor");                 
                } 
                Net_Connect=true;
			}  

    if (Net_Connect)
    {
        if (NetworkConfig.wan_type==WAN_WIFI)
		{
			ESP_LOGI(TAG, "WEB START");
            esp_wifi_set_ps(WIFI_PS_NONE);
			ESP_ERROR_CHECK(start_rest_server("/config",NetworkConfig, GlobalConfig, &webwrite));
            udpserver.start(5000, udp_callback); 
            dali_hat_status=false;
		}
    }        

    dali.initialize(DALI_TX,DALI_RX,ADC_CHANNEL_7,dali_callback, LED1);

    if (NetworkConfig.wan_type==WAN_WIFI && NetworkConfig.WIFI_MAXIMUM_RETRY<20)
     {
        NetworkConfig.WIFI_MAXIMUM_RETRY = 20;
        disk.write_file(NETWORK_FILE,&NetworkConfig,sizeof(NetworkConfig),0);
     }

        gptimer_config_t gpc1 = {};
        gpc1.clk_src = GPTIMER_CLK_SRC_DEFAULT;
        gpc1.direction = GPTIMER_COUNT_UP;
        gpc1.resolution_hz = 1 * 1000 * 1000; // 1MHz, 1 tick = 1us
        ESP_ERROR_CHECK(gptimer_new_timer(&gpc1, &addr_timer));
        gptimer_alarm_config_t alarm_cfg = {};
        alarm_cfg.reload_count = 0;
        alarm_cfg.alarm_count = 9000000; 
        alarm_cfg.flags.auto_reload_on_alarm = false;
        ESP_ERROR_CHECK(gptimer_set_alarm_action(addr_timer, &alarm_cfg));

        gptimer_event_callbacks_t cbs = {};
        cbs.on_alarm = addr_alarm_cb; // register user callback

        ESP_ERROR_CHECK(gptimer_register_event_callbacks(addr_timer, &cbs, NULL));
        ESP_ERROR_CHECK(gptimer_set_alarm_action(addr_timer, &alarm_cfg));
        ESP_ERROR_CHECK(gptimer_enable(addr_timer));
        xTaskCreate(error_print_task, "task_09", 2048, NULL, 10, NULL);

     xTaskCreatePinnedToCore(heard_task, "task_00", 2048, NULL, 20, NULL,1);


    ESP_LOGI(TAG, "Ready");

    while(true)
    {           
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    
}

static void error_print_task(void *args)
{
    uint8_t kk = 0;
    while(1)
    {
        if (xQueueReceive(addr_queue, &kk, portMAX_DELAY))
        {
            initialize_mod=false;
            have_random=false; 
            AddrOK = 0;
            dali.Hat_Kontrol(true);
            dali.re_init();
            dali.int_enable();
            
            gptimer_stop(addr_timer);
            ESP_LOGI(TAG,"Adresleme modundan ANORMAL yolla çıkıldı");
        }
    }
}

bool addr_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_awoken = pdFALSE;   
    uint8_t kk= 0;
    xQueueSendFromISR(addr_queue, &kk, &high_task_awoken);   
    return high_task_awoken == pdTRUE;
}


#define DEBUG0
//#define DEBUG1

void dali_callback(package_t *data, backword_t *backword)
{
    
    backword->backword = BACK_NONE;

    if (data->data.type==0) return;
   
    #ifdef DEBUG0
        ESP_LOGI("DALI", "Gelen << 0=%02x 1=%02X 2=%02X %d", data->data.data0, data->data.data1 , data->data.data2,  data->data.type);
    #endif
        
    address_t adres = {};
    unpackage_address(data->data.data0,&adres);

   #ifdef DEBUG1 
   if (!adres.error)
   {      
        printf("arc power %d\n", adres.arc_power);
        printf("short adr %d\n", adres.short_adr);
        printf("group adr %d\n", adres.group_adr);
        printf("broadcast %d\n", adres.broadcast_adr);
        printf("special %d\n", adres.special);
        printf("data %02X\n", adres.data);
        printf("error %d\n", adres.error);
        printf("COMMAND %02X %02X\n", data->data.data0,data->data.data1);       
   }
   #endif
  
    if (!adres.error)
    {
        if (!initialize_mod)
        {
            if (adres.arc_power) dimmer0.direct_arc_power(data->data.data1, adres);
            if (!adres.arc_power && !adres.special)
            {
                    switch(data->data.data1)
                    {
                        case 0x00: dimmer0.command_off(adres); break;
                        case 0x01: dimmer0.command_up(adres); break;
                        case 0x02: dimmer0.command_down(adres); break;
                        case 0x03: dimmer0.command_step_up(adres); break;
                        case 0x04: dimmer0.command_step_down(adres); break;
                        case 0x05: dimmer0.command_recall_max_level(adres); break;
                        case 0x06: dimmer0.command_recall_min_level(adres); break;
                        case 0x07: dimmer0.command_step_down_and_off(adres); break;
                        case 0x08: dimmer0.command_on_and_step_up(adres); break;
                        case 0x0A: dimmer0.command_goto_last_active_level(adres); break;
                        default:
                           addr_find_t adres_ara = {};
                           adres_ara.data = data->data.data1;
                           if (adres_ara.is_scene()) dimmer0.command_goto_scene(data->data.data1&0x0F,adres);
                           if (adres_ara.is_config()) dimmer0.command_config(data->data.data1,adres,true); 
                           if (adres_ara.is_query()) {
                               int i = dimmer0.command_query(data->data.data1,adres);
                               if (i>-1) {
                                    backword->backword = BACK_DATA;
                                    backword->data = i;
                                    if (data->data.data1==0x99 && NetworkConfig.wan_type==WAN_WIFI) backword->data=backword->data+10;
                                    //printf("Query %02X Ret=%02X\n",adres.data, backword->data=backword->data);
                               }                               
                           }
                    }
               
            }
        }

        if (adres.special) 
            {
                //if (adres.data!=0x04)
                // ESP_LOGI(TAG,"       Special 0x%02X 0x%02X",adres.data,data->data.data1);
                 //vTaskDelay(500 / portTICK_PERIOD_MS);

                    if (initialize_mod)
                    {
                        gptimer_stop(addr_timer);
                        gptimer_set_raw_count(addr_timer,0);
                        gptimer_start(addr_timer);
                    }

                if (adres.data==0x01)  dimmer0.special_command(adres.data,data->data.data1,true); //DTR0 yaz
                if (adres.data==0x00 && initialize_mod) {
                    initialize_mod=false;
                    have_random=false; 
                    dali.Hat_Kontrol(true);
                    gptimer_set_raw_count(addr_timer,0);
                    gptimer_stop(addr_timer);
                    AddrOK = 0;
                    ESP_LOGI(TAG,"Adresleme modundan çıkıldı");
                }
            
                if (adres.data==0x02 && !initialize_mod && dimmer0.special_command(adres.data,data->data.data1,true)==0xFF)
                {
                   if (data->data.data1==0x00)
                    {
                        initialize_mod=true;
                        for (int i=0;i<RELAY_COUNT;i++) dimmer0.clear_short_address();                      
                        ESP_LOGW(TAG,"Cihaz adresleme modunda");  
                        Heard=false;
                        dali.Hat_Kontrol(false);
                        gptimer_set_raw_count(addr_timer,0);
                        gptimer_start(addr_timer);
                        AddrOK = 1;
                    }  
                    if (data->data.data1==0xFF)
                    {
                        if (dimmer0.get_short_address()==0xFF) 
                        {
                            initialize_mod=true;
                            ESP_LOGW(TAG,"Cihaz adresleme modunda");  
                            Heard=false;
                            dali.Hat_Kontrol(false);
                            gptimer_set_raw_count(addr_timer,0);
                            gptimer_start(addr_timer);
                            AddrOK = 1;
                                                        
                        }
                    } 
                }
               
                if (initialize_mod)
                {
                    if (!have_random && adres.data==0x03 && dimmer0.special_command(adres.data,data->data.data1,true)==0xFF)
                    {
                        long_addr_t s = {};
                        dimmer0.get_random(&s);
                        ESP_LOGI(TAG,"Random ADDR0 %08X",(unsigned int)s.random.long_num);
                        have_random =  true;
                        ESP_LOGI(TAG,"Adresleniyor....");
                        AddrOK = 2;
                    }                   
                    
                    if (have_random && AddrOK==2)
                    {
                        switch (adres.data)
                        {
                            case 0x08 ... 0x0B: {
                                                dimmer0.special_command(adres.data,data->data.data1); 
                                                } 
                                                break;
                            case 0x04:
                            case 0x0C: {
                                        uint8_t  ret = dimmer0.special_command(adres.data,data->data.data1); 
                                        if (ret==0xFF) backword->backword = BACK_YES;    
                                       }    
                                       break;                                                         
                            case 0x05: {
                                        uint8_t ret = dimmer0.special_command(adres.data,data->data.data1);
                                        if (ret==0xFF) {
                                            //listeden sil
                                            ESP_LOGW(TAG,"Adresleme Tamamlandı");
                                            backword->backword = BACK_YES; 
                                            AddrOK = 1;
                                                       }
                                       } 
                                       break;           
                        }
                        
                    }                    
                }
            }   
            
    }
  
   // vTaskDelay(10 / portTICK_PERIOD_MS);
   // if (backword->backword==BACK_YES) printf("YES Bulundu\n");
   // if (backword->backword==BACK_NO) printf("NO Bulundu\n");
       // gpio_set_level(LED,0);
}