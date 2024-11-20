
#include "led_dim.h"
#include "math.h"
#include "bootloader_random.h"
#include <sys/random.h>

static const char *TAG = "PWM";
#define MAX_DUTY 255
#define MIN_DUTY 0
#define FADE_TIME 2000

static const uint16_t cie1931[256] = {
	0, 2, 4, 5, 7, 9, 11, 13, 15, 16, 
	18, 20, 22, 24, 26, 27, 29, 31, 33, 35, 
	34, 36, 37, 39, 41, 43, 45, 47, 49, 52, 
	54, 56, 59, 61, 64, 67, 69, 72, 75, 78, 
	81, 84, 87, 90, 94, 97, 100, 104, 108, 111, 
	115, 119, 123, 127, 131, 136, 140, 144, 149, 154, 
	158, 163, 168, 173, 178, 183, 189, 194, 200, 205, 
	211, 217, 223, 229, 235, 241, 247, 254, 261, 267, 
	274, 281, 288, 295, 302, 310, 317, 325, 333, 341, 
	349, 357, 365, 373, 382, 391, 399, 408, 417, 426, 
	436, 445, 455, 464, 474, 484, 494, 505, 515, 526, 
	536, 547, 558, 569, 580, 592, 603, 615, 627, 639, 
	651, 663, 676, 689, 701, 714, 727, 741, 754, 768, 
	781, 795, 809, 824, 838, 853, 867, 882, 897, 913, 
	928, 943, 959, 975, 991, 1008, 1024, 1041, 1058, 1075, 
	1092, 1109, 1127, 1144, 1162, 1180, 1199, 1217, 1236, 1255, 
	1274, 1293, 1312, 1332, 1352, 1372, 1392, 1412, 1433, 1454, 
	1475, 1496, 1517, 1539, 1561, 1583, 1605, 1628, 1650, 1673, 
	1696, 1719, 1743, 1767, 1791, 1815, 1839, 1864, 1888, 1913, 
	1939, 1964, 1990, 2016, 2042, 2068, 2095, 2121, 2148, 2176, 
	2203, 2231, 2259, 2287, 2315, 2344, 2373, 2402, 2431, 2461, 
	2491, 2521, 2551, 2581, 2612, 2643, 2675, 2706, 2738, 2770, 
	2802, 2835, 2867, 2900, 2934, 2967, 3001, 3035, 3069, 3104, 
	3138, 3174, 3209, 3244, 3280, 3316, 3353, 3389, 3426, 3463, 
	3501, 3539, 3576, 3615, 3653, 3692, 3731, 3770, 3810, 3850, 
	3890, 3930, 3971, 4012, 4053, 4095
};


uint32_t LedDimmer::get_raw_fade_time(void)
        {
            //Fade süresini belirler milisaniye cinsinden 
            switch( var.fade_time ) 
            { 
                case 1: return 700;
                case 2: return 1000;
                case 3: return 1400;
                case 4: return 2000;
                case 5: return 2800;
                case 6: return 4000; //def
                case 7: return 5700;
                case 8: return 8000;
                case 9: return 11300;
                case 10: return 16000;
                case 11: return 22600;
                case 12: return 32000;
                case 13: return 45300;
                case 14: return 64000;
                case 15: return 90500;
            };
            return 0;
        };  

uint16_t LedDimmer::get_raw_fade_rate(void)
        {
            //dönen deger step cinsindendir. Fade hızını belirler step/sn
            switch(var.fade_rate)
            {
                case  1 : return 357;
                case  2 : return 253;
                case  3 : return 178;
                case  4 : return 126;
                case  5 : return 89;
                case  6 : return 63;
                case  7 : return 45; //def
                case  8 : return 32;
                case  9 : return 22;
                case  10 : return 16;
                case  11 : return 11;
                case  12 : return 8;
                case  13 : return 6;
                case  14 : return 4;
                case  15 : return 3;
            }
            return 0;
        };

uint16_t LedDimmer::get_multiplayer(void)
        {
            //ms olarak döndürür
            switch(var.extended_fade_time_multiplier) 
            {
                case 0 : return 0;
                case 1 : return 100;
                case 2 : return 1000;
                case 3 : return 10000;
                case 4 : return 60000;
            }
            return 0;
        } 

uint32_t LedDimmer::calc_time(void)
{
    uint32_t tm = 0;
    if (var.fade_time==0) {
        //fade time degeri microsaniye olarak
        tm = (get_multiplayer() * var.extended_fade_time_base) ;
    } else {
        tm = get_raw_fade_time(); 
    }
    if (tm==0) tm=1000;
    return tm;
}

/*
void LedDimmer::virtual_fade_task(void *arg)
{
    vf_arg_t *ar0 = (vf_arg_t *)arg; 
    vf_arg_t ar = {};
    ar.channel = ar0->channel;
    ar.d_off = ar0->d_off;
    ar.start = ar0->start;
    ar.stop = ar0->stop;
    ar.time = ar0->time;
    ar.u_on = ar0->u_on;
    ar.yon = ar0->yon; 
    ar.gpio = ar0->gpio;
    ar.ths = ar0->ths;

    //printf("str:%d stp:%d yon:%d tm:%d chn:%d on:%d off:%d\n",ar.start,ar.stop,ar.yon,ar.time,ar.channel,ar.u_on,ar.d_off);
    ar.ths->fade_running = true;    

    if (ar.time==0) ar.time=1;
        int vv=0;
        if (ar.yon==0) {
            //yukarı
            uint8_t kk = ar.start;
            while(kk<ar.stop && vv<260) {
                if (kk==ar.u_on) gpio_set_level((gpio_num_t)ar.gpio,1);
                kk++;vv++;
                vTaskDelay(ar.time/portTICK_PERIOD_MS);
            }
        }
        if (ar.yon==1) {
            //asagı
            uint8_t kk = ar.start;
            while(kk>ar.stop && vv<260) {
                if (kk==ar.d_off) gpio_set_level((gpio_num_t)ar.gpio,0);
                kk--;vv++;
                vTaskDelay(ar.time/portTICK_PERIOD_MS);
            }
        }
    ar.ths->busy = false;
    ar.ths->fade_running = false;    
    esp_event_post(DIMMER_EVENTS,DIMMER_BUSY_END,&ar.channel,sizeof(uint8_t),10/portTICK_PERIOD_MS);        
    vTaskDelete(NULL);    
}
*/

typedef struct {
    uint8_t yon;
    uint16_t time;
    uint8_t basla;
    uint8_t bitir;
    LedDimmer* ths;
} vf_arg_t;


void LedDimmer::motor_task(void *arg)
{
   vf_arg_t *par = (vf_arg_t *)arg;
   uint8_t stp = par->basla, count = 0; 
   uint16_t stp_time = par->time / 254;
   bool dn = true;
   if (par->ths->Led!=-1) gpio_set_level(par->ths->Led,1);
   if (stp_time<10) stp_time = 10; 
   esp_event_post(DIMMER_EVENTS,DIMMER_MOTOR_START,NULL,0,10/portTICK_PERIOD_MS);

   printf("basla %d bitir %d time %d %d %d\n",stp,par->bitir, stp_time,par->time,par->yon);

   while(dn) 
   {      
       if (par->yon==MOTOR_UP) gpio_set_level((gpio_num_t)par->ths->config.up_gpio_num,1); 
       if (par->yon==MOTOR_DOWN) gpio_set_level((gpio_num_t)par->ths->config.down_gpio_num,1); 
       count++;
       if(stp++>=par->bitir) dn=false;
       if (xSemaphoreTake(par->ths->Motor_Kilit,stp_time/portTICK_PERIOD_MS)==pdTRUE) dn=false;
       //printf("%d %d\n",stp,count);
   }
    gpio_set_level((gpio_num_t)par->ths->config.up_gpio_num,0); 
    gpio_set_level((gpio_num_t)par->ths->config.down_gpio_num,0);
    printf("STP %d %d\n",par->ths->var.last_actual_level,count-1);
    uint8_t kk = 0;
    if (par->yon==MOTOR_UP) kk = (par->ths->var.last_actual_level) + (count -1);
    if (par->yon==MOTOR_DOWN) kk = (par->ths->var.last_actual_level) - (count - 1);
    esp_event_post(DIMMER_EVENTS,DIMMER_MOTOR_STOP,&kk,sizeof(uint8_t),10/portTICK_PERIOD_MS);
    if (par->ths->Led!=-1) gpio_set_level(par->ths->Led,0);
    vTaskDelete(NULL);
}

void LedDimmer::motor_stop(void)
{
    while (Motor_Status==MOTOR_PROCESS) {
            xSemaphoreGive(Motor_Kilit);
            vTaskDelay(10/portTICK_PERIOD_MS);
        } 
}
void LedDimmer::motor(void)
{
    /*
        tm ms cinsinden toplam süredir. Bu süre 254 e bölünerek 1 adımın süresi bulunur.
    */
    static vf_arg_t arg = {};
    arg.ths = this;
    arg.time = calc_time();

    uint8_t stp = abs(var.actual_level-var.last_actual_level);
    arg.yon = MOTOR_UNKNOWN;
    if (stp>0) {
        if (var.actual_level>var.last_actual_level)  {arg.yon = MOTOR_UP;arg.bitir =var.actual_level; arg.basla = var.last_actual_level;}
        if (var.last_actual_level>var.actual_level)  {arg.yon = MOTOR_DOWN;arg.bitir =var.last_actual_level; arg.basla = var.actual_level;}
        xTaskCreate(motor_task, "motor_task", 2048, (void*)&arg, 10, NULL);
    }
}



void LedDimmer::special_timer_callback(void *arg)
{
    LedDimmer *ths = (LedDimmer *)arg;
    ths->first_special =0xFF;
    ths->active_special=0xFF;
    ESP_LOGE("LED_DIM","Special Command Error [clear]");
}

void LedDimmer::config_timer_callback(void *arg)
{
    LedDimmer *ths = (LedDimmer *)arg;
    ths->first_config =0xFF;
    ths->active_config=0xFF;
    ESP_LOGE("LED_DIM","Config Command Error [clear]");
}

void LedDimmer::motor_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
    uint8_t *kk = (uint8_t *)event_data;
    LedDimmer *ths = (LedDimmer *)handler_args;

    if (id==DIMMER_MOTOR_START) {
        ESP_LOGW(TAG,"Motor CALISIYOR"); 
        ths->Motor_Status = MOTOR_PROCESS;
    }

    if (id==DIMMER_MOTOR_STOP) {
        ESP_LOGW(TAG,"Motor Durdu. Durma ACISI : %d",*kk); 
        ths->var.actual_level = *kk;
        ths->disk->write_file(ths->file_name,&ths->var,sizeof(variable_t),0);
        ths->Motor_Status = MOTOR_STOP;
        }
    
}

void LedDimmer::init(blind_config_t cfg, Storage *dsk)
{
    config.up_gpio_num = cfg.up_gpio_num;
    config.down_gpio_num = cfg.down_gpio_num;
    config.sure = cfg.sure;

    Led = cfg.Led;
    disk = dsk;
    
    
    file_name = (char*)calloc(1,20);
    sprintf(file_name,"/config/cfg%ld.bin",config.up_gpio_num);     
    int zz = disk->file_size(file_name);

    if (zz==0) {
        memset(&var,0,sizeof(var));
        var.actual_level  = 0;
        var.last_actual_level  = 0;
        var.power_on_level  = 128;
        var.system_failure_level = 35;
        var.min_level  = 0;
        var.max_level  = 254;
        var.fade_time = 8;
        var.fade_rate = 1;
        var.extended_fade_time_base = 1; //1-16
        var.extended_fade_time_multiplier = 0; //0-4
        //var.scene[16] ; //Senaryo degerlerini tutar. 
        //var.gear_groups[2]; //Bit bazlı çalışır   
        for (int i=0;i<2;i++) var.gear_groups[i]=0;
        for (int i=0;i<16;i++) var.scene[i]=255;
        var.short_address = 255;
        //uint8_t search_address[3];
        //uint8_t random_address[3];
        var.DTR0=0;
        var.DTR1=0;
        var.DTR2=0;
        var.operating_mode=0;
        var.Light_sourge_type=7;

        var.version = 1;
        var.physical_min_level = 0;
        var.device_type = 7;
        var.enable_write_memory = false;
        var.def = 1;

        var.up_off = 255;
        var.down_on = 255;
        var.up_on   = 60;
        var.down_off= 100;

        disk->file_control(file_name);
        disk->write_file(file_name,&var,sizeof(variable_t),0);
       // printf("default yazıldı %d\n",var.short_address);
    }
    disk->read_file(file_name,&var,sizeof(variable_t),0);
    //printf("default okundu %d\n",var.short_address);
    //var.short_address = 0x01;
    //disk->write_file(file_name,&var,sizeof(variable_t),0);
    //printf("last %d\n",var.last_actual_level);
    fade_running = false;
    busy = false;

    esp_timer_create_args_t arg0 = {};
    arg0.callback = special_timer_callback;
    arg0.arg = (void*) this;
    arg0.name = "sptm";
    ESP_ERROR_CHECK(esp_timer_create(&arg0, &special_timer));

    esp_timer_create_args_t arg1 = {};
    arg1.callback = config_timer_callback;
    arg1.arg = (void*) this;
    arg1.name = "cfgtm";
    ESP_ERROR_CHECK(esp_timer_create(&arg1, &config_timer));

    Motor_Kilit =  xSemaphoreCreateBinary();

    ESP_ERROR_CHECK(esp_event_handler_instance_register(DIMMER_EVENTS, DIMMER_MOTOR_STOP, motor_handler, (void *)this, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(DIMMER_EVENTS, DIMMER_MOTOR_START, motor_handler, (void *)this, NULL));

}


bool LedDimmer::direct_arc_power(uint16_t level,address_t adr)
{
    //Actual_level degerini Min-Max level arasında direkt değiştirmek için kullanılır. 
    //Actual_level değişimi fade ile olur. Fade süresi fade_time da belirlenen süredir.    
    //DESTEK DT6 DT7 
    if (is_addr(adr))
    if (!fade_running)
     { 
        ESP_LOGI(TAG,"ArcPower %02X",level);
        if (level<=var.min_level) level=var.min_level;
        if (level>=var.max_level) level=var.max_level;
        //var.actual_level = level;
        if (level>=var.min_level && level<=var.max_level)
            {
                motor_stop();
                var.lamp_on = true;
                var.last_actual_level = var.actual_level;  
                var.actual_level = level;               
                motor();                
            } else {
              limit_error = true;
              ESP_LOGE(TAG,"Limit error level=%02x Min=%02x Max=%02x",level,var.min_level,var.max_level);
              return false;
            }
            return true;
     }
    //ESP_LOGE(TAG,"Fade running error"); 
    return false;
}

bool LedDimmer::direct_power(uint16_t level)
{ 
    return true;
}

bool LedDimmer::command_off(address_t adr)
{ 
    if (is_addr(adr))
    { 
        ESP_LOGI(TAG,"Off Command");
        var.lamp_on = false;    
        motor_stop();    
        return true;
    } ;
    return false;
}

bool LedDimmer::command_up(address_t adr)
{
    if (is_addr(adr))
    {
        motor_stop();
        var.last_actual_level = var.actual_level;
        if (var.actual_level<=244) var.actual_level=var.actual_level+10; else var.actual_level=254; 
        motor();             
        return true;      
    }
    return false;
}

bool LedDimmer::command_down(address_t adr)
{
    if (is_addr(adr))
    {
        motor_stop();
        var.last_actual_level = var.actual_level;
        if (var.actual_level>=10) var.actual_level=var.actual_level-10; else var.actual_level=0; 
        motor();             
        return true;      
    }
    return false;
}

bool LedDimmer::command_goto_last_active_level(address_t adr)
{
    //DESTEK DT6 DT7
    if (is_addr(adr))
    {
        uint8_t kk = var.last_actual_level;
        motor_stop();
        var.last_actual_level = var.actual_level;
        var.actual_level = kk;
        motor();
    }
    return false;
}

void LedDimmer::without_fade_change(uint8_t level)
{
}

void LedDimmer::without_fade_change_nowrite(uint8_t level)
{ 
}


bool LedDimmer::command_step_up(address_t adr)
{
    if (is_addr(adr))
    {
        return command_up(adr);
    }
    return false;
}

bool LedDimmer::command_step_down(address_t adr)
{
    if (is_addr(adr))
    {
        return command_down(adr);
    }
    return false;
}

bool LedDimmer::command_recall_max_level(address_t adr)
{
    if (is_addr(adr))
    {
        limit_error=false;
        if (!var.lamp_on) var.lamp_on=true;
        motor_stop();
        var.last_actual_level = var.actual_level;
        var.actual_level = 254; 
        motor();       
    }
    return true;
}

bool LedDimmer::command_recall_min_level(address_t adr)
{
    if (is_addr(adr))
    {
        limit_error=false;
        if (!var.lamp_on) var.lamp_on=true;
        motor_stop();
        var.last_actual_level = var.actual_level;
        var.actual_level = 0; 
        motor();
    }
    return true;
}

bool LedDimmer::command_step_down_and_off(address_t adr)
{
    if (is_addr(adr))
    {
        return command_down(adr);
    }
    return false;
}

bool LedDimmer::command_on_and_step_up(address_t adr)
{
    if (is_addr(adr))
    {
        return command_up(adr);
    }
    return false;
}

bool LedDimmer::command_goto_scene(uint8_t scn, address_t adr)
{
    //DESTEK DT6 DT7
    if (is_addr(adr))
    {
        uint8_t lvl = var.scene[scn];    
        ESP_LOGI(TAG,"Command goto scene Level:%d ",lvl);   
        if (lvl<255)
                {
                    motor_stop();
                    var.lamp_on = true;
                    var.last_actual_level = var.actual_level;
                    var.actual_level = lvl;
                    motor();
                };
                return true;
        
    }
    return false;
}


int LedDimmer::command_query(uint8_t comm, address_t adr)
{
    if(is_short_address(adr.data))
    {
        switch(comm)
        {
            case 0x90 : {
                //Query Status
                uint8_t aa = 0x00;
                if (control_Gear_Failure) aa = aa|0x01;
                if (lamp_failure) aa = aa|0x02;
                if (var.lamp_on) aa = aa|0x04;
                if (limit_error) aa = aa|0x08;
                if (fade_running) aa = aa|0x10;
                if (reset_state) aa = aa|0x20;
                if (var.short_address==255) aa = aa|0x40;
                if (power_cycle_seen) aa = aa|0x80;
                return aa;
            }
            break;
            case 0x91: {
                if (control_Gear_Failure) return 0xFF; else return 0x00;
            }
            break;
            case 0x92: {
                if (lamp_failure) return 0xFF; else return 0x00;
            }
            break;
            case 0x93:{
                if (var.lamp_on) return 0xFF; else return 0x00;
            }
            case 0x94:{
                if (limit_error) return 0xFF; else return 0x00;
            }
            case 0x95:{
                if (reset_state) return 0xFF; else return 0x00;
            }
            case 0x96:{
                if (var.short_address==255) return 0xFF; else return 0x00;
            }
            case 0x97: {
                return var.version;
            }
            case 0x98: {
                return var.DTR0;
            }
            case 0x99: {
                return DEVICE_TYPE; //Relay
            }
            case 0x9A: {
                return var.physical_min_level;
            }
            case 0x9B: {
                return power_cycle_seen;
            }
            case 0x9C: {
                return var.DTR1;
            }
            case 0x9D: {
                return var.DTR2;
            }
            case 0x9E: {
                return var.operating_mode;
            }
            case 0x9F: {
                return var.Light_sourge_type;
            }
            case 0xA0 : {
                return var.actual_level;
            }
            case 0xA1 : {
                return var.max_level;
            }
            case 0xA2 : {
                return var.min_level;
            }
            case 0xA3 : {
                return var.power_on_level;
            }
            case 0xA4 : {
                return var.system_failure_level;
            }
            case 0xA5 : {
                return var.fade_time<<4 | var.fade_rate;
            }
            case 0xA6 : {
                //Boş
                return 0;
            }
            case 0xA7 : {
                //Boş
                return NEXT_DEVICE_TYPE; //bu deger rölenin altta ne iş yaptıgını gösterir. 
                   //Devive type ın kendisi dönüyorsa normal röle
                   //0x77 dönüyorsa bu perdedir
            }
            case 0xA8 : {
                return var.extended_fade_time_multiplier<<4 | var.extended_fade_time_base;
            }
            case 0xAA : {
                return control_Gear_Failure;
            }
            case 0xB0 ... 0xBF : {
                uint8_t aa = comm & 0x0F;
                return var.scene[aa];
            }
            case 0xC0 : {
                return var.gear_groups[0];
            }
            case 0xC1 : {
                return var.gear_groups[1];
            }
            case 0xC2 : {
                return var.random_address[0];
            }
            case 0xC3 : {
                return var.random_address[1];
            }
            case 0xC4 : {
                return var.random_address[2];
            }
            case 0xC5 : {
                return var.bank1[var.DTR0];
            }
            case 0XF0 : {
            //DT7 Query Features 
            return 0x0A;
            }
            case 0XF1 : {
            //DT7 Query Switch Status 
            return var.lamp_on;
            }
            case 0XF2 : {
            //DT7 Query Up Switch On Threshold  
            return var.up_on;
            }
            case 0XF3 : {
            //DT7 Query Up Switch Off Threshold  
            return var.up_off;
            }
            case 0XF4 : {
            //DT7 Query Down Switch On Threshold 
            return var.down_on;
            }
            case 0XF6 : {
            //DT7 Query Error Hold
            return 0;
            }
            case 0XF7 : {
            //DT7 Query Gear Type
            return type;
            }
            default: {return 0;} break;
        }
    }
    return -1;
}

void LedDimmer::convert_long(long_addr_t *s)
{
    s->random.data.data0 = var.random_address[2];
    s->random.data.data1 = var.random_address[1];
    s->random.data.data2 = var.random_address[0];
    s->random.data.data3 = 0x00;

    s->search.data.data0 = var.search_address[2];
    s->search.data.data1 = var.search_address[1];
    s->search.data.data2 = var.search_address[0];
    s->search.data.data3 = 0x00;
    //printf("%08X %08X  %02x %02x %02x\n",(unsigned int)s->random.long_num,(unsigned int)s->search.long_num, var.search_address[0],var.search_address[1],var.search_address[2]);
}

uint8_t LedDimmer::special_command(uint8_t comm, uint8_t dat, bool reply)
{
    uint8_t ret = 0x00;
    if (reply)
    {
        if (first_special==0xFF)
        {
                //İlk komut geldi Hafızaya al
                first_special = comm;
                active_special = 0xFF;
                if (esp_timer_is_active(special_timer)) esp_timer_stop(special_timer);
                esp_timer_start_once(special_timer,200000);
        } else {
            if (first_special==comm) {
                //İkinci komut ile ilki eşit işlem yap
                active_special=comm;
                first_special = 0xFF;
                if (esp_timer_is_active(special_timer)) esp_timer_stop(special_timer);
                ESP_LOGI("SPECIAL_COMMAND","Command Special %02x",active_special);
            } else {
                //eşit degil işlem yapma
                first_special =0xFF;
                active_special=0xFF;
                if (esp_timer_is_active(special_timer)) esp_timer_stop(special_timer);
            }
        }
    } else {active_special = comm;first_special=0xFF;}

    switch(active_special) {
        case 0: break;
        case 1: {
                    var.DTR0 = dat; 
                      // ESP_LOGI("SPECIAL_COMMAND","         DTR0 %02x Yazıldı",var.DTR0);
                } break;
        case 2: {
                    return 0xFF;
                }        
        case 3: {
                  getrandom((uint8_t *)var.random_address,4,0); 
                  //printf("Random ADDR : 0x%02X%02X%02X\n",var.random_address[0],var.random_address[1],var.random_address[2]); 
                  return 0xFF;
                };
                break;
        case 4: {
                     long_addr_t s;
                     convert_long(&s); 
                     if (s.is_less_than_equal()) ret = (uint8_t)0xFF;
                }
                break; 
        case 0x5: {
                    long_addr_t s;
                    convert_long(&s); 
                    if (s.is_equal()) ret = (uint8_t)0xFF;
                  } break;               
        case 0x8: {var.search_address[0]=dat; }; break;
        case 0x9: {var.search_address[1]=dat; }; break;
        case 0xA: {var.search_address[2]=dat; }; break;
        case 0xB: {
                    long_addr_t s;
                    convert_long(&s); 
                    ESP_LOGI(TAG,"Set Req %08X=%08X %d",(unsigned int)s.random.long_num, (unsigned int)s.search.long_num, s.is_equal());
                    if (s.is_equal()) {
                         var.short_address = (dat>>1) & 0x3F;
                         ESP_LOGW(TAG,"Short ADDR Atandı. Adres :  %02X",var.short_address);
                         disk->write_file(file_name,&var,sizeof(variable_t),0);
                       }
                  } break;
        case 0x0C : {
                     long_addr_t s;
                     convert_long(&s); 
                     uint8_t kk = (dat>>1) & 0x3F;
                     //if (var.short_address == () && s.is_equal())
                     //printf("R=%08X S=%08X  %02x %02x %02x\n",(unsigned int)s.random.long_num,(unsigned int)s.search.long_num, var.search_address[0],var.search_address[1],var.search_address[2]);
                     //printf("SA  %02X=%02d %d\n", kk,var.short_address,s.is_equal());
                     if (var.short_address == kk && s.is_equal())
                     {
                        ret = (uint8_t)0xFF; 
                        ESP_LOGW(TAG,"%02X Short ADDR Kontrol edildi",var.short_address);
                     }
                        
                    } break;          
        case 0x0D : return var.short_address;
        case 0x10 : var.enable_dt7 = true;
        default : break;
    }
    //printf("DTR 0 = %d\n",DTR0);
    return ret;
}


uint8_t LedDimmer::command_config(uint8_t comm, address_t adr, bool reply)
{   
    var.enable_write_memory = false;

    if (is_short_address(adr.data))
    {
        if (reply)
        {
            if (first_config==0xFF)
            {
                    //İlk komut geldi Hafızaya al
                    first_config = comm;
                    active_config = 0xFF;
                    if (esp_timer_is_active(config_timer)) esp_timer_stop(config_timer);
                    esp_timer_start_once(config_timer,200000);
            } else {
                if (first_config==comm) {
                    //İkinci komut ile ilki eşit işlem yap
                    active_config=comm;
                    first_config = 0xFF;
                    if (esp_timer_is_active(config_timer)) esp_timer_stop(config_timer);
                    ESP_LOGI("CONFIG_COMMAND","Command config %02x",active_config);
                } else {
                    //eşit degil işlem yapma
                    first_config = 0xFF;
                    active_config= 0xFF;
                    if (esp_timer_is_active(config_timer)) esp_timer_stop(config_timer);
                }
            }
        }  else {active_config = comm;first_config=0xFF;}

        bool yaz = false;
        switch(active_config) {
            case 0x20 : {
                //Reset
                var.actual_level = 0;
                var.last_actual_level = 0;
                var.physical_min_level = 0;
                var.device_type = 7;
                control_Gear_Failure = false;
                lamp_failure = false;
                var.lamp_on = false;
                limit_error = false;
                fade_running = false;
                reset_state = true;
                power_cycle_seen = false;
                Reset();
                yaz=true;
            }
            break;
            case 0x21:{var.DTR0 = var.actual_level; yaz=true;} break;
            case 0x22:{yaz=true;}break;
            case 0x23:{yaz=true; var.operating_mode = var.DTR0;}break;
            case 0x24:break;//{dali2}break;
            case 0x25:break;
            case 0x2A:{var.max_level=var.DTR0;yaz=true;}break;
            case 0x2B:{var.min_level=var.DTR0;yaz=true;}break;
            case 0x2C:{var.system_failure_level=var.DTR0;yaz=true;}break;
            case 0x2D:{var.power_on_level=var.DTR0;yaz=true;}break;
            case 0x2E:{var.fade_time=var.DTR0;yaz=true;}break;
            case 0x2F:{var.fade_rate=var.DTR0;yaz=true;}break;
            case 0x30:{var.extended_fade_time_base=var.DTR0&0x0F;
                    var.extended_fade_time_multiplier = var.DTR0>>4;
                    yaz=true;
                    }break;
            case 0x40 ... 0x4F: {
                uint8_t aa = active_config & 0x0F;
                var.scene[aa]=var.DTR0;
                yaz=true;
                break;
            }        
            case 0x50 ... 0x5F : {
                uint8_t aa = active_config & 0x0F;
                var.scene[aa]=255;
                yaz=true;
                break;
            }
            case 0x60 ... 0x6F: {
                uint8_t aa=active_config&0x0F;
                //printf("%02x aa=%02X\n",active_config,aa);
                if ((aa&0x08)!=0x08) {
                    uint8_t bb = pow(2,aa);
                    //printf("bb=%02x\n",bb);
                    var.gear_groups[0] = var.gear_groups[0] | bb;
                    //printf("grp=%02x\n",var.gear_groups[0]);
                    yaz=true;
                } else {
                    uint8_t bb = pow(2,(aa&0x07));
                    var.gear_groups[1] = var.gear_groups[1] | bb;
                    yaz=true;
                }
            } break;
            case 0x70 ... 0x7F: {
                uint8_t aa=active_config&0x0F;
                if ((aa&0x08)!=0x08) {
                    uint8_t bb = 0xFF - pow(2,aa);
                    var.gear_groups[0] = var.gear_groups[0] & bb;
                    yaz=true;
                } else {
                    uint8_t bb = 0xFF - pow(2,(aa&0x07));
                    var.gear_groups[1] = var.gear_groups[1] & bb;
                    yaz=true;
                }
            } break;
            case 0x80: {
                var.short_address = var.DTR0; 
                ESP_LOGW("BLIND","Short ADDR %d",var.short_address);
                yaz=true;
            } break;
            case 0x81: {
                var.enable_write_memory = true;
                yaz=true;            
            }break;

            //----------------
            case 0xE0: {
                //Reference System Power //DT7 config command
                var.enable_dt7 = false;
                var.last_actual_level = 0;
                var.actual_level = 0;
                yaz=true;
            } break;
            case 0xE1: {
                //Store DTR as Up Switch On Threshold //DT7 config command
                if (var.enable_dt7==true) var.up_on = var.DTR0;
                var.enable_dt7 = false;
                yaz = true;
            } break;
            case 0xE2: {
                //Store DTR as Up Switch Off Threshold //DT7 config command
                if (var.enable_dt7==true) var.up_off = var.DTR0;
                var.enable_dt7 = false;
                yaz = true;
            } break;
            case 0xE3: {
                //Store DTR as Down Switch On Threshold //DT7 config command
                if (var.enable_dt7==true) var.down_on = var.DTR0;
                var.enable_dt7 = false;
                yaz = true;
            } break;
            case 0xE4: {
                //Store DTR as Down Switch Off Threshold  //DT7 config command
                if (var.enable_dt7==true) var.down_off = var.DTR0;
                var.enable_dt7 = false;
                yaz = true;
            } break;
            case 0xE5: {
                //Store DTR as Error Hold Time //DT7 config command
                var.enable_dt7 = false;
                yaz=true;
            } break;
            


            //----------------
            default: return 0;
        }
        if (yaz) disk->write_file(file_name,&var,sizeof(variable_t),0); 
    }
    return 0;
}

void LedDimmer::Reset(void)
{
    var.actual_level=0;
    var.last_actual_level=0;
    var.power_on_level=255;
    var.system_failure_level=255;
    var.min_level=var.physical_min_level;
    var.max_level=254;
    var.fade_rate=7;
    var.fade_time=1;
    var.extended_fade_time_base=1;
    var.extended_fade_time_multiplier=0;
    var.up_off = 255;
    var.down_on = 255;
    var.up_on   = 50;
    var.down_off = 200;
    for (int i=0;i<3;i++) var.search_address[i]=255;
    for (int i=0;i<3;i++) var.random_address[i]=255;
    for (int i=0;i<2;i++) var.gear_groups[i]=0;
    for (int i=0;i<16;i++) var.scene[i]=255;
}