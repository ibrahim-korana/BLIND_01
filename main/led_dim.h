#ifndef _LED_DIM_H_
#define _LED_DIM_H_

#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "storage.h"
#include <math.h>
#include "dali_global.h"

#define DEVICE_TYPE 0x07
#define NEXT_DEVICE_TYPE 0x77

enum {
	MOTOR_UP=0,
	MOTOR_DOWN,
	MOTOR_PROCESS,
    MOTOR_STOP,
    MOTOR_UNKNOWN,
};

union long_number_t {
     struct {
        uint8_t data0;      
        uint8_t data1;
        uint8_t data2;
        uint8_t data3;
     } data;  
     uint32_t long_num;
};

typedef struct {
    uint8_t data;
    bool is_scene(void)
      {
        return (data>=0x10 && data<=0x1F);
      }
    bool is_config(void)
      {
        return (data>0x1F && data<0x90) ||  //DT6 command
                (data>=0xE0 && data<0xF0);     //DT7 command
      }  
    bool is_query(void)
      {
        return (data>=0x90 && data<=0xC6) ||  //DT6 Command
                (data>=0xF0 && data<=0xFE) ; //DT7 command
      }  
} addr_find_t;


typedef struct {
   long_number_t search;
   long_number_t random; 
   bool is_equal(void) {if (random.long_num==search.long_num) return true; else return false;}
   bool is_less_than_equal(void) {if (random.long_num<=search.long_num) return true; else return false;}
} long_addr_t;

typedef struct {
    int32_t up_gpio_num;
    int32_t down_gpio_num;
    uint16_t sure;
    gpio_num_t Led;
} blind_config_t;

ESP_EVENT_DEFINE_BASE(DIMMER_EVENTS);

enum {
    DIMMER_BUSY_START,
    DIMMER_BUSY_END,
    DIMMER_MOTOR_STOP,
    DIMMER_MOTOR_START,
};

typedef struct {
        uint8_t actual_level;
        uint8_t last_actual_level;
        uint8_t power_on_level;
        uint8_t system_failure_level;
        uint8_t min_level;
        uint8_t max_level;
        uint8_t fade_time ;
        uint8_t fade_rate;
        uint8_t extended_fade_time_base ; //1-16
        uint8_t extended_fade_time_multiplier ; //0-4
        uint8_t scene[16] ; //Senaryo degerlerini tutar. 
        uint8_t gear_groups[2]; //Bit bazlı çalışır   
        uint8_t short_address;
        uint8_t search_address[3];
        uint8_t random_address[3];
        uint8_t DTR0;
        uint8_t DTR1;
        uint8_t DTR2;
        uint8_t operating_mode;
        uint8_t Light_sourge_type;
        //---------
        uint8_t up_on;
        uint8_t up_off;
        uint8_t down_on;
        uint8_t down_off;
        uint8_t enable_dt7;

        uint8_t version;
        uint8_t physical_min_level ;
        uint8_t device_type;
        bool enable_write_memory ;
        uint8_t bank0[10];
        uint8_t bank1[10];
        uint8_t lamp_on;

        uint8_t def;
} variable_t;

class LedDimmer {
    public:
        LedDimmer() {};
        ~LedDimmer() {};
        void init(blind_config_t cfg, Storage *dsk);
        void without_fade_change(uint8_t level);
        void without_fade_change_nowrite(uint8_t level);

        bool is_on() {return var.lamp_on;}
        bool direct_power(uint16_t level);

        bool direct_arc_power(uint16_t level, address_t adr);
        bool command_off(address_t adr);
        bool command_up(address_t adr);
        bool command_down(address_t adr);
        bool command_step_up(address_t adr);
        bool command_step_down(address_t adr);
        bool command_recall_max_level(address_t adr);
        bool command_recall_min_level(address_t adr);
        bool command_step_down_and_off(address_t adr);
        bool command_on_and_step_up(address_t adr);
        bool command_goto_last_active_level(address_t adr);
        bool command_goto_scene(uint8_t scn, address_t adr);

        uint8_t command_config(uint8_t comm, address_t adr, bool reply=true);
        int command_query(uint8_t comm, address_t adr);
        uint8_t special_command(uint8_t comm, uint8_t dat, bool reply=false);
        

        uint8_t get_actual_level(void) {return var.actual_level;}
        void set_actual_level(uint8_t lvl) {
            var.actual_level=lvl; 
            disk->write_file(file_name,&var,sizeof(variable_t),0);
            }
        uint8_t get_max_level(void) {return var.max_level;}
        uint8_t get_min_level(void) {return var.min_level;}
        uint8_t get_power_on_level(void) {return var.power_on_level;}
        uint8_t get_failure_level(void) {return var.system_failure_level;}

        bool get_random(long_addr_t *rd) {
           rd->random.data.data0 = var.random_address[2];
           rd->random.data.data1 = var.random_address[1];
           rd->random.data.data2 = var.random_address[0];
           return true;
        }
        bool set_random(long_addr_t *rd) {
           var.random_address[2] = rd->random.data.data0;
           var.random_address[1] = rd->random.data.data1;
           var.random_address[0] = rd->random.data.data2;
           return true;
        }

        bool is_fade_running(void) {return fade_running;}
        bool is_open(void) {return var.lamp_on;}

        bool is_short_address(uint8_t addr) {
            if (var.short_address==addr) return true;
            return false;
            }
        uint8_t get_short_address(void) {return var.short_address;}   
        void set_short_address(uint8_t addr) {var.short_address=addr;disk->write_file(file_name,&var,sizeof(variable_t),0);} 
        void clear_short_address(void) {var.short_address=0xFF;disk->write_file(file_name,&var,sizeof(variable_t),0); } 
        bool is_group_address(uint8_t addr) {
            if (addr>15) return false;
            if (var.gear_groups[0]==0xFF && var.gear_groups[1]==0xFF) return false;
            if (((uint8_t)pow(2,addr)&var.gear_groups[0])>0x00) return true;
            if (addr>8) {
                uint8_t tmp=addr-8;
                if (((uint8_t)pow(2,tmp)&var.gear_groups[1])>0x00) return true;
            }
            return false;
        } 
        bool is_addr(address_t adr) {
            return  ((adr.short_adr && is_short_address(adr.data)) || (adr.group_adr && is_group_address(adr.data))  || adr.broadcast_adr );
        }

        uint8_t kanal;
        uint8_t type = 7;

    protected:
    //Türetilenden erişilir
        blind_config_t config; 
        volatile uint8_t Motor_Status = MOTOR_STOP;  
        void motor(void);
        void motor_stop(void);
        uint32_t calc_time(void);

        bool busy=false;
        SemaphoreHandle_t Motor_Kilit;
        
        
        variable_t var={};
        Storage *disk;
        char *file_name;
        gpio_num_t Led = GPIO_NUM_NC;

        bool control_Gear_Failure = false;
        bool lamp_failure = false;
       // bool lamp_on = false;
        bool limit_error = false;
        bool fade_running = false;
        bool reset_state = false;
        bool power_cycle_seen = false;

        uint32_t get_raw_fade_time(void);
        uint16_t get_raw_fade_rate(void);
        uint16_t get_multiplayer(void);
        //void without_fade_change(uint8_t level);
        void Reset(void);
        void convert_long(long_addr_t *s);
        

        

        esp_timer_handle_t special_timer;
        esp_timer_handle_t config_timer;
        uint8_t first_special=0xFF, active_special=0xFF;
        uint8_t first_config=0xFF, active_config=0xFF;

        //void fade_start(uint16_t duty, uint16_t time);
      

    private:
        
        
       // static bool IRAM_ATTR fade_end_event(const ledc_cb_param_t *param, void *user_arg);
        static void motor_task(void *arg);
        static void special_timer_callback(void *arg);
        static void config_timer_callback(void *arg);
        static void motor_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data);

};



#endif