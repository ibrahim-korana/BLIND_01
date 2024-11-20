
void default_config(void)
{
    ESP_LOGW(TAG,"DEFAULT CONFIG");
     GlobalConfig.open_default = 1;
     GlobalConfig.atama_sirasi = 1;
     GlobalConfig.random_mac = 0;

     strcpy((char *)GlobalConfig.mqtt_server,"mqtt.smartq.com.tr");
     GlobalConfig.mqtt_keepalive = 30;
     GlobalConfig.device_id = 1;
     GlobalConfig.project_number=1;
     GlobalConfig.http_start=1;
     GlobalConfig.tcpserver_start=0;
     GlobalConfig.daliserver_start=0;
     GlobalConfig.comminication=0;
     GlobalConfig.time_sync=0;
     GlobalConfig.short_addr = 0;
     GlobalConfig.group = 0;
     GlobalConfig.power = 0;
     GlobalConfig.type1 = 7;
     GlobalConfig.type2 = 7;
     GlobalConfig.type3 = 7;
     GlobalConfig.outtype = 1;

     strcpy((char *)GlobalConfig.dali_server,"192.168.7.1");

     disk.file_control(GLOBAL_FILE);
     disk.write_file(GLOBAL_FILE,&GlobalConfig,sizeof(GlobalConfig),0);
}

void network_default_config(void)
{
    ESP_LOGW(TAG,"NETWORK DEFAULT CONFIG");
     NetworkConfig.home_default = 1;
     NetworkConfig.wifi_type = HOME_WIFI_AP;
     
        NetworkConfig.wan_type = WAN_WIFI; //WAN_ETHERNET; //WAN_WIFI;
        
     NetworkConfig.ipstat = DYNAMIC_IP;
     //strcpy((char*)NetworkConfig.wifi_ssid, "Lords Palace");
     strcpy((char*)NetworkConfig.wifi_pass, "");
     strcpy((char*)NetworkConfig.ip,"192.168.7.1");
     strcpy((char*)NetworkConfig.netmask,"255.255.255.0");
     strcpy((char*)NetworkConfig.gateway,"192.168.7.1");
     strcpy((char*)NetworkConfig.dns,"4.4.4.4");
     strcpy((char*)NetworkConfig.backup_dns,"8.8.8.8");
     NetworkConfig.channel = 1;
     NetworkConfig.WIFI_MAXIMUM_RETRY=5;
/*
    NetworkConfig.wifi_type = HOME_WIFI_STA;
    strcpy((char *)NetworkConfig.wifi_ssid,(char *)"Akdogan_2.4G");
    strcpy((char *)NetworkConfig.wifi_pass,(char *)"651434_2.4");

    NetworkConfig.wifi_type = HOME_WIFI_STA;
    strcpy((char *)NetworkConfig.wifi_ssid,(char *)"Baguette Modem");
    strcpy((char *)NetworkConfig.wifi_pass,(char *)"Baguette2024");
*/

/*
    NetworkConfig.wifi_type = HOME_WIFI_STA;
    strcpy((char *)NetworkConfig.wifi_ssid,(char *)"SMQ");
    strcpy((char *)NetworkConfig.wifi_pass,(char *)"12345678");
*/    
    NetworkConfig.wifi_type = HOME_WIFI_STA;
    strcpy((char *)NetworkConfig.wifi_ssid,(char *)"IMS_YAZILIM");
    strcpy((char *)NetworkConfig.wifi_pass,(char *)"mer6514a4c");
    
    disk.file_control(NETWORK_FILE);
    disk.write_file(NETWORK_FILE,&NetworkConfig,sizeof(NetworkConfig),0);
}


void config(void)
{
    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<OUTPUT_RL1) | (1ULL<<OUTPUT_RL2) |  (1ULL<<LED0) | (1ULL<<LED1);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE; 
    gpio_config(&io_conf);
    gpio_set_level(OUTPUT_RL1,0);
    gpio_set_level(OUTPUT_RL2,0);
    bool kk=false;
    gpio_set_level(LED0,kk);
    gpio_set_level(LED1,kk);

    kk=!kk;
    for (int i=0;i<4;i++)
    {
       gpio_set_level(LED0,kk); 
       gpio_set_level(LED1,kk);
       kk=!kk;
       vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    gpio_install_isr_service(0);
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    //NVS Initialize    
    //ESP_ERROR_CHECK(nvs_flash_erase());
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    ESP_LOGI(TAG,"Disc Init");
    //disk.format();
    ESP_ERROR_CHECK(!disk.init());

    disk.read_file(GLOBAL_FILE,&GlobalConfig,sizeof(GlobalConfig), 0);
	if (GlobalConfig.open_default==0 ) {
		//Global ayarlar diskte kayıtlı değil. Kaydet.
		 default_config();
		 disk.read_file(GLOBAL_FILE,&GlobalConfig,sizeof(GlobalConfig),0);
		 FATAL_MSG(GlobalConfig.open_default,"Global Initilalize File ERROR !...");
	}

    disk.read_file(NETWORK_FILE,&NetworkConfig,sizeof(NetworkConfig), 0);
	if (NetworkConfig.home_default==0 ) {
		//Network ayarları diskte kayıtlı değil. Kaydet.
		 network_default_config();
		 disk.read_file(NETWORK_FILE,&NetworkConfig,sizeof(NetworkConfig),0);
		 FATAL_MSG(NetworkConfig.home_default, "Network Initilalize File ERROR !...");
	}


    disk.list("/config/","*.*");

/*
    Button_Config(BUTTON1,"3",&btn3);
    Button_Config(BUTTON2,"2",&btn2);
    Button_Config(BUTTON3,"1",&btn1);
*/
    blind_config_t cfg1 = {};
    if (GlobalConfig.outtype==1) {
        cfg1.up_gpio_num = OUTPUT_RL1;
        cfg1.down_gpio_num = OUTPUT_RL2;
    } else {
        cfg1.up_gpio_num = OUTPUT_RL2;
        cfg1.down_gpio_num = OUTPUT_RL1;
    }
    cfg1.sure = 10;
    cfg1.Led = LED0;

    dimmer0.init(cfg1, &disk);

}