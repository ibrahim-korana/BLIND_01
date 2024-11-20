

void ip_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
    //ESP_LOGW(TAG, "IP %ld %ld", id , base, id);
    if (id==IP_EVENT_STA_GOT_IP || id==IP_EVENT_ETH_GOT_IP)
              {
                // Net.wifi_update_clients();
                ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
                NetworkConfig.home_ip = event->ip_info.ip.addr;
                NetworkConfig.home_netmask = event->ip_info.netmask.addr;
                NetworkConfig.home_gateway = event->ip_info.gw.addr;
                NetworkConfig.home_broadcast = (uint32_t)(NetworkConfig.home_ip) | (uint32_t)0xFF000000UL;
                //tcpclient.wait = false;
                #ifdef ETHERNET
                    if (id==IP_EVENT_ETH_GOT_IP) w5500.set_connect_bit();
                #endif
                if (id==IP_EVENT_STA_GOT_IP) wifi.set_connection_bit();
                ESP_LOGI(TAG, "IP Received %s",Addr.to_string(NetworkConfig.home_ip));
               // sntp_set_time_sync_notification_cb(time_sync);
               // Set_SystemTime_SNTP();             
              }
}


void wifi_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
    if (id==13) {}
    if (id==WIFI_EVENT_STA_DISCONNECTED)
              {
                 // tcpclient.wait = true;
                  if (wifi.retry < NetworkConfig.WIFI_MAXIMUM_RETRY) {
                	  wifi.Station_connect();;
                      wifi.retry++;
                      ESP_LOGW(TAG, "Tekrar Baglanıyor %d",NetworkConfig.WIFI_MAXIMUM_RETRY-wifi.retry);
                                                      } else {
                      ESP_LOGE(TAG,"Wifi Başlatılamadı.. Wifi AP Modda açılıyor.");
                      wifi.ap_init();
                                                             }
              }

    if (id==WIFI_EVENT_STA_START)
        {
        wifi.retry=0;
        ESP_LOGW(TAG, "Wifi Network Connecting..");
        wifi.Station_connect();
        }

}

