
#include "mdns.h"
#include "esp_mac.h"

const char *MDNS_HOSTNAME = "SMQ_DT7";

char *generate_hostname(void)
{
    uint8_t mac[6];
    char   *hostname;
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    if (-1 == asprintf(&hostname, "%s_%02X%02X", MDNS_HOSTNAME, mac[4], mac[5])) {
        abort();
    }
    return hostname;
}

void initialise_mdns(void)
{
    char *hostname = (char*)GlobalConfig.mdnshost;

    //initialize mDNS
    ESP_ERROR_CHECK( mdns_init() );
    ESP_ERROR_CHECK( mdns_hostname_set(hostname) );
    ESP_LOGI(TAG, "mdns hostname set to: [%s]", hostname);
    //set default mDNS instance name
    ESP_ERROR_CHECK( mdns_instance_name_set(hostname) );

    char *ad0;
    asprintf(&ad0,"%d",dimmer0.get_short_address());
    mdns_txt_item_t attr[2] = {
        {"addr0", ad0},      
        {"board", "SMQ.03"}
    };
    
    //initialize service
    ESP_ERROR_CHECK( mdns_service_add(hostname, "_http", "_tcp", 80, attr, 2) );
    ESP_ERROR_CHECK( mdns_service_subtype_add_for_host(hostname, "_http", "_tcp", NULL, "_server") );

    //free(hostname);
}