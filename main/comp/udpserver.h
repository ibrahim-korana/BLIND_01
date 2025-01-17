

#ifndef _UDP_SERVER_H
#define _UDP_SERVER_H


#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

typedef void (*udpserver_callback_t)(uint8_t *data, uint8_t datalen, uint8_t *recv, uint8_t *reclen);

class UdpServer {
    public:
      UdpServer() {};
      ~UdpServer(){};
      bool start(uint16_t prt, udpserver_callback_t callb);
      bool send_and_receive(const char* hostip, uint8_t *data, int datalen, uint8_t* recv, uint8_t *reclen);
      
    private:
        int port;
        udpserver_callback_t callback;
        static void server_task(void *param);
      //static void data_transmit(const int sock, void *arg);
      //static void wait_data(void *arg);  
};

#endif
