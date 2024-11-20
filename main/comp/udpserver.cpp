

#include "udpserver.h"

static const char *UDP_TAG = "UDP_SERVER";
#define MAX_DATA_LEN 64

void UdpServer::server_task(void *pvParameters)
{
    UdpServer *ths =( UdpServer *)pvParameters;
    uint8_t *rx_buffer;
    uint8_t *back;
    char addr_str[128];
    int addr_family = AF_INET;
    int ip_protocol = 0;
    int port = ths->port;
    struct sockaddr_in dest_addr;

    while (1) 
    {
        dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(port);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(UDP_TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(UDP_TAG, "Socket created");
        int broadcast = 1;
        if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast))<0)
        {
            ESP_LOGE(UDP_TAG,"Soket Broadcast olarak işaretlenemedi");
            break;
        }

        // Set timeout
       /*
        struct timeval timeout;
        timeout.tv_sec = 10;
        timeout.tv_usec = 0;
        setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);
       */
        
        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(UDP_TAG, "Socket unable to bind: errno %d", errno);
            break;
        }
        ESP_LOGI(UDP_TAG, "Socket bound, port %d", port);
        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t socklen = sizeof(source_addr);
        rx_buffer = (uint8_t *)calloc(1,MAX_DATA_LEN);
        back = (uint8_t *)calloc(1,MAX_DATA_LEN);
        while (1) 
        {
            memset(rx_buffer,0,MAX_DATA_LEN);
            int len = recvfrom(sock, rx_buffer, MAX_DATA_LEN - 1, 0, (struct sockaddr *)&source_addr, &socklen);
            if (len < 0) {
                ESP_LOGE(UDP_TAG, "recvfrom failed: errno %d", errno);
                break;
            } else {
                inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
                rx_buffer[len] = 0;
                uint8_t bcklen=0;
                ths->callback(rx_buffer, len, back, &bcklen);
                if (bcklen>0)
                  {
                        int err = sendto(sock, back, bcklen, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                        if (err < 0) {
                            ESP_LOGE(UDP_TAG, "Error occurred during sending: errno %d", errno);
                            break;
                        };// else ESP_LOGI(UDP_TAG,"%s Gönderildi",addr_str);
                  }
            }
        } //while
        free(rx_buffer);
        free(back);
        if (sock != -1) {
            ESP_LOGE(UDP_TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    } //while
    vTaskDelete(NULL);
};


bool UdpServer::send_and_receive(const char* hostip, uint8_t *data, int datalen, uint8_t* recv, uint8_t *reclen)
{
    int addr_family = 0;
    int ip_protocol = 0;

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(hostip);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;

    int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(UDP_TAG, "Unable to create socket: errno %d", errno);
        return false;
    }
    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);
    int err = sendto(sock, data, datalen, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        ESP_LOGE(UDP_TAG, "Error occurred during sending: errno %d", errno);
        shutdown(sock, 0);
        close(sock);
        return false;
    }
    if (recv!=NULL)
    {
        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(sock, recv, *reclen, 0, (struct sockaddr *)&source_addr, &socklen);
        if (len < 0) {
                //ESP_LOGE(UDP_TAG, "recvfrom failed: errno %d", errno);
                *reclen=0;
                shutdown(sock, 0);
                close(sock);
                return false;
            } else {
                recv[len] = 0; // Null-terminate whatever we received and treat like a string
                *reclen=len;
            }   
    }
    
    shutdown(sock, 0);
    close(sock);
    return true;
}

bool UdpServer::start(uint16_t prt, udpserver_callback_t callb)
{
    port = prt;
    callback = callb;
    xTaskCreate(server_task, "udp_server", 4096, (void*)this, 5, NULL);
    return true;
}