/*
 * tcp_client_helper.h
 *
 *  Created on: 9 gru 2019
 *      Author: Piokli
 */

#ifndef COMPONENTS_TCP_CLIENT_HELPER_TCP_CLIENT_HELPER_H_
#define COMPONENTS_TCP_CLIENT_HELPER_TCP_CLIENT_HELPER_H_


#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
//#include "nvs_flash.h"
#include "esp_netif.h"
//#include "protocol_examples_common.h"
#include "../wifi_station/wifi_station.h"


#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#define HOST_IP_ADDR "192.168.1.27"
#define PORT 65531

void tcp_client_task(void *pvParameters);


#endif /* COMPONENTS_TCP_CLIENT_HELPER_TCP_CLIENT_HELPER_H_ */
