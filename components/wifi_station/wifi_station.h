/*
 * wifi_station.h
 *
 *  Created on: 
 *      Author: ESP-IDF wifi station example
 */

#ifndef COMPONENTS_WIFI_STATION_WIFI_STATION_H_
#define COMPONENTS_WIFI_STATION_WIFI_STATION_H_

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
//#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "../../credentials/credentials.h"

/* The examples use WiFi configuration that you can set via project configuration menu

   But here WIFI_SSID and WIFI_PASS are defined in credentials.h
*/
#define EXAMPLE_ESP_WIFI_SSID      WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      WIFI_PASS 
#define EXAMPLE_ESP_MAXIMUM_RETRY  5


/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

void wifi_init_sta();

#endif /* COMPONENTS_WIFI_STATION_WIFI_STATION_H_ */


