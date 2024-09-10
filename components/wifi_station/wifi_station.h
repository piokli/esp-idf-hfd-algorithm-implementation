/*
 * wifi_station.h
 *
 *  Created on: 9 gru 2019
 *      Author: Piokli
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

/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID      "PLAY_Swiatlowodowy_D5DD"    //"Wifi-Dom"
#define EXAMPLE_ESP_WIFI_PASS      "NcMmKqw4x9"  //"Klimek47" 
#define EXAMPLE_ESP_MAXIMUM_RETRY  5


/* FreeRTOS event group to signal when we are connected*/
EventGroupHandle_t s_wifi_event_group;

void wifi_init_sta();

#endif /* COMPONENTS_WIFI_STATION_WIFI_STATION_H_ */


