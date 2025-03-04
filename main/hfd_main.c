/* Hfd main source */

#define BIT_0 (1 << 0)
#define BIT_1 (1 << 1)

#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "nvs_flash.h"
#include "esp_http_client.h"
#include "esp_sntp.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "../components/wifi_station/wifi_station.h"
#include "../components/i2c_helper/i2c_helper.h"
#include "../components/lsm6ds33/lsm6ds33.h"
#include "../components/lps25h/lps25h.h"
#include "../credentials/credentials.h"

QueueHandle_t xQueueAccelerometerData;
QueueHandle_t xQueueGyroscopeData;
QueueHandle_t xQueueBarometerData;
EventGroupHandle_t xFallDetectionGroupHandle;

#define TELEGRAM_MSG_BUFF_SIZE 128

// extern EventGroupHandle_t s_wifi_event_group;

static const char *TAG_main = "main";

struct vector_and_time_pack {
	struct vector vector_data;
	int64_t time_us;
};

struct press_and_time_pack {
	uint32_t press_data;
	int64_t time_us;
};


//////////////////////////////////////////////

void read_sensors_data_task(void *pvParameters)
{
	struct vector_and_time_pack acc_and_time;
	struct vector_and_time_pack gyro_and_time;
	struct press_and_time_pack press_and_time;

	struct vector acc_data;
	struct vector gyro_data;
	uint32_t press_data;
	int64_t time_us;

	vTaskDelay(1000 / portTICK_PERIOD_MS);

	while(1)
	{
		//ESP_LOGI(TAG_main, "Reading sensors data %d", i++);

		time_us = esp_timer_get_time();
		lsm6ds33_read_acc_raw(&acc_data);
		lsm6ds33_read_gyro_raw(&gyro_data);
		lps25h_read_press_raw(&press_data);

		acc_and_time.vector_data = acc_data;
		acc_and_time.time_us = time_us;
		gyro_and_time.vector_data = gyro_data;
		gyro_and_time.time_us = time_us;
		press_and_time.press_data = press_data;
		press_and_time.time_us = time_us;

		xQueueSendToBack(xQueueAccelerometerData, &acc_and_time, pdMS_TO_TICKS(20));
		xQueueSendToBack(xQueueGyroscopeData, &gyro_and_time, pdMS_TO_TICKS(20));
		xQueueSendToBack(xQueueBarometerData, &press_and_time, pdMS_TO_TICKS(20));

		vTaskDelay(pdMS_TO_TICKS(20)); // vTaskDelay(pdMS_TO_TICKS(20));  == vTaskDelay(20 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

static void send_telegram_msg(void *pvParameters)
{
	// Set message and chat ID to send to
	char telegram_msg[TELEGRAM_MSG_BUFF_SIZE];
	strcpy(telegram_msg, (char *)pvParameters);
	char post_data[128];
	sprintf(post_data, "chat_id=%s&text=", TELEGRAM_CHAT_ID);
	strcat(post_data, telegram_msg);

	// Set http post API (url and bot token)
	char post_url[128] = "https://api.telegram.org/bot";
	char post_url_end[] = "/sendMessage";
	const char telegram_token[] = TELEGRAM_BOT_TOKEN;
	strcat(post_url, telegram_token);
	strcat(post_url, post_url_end);
	esp_http_client_config_t config = {
		.url = post_url,
		.method = HTTP_METHOD_POST,
	};
	esp_http_client_handle_t client = esp_http_client_init(&config);
	
	// Perform http post
	ESP_ERROR_CHECK(esp_http_client_set_post_field(client, post_data, strlen(post_data)));
	ESP_ERROR_CHECK(esp_http_client_perform(client));
	esp_http_client_cleanup(client);

	// Avoid spamming messages
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	vTaskDelete(NULL);
}

void accelerometer_maths_task(void *pvParameters)
{
	QueueHandle_t getQueue = pvParameters;
	struct vector_and_time_pack acc_and_time;
	struct vector acc_data;
	float acc_mag;
	int64_t time_us = 0;

	int ACC_STATE = 0;
	const float UFT = 1.1; // Upper Fall Threshold (1.7+0.5)/2
	const float LFT = -0.65; // Lower Fall Threshold (1.0+0.3)/2
	float LFT_time_us = 0;
	const int64_t MAX_FALLING_TIME_THRESHOLD = 600000;
	const int64_t MIN_FALLING_TIME_THRESHOLD = 150;

	// float highpass_acc = 0;
	// float last_highpass_acc = 0;
	// float last_acc_mag = 0;
	// const float highpass_alpha = 0.9;

	// float lowpass_acc = 0;
	// float last_lowpass_acc = 0;
	// const float lowpass_alpha = 0.25;

	// custom filter
	const int filt_order = 2;
	const int num_coeffs = filt_order*2+1; // = 5
	const double b[5] = {0.206572083826148,0,-0.413144167652296,0,0.206572083826148};
	const double a[5] = {1,-0.905078920874777,0.597907856327917,-0.290736791781056,0.195815712655833};
	double inputHistory[5] = {0.0, 0.0, 0.0, 0.0, 0.0,};
	double outputHistory[5] = {0.0, 0.0, 0.0, 0.0, 0.0,};
	double filtered = 0;

	vTaskDelay(pdMS_TO_TICKS(1000));

	while(1)
	{
    	if (!xQueueReceive(getQueue, &acc_and_time, portMAX_DELAY)) {
    		ESP_LOGE(TAG_main, "Accelerometer Queue Problem!");
    	}
    	acc_data = acc_and_time.vector_data;
    	time_us = acc_and_time.time_us;						// time of sample
		lsm6ds33_vector_calculate_acc_raw(&acc_data);
		acc_mag = lsm6ds33_vector_magnitude_of(acc_data); 	// linear acceleration magnitude in [mg]

		acc_mag = acc_mag / 1000.0 - 1;


		// filtr
		// Przesuń historię
		for (int i = num_coeffs - 1; i > 0; i--) {
			inputHistory[i] = inputHistory[i - 1];
			outputHistory[i] = outputHistory[i - 1];
		}
		inputHistory[0] = (double)acc_mag;
	
		// Obliczanie wyjścia
		filtered = 0.0;
		for (int i = 0; i < num_coeffs; i++) {
			filtered = filtered + b[i] * inputHistory[i];
			if (i > 1) {
				filtered = filtered - a[i] * outputHistory[i];
			}
		}
		outputHistory[0] = filtered;


		// // highpass - TO REDO
		// highpass_acc = highpass_alpha * (last_highpass_acc + acc_mag - last_acc_mag);
		// last_highpass_acc = highpass_acc;
		// last_acc_mag = acc_mag;

		// //lowpass - TO REDO
		// lowpass_acc = last_lowpass_acc + lowpass_alpha * (highpass_acc - last_lowpass_acc);
		// last_lowpass_acc = lowpass_acc;

		//ESP_LOGI(TAG_main, "time_us, acc_mag, highpass_acc, lowpass_acc == %lld, %f, %f, %f", time_us, acc_mag, highpass_acc, lowpass_acc);

		//change acc_mag after filters
		acc_mag = (float)filtered * (-1);

		// THE ALGORITHM
		switch (ACC_STATE) {
			case 0: // IDLE/LFT
				LFT_time_us = 0;
				if (acc_mag > LFT) {
					LFT_time_us = esp_timer_get_time();
					ACC_STATE = 1;
				} else {
					ACC_STATE = 0;
				}
				break;
			case 1: // UFT/TIMEOUT
				if (acc_mag > UFT) {
					if (esp_timer_get_time() - LFT_time_us < MIN_FALLING_TIME_THRESHOLD) {
						ACC_STATE = 1;
					} else {
						ACC_STATE = 2;
					}
				} else {
					if (esp_timer_get_time() - LFT_time_us > MAX_FALLING_TIME_THRESHOLD) {
						ACC_STATE = 0;
					} else {
						ACC_STATE = 1;
					}
				}
				break;
			case 2: // FALL DETECTED!
				xEventGroupSetBits(xFallDetectionGroupHandle, BIT_0);
				ESP_LOGI(TAG_main, "BIT_0 set (Accelerometer)");
				vTaskDelay(pdMS_TO_TICKS(2000));
				xEventGroupClearBits(xFallDetectionGroupHandle, BIT_0);
				ACC_STATE = 0;
				break;
			default:
				ACC_STATE = 0;
		}
	}
	vTaskDelete(NULL);
}

void gyroscope_maths_task(void *pvParameters)
{
	QueueHandle_t getQueue = pvParameters;
	struct vector_and_time_pack gyro_and_time;
	struct vector gyro_data;
	float gyro_mag;
	int64_t time_us = 0;
	int GYRO_STATE = 0;

	double lowpass = 0;
	double last_lowpass = 0;
	const double alpha = 0.385869545095038;

	const float GYRO_FALL_THRESHOLD = 241.675; //((365.7+304.6)/2+148.2)/2 = 241.675

	vTaskDelay(pdMS_TO_TICKS(1000));

	while(1)
	{
    	if (!xQueueReceive(getQueue, &gyro_and_time, portMAX_DELAY)) {
    		ESP_LOGE(TAG_main, "Gyroscope Queue Problem!");
    	}
    	gyro_data = gyro_and_time.vector_data;
    	time_us = gyro_and_time.time_us;					// time of sample
		lsm6ds33_vector_calculate_gyro_raw(&gyro_data);
		gyro_mag = lsm6ds33_vector_magnitude_of(gyro_data); // angular acceleration magnitude in [dps]

		// low pass I order
		lowpass = alpha * (double)gyro_mag + (1 - alpha) * last_lowpass;
		last_lowpass = lowpass;

		gyro_mag = (float)lowpass;

		// now time for THE ALGORITHM
		switch(GYRO_STATE) {
		case 0:
			if (gyro_mag > GYRO_FALL_THRESHOLD) {
				GYRO_STATE = 1;
			}
			break;
		case 1:
			xEventGroupSetBits(xFallDetectionGroupHandle, BIT_1);
			ESP_LOGI(TAG_main, "BIT_1 set (Gyroscope)");
			vTaskDelay(pdMS_TO_TICKS(2000));
			xEventGroupClearBits(xFallDetectionGroupHandle, BIT_1);
			GYRO_STATE = 0;
			break;
		}

	}
	vTaskDelete(NULL);
}

void barometer_maths_task(void *pvParameters)
{
	QueueHandle_t getQueue = pvParameters;
	struct press_and_time_pack press_and_time;
	float press_data;
	int64_t time_us = 0;
	int isFirstLoop = 1;

	float lowpass_press = 0;
	float last_lowpass_press = 0;
	const float lowpass_alpha = 0.01;

	vTaskDelay(pdMS_TO_TICKS(1000));

	while(1)
	{
		//ESP_LOGI(TAG_main, "Barometer maths %d", i++);

    	if (!xQueueReceive(getQueue, &press_and_time, portMAX_DELAY)) {
    		ESP_LOGE(TAG_main, "Barometer Queue Problem!");
    	}
    	press_data = (float)press_and_time.press_data / 4096.0;	// pressure in [hPa]
    	time_us = press_and_time.time_us;					// time of sample

    	// now time for THE ALGORITHM
    	if (isFirstLoop == 1) {
    		isFirstLoop = 0;
    		last_lowpass_press = press_data;
    	}

    	//lowpass - TO REDO
    	lowpass_press = last_lowpass_press + lowpass_alpha * (press_data - last_lowpass_press);
    	last_lowpass_press = lowpass_press;

    	press_data = lowpass_press;

    	//ESP_LOGI(TAG_main, "time_us, press == %lld, %f, ", time_us, press_data);
	}
	vTaskDelete(NULL);
}

void blinky(void *pvParameters)
{
	const int blink_gpio = 5;
	gpio_reset_pin(blink_gpio);
	gpio_set_direction(blink_gpio, GPIO_MODE_OUTPUT);

	vTaskDelay(5000 / portTICK_PERIOD_MS);
	while(1)
	{
	    /* Blink off (output low) */;
	    gpio_set_level(blink_gpio, 0);
	    vTaskDelay(1000 / portTICK_PERIOD_MS);
	    /* Blink on (output high) */
	    gpio_set_level(blink_gpio, 1);
	    vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

void fall_detected(void *pvParameters)
{
	EventBits_t fallDetectionBits;

	//int64_t time_us = esp_timer_get_time();
	//const int64_t ALARM_TIMEOUT = 10000000;
	const int alarm_gpio = 18;
	gpio_reset_pin(alarm_gpio);
	gpio_set_direction(alarm_gpio, GPIO_MODE_OUTPUT);

	while(1)
	{
		fallDetectionBits = xEventGroupWaitBits(
								xFallDetectionGroupHandle,		// event group handle
								BIT_0 | BIT_1,					// bits to wait for (acc and gyro)
								pdTRUE,							// clear bits on exit
								pdTRUE,							// wait for all bits
								portMAX_DELAY);					// wait indefinitely

		vTaskDelay(100 / portTICK_PERIOD_MS);

		if (eTaskGetState(send_telegram_msg) != eRunning) {
			time_t now = 0;
			struct tm timeinfo = { 0 };
			time(&now);
			localtime_r(&now, &timeinfo);
			char telegram_fall_msg[TELEGRAM_MSG_BUFF_SIZE];
			// exmple output: "[30/08/24, 07:30:18] Wykryto upadek!"
			strftime(telegram_fall_msg, sizeof(telegram_fall_msg), "%%5B%d%%2F%m%%2F%y,%%20%H:%M:%S%%5D%%20%%20%%20Wykryto%%20Upadek%%21", &timeinfo);

			xTaskCreate(send_telegram_msg, "send_telegram_msg", 4096, telegram_fall_msg, 2, NULL);
			ESP_LOGW(TAG_main, "Fall detected!");
		}

		for (int i = 0; i <= 10; i++) {
			/* Blink on (output high) */
			gpio_set_level(alarm_gpio, 1);
			vTaskDelay(250 / portTICK_PERIOD_MS);
			/* Blink off (output low) */;
			gpio_set_level(alarm_gpio, 0);
			vTaskDelay(250 / portTICK_PERIOD_MS);
		}
		vTaskDelay(3000 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

void check_battery(void *pvParameters)
{
	float vbat;
	gpio_num_t gpio_num;

	adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_12); //max 3.9V
	while(1)
	{
		int val = adc1_get_raw(ADC1_CHANNEL_7);
		adc1_pad_get_io_num(ADC1_CHANNEL_7, &gpio_num);

		//vbat = gpio_get_level(35);
		vbat = (3.9 * val / 4096) * 2;
		ESP_LOGI(TAG_main, "Battery: %f V, at GPIO%d\n", vbat, gpio_num);

		if (eTaskGetState(send_telegram_msg) != eRunning) {
			char telegram_fall_msg[TELEGRAM_MSG_BUFF_SIZE];
			sprintf(telegram_fall_msg, "Battery status: %0.2fV", vbat);
			xTaskCreate(send_telegram_msg, "send_telegram_msg", 4096, telegram_fall_msg, 2, NULL);
			ESP_LOGW(TAG_main, "Sent battery telegram message");
		}
		vTaskDelay(15*60*1000 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}

// Filter functions

float lowpass_filter(float x[], float y[], float a)
{
	int n = 1;
	return a * x[n] + (1 - a) * y[n-1];
}

float highpass_filter(float x[], float y[], float a)
{
	int n = 1;
	return a * (y[n-1] + x[n] - x[n-1]);
}

void app_main()
{
	// Initialize NVS (for wifi_init to use)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize Wifi, I2C and connect to sensors
	wifi_init_sta();
	i2c_helper_master_init();
	lps25h_test_connection();
	lsm6ds33_test_connection();
	
	//SNTP
	esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
	esp_sntp_setservername(0, "pl.pool.ntp.org");
	//sntp_set_time_sync_notification_cb(time_sync_notification_cb);
	//sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
	esp_sntp_init();
	time_t now = 0;
	struct tm timeinfo = { 0 };
	int retry = 0;
	const int retry_count = 10;
	while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
		ESP_LOGI(TAG_main, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
		vTaskDelay(2000 / portTICK_PERIOD_MS);
	}
	setenv("TZ", "CET-01CEST,M3.5.0,M10.5.0/3", 1);
	tzset();
	time(&now);
	localtime_r(&now, &timeinfo);
	char strftime_buf[64];
	strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
	ESP_LOGI(TAG_main, "The current date/time is: %s", strftime_buf);

	// Set up sensors
	lsm6ds33_default_setup();
	lps25h_default_setup();

	// TASKS AND EVENT GROUPS
	xFallDetectionGroupHandle = xEventGroupCreate();
	xQueueAccelerometerData = xQueueCreate(10, sizeof(struct vector_and_time_pack));
	xQueueGyroscopeData = xQueueCreate(10, sizeof(struct vector_and_time_pack));
	xQueueBarometerData = xQueueCreate(10, sizeof(struct press_and_time_pack));

	if(xQueueAccelerometerData != NULL && xQueueGyroscopeData != NULL && xQueueBarometerData != NULL && xFallDetectionGroupHandle != NULL)
	{
		ESP_LOGW(TAG_main, "tasks start");
		xTaskCreate(blinky, "blinky", 2048, NULL, 1, NULL);
		xTaskCreate(check_battery, "check_battery", 2048, NULL, 1, NULL);
		xTaskCreate(fall_detected, "fall_detected", 4096, NULL, 3, NULL);
		xTaskCreate(read_sensors_data_task, "read_sensors_data_task", 4096, NULL, 3, NULL);
		xTaskCreate(accelerometer_maths_task, "accelerometer_maths_task", 2048, (void*)xQueueAccelerometerData, 2, NULL);
		xTaskCreate(gyroscope_maths_task, "gyroscope_maths_task", 2048, (void*)xQueueGyroscopeData, 2, NULL);
		// xTaskCreate(barometer_maths_task, "barometer_maths_task", 2048, (void*)xQueueBarometerData, 2, NULL);
	}
}
