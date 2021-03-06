/* Hfd main source
*/

#define BIT_0 ( 1 << 0)
#define BIT_1 ( 1 << 1)

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include <math.h>
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/adc.h"

#include "../components/i2c_helper/i2c_helper.h"
#include "../components/lps25h/lps25h.h"
#include "../components/lsm6ds33/lsm6ds33.h"
#include "../components/wifi_station/wifi_station.h"
#include "../components/tcp_client_helper/tcp_client_helper.h"

QueueHandle_t xQueueAccelerometerData;
QueueHandle_t xQueueGyroscopeData;
QueueHandle_t xQueueBarometerData;
TaskHandle_t xFallDetectedHandle;
EventGroupHandle_t xFallDetectionGroupHandle;


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
	int i = 0;

	vTaskDelay(1000 / portTICK_PERIOD_MS); //dla zasady

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

void accelerometer_maths_task(void *pvParameters)
{
	QueueHandle_t getQueue = pvParameters;
	struct vector_and_time_pack acc_and_time;
	struct vector acc_data;
	float acc_mag;
	int64_t time_us = 0;
	int i = 0;


	float UPV = 0; // Upper Peak Value
	float LPV = 0; // Lower Peak Value
	const float UFT = 600; // Upper Fall Threshold
	const float LFT = -250; // Lower Fall Threshold
	int ACC_STATE = 0;
	float LPV_time_us = 0;
	float UPV_time_us = 0;
	const int64_t MAX_FALLING_TIME_THRESHOLD = 600000;//650;

	const int64_t MIN_FALLING_TIME_THRESHOLD = 150;
	const int64_t MAX_FALLING_TIME_OUT = 200000;
	int64_t last_min_val_time_us = 0;
	int64_t last_max_val_time_us = 0;
	int64_t min_max_delta_time_us = 0;

	float highpass_acc = 0;
	float last_highpass_acc = 0;
	float last_acc_mag = 0;
	const float highpass_alpha = 0.9;

	float lowpass_acc = 0;
	float last_lowpass_acc = 0;
	const float lowpass_alpha = 0.25;

	vTaskDelay(pdMS_TO_TICKS(1000));

	while(1)
	{
		//ESP_LOGI(TAG_main, "Accelerometer maths %d", i++);

    	if (!xQueueReceive(getQueue, &acc_and_time, portMAX_DELAY))
    	{
    		ESP_LOGE(TAG_main, "Accelerometer Queue Problem!");
    	}
    	acc_data = acc_and_time.vector_data;
    	time_us = acc_and_time.time_us;						// time of sample
		lsm6ds33_vector_calculate_acc_raw(&acc_data);
		acc_mag = lsm6ds33_vector_magnitude_of(acc_data); 	// linear acceleration magnitude in [mg]

		// COS TU NIE TAK DZIAUA :< ??
		// highpass - TO REDO
		highpass_acc = highpass_alpha * (last_highpass_acc + acc_mag - last_acc_mag);
		last_highpass_acc = highpass_acc;
		last_acc_mag = acc_mag;

		//lowpass - TO REDO
		lowpass_acc = last_lowpass_acc + lowpass_alpha * (highpass_acc - last_lowpass_acc);
		last_lowpass_acc = lowpass_acc;

		//ESP_LOGI(TAG_main, "time_us, acc_mag, highpass_acc, lowpass_acc == %lld, %f, %f, %f", time_us, acc_mag, highpass_acc, lowpass_acc);

		//change acc_mag after filters
		acc_mag = lowpass_acc;

		// now time for THE ALGORITHM
		switch (ACC_STATE) {
			case 0: // Idle
				if (acc_mag < LFT) {
					LPV = acc_mag;
					ACC_STATE = 1;
				} else {
					ACC_STATE = 0;
				}
				break;
			case 1: // Lower Fall Zone
				//ESP_LOGI(TAG_main, " 1 ");
				if (acc_mag < LPV) {
					LPV = acc_mag;
					ACC_STATE = 1;
				} else {
					LPV_time_us = esp_timer_get_time();
					ACC_STATE = 2;
				}
				break;
			case 2: // LPV got, 2nd Idle with "counting down"
				//ESP_LOGI(TAG_main, " 2 ");
				UPV_time_us = esp_timer_get_time();
				//ESP_LOGW(TAG_main, "%f", UPV_time_us - LPV_time_us);
				if (UPV_time_us - LPV_time_us < MAX_FALLING_TIME_THRESHOLD) {
					if (acc_mag > UFT) {
						UPV = acc_mag;
						ACC_STATE = 3;
					} else {
						ACC_STATE = 2;
					}
				} else {
					ACC_STATE = 0; //time out, going to Idle
					//ESP_LOGI(TAG_main, " 0 ");
				}
				break;
			case 3: // Upper Fall Zone (practically this is a FALL, but first I'm looking for LPV)
				//ESP_LOGI(TAG_main, " 3 ");
				UPV_time_us = esp_timer_get_time();
				if (UPV_time_us - LPV_time_us < MAX_FALLING_TIME_THRESHOLD) {
					if (acc_mag > UPV) {
						UPV = acc_mag;
						ACC_STATE = 3;
					} else {
						//FALL DETECTED !!!
						ESP_LOGE(TAG_main, " F A L L,  D E T E C T E D ! ! !");
						//vTaskResume(xFallDetectedHandle);
						xEventGroupSetBits(xFallDetectionGroupHandle, BIT_0 | BIT_1);
					}
				} else {
					ACC_STATE = 0; //time out, going to Idle
					//ESP_LOGI(TAG_main, " 0 ");
				}
				break;
			default:
				ESP_LOGE(TAG_main, " default stateee?");
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
	int i = 0;

	const float GYRO_FALL_THRESHOLD = 280; // strzelam, needs to be tested and changed

	vTaskDelay(pdMS_TO_TICKS(1000));

	while(1)
	{
		//ESP_LOGI(TAG_main, "Gyroscope maths %d", i++);

    	if (!xQueueReceive(getQueue, &gyro_and_time, portMAX_DELAY))
    	{
    		ESP_LOGE(TAG_main, "Gyroscope Queue Problem!");
    	}
    	gyro_data = gyro_and_time.vector_data;
    	time_us = gyro_and_time.time_us;					// time of sample
		lsm6ds33_vector_calculate_gyro_raw(&gyro_data);
		gyro_mag = lsm6ds33_vector_magnitude_of(gyro_data); // angular acceleration magnitude in [dps]

		//ESP_LOGI(TAG_main, "%f", gyro_mag);

		// now time for THE ALGORITHM
		if (gyro_mag > 280) { 				//totally not bad, totally should work
			//FALL DETECTED !!!
			//vTaskResume(xFallDetectedHandle);
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

    	if (!xQueueReceive(getQueue, &press_and_time, portMAX_DELAY))
    	{
    		ESP_LOGE(TAG_main, "Barometer Queue Problem!");
    	}
    	press_data = (float)press_and_time.press_data / 4096.0;	// pressure in [hPa]
    	time_us = press_and_time.time_us;					// time of sample

    	// now time for THE ALGORITHM
    	if (isFirstLoop == 1)
    	{
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

void test(void *pvParameters)
{
	size_t memo;
	while(1)
	{
		vTaskDelay(100 / portTICK_PERIOD_MS);
		memo = xPortGetFreeHeapSize();
		printf("Free memory: %d\n", memo);
	}
	vTaskDelete(NULL);
}

void blinky(void *pvParameters)
{
	const int blink_gpio = 5;
	gpio_pad_select_gpio(blink_gpio);
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
	fallDetectionBits = xEventGroupWaitBits(
							xFallDetectionGroupHandle,		// event group handle
							BIT_0 | BIT_1,					// bits to wait for (acc and gyro)
							pdTRUE,							// clear bits on exit
							pdTRUE,							// wait for all bits
							portMAX_DELAY);					// wait indefinitely

	int64_t time_us = esp_timer_get_time();
	const int64_t ALARM_TIMEOUT = 10000000;
	const int alarm_gpio = 18;
	gpio_pad_select_gpio(alarm_gpio);
	gpio_set_direction(alarm_gpio, GPIO_MODE_OUTPUT);

	int counter = 0;
	const int max_times = 44;

	while(1)
	{
	    /* Blink off (output low) */;
	    gpio_set_level(alarm_gpio, 0);
	    vTaskDelay(250 / portTICK_PERIOD_MS);
	    /* Blink on (output high) */
	    gpio_set_level(alarm_gpio, 1);
	    vTaskDelay(250 / portTICK_PERIOD_MS);

	    if(counter++ >= max_times)
	    {
	    	counter = 0;
	    	gpio_set_level(alarm_gpio, 0);
	    	//vTaskSuspend(xFallDetectedHandle);
	    }
	}
	vTaskDelete(NULL);
}


void check_battery(void *pvParameters)
{
	//gpio_set_direction(35, GPIO_MODE_INPUT);
	float vbat;
	gpio_num_t gpio_num;

	adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11); //max 3.9V
	while(1)
	{
		int val = adc1_get_raw(ADC1_CHANNEL_7);
		adc1_pad_get_io_num(ADC1_CHANNEL_7, &gpio_num);

		//vbat = gpio_get_level(35);
		vbat = (3.9 * val / 4096) * 2;
		ESP_LOGI(TAG_main, "Battery: %f V, at GPIO%d\n", vbat, gpio_num);
		vTaskDelay(100 / portTICK_PERIOD_MS);
		//printf("Battery: %d V\n", vbat);
	}
	vTaskDelete(NULL);
}

//Functiones filteros

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
	//esp_log_level_set(tag, ESP_LOG_DEBUG);

    //Initialize NVS (for wifi_init to use)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize Wifi, I2C and connect to sensors
    vTaskDelay(100 / portTICK_PERIOD_MS);
	wifi_init_sta();
	i2c_helper_master_init();
	lps25h_test_connection();
	lsm6ds33_test_connection();

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
		xTaskCreate(blinky, "blinky", 1024, NULL, 0, NULL); // "I'm alive!!!"
		xTaskCreate(fall_detected, "fall_detected", 1024, NULL, 0, &xFallDetectedHandle);
		//vTaskSuspend(xFallDetectedHandle);
		//xTaskCreate(check_battery, "check_battery", 2048, NULL, 0, NULL);
		xTaskCreate(read_sensors_data_task, "read_sensors_data_task", 4096, NULL, 3, NULL);
		xTaskCreate(accelerometer_maths_task, "accelerometer_maths_task", 2048, (void*)xQueueAccelerometerData, 2, NULL);
		xTaskCreate(gyroscope_maths_task, "gyroscope_maths_task", 2048, (void*)xQueueGyroscopeData, 2, NULL);
		xTaskCreate(barometer_maths_task, "barometer_maths_task", 2048, (void*)xQueueBarometerData, 2, NULL);

		//xTaskCreate(tcp_client_task, "tcp_client", 4096, (void*)xQueue, 3, NULL);
		//xTaskCreate(test, "test", 4096, (void*)xQueue, 0, NULL);
	}
}
