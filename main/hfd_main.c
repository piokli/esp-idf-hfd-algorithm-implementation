/* Hfd main source
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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

	const float MAX_ACC_THRESHOLD = 600;
	const float MIN_ACC_THRESHOLD = -250;
	const int64_t MAX_FALLING_TIME_THRESHOLD = 650;
	const int64_t MIN_FALLING_TIME_THRESHOLD = 150;
	const int64_t MAX_FALLING_TIME_OUT = 200000;
	float last_max_acc_val = 0;
	float last_min_acc_val = 0;
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

		// COS TU NIE TAK DZIAUA :<
		// highpass - TO REDO
		highpass_acc = highpass_alpha * (last_highpass_acc + acc_mag - last_acc_mag);
		last_highpass_acc = highpass_acc;
		last_acc_mag = acc_mag;

		//lowpass - TO REDO
		lowpass_acc = last_lowpass_acc + lowpass_alpha * (highpass_acc - last_lowpass_acc);
		last_lowpass_acc = lowpass_acc;

		ESP_LOGI(TAG_main, "time_us, acc_mag, highpass_acc, lowpass_acc == %lld, %f, %f, %f", time_us, acc_mag, highpass_acc, lowpass_acc);

		//change acc_mag after filters
		acc_mag = lowpass_acc;

		// now time for THE ALGORITHM
/*
		// get latest maximal accelerometer value
		if (acc_mag > last_max_acc_val  && acc_mag > MAX_ACC_THRESHOLD) {
			last_max_acc_val = acc_mag;
			last_max_val_time_us = time_us;
			ESP_LOGW(TAG_main, "last_max_acc = %f", last_max_acc_val);
		}
		// get latest minimal accelerometer value
		if (acc_mag < last_min_acc_val && acc_mag < MIN_ACC_THRESHOLD) {
			last_min_acc_val = acc_mag;
			last_min_val_time_us = time_us;
			ESP_LOGW(TAG_main, "last_min_acc = %f", last_min_acc_val);
		}

		min_max_delta_time_us = last_max_val_time_us - last_min_val_time_us;
		if (min_max_delta_time_us < MAX_FALLING_TIME_THRESHOLD && min_max_delta_time_us > MIN_FALLING_TIME_THRESHOLD) {
			ESP_LOGE(TAG_main, "FALL DETECTED!");
			// FALL DETECTED!
		}

		// poinno nie za kazdym razem wszystko sprawdzac ale jakies drzewko warunkowe
		if (last_max_val_time_us > MAX_FALLING_TIME_OUT) {
			last_max_acc_val = 0;
			last_min_acc_val = 0;
			last_min_val_time_us = 0;
			last_max_val_time_us = 0;
			min_max_delta_time_us = 0;
			ESP_LOGI(TAG_main, "RESET");
		}
		*/

		// ??????????? jak i kiedy resetowac warunki max i min? przed i po kazdym upadku
		//??????????????????????????????????
		//moze znalezc gdzies gotowca po prostu

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

		// now time for THE ALGORITHM

	}
	vTaskDelete(NULL);
}

void barometer_maths_task(void *pvParameters)
{
	QueueHandle_t getQueue = pvParameters;
	struct press_and_time_pack press_and_time;
	float press_data;
	int64_t time_us = 0;
	int i = 0;

/*
	float press_after_CMA;
	float press_after_EMA;
	static const float EMA_alpha = 0.01; // jaki ustawic?
	unsigned int n = 0;

	enum press_algorithm_state {WAITING_FOR_CHANGE, CHECKING_IF_FALL} state = WAITING_FOR_CHANGE;
	uint32_t saved_press_after_CMA = 0;
	int64_t saved_time_us = 0;
*/

	vTaskDelay(pdMS_TO_TICKS(1000));

	while(1)
	{
		//ESP_LOGI(TAG_main, "Barometer maths %d", i++);

    	if (!xQueueReceive(getQueue, &press_and_time, portMAX_DELAY))
    	{
    		ESP_LOGE(TAG_main, "Barometer Queue Problem!");
    	}
    	press_data = press_and_time.press_data / 4096.0;	// pressure in [hPa]
    	time_us = press_and_time.time_us;					// time of sample

    	// now time for THE ALGORITHM

    /*
    	// initialize CMA and EMA once
    	if (n == 0) {
    		press_after_CMA = press_data;
    		press_after_EMA = press_data;
    	}

    	// CMA - Cumulative Moving Average - slowest changing average for reference
    	press_after_CMA = (press_data + n * press_after_CMA) / (n + 1);
    	n++;

    	// EMA - Expotentional Moving Average - for finding height changes optimally fast
    	press_after_EMA = press_after_EMA + EMA_alpha * (press_data - press_after_EMA);

    	// Teraz je¿eli EMA jest wiêksze od CMA w danym momencie o wartosc wieksza o ?minimalna zmiana przy upadku?
    	// ale nie wiêksz¹ ni¿ ?zmiana wiêksza niz przy upadku by by³a? to dla tego samego CMA porównaj kolejne EMA ale te po
    	// kilku/nastu sekundach - ¿eby wyeliminowac zmiany nie wynikajace z upadku i sprawdzic czy wysokosc sie zmienila czy nie

    	ESP_LOGI(TAG_main,"EMA: %f, CMA: %f", press_after_EMA, press_after_CMA);

		switch(state)
		{
			case WAITING_FOR_CHANGE:
				if (press_after_EMA - press_after_CMA > 0.05) {
					saved_press_after_CMA = press_after_CMA;
					saved_time_us = time_us;
					state = CHECKING_IF_FALL;
					ESP_LOGI(TAG_main, "CHECKING_IF_FALL");
				}
				break;

			case CHECKING_IF_FALL:
				if (!(press_after_EMA - saved_press_after_CMA > 0.05)) {
					if (time_us - saved_time_us > 15000000) {
						// FALL DETECTED!
						ESP_LOGW(TAG_main, "FALL DETECTED! ~barometer");
					} else {
						state = WAITING_FOR_CHANGE;
						ESP_LOGI(TAG_main, "WAITING_FOR_CHANGE");
					}
				}
				break;
		}
*/
		// WNIOSKI!!!!!!! Jednak CMA wgl sie nie sprawdza tutaj i lepiej bedzie przechowywac stara probke ema sprzed X czasu i do niej porównywac
		// albo wgl jako ze i tak bedzie ttrzeba stworzyc na to bufor to moze po prostu wykorzystac tylko bufor kolowy dla ok. 100-200 próbek i
		// wyliczac srednia?? nie zle mowie - potrzebuje bufor usrednionych wartosci a nie probek samych w sobie...

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

	// TASKS
	xQueueAccelerometerData = xQueueCreate(10, sizeof(struct vector_and_time_pack));
	xQueueGyroscopeData = xQueueCreate(10, sizeof(struct vector_and_time_pack));
	xQueueBarometerData = xQueueCreate(10, sizeof(struct press_and_time_pack));
	if(xQueueAccelerometerData != NULL && xQueueGyroscopeData != NULL && xQueueBarometerData != NULL)
	{
		xTaskCreate(blinky, "blinky", 1024, NULL, 0, NULL); // "I'm alive!!!"
		//xTaskCreate(check_battery, "check_battery", 2048, NULL, 0, NULL);
		xTaskCreate(read_sensors_data_task, "read_sensors_data_task", 4096, NULL, 3, NULL);
		xTaskCreate(accelerometer_maths_task, "accelerometer_maths_task", 2048, (void*)xQueueAccelerometerData, 2, NULL);
		xTaskCreate(gyroscope_maths_task, "gyroscope_maths_task", 2048, (void*)xQueueGyroscopeData, 2, NULL);
		xTaskCreate(barometer_maths_task, "barometer_maths_task", 2048, (void*)xQueueBarometerData, 2, NULL);

		//xTaskCreate(tcp_client_task, "tcp_client", 4096, (void*)xQueue, 3, NULL);
		//xTaskCreate(test, "test", 4096, (void*)xQueue, 0, NULL);
	}
}
