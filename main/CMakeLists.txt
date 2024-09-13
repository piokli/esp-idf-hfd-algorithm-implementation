idf_component_register(SRCS "hfd_main.c"
                    INCLUDE_DIRS "."
                    REQUIRES "driver" "lps25h" "lsm6ds33" "i2c_helper" "wifi_station" "tcp_client_helper" esp_http_client nvs_flash esp_timer)