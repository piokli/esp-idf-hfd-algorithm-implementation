# esp-idf-hfd-algorithm-implementation

This repo contains code to an old prototype of fall detection device. The main board used is a Feather HUZZAH32 from Adafruit 
which is basically an ESP-WROOM-32 dev board with the addition of a LiPo battery charger. The board is also connected via I2C to 
an AltIMU-10v5 board which has 4 sensors: LSM6DS33 (accelerometer + gyroscope), LPS25H (barometer) and LIS3MDL (magnetometer).

How it works (completed/not completed objectives):
1. Device connects to a known WiFi network
2. Reads data from sensors
3. Checks indepentendly for each sensor if fall was detected
4. Sends a "fall detected" message via Telegram bot

It uses ESP-IDF and freeRTOS.
Component folder contains custom made components (some based on exmples from ESP-IDF some written from scratch).
