# ESP32-IZ-POC
This repository was created to document all important information and codes about the recreated IZ POC using ESP32 board. Later, not only RSSI (hw) will be used, but also [SCI](https://github.com/espressif/esp-csi). It will be based on the [IZ Codes](https://github.com/Rafaelatff/IZ-codes), but not necessary be placed on IZ again. 

# Project explanation

[TBW]

# Board identification

The ESP32 development kits are identificated by its MAC. To aid with the identification, the board will have a colored label with its MAC address and will also be identificated by color.

![image](https://github.com/Rafaelatff/ESP32-IZ-POC/assets/58916022/92d28037-9e6d-4f47-ba7e-cf0b993fe9e1)

* `48:e7:29:ca:f9:78` - Green - GATEWAY
* `a0:b7:65:63:96:04` - Blue - Sensor node 1 
* `cc:db:a7:69:b7:e4` - Pink - Sensor node 2
* `48:e7:29:c8:fb:e8` - Orange - Sensor node 3
* `48:e7:29:c9:3b:20` - Yellow - Sensor node 4

# Code arranjment

The repositorie must hold the following folders, with the following codes:

* Hardware
  - Green board (Gateway)
  - Blue board (Sensor node 1)
  - Pink board (Sensor node 2)
  - Orange board (Sensor node 3)
  - Yellow (Sensor node 4)
* Firmware
  - Python N3 (Gathering RSSI PSR)
  - Python N5 (Parametrization)
  - Python N6 (Exibition)
 
# Hardware - Code explanation

All the ESP32 boards will have the main functions used and already explained on the [ESP NOW Repositorie](https://github.com/Rafaelatff/ESP32-WROOM-32-ESP-NOW/tree/main). Any difference from those code will be explained in this repositorie.

The functions are:
* `static esp_err_t init_wifi(void)` - Function used to do the WiFi Initialization.
* `static esp_err_t init_esp_now(void)` - Function used to do the ESP NOW Initialization.
* `static esp_err_t register_peer(uint8_t *peer_addr)` - Function used to register the Peers in the network.
* `static esp_err_t esp_now_send_data(const uint8_t *peer_addr, const uint8_t *data, uint8_t len)` - Function used to send data over the network.
* `void send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)` - Callback function to LOGI the sent data.
* `void recv_cb(const esp_now_recv_info_t * esp_now_info, const uint8_t *data, int data_len)` - Callback function to treat the received information. 

## Green board (Gateway)

## Blue board (Sensor node 1)

## Pink board (Sensor node 2)

## Orange board (Sensor node 3)

## Yellow (Sensor node 4)

# Firmware - Code explanation

## Python N3 (Gathering RSSI PSR)

## Python N5 (Parametrization)

## Python N6 (Exibition)
