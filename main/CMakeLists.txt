idf_component_register(
  SRC_DIRS "."
  INCLUDE_DIRS "."
  EMBED_FILES "../www/index.html"
  REQUIRES
        nvs_flash
        esp_event
  PRIV_REQUIRES
        esp_adc
        esp_timer
        esp_netif
        esp_wifi
        driver
        esp_driver_gpio
        esp_driver_i2c
        esp_http_server
        esp-zigbee-lib
        esp-zboss-lib
)