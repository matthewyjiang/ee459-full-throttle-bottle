# CMakeLists.txt for the main component

# Specify the source file, include directories, and all requirements
# for the 'main' component in a single registration call.
idf_component_register(SRCS "esp32_master.c"        # <<< Make sure this filename is correct
                       INCLUDE_DIRS "."
                       REQUIRES
                           nvs_flash                # For Non-Volatile Storage
                           esp_event                # For ESP event loop system
                           freertos                 # For FreeRTOS features (tasks, mutex, delay)
                           esp_netif                # For TCP/IP network interface layer
                           esp_wifi                 # For Wi-Fi driver and configuration
                           log
                           lwip                     # For LwIP TCP/IP stack
                           driver                   # For I2C peripheral driver
                           esp_http_server          # For the HTTP Web Server
                      )