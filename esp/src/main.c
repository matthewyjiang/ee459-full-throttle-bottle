#include "esp_common.h"
#include "freertos/task.h"
#include "gpio.h"
#include "i2c_master.h" // Include the ESP8266 I2C master driver
#include "espconn.h"
#include "user_config.h"
#include <string.h>


// Defines for I2C operations
#define MAX_I2C_DEVICES 10    // Maximum number of I2C devices to scan for
#define I2C_SCAN_INTERVAL_MS 500  // Scan I2C bus every 5 seconds
#define I2C_WRITE_VALUE 0x55 // Example value to write to devices

// Global PWM variables
static uint8_t pwm_duty = 90;  // Initial duty cycle (90%)
static bool pwm_direction = true; // For breathing effect

// Array to store found I2C device addresses
static uint8_t i2c_devices[MAX_I2C_DEVICES];
static uint8_t i2c_device_count = 0;

// Function prototypes for I2C operations
void i2c_scan_task(void* pvParameters);
void i2c_read_write_task(void* pvParameters);
bool i2c_scan_bus(void);

#include "html.h"
// HTML content to serve
                          // HTTP server structures
struct espconn esp_conn;
esp_tcp esptcp;

// HTTP response header
const char *http_ok_header = "HTTP/1.1 200 OK\r\n"
                            "Content-Type: text/html\r\n"
                            "Connection: close\r\n\r\n";
const char *http_json_header = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n";


// Function prototypes
void user_conn_init(void);
void tcp_server_listen(void *arg);
void tcp_server_recv_cb(void *arg, char *data, unsigned short len);
void tcp_server_sent_cb(void *arg);
void tcp_server_discon_cb(void *arg);


// Called when client connects to the server
void tcp_server_listen(void *arg) {
    struct espconn *pesp_conn = (struct espconn *)arg;
    
    espconn_regist_recvcb(pesp_conn, tcp_server_recv_cb);
    espconn_regist_sentcb(pesp_conn, tcp_server_sent_cb);
    espconn_regist_disconcb(pesp_conn, tcp_server_discon_cb);
}

// Function to parse HTTP request and identify the endpoint
void parse_request(char *data, char *method, char *path, int max_len) {
    // Extract method (GET, POST, etc.)
    char *method_end = os_strstr(data, " ");
    if (method_end != NULL) {
        int method_len = method_end - data;
        if (method_len < max_len) {
            os_memcpy(method, data, method_len);
            method[method_len] = '\0';
        }
    }
    
    // Extract path
    char *path_start = method_end + 1;
    char *path_end = os_strstr(path_start, " ");
    if (path_end != NULL) {
        int path_len = path_end - path_start;
        if (path_len < max_len) {
            os_memcpy(path, path_start, path_len);
            path[path_len] = '\0';
        }
    }
}
// Called when data is received from the client
void tcp_server_recv_cb(void *arg, char *data, unsigned short len) {
    struct espconn *pesp_conn = (struct espconn *)arg;
    char method[10] = {0};
    char path[50] = {0};
    
    // Parse the HTTP request
    parse_request(data, method, path, sizeof(path));
    
    // Check if it's a GET request
    if (strcmp(method, "GET") == 0) {
        if (strcmp(path, "/") == 0) {
            // Serve the main HTML page
            char *response = (char *)os_zalloc(strlen(http_ok_header) + strlen(html_content) + 1);
            if (response != NULL) {
                strcpy(response, http_ok_header);
                strcat(response, html_content);
                espconn_send(pesp_conn, (uint8 *)response, strlen(response));
                free(response);
            } else {
                espconn_send(pesp_conn, (uint8 *)"HTTP/1.1 500 Internal Server Error\r\n\r\n", 36);
            }
        } 
        else if (strcmp(path, "/api/heat/on") == 0) {
            // Turn ON the heat
            
            
            // Send response
            char *response = (char *)os_zalloc(strlen(http_json_header) + strlen(heat_on_response) + 1);
            if (response != NULL) {
                strcpy(response, http_json_header);
                strcat(response, heat_on_response);
                espconn_send(pesp_conn, (uint8 *)response, strlen(response));
                free(response);
            } else {
                espconn_send(pesp_conn, (uint8 *)"HTTP/1.1 500 Internal Server Error\r\n\r\n", 36);
            }
        } 
        else if (strcmp(path, "/api/heat/off") == 0) {
            // Turn OFF the heat
            
            
            
            // Send response
            char *response = (char *)zalloc(strlen(http_json_header) + strlen(heat_off_response) + 1);
            if (response != NULL) {
                strcpy(response, http_json_header);
                strcat(response, heat_off_response);
                espconn_send(pesp_conn, (uint8 *)response, strlen(response));
                free(response);
            } else {
                espconn_send(pesp_conn, (uint8 *)"HTTP/1.1 500 Internal Server Error\r\n\r\n", 36);
            }
        } 
        else if (strcmp(path, "/api/heat/status") == 0) {
            // Return current heat status
            const char *status_json = heat_state ? 
                "{\"status\":\"on\",\"on\":true}" : 
                "{\"status\":\"off\",\"on\":false}";
            
            // Send response
            char *response = (char *)os_zalloc(strlen(http_json_header) + strlen(status_json) + 1);
            if (response != NULL) {
                strcpy(response, http_json_header);
                strcat(response, status_json);
                espconn_send(pesp_conn, (uint8 *)response, strlen(response));
                free(response);
            } else {
                espconn_send(pesp_conn, (uint8 *)"HTTP/1.1 500 Internal Server Error\r\n\r\n", 36);
            }
        } 
        else {
            // 404 Not Found for unknown endpoints
            espconn_send(pesp_conn, (uint8 *)"HTTP/1.1 404 Not Found\r\n\r\n", 26);
        }
    } else {
        // Method not supported
        espconn_send(pesp_conn, (uint8 *)"HTTP/1.1 405 Method Not Allowed\r\n\r\n", 35);
    }
}

// Called when data is sent to the client
void tcp_server_sent_cb(void *arg) {
    struct espconn *pesp_conn = (struct espconn *)arg;
    espconn_disconnect(pesp_conn);
}

// Called when connection is disconnected
void tcp_server_discon_cb(void *arg) {
    // Connection disconnected
    os_printf("Client disconnected\n");
}

// Initialize the HTTP server
void user_conn_init(void) {
    esp_conn.type = ESPCONN_TCP;
    esp_conn.state = ESPCONN_NONE;
    esp_conn.proto.tcp = &esptcp;
    esp_conn.proto.tcp->local_port = SERVER_PORT;
    
    espconn_regist_connectcb(&esp_conn, tcp_server_listen);
    
    espconn_accept(&esp_conn);
    
    os_printf("HTTP server started on port %d\n", SERVER_PORT);
}

/**
 * @brief Write data to an I2C device
 * 
 * @param device_addr Device address (7-bit)
 * @param data Pointer to data buffer
 * @param data_len Length of data to write
 * @return bool True if successful, false otherwise
 */
bool i2c_write_data(uint8_t device_addr, uint8_t* data, uint8_t data_len)
{
    uint8_t i;
    
    // Start condition
    i2c_master_start();
    
    // Send device address with write bit
    i2c_master_writeByte(device_addr << 1);
    if (!i2c_master_checkAck()) {
        i2c_master_stop();
        return false;
    }
    
    // Send data bytes
    for (i = 0; i < data_len; i++) {
        i2c_master_writeByte(data[i]);
        if (!i2c_master_checkAck()) {
            i2c_master_stop();
            return false;
        }
    }
    
    // Stop condition
    i2c_master_stop();
    return true;
}

/**
 * @brief Read data from an I2C device
 * 
 * @param device_addr Device address (7-bit)
 * @param data Pointer to data buffer to store read data
 * @param data_len Length of data to read
 * @return bool True if successful, false otherwise
 */
bool i2c_read_data(uint8_t device_addr, uint8_t* data, uint8_t data_len)
{
    uint8_t i;
    
    // Start condition
    i2c_master_start();
    
    // Send device address with read bit
    i2c_master_writeByte((device_addr << 1) | 0x01);
    if (!i2c_master_checkAck()) {
        i2c_master_stop();
        return false;
    }
    
    // Read data bytes
    for (i = 0; i < data_len; i++) {
        data[i] = i2c_master_readByte();
        
        // Send ACK for all bytes except the last one
        if (i < data_len - 1) {
            i2c_master_send_ack();
        } else {
            i2c_master_send_nack();  // Last byte gets NACK
        }
    }
    
    // Stop condition
    i2c_master_stop();
    return true;
}



/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
*******************************************************************************/
uint32 user_rf_cal_sector_set(void)
{
    flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;
    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;
        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;
        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;
        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;
        default:
            rf_cal_sec = 0;
            break;
    }
    return rf_cal_sec;
}


// I2C bus scanner task
bool i2c_scan_bus(void)
{
    uint8_t i;
    uint8_t found_count = 0;
    bool ack;
    
    // Clear previous scan results
    for (i = 0; i < MAX_I2C_DEVICES; i++) {
        i2c_devices[i] = 0;
    }
    
    // Scan all possible addresses (0x01-0x7F)
    for (i = 1; i < 128; i++) {
        i2c_master_start();
        i2c_master_writeByte(i << 1);
        i2c_master_stop();
        
        if (ack) {
            // Store the device address if we have space
            if (found_count < MAX_I2C_DEVICES) {
                i2c_devices[found_count] = i;
                found_count++;
            } else {
                break;
            }
        }
        
        // Small delay between scans
        os_delay_us(1000);
    }
    
    // Update global device count
    i2c_device_count = found_count;
    
    return (found_count > 0);
}

// I2C bus scanner task
void i2c_scan_task(void* pvParameters)
{
    while (true) {
        // Scan the I2C bus
        i2c_scan_bus();
        
        // Wait before next scan
        vTaskDelay(I2C_SCAN_INTERVAL_MS / portTICK_RATE_MS);
    }
    
    vTaskDelete(NULL);
}

// I2C read/write task for multiple devices
void i2c_read_write_task(void* pvParameters)
{
    uint8_t i;
    uint8_t data;
    
    while (true) {
        // Only proceed if we have devices
        if (i2c_device_count > 0) {
            // Loop through all found devices
            for (i = 0; i < i2c_device_count; i++) {
                uint8_t device_addr = i2c_devices[i];
                
                // Skip if the address is 0 (invalid)
                if (device_addr == 0) {
                    continue;
                }

                uint8_t* send_buffer;
                send_buffer = (uint8_t*)malloc(1);
                if (send_buffer == NULL) {
                    // Handle memory allocation failure
                    continue;
                }
                send_buffer[0] = 0x41; 

                
                i2c_write_data(device_addr, send_buffer, 1); // Write example value
                
                free(send_buffer); 
                
                
                // Small delay between devices
                vTaskDelay(100 / portTICK_RATE_MS);
            }
        }
        
        // Wait before next read/write cycle
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
    
    vTaskDelete(NULL);
}


void wifi_event_handler_cb(System_Event_t *event)
{
    if (event == NULL) {
        return;
    }

    switch (event->event_id) {
        case EVENT_STAMODE_GOT_IP:
            os_printf("sta got ip ,create task and free heap size is %d\n", system_get_free_heap_size());
            user_conn_init();
            break;

        case EVENT_STAMODE_CONNECTED:
            os_printf("sta connected\n");
            break;

        case EVENT_STAMODE_DISCONNECTED:
            wifi_station_connect();
            break;

        default:
            break;
    }
}

/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void user_init(void)
{
    
    // Initialize I2C master
    i2c_master_gpio_init();
    i2c_master_init();
    
    wifi_set_opmode(STATION_MODE); // Set WiFi mode to Station
    struct station_config config;
    bzero(&config, sizeof(struct station_config));
    sprintf(config.ssid, SSID);
    sprintf(config.password, PASSWORD);
    wifi_station_set_config(&config); // Set WiFi configuration


    wifi_station_set_hostname("ESP8266-WebServer");
    wifi_set_event_handler_cb(wifi_event_handler_cb); 

    wifi_station_connect();
    
    // Create I2C scanner task with lower priority
    xTaskCreate(&i2c_scan_task, "i2c_scan", 2048, NULL, 1, NULL);
    
    // Create I2C read/write task with medium priority
    xTaskCreate(&i2c_read_write_task, "i2c_rw", 2048, NULL, 2, NULL);
}