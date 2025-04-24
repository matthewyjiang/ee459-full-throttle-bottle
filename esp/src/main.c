// --- FreeRTOS Includes (Essential for RTOS features) ---
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/projdefs.h" // For pdPASS etc.

// --- Standard ESP8266 RTOS SDK Includes ---
#include "esp_common.h"

// --- Peripheral/Driver Includes ---
#include "gpio.h"
#include "i2c_master.h"    // Your custom or library I2C driver
// #include "uart_register.h" // <<< REMOVED >>>

// --- Networking Includes ---
#include "espconn.h"     // For TCP server functions
#include "lwip/netif.h"  // Defines 'struct ip_info'

// --- Standard C Library Includes ---
#include <string.h>
#include <stdlib.h>
#include <stdio.h> // Defines sprintf

// --- Project Specific Includes ---
#include "user_config.h" // MUST contain SSID, PASSWORD, SERVER_PORT defines

// --- Configuration ---
#define ATTINY_SLAVE_ADDR   0x2A
#define CMD_READ_TEMP_REQ   0x01
#define CMD_WRITE_PWM_VAL   0x02
#define TEMP_READ_INTERVAL_MS 5000
#define UPDATE_INTERVAL_MS    2000

// --- I2C Pin Configuration (Verify!) ---
// #define I2C_MASTER_SDA_GPIO 4
// #define I2C_MASTER_SCL_GPIO 5

// --- Global Variables ---
volatile uint16_t g_current_temperature_raw = 0xFFFF;
volatile uint8_t g_current_pwm_setting = 0;
volatile bool g_heat_state = false;

// --- Web Interface HTML & JavaScript (Unchanged) ---
const char *html_page_content = // ... (Keep the HTML string from before) ...
    "<!DOCTYPE html><html><head><title>ATTiny Controller</title>"
    "<style>"
      "body { font-family: sans-serif; padding: 20px; background-color: #f4f4f4; }"
      "h1 { color: #333; border-bottom: 1px solid #ccc; padding-bottom: 10px; }"
      ".container { background-color: #fff; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); max-width: 500px; margin: auto; }"
      ".status-item { margin-bottom: 10px; } "
      ".label { display: inline-block; width: 120px; font-weight: bold; color: #555; }"
      ".value { font-weight: normal; color: #222; }"
      "#pwm_value { font-weight: bold; color: blue; }"
      "#temp_c { font-weight: bold; color: green; }"
      ".control-section { margin-top: 20px; padding-top: 15px; border-top: 1px dashed #ddd; }"
      "input[type=number] { width: 80px; padding: 5px; margin-right: 10px; }"
      "button { padding: 8px 15px; cursor: pointer; background-color: #007bff; color: white; border: none; border-radius: 4px; }"
      "button:hover { background-color: #0056b3; }"
      "#message { margin-top: 15px; color: #888; font-style: italic; min-height: 1.2em; }"
      "#temp_error { color: red; font-weight: bold; } "
    "</style></head>"
    "<body><div class='container'>"
      "<h1>ATTiny85 Controller</h1>"
      "<div class='status-item'><span class='label'>Heat Status:</span> <span class='value' id='heat_status'>--</span></div>"
      "<div class='status-item'><span class='label'>Current PWM:</span> <span class='value' id='pwm_value'>--</span></div>"
      "<div class='status-item'><span class='label'>Temperature:</span> <span class='value' id='temp_c'>--</span> °C <span id='temp_error'></span></div>"
      "<div class='status-item'><span class='label'>Raw ADC Temp:</span> <span class='value' id='raw_temp'>--</span></div>"
      "<div class='control-section'>"
        "<h2>Set PWM Duty Cycle</h2>"
        "<input type='number' id='pwm_input' min='0' max='255' placeholder='0-255'>"
        "<button onclick='setPWM()'>Set PWM</button>"
        "<div id='message'></div>"
      "</div>"
    "</div>"
    "<script>"
      "function updateStatus() {"
        "fetch('/api/status').then(response => response.json()).then(data => {"
          "document.getElementById('heat_status').innerText = data.status;"
          "document.getElementById('pwm_value').innerText = data.pwm;"
          "const tempErrorSpan = document.getElementById('temp_error');"
          "if (data.raw_temp === 0 && data.temp_c === -999.99) { "
              "document.getElementById('temp_c').innerText = 'Error';"
              "document.getElementById('raw_temp').innerText = 'Error';"
              "if (tempErrorSpan) tempErrorSpan.innerText = '(I2C?)';"
          "} else {"
              "document.getElementById('temp_c').innerText = data.temp_c.toFixed(2);"
              "document.getElementById('raw_temp').innerText = data.raw_temp;"
              "if (tempErrorSpan) tempErrorSpan.innerText = '';"
          "}"
          "if(data.status === 'on') { document.getElementById('heat_status').style.color = 'red'; } else { document.getElementById('heat_status').style.color = 'green'; }"
        "}).catch(error => {"
            "console.error('Error fetching status:', error);"
            "document.getElementById('message').innerText = 'Network error fetching status.'; "
        "});"
      "}"
      "function setPWM() {"
        "const pwmValue = document.getElementById('pwm_input').value;"
        "const messageDiv = document.getElementById('message');"
        "messageDiv.innerText = 'Setting PWM...';"
        "if (pwmValue >= 0 && pwmValue <= 255) {"
          "fetch(`/api/set_pwm?value=${pwmValue}`).then(response => response.json()).then(data => {"
            "messageDiv.innerText = data.message || 'Request sent.';"
            "if(data.success) { document.getElementById('pwm_value').innerText = data.pwm_set; }"
            "else { messageDiv.innerText = 'Error: ' + (data.message || 'Failed to set PWM'); }"
            "setTimeout(() => { messageDiv.innerText = ''; updateStatus(); }, 2500);"
          "}).catch(error => {"
              "messageDiv.innerText = 'Network error sending command.'; "
              "setTimeout(() => { messageDiv.innerText = ''; }, 3000);"
          "});"
        "} else {"
          "messageDiv.innerText = 'Invalid PWM value (0-255).';"
          "setTimeout(() => { messageDiv.innerText = ''; }, 3000);"
        "}"
      "}"
      "document.addEventListener('DOMContentLoaded', () => { updateStatus(); setInterval(updateStatus, %d); });"
    "</script></body></html>";


// --- HTTP Header Strings (Unchanged) ---
const char *http_ok_header = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n";
const char *http_json_header = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n";
const char *http_400_header = "HTTP/1.1 400 Bad Request\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n";
const char *http_404_header = "HTTP/1.1 404 Not Found\r\nContent-Type: text/plain\r\nConnection: close\r\n\r\n";
const char *http_500_header = "HTTP/1.1 500 Internal Server Error\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n";

// --- I2C Communication Functions (Unchanged) ---
bool attiny_read_temperature(uint16_t *raw_temp);
bool attiny_set_pwm(uint8_t pwm_value);

// --- FreeRTOS Task for Temperature Reading (Unchanged) ---
void temperature_reader_task(void* pvParameters);

// --- Web Server Code (Unchanged) ---
struct espconn esp_conn;
esp_tcp esptcp;

// Forward declarations (Unchanged)
void user_conn_init(void);
void tcp_server_listen(void *arg);
void tcp_server_recv_cb(void *arg, char *data, unsigned short len);
void tcp_server_sent_cb(void *arg);
void tcp_server_discon_cb(void *arg);
void parse_request(char *data, char *method, char *path_and_query, int max_len);

// --- Function Implementations ---

bool attiny_read_temperature(uint16_t *raw_temp) {
    uint8_t read_buffer[2];
    bool success = false;
    os_printf(">>> Attempting I2C Read...\n");
    
    i2c_master_start();
    i2c_master_writeByte((ATTINY_SLAVE_ADDR << 1) | 1);
    if (!i2c_master_checkAck()) { goto read_fail; }
    read_buffer[0] = i2c_master_readByte(); i2c_master_send_ack();
    read_buffer[1] = i2c_master_readByte(); i2c_master_send_nack();
    *raw_temp = ((uint16_t)read_buffer[1] << 8) | read_buffer[0];
    success = true;
read_fail:
    i2c_master_stop();
    if (!success) os_printf("!!! I2C Read Temp Failed !!!\n");
    os_printf("<<< I2C Read Finished (Success: %d)\n", success); 
    return success;
}

bool attiny_set_pwm(uint8_t pwm_value) {
    bool success = false;
    i2c_master_start();
    i2c_master_writeByte(ATTINY_SLAVE_ADDR << 1);
    if (!i2c_master_checkAck()) { goto write_fail; }
    i2c_master_writeByte(CMD_WRITE_PWM_VAL);
    if (!i2c_master_checkAck()) { goto write_fail; }
    i2c_master_writeByte(pwm_value);
    if (!i2c_master_checkAck()) { goto write_fail; }
    g_current_pwm_setting = pwm_value;
    g_heat_state = (pwm_value > 0);
    success = true;
write_fail:
    i2c_master_stop();
     if (!success) os_printf("!!! I2C Set PWM Failed (Value: %u) !!!\n", pwm_value);
    return success;
}

void temperature_reader_task(void* pvParameters) {
    uint16_t temp_raw_local;
    while (true) {
        if (attiny_read_temperature(&temp_raw_local)) {
            g_current_temperature_raw = temp_raw_local;
        } else {
            g_current_temperature_raw = 0xFFFF;
        }
        vTaskDelay(TEMP_READ_INTERVAL_MS / portTICK_RATE_MS);
    }
}

void tcp_server_listen(void *arg) {
    struct espconn *pesp_conn = (struct espconn *)arg;
    os_printf("Web Client connected\n");
    espconn_regist_recvcb(pesp_conn, tcp_server_recv_cb);
    espconn_regist_sentcb(pesp_conn, tcp_server_sent_cb);
    espconn_regist_disconcb(pesp_conn, tcp_server_discon_cb);
}

void parse_request(char *data, char *method, char *path_and_query, int max_len) { // Unchanged
    char *method_end = strchr(data, ' ');
    if (method_end == NULL || method_end - data >= 10) {
        method[0] = '\0'; path_and_query[0] = '\0'; return;
    }
    memcpy(method, data, method_end - data);
    method[method_end - data] = '\0';
    char *path_start = method_end + 1;
    while (*path_start == ' ') path_start++;
    char *path_end = strchr(path_start, ' ');
    if (path_end == NULL) path_end = strstr(path_start, "\r\n");
    if (path_end == NULL) path_end = path_start + strlen(path_start);
    int path_len = path_end - path_start;
    if (path_len < max_len) {
        memcpy(path_and_query, path_start, path_len);
        path_and_query[path_len] = '\0';
    } else {
        memcpy(path_and_query, path_start, max_len - 1);
        path_and_query[max_len - 1] = '\0';
    }
}

void send_json_response(struct espconn *pesp_conn, const char *header, const char *json_body) { // Unchanged
    size_t header_len = strlen(header);
    size_t body_len = strlen(json_body);
    size_t total_len = header_len + body_len;
    char *response = (char *)os_zalloc(total_len + 1);
    if (response != NULL) {
        memcpy(response, header, header_len);
        memcpy(response + header_len, json_body, body_len);
        response[total_len] = '\0';
        espconn_send(pesp_conn, (uint8 *)response, total_len);
        os_free(response);
    } else {
        os_printf("!!! Error: Failed alloc for JSON response (%u bytes) !!!\n", (unsigned int)(total_len + 1));
        espconn_send(pesp_conn, (uint8 *)"HTTP/1.1 500 Internal Server Error\r\n\r\n", 36);
    }
}

void tcp_server_recv_cb(void *arg, char *data, unsigned short len) { // Unchanged
    struct espconn *pesp_conn = (struct espconn *)arg;
    char method[10] = {0};
    char path_and_query[128] = {0};
    if (len == 0 || data == NULL) { return; }
    parse_request(data, method, path_and_query, sizeof(path_and_query));
    char *query_string = strchr(path_and_query, '?');
    char *path = path_and_query;
    if (query_string != NULL) { *query_string = '\0'; query_string++; }

    if (strcmp(method, "GET") == 0) {
        if (strcmp(path, "/") == 0) {
            size_t html_base_len = strlen(html_page_content);
            char *html_response_body = (char*)os_zalloc(html_base_len + 20);
            if (html_response_body != NULL) {
                int formatted_len = sprintf(html_response_body, html_page_content, UPDATE_INTERVAL_MS);
                size_t header_len = strlen(http_ok_header);
                size_t body_len = (formatted_len > 0) ? formatted_len : html_base_len;
                char *full_response = (char *)os_zalloc(header_len + body_len + 1);
                if (full_response != NULL) {
                    memcpy(full_response, http_ok_header, header_len);
                    memcpy(full_response + header_len, html_response_body, body_len);
                    full_response[header_len + body_len] = '\0';
                    espconn_send(pesp_conn, (uint8 *)full_response, header_len + body_len);
                    os_free(full_response);
                } else {
                     os_printf("!!! Error: Failed mem for full HTML response !!!\n");
                    espconn_send(pesp_conn, (uint8 *)"HTTP/1.1 500 Internal Server Error\r\n\r\n", 36);
                }
                 os_free(html_response_body);
            } else {
                 os_printf("!!! Error: Failed mem for HTML body buffer !!!\n");
                 espconn_send(pesp_conn, (uint8 *)"HTTP/1.1 500 Internal Server Error\r\n\r\n", 36);
            }
        }
        else if (strcmp(path, "/api/status") == 0) {
            char status_json[180];
            float temp_celsius = -999.99;
            uint16_t temp_raw_copy = g_current_temperature_raw;
            if (temp_raw_copy != 0xFFFF) {
                 float voltage = (temp_raw_copy / 1023.0) * 3.3; // Assumes 3.3V Vref
                 temp_celsius = voltage * 100.0; // Example for LM35
            }
            bool heat_state_copy = g_heat_state;
            uint8_t pwm_setting_copy = g_current_pwm_setting;
            sprintf(status_json,
                    "{\"status\":\"%s\",\"on\":%s,\"raw_temp\":%u,\"temp_c\":%.2f,\"pwm\":%u}",
                    heat_state_copy ? "on" : "off",
                    heat_state_copy ? "true" : "false",
                    (temp_raw_copy == 0xFFFF) ? 0 : temp_raw_copy,
                    temp_celsius,
                    pwm_setting_copy);
            send_json_response(pesp_conn, http_json_header, status_json);
        }
        else if (strcmp(path, "/api/set_pwm") == 0) {
            int pwm_val = -1;
            if (query_string != NULL) {
                char *val_str = strstr(query_string, "value=");
                if (val_str != NULL && val_str[6] >= '0' && val_str[6] <= '9') {
                    pwm_val = atoi(val_str + 6);
                }
            }
            if (pwm_val >= 0 && pwm_val <= 255) {
                if (attiny_set_pwm((uint8_t)pwm_val)) {
                    char resp_json[100];
                    uint8_t current_pwm = g_current_pwm_setting;
                    sprintf(resp_json, "{\"success\":true, \"message\":\"PWM set to %d\", \"pwm_set\":%u}", pwm_val, current_pwm);
                    send_json_response(pesp_conn, http_json_header, resp_json);
                } else {
                    send_json_response(pesp_conn, http_500_header, "{\"success\":false, \"message\":\"I2C communication failed\"}");
                }
            } else {
                 send_json_response(pesp_conn, http_400_header, "{\"success\":false, \"message\":\"Invalid 'value' parameter (0-255)\"}");
            }
        }
        else { // Route Not Found
             os_printf("!!! 404 Not Found: %s !!!\n", path);
            char response_body[] = "Resource not found.";
            char *full_response = (char*)os_zalloc(strlen(http_404_header) + sizeof(response_body));
            if(full_response) {
                strcpy(full_response, http_404_header);
                strcat(full_response, response_body);
                espconn_send(pesp_conn, (uint8 *)full_response, strlen(full_response));
                os_free(full_response);
            } else {
                espconn_send(pesp_conn, (uint8 *)"HTTP/1.1 404 Not Found\r\n\r\n", 26);
            }
        }
    } else { // Method Not Supported
         os_printf("!!! Unsupported method: %s !!!\n", method);
        send_json_response(pesp_conn, http_500_header, "{\"success\":false, \"message\":\"Method Not Allowed\"}");
    }
} // END of tcp_server_recv_cb


void tcp_server_sent_cb(void *arg) {
    struct espconn *pesp_conn = (struct espconn *)arg;
    espconn_disconnect(pesp_conn);
}


void tcp_server_discon_cb(void *arg) {
     os_printf("Web Client disconnected\n");
}


void wifi_event_handler_cb(System_Event_t *event) { // Unchanged
    if (event == NULL) { return; }
     os_printf("WiFi Event: %d\n", event->event_id);
    switch (event->event_id) {
        case EVENT_STAMODE_GOT_IP: {
            struct ip_info ip_config;
            wifi_get_ip_info(STATION_IF, &ip_config);
             os_printf("--> WiFi Got IP: " IPSTR "\n", IP2STR(&ip_config.ip));
             os_printf("--> Free heap: %d\n", system_get_free_heap_size());
            user_conn_init(); // Start the server now
            break;
        }
        case EVENT_STAMODE_CONNECTED:
             os_printf("--> WiFi Connected to AP\n");
            break;
        case EVENT_STAMODE_DISCONNECTED: {
            Event_StaMode_Disconnected_t *info = &event->event_info.disconnected;
             os_printf("--> WiFi Disconnected (Reason: %d). Reconnecting...\n", info->reason);
            vTaskDelay(1000 / portTICK_RATE_MS); // Delay 1 second
            wifi_station_connect();
            break;
        }
        default: break;
    }
}


uint32 user_rf_cal_sector_set(void) { // Unchanged
    flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;
    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256: rf_cal_sec = 128 - 5; break;
        case FLASH_SIZE_8M_MAP_512_512: rf_cal_sec = 256 - 5; break;
        case FLASH_SIZE_16M_MAP_512_512: case FLASH_SIZE_16M_MAP_1024_1024: rf_cal_sec = 512 - 5; break;
        case FLASH_SIZE_32M_MAP_512_512: case FLASH_SIZE_32M_MAP_1024_1024: rf_cal_sec = 1024 - 5; break;
        case FLASH_SIZE_64M_MAP_1024_1024: rf_cal_sec = 2048 - 5; break;
        case FLASH_SIZE_128M_MAP_1024_1024: rf_cal_sec = 4096 - 5; break;
        default: rf_cal_sec = 0; break;
    }
    return rf_cal_sec;
}


void user_conn_init(void) { // Unchanged
    memset(&esp_conn, 0, sizeof(esp_conn));
    memset(&esptcp, 0, sizeof(esptcp));
    esp_conn.type = ESPCONN_TCP;
    esp_conn.state = ESPCONN_NONE;
    esp_conn.proto.tcp = &esptcp;
    esp_conn.proto.tcp->local_port = SERVER_PORT;
    espconn_regist_connectcb(&esp_conn, tcp_server_listen);
    sint8 ret = espconn_accept(&esp_conn);
    if (ret == 0) {
         os_printf("--> HTTP server listening on port %d\n", SERVER_PORT);
    } else {
         os_printf("!!! Error starting HTTP listener: %d !!!\n", ret);
    }
}


void user_init(void) {
    // <<< REMOVED explicit UART initialization >>>
    // The SDK usually initializes UART0 by default for os_printf

    // Add a small delay to allow serial monitor to connect after flashing
    vTaskDelay(1000 / portTICK_RATE_MS); // Delay 1 second

    os_printf("\n\n===================================\n");
    os_printf("ATTiny Controller - Starting Up\n");
    os_printf("SDK version: %s\n", system_get_sdk_version());
    os_printf("Initial free heap: %d\n", system_get_free_heap_size());

    // I2C Init
    i2c_master_gpio_init();
    i2c_master_init();
    os_printf("I2C Master Initialized\n");

    // WiFi Init
    os_printf("Initializing WiFi...\n");
    wifi_set_opmode(STATION_MODE);
    struct station_config config;
    bzero(&config, sizeof(struct station_config));
    strncpy((char*)config.ssid, SSID, sizeof(config.ssid) - 1);
    strncpy((char*)config.password, PASSWORD, sizeof(config.password) - 1);
    config.ssid[sizeof(config.ssid) - 1] = '\0';
    config.password[sizeof(config.password) - 1] = '\0';
    os_printf("Setting WiFi config for SSID: %s\n", config.ssid);
    wifi_station_set_config(&config);
    wifi_station_set_hostname("ATTiny-Ctrl");
    wifi_set_event_handler_cb(wifi_event_handler_cb);

    // Create Task
    os_printf("Creating tasks...\n");
    #ifndef pdPASS
    #define pdPASS (1)
    #endif
    int task_created = xTaskCreate(
                                  &temperature_reader_task, "TempReader",
                                  256, NULL, 5, NULL);

    if (task_created == pdPASS) {
        os_printf("Temperature reader task created.\n");
    } else {
        os_printf("!!! Error: Failed to create Temp task! (Code: %d) !!!\n", task_created);
    }

    os_printf("Connecting to WiFi...\n");
    wifi_station_connect();

    os_printf("System Init Complete. Heap: %d\n", system_get_free_heap_size());
    os_printf("===================================\n");

    // Scheduler starts automatically
}