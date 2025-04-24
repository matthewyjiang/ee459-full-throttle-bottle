#include <stdio.h>
#include <string.h>
#include <stdlib.h> // For atoi
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h" // For mutex

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "driver/i2c.h"
#include "esp_http_server.h"

// --- Configuration ---
// WiFi Credentials
#define WIFI_SSID      "Xiaolei"       // Set via menuconfig
#define WIFI_PASSWORD  "pear8888"   // Set via menuconfig

// I2C Configuration
#define I2C_MASTER_SCL_IO           GPIO_NUM_22      // ESP32 SCL Pin
#define I2C_MASTER_SDA_IO           GPIO_NUM_21      // ESP32 SDA Pin
#define I2C_MASTER_NUM              I2C_NUM_0        // I2C port number
#define I2C_MASTER_FREQ_HZ          50000           // I2C master clock frequency (100kHz)
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0

// ATTiny Communication
#define ATTINY_SLAVE_ADDR           0x2A             // Slave address (must match ATTiny code)
#define READ_TEMP_CMD               0x01             // Command byte to request temperature (ATTiny needs to handle this if used)
#define WRITE_PWM_CMD               0x02             // Command byte to write PWM value

// Web Server & Control
#define UPDATE_INTERVAL_MS          5000             // How often to fetch temp for display update

// --- Global Variables & Handles ---
static const char *TAG = "ATTINY_CTRL";

// WiFi connection status
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int s_retry_num = 0;

// I2C Communication Data (shared between tasks and ISRs/Handlers)
// Use mutex for thread safety when accessing shared I2C resources/data
SemaphoreHandle_t i2c_mutex = NULL;
volatile uint16_t current_raw_temp = 0xFFFF; // Current temp reading (0xFFFF indicates error/not read yet)
volatile uint8_t current_pwm_value = 0;     // Current PWM value sent

// --- Function Prototypes ---
// WiFi
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data);
static void wifi_init_sta(void);

// I2C
static esp_err_t i2c_master_init(void);
static esp_err_t attiny_read_temperature_raw(uint16_t *raw_temp);
static esp_err_t attiny_set_pwm_value(uint8_t pwm_value);

// Web Server Handlers
static esp_err_t root_get_handler(httpd_req_t *req);
static esp_err_t set_pwm_get_handler(httpd_req_t *req);
static esp_err_t get_data_get_handler(httpd_req_t *req);
static httpd_handle_t start_webserver(void);

// Tasks
void i2c_update_task(void *pvParameters);

// --- WiFi Initialization ---
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 5) { // Example: Retry 5 times
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK, // Adjust if needed
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    // Wait for connection or failure
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s", WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s", WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
    // Event group is not deleted here; could be used for disconnect handling
}

// --- I2C Initialization & Communication ---
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(err));
        return err;
    }
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
     if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
    }
    // Create Mutex for I2C access
    i2c_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C mutex");
        // Handle error appropriately - perhaps halt or return specific error
        return ESP_FAIL;
    }

    return err;
}

// Reads the 2-byte raw temperature value from the ATTiny
static esp_err_t attiny_read_temperature_raw(uint16_t *raw_temp) {
    if (raw_temp == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t read_buffer[2] = {0};
    esp_err_t err = ESP_FAIL;

    // Try to take the mutex, wait max 100ms
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        // Send slave address with R/W bit set to READ (1)
        i2c_master_write_byte(cmd, (ATTINY_SLAVE_ADDR << 1) | I2C_MASTER_READ, true /* ACK_CHECK_EN */);
        // Read two bytes from slave
        i2c_master_read(cmd, read_buffer, 2, I2C_MASTER_LAST_NACK); // NACK the last byte
        i2c_master_stop(cmd);

        err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(500)); // 100ms timeout
        i2c_cmd_link_delete(cmd);

        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Received bytes: Byte0=0x%02X, Byte1=0x%02X", read_buffer[0], read_buffer[1]); // <<< ADD THIS
            *raw_temp = ((uint16_t)read_buffer[1] << 8) | read_buffer[0];
            // ESP_LOGI(TAG, "Combined value = %u", *raw_temp); // Optional
        } else {
            ESP_LOGE(TAG, "I2C Read Failed: %s", esp_err_to_name(err));
            *raw_temp = 0xFFFF; // Indicate error
        }
        xSemaphoreGive(i2c_mutex); // Release mutex
    } else {
        ESP_LOGW(TAG, "Could not obtain I2C mutex for reading");
        err = ESP_ERR_TIMEOUT; // Indicate mutex timeout
         *raw_temp = 0xFFFF;
    }

    return err;
}

// Sends the PWM command and value to the ATTiny
static esp_err_t attiny_set_pwm_value(uint8_t pwm_value) {
    esp_err_t err = ESP_FAIL;
    uint8_t write_buffer[2];

    write_buffer[0] = WRITE_PWM_CMD; // Command byte
    write_buffer[1] = pwm_value;     // PWM value

    // Try to take the mutex, wait max 100ms
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        // Send slave address with R/W bit set to WRITE (0)
        i2c_master_write_byte(cmd, (ATTINY_SLAVE_ADDR << 1) | I2C_MASTER_WRITE, true /* ACK_CHECK_EN */);
        // Send the command and value bytes
        i2c_master_write(cmd, write_buffer, 2, true /* ACK_CHECK_EN */);
        i2c_master_stop(cmd);

        err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100)); // 100ms timeout
        i2c_cmd_link_delete(cmd);

        if (err == ESP_OK) {
            ESP_LOGD(TAG, "I2C Write successful. Set PWM = %u", pwm_value);
            // Update global state only on successful write
            current_pwm_value = pwm_value;
        } else {
            ESP_LOGE(TAG, "I2C Write Failed: %s", esp_err_to_name(err));
        }
         xSemaphoreGive(i2c_mutex); // Release mutex
    } else {
        ESP_LOGW(TAG, "Could not obtain I2C mutex for writing");
        err = ESP_ERR_TIMEOUT; // Indicate mutex timeout
    }
    return err;
}


// --- Web Server Handlers ---

// Serves the main HTML page
static esp_err_t root_get_handler(httpd_req_t *req) {
    // Simple HTML page with JS for interaction
    const char* html_page = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>ATTiny Control</title>
    <meta name='viewport' content='width=device-width, initial-scale=1'>
    <style>
        body { font-family: sans-serif; padding: 20px; }
        .control { margin-bottom: 15px; }
        label { display: inline-block; width: 100px; }
        input[type=range] { width: 200px; }
        #pwmValueLabel { font-weight: bold; }
        #temp { font-weight: bold; color: blue; }
        #status { margin-top: 10px; font-style: italic; color: grey; }
    </style>
</head>
<body>
    <h1>ATTiny Temperature & Heater Control</h1>

    <div class='control'>
        <label>Temperature:</label>
        <span id='temp'>--</span> °C (Raw: <span id='tempRaw'>--</span>)
    </div>

    <div class='control'>
        <label for='pwmSlider'>Heater PWM:</label>
        <input type='range' id='pwmSlider' name='pwm' min='0' max='255' value='0' oninput='updatePwmLabel(this.value)'>
        <span id='pwmValueLabel'>0</span> / 255
    </div>
    <button onclick='setPWM()'>Set PWM</button>

    <div id='status'>Loading...</div>

    <script>
        let currentRawTemp = null;
        let currentPwm = 0;

        function updatePwmLabel(value) {
            document.getElementById('pwmValueLabel').innerText = value;
            // Optional: Send PWM immediately on slider change (can cause many requests)
            // setPWM(value);
        }

        function setPWM() {
            const pwmVal = document.getElementById('pwmSlider').value;
            const statusEl = document.getElementById('status');
            statusEl.innerText = 'Setting PWM to ' + pwmVal + '...';
            fetch('/set-pwm?value=' + pwmVal)
                .then(response => response.text())
                .then(data => {
                    statusEl.innerText = 'Set PWM Response: ' + data;
                    if (data.startsWith("OK")) {
                       currentPwm = parseInt(pwmVal); // Update local state on success
                       document.getElementById('pwmValueLabel').innerText = currentPwm;
                    } else {
                       // Optionally revert slider if failed?
                    }
                })
                .catch(error => {
                    statusEl.innerText = 'Error setting PWM: ' + error;
                });
        }

        function fetchData() {
            const statusEl = document.getElementById('status');
            // statusEl.innerText = 'Fetching data...'; // Can be noisy
            fetch('/get-data')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('tempRaw').innerText = data.raw_temp !== null ? data.raw_temp : 'Error';
                    document.getElementById('pwmValueLabel').innerText = data.current_pwm;
                    document.getElementById('pwmSlider').value = data.current_pwm; // Sync slider

                    // --- Temperature Conversion Placeholder ---
                    // Replace this with your actual sensor conversion logic
                    let tempC = 'N/A';
                    if (data.raw_temp !== null && data.raw_temp !== 65535) { // 65535 is our error code
                        // Example for a sensor where voltage proportional to Temp (like LM35)
                        // Assuming Vref = 3.3V, 10-bit ADC on ATTiny
                         // float voltage = (data.raw_temp / 1023.0) * 3.3;
                         // tempC = (voltage - 0.5) * 100.0; // Example for LM35 (adjust offset/scale)

                        // Example for a thermistor with voltage divider - more complex calculation needed
                         tempC = data.raw_temp; // Show raw value if no conversion done
                    } else {
                        tempC = 'Error';
                    }
                    // --- End Placeholder ---

                    document.getElementById('temp').innerText = (typeof tempC === 'number') ? tempC.toFixed(2) : tempC;

                    statusEl.innerText = 'Data updated: ' + new Date().toLocaleTimeString();
                })
                .catch(error => {
                    document.getElementById('temp').innerText = 'Error';
                    document.getElementById('tempRaw').innerText = 'Error';
                    statusEl.innerText = 'Error fetching data: ' + error;
                });
        }

        // Initial fetch and set interval
        document.addEventListener('DOMContentLoaded', (event) => {
            fetchData(); // Initial data load
            setInterval(fetchData, %d); // Periodic update
             // Set initial slider value (in case page load is slow)
             document.getElementById('pwmSlider').value = currentPwm;
             updatePwmLabel(currentPwm);
        });
    </script>
</body>
</html>
)rawliteral";

    // Calculate buffer size needed for the formatted HTML
    int required_size = snprintf(NULL, 0, html_page, UPDATE_INTERVAL_MS) + 1;
    char *formatted_html = malloc(required_size);
    if (!formatted_html) {
        ESP_LOGE(TAG, "Failed to allocate memory for HTML page");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    snprintf(formatted_html, required_size, html_page, UPDATE_INTERVAL_MS);

    httpd_resp_set_type(req, "text/html");
    esp_err_t ret = httpd_resp_send(req, formatted_html, HTTPD_RESP_USE_STRLEN);
    free(formatted_html); // Free the allocated memory
    return ret;
}


// Handles setting the PWM value via a GET request (/set-pwm?value=XXX)
static esp_err_t set_pwm_get_handler(httpd_req_t *req) {
    char buf[32]; // Buffer for query parameter
    size_t buf_len = sizeof(buf);
    uint8_t pwm_val = 0;

    // Get query parameter "value"
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
        ESP_LOGI(TAG, "Got query: %s", buf);
        char param[10]; // Buffer for parsed value
        if (httpd_query_key_value(buf, "value", param, sizeof(param)) == ESP_OK) {
            ESP_LOGI(TAG, "Got value: %s", param);
            int temp_val = atoi(param); // Convert string to integer
            if (temp_val >= 0 && temp_val <= 255) {
                pwm_val = (uint8_t)temp_val;
                 ESP_LOGI(TAG, "Parsed PWM value: %u", pwm_val);

                // Call the I2C function to set PWM
                esp_err_t i2c_err = attiny_set_pwm_value(pwm_val);

                if (i2c_err == ESP_OK) {
                    httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
                    return ESP_OK;
                } else {
                    ESP_LOGE(TAG, "Failed to set PWM via I2C");
                    httpd_resp_send_500(req); // Internal Server Error
                    return ESP_FAIL;
                }
            } else {
                 ESP_LOGW(TAG, "Invalid PWM value received: %d", temp_val);
            }
        }
    }

    // If query parameter is missing or invalid
    // If query parameter is missing or invalid
    ESP_LOGW(TAG, "Sending 400 Bad Request response");
    httpd_resp_set_status(req, "400 Bad Request"); // Set the HTTP status code
    httpd_resp_send(req, NULL, 0); // Send an empty response body
    return ESP_FAIL;
}

// Handles providing current data (temp, pwm) as JSON
static esp_err_t get_data_get_handler(httpd_req_t *req) {
    char json_buffer[100];

    // Access the globally stored values (read directly is usually fine for volatile reads)
    uint16_t temp_to_send = current_raw_temp;
    uint8_t pwm_to_send = current_pwm_value;


    // Format as JSON. Handle potential error value for temp.
    snprintf(json_buffer, sizeof(json_buffer),
             "{\"raw_temp\": %s, \"current_pwm\": %u}",
             (temp_to_send == 0xFFFF) ? "null" : "%u", // Send null if temp is invalid
             pwm_to_send);

     // Correctly format if temp is valid
     if (temp_to_send != 0xFFFF) {
         snprintf(json_buffer, sizeof(json_buffer),
             "{\"raw_temp\": %u, \"current_pwm\": %u}",
             temp_to_send, pwm_to_send);
     }


    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_buffer, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

// --- Web Server Setup ---
static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true; // Purge oldest session to make room for new

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_uri_t root_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = root_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &root_uri);

         httpd_uri_t set_pwm_uri = {
            .uri       = "/set-pwm",
            .method    = HTTP_GET, // Using GET for simplicity here, POST is often preferred for actions
            .handler   = set_pwm_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &set_pwm_uri);

         httpd_uri_t get_data_uri = {
            .uri       = "/get-data",
            .method    = HTTP_GET,
            .handler   = get_data_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &get_data_uri);

        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}


// --- Background Task ---
void i2c_update_task(void *pvParameters) {
     ESP_LOGI(TAG, "Starting I2C Update Task");
     uint16_t temp_buffer; // Local buffer to store read result

     while (1) {
        esp_err_t err = attiny_read_temperature_raw(&temp_buffer);

        if (err == ESP_OK) {
            // Successfully read temperature, update global variable
            current_raw_temp = temp_buffer;
             ESP_LOGI(TAG, "Background read successful. Raw Temp = %u", current_raw_temp);
        } else {
            // Failed to read, keep the last value or set error code
            current_raw_temp = 0xFFFF; // Indicate error state globally
            ESP_LOGE(TAG, "Background read failed.");
        }

        // Wait before the next read
        vTaskDelay(pdMS_TO_TICKS(UPDATE_INTERVAL_MS));
     }
     // Task should not exit, but if it does:
     vTaskDelete(NULL);
}


// --- Main Application Entry Point ---
void app_main(void)
{
    // 1. Initialize NVS (needed for WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Initialize I2C Master
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C Master initialized successfully");

    // 3. Initialize WiFi Station
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    // Check if WiFi connected before starting server & task
    EventBits_t bits = xEventGroupGetBits(wifi_event_group);
    if (bits & WIFI_CONNECTED_BIT) {
        // 4. Start the Web Server
        httpd_handle_t server = start_webserver();
        if (server == NULL) {
             ESP_LOGE(TAG, "Failed to start webserver!");
             // Handle error - maybe reboot or enter error state
             return; // Stop app_main if server fails
        }

        // 5. Start background task for periodic I2C reads
        xTaskCreate(i2c_update_task, "i2c_update_task", 2048, NULL, 5, NULL);

    } else {
        ESP_LOGE(TAG, "WiFi connection failed, cannot start web server or I2C task.");
        // Handle lack of connection (e.g., retry, wait, error LED)
    }

    // app_main should not exit for FreeRTOS apps
    ESP_LOGI(TAG, "Initialization complete. Entering idle loop (tasks running).");
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // Idle delay in main
    }
}