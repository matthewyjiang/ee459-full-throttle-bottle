#include <stdio.h>
#include <stdlib.h> // For atoi
#include <string.h>
// No longer need <vector>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h" // Include for mutexes
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "driver/i2c.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "cJSON.h" // Include for JSON generation

// --- Configuration ---
// Wi-Fi Credentials (Hardcoded)
#define WIFI_SSID      "Xiaolei" // <-- REPLACE WITH YOUR WIFI SSID
#define WIFI_PASS      "pear8888" // <-- REPLACE WITH YOUR WIFI PASSWORD
#define MAXIMUM_RETRY  5         // Maximum retry attempts for Wi-Fi connection

// I2C Configuration
#define I2C_MASTER_SCL_IO    22        /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO    21        /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM       I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ   50000    /*!< I2C master clock frequency (50kHz) */
#define I2C_MASTER_TX_BUF_DISABLE 0    /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0    /*!< I2C master doesn't need buffer */
#define I2C_TIMEOUT_MS       100       /*!< Timeout for I2C operations */

// ATTiny85 Configuration
#define ATTINY_CMD_SET_SETPOINT 0x01   /*!< Command byte to set setpoint */
#define ATTINY_BASE_ADDR        0x08   /*!< Starting address to scan for slaves */
#define ATTINY_ADDR_SCAN_RANGE  8      /*!< How many addresses to scan (e.g., 8 scans 0x08 to 0x0F) */
#define MAX_HEATERS             ATTINY_ADDR_SCAN_RANGE /*!< Max heaters matches scan range */

#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL        0x0     /*!< I2C ack value */
#define NACK_VAL       0x1     /*!< I2C nack value */

#define TEMP_READ_INTERVAL_MS   1000     // Read temperatures every 1000ms

// --- Data Structure for Heater Info ---
typedef struct {
    uint8_t address;        // I2C address of the heater
    float current_temp;   // Last read temperature (Celsius)
    int8_t setpoint;      // Current target temperature (Celsius)
    bool read_ok;         // Flag indicating if the last read was successful
} HeaterInfo;

// --- Global Variables ---
static const char *TAG = "MULTI_THERMOSTAT_C";

// Wi-Fi Event Group
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int s_retry_num = 0;

// Heater Management
static HeaterInfo g_discovered_heaters[MAX_HEATERS]; // Fixed-size array for discovered heaters
static int g_num_heaters = 0;                        // Counter for number of discovered heaters
static SemaphoreHandle_t g_heater_list_mutex = NULL; // Mutex to protect access to the array/counter

// HTTP Server Handle
static httpd_handle_t server = NULL;

// --- Function Prototypes ---
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void wifi_init_sta(void);
static esp_err_t i2c_master_init(void);
static esp_err_t scan_i2c_bus(void);
static esp_err_t i2c_read_attiny_temp(uint8_t slave_addr, float *temp_c);
static esp_err_t i2c_write_attiny_setpoint(uint8_t slave_addr, uint8_t setpoint);
static void temp_reader_task(void *pvParameters);
static httpd_handle_t start_webserver(void);
static void stop_webserver(httpd_handle_t server_handle);
static void disconnect_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void connect_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static esp_err_t root_get_handler(httpd_req_t *req);
static esp_err_t data_get_handler(httpd_req_t *req);
static esp_err_t set_get_handler(httpd_req_t *req);
// No extern "C" needed for app_main in pure C project
void app_main(void);


// --- I2C Functions ---

/**
 * @brief Initialize I2C master interface.
 */
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0, // Optional flags
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
     ESP_LOGI(TAG, "I2C Master Initialized (SDA: %d, SCL: %d)", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    return err;
}

/**
 * @brief Scan the I2C bus for ATTiny slaves within the defined range.
 * Populates the g_discovered_heaters array and updates g_num_heaters.
 * Assumes g_heater_list_mutex is available.
 */
static esp_err_t scan_i2c_bus() {
    if (xSemaphoreTake(g_heater_list_mutex, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take heater list mutex for scanning");
        return ESP_FAIL;
    }

    g_num_heaters = 0; // Reset count of discovered heaters
    ESP_LOGI(TAG, "Scanning I2C bus for heaters from 0x%02X to 0x%02X...", ATTINY_BASE_ADDR, ATTINY_BASE_ADDR + ATTINY_ADDR_SCAN_RANGE - 1);

    for (uint8_t i = 0; i < ATTINY_ADDR_SCAN_RANGE; ++i) {
       uint8_t current_addr = ATTINY_BASE_ADDR + i;
       if (current_addr > 0x77) break; // Stay within valid 7-bit address range

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        // Try a simple write operation just to check for an ACK
        i2c_master_write_byte(cmd, (current_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
        i2c_master_stop(cmd);

        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50)); // Short timeout for scan
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Found heater at address: 0x%02X", current_addr);
            // Add to array if space available
            if (g_num_heaters < MAX_HEATERS) {
                 HeaterInfo new_heater = {
                    .address = current_addr,
                    .current_temp = -99.9f, // Initial invalid temp
                    .setpoint = 20,        // Default setpoint
                    .read_ok = false       // Initially not read successfully
                 };
                 g_discovered_heaters[g_num_heaters] = new_heater; // Add to array
                 g_num_heaters++; // Increment count
            } else {
                 ESP_LOGW(TAG, "Reached maximum number of heaters (%d). Cannot add 0x%02X.", MAX_HEATERS, current_addr);
                 break; // Stop scanning if array is full
            }
        } else if (ret == ESP_ERR_TIMEOUT || ret == ESP_FAIL) { // ESP_FAIL can occur if NACK received
            // No device responded at this address (normal)
        } else {
            ESP_LOGW(TAG, "Error checking address 0x%02X: %s", current_addr, esp_err_to_name(ret));
        }
        // Small delay between checks if needed, but usually not necessary at 50kHz
        // vTaskDelay(pdMS_TO_TICKS(5));
    }
    ESP_LOGI(TAG, "Scan complete. Found %d heaters.", g_num_heaters);

    xSemaphoreGive(g_heater_list_mutex);
    return ESP_OK;
}


/**
 * @brief Read temperature from a specific ATTiny slave.
 * @param slave_addr The I2C address of the target slave.
 * @param temp_c Pointer to store the read temperature.
 * @return ESP_OK on success, ESP_FAIL or other error code on failure.
 */
static esp_err_t i2c_read_attiny_temp(uint8_t slave_addr, float *temp_c) {
    uint8_t data_rd[2] = {0}; // Buffer for integer and fractional parts
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // Send address byte with READ bit set
    i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    // Read first byte (integer part), send ACK
    i2c_master_read_byte(cmd, &data_rd[0], ACK_VAL);
    // Read second byte (fractional part), send NACK
    i2c_master_read_byte(cmd, &data_rd[1], NACK_VAL);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        int8_t temp_int = (int8_t)data_rd[0]; // Cast first byte to signed int
        // Combine integer and fractional parts (fraction is 0-99)
        *temp_c = (float)temp_int + ((float)data_rd[1] / 100.0f);
        ESP_LOGD(TAG, "Read Temp [0x%02X]: Raw Int=%d, Frac=%d -> Processed: %.2f C", slave_addr, temp_int, data_rd[1], *temp_c);
    } else {
        ESP_LOGE(TAG, "I2C Read Failed [0x%02X]: %s", slave_addr, esp_err_to_name(ret));
        *temp_c = -99.9; // Indicate error
    }
    return ret;
}

/**
 * @brief Write setpoint temperature to a specific ATTiny slave.
 * @param slave_addr The I2C address of the target slave.
 * @param setpoint The target temperature (0-100).
 * @return ESP_OK on success, ESP_FAIL or other error code on failure.
 */
static esp_err_t i2c_write_attiny_setpoint(uint8_t slave_addr, uint8_t setpoint) {
     if (setpoint > 100) {
        ESP_LOGW(TAG, "Setpoint %d for 0x%02X out of range (0-100), clamping.", setpoint, slave_addr);
        setpoint = 100;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // Send address byte with WRITE bit set
    i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    // Send the command byte
    i2c_master_write_byte(cmd, ATTINY_CMD_SET_SETPOINT, ACK_CHECK_EN);
    // Send the setpoint value byte
    i2c_master_write_byte(cmd, setpoint, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Wrote Setpoint to [0x%02X]: %d C", slave_addr, setpoint);
        // Update local state in the main structure (requires mutex)
        if (xSemaphoreTake(g_heater_list_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            // Iterate through the array of discovered heaters
            for (int i = 0; i < g_num_heaters; i++) {
                if (g_discovered_heaters[i].address == slave_addr) {
                    g_discovered_heaters[i].setpoint = setpoint;
                    break; // Found the heater, no need to continue loop
                }
            }
            xSemaphoreGive(g_heater_list_mutex);
        } else {
             ESP_LOGW(TAG, "Could not take mutex to update local setpoint for 0x%02X", slave_addr);
        }
    } else {
        ESP_LOGE(TAG, "I2C Write Failed [0x%02X]: %s", slave_addr, esp_err_to_name(ret));
    }
    return ret;
}


// --- Wi-Fi Event Handler ---
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        disconnect_handler(arg, event_base, event_id, event_data);
        if (s_retry_num < MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry connecting to the AP (%d/%d)", s_retry_num, MAXIMUM_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG,"Connect to the AP failed after %d attempts", MAXIMUM_RETRY);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        connect_handler(arg, event_base, event_id, event_data);
    }
}

// --- Wi-Fi Initialization ---
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    // Use wifi_config_t structure for Wi-Fi settings
    wifi_config_t wifi_config = {
        .sta = {
            // .ssid = WIFI_SSID, // Direct assignment deprecated for arrays
            // .password = WIFI_PASS, // Direct assignment deprecated for arrays
            .threshold.authmode = WIFI_AUTH_WPA2_PSK, // Common setting
            .pmf_cfg = {
                .capable = true,
                .required = false,
            },
        },
    };
    // Safely copy SSID and password
    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password) - 1);
    // Ensure null termination
    wifi_config.sta.ssid[sizeof(wifi_config.sta.ssid) - 1] = '\0';
    wifi_config.sta.password[sizeof(wifi_config.sta.password) - 1] = '\0';


    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    ESP_LOGI(TAG, "Connecting to SSID: %s", WIFI_SSID);

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP SSID: %s", WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to SSID: %s", WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED WIFI EVENT");
    }
}

// --- Background Task to Read Temperatures ---

/**
 * @brief Task that periodically reads temperature from all discovered heaters.
 */
static void temp_reader_task(void *pvParameters) {
    float temp_reading;
    esp_err_t result;

    while(1) {
        // Take mutex to safely iterate through the heater list
        if (xSemaphoreTake(g_heater_list_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {

            // Iterate through the array of discovered heaters
            for (int i = 0; i < g_num_heaters; i++) {
                 // Read current temperature for this specific heater
                result = i2c_read_attiny_temp(g_discovered_heaters[i].address, &temp_reading);

                // Update the heater's info in the shared structure
                g_discovered_heaters[i].read_ok = (result == ESP_OK);
                if (g_discovered_heaters[i].read_ok) {
                    g_discovered_heaters[i].current_temp = temp_reading;
                } else {
                    g_discovered_heaters[i].current_temp = -99.9f; // Mark as invalid on error
                    // Optionally add retry logic here
                }
                 // Small delay between reads to avoid overwhelming the bus/slaves
                 vTaskDelay(pdMS_TO_TICKS(20));
            } // end for loop

            // Release mutex
            xSemaphoreGive(g_heater_list_mutex);

        } else {
            ESP_LOGW(TAG, "TempReader Task: Could not take heater list mutex");
        }

        // Wait before next round of readings
        vTaskDelay(pdMS_TO_TICKS(TEMP_READ_INTERVAL_MS));
    }
}


// --- HTTP Server Handlers ---

/**
 * @brief Handler for the root ("/") URL. Generates dynamic HTML.
 */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    // Buffer for HTML chunks. Use for dynamic parts like heater cards.
    char chunk_buf[1024];
    esp_err_t res = ESP_OK;

    // --- Send HTML Head in Chunks ---
    httpd_resp_set_type(req, "text/html");

    // Chunk 1: Doctype, Head Start, Title, Viewport
    const char *html_part1 = "<!DOCTYPE html><html><head><title>Multi-Heater Control (C)</title>"
                             "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    res = httpd_resp_send_chunk(req, html_part1, strlen(html_part1));
    if (res != ESP_OK) { return res; }

    // --- Chunk 2: CSS (Split into multiple parts) ---
    const char *css_part1 =
        "<style>"
        "body { font-family: sans-serif; padding: 15px; background-color: #f8f9fa; margin: 0; }"
        ".container { max-width: 800px; margin: 20px auto; background-color: #fff; padding: 25px; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.1); }"
        "h1 { color: #343a40; text-align: center; margin-bottom: 30px; }"
        ".heater-card { background-color: #f1f3f5; border: 1px solid #dee2e6; border-radius: 6px; padding: 20px; margin-bottom: 25px; }"
        ".heater-title { font-size: 1.4em; font-weight: bold; color: #495057; margin-bottom: 15px; border-bottom: 1px solid #ced4da; padding-bottom: 10px; }";
    res = httpd_resp_send_chunk(req, css_part1, strlen(css_part1));
    if (res != ESP_OK) { return res; }

    const char *css_part2 =
        ".data-item { margin-bottom: 12px; font-size: 1.1em; display: flex; justify-content: space-between; align-items: center; }"
        ".data-label { font-weight: bold; color: #6c757d; min-width: 150px; }"
        ".data-value { color: #007bff; font-weight: bold; }"
        ".data-value.error { color: #dc3545; }"
        ".form-group { margin-top: 20px; padding-top: 15px; border-top: 1px solid #e9ecef; text-align: center; }"
        ".form-group label { display: block; margin-bottom: 12px; font-weight: bold; color: #555; }";
     res = httpd_resp_send_chunk(req, css_part2, strlen(css_part2));
    if (res != ESP_OK) { return res; }

    const char *css_part3 =
        ".button-group { display: flex; flex-wrap: wrap; justify-content: center; gap: 10px; }"
        "button { padding: 10px 15px; color: white; border: none; border-radius: 5px; cursor: pointer; font-size: 1em; transition: background-color 0.2s ease; min-width: 100px; }"
        "button:hover { opacity: 0.85; }"
        ".btn-20 { background-color: #6c757d; } /* Grey */"
        ".btn-30 { background-color: #17a2b8; } /* Teal */"
        ".btn-40 { background-color: #ffc107; } /* Yellow */"
        ".btn-60 { background-color: #fd7e14; } /* Orange */"
        ".btn-80 { background-color: #dc3545; } /* Red */"
        "#status { margin-top: 20px; text-align: center; font-style: italic; color: #6c757d; }"
        "</style>";
    res = httpd_resp_send_chunk(req, css_part3, strlen(css_part3));
    if (res != ESP_OK) { return res; }

    // --- Chunk 3: JavaScript (Split into multiple parts) ---
    const char *js_part1 =
        "<script>"
        "function updateData() {"
        "  fetch('/data')"
        "    .then(response => response.json())"
        "    .then(data => {"
        "      if (data && data.heaters) {"
        "        data.heaters.forEach(heater => {"
        "          const addrHex = '0x' + heater.address.toString(16).toUpperCase().padStart(2, '0');"
        "          const tempElement = document.getElementById('currentTemp_' + addrHex);"
        "          const setElement = document.getElementById('currentSet_' + addrHex);"
        "          if (tempElement) {"
        "             if (heater.read_ok) { "
        "               tempElement.innerText = heater.current.toFixed(2) + ' C';"
        "               tempElement.classList.remove('error'); "
        "             } else { "
        "               tempElement.innerText = 'Read Error';"
        "               tempElement.classList.add('error'); "
        "             }"
        "          }";
     res = httpd_resp_send_chunk(req, js_part1, strlen(js_part1));
    if (res != ESP_OK) { return res; }

     const char *js_part2 =
        "          if (setElement) {"
        "             setElement.innerText = heater.setpoint + ' C';"
        "          }"
        "        });"
        "        document.getElementById('status').innerText = 'Data updated: ' + new Date().toLocaleTimeString();"
        "      } else {"
        "        document.getElementById('status').innerText = 'Error fetching data or no heaters found.';"
        "      }"
        "    })"
        "    .catch(error => {"
        "       console.error('Error fetching data:', error);"
        "       document.getElementById('status').innerText = 'Fetch error: ' + error;"
        "     });"
        "}";
    res = httpd_resp_send_chunk(req, js_part2, strlen(js_part2));
    if (res != ESP_OK) { return res; }

    const char *js_part3 =
        "function setTemp(address, temp) {"
        "  const addrHex = '0x' + address.toString(16).toUpperCase().padStart(2, '0');"
        "  document.getElementById('status').innerText = 'Setting ' + addrHex + ' to ' + temp + ' C...';"
        "  fetch(`/set?address=${address}&temp=${temp}`)"
        "    .then(response => {"
        "       if (response.ok) { "
        "         console.log('Setpoint set successfully for ' + addrHex + ' to ' + temp + ' C'); "
        "         document.getElementById('status').innerText = 'Setpoint for ' + addrHex + ' set to ' + temp + ' C.';"
        "         setTimeout(updateData, 500);"
        "       } else { "
        "         console.error('Failed to set setpoint for ' + addrHex);"
        "         response.text().then(text => { document.getElementById('status').innerText = 'Failed to set ' + addrHex + ': ' + text; });"
        "       } "
        "    })"
        "    .catch(error => { "
        "      console.error('Error setting temperature for ' + addrHex + ':', error); "
        "      document.getElementById('status').innerText = 'Error sending setpoint for ' + addrHex + ': ' + error;"
        "    });"
        "  return false; "
        "}";
    res = httpd_resp_send_chunk(req, js_part3, strlen(js_part3));
    if (res != ESP_OK) { return res; }

    const char *js_part4 =
        "document.addEventListener('DOMContentLoaded', () => {"
        "  updateData();"
        "  setInterval(updateData, 5000);"
        "});"
        "</script>"
        "</head>"; // Close head tag
    res = httpd_resp_send_chunk(req, js_part4, strlen(js_part4));
    if (res != ESP_OK) { return res; }


    // Chunk 4: Body Start, Container, Heading
    const char *html_part4 = "<body><div class='container'>"
                             "<h1>Multi-Heater Control Panel (C Version)</h1>";
    res = httpd_resp_send_chunk(req, html_part4, strlen(html_part4));
    if (res != ESP_OK) { return res; }


    // --- Generate Heater Cards (Chunk per card or group if needed) ---
    if (xSemaphoreTake(g_heater_list_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        if (g_num_heaters == 0) {
             const char *no_heaters_msg = "<p style='text-align: center; color: #6c757d;'>No heaters detected. Please check connections and scan range.</p>";
             res = httpd_resp_send_chunk(req, no_heaters_msg, strlen(no_heaters_msg));
             if (res != ESP_OK) { goto cleanup; } // Use goto for cleanup on error
        } else {
            for (int i = 0; i < g_num_heaters; i++) {
                HeaterInfo heater = g_discovered_heaters[i];
                char addr_hex_str[5]; // Still need hex string for element IDs
                snprintf(addr_hex_str, sizeof(addr_hex_str), "0x%02X", heater.address);

                // *** MODIFICATION: Use loop index 'i + 1' for the title ***
                int n = snprintf(chunk_buf, sizeof(chunk_buf),
                    "<div class='heater-card'>"
                    // Use "Bottle %d" and pass 'i + 1' as the argument
                    "<div class='heater-title'>Bottle %d</div>"
                    "<div class='data-item'><span class='data-label'>Current Temp:</span> <span class='data-value %s' id='currentTemp_%s'>%.2f C</span></div>"
                    "<div class='data-item'><span class='data-label'>Setpoint:</span> <span class='data-value' id='currentSet_%s'>%d C</span></div>"
                    "<div class='form-group'>"
                    "<label>Set Temperature:</label>"
                    "<div class='button-group'>"
                    "<button class='btn-20' onclick='setTemp(%d, 20)'>20 C</button>"
                    "<button class='btn-30' onclick='setTemp(%d, 30)'>30 C</button>"
                    "<button class='btn-40' onclick='setTemp(%d, 40)'>40 C</button>"
                    "<button class='btn-60' onclick='setTemp(%d, 60)'>60 C</button>"
                    "<button class='btn-80' onclick='setTemp(%d, 80)'>80 C</button>"
                    "</div></div></div>",
                    i + 1, // Display "Bottle 1", "Bottle 2", etc.
                    heater.read_ok ? "" : "error", addr_hex_str, heater.read_ok ? heater.current_temp : -99.9, // Keep using addr_hex_str for IDs
                    addr_hex_str, heater.setpoint, // Keep using addr_hex_str for IDs
                    heater.address, heater.address, heater.address, heater.address, heater.address // Keep passing actual address to JS
                );
                // *** END MODIFICATION ***

                // Check for snprintf truncation (optional but good practice)
                if (n >= sizeof(chunk_buf)) {
                    ESP_LOGW(TAG, "Heater card HTML truncated for Bottle %d (Addr: %s)", i + 1, addr_hex_str);
                }

                res = httpd_resp_send_chunk(req, chunk_buf, strlen(chunk_buf));
                if (res != ESP_OK) { goto cleanup; }
            }
        }
    cleanup: // Label for mutex release
        xSemaphoreGive(g_heater_list_mutex);
        if (res != ESP_OK) { return res; } // Return error if occurred inside mutex lock

    } else {
        ESP_LOGW(TAG, "Root Handler: Could not take heater list mutex");
        const char *err_msg = "<p>Error accessing heater data.</p>";
        httpd_resp_send_chunk(req, err_msg, strlen(err_msg)); // Ignore result, will finish anyway
    }


    // --- Send Final HTML Chunks ---
    const char *html_part_final = "<div id='status'>Loading initial data...</div></div></body></html>";
    res = httpd_resp_send_chunk(req, html_part_final, strlen(html_part_final));
    if (res != ESP_OK) { return res; }

    // Finish response by sending a zero-length chunk
    res = httpd_resp_send_chunk(req, NULL, 0);
    return res;
}


/**
 * @brief Handler for the /data URI. Returns JSON array of heater statuses.
 */
static esp_err_t data_get_handler(httpd_req_t *req)
{
    cJSON *root = cJSON_CreateObject();
    if (!root) {
        ESP_LOGE(TAG, "Failed to create cJSON root object.");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    cJSON *heaters_array = cJSON_CreateArray();
    if (!heaters_array) {
        ESP_LOGE(TAG, "Failed to create cJSON heaters array.");
        cJSON_Delete(root);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    cJSON_AddItemToObject(root, "heaters", heaters_array);

    // Take mutex to safely read the heater list
    if (xSemaphoreTake(g_heater_list_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Iterate through the array of discovered heaters
        for (int i = 0; i < g_num_heaters; i++) {
            HeaterInfo heater = g_discovered_heaters[i];
            cJSON *heater_obj = cJSON_CreateObject();
            if (!heater_obj) {
                ESP_LOGW(TAG, "Failed to create cJSON object for heater %d", i);
                continue; // Skip this heater if allocation fails
            }
            cJSON_AddNumberToObject(heater_obj, "address", heater.address);
            cJSON_AddNumberToObject(heater_obj, "current", heater.current_temp);
            cJSON_AddNumberToObject(heater_obj, "setpoint", heater.setpoint);
            cJSON_AddBoolToObject(heater_obj, "read_ok", heater.read_ok);
            cJSON_AddItemToArray(heaters_array, heater_obj);
        }
        xSemaphoreGive(g_heater_list_mutex);
    } else {
        ESP_LOGW(TAG, "Data Handler: Could not take heater list mutex");
        // Optionally add an error field to the JSON
        cJSON_AddStringToObject(root, "error", "Could not access heater data");
    }

    char *json_string = cJSON_PrintUnformatted(root); // Use Unformatted for smaller size
    cJSON_Delete(root); // Free cJSON object memory

    if (json_string == NULL) {
        ESP_LOGE(TAG, "Failed to print cJSON to string.");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));

    free(json_string); // Free the string memory allocated by cJSON_Print

    return ESP_OK;
}

/**
 * @brief Handler for the /set URI. Sets the setpoint for a specific heater.
 * Expects query parameters: address=<int>&temp=<int>
 */
static esp_err_t set_get_handler(httpd_req_t *req)
{
    char buf[100]; // Increased buffer size
    int ret, remaining = httpd_req_get_url_query_len(req) + 1;

    if (remaining > sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Query string too long");
        return ESP_FAIL;
    }
    ret = httpd_req_get_url_query_str(req, buf, sizeof(buf));
    if (ret != ESP_OK) {
         httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to get query string");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Set Query string: %s", buf);

    char addr_str[10];
    char temp_str[10];
    int target_address = -1;
    int new_setpoint = -1;

    // Parse 'address' parameter
    if (httpd_query_key_value(buf, "address", addr_str, sizeof(addr_str)) == ESP_OK) {
        target_address = atoi(addr_str); // Use atoi from stdlib.h
        ESP_LOGD(TAG, "Parsed address: %d", target_address);
    } else {
        ESP_LOGE(TAG, "Parameter 'address' not found in query: %s", buf);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "'address' parameter missing");
        return ESP_FAIL;
    }

    // Parse 'temp' parameter
    if (httpd_query_key_value(buf, "temp", temp_str, sizeof(temp_str)) == ESP_OK) {
        new_setpoint = atoi(temp_str); // Use atoi from stdlib.h
         ESP_LOGD(TAG, "Parsed temp: %d", new_setpoint);
    } else {
         ESP_LOGE(TAG, "Parameter 'temp' not found in query: %s", buf);
         httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "'temp' parameter missing");
         return ESP_FAIL;
    }

    // Validate parsed values
    if (target_address < 0 || target_address > 127) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid 'address' value");
        return ESP_FAIL;
    }
     if (new_setpoint < 0 || new_setpoint > 100) { // Assuming 0-100 is valid range
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid 'temp' value (must be 0-100)");
        return ESP_FAIL;
    }

    // --- Check if the target address exists in our discovered list ---
    bool address_found = false;
    if (xSemaphoreTake(g_heater_list_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Iterate through the array of discovered heaters
        for (int i = 0; i < g_num_heaters; i++) {
            if (g_discovered_heaters[i].address == (uint8_t)target_address) {
                address_found = true;
                break; // Found it
            }
        }
        xSemaphoreGive(g_heater_list_mutex);
    } else {
         ESP_LOGW(TAG, "Set Handler: Could not take mutex to verify address");
         httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Server error checking address");
         return ESP_FAIL;
    }

    if (!address_found) {
         ESP_LOGE(TAG, "Target address 0x%02X not found in discovered list", target_address);
         httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Heater address not found");
         return ESP_FAIL;
    }
    // --- End address check ---


    // Attempt to write the setpoint via I2C
    esp_err_t write_result = i2c_write_attiny_setpoint((uint8_t)target_address, (uint8_t)new_setpoint);

    if (write_result == ESP_OK) {
        // Respond with OK status (client-side JS handles UI update)
        httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to write setpoint %d to address 0x%02X", new_setpoint, target_address);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to communicate with heater");
        return ESP_FAIL;
    }
}


// --- HTTP Server Start/Stop ---

// URI Definitions
static const httpd_uri_t uri_root = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler, .user_ctx = NULL };
static const httpd_uri_t uri_data = { .uri = "/data", .method = HTTP_GET, .handler = data_get_handler, .user_ctx = NULL };
static const httpd_uri_t uri_set = { .uri = "/set", .method = HTTP_GET, .handler = set_get_handler, .user_ctx = NULL };

// Start Web Server
static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server_handle = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true; // Enable LRU purge for idle connections
    config.stack_size = 8192; // Increase stack size if needed for complex handlers

    ESP_LOGI(TAG, "Starting HTTP server on port: '%d'", config.server_port);
    if (httpd_start(&server_handle, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server_handle, &uri_root);
        httpd_register_uri_handler(server_handle, &uri_data);
        httpd_register_uri_handler(server_handle, &uri_set);
        return server_handle;
    }
    ESP_LOGE(TAG, "Error starting HTTP server!");
    return NULL;
}

// Stop Web Server
static void stop_webserver(httpd_handle_t server_handle)
{
    if (server_handle) {
        httpd_stop(server_handle);
         ESP_LOGI(TAG, "HTTP server stopped");
    }
}

// --- Connection Handlers (Using Global Server Handle) ---
static void disconnect_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (server) {
        ESP_LOGW(TAG, "Wi-Fi disconnected, stopping webserver");
        stop_webserver(server);
        server = NULL;
    }
}

static void connect_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (server == NULL) {
        ESP_LOGI(TAG, "Wi-Fi connected, starting webserver");
        server = start_webserver();
    }
}


// --- Main Application ---
void app_main(void)
{
    // Initialize NVS (Needed for Wi-Fi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Create Mutex for heater list protection BEFORE initializing I2C/tasks
    g_heater_list_mutex = xSemaphoreCreateMutex();
    if (g_heater_list_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create heater list mutex! Halting.");
        // Consider adding a reboot or other error handling here
        while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); } // Halt execution
    }

    // Initialize I2C Master
    ESP_ERROR_CHECK(i2c_master_init());

    // Scan I2C bus to find heaters
    ESP_ERROR_CHECK(scan_i2c_bus()); // Populates g_discovered_heaters and sets g_num_heaters

    // Start background temperature reader task only if heaters were found
    if (g_num_heaters > 0) {
        // Increased stack size slightly due to potential logging and loop complexity
        xTaskCreate(temp_reader_task, "temp_reader", 3072, NULL, 5, NULL);
    } else {
        ESP_LOGW(TAG, "No heaters found during scan. Temperature reader task not started.");
    }


    // Initialize WiFi (This blocks until connected or failed)
    wifi_init_sta();

    // The connect_handler (called via event_handler if connection succeeds)
    // will start the web server using the global 'server' handle.
    // If Wi-Fi fails to connect, the server won't start.

    ESP_LOGI(TAG, "Initialization sequence complete. %d heaters found initially.", g_num_heaters);
}
