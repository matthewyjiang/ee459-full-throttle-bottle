#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
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
// #include "sdkconfig.h" // No longer needed for the hardcoded defines below

// --- Configuration ---
// Wi-Fi Credentials (Hardcoded)
#define WIFI_SSID      "Xiaolei"
#define WIFI_PASS      "pear8888"
#define MAXIMUM_RETRY  5         // Maximum retry attempts for Wi-Fi connection

// I2C Configuration (Hardcoded - VERIFY THESE PINS MATCH YOUR WIRING!)
#define I2C_MASTER_SCL_IO    22        /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO    21        /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM       I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ   100000    /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0    /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0    /*!< I2C master doesn't need buffer */

// ATTiny85 Configuration
#define ATTINY_SLAVE_ADDR    0x08  /*!< Slave address of the ATTiny85 */
#define ATTINY_CMD_SET_SETPOINT 0x01

#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL        0x0     /*!< I2C ack value */
#define NACK_VAL       0x1     /*!< I2C nack value */

// --- Global Variables ---
static const char *TAG = "THERMOSTAT_WEB";

// Wi-Fi Event Group
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int s_retry_num = 0;

// Shared temperature data
static float g_current_temperature = -99.9;
static int8_t g_current_setpoint = 25;

// ****************************************************************
// ** Declare the server handle globally BEFORE functions using it **
// ****************************************************************
static httpd_handle_t server = NULL;

// --- Function Prototypes ---
// (Prototypes for handlers are often optional if defined before use,
// but good practice to keep them)
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data);
static void wifi_init_sta(void);
static esp_err_t i2c_master_init(void);
static esp_err_t i2c_read_attiny_temp(float *temp_c);
static esp_err_t i2c_write_attiny_setpoint(uint8_t setpoint);
static httpd_handle_t start_webserver(void);
static void stop_webserver(httpd_handle_t server_handle);
static void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data);
static void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data);
// (HTTP URI handlers don't strictly need prototypes if defined before registration)
// static esp_err_t root_get_handler(httpd_req_t *req);
// static esp_err_t data_get_handler(httpd_req_t *req);
// static esp_err_t set_get_handler(httpd_req_t *req);


// --- I2C Functions ---
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO, // Using hardcoded define
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO, // Using hardcoded define
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
     ESP_LOGI(TAG, "I2C Master Initialized (SDA: %d, SCL: %d)", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    return err;
}

static esp_err_t i2c_read_attiny_temp(float *temp_c) {
    uint8_t data_rd[2] = {0};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ATTINY_SLAVE_ADDR << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data_rd[0], ACK_VAL);
    i2c_master_read_byte(cmd, &data_rd[1], NACK_VAL);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100)); // Use pdMS_TO_TICKS for timeout
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        int8_t temp_int = (int8_t)data_rd[0];
        *temp_c = (float)temp_int + ((float)data_rd[1] / 100.0f);
        ESP_LOGD(TAG, "Read Temp Raw: Int=%d, Frac=%d -> Processed: %.2f C", temp_int, data_rd[1], *temp_c);
    } else {
        ESP_LOGE(TAG, "I2C Read Failed: %s", esp_err_to_name(ret));
        *temp_c = -99.9; // Indicate error
    }
    return ret;
}


static esp_err_t i2c_write_attiny_setpoint(uint8_t setpoint) {
     if (setpoint > 100) {
        ESP_LOGW(TAG, "Setpoint %d out of range (0-100), clamping.", setpoint);
        setpoint = 100;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ATTINY_SLAVE_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ATTINY_CMD_SET_SETPOINT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, setpoint, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Wrote Setpoint: %d C", setpoint);
        g_current_setpoint = setpoint; // Update global state
    } else {
        ESP_LOGE(TAG, "I2C Write Failed: %s", esp_err_to_name(ret));
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
        // Call disconnect handler which accesses the global server handle
        disconnect_handler(arg, event_base, event_id, event_data); // arg is NULL here, but disconnect_handler uses global server

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

        // Call connect handler which accesses the global server handle
        connect_handler(arg, event_base, event_id, event_data); // arg is NULL here, but connect_handler uses global server
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

    // Register the event handler. Pass NULL as arg since connect/disconnect handlers
    // will use the global 'server' variable directly.
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL, // Argument passed to event_handler
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL, // Argument passed to event_handler
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
             .pmf_cfg = {
                 .capable = true,
                 .required = false
             },
        },
    };
    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid) -1);
    strncpy((char *)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password) -1);
    wifi_config.sta.ssid[sizeof(wifi_config.sta.ssid) - 1] = '\0';
    wifi_config.sta.password[sizeof(wifi_config.sta.password) - 1] = '\0';


    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    ESP_LOGI(TAG, "Connecting to SSID: %s", WIFI_SSID);

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP SSID: %s", WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to SSID: %s", WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED WIFI EVENT");
    }
}


// --- HTTP Server Handlers ---

// --- HTTP Server Handlers ---

// Root handler serves the HTML page - Sends response in chunks
static esp_err_t root_get_handler(httpd_req_t *req)
{
    // Split the HTML into two parts around the place where the setpoint is inserted
    const char* html_part1 =
        "<!DOCTYPE html><html><head><title>ATTiny Thermostat</title>"
        "<meta http-equiv='refresh' content='5'>"
        "<style>"
        "body { font-family: sans-serif; padding: 20px; background-color: #f4f4f4; }"
        ".container { background-color: #fff; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); max-width: 400px; margin: auto; }"
        "h1 { color: #333; text-align: center; }"
        ".data-item { margin-bottom: 15px; font-size: 1.2em; }"
        ".data-label { font-weight: bold; color: #555; min-width: 180px; display: inline-block;}"
        ".data-value { color: #007bff; }"
        ".form-group { margin-top: 20px; padding-top: 15px; border-top: 1px solid #eee; }"
        "label { display: block; margin-bottom: 5px; font-weight: bold; color: #555;}"
        "input[type='number'] { width: 80px; padding: 8px; border: 1px solid #ccc; border-radius: 4px; margin-right: 10px; }"
        "button { padding: 10px 15px; background-color: #28a745; color: white; border: none; border-radius: 4px; cursor: pointer; }"
        "button:hover { background-color: #218838; }"
        "</style>"
        "<script>"
        "function updateData() {"
        "  fetch('/data')"
        "    .then(response => response.json())"
        "    .then(data => {"
        "      document.getElementById('currentTemp').innerText = data.current.toFixed(2) + ' C';"
        "      document.getElementById('currentSet').innerText = data.setpoint + ' C';"
        "    })"
        "    .catch(error => console.error('Error fetching data:', error));"
        "}"
        "function setTemp() {"
        "  var temp = document.getElementById('setpoint').value;"
        "  fetch('/set?temp=' + temp)"
        "    .then(response => {"
        "       if (response.ok) { console.log('Setpoint set successfully'); updateData(); } " // Update data on success
        "       else { console.error('Failed to set setpoint'); alert('Failed to set setpoint'); } "
        "    })"
        "    .catch(error => { console.error('Error setting temperature:', error); alert('Error sending setpoint'); });"
        "  return false; "
        "}"
        "</script>"
        "</head><body>"
        "<div class='container'>"
        "<h1>Thermostat Control</h1>"
        "<div class='data-item'><span class='data-label'>Current Temperature:</span> <span class='data-value' id='currentTemp'>Loading...</span></div>"
        "<div class='data-item'><span class='data-label'>Current Setpoint:</span> <span class='data-value' id='currentSet'>Loading...</span></div>"
        "<div class='form-group'>"
        "<label for='setpoint'>Set New Temperature (0-100 C):</label>"
        "<input type='number' id='setpoint' name='setpoint' min='0' max='100' value='"; // Split point 1

    const char* html_part2 =
        "'> " // Split point 2
        "<button onclick='setTemp()'>Set</button>"
        "</div>"
        "</div>"
        "<script>document.addEventListener('DOMContentLoaded', updateData);</script>" // Load data once on page load
        "</body></html>";

    // Buffer for the formatted number (integer - max 3 digits + sign + null = ~5 bytes)
    char num_buffer[10];
    snprintf(num_buffer, sizeof(num_buffer), "%d", g_current_setpoint);

    // Set the response content type
    httpd_resp_set_type(req, "text/html");

    // Send the first part of the HTML
    httpd_resp_send_chunk(req, html_part1, HTTPD_RESP_USE_STRLEN);

    // Send the formatted number
    httpd_resp_send_chunk(req, num_buffer, HTTPD_RESP_USE_STRLEN);

    // Send the second part of the HTML
    httpd_resp_send_chunk(req, html_part2, HTTPD_RESP_USE_STRLEN);

    // Send zero-length chunk to finish the response
    httpd_resp_send_chunk(req, NULL, 0);

    return ESP_OK;
}

// Handler for /data URI
static esp_err_t data_get_handler(httpd_req_t *req)
{
    esp_err_t i2c_status = i2c_read_attiny_temp(&g_current_temperature);
    char json_buffer[100];
    snprintf(json_buffer, sizeof(json_buffer),
             "{\"current\": %.2f, \"setpoint\": %d, \"read_ok\": %s}",
             g_current_temperature,
             g_current_setpoint,
             (i2c_status == ESP_OK) ? "true" : "false");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_buffer, strlen(json_buffer));
    return ESP_OK;
}

// Handler for /set URI
static esp_err_t set_get_handler(httpd_req_t *req)
{
    char buf[50];
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
    ESP_LOGD(TAG, "Set Query string: %s", buf); // Debug level

    char param_val[10];
    if (httpd_query_key_value(buf, "temp", param_val, sizeof(param_val)) == ESP_OK) {
        ESP_LOGD(TAG, "Found value for 'temp': %s", param_val); // Debug level
        int new_setpoint = atoi(param_val);
        if (new_setpoint >= 0 && new_setpoint <= 100) {
            i2c_write_attiny_setpoint((uint8_t)new_setpoint);
            // Respond with OK status, client side handles refresh/update
            httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
            // httpd_resp_set_status(req, "302 Found"); // Redirect can cause issues with fetch API
            // httpd_resp_set_hdr(req, "Location", "/");
            // httpd_resp_send(req, NULL, 0);
            return ESP_OK;
        } else {
            ESP_LOGE(TAG, "Invalid temperature value received: %s", param_val);
             httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid temperature value (must be 0-100)");
             return ESP_FAIL;
        }
    } else {
         ESP_LOGE(TAG, "Parameter 'temp' not found in query: %s", buf);
         httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "'temp' parameter missing");
         return ESP_FAIL;
    }
}


// --- HTTP Server Start/Stop ---

// URI Definitions
static const httpd_uri_t uri_root = {
    .uri       = "/", .method = HTTP_GET, .handler = root_get_handler, .user_ctx = NULL };
static const httpd_uri_t uri_data = {
    .uri       = "/data", .method = HTTP_GET, .handler = data_get_handler, .user_ctx = NULL };
static const httpd_uri_t uri_set = {
    .uri       = "/set", .method = HTTP_GET, .handler = set_get_handler, .user_ctx = NULL };

// Start Web Server
static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server_handle = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

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

static void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    // Access the global server handle directly
    if (server) {
        ESP_LOGW(TAG, "Wi-Fi disconnected, stopping webserver");
        stop_webserver(server); // Stop the server using the global handle
        server = NULL;          // Set global handle to NULL
    }
}

static void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    // Access the global server handle directly
    if (server == NULL) { // Check if server is not already running
        ESP_LOGI(TAG, "Wi-Fi connected, starting webserver");
        server = start_webserver(); // Start server and store handle globally
    }
}


// --- Main Application ---
void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize I2C Master
    ESP_ERROR_CHECK(i2c_master_init());

     // Initialize WiFi (This blocks until connected or failed)
    wifi_init_sta();

    // The connect_handler (called via event_handler if connection succeeds)
    // will start the web server using the global 'server' handle.

    ESP_LOGI(TAG, "Initialization sequence complete.");
}