#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "bluetooth_control.h"

#include "esp_bt_device.h"
#include "telescope_protocol.h"

static const char *TAG = "bluetooth_control";

// Bluetooth SPP configuration
#define SPP_SERVER_NAME "ESP_SPP_SERVER"
#define SPP_DEVICE_NAME "Telescope_Controller"
#define SPP_SHOW_DATA 0
#define SPP_SHOW_SPEED 1
#define SPP_SHOW_MODE SPP_SHOW_DATA

// NVS storage keys
#define NVS_NAMESPACE "telescope"
#define NVS_RA_GEAR_KEY "ra_gear"
#define NVS_DEC_GEAR_KEY "dec_gear"

// Default gear ratios (steps per degree)
#define DEFAULT_RA_GEAR_RATIO  3600.0f   // Example: 1:3600 reduction
#define DEFAULT_DEC_GEAR_RATIO 3600.0f   // Example: 1:3600 reduction

// Jog step size
#define JOG_STEP_SIZE 100

// Global state
static uint32_t spp_conn_id = 0;
static bool bluetooth_connected = false;
static gear_config_t current_gear_config;
static coordinate_offsets_t coord_offsets = {0, 0};
static TaskHandle_t bluetooth_task_handle = NULL;

// Function prototypes
static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
static void esp_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
static void send_coordinate_report(void);
static bool parse_float(const char* str, float* value);

/**
 * Initialize Bluetooth control system
 */
esp_err_t bluetooth_control_init(void) {
    ESP_LOGI(TAG, "Initializing Bluetooth control...");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Load gear configuration from NVS
    ret = load_gear_config(&current_gear_config);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load gear config, using defaults");
        current_gear_config.ra_gear_ratio = DEFAULT_RA_GEAR_RATIO;
        current_gear_config.dec_gear_ratio = DEFAULT_DEC_GEAR_RATIO;
        save_gear_config(&current_gear_config);
    }

    ESP_LOGI(TAG, "Gear ratios - RA: %.1f steps/deg, DEC: %.1f steps/deg",
             current_gear_config.ra_gear_ratio, current_gear_config.dec_gear_ratio);

    // Initialize Bluetooth controller
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth controller init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth controller enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    if ((ret = esp_bt_gap_register_callback(esp_gap_cb)) != ESP_OK) {
        ESP_LOGE(TAG, "Gap register failed: %s", esp_err_to_name(ret));
        return ret;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(TAG, "SPP register failed: %s", esp_err_to_name(ret));
        return ret;
    }

    if ((ret = esp_spp_init(ESP_SPP_MODE_CB)) != ESP_OK) {
        ESP_LOGE(TAG, "SPP init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set device name and start discovery
    // esp_bt_dev_set_device_name(SPP_DEVICE_NAME);
    esp_bt_gap_set_device_name(SPP_DEVICE_NAME);
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    // Create Bluetooth task
    BaseType_t task_ret = xTaskCreate(bluetooth_task, "bluetooth_task", 4096, NULL, 5, &bluetooth_task_handle);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Bluetooth task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Bluetooth initialized successfully, device name: %s", SPP_DEVICE_NAME);
    return ESP_OK;
}

/**
 * SPP callback function
 */
static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(TAG, "ESP_SPP_INIT_EVT");
        esp_spp_start_srv(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_SLAVE, 0, SPP_SERVER_NAME);
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_SPP_OPEN_EVT");
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_SPP_CLOSE_EVT");
        bluetooth_connected = false;
        break;
    case ESP_SPP_START_EVT:
        ESP_LOGI(TAG, "ESP_SPP_START_EVT");
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(TAG, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:
        // ESP_LOGI(TAG, "ESP_SPP_DATA_IND_EVT len=%d handle=%l",
        //          param->data_ind.len, param->data_ind.handle);
        if (param->data_ind.len > 0) {
            char *data = malloc(param->data_ind.len + 1);
            memcpy(data, param->data_ind.data, param->data_ind.len);
            data[param->data_ind.len] = '\0';

            // Process the received command
            process_bluetooth_command(data);
            free(data);
        }
        break;
    case ESP_SPP_CONG_EVT:
        ESP_LOGI(TAG, "ESP_SPP_CONG_EVT");
        break;
    case ESP_SPP_WRITE_EVT:
        ESP_LOGI(TAG, "ESP_SPP_WRITE_EVT");
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_SPP_SRV_OPEN_EVT");
        spp_conn_id = param->srv_open.handle;
        bluetooth_connected = true;
        send_bluetooth_response("Telescope Controller Connected\n");
        send_bluetooth_response("Available commands: SET_RA_GEAR <ratio>, SET_DEC_GEAR <ratio>, JOG_RA_POS, JOG_RA_NEG, JOG_DEC_POS, JOG_DEC_NEG, SYNC, STATUS\n");
        break;
    default:
        break;
    }
}

/**
 * GAP callback function
 */
static void esp_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT: {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Authentication success: %s", param->auth_cmpl.device_name);
        } else {
            // ESP_LOGE(TAG, "Authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT: {
        // ESP_LOGI(TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }
    default: {
        // ESP_LOGI(TAG, "GAP event: %d", event);
        break;
    }
    }
    return;
}

/**
 * Parse float from string
 */
static bool parse_float(const char* str, float* value) {
    char* endptr;
    *value = strtof(str, &endptr);
    return endptr != str && *endptr == '\0';
}

/**
 * Process Bluetooth commands
 */
void process_bluetooth_command(const char* command) {
    ESP_LOGI(TAG, "Received command: %s", command);

    char response[256];
    char cmd[64];
    char arg[64];

    // Parse command and argument
    int parsed = sscanf(command, "%63s %63s", cmd, arg);

    if (strcmp(cmd, "SET_RA_GEAR") == 0 && parsed == 2) {
        float ratio;
        if (parse_float(arg, &ratio) && ratio > 0) {
            set_ra_gear_ratio(ratio);
            snprintf(response, sizeof(response), "RA gear ratio set to %.1f steps/degree\n", ratio);
        } else {
            snprintf(response, sizeof(response), "Error: Invalid RA gear ratio\n");
        }
    }
    else if (strcmp(cmd, "SET_DEC_GEAR") == 0 && parsed == 2) {
        float ratio;
        if (parse_float(arg, &ratio) && ratio > 0) {
            set_dec_gear_ratio(ratio);
            snprintf(response, sizeof(response), "DEC gear ratio set to %.1f steps/degree\n", ratio);
        } else {
            snprintf(response, sizeof(response), "Error: Invalid DEC gear ratio\n");
        }
    }
    else if (strcmp(cmd, "JOG_RA_POS") == 0) {
        esp_err_t ret = jog_ra_positive();
        snprintf(response, sizeof(response), "RA jog positive: %s\n",
                ret == ESP_OK ? "OK" : "FAILED");
    }
    else if (strcmp(cmd, "JOG_RA_NEG") == 0) {
        esp_err_t ret = jog_ra_negative();
        snprintf(response, sizeof(response), "RA jog negative: %s\n",
                ret == ESP_OK ? "OK" : "FAILED");
    }
    else if (strcmp(cmd, "JOG_DEC_POS") == 0) {
        esp_err_t ret = jog_dec_positive();
        snprintf(response, sizeof(response), "DEC jog positive: %s\n",
                ret == ESP_OK ? "OK" : "FAILED");
    }
    else if (strcmp(cmd, "JOG_DEC_NEG") == 0) {
        esp_err_t ret = jog_dec_negative();
        snprintf(response, sizeof(response), "DEC jog negative: %s\n",
                ret == ESP_OK ? "OK" : "FAILED");
    }
    else if (strcmp(cmd, "SYNC") == 0) {
        sync_coordinates();
        snprintf(response, sizeof(response), "Coordinates synchronized\n");
    }
    else if (strcmp(cmd, "STATUS") == 0) {
        send_coordinate_report();
        return; // send_coordinate_report handles the response
    }
    else {
        snprintf(response, sizeof(response), "Unknown command: %s\n", cmd);
    }

    send_bluetooth_response(response);
}

/**
 * Send response over Bluetooth
 */
void send_bluetooth_response(const char* response) {
    if (bluetooth_connected && spp_conn_id != 0) {
        esp_spp_write(spp_conn_id, strlen(response), (uint8_t*)response);
    }
}

/**
 * Save gear configuration to NVS
 */
esp_err_t save_gear_config(const gear_config_t* config) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) return err;

    err = nvs_set_blob(nvs_handle, NVS_RA_GEAR_KEY, &config->ra_gear_ratio, sizeof(float));
    if (err == ESP_OK) {
        err = nvs_set_blob(nvs_handle, NVS_DEC_GEAR_KEY, &config->dec_gear_ratio, sizeof(float));
    }
    if (err == ESP_OK) {
        err = nvs_commit(nvs_handle);
    }

    nvs_close(nvs_handle);
    return err;
}

/**
 * Load gear configuration from NVS
 */
esp_err_t load_gear_config(gear_config_t* config) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) return err;

    size_t required_size = sizeof(float);
    err = nvs_get_blob(nvs_handle, NVS_RA_GEAR_KEY, &config->ra_gear_ratio, &required_size);
    if (err == ESP_OK) {
        required_size = sizeof(float);
        err = nvs_get_blob(nvs_handle, NVS_DEC_GEAR_KEY, &config->dec_gear_ratio, &required_size);
    }

    nvs_close(nvs_handle);
    return err;
}

/**
 * Set RA gear ratio
 */
esp_err_t set_ra_gear_ratio(float ratio) {
    current_gear_config.ra_gear_ratio = ratio;
    return save_gear_config(&current_gear_config);
}

/**
 * Set DEC gear ratio
 */
esp_err_t set_dec_gear_ratio(float ratio) {
    current_gear_config.dec_gear_ratio = ratio;
    return save_gear_config(&current_gear_config);
}

/**
 * Get RA gear ratio
 */
float get_ra_gear_ratio(void) {
    return current_gear_config.ra_gear_ratio;
}

/**
 * Get DEC gear ratio
 */
float get_dec_gear_ratio(void) {
    return current_gear_config.dec_gear_ratio;
}

/**
 * Convert steps to RA DMS
 */
void steps_to_ra_dms(int32_t steps, dms_t* dms) {
    // Apply offset
    int32_t adjusted_steps = steps - coord_offsets.ra_offset_steps;

    // Convert steps to degrees (RA is 0-360 degrees)
    float degrees = (float)adjusted_steps / current_gear_config.ra_gear_ratio;

    // Normalize to 0-360 range
    while (degrees < 0) degrees += 360.0f;
    while (degrees >= 360.0f) degrees -= 360.0f;

    // Convert to DMS
    dms->negative = false;
    dms->degrees = (int16_t)degrees;
    float remainder = (degrees - dms->degrees) * 60.0f;
    dms->minutes = (uint8_t)remainder;
    dms->seconds = (uint8_t)((remainder - dms->minutes) * 60.0f);
}

/**
 * Convert steps to DEC DMS
 */
void steps_to_dec_dms(int32_t steps, dms_t* dms) {
    // Apply offset
    int32_t adjusted_steps = steps - coord_offsets.dec_offset_steps;

    // Convert steps to degrees (DEC is -90 to +90 degrees)
    float degrees = (float)adjusted_steps / current_gear_config.dec_gear_ratio;

    // Handle negative values
    dms->negative = degrees < 0;
    if (dms->negative) degrees = -degrees;

    // Convert to DMS
    dms->degrees = (int16_t)degrees;
    float remainder = (degrees - dms->degrees) * 60.0f;
    dms->minutes = (uint8_t)remainder;
    dms->seconds = (uint8_t)((remainder - dms->minutes) * 60.0f);
}

/**
 * Convert RA DMS to steps
 */
int32_t ra_dms_to_steps(const dms_t* dms) {
    float degrees = dms->degrees + (dms->minutes / 60.0f) + (dms->seconds / 3600.0f);
    int32_t steps = (int32_t)(degrees * current_gear_config.ra_gear_ratio);
    return steps + coord_offsets.ra_offset_steps;
}

/**
 * Convert DEC DMS to steps
 */
int32_t dec_dms_to_steps(const dms_t* dms) {
    float degrees = dms->degrees + (dms->minutes / 60.0f) + (dms->seconds / 3600.0f);
    if (dms->negative) degrees = -degrees;
    int32_t steps = (int32_t)(degrees * current_gear_config.dec_gear_ratio);
    return steps + coord_offsets.dec_offset_steps;
}

/**
 * Sync coordinates (zero out RA/DEC by setting offsets)
 */
void sync_coordinates(void) {
    coord_offsets.ra_offset_steps = get_current_ra_position();
    coord_offsets.dec_offset_steps = get_current_dec_position();
    ESP_LOGI(TAG, "Coordinates synced - RA offset: %ld, DEC offset: %ld",
             coord_offsets.ra_offset_steps, coord_offsets.dec_offset_steps);
}

/**
 * Get current RA/DEC coordinates
 */
void get_current_ra_dec(dms_t* ra, dms_t* dec) {
    int32_t ra_steps = get_current_ra_position();
    int32_t dec_steps = get_current_dec_position();

    steps_to_ra_dms(ra_steps, ra);
    steps_to_dec_dms(dec_steps, dec);
}

/**
 * Jog functions
 */
esp_err_t jog_ra_positive(void) {
    status_msg_t status;
    if (!get_latest_status(&status)) {
        return ESP_FAIL;
    }

    int32_t target_pos = status.ra_position + JOG_STEP_SIZE;
    uint64_t target_time = status.ra_update_time + 100000; // 100ms from last update

    return send_pvt_target(0, target_pos, status.ra_velocity, target_time);
}

esp_err_t jog_ra_negative(void) {
    status_msg_t status;
    if (!get_latest_status(&status)) {
        return ESP_FAIL;
    }

    int32_t target_pos = status.ra_position - JOG_STEP_SIZE;
    uint64_t target_time = status.ra_update_time + 100000; // 100ms from last update

    return send_pvt_target(0, target_pos, status.ra_velocity, target_time);
}

esp_err_t jog_dec_positive(void) {
    status_msg_t status;
    if (!get_latest_status(&status)) {
        return ESP_FAIL;
    }

    int32_t target_pos = status.dec_position + JOG_STEP_SIZE;
    uint64_t target_time = status.dec_update_time + 100000; // 100ms from last update

    return send_pvt_target(1, target_pos, status.dec_velocity, target_time);
}

esp_err_t jog_dec_negative(void) {
    status_msg_t status;
    if (!get_latest_status(&status)) {
        return ESP_FAIL;
    }

    int32_t target_pos = status.dec_position - JOG_STEP_SIZE;
    uint64_t target_time = status.dec_update_time + 100000; // 100ms from last update

    return send_pvt_target(1, target_pos, status.dec_velocity, target_time);
}

/**
 * Send coordinate report over Bluetooth
 */
static void send_coordinate_report(void) {
    char response[512];
    dms_t ra, dec;
    status_msg_t status;

    if (!get_latest_status(&status)) {
        send_bluetooth_response("No telescope status available\n");
        return;
    }

    get_current_ra_dec(&ra, &dec);

    snprintf(response, sizeof(response),
        "=== TELESCOPE STATUS ===\n"
        "RA:  %02d°%02d'%02d\" (steps: %ld)\n"
        "DEC: %s%02d°%02d'%02d\" (steps: %ld)\n"
        "Gear ratios: RA=%.1f, DEC=%.1f steps/deg\n"
        "Status flags: 0x%02X\n"
        "========================\n",
        ra.degrees, ra.minutes, ra.seconds, status.ra_position,
        dec.negative ? "-" : "+", dec.degrees, dec.minutes, dec.seconds, status.dec_position,
        current_gear_config.ra_gear_ratio, current_gear_config.dec_gear_ratio,
        status.status_flags);

    send_bluetooth_response(response);
}

/**
 * Bluetooth task - sends periodic coordinate reports
 */
void bluetooth_task(void *pvParameters) {
    ESP_LOGI(TAG, "Bluetooth task started");

    TickType_t last_report = 0;
    const TickType_t report_interval = pdMS_TO_TICKS(5000); // Report every 5 seconds

    while (1) {
        TickType_t now = xTaskGetTickCount();

        // Send periodic coordinate reports when connected
        if (bluetooth_connected && (now - last_report >= report_interval)) {
            send_coordinate_report();
            last_report = now;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}