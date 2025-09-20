#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_gatt_common_api.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "bluetooth_control.h"

#include "esp_bt_device.h"
#include "telescope_protocol.h"

static const char *TAG = "bluetooth_control";

// BLE configuration
#define DEVICE_NAME "Telescope_Controller"

// Custom Service UUID: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E (Nordic UART Service compatible)
#define TELESCOPE_SERVICE_UUID_128 {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E}

// Characteristic UUIDs (16-bit for simplicity)
#define CHAR_RA_GEAR_RATIO_UUID       0xFF01  // RA gear ratio (read/write float)
#define CHAR_DEC_GEAR_RATIO_UUID      0xFF02  // DEC gear ratio (read/write float)
#define CHAR_JOG_CONTROL_UUID         0xFF03  // Jog control (write byte: 1=RA+, 2=RA-, 3=DEC+, 4=DEC-)
#define CHAR_STATUS_UUID              0xFF04  // Status info (read/notify)
#define CHAR_SYNC_UUID                0xFF05  // Sync command (write byte: 1=sync)
#define CHAR_TRACKING_MODE_UUID       0xFF06  // Tracking mode (read/write byte: 0=off, 1=sidereal, 2=solar, 3=lunar)
#define CHAR_STEP_TARGET_UUID         0xFF07  // Step target position (write: axis byte + int32 position)

// GATT server configuration
#define GATTS_NUM_HANDLE             20
#define GATTS_DEMO_CHAR_VAL_LEN_MAX  64

// NVS storage keys
#define NVS_NAMESPACE "telescope"
#define NVS_RA_GEAR_KEY "ra_gear"
#define NVS_DEC_GEAR_KEY "dec_gear"

// Default gear ratios (steps per full rotation)
#define DEFAULT_RA_GEAR_RATIO  1296000.0f  // 3600 steps/deg * 360 deg = 1,296,000 steps/rotation
#define DEFAULT_DEC_GEAR_RATIO 1296000.0f  // 3600 steps/deg * 360 deg = 1,296,000 steps/rotation

// Jog step size
#define JOG_STEP_SIZE 100

// Tracking rates (steps per second)
#define SIDEREAL_RATE_ARCSEC_PER_SEC  15.041067  // 15.041067 arcseconds per second
#define SOLAR_RATE_ARCSEC_PER_SEC     15.000000  // 15.000000 arcseconds per second (slightly slower)
#define LUNAR_RATE_ARCSEC_PER_SEC     14.492000  // 14.492000 arcseconds per second (much slower)

// Global state
static bool bluetooth_connected = false;
static gear_config_t current_gear_config;
static coordinate_offsets_t coord_offsets = {0, 0};
static TaskHandle_t bluetooth_task_handle = NULL;
static tracking_mode_t current_tracking_mode = TRACKING_OFF;
static uint64_t tracking_start_time = 0;  // Time when tracking started (microseconds)
static int32_t tracking_start_position = 0;  // RA position when tracking started

// BLE GATT server state
static uint16_t gatts_if = ESP_GATT_IF_NONE;
static uint16_t conn_id = 0;
static uint16_t service_handle;
static uint16_t ra_gear_char_handle;
static uint16_t dec_gear_char_handle;
static uint16_t jog_control_char_handle;
static uint16_t status_char_handle;
static uint16_t sync_char_handle;
static uint16_t tracking_mode_char_handle;
static uint16_t step_target_char_handle;

// Advertising configuration
static uint8_t adv_config_done = 0;
#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 16,
    .p_service_uuid = (uint8_t[]){0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E},
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 0,
    .p_service_uuid = NULL,
    .flag = 0,
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// Function prototypes
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void send_coordinate_report(void);
static void create_telescope_service(void);
static void handle_char_write(uint16_t handle, uint8_t *value, uint16_t len);

/**
 * Calculate tracking rate in steps per second for given mode
 */
static float get_tracking_rate_steps_per_sec(tracking_mode_t mode) {
    float arcsec_per_sec;

    switch (mode) {
    case TRACKING_SIDEREAL:
        arcsec_per_sec = SIDEREAL_RATE_ARCSEC_PER_SEC;
        break;
    case TRACKING_SOLAR:
        arcsec_per_sec = SOLAR_RATE_ARCSEC_PER_SEC;
        break;
    case TRACKING_LUNAR:
        arcsec_per_sec = LUNAR_RATE_ARCSEC_PER_SEC;
        break;
    default:
        return 0.0f;
    }

    // Convert arcseconds per second to steps per second
    // 1 degree = 3600 arcseconds, 1 rotation = 360 degrees = 1,296,000 arcseconds
    // So: steps_per_sec = (arcsec_per_sec / 1,296,000) * steps_per_rotation
    float arcsec_per_rotation = 360.0f * 3600.0f; // 1,296,000 arcseconds per rotation
    return (arcsec_per_sec / arcsec_per_rotation) * current_gear_config.ra_gear_ratio;
}


/**
 * Initialize Bluetooth control system
 */
esp_err_t bluetooth_control_init(void) {
    ESP_LOGI(TAG, "Initializing BLE control...");

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

    ESP_LOGI(TAG, "Gear ratios - RA: %.0f steps/rotation, DEC: %.0f steps/rotation",
             current_gear_config.ra_gear_ratio, current_gear_config.dec_gear_ratio);

    // Initialize Bluetooth controller for BLE
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth controller init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_BLE)) != ESP_OK) {
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

    if ((ret = esp_ble_gatts_register_callback(gatts_event_handler)) != ESP_OK) {
        ESP_LOGE(TAG, "GATTS register failed: %s", esp_err_to_name(ret));
        return ret;
    }

    if ((ret = esp_ble_gap_register_callback(gap_event_handler)) != ESP_OK) {
        ESP_LOGE(TAG, "GAP register failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set security parameters for pairing
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_NO_BOND;  // No bonding required
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;           // No input/output capability
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;

    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    if ((ret = esp_ble_gatts_app_register(0)) != ESP_OK) {
        ESP_LOGE(TAG, "GATTS app register failed: %s", esp_err_to_name(ret));
        return ret;
    }

    if ((ret = esp_ble_gatt_set_local_mtu(517)) != ESP_OK) {
        ESP_LOGE(TAG, "Set local MTU failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Create Bluetooth task
    BaseType_t task_ret = xTaskCreate(bluetooth_task, "bluetooth_task", 4096, NULL, 5, &bluetooth_task_handle);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Bluetooth task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "BLE initialized successfully, device name: %s", DEVICE_NAME);
    return ESP_OK;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0) {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0) {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Advertising start failed");
        } else {
            ESP_LOGI(TAG, "Advertising started successfully");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Advertising stop failed");
        } else {
            ESP_LOGI(TAG, "Stop adv successfully");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                 param->update_conn_params.status,
                 param->update_conn_params.min_int,
                 param->update_conn_params.max_int,
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;
    case ESP_GAP_BLE_PASSKEY_REQ_EVT:
        ESP_LOGI(TAG, "ESP_GAP_BLE_PASSKEY_REQ_EVT");
        break;
    case ESP_GAP_BLE_OOB_REQ_EVT:
        ESP_LOGI(TAG, "ESP_GAP_BLE_OOB_REQ_EVT");
        break;
    case ESP_GAP_BLE_LOCAL_IR_EVT:
        ESP_LOGI(TAG, "ESP_GAP_BLE_LOCAL_IR_EVT");
        break;
    case ESP_GAP_BLE_LOCAL_ER_EVT:
        ESP_LOGI(TAG, "ESP_GAP_BLE_LOCAL_ER_EVT");
        break;
    case ESP_GAP_BLE_NC_REQ_EVT:
        ESP_LOGI(TAG, "ESP_GAP_BLE_NC_REQ_EVT");
        esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        ESP_LOGI(TAG, "ESP_GAP_BLE_SEC_REQ_EVT");
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:
        ESP_LOGI(TAG, "ESP_GAP_BLE_PASSKEY_NOTIF_EVT passkey:%d", param->ble_security.key_notif.passkey);
        break;
    case ESP_GAP_BLE_KEY_EVT:
        ESP_LOGI(TAG, "ESP_GAP_BLE_KEY_EVT key_type = %s", param->ble_security.ble_key.key_type);
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        ESP_LOGI(TAG, "ESP_GAP_BLE_AUTH_CMPL_EVT auth mode = %s, success = %s",
                 param->ble_security.auth_cmpl.auth_mode,
                 param->ble_security.auth_cmpl.success ? "success" : "fail");
        if (!param->ble_security.auth_cmpl.success) {
            ESP_LOGI(TAG, "fail reason = 0x%x", param->ble_security.auth_cmpl.fail_reason);
        }
        break;
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatt_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(TAG, "REGISTER_APP_EVT, status %d, app_id %d", param->reg.status, param->reg.app_id);
        gatts_if = gatt_if;

        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(DEVICE_NAME);
        if (set_dev_name_ret) {
            ESP_LOGE(TAG, "set device name failed, error code = %x", set_dev_name_ret);
        }

        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret) {
            ESP_LOGE(TAG, "config adv data failed, error code = %x", ret);
        }
        adv_config_done |= adv_config_flag;

        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret) {
            ESP_LOGE(TAG, "config scan response data failed, error code = %x", ret);
        }
        adv_config_done |= scan_rsp_config_flag;

        create_telescope_service();
        break;

    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(TAG, "CREATE_SERVICE_EVT, status %d, service_handle %d", param->create.status, param->create.service_handle);
        service_handle = param->create.service_handle;
        esp_ble_gatts_start_service(service_handle);

        // Add RA gear ratio characteristic
        esp_bt_uuid_t ra_gear_uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid = {.uuid16 = CHAR_RA_GEAR_RATIO_UUID}
        };
        esp_ble_gatts_add_char(service_handle, &ra_gear_uuid,
                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                               ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE,
                               NULL, NULL);
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        ESP_LOGI(TAG, "ADD_CHAR_EVT, status %d, attr_handle %d, service_handle %d",
                param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);

        // Store characteristic handles
        if (param->add_char.char_uuid.uuid.uuid16 == CHAR_RA_GEAR_RATIO_UUID) {
            ra_gear_char_handle = param->add_char.attr_handle;
            // Add next characteristic
            esp_bt_uuid_t dec_gear_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = {.uuid16 = CHAR_DEC_GEAR_RATIO_UUID}
            };
            esp_ble_gatts_add_char(service_handle, &dec_gear_uuid,
                                   ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                   ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE,
                                   NULL, NULL);
        } else if (param->add_char.char_uuid.uuid.uuid16 == CHAR_DEC_GEAR_RATIO_UUID) {
            dec_gear_char_handle = param->add_char.attr_handle;
            // Add jog control characteristic
            esp_bt_uuid_t jog_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = {.uuid16 = CHAR_JOG_CONTROL_UUID}
            };
            esp_ble_gatts_add_char(service_handle, &jog_uuid,
                                   ESP_GATT_PERM_WRITE,
                                   ESP_GATT_CHAR_PROP_BIT_WRITE,
                                   NULL, NULL);
        } else if (param->add_char.char_uuid.uuid.uuid16 == CHAR_JOG_CONTROL_UUID) {
            jog_control_char_handle = param->add_char.attr_handle;
            // Add status characteristic
            esp_bt_uuid_t status_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = {.uuid16 = CHAR_STATUS_UUID}
            };
            esp_ble_gatts_add_char(service_handle, &status_uuid,
                                   ESP_GATT_PERM_READ,
                                   ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                   NULL, NULL);
        } else if (param->add_char.char_uuid.uuid.uuid16 == CHAR_STATUS_UUID) {
            status_char_handle = param->add_char.attr_handle;
            // Add sync characteristic
            esp_bt_uuid_t sync_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = {.uuid16 = CHAR_SYNC_UUID}
            };
            esp_ble_gatts_add_char(service_handle, &sync_uuid,
                                   ESP_GATT_PERM_WRITE,
                                   ESP_GATT_CHAR_PROP_BIT_WRITE,
                                   NULL, NULL);
        } else if (param->add_char.char_uuid.uuid.uuid16 == CHAR_SYNC_UUID) {
            sync_char_handle = param->add_char.attr_handle;
            // Add tracking mode characteristic
            esp_bt_uuid_t tracking_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = {.uuid16 = CHAR_TRACKING_MODE_UUID}
            };
            esp_ble_gatts_add_char(service_handle, &tracking_uuid,
                                   ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                   ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE,
                                   NULL, NULL);
        } else if (param->add_char.char_uuid.uuid.uuid16 == CHAR_TRACKING_MODE_UUID) {
            tracking_mode_char_handle = param->add_char.attr_handle;
            // Add step target characteristic
            esp_bt_uuid_t step_target_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid = {.uuid16 = CHAR_STEP_TARGET_UUID}
            };
            esp_ble_gatts_add_char(service_handle, &step_target_uuid,
                                   ESP_GATT_PERM_WRITE,
                                   ESP_GATT_CHAR_PROP_BIT_WRITE,
                                   NULL, NULL);
        } else if (param->add_char.char_uuid.uuid.uuid16 == CHAR_STEP_TARGET_UUID) {
            step_target_char_handle = param->add_char.attr_handle;
            ESP_LOGI(TAG, "All characteristics added successfully");
        }
        break;

    case ESP_GATTS_READ_EVT:
        // ESP_LOGI(TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d",
        //         param->read.conn_id, param->read.trans_id, param->read.handle);

        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;

        if (param->read.handle == ra_gear_char_handle) {
            memcpy(rsp.attr_value.value, &current_gear_config.ra_gear_ratio, sizeof(float));
            rsp.attr_value.len = sizeof(float);
        } else if (param->read.handle == dec_gear_char_handle) {
            memcpy(rsp.attr_value.value, &current_gear_config.dec_gear_ratio, sizeof(float));
            rsp.attr_value.len = sizeof(float);
        } else if (param->read.handle == tracking_mode_char_handle) {
            uint8_t mode = (uint8_t)current_tracking_mode;
            memcpy(rsp.attr_value.value, &mode, sizeof(uint8_t));
            rsp.attr_value.len = sizeof(uint8_t);
        } else if (param->read.handle == status_char_handle) {
            // Return status as compact binary data (more efficient than JSON)
            struct {
                float ra_degrees;
                float dec_degrees;
                float ra_velocity;    // steps per second
                float dec_velocity;   // steps per second
                uint8_t tracking_mode;
                uint8_t status_flags;
            } __attribute__((packed)) status_data;

            dms_t ra, dec;
            get_current_ra_dec(&ra, &dec);

            // Convert DMS to decimal degrees
            status_data.ra_degrees = ra.degrees + (ra.minutes / 60.0f) + (ra.seconds / 3600.0f);
            status_data.dec_degrees = dec.degrees + (dec.minutes / 60.0f) + (dec.seconds / 3600.0f);
            if (dec.negative) status_data.dec_degrees = -status_data.dec_degrees;

            // status_data.ra_gear = current_gear_config.ra_gear_ratio;
            // status_data.dec_gear = current_gear_config.dec_gear_ratio;

            // Get current velocities
            status_data.ra_velocity = get_current_ra_velocity();
            status_data.dec_velocity = get_current_dec_velocity();

            // ESP_LOGI(TAG, "Current RA Velocity: %d steps/s", status_data.ra_velocity);
            // ESP_LOGI(TAG, "Current DEC Velocity: %d steps/s", status_data.dec_velocity);

            // Get tracking mode and status flags
            status_data.tracking_mode = (uint8_t)current_tracking_mode;
            status_data.status_flags = get_current_status_flags();

            memcpy(rsp.attr_value.value, &status_data, sizeof(status_data));
            rsp.attr_value.len = sizeof(status_data);
        }

        esp_ble_gatts_send_response(gatt_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        break;

    case ESP_GATTS_WRITE_EVT:
        ESP_LOGI(TAG, "GATT_WRITE_EVT, handle = %d, value len = %d", param->write.handle, param->write.len);
        handle_char_write(param->write.handle, param->write.value, param->write.len);

        if (param->write.need_rsp) {
            esp_ble_gatts_send_response(gatt_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        }
        break;

    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
        conn_id = param->connect.conn_id;
        bluetooth_connected = true;
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
        bluetooth_connected = false;
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GATTS_START_EVT:
        ESP_LOGI(TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
        break;

    default:
        break;
    }
}

static void create_telescope_service(void) {
    esp_gatt_srvc_id_t service_id = {
        .is_primary = true,
        .id.inst_id = 0x00,
        .id.uuid.len = ESP_UUID_LEN_128,
    };
    memcpy(service_id.id.uuid.uuid.uuid128, (uint8_t[]){0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E}, 16);

    esp_ble_gatts_create_service(gatts_if, &service_id, GATTS_NUM_HANDLE);
}

static void handle_char_write(uint16_t handle, uint8_t *value, uint16_t len) {
    if (handle == ra_gear_char_handle && len == sizeof(float)) {
        float new_ratio;
        memcpy(&new_ratio, value, sizeof(float));
        if (new_ratio > 0) {
            set_ra_gear_ratio(new_ratio);
            ESP_LOGI(TAG, "RA gear ratio updated to %.1f", new_ratio);
        }
    } else if (handle == dec_gear_char_handle && len == sizeof(float)) {
        float new_ratio;
        memcpy(&new_ratio, value, sizeof(float));
        if (new_ratio > 0) {
            set_dec_gear_ratio(new_ratio);
            ESP_LOGI(TAG, "DEC gear ratio updated to %.1f", new_ratio);
        }
    } else if (handle == jog_control_char_handle && len == 1) {
        uint8_t jog_cmd = value[0];
        switch (jog_cmd) {
        case 1: // RA positive
            jog_ra_positive();
            ESP_LOGI(TAG, "Jog RA positive");
            break;
        case 2: // RA negative
            jog_ra_negative();
            ESP_LOGI(TAG, "Jog RA negative");
            break;
        case 3: // DEC positive
            jog_dec_positive();
            ESP_LOGI(TAG, "Jog DEC positive");
            break;
        case 4: // DEC negative
            jog_dec_negative();
            ESP_LOGI(TAG, "Jog DEC negative");
            break;
        default:
            ESP_LOGW(TAG, "Unknown jog command: %d", jog_cmd);
            break;
        }
    } else if (handle == sync_char_handle && len == 1) {
        if (value[0] == 1) {
            sync_coordinates();
            ESP_LOGI(TAG, "Coordinates synchronized");
        }
    } else if (handle == tracking_mode_char_handle && len == 1) {
        uint8_t new_mode = value[0];
        if (new_mode <= TRACKING_LUNAR) {
            set_tracking_mode((tracking_mode_t)new_mode);
            ESP_LOGI(TAG, "Tracking mode updated to %d", new_mode);
        } else {
            ESP_LOGW(TAG, "Invalid tracking mode: %d", new_mode);
        }
    } else if (handle == step_target_char_handle && len == 5) {
        // Format: 1 byte axis (0=RA, 1=DEC) + 4 bytes int32 position
        uint8_t axis = value[0];
        int32_t target_position;
        memcpy(&target_position, &value[1], sizeof(int32_t));

        if (axis == 0) {
            set_ra_step_target(target_position);
            ESP_LOGI(TAG, "RA step target set to %ld", target_position);
        } else if (axis == 1) {
            set_dec_step_target(target_position);
            ESP_LOGI(TAG, "DEC step target set to %ld", target_position);
        } else {
            ESP_LOGW(TAG, "Invalid axis for step target: %d", axis);
        }
    }
}


/**
 * Process Bluetooth commands (legacy function for compatibility)
 */
void process_bluetooth_command(const char* command) {
    // This function is now deprecated as we use BLE characteristics
    ESP_LOGW(TAG, "process_bluetooth_command is deprecated in BLE mode");
}

/**
 * Send response over BLE
 */
void send_bluetooth_response(const char* response) {
    if (bluetooth_connected && conn_id != 0 && status_char_handle != 0) {
        esp_ble_gatts_send_indicate(gatts_if, conn_id, status_char_handle,
                                   strlen(response), (uint8_t*)response, false);
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

    // If tracking is enabled, subtract the sidereal rate movement since tracking started
    if (current_tracking_mode != TRACKING_OFF && tracking_start_time > 0) {
        status_msg_t status;
        if (get_latest_status(&status)) {
            // Calculate time elapsed since tracking started (in seconds)
            float elapsed_time_sec = (float)(status.system_time - tracking_start_time) / 1000000.0f;

            // Calculate how much the sky has moved (always use sidereal rate for RA coordinate calculation)
            float sidereal_steps_per_sec = get_tracking_rate_steps_per_sec(TRACKING_SIDEREAL);
            int32_t sidereal_movement_steps = (int32_t)(sidereal_steps_per_sec * elapsed_time_sec);

            // Subtract the sidereal movement to get the actual RA coordinate
            adjusted_steps -= sidereal_movement_steps;
        }
    }

    // Convert steps to degrees (RA is 0-360 degrees)
    // gear_ratio is now steps per full rotation (360 degrees)
    float degrees = (float)adjusted_steps * 360.0f / current_gear_config.ra_gear_ratio;

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
    // gear_ratio is now steps per full rotation (360 degrees)
    float degrees = (float)adjusted_steps * 360.0f / current_gear_config.dec_gear_ratio;

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
    int32_t steps = (int32_t)(degrees * current_gear_config.ra_gear_ratio / 360.0f);
    return steps + coord_offsets.ra_offset_steps;
}

/**
 * Convert DEC DMS to steps
 */
int32_t dec_dms_to_steps(const dms_t* dms) {
    float degrees = dms->degrees + (dms->minutes / 60.0f) + (dms->seconds / 3600.0f);
    if (dms->negative) degrees = -degrees;
    int32_t steps = (int32_t)(degrees * current_gear_config.dec_gear_ratio / 360.0f);
    return steps + coord_offsets.dec_offset_steps;
}

/**
 * Sync coordinates (zero out RA/DEC by setting offsets)
 */
void sync_coordinates(void) {
    coord_offsets.ra_offset_steps = get_current_ra_position();
    coord_offsets.dec_offset_steps = get_current_dec_position();

    // Reset tracking start time to current time if tracking is active
    if (current_tracking_mode != TRACKING_OFF) {
        status_msg_t status;
        if (get_latest_status(&status)) {
            tracking_start_time = status.system_time;
            tracking_start_position = coord_offsets.ra_offset_steps;
            ESP_LOGI(TAG, "Tracking reference reset to current time and position");
        }
    }

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

    return send_pvt_target(0, target_pos, get_tracking_rate_steps_per_sec(current_tracking_mode), target_time);
}

esp_err_t jog_ra_negative(void) {
    status_msg_t status;
    if (!get_latest_status(&status)) {
        return ESP_FAIL;
    }

    int32_t target_pos = status.ra_position - JOG_STEP_SIZE;
    uint64_t target_time = status.ra_update_time + 100000; // 100ms from last update

    return send_pvt_target(0, target_pos, get_tracking_rate_steps_per_sec(current_tracking_mode), target_time);
}

esp_err_t jog_dec_positive(void) {
    status_msg_t status;
    if (!get_latest_status(&status)) {
        return ESP_FAIL;
    }

    int32_t target_pos = status.dec_position + JOG_STEP_SIZE;
    uint64_t target_time = status.dec_update_time; // 100ms from last update

    return send_pvt_target(1, target_pos, 0, target_time); // status.dec_velocity, target_time);
}

esp_err_t jog_dec_negative(void) {
    status_msg_t status;
    if (!get_latest_status(&status)) {
        return ESP_FAIL;
    }

    int32_t target_pos = status.dec_position - JOG_STEP_SIZE;
    uint64_t target_time = status.dec_update_time; // 100ms from last update

    return send_pvt_target(1, target_pos, 0, target_time);
}

/**
 * Set RA step target position (debugging function - disables tracking)
 */
esp_err_t set_ra_step_target(int32_t target_position) {
    // Turn off tracking for debugging
    set_tracking_mode(TRACKING_OFF);

    status_msg_t status;
    if (!get_latest_status(&status)) {
        ESP_LOGE(TAG, "Failed to get current status for RA step target");
        return ESP_FAIL;
    }

    // Use current time + small offset
    uint64_t target_time = status.ra_update_time; // 100ms from last update

    // Send PVT target with specified position, zero velocity, and current time
    return send_pvt_target(0, target_position, 0.0f, target_time);
}

/**
 * Set DEC step target position (debugging function - disables tracking)
 */
esp_err_t set_dec_step_target(int32_t target_position) {
    // Turn off tracking for debugging
    set_tracking_mode(TRACKING_OFF);

    status_msg_t status;
    if (!get_latest_status(&status)) {
        ESP_LOGE(TAG, "Failed to get current status for DEC step target");
        return ESP_FAIL;
    }

    // Use current time + small offset
    uint64_t target_time = status.dec_update_time; // 100ms from last update

    // Send PVT target with specified position, zero velocity, and current time
    return send_pvt_target(1, target_position, 0.0f, target_time);
}

/**
 * Set tracking mode and update RA tracking velocity
 */
esp_err_t set_tracking_mode(tracking_mode_t mode) {
    current_tracking_mode = mode;

    status_msg_t status;
    if (!get_latest_status(&status)) {
        ESP_LOGE(TAG, "Failed to get current status for tracking");
        return ESP_FAIL;
    }

    float tracking_velocity = 0.0f;
    if (mode != TRACKING_OFF) {
        // Record tracking start time and position for RA coordinate calculation
        tracking_start_time = status.system_time;
        tracking_start_position = status.ra_position;

        tracking_velocity = get_tracking_rate_steps_per_sec(mode);
        ESP_LOGI(TAG, "Setting tracking velocity to %.1f steps/sec", tracking_velocity);
        ESP_LOGI(TAG, "Tracking started at time %llu, position %ld", tracking_start_time, tracking_start_position);
    } else {
        tracking_start_time = 0;
        tracking_start_position = 0;
        ESP_LOGI(TAG, "Tracking disabled");
    }

    // Send PVT target with current position and new velocity
    uint64_t target_time = status.ra_update_time + 100000; // 100ms from last update
    return send_pvt_target(0, status.ra_position, tracking_velocity, target_time);
}

/**
 * Get current tracking mode
 */
tracking_mode_t get_tracking_mode(void) {
    return current_tracking_mode;
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
        "RA:  %02d°%02d'%02d\" (steps: %ld, vel: %.1f)\n"
        "DEC: %s%02d°%02d'%02d\" (steps: %ld, vel: %.1f)\n"
        "Gear ratios: RA=%.0f, DEC=%.0f steps/rotation\n"
        "Tracking: %s\n"
        "Status flags: 0x%02X\n"
        "========================\n",
        ra.degrees, ra.minutes, ra.seconds, status.ra_position, status.ra_velocity,
        dec.negative ? "-" : "+", dec.degrees, dec.minutes, dec.seconds, status.dec_position, status.dec_velocity,
        current_gear_config.ra_gear_ratio, current_gear_config.dec_gear_ratio,
        (current_tracking_mode == TRACKING_OFF) ? "Off" :
        (current_tracking_mode == TRACKING_SIDEREAL) ? "Sidereal" :
        (current_tracking_mode == TRACKING_SOLAR) ? "Solar" :
        (current_tracking_mode == TRACKING_LUNAR) ? "Lunar" : "Unknown",
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
