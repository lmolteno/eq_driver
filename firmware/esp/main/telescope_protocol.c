#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "telescope_protocol.h"

static const char *TAG = "telescope_protocol";

// UART configuration
#define UART_NUM            UART_NUM_1
#define UART_TX_PIN         45
#define UART_RX_PIN         0
#define UART_BAUD_RATE      115200
#define ESP_UART_BUFFER_SIZE    1024

// Message parsing state
static message_frame_t current_frame;
static uint8_t frame_state = 0;  // 0=waiting_start, 1=msg_id, 2=length, 3=payload, 4=checksum
static uint8_t payload_index = 0;
static uint8_t expected_length = 0;

// Task handle
static TaskHandle_t protocol_task_handle = NULL;

// Latest telescope status (stored in memory for calculations)
static status_msg_t latest_status;
static bool status_received = false;
static uint64_t last_status_time = 0;

// Function prototypes
static void reset_frame_parser(void);
static void process_received_bytes(void);
static void handle_status_report(const status_msg_t* status);
static void handle_ack_nak(const ack_msg_t* ack);
static const char* get_status_flags_string(uint8_t flags);
static void print_current_status(void);

/**
 * Initialize the telescope protocol system
 */
esp_err_t protocol_init(void) {
    ESP_LOGI(TAG, "Initializing telescope protocol...");

    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Install UART driver
    esp_err_t ret = uart_driver_install(UART_NUM, ESP_UART_BUFFER_SIZE * 2, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_param_config(UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        return ret;
    }

    // Reset parser state
    reset_frame_parser();

    // Create protocol task
    BaseType_t task_ret = xTaskCreate(protocol_task, "protocol_task", 4096, NULL, 5, &protocol_task_handle);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create protocol task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Protocol initialized on UART%d (TX=IO%d, RX=IO%d)", UART_NUM, UART_TX_PIN, UART_RX_PIN);
    return ESP_OK;
}

/**
 * Calculate XOR checksum
 */
uint8_t calculate_checksum(const uint8_t* data, uint8_t length) {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

/**
 * Validate received message frame
 */
bool validate_message(const message_frame_t* frame) {
    if (frame->start_byte != START_BYTE) {
        return false;
    }

    if (frame->length > MAX_PAYLOAD_SIZE) {
        return false;
    }

    // Calculate expected checksum (excluding the checksum byte itself)
    uint8_t expected_checksum = frame->start_byte ^ frame->msg_id ^ frame->length;
    for (uint8_t i = 0; i < frame->length; i++) {
        expected_checksum ^= frame->payload[i];
    }

    return (expected_checksum == frame->checksum);
}

/**
 * Reset frame parser to initial state
 */
static void reset_frame_parser(void) {
    frame_state = 0;
    payload_index = 0;
    expected_length = 0;
    memset(&current_frame, 0, sizeof(current_frame));
}

/**
 * Process received bytes and parse frames
 */
static void process_received_bytes(void) {
    uint8_t byte;
    int len = uart_read_bytes(UART_NUM, &byte, 1, 0);

    while (len > 0) {
        switch (frame_state) {
            case 0:  // Waiting for start byte
                if (byte == START_BYTE) {
                    current_frame.start_byte = byte;
                    frame_state = 1;
                }
                break;

            case 1:  // Message ID
                current_frame.msg_id = byte;
                frame_state = 2;
                break;

            case 2:  // Length
                current_frame.length = byte;
                expected_length = byte;
                if (expected_length > MAX_PAYLOAD_SIZE) {
                    ESP_LOGW(TAG, "Invalid payload length: %d", expected_length);
                    reset_frame_parser();
                } else if (expected_length == 0) {
                    frame_state = 4;  // No payload, go to checksum
                } else {
                    frame_state = 3;  // Expect payload
                    payload_index = 0;
                }
                break;

            case 3:  // Payload
                current_frame.payload[payload_index++] = byte;
                if (payload_index >= expected_length) {
                    frame_state = 4;  // Ready for checksum
                }
                break;

            case 4:  // Checksum
                current_frame.checksum = byte;

                // Validate and process complete frame
                if (validate_message(&current_frame)) {
                    handle_received_message(&current_frame);
                } else {
                    ESP_LOGW(TAG, "Invalid frame: ID=0x%02X, checksum error", current_frame.msg_id);
                }

                reset_frame_parser();
                break;

            default:
                reset_frame_parser();
                break;
        }

        // Try to read next byte
        len = uart_read_bytes(UART_NUM, &byte, 1, 0);
    }
}

/**
 * Send a message frame
 */
esp_err_t send_message(uint8_t msg_id, const void* payload, uint8_t payload_len) {
    if (payload_len > MAX_PAYLOAD_SIZE) {
        ESP_LOGE(TAG, "Payload too large: %d", payload_len);
        return ESP_ERR_INVALID_ARG;
    }

    // Build frame
    message_frame_t frame;
    frame.start_byte = START_BYTE;
    frame.msg_id = msg_id;
    frame.length = payload_len;

    if (payload && payload_len > 0) {
        memcpy(frame.payload, payload, payload_len);
    }

    // Calculate checksum
    frame.checksum = frame.start_byte ^ frame.msg_id ^ frame.length;
    for (uint8_t i = 0; i < payload_len; i++) {
        frame.checksum ^= frame.payload[i];
    }

    // Send frame
    uint8_t* frame_data = (uint8_t*)&frame;
    int frame_size = 3 + payload_len + 1; // start_byte + msg_id + length + payload + checksum

    int bytes_written = uart_write_bytes(UART_NUM, frame_data, frame_size);
    if (bytes_written != frame_size) {
        ESP_LOGE(TAG, "Failed to send complete frame: %d/%d bytes", bytes_written, frame_size);
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "Sent message ID 0x%02X, %d bytes", msg_id, payload_len);
    return ESP_OK;
}

/**
 * Send get status command
 */
esp_err_t send_get_status(void) {
    ESP_LOGI(TAG, "Requesting status from RP2040");
    return send_message(MSG_GET_STATUS, NULL, 0);
}

/**
 * Send emergency stop command
 */
esp_err_t send_emergency_stop(void) {
    ESP_LOGI(TAG, "Sending emergency stop to RP2040");
    return send_message(MSG_EMERGENCY_STOP, NULL, 0);
}

/**
 * Send PVT target command
 */
esp_err_t send_pvt_target(uint8_t axis, int32_t position, int32_t velocity, uint64_t time) {
    pvt_target_msg_t msg = {
        .axis = axis,
        .target_position = position,
        .target_velocity = velocity,
        .target_time = time
    };

    ESP_LOGI(TAG, "Sending PVT target: axis=%d, pos=%ld, vel=%ld, time=%llu",
             axis, position, velocity, time);

    return send_message(MSG_SET_PVT_TARGET, &msg, sizeof(msg));
}

/**
 * Handle received message
 */
void handle_received_message(const message_frame_t* frame) {
    switch (frame->msg_id) {
        case MSG_STATUS_REPORT:
            if (frame->length == sizeof(status_msg_t)) {
                handle_status_report((const status_msg_t*)frame->payload);
            } else {
                ESP_LOGW(TAG, "Invalid status report length: %d", frame->length);
            }
            break;

        case MSG_ACK_NAK:
            if (frame->length == sizeof(ack_msg_t)) {
                handle_ack_nak((const ack_msg_t*)frame->payload);
            } else {
                ESP_LOGW(TAG, "Invalid ACK/NAK length: %d", frame->length);
            }
            break;

        default:
            ESP_LOGW(TAG, "Unknown message ID: 0x%02X", frame->msg_id);
            break;
    }
}

/**
 * Get human-readable status flags string
 */
static const char* get_status_flags_string(uint8_t flags) {
    static char status_str[256];
    status_str[0] = '\0';

    if (flags & STATUS_RA_MOVING) strcat(status_str, "RA_MOVING ");
    if (flags & STATUS_DEC_MOVING) strcat(status_str, "DEC_MOVING ");
    if (flags & STATUS_RA_LIMIT) strcat(status_str, "RA_LIMIT ");
    if (flags & STATUS_DEC_LIMIT) strcat(status_str, "DEC_LIMIT ");
    if (flags & STATUS_RA_ERROR) strcat(status_str, "RA_ERROR ");
    if (flags & STATUS_DEC_ERROR) strcat(status_str, "DEC_ERROR ");
    if (flags & STATUS_EMERGENCY) strcat(status_str, "EMERGENCY ");
    if (flags & STATUS_TMC_ERROR) strcat(status_str, "TMC_ERROR ");

    if (strlen(status_str) == 0) {
        strcpy(status_str, "IDLE");
    }

    return status_str;
}

/**
 * Handle status report from RP2040
 */
static void handle_status_report(const status_msg_t* status) {
    // Store the latest status in memory for calculations
    memcpy(&latest_status, status, sizeof(status_msg_t));
    status_received = true;
    last_status_time = esp_timer_get_time();

    ESP_LOGD(TAG, "Status received: RA=%ld, DEC=%ld, flags=0x%02X",
             status->ra_position, status->dec_position, status->status_flags);
}

/**
 * Handle ACK/NAK from RP2040
 */
static void handle_ack_nak(const ack_msg_t* ack) {
    const char* result_str;
    switch (ack->result) {
        case RESULT_SUCCESS:      result_str = "SUCCESS"; break;
        case RESULT_INVALID_MSG:  result_str = "INVALID_MSG"; break;
        case RESULT_INVALID_LEN:  result_str = "INVALID_LEN"; break;
        case RESULT_CHECKSUM_ERR: result_str = "CHECKSUM_ERR"; break;
        case RESULT_AXIS_ERROR:   result_str = "AXIS_ERROR"; break;
        case RESULT_LIMIT_ERROR:  result_str = "LIMIT_ERROR"; break;
        default:                  result_str = "UNKNOWN"; break;
    }

    printf("ACK/NAK: Message 0x%02X -> %s (0x%02X)\n",
           ack->original_msg_id, result_str, ack->result);
}

/**
 * Print current telescope status (called periodically)
 */
static void print_current_status(void) {
    if (!status_received) {
        printf("No telescope status received yet\n");
        return;
    }

    uint64_t current_time = esp_timer_get_time();
    uint64_t age_ms = (current_time - last_status_time) / 1000;

    printf("\n=== TELESCOPE STATUS (age: %llu ms) ===\n", age_ms);
    printf("System Time: %llu us\n", latest_status.system_time);
    printf("RA  Position: %ld steps, Velocity: %ld steps/s, Update: %llu us, Current: %u\n",
           latest_status.ra_position, latest_status.ra_velocity,
           latest_status.ra_update_time, latest_status.ra_current);
    printf("DEC Position: %ld steps, Velocity: %ld steps/s, Update: %llu us, Current: %u\n",
           latest_status.dec_position, latest_status.dec_velocity,
           latest_status.dec_update_time, latest_status.dec_current);
    printf("Status Flags: 0x%02X (%s)\n", latest_status.status_flags,
           get_status_flags_string(latest_status.status_flags));
    printf("=======================================\n\n");
}

/**
 * Main protocol task
 */
void protocol_task(void *pvParameters) {
    ESP_LOGI(TAG, "Protocol task started");

    TickType_t last_status_request = 0;
    TickType_t last_status_print = 0;
    const TickType_t status_request_interval = pdMS_TO_TICKS(5000); // Request status every 5 seconds
    const TickType_t status_print_interval = pdMS_TO_TICKS(1000);   // Print status every 1 second

    while (1) {
        // Process any received messages
        process_received_bytes();

        TickType_t now = xTaskGetTickCount();

        // Periodically request status
        if (now - last_status_request >= status_request_interval) {
            send_get_status();
            last_status_request = now;
        }

        // Periodically print current status
        if (now - last_status_print >= status_print_interval) {
            print_current_status();
            last_status_print = now;
        }

        // Small delay to prevent busy waiting
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * Status access functions for calculations
 */
bool get_latest_status(status_msg_t* status) {
    if (!status_received || !status) {
        return false;
    }
    memcpy(status, &latest_status, sizeof(status_msg_t));
    return true;
}

int32_t get_current_ra_position(void) {
    return status_received ? latest_status.ra_position : 0;
}

int32_t get_current_dec_position(void) {
    return status_received ? latest_status.dec_position : 0;
}

int32_t get_current_ra_velocity(void) {
    return status_received ? latest_status.ra_velocity : 0;
}

int32_t get_current_dec_velocity(void) {
    return status_received ? latest_status.dec_velocity : 0;
}

uint8_t get_current_status_flags(void) {
    return status_received ? latest_status.status_flags : 0;
}