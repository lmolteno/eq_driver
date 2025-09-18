#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "telescope_protocol.h"
#include "motor_control.h"
#include "tmc2209.h"

#if USE_USB_CDC
#include "pico/stdio_usb.h"
#include "tusb.h"
#endif

// Global state
#if USE_USB_CDC
// USB CDC mode - no UART needed
#else
static uart_inst_t* protocol_uart = uart1;
#endif
static motor_system_t* motor_sys = NULL;

// Receive buffer and state
static uint8_t rx_buffer[UART_BUFFER_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;
static volatile bool rx_overflow = false;

// Message parsing state
static message_frame_t current_frame;
static uint8_t frame_state = 0;  // 0=waiting_start, 1=msg_id, 2=length, 3=payload, 4=checksum
static uint8_t payload_index = 0;
static uint8_t expected_length = 0;

// Timing
static absolute_time_t last_status_time;
static const uint32_t STATUS_INTERVAL_US = 1000000 / STATUS_REPORT_HZ; // 20Hz status reports

// Function prototypes
static void uart_rx_isr(void);
static bool get_rx_byte(uint8_t* byte);
static void process_received_bytes(void);
static void handle_set_velocity(const set_velocity_msg_t* msg);
static void handle_set_position(const set_position_msg_t* msg);
static void handle_set_pvt_target(const pvt_target_msg_t* msg);
static void handle_get_status(void);
static void handle_emergency_stop(void);
static void handle_set_correction(const correction_msg_t* msg);
static void reset_frame_parser(void);

/**
 * Initialize the telescope protocol system
 */
int protocol_init(void) {
#if USE_USB_CDC
    // USB CDC mode - just ensure stdio is initialized
    stdio_usb_init();
    
    printf("Protocol system initialized on USB CDC\\n");
#else
    // Initialize UART for ESP32 communication
    uart_init(protocol_uart, 115200);
    
    // Set up GPIO pins for UART1
    gpio_set_function(28, GPIO_FUNC_UART);  // TX (GP28)
    gpio_set_function(29, GPIO_FUNC_UART);  // RX (GP29)
    
    // Configure UART parameters
    uart_set_hw_flow(protocol_uart, false, false);  // No hardware flow control
    uart_set_format(protocol_uart, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(protocol_uart, true);
    
    // Set up RX interrupt
    irq_set_exclusive_handler(UART1_IRQ, uart_rx_isr);
    irq_set_enabled(UART1_IRQ, true);
    uart_set_irq_enables(protocol_uart, true, false);  // RX IRQ only
    
    printf("Protocol system initialized on UART1 (pins GP28/GP29)\\n");
#endif
    
    // Initialize timing
    last_status_time = get_absolute_time();
    
    // Reset parser state
    reset_frame_parser();
    
    return 0;
}

/**
 * Set motor system reference for protocol handlers
 */
void protocol_set_motor_system(motor_system_t* sys) {
    motor_sys = sys;
}

/**
 * RX interrupt handler - stores bytes in circular buffer
 */
#if USE_USB_CDC
// USB CDC doesn't use interrupts - polling in main loop
#else
static void uart_rx_isr(void) {
    while (uart_is_readable(protocol_uart)) {
        uint8_t byte = uart_getc(protocol_uart);
        
        uint16_t next_head = (rx_head + 1) % UART_BUFFER_SIZE;
        if (next_head != rx_tail) {
            rx_buffer[rx_head] = byte;
            rx_head = next_head;
        } else {
            rx_overflow = true;  // Buffer overflow
        }
    }
}
#endif

/**
 * Get next byte from RX buffer (non-blocking)
 */
static bool get_rx_byte(uint8_t* byte) {
#if USE_USB_CDC
    // Read directly from USB CDC
    int c = getchar_timeout_us(0);  // Non-blocking
    if (c != PICO_ERROR_TIMEOUT) {
        *byte = (uint8_t)c;
        return true;
    }
    return false;
#else
    if (rx_head == rx_tail) {
        return false;  // Buffer empty
    }
    
    *byte = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1) % UART_BUFFER_SIZE;
    return true;
#endif
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
    
    while (get_rx_byte(&byte)) {
        switch (frame_state) {
            case 0:  // Waiting for start byte
                if (byte == START_BYTE) {
                    current_frame.start_byte = byte;
                    frame_state = 1;
                } else {
                    // Stay in state 0, discard invalid bytes
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
                    // Invalid length, reset parser
                    reset_frame_parser();
                } else if (expected_length == 0) {
                    // No payload, go directly to checksum
                    frame_state = 4;
                } else {
                    // Expect payload
                    frame_state = 3;
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
                    printf("Invalid frame: ID=0x%02X, checksum error\n", current_frame.msg_id);
                    send_ack(current_frame.msg_id, RESULT_CHECKSUM_ERR);
                }
                
                reset_frame_parser();
                break;
                
            default:
                // Should never reach here, reset parser
                reset_frame_parser();
                break;
        }
    }
}

/**
 * Send a message frame
 */
void send_message(uint8_t msg_id, const void* payload, uint8_t payload_len) {
    if (payload_len > MAX_PAYLOAD_SIZE) {
        return;  // Invalid payload size
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
    
#if USE_USB_CDC
    // Send via USB CDC
    putchar_raw(frame.start_byte);
    putchar_raw(frame.msg_id);
    putchar_raw(frame.length);
    
    for (uint8_t i = 0; i < payload_len; i++) {
        putchar_raw(frame.payload[i]);
    }
    
    putchar_raw(frame.checksum);
#else
    // Send via UART
    uart_putc_raw(protocol_uart, frame.start_byte);
    uart_putc_raw(protocol_uart, frame.msg_id);
    uart_putc_raw(protocol_uart, frame.length);
    
    for (uint8_t i = 0; i < payload_len; i++) {
        uart_putc_raw(protocol_uart, frame.payload[i]);
    }
    
    uart_putc_raw(protocol_uart, frame.checksum);
#endif
}

/**
 * Send ACK/NAK response
 */
void send_ack(uint8_t original_msg_id, uint8_t result) {
    ack_msg_t ack = {
        .original_msg_id = original_msg_id,
        .result = result
    };
    
    send_message(MSG_ACK_NAK, &ack, sizeof(ack));
}

/**
 * Send status report
 */
void send_status_report(void) {
    if (!motor_sys) return;
    
    status_msg_t status;
    
    // Get status for both axes
    motor_get_status(motor_sys, AXIS_RA, &status.ra_position, &status.ra_velocity, 
                    NULL, NULL, NULL, &status.ra_current);
    motor_get_status(motor_sys, AXIS_DEC, &status.dec_position, &status.dec_velocity,
                    NULL, NULL, NULL, &status.dec_current);
    
    // Get timing information
    status.ra_update_time = motor_get_axis_update_time(motor_sys, AXIS_RA);
    status.dec_update_time = motor_get_axis_update_time(motor_sys, AXIS_DEC);
    status.system_time = time_us_64();
    
    // Build status flags
    status.status_flags = 0;
    bool moving, limit, error;
    
    motor_get_status(motor_sys, AXIS_RA, NULL, NULL, &moving, &limit, &error, NULL);
    if (moving) status.status_flags |= STATUS_RA_MOVING;
    if (limit) status.status_flags |= STATUS_RA_LIMIT;
    if (error) status.status_flags |= STATUS_RA_ERROR;
    
    motor_get_status(motor_sys, AXIS_DEC, NULL, NULL, &moving, &limit, &error, NULL);
    if (moving) status.status_flags |= STATUS_DEC_MOVING;
    if (limit) status.status_flags |= STATUS_DEC_LIMIT;
    if (error) status.status_flags |= STATUS_DEC_ERROR;
    
    if (motor_sys->emergency_stop) status.status_flags |= STATUS_EMERGENCY;
    
    send_message(MSG_STATUS_REPORT, &status, sizeof(status));
}

/**
 * Handle received message
 */
void handle_received_message(const message_frame_t* frame) {
    switch (frame->msg_id) {
        case MSG_SET_VELOCITY:
            if (frame->length == sizeof(set_velocity_msg_t)) {
                handle_set_velocity((const set_velocity_msg_t*)frame->payload);
                send_ack(frame->msg_id, RESULT_SUCCESS);
            } else {
                send_ack(frame->msg_id, RESULT_INVALID_LEN);
            }
            break;
            
        case MSG_SET_POSITION:
            if (frame->length == sizeof(set_position_msg_t)) {
                handle_set_position((const set_position_msg_t*)frame->payload);
                send_ack(frame->msg_id, RESULT_SUCCESS);
            } else {
                send_ack(frame->msg_id, RESULT_INVALID_LEN);
            }
            break;
            
        case MSG_GET_STATUS:
            if (frame->length == 0) {
                handle_get_status();
                send_ack(frame->msg_id, RESULT_SUCCESS);
            } else {
                send_ack(frame->msg_id, RESULT_INVALID_LEN);
            }
            break;
            
        case MSG_EMERGENCY_STOP:
            if (frame->length == 0) {
                handle_emergency_stop();
                send_ack(frame->msg_id, RESULT_SUCCESS);
            } else {
                send_ack(frame->msg_id, RESULT_INVALID_LEN);
            }
            break;
            
        case MSG_SET_CORRECTION:
            if (frame->length == sizeof(correction_msg_t)) {
                handle_set_correction((const correction_msg_t*)frame->payload);
                send_ack(frame->msg_id, RESULT_SUCCESS);
            } else {
                send_ack(frame->msg_id, RESULT_INVALID_LEN);
            }
            break;
            
        case MSG_SET_PVT_TARGET:
            if (frame->length == sizeof(pvt_target_msg_t)) {
                handle_set_pvt_target((const pvt_target_msg_t*)frame->payload);
                send_ack(frame->msg_id, RESULT_SUCCESS);
            } else {
                send_ack(frame->msg_id, RESULT_INVALID_LEN);
            }
            break;
            
        default:
            printf("Unknown message ID: 0x%02X\n", frame->msg_id);
            send_ack(frame->msg_id, RESULT_INVALID_MSG);
            break;
    }
}

/**
 * Message handlers
 */
static void handle_set_velocity(const set_velocity_msg_t* msg) {
    if (!motor_sys) return;
    
    printf("Set velocity: RA=%d, DEC=%d\n", (int)msg->ra_velocity, (int)msg->dec_velocity);
    motor_set_velocity(motor_sys, AXIS_RA, msg->ra_velocity);
    motor_set_velocity(motor_sys, AXIS_DEC, msg->dec_velocity);
}

static void handle_set_position(const set_position_msg_t* msg) {
    if (!motor_sys) return;
    
    printf("Set position: RA=%d, DEC=%d, speed=%d\n", 
           (int)msg->ra_target, (int)msg->dec_target, msg->max_speed);
    motor_set_position(motor_sys, AXIS_RA, msg->ra_target, msg->max_speed);
    motor_set_position(motor_sys, AXIS_DEC, msg->dec_target, msg->max_speed);
}

static void handle_set_pvt_target(const pvt_target_msg_t* msg) {
    if (!motor_sys) return;
    
    if (msg->axis >= NUM_AXES) {
        return;  // Invalid axis
    }
    
    printf("Set PVT target: axis=%d, pos=%d, vel=%d, time=%llu\n", 
           msg->axis, (int)msg->target_position, (int)msg->target_velocity, msg->target_time);
    motor_set_pvt_target(motor_sys, msg->axis, msg->target_position, msg->target_velocity, msg->target_time);
}

static void handle_get_status(void) {
    // Status will be sent by periodic task
    printf("Status requested\n");
}

static void handle_emergency_stop(void) {
    if (!motor_sys) return;
    
    printf("Emergency stop triggered!\n");
    motor_emergency_stop(motor_sys);
}

static void handle_set_correction(const correction_msg_t* msg) {
    if (!motor_sys) return;
    
    if (msg->axis >= NUM_AXES) {
        return;  // Invalid axis
    }
    
    printf("Correction: axis=%d, steps=%d\n", msg->axis, msg->correction);
    motor_apply_correction(motor_sys, msg->axis, msg->correction);
}

/**
 * Main protocol task - call this regularly from main loop
 */
void protocol_task(void) {
    // Process any received bytes
    process_received_bytes();
    
    // Check for RX buffer overflow (UART mode only)
#if !USE_USB_CDC
    if (rx_overflow) {
        printf("UART RX buffer overflow!\n");
        rx_overflow = false;
        // Could send error status here
    }
#endif
    
    // Send periodic status reports
    absolute_time_t now = get_absolute_time();
    if (absolute_time_diff_us(last_status_time, now) >= STATUS_INTERVAL_US) {
        // send_status_report();
        last_status_time = now;
    }
}