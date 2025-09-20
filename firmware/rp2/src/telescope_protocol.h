#ifndef TELESCOPE_PROTOCOL_H
#define TELESCOPE_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#include "motor_control.h"

// Communication interface selection
// #define USE_USB_CDC         1   // 1 = USB CDC, 0 = UART0
#define USE_USB_CDC      0   // Uncomment and set to 0 for UART mode

// Protocol constants
#define START_BYTE          0xAA
#define MAX_PAYLOAD_SIZE    64
#define UART_BUFFER_SIZE    256
#define ACK_TIMEOUT_MS      50
#define STATUS_REPORT_HZ    20

// Message IDs - ESP32 → RP2040 Commands
// #define MSG_SET_VELOCITY    0x01
// #define MSG_SET_POSITION    0x02
#define MSG_GET_STATUS      0x03
#define MSG_EMERGENCY_STOP  0x04
// #define MSG_SET_CORRECTION  0x05
#define MSG_SET_PVT_TARGET  0x06  // Position/Velocity/Time target

// Message IDs - RP2040 → ESP32 Responses
#define MSG_STATUS_REPORT   0x81
#define MSG_ACK_NAK         0x82

// Status flags
#define STATUS_RA_MOVING    0x01
#define STATUS_DEC_MOVING   0x02
#define STATUS_RA_LIMIT     0x04
#define STATUS_DEC_LIMIT    0x08
#define STATUS_RA_ERROR     0x10
#define STATUS_DEC_ERROR    0x20
#define STATUS_EMERGENCY    0x40
#define STATUS_TMC_ERROR    0x80

// Error codes for ACK/NAK
#define RESULT_SUCCESS      0x00
#define RESULT_INVALID_MSG  0x01
#define RESULT_INVALID_LEN  0x02
#define RESULT_CHECKSUM_ERR 0x03
#define RESULT_AXIS_ERROR   0x04
#define RESULT_LIMIT_ERROR  0x05

// Message frame structure
typedef struct {
    uint8_t start_byte;
    uint8_t msg_id;
    uint8_t length;
    uint8_t payload[MAX_PAYLOAD_SIZE];
    uint8_t checksum;
} __attribute__((packed)) message_frame_t;

// Message payload structures
typedef struct {
    uint8_t axis;          // 0=RA, 1=DEC
    int32_t target_position;  // target step position
    float target_velocity;    // target velocity (steps/sec)
    uint64_t target_time;     // target time (microseconds since boot)
} __attribute__((packed)) pvt_target_msg_t;

typedef struct {
    int32_t ra_position;   // current step position
    int32_t dec_position;  // current step position
    float ra_velocity;     // current velocity
    float dec_velocity;    // current velocity
    uint64_t ra_update_time;  // last update time for RA axis (microseconds)
    uint64_t dec_update_time; // last update time for DEC axis (microseconds)
    uint64_t system_time;     // current system time (microseconds since boot)
    uint8_t status_flags;  // bit flags for errors, limits, etc.
    uint8_t padding;       // Alignment padding
    uint16_t ra_current;   // TMC2209 current reading (optional)
    uint16_t dec_current;  // TMC2209 current reading (optional)
} __attribute__((packed)) status_msg_t;

typedef struct {
    uint8_t original_msg_id;  // ID of message being acknowledged
    uint8_t result;           // 0=success, error codes otherwise
} __attribute__((packed)) ack_msg_t;

// Function prototypes
int protocol_init(void);
void protocol_task(void);
void protocol_set_motor_system(motor_system_t* sys);
uint8_t calculate_checksum(const uint8_t* data, uint8_t length);
bool validate_message(const message_frame_t* frame);
void send_message(uint8_t msg_id, const void* payload, uint8_t payload_len);
void send_ack(uint8_t original_msg_id, uint8_t result);
void send_status_report(void);
void handle_received_message(const message_frame_t* frame);

#endif // TELESCOPE_PROTOCOL_H