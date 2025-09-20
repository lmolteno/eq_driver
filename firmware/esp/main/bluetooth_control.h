#ifndef BLUETOOTH_CONTROL_H
#define BLUETOOTH_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// Gear reduction structure
typedef struct {
    float ra_gear_ratio;    // Steps per full rotation (360°) for RA axis
    float dec_gear_ratio;   // Steps per full rotation (360°) for DEC axis
} gear_config_t;

// Coordinate offsets for SYNC functionality
typedef struct {
    int32_t ra_offset_steps;   // RA offset in steps
    int32_t dec_offset_steps;  // DEC offset in steps
} coordinate_offsets_t;

// DMS (Degrees, Minutes, Seconds) structure
typedef struct {
    int16_t degrees;
    uint8_t minutes;
    uint8_t seconds;
    bool negative;
} dms_t;

// Tracking modes
typedef enum {
    TRACKING_OFF = 0,
    TRACKING_SIDEREAL = 1,
    TRACKING_SOLAR = 2,
    TRACKING_LUNAR = 3
} tracking_mode_t;

// Bluetooth control functions
esp_err_t bluetooth_control_init(void);
void bluetooth_task(void *pvParameters);

// Gear configuration functions
esp_err_t save_gear_config(const gear_config_t* config);
esp_err_t load_gear_config(gear_config_t* config);
esp_err_t set_ra_gear_ratio(float ratio);
esp_err_t set_dec_gear_ratio(float ratio);
float get_ra_gear_ratio(void);
float get_dec_gear_ratio(void);

// Coordinate conversion functions
void steps_to_ra_dms(int32_t steps, dms_t* dms);
void steps_to_dec_dms(int32_t steps, dms_t* dms);
int32_t ra_dms_to_steps(const dms_t* dms);
int32_t dec_dms_to_steps(const dms_t* dms);

// Sync and offset functions
void sync_coordinates(void);
void get_current_ra_dec(dms_t* ra, dms_t* dec);

// Jog functions
esp_err_t jog_ra_positive(void);
esp_err_t jog_ra_negative(void);
esp_err_t jog_dec_positive(void);
esp_err_t jog_dec_negative(void);

// Step target position functions
esp_err_t set_ra_step_target(int32_t target_position);
esp_err_t set_dec_step_target(int32_t target_position);

// Tracking functions
esp_err_t set_tracking_mode(tracking_mode_t mode);
tracking_mode_t get_tracking_mode(void);

// Command processing
void process_bluetooth_command(const char* command);
void send_bluetooth_response(const char* response);

#endif // BLUETOOTH_CONTROL_H