#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/pio.h"
#include "hardware/uart.h"

// Motor axis definitions
#define AXIS_RA     0
#define AXIS_DEC    1
#define NUM_AXES    2

// PIO step generator configuration
typedef struct {
    PIO pio;
    uint sm;
    uint offset;
    uint step_pin;
    uint dir_pin;
    float clock_div;
} step_generator_t;

// Motor axis state
typedef struct {
    step_generator_t step_gen;
    int32_t position;           // Current step position
    int32_t velocity;           // Current velocity (steps/sec)
    int32_t target_position;    // Target position for goto moves
    int32_t target_velocity;    // Target velocity (steps/sec)
    uint64_t target_time;       // Target time (for propagating estimated position)
    uint16_t max_speed;         // Maximum speed for goto moves
    uint16_t current_reading;   // TMC2209 current reading
    int32_t p_gain;
    int32_t i_gain;
    int32_t d_gain;
    bool direction;             // Current direction (true = positive)
    bool moving;                // Is axis currently moving
    bool limit_triggered;       // Limit switch state
    bool error_state;           // Error condition
} motor_axis_t;

// System state
typedef struct {
    motor_axis_t axes[NUM_AXES];
    uart_inst_t* tmc_uart;
    bool emergency_stop;
    uint32_t system_clock_hz;
} motor_system_t;

// Function prototypes
int motor_system_init(motor_system_t* sys, uint32_t clock_hz);
void motor_set_velocity(motor_system_t* sys, uint8_t axis, int32_t velocity);
void motor_set_position(motor_system_t* sys, uint8_t axis, int32_t target, uint16_t max_speed);
void motor_set_pvt_target(motor_system_t* sys, uint8_t axis, int32_t target_pos, int32_t target_vel, uint64_t target_time);
void motor_apply_correction(motor_system_t* sys, uint8_t axis, int16_t correction);
void motor_emergency_stop(motor_system_t* sys);
void motor_update_task(motor_system_t* sys);
void motor_get_status(motor_system_t* sys, uint8_t axis, 
                     int32_t* position, int32_t* velocity, bool* moving, 
                     bool* limit, bool* error, uint16_t* current);
uint64_t motor_get_axis_update_time(motor_system_t* sys, uint8_t axis);

// Internal functions
void setup_step_generator(step_generator_t* gen, PIO pio, uint step_pin, uint dir_pin, float sys_clock, uint program_offset);
void update_step_rate(step_generator_t* gen, int32_t velocity, float sys_clock);
void step_isr_handler(void);

// TMC2209 helper functions  
void set_velocity(int32_t velocity, uint8_t motor);
uint32_t generate_chopconf(uint8_t diss2vs, uint8_t diss2g, uint8_t dedge, uint8_t intpol, 
                          uint8_t mres, uint8_t vsense, uint8_t tbl, uint8_t hend, 
                          uint8_t hstrt, uint8_t toff);
uint32_t ihold_irun(uint8_t ihold, uint8_t irun, uint8_t iholddelay);
void init(void);

#endif // MOTOR_CONTROL_H
