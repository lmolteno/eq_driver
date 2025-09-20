#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/uart.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "motor_control.h"
#include "tmc2209.h"
#include "step_generator.pio.h"

#define INTEGRAL_MAX 100000
#define INTEGRAL_MIN (-100000)
#define MAX_ACCELERATION 8000 // steps per second per second
#define STARTUP_VELOCITY 500
#define MAX_VELOCITY 15000

// Global motor system state
static motor_system_t *g_motor_system = NULL;

// PIO state
static PIO step_pio = pio0;
static uint step_program_offset;

// TMC2209 UART
uart_inst_t *motor_uart = uart0;

// Position tracking and FIFO management
static volatile int32_t step_counts[NUM_AXES] = {0, 0};
static volatile uint16_t current_step_delay[NUM_AXES] = {0xFFFF, 0xFFFF}; // Current delay for each axis
static volatile bool current_direction[NUM_AXES] = {true, true}; // Current direction for each axis
static volatile double previous_error[NUM_AXES] = {0, 0};
static volatile double_t error_integral[NUM_AXES] = {0, 0};
static volatile uint64_t previous_update_time[NUM_AXES] = {0, 0};

// Internal function prototypes
static void configure_tmc2209(motor_system_t *sys, uint8_t axis);

static void feed_step_fifo(motor_system_t *sys, uint8_t axis);

static void pio_step_irq_handler(void);

static uint16_t velocity_to_step_delay(float velocity_steps_per_sec, float pio_freq);

/**
 * Initialize the motor control system
 */
int motor_system_init(motor_system_t *sys, uint32_t clock_hz) {
    if (!sys) return -1;

    memset(sys, 0, sizeof(motor_system_t));
    sys->system_clock_hz = clock_hz;
    sys->tmc_uart = motor_uart;

    // Store global reference
    g_motor_system = sys;

    // Initialize TMC2209 UART
    tmc2209_config_t tmc_config = {
        .uart = motor_uart,
        .tx_pin = 0,
        .rx_pin = 1,
        .baudrate = 115200
    };

    printf("Initializing TMC2209 UART...\n");
    int tmc_result = tmc2209_init(&tmc_config);
    if (tmc_result != 0) {
        printf("Failed to initialize TMC2209 UART, code: %d\n", tmc_result);
        return -1;
    }
    printf("TMC2209 UART initialized successfully\n");

    // Load PIO program once
    step_program_offset = pio_add_program(step_pio, &step_generator_program);

    // Initialize step generators for both axes using same PIO, different state machines
    // RA axis: Step=GP6, Dir=GP7, PIO0 SM0
    sys->axes[AXIS_RA].step_gen.sm = 0;
    setup_step_generator(&sys->axes[AXIS_RA].step_gen, step_pio, 6, 7, clock_hz, step_program_offset);

    // DEC axis: Step=GP12, Dir=GP13, PIO0 SM1
    sys->axes[AXIS_DEC].step_gen.sm = 1;
    setup_step_generator(&sys->axes[AXIS_DEC].step_gen, step_pio, 12, 13, clock_hz, step_program_offset);

    // Configure TMC2209 drivers
    configure_tmc2209(sys, AXIS_RA);
    configure_tmc2209(sys, AXIS_DEC);

    for (int i = 0; i < NUM_AXES; i++) {
        sys->axes[i].p_gain = 2000;
        sys->axes[i].i_gain = 500;
        sys->axes[i].d_gain = 800;
        sys->axes[i].velocity = 0;
    }

    printf("Motor system initialized with %d axes\n", NUM_AXES);
    return 0;
}

/**
 * Set up a step generator PIO state machine
 */
void setup_step_generator(step_generator_t *gen, PIO pio, uint step_pin, uint dir_pin, float sys_clock,
                          uint program_offset) {
    gen->pio = pio;
    gen->step_pin = step_pin;
    gen->dir_pin = dir_pin;
    gen->offset = program_offset;

    // Configure step pin for PIO control
    pio_gpio_init(pio, step_pin);

    // Configure direction pin for regular GPIO control
    gpio_init(dir_pin);
    gpio_set_dir(dir_pin, GPIO_OUT);
    gpio_put(dir_pin, false); // Default direction

    // Configure state machine
    pio_sm_config c = step_generator_program_get_default_config(gen->offset);

    // Set pins: SET for step pulse output
    sm_config_set_set_pins(&c, step_pin, 1);

    // Set GPIO direction for step pin
    pio_sm_set_consecutive_pindirs(pio, gen->sm, step_pin, 1, true);

    // Set clock divider for precise timing (target ~10MHz for 100ns resolution)
    float target_freq = 10000000.0f; // 10MHz
    gen->clock_div = sys_clock / target_freq;
    if (gen->clock_div < 1.0f) gen->clock_div = 1.0f;
    sm_config_set_clkdiv(&c, gen->clock_div);

    // Initialize and start state machine
    pio_sm_init(pio, gen->sm, gen->offset, &c);
    pio_sm_set_enabled(pio, gen->sm, true);

    printf("PIO%d SM%d: step_pin=%d, dir_pin=%d, clock_div=%.2f\\n",
           pio_get_index(pio), gen->sm, step_pin, dir_pin, gen->clock_div);
}

/**
 * Configure TMC2209 driver for telescope application
 */
static void configure_tmc2209(motor_system_t *sys, uint8_t axis) {
    if (axis >= NUM_AXES) return;

    // Configure GCONF register
    uint32_t gconf = 0x000001C0; // Enable UART, set PDN_UART, set MSTEP_REG_SELECT
    tmc2209_write_register(sys->tmc_uart, axis, 0x00, gconf);

    // Configure CHOPCONF for smooth operation
    uint32_t chopconf = generate_chopconf(
        0, // diss2vs
        0, // diss2g
        0, // dedge
        1, // intpol (enable interpolation)
        4, // mres (16 microsteps)
        0, // vsense
        2, // tbl
        0, // hend
        5, // hstrt
        3 // toff
    );
    tmc2209_write_register(sys->tmc_uart, axis, 0x6C, chopconf);

    // Set current levels (adjust for your motors)
    uint32_t ihold_irun_val = ihold_irun(
        8, // ihold (hold current)
        16, // irun (run current)
        10 // iholddelay
    );
    tmc2209_write_register(sys->tmc_uart, axis, 0x10, ihold_irun_val);

    printf("TMC2209 axis %d configured\n", axis);
}

/**
 * Set velocity for an axis
 */
void motor_set_velocity(motor_system_t *sys, uint8_t axis, float velocity) {
    if (!sys || axis >= NUM_AXES) return;

    motor_axis_t *ax = &sys->axes[axis];
    ax->velocity = velocity;
    ax->direction = (velocity >= 0);

    // Update PIO step generator
    update_step_rate(&ax->step_gen, velocity, sys->system_clock_hz);
}

/**
 * Set target position for goto move
 */
void motor_set_position(motor_system_t *sys, uint8_t axis, int32_t target, uint16_t max_speed) {
    if (!sys || axis >= NUM_AXES) return;

    motor_axis_t *ax = &sys->axes[axis];
    ax->max_speed = max_speed;

    // For now, implement simple move - could add acceleration profiles later
    int32_t position_diff = target - ax->position;
    if (position_diff != 0) {
        float move_velocity = (position_diff > 0) ? max_speed : -max_speed;
        motor_set_velocity(sys, axis, move_velocity);
        printf("Axis %d goto: current=%d, target=%d, velocity=%d\n",
               axis, (int) ax->position, (int) target, move_velocity);
    }
}

/**
 * Set position/velocity/time target for PID control
 */
void motor_set_pvt_target(motor_system_t *sys, uint8_t axis, int32_t target_pos, float target_vel,
                          uint64_t target_time) {
    if (!sys || axis >= NUM_AXES) return;

    motor_axis_t *ax = &sys->axes[axis];
    ax->target_position = target_pos;
    ax->target_velocity = target_vel;
    ax->target_time = target_time;
    ax->moving = true;

    // Reset PID state
    error_integral[axis] = 0;
    previous_error[axis] = 0;
    previous_update_time[axis] = time_us_64();

    printf("Axis %d PVT target: pos=%d, vel=%d, time=%llu\n",
           axis, (int) target_pos, target_vel, target_time);
}

/**
 * Apply periodic error correction
 */
void motor_apply_correction(motor_system_t *sys, uint8_t axis, int16_t correction) {
    if (!sys || axis >= NUM_AXES) return;

    // Apply correction as velocity adjustment
    motor_axis_t *ax = &sys->axes[axis];
    float corrected_velocity = ax->target_velocity + correction;

    motor_set_velocity(sys, axis, corrected_velocity);
    printf("Axis %d correction: %d steps, new velocity: %d\n",
           axis, correction, corrected_velocity);
}

/**
 * Emergency stop - halt all motion immediately
 */
void motor_emergency_stop(motor_system_t *sys) {
    if (!sys) return;

    sys->emergency_stop = true;

    // Stop both axes immediately
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        motor_set_velocity(sys, i, 0);
        sys->axes[i].moving = false;
        sys->axes[i].error_state = true;
    }

    printf("EMERGENCY STOP activated!\n");
}

/**
 * Update step rate for PIO generator
 */
void update_step_rate(step_generator_t *gen, float velocity, float sys_clock) {
    bool direction = (velocity >= 0);

    // Set direction via GPIO
    gpio_put(gen->dir_pin, direction);

    // Calculate step delay for new velocity
    float pio_freq = sys_clock / gen->clock_div;
    uint16_t step_delay = velocity_to_step_delay(fabsf(velocity), pio_freq);

    // Update current delay and direction for this axis
    if (gen->sm == 0) {
        current_step_delay[AXIS_RA] = step_delay;
        current_direction[AXIS_RA] = direction;
    } else {
        current_step_delay[AXIS_DEC] = step_delay;
        current_direction[AXIS_DEC] = direction;
    }
}

/**
 * Convert velocity to step delay in 30-cycle units
 */
static uint16_t velocity_to_step_delay(float velocity_steps_per_sec, float pio_freq) {
    if (velocity_steps_per_sec <= 0) return 0xFFFF; // Max delay (stopped)

    return calculate_step_delay(velocity_steps_per_sec, pio_freq);
}

/**
 * Get motor axis status
 */
void motor_get_status(motor_system_t *sys, uint8_t axis,
                      int32_t *position, float *velocity, bool *moving,
                      bool *limit, bool *error, uint16_t *current) {
    if (!sys || axis >= NUM_AXES) return;

    motor_axis_t *ax = &sys->axes[axis];

    if (!ax) return;

    if (position) *position = ax->position;
    if (velocity) *velocity = ax->velocity;
    if (moving) *moving = ax->moving;
    if (limit) *limit = ax->limit_triggered;
    if (error) *error = ax->error_state;
    if (current) *current = ax->current_reading;
}

/**
 * Get axis update time
 */
uint64_t motor_get_axis_update_time(motor_system_t *sys, uint8_t axis) {
    if (!sys || axis >= NUM_AXES) return 0;
    return previous_update_time[axis];
}

/**
 * Feed step delays to PIO FIFO to keep it running
 */
static void feed_step_fifo(motor_system_t *sys, uint8_t axis) {
    if (!sys || axis >= NUM_AXES) return;

    motor_axis_t *ax = &sys->axes[axis];
    step_generator_t *gen = &ax->step_gen;

    // pio_sm_clear_fifos(gen->pio, gen->sm);

    uint16_t delay = current_step_delay[axis];

    // Calculate how many delays to queue
    int queue_depth;
    if (delay > 1000) {
        queue_depth = 1; // Slow movement: minimal buffering
    } else if (delay > 100) {
        queue_depth = 2; // Medium speed: small buffer
    } else {
        queue_depth = 4; // Fast movement: larger buffer
    }
    bool filled = false;

    // Keep FIFO full with current step delay
    while (pio_sm_get_tx_fifo_level(gen->pio, gen->sm) < queue_depth) {
        // (!pio_sm_is_tx_fifo_full(gen->pio, gen->sm)) {
        filled = true;
        if (delay == 0xFFFF) {
            return;
        }

        // Pack two delays into 32-bit word (each step gets same delay)
        uint32_t fifo_word = pack_step_delays(delay, delay);
        pio_sm_put(gen->pio, gen->sm, fifo_word);

        // Count the steps we're about to generate
        if (delay != 0xFFFF) {
            if (current_direction[axis]) {
                step_counts[axis] += 2; // Two steps per FIFO word
            } else {
                step_counts[axis] -= 2;
            }
        }
    }

    // if (!filled)
    // {
    //     printf("called fill fifo without any work to do\r\n");
    // }

    return;
}

/**
 * Motor update task - call periodically from main loop
 */
void motor_update_task(motor_system_t *sys) {
    if (!sys) return;

    // Feed step delays to PIO FIFOs to keep them running
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        feed_step_fifo(sys, i);
    }

    // Update position counters
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        sys->axes[i].position = step_counts[i];
    }

    // pid loop
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        // handle startup.
        if (previous_update_time[i] == 0) {
            previous_update_time[i] = time_us_64();
            continue;
        }
        motor_axis_t *ax = &sys->axes[i];
        uint64_t current_time = time_us_64();

        double time_since_target = (current_time - ax->target_time) / 1000000.0;
        double target_position = ax->target_position + ax->target_velocity * time_since_target;
        double delta_t = (current_time - previous_update_time[i]) / 1000000.0;

        double pos_err = target_position - ax->position;


        if (fabs(pos_err) < 800) {
            error_integral[i] += (pos_err * delta_t);
        } else {
            // Optional: slowly decay integral during large errors
            error_integral[i] *= 0.95; // or set to 0
        }

        if (error_integral[i] > INTEGRAL_MAX) error_integral[i] = INTEGRAL_MAX;
        if (error_integral[i] < INTEGRAL_MIN) error_integral[i] = INTEGRAL_MIN;

        double derivative_error = 0;

        if (previous_error[i] != 0) {
            derivative_error = (pos_err - previous_error[i]) / delta_t;
        }

        // feedforward target_velocity into PID output
        float new_vel = ax->target_velocity + ((ax->p_gain * pos_err) + (ax->i_gain * error_integral[i]) + (
                                                   ax->d_gain * derivative_error)) / 1000.0;
        if (new_vel > 100000) new_vel = 100000;
        if (new_vel < -100000) new_vel = -100000;


        // if (time_since_target > 5.0)
        // {
        //     int32_t debugging = 100;
        //     printf("debugging\r\n");
        // }

        float vel_change = new_vel - ax->velocity;
        float max_change = (MAX_ACCELERATION * delta_t);
        if (ax->velocity == 0 && fabsf(vel_change) > STARTUP_VELOCITY) {
            new_vel = (vel_change > 0 ? STARTUP_VELOCITY : -STARTUP_VELOCITY);
        } else if (fabsf(vel_change) > max_change) {
            new_vel = ax->velocity + (vel_change > 0 ? max_change : -max_change);
        }

        if (fabsf(new_vel) > MAX_VELOCITY) {
            new_vel = new_vel > 0 ? MAX_VELOCITY : -MAX_VELOCITY;
        }

        motor_set_velocity(g_motor_system, i, new_vel);

        previous_error[i] = pos_err;
        previous_update_time[i] = current_time;
    }

    // printf("RA Position: %d, velocity: %d\n\r", sys->axes[0].position, sys->axes[0].velocity);

    // TODO: Read TMC2209 status periodically
    // TODO: Check limit switches
    // TODO: Monitor for stalls/errors
}

// Legacy TMC2209 helper functions (kept for compatibility)
void set_velocity(float velocity, uint8_t motor) {
    if (g_motor_system) {
        motor_set_velocity(g_motor_system, motor, velocity);
    }
}

uint32_t generate_chopconf(
    uint8_t diss2vs, uint8_t diss2g, uint8_t dedge, uint8_t intpol,
    uint8_t mres, uint8_t vsense, uint8_t tbl, uint8_t hend,
    uint8_t hstrt, uint8_t toff
) {
    // Validate input ranges
    if (diss2vs > 1) diss2vs = 1;
    if (diss2g > 1) diss2g = 1;
    if (dedge > 1) dedge = 1;
    if (intpol > 1) intpol = 1;
    if (mres > 15) mres = 15;
    if (vsense > 1) vsense = 1;
    if (tbl > 3) tbl = 3;
    if (hend > 15) hend = 15;
    if (hstrt > 7) hstrt = 7;
    if (toff > 15) toff = 15;

    uint32_t result = 0;
    result |= ((uint32_t) (diss2vs & 0x01) << 31);
    result |= ((uint32_t) (diss2g & 0x01) << 30);
    result |= ((uint32_t) (dedge & 0x01) << 29);
    result |= ((uint32_t) (intpol & 0x01) << 28);
    result |= ((uint32_t) (mres & 0x0F) << 24);
    result |= ((uint32_t) (vsense & 0x01) << 17);
    result |= ((uint32_t) (tbl & 0x03) << 15);
    result |= ((uint32_t) (hend & 0x0F) << 7);
    result |= ((uint32_t) (hstrt & 0x07) << 4);
    result |= (toff & 0x0F);
    return result;
}

uint32_t ihold_irun(uint8_t ihold, uint8_t irun, uint8_t iholddelay) {
    if (ihold > 31) ihold = 31;
    if (irun > 31) irun = 31;
    if (iholddelay > 15) iholddelay = 15;

    uint32_t result = 0;
    result |= (ihold & 0x1F);
    result |= ((irun & 0x1F) << 8);
    result |= ((iholddelay & 0x0F) << 16);
    return result;
}

void init() {
    // This function is kept for compatibility but motor_system_init should be used instead
    printf("Warning: init() is deprecated, use motor_system_init()\n");
}
