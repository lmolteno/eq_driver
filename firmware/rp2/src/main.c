#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "motor_control.h"
#include "telescope_protocol.h"

// Global system state
static motor_system_t motor_system;

int main() {
    // Initialize standard I/O
    stdio_init_all();
    
    printf("\\n=== RP2040 Telescope Mount Controller ===\\n");
    printf("Initializing system...\\n");
    
    // Get system clock frequency
    uint32_t sys_clock = clock_get_hz(clk_sys);
    printf("System clock: %u Hz\\n", (unsigned int)sys_clock);
    
    // Add delay for system stability
    sleep_ms(100);
    
    // Initialize motor control system
    int motor_init_result = motor_system_init(&motor_system, sys_clock);
    if (motor_init_result != 0) {
        printf("ERROR: Failed to initialize motor system! Code: %d\\n", motor_init_result);
        while (1) {
            sleep_ms(1000);
            printf("Motor system init failed - halted\\n");
        }
    }
    
    // Initialize communication protocol
    if (protocol_init() != 0) {
        printf("ERROR: Failed to initialize protocol system!\\n");
        while (1) {
            sleep_ms(1000);
            printf("Protocol init failed - halted\\n");
        }
    }

    sleep_ms(1000);


    // Link motor system to protocol handler
    protocol_set_motor_system(&motor_system);
    
    printf("System initialized successfully!\\n");
    printf("Waiting for ESP32 commands...\\n");
    
    // Main control loop
    absolute_time_t last_update = get_absolute_time();
    // const uint32_t UPDATE_INTERVAL_US = 10000;  // 100Hz update rate
    bool updated_pvt = false;
    
    while (1) {
        // Process protocol messages and send status reports
        protocol_task();
        
        // Update motor control system at regular intervals
        motor_update_task(&motor_system);

        // Small delay to prevent busy-waiting
        // sleep_us(1000);
        tight_loop_contents();

        if (get_absolute_time() > 8000000 && !updated_pvt)
        {
            // after 20 seconds
            motor_set_pvt_target(&motor_system, 0, -2000, 1000, 8000000);
            updated_pvt = true;
        }
    }
    
    return 0;
}
