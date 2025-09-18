#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/sem.h"
#include "pico/mutex.h"
#include "hardware/uart.h"
#include "tmc2209.h"

// TMC2209 UART constants
#define TMC2209_SYNC_NIBBLE     0x05    // 0101 sync pattern
#define TMC2209_MASTER_ADDR     0xFF    // Master address for read replies
#define TMC2209_WRITE_FLAG      0x80    // Bit 7 set for write access
#define TMC2209_READ_FLAG       0x00    // Bit 7 clear for read access

// Datagram sizes
#define TMC2209_WRITE_DATAGRAM_SIZE  8   // bytes
#define TMC2209_READ_REQUEST_SIZE    4   // bytes
#define TMC2209_READ_REPLY_SIZE      8   // bytes

// Timing constants
#define TMC2209_REPLY_TIMEOUT_MS    50   // Timeout for read replies
#define TMC2209_BYTE_TIMEOUT_MS     10   // Timeout between bytes

// RX buffer management
#define RX_BUFFER_SIZE 32
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static volatile size_t rx_index = 0;

// Synchronization
static semaphore_t rx_sem;
static mutex_t uart_mutex;
static bool initialized = false;

// Function prototypes
void tmc_uart_crc(uint8_t *data, uint8_t data_len);

void tmc_uart_crc(uint8_t *data, uint8_t data_len) {
    int i, j;
    uint8_t *crc = data + (data_len - 1); // CRC located in last byte of message
    uint8_t currentByte;
    *crc = 0;
    for (i = 0; i < (data_len - 1); i++) { // Execute for all bytes of a message
        currentByte = data[i]; // Retrieve a byte to be sent from Array
        for (j = 0; j < 8; j++) {
            if ((*crc >> 7) ^ (currentByte & 0x01)) // update CRC based result of XOR operation
                *crc = (*crc << 1) ^ 0x07;
            else
                *crc = (*crc << 1);
            currentByte = currentByte >> 1;
        } // for CRC bit
    } // for message byte
}

/**
 * Initialize UART for TMC2209 communication
 *
 * @param config Configuration structure with UART settings
 * @return 0 on success, negative error code on failure
 */
int tmc2209_init(tmc2209_config_t *config)
{
    if (!config || !config->uart) {
        return -1;
    }

    // Initialize synchronization primitives
    if (!initialized) {
        sem_init(&rx_sem, 0, 1);
        mutex_init(&uart_mutex);
        initialized = true;
    }

    // Initialize UART
    uart_init(config->uart, config->baudrate);

    // Set up GPIO pins
    gpio_set_function(config->tx_pin, GPIO_FUNC_UART);
    gpio_set_function(config->rx_pin, GPIO_FUNC_UART);

    // Configure UART format: 8 data bits, 1 stop bit, no parity
    uart_set_format(config->uart, 8, 1, UART_PARITY_NONE);

    // Enable FIFO
    uart_set_fifo_enabled(config->uart, true);

    printf("TMC2209 UART initialized on pins TX=%d, RX=%d at %d baud\n", 
           config->tx_pin, config->rx_pin, config->baudrate);
    return 0;
}

/**
 * Clear/flush RX buffer of any stale data
 */
static void flush_rx_buffer(uart_inst_t *uart)
{
    uint8_t dummy;
    int count = 0;

    // Read out any stale bytes
    while (uart_is_readable(uart) && count < 32) {
        count++;
    }

    // Reset our buffer state
    rx_index = 0;
}

/**
 * Wait for TX transmission to complete
 */
static void wait_tx_complete(uart_inst_t *uart)
{
    uart_tx_wait_blocking(uart);
}

/**
 * Write a 32-bit value to a TMC2209 register
 *
 * @param uart      UART instance
 * @param node_addr Node address (0-3, set by MS1/MS2 pins)
 * @param reg_addr  Register address (0-127)
 * @param data      32-bit data to write
 * @return 0 on success, negative error code on failure
 */
int tmc2209_write_register(uart_inst_t *uart, uint8_t node_addr,
                          uint8_t reg_addr, uint32_t data)
{
    uint8_t datagram[TMC2209_WRITE_DATAGRAM_SIZE];

    if (!uart) {
        return -1;
    }

    if (node_addr > 3) {
        printf("Invalid node address: %d (must be 0-3)\n", node_addr);
        return -1;
    }

    if (reg_addr > 0x7F) {
        printf("Invalid register address: 0x%02X (must be 0-127)\n", reg_addr);
        return -1;
    }

    mutex_enter_blocking(&uart_mutex);

    // Clear any stale RX data first
    flush_rx_buffer(uart);

    // Build write datagram according to specification
    // Byte 0: Sync nibble (0101) + reserved nibble (included in CRC)
    datagram[0] = TMC2209_SYNC_NIBBLE;

    // Byte 1: Node address (bits 0-1), reserved bits 2-7
    datagram[1] = node_addr & 0x03;

    // Byte 2: Write flag (bit 7) + register address (bits 0-6)
    datagram[2] = TMC2209_WRITE_FLAG | (reg_addr & 0x7F);

    // Bytes 3-6: 32-bit data, highest byte first
    datagram[3] = (data >> 24) & 0xFF;  // Data byte 3 (MSB)
    datagram[4] = (data >> 16) & 0xFF;  // Data byte 2
    datagram[5] = (data >> 8) & 0xFF;   // Data byte 1
    datagram[6] = data & 0xFF;          // Data byte 0 (LSB)

    // Byte 7: CRC (calculated by tmc_uart_crc)
    datagram[7] = 0x00;  // Will be filled by CRC function

    // Calculate CRC
    tmc_uart_crc(datagram, TMC2209_WRITE_DATAGRAM_SIZE);

    printf("Write reg 0x%02X to node %d: 0x%08X\n", reg_addr, node_addr, (unsigned int)data);

    // Transmit datagram
    for (int i = 0; i < TMC2209_WRITE_DATAGRAM_SIZE; i++) {
        uart_putc_raw(uart, datagram[i]);
    }

    // Wait for transmission to complete
    wait_tx_complete(uart);

    mutex_exit(&uart_mutex);
    return 0;
}

/**
 * Read a 32-bit value from a TMC2209 register
 *
 * @param uart      UART instance
 * @param node_addr Node address (0-3, set by MS1/MS2 pins)
 * @param reg_addr  Register address (0-127)
 * @param data      Pointer to store the read 32-bit data
 * @return 0 on success, negative error code on failure
 */
int tmc2209_read_register(uart_inst_t *uart, uint8_t node_addr,
                         uint8_t reg_addr, uint32_t *data)
{
    uint8_t request[TMC2209_READ_REQUEST_SIZE];
    absolute_time_t timeout;
    const uint32_t ECHO_DELAY_MAX_US = 5000;
    const uint32_t REPLY_DELAY_MAX_US = 50000; // 50ms timeout

    if (!uart || !data) {
        return -1;
    }

    if (node_addr > 3) {
        printf("Invalid node address: %d (must be 0-3)\n", node_addr);
        return -1;
    }

    if (reg_addr > 0x7F) {
        printf("Invalid register address: 0x%02X (must be 0-127)\n", reg_addr);
        return -1;
    }

    mutex_enter_blocking(&uart_mutex);

    // Build read request datagram
    // Byte 0: Sync nibble (0101) + reserved nibble
    request[0] = TMC2209_SYNC_NIBBLE;

    // Byte 1: Node address (bits 0-1), reserved bits 2-7
    request[1] = node_addr & 0x03;

    // Byte 2: Read flag (bit 7 clear) + register address (bits 0-6)
    request[2] = TMC2209_READ_FLAG | (reg_addr & 0x7F);

    // Byte 3: CRC
    request[3] = 0x00;  // Will be filled by CRC function

    // Calculate CRC for request
    tmc_uart_crc(request, TMC2209_READ_REQUEST_SIZE);

    printf("Read reg 0x%02X from node %d\n", reg_addr, node_addr);

    // Clear RX buffer before starting
    flush_rx_buffer(uart);

    // Transmit read request
    for (int i = 0; i < TMC2209_READ_REQUEST_SIZE; i++) {
        uart_putc_raw(uart, request[i]);
    }

    // Wait for transmission to complete
    wait_tx_complete(uart);

    // Wait for echo and discard it
    timeout = make_timeout_time_us(ECHO_DELAY_MAX_US);
    rx_index = 0;
    while (rx_index < TMC2209_READ_REQUEST_SIZE && !time_reached(timeout)) {
        if (uart_is_readable(uart)) {
            uint8_t byte = uart_getc(uart);
            printf("Discarding echo byte: 0x%02X\n", byte);
            rx_index++;
        } else {
            sleep_us(100);
        }
    }

    // Now wait for the actual reply
    rx_index = 0;
    timeout = make_timeout_time_us(REPLY_DELAY_MAX_US);

    while (rx_index < TMC2209_READ_REPLY_SIZE && !time_reached(timeout)) {
        if (uart_is_readable(uart)) {
            rx_buffer[rx_index++] = uart_getc(uart);
        } else {
            sleep_us(1000);
        }
    }

    if (rx_index < TMC2209_READ_REPLY_SIZE) {
        printf("Incomplete reply received: %zu/%d bytes (timeout)\n",
               rx_index, TMC2209_READ_REPLY_SIZE);
        mutex_exit(&uart_mutex);
        return -2; // Timeout error
    }

    // Verify reply structure
    if (rx_buffer[1] != TMC2209_MASTER_ADDR) {
        printf("Invalid master address in reply: 0x%02X (expected 0xFF)\n", rx_buffer[1]);
        mutex_exit(&uart_mutex);
        return -3; // Bad message error
    }

    if ((rx_buffer[2] & 0x7F) != reg_addr) {
        printf("Register address mismatch: got 0x%02X, expected 0x%02X\n",
               rx_buffer[2] & 0x7F, reg_addr);
        mutex_exit(&uart_mutex);
        return -3; // Bad message error
    }

    // Verify CRC
    uint8_t expected_crc = rx_buffer[7];
    uint8_t reply_copy[TMC2209_READ_REPLY_SIZE];
    memcpy(reply_copy, rx_buffer, TMC2209_READ_REPLY_SIZE);
    tmc_uart_crc(reply_copy, TMC2209_READ_REPLY_SIZE);

    if (reply_copy[7] != expected_crc) {
        printf("CRC mismatch: got 0x%02X, calculated 0x%02X\n",
               expected_crc, reply_copy[7]);
        mutex_exit(&uart_mutex);
        return -3; // Bad message error
    }

    // Extract 32-bit data (highest byte first)
    *data = ((uint32_t)rx_buffer[3] << 24) |
            ((uint32_t)rx_buffer[4] << 16) |
            ((uint32_t)rx_buffer[5] << 8) |
            ((uint32_t)rx_buffer[6]);

    printf("Read successful: 0x%08X\n", (unsigned int)*data);

    mutex_exit(&uart_mutex);
    return 0;
}