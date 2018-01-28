//
// Created by atmelfan on 2018-01-28.
//

#ifndef SENSOR_MODULES_ASPI_H
#define SENSOR_MODULES_ASPI_H

#include <stdint.h>

#define ASPI_IS_CHAIN(addr) (addr & (1 << 7))
#define ASPI_IS_DIRECT(addr) (!(addr & (1 << 7)))

#define ASPI_IS_READ(addr) (addr & (1 << 6))

#define ASPI_NUM_ADDR(this, addr) ((this) >= (addr) ? (this) - (addr) + 1 : 0)

#define ASPI_USER_HEADER 0
#define ASPI_USER_HEADER_SIZE 1

typedef enum {
    ASPI_ERR_SUCCESS = 0,
    ASPI_ERR_HOST,
    ASPI_ERR_SLAVE
} aspi_error_t;

typedef enum {
    ASPI_STAT_FREE = 0, /*No active transaction*/
    ASPI_STAT_HEADER, /*CS selected, waiting for header*/

    /**Direct*/
    ASPI_STAT_DIRECT, /*Direct transaction in progress*/

    /**Chain*/
    ASPI_STAT_CHAIN_LEN, /*Waiting for transaction length*/
    ASPI_STAT_CHAIN_WAIT, /*Waiting for my time to shine*/
    ASPI_STAT_CHAIN, /*Chain transaction in progress*/
    ASPI_STAT_CHAIN_DONE /*Chain transaction complete*/
} aspi_status;

/**
 * ASPI Handler functions
 */

/**
 * RX Handler function, called for each received byte (data)
 * @param data Byte received
 * @return Data to be sent back next transfer, last byte ignored.
 */
extern uint8_t aspi_rx_handler(uint8_t data, uint8_t i);
#define ASPI_RX(name1, name2) uint8_t aspi_rx_handler(uint8_t name1, uint8_t name2)

/**
 * Start handler function, called when CS is selected and header received
 * @param addr, header address received
 * @param chain, length of transfer (if chain)
 * @return 0 to reject, 1-255 to accept (if chain, will wait for (N-1)*len bytes)
 */
extern uint8_t aspi_start_handler(uint8_t addr, uint8_t chain);
#define ASPI_START(name1, name2) uint8_t aspi_start_handler(uint8_t name1, uint8_t name2)

/**
 * End handler, called when transaction has completed
 * @param status, Status of transaction:
 * - ASPI_ERR_SUCCESS if transaction was successfull
 * - ASPI_ERR_HOST if CS was deselected before transaction was completed (chain mode only)
 * - ASPI_ERR_SLAVE if aspi_error() was called during transfer
 */
extern void aspi_end_handler(aspi_error_t status);
#define ASPI_END(name) void aspi_end_handler(aspi_error_t name)

/**
 * ASPI Platform functions
 */

/**
 * Call this on select
 */
void aspi_comm_select();

/**
 * Call this on deselect
 */
void aspi_comm_deselect();

/**
 * Call this on RX/TX
 * @param d, data received
 * @param t, data to transmit
 * @return 0=Do not drive, 1=Send
 */
uint8_t aspi_comm_rxtx(uint8_t d, uint8_t *t);

/**
 * ASPI API functions
 */

/**
 * Returns the current ASPI status
 * @return
 */
aspi_status aspi_current_status();

/**
 * Returns transaction length
 * @return N if in chain mode, 255 otherwise
 */
uint8_t aspi_get_length();

/**
 * Returns active address
 * @return addr
 */
uint8_t aspi_get_addr();

/**
 * Sets error flag to err and stops transaction
 * @param err, error flag
 */
void aspi_throw(uint8_t err);

/**
 * Get last error flag set by aspi_throw
 * @return Error flag
 */
uint8_t aspi_get_error();


#endif //SENSOR_MODULES_ASPI_H
