//
// Created by atmelfan on 2018-01-28.
//

#include "aspi.h"

aspi_status current_state = ASPI_STAT_FREE;
uint8_t len, addr, err, count;

int wait;

void aspi_comm_select() {
    current_state = ASPI_STAT_HEADER;
}

void aspi_comm_deselect() {
    if(current_state == ASPI_STAT_FREE || current_state == ASPI_STAT_HEADER){
        /*Do nothing if deselected without previously selected/or received header*/
    }else{
        /*Call end handler*/
        if(current_state == ASPI_STAT_DIRECT || current_state == ASPI_STAT_CHAIN_DONE){
            aspi_end_handler(ASPI_ERR_SUCCESS);
        }else{
            aspi_end_handler(ASPI_ERR_HOST);
        }

    }
    /*Reset state*/
    current_state = ASPI_STAT_FREE;
    count = 0;
}

uint8_t aspi_comm_rxtx(uint8_t d, uint8_t *t) {
    switch(current_state){
        /** Handle header data*/
        case ASPI_STAT_HEADER:
            addr = d;
            if(ASPI_IS_CHAIN(d)){
                current_state = ASPI_STAT_CHAIN_LEN;
            }else{
                //Call start handler
                uint8_t w = aspi_start_handler(addr, 255);
                //Reset if start handler does not recognise address
                if(w == 0){
                    current_state = ASPI_STAT_FREE;
                }else{
                    current_state = ASPI_STAT_DIRECT;
                }
            }
            break;


        /** Handle direct transaction*/
        case ASPI_STAT_DIRECT:
            *t = aspi_rx_handler(d, count++);
            break;

        /** Handle chain transaction*/
        case ASPI_STAT_CHAIN_LEN:
            len = d;
            //Call start handler with length
            uint8_t w = aspi_start_handler(addr, len);
            //Reset if start handler does not recognise address
            if(w == 0){
                //Address is rejected, reset
                current_state = ASPI_STAT_FREE;
            }else if(w == 1){
                //Zeroth device in chain, do not wait
                current_state = ASPI_STAT_CHAIN;
            }else{
                //(w - 1)'th in chain, wait (w - 1)*len bytes before
                wait = (w - 1)*len;//Number of bytes to wait for
                current_state = ASPI_STAT_CHAIN_WAIT;
            }
            break;

        case ASPI_STAT_CHAIN_WAIT:
            //Wait for N*(w - 1) bytes...
            wait--;
            if(wait == 0){
                current_state = ASPI_STAT_CHAIN;
            }
            break;

        case ASPI_STAT_CHAIN:
            len--;
            if(len == 0){
                current_state = ASPI_STAT_CHAIN_DONE;
            }
            *t = aspi_rx_handler(d, count++);
            return 1;

        default:
            break;
    }
    return 0;
}

aspi_status aspi_current_status() {
    return current_state;
}

uint8_t aspi_get_length() {
    return len;
}

uint8_t aspi_get_addr(){
    return addr;
}

void aspi_throw(uint8_t _err) {
    err = _err;
    //Reset state and call end handler if relevant
    aspi_comm_deselect();
}

uint8_t aspi_get_error() {
    return err;
}






