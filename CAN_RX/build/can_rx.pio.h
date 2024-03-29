// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#pragma once

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// ------ //
// can_rx //
// ------ //

#define can_rx_wrap_target 0
#define can_rx_wrap 27

static const uint16_t can_rx_program_instructions[] = {
            //     .wrap_target
    0xe048, //  0: set    y, 8                       
    0x25a0, //  1: wait   1 pin, 0               [5] 
    0x0005, //  2: jmp    5                          
    0x04c5, //  3: jmp    pin, 5                 [4] 
    0x0000, //  4: jmp    0                          
    0x0483, //  5: jmp    y--, 3                 [4] 
    0x2020, //  6: wait   0 pin, 0                   
    0xe046, //  7: set    y, 6                       
    0x4001, //  8: in     pins, 1                    
    0x00cc, //  9: jmp    pin, 12                    
    0xe420, // 10: set    x, 0                   [4] 
    0x0208, // 11: jmp    8                      [2] 
    0x002f, // 12: jmp    !x, 15                     
    0x0688, // 13: jmp    y--, 8                 [6] 
    0x0012, // 14: jmp    18                         
    0xe021, // 15: set    x, 1                       
    0xe246, // 16: set    y, 6                   [2] 
    0x0208, // 17: jmp    8                      [2] 
    0xe020, // 18: set    x, 0                       
    0xa049, // 19: mov    y, !x                      
    0xa026, // 20: mov    x, isr                     
    0x0038, // 21: jmp    !x, 24                     
    0x4041, // 22: in     y, 1                       
    0x0014, // 23: jmp    20                         
    0xe020, // 24: set    x, 0                       
    0xa0c9, // 25: mov    isr, !x                    
    0x8020, // 26: push   block                      
    0x0000, // 27: jmp    0                          
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program can_rx_program = {
    .instructions = can_rx_program_instructions,
    .length = 28,
    .origin = -1,
};

static inline pio_sm_config can_rx_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + can_rx_wrap_target, offset + can_rx_wrap);
    return c;
}

#include "hardware/clocks.h"
static inline void can_rx_program_init(PIO pio, uint sm, uint offset, uint input_pin ) {
    pio_sm_config c = can_rx_program_get_default_config( offset ) ;
	pio_gpio_init( pio , input_pin ) ;
    sm_config_set_in_pins( &c , input_pin ) ;
    sm_config_set_in_shift( &c , false , true , 32 ) ;
    sm_config_set_jmp_pin( &c , input_pin ) ;
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv( &c , 25 ) ;
    pio_sm_init( pio , sm , offset , &c ) ;
    pio_sm_set_enabled( pio , sm , true ) ;
}

#endif

