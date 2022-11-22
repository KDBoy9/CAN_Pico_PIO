// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#pragma once

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// ------ //
// can_tx //
// ------ //

#define can_tx_wrap_target 0
#define can_tx_wrap 27

static const uint16_t can_tx_program_instructions[] = {
            //     .wrap_target
    0xe001, //  0: set    pins, 1                    
    0x80a0, //  1: pull   block                      
    0xe046, //  2: set    y, 6                       
    0x20a0, //  3: wait   1 pin, 0                   
    0x0886, //  4: jmp    y--, 6                 [8] 
    0x0008, //  5: jmp    8                          
    0x00c4, //  6: jmp    pin, 4                     
    0x0002, //  7: jmp    2                          
    0xe020, //  8: set    x, 0                       
    0x6021, //  9: out    x, 1                       
    0x002e, // 10: jmp    !x, 14                     
    0xe001, // 11: set    pins, 1                    
    0x00d0, // 12: jmp    pin, 16                    
    0x0018, // 13: jmp    24                         
    0xe000, // 14: set    pins, 0                    
    0x00d8, // 15: jmp    pin, 24                    
    0x04e8, // 16: jmp    !osre, 8               [4] 
    0x8080, // 17: pull   noblock                    
    0xa047, // 18: mov    y, osr                     
    0x01a8, // 19: jmp    x != y, 8              [1] 
    0xe02b, // 20: set    x, 11                      
    0xa0c1, // 21: mov    isr, x                     
    0x8000, // 22: push   noblock                    
    0x0000, // 23: jmp    0                          
    0xe036, // 24: set    x, 22                      
    0xa0c1, // 25: mov    isr, x                     
    0x8000, // 26: push   noblock                    
    0x0000, // 27: jmp    0                          
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program can_tx_program = {
    .instructions = can_tx_program_instructions,
    .length = 28,
    .origin = -1,
};

static inline pio_sm_config can_tx_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + can_tx_wrap_target, offset + can_tx_wrap);
    return c;
}

#include "hardware/clocks.h"
#include "hardware/dma.h"
static inline void can_tx_program_init(PIO pio, uint sm, uint offset, uint output_pin , uint input_pin , uint dma_chan ) {
    pio_sm_config c = can_tx_program_get_default_config( offset ) ;
    dma_channel_config d = dma_channel_get_default_config( dma_chan ) ;
    sm_config_set_out_pins( &c , output_pin , 1 ) ;
    sm_config_set_out_shift( &c , true , true , 32 ) ;
    sm_config_set_in_pins( &c , input_pin ) ;
    sm_config_set_in_shift( &c , true , true , 32 ) ;
    sm_config_set_set_pins( &c , output_pin , 1 ) ;
    sm_config_set_jmp_pin( &c , input_pin ) ;
	channel_config_set_read_increment( &d , true ) ;
	channel_config_set_write_increment( &d , false ) ;
	channel_config_set_dreq( &d , pio_get_dreq( pio , sm , true ) ) ;
	channel_config_set_transfer_data_size( &d , DMA_SIZE_32 ) ;
	dma_channel_set_write_addr( dma_chan , &pio->txf[sm] , false ) ;
    pio_gpio_init( pio , output_pin ) ;
    pio_gpio_init( pio , input_pin ) ;
    pio_sm_set_consecutive_pindirs( pio , sm , output_pin , 1 , true ) ;
    pio_sm_set_consecutive_pindirs( pio , sm , input_pin , 1 , false ) ;
    sm_config_set_clkdiv( &c , 250 ) ;
    pio_sm_init( pio , sm , offset , &c ) ;
    dma_channel_set_config( dma_chan , &d , false ) ;
    dma_channel_start( dma_chan ) ;
    pio_sm_set_enabled( pio , sm , true ) ;
}

#endif

