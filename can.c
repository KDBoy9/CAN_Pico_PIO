#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "can.pio.h"

#include <inttypes.h>
#include <stdbool.h>
#define CANHACK_MAX_BITS                        (160U)

typedef struct {
    uint8_t tx_bitstream[CANHACK_MAX_BITS];     ///< The bitstream of the CAN frame
    bool stuff_bit[CANHACK_MAX_BITS];           ///< Indicates if the corresponding bit is a stuff bit
    uint8_t tx_bits;                            ///< Number of  bits in the frame
    uint32_t tx_arbitration_bits;               ///< Number of bits in arbitartion (including stuff bits); the fields are ID A + RTR (standard) or ID A + SRR + IDE + ID B + RTR (extended)

    // Fields set when creating the CAN frame
    uint32_t crc_rg;                            ///< CRC value (15 bit value)
    uint32_t last_arbitration_bit;              ///< Bit index of last arbitration bit (always the RTR bit for both IDE = 0 and IDE = 1); may be a stuff bit
    uint32_t last_dlc_bit;                      ///< Bit index of last bit of DLC field; may be a stuff bit
    uint32_t last_data_bit;                     ///< Bit index of the last bit of the data field; may be a stuff bit
    uint32_t last_crc_bit;                      ///< Bit index of last bit of the CRC field; may be a stuff bit
    uint32_t last_eof_bit;                      ///< Bit index of the last bit of the EOF field; may be a stuff bit
    bool frame_set;                             ///< True when the frame has been set; may be a stuff bit

    // Fields used during creation of the CAN frame
    uint32_t dominant_bits;                     ///< Dominant bits in a row
    uint32_t recessive_bits;                    ///< Recessive bits in a row
    bool stuffing;                              ///< True if stuffing enabled
    bool crcing;                                ///< True if CRCing enabled
} canhack_frame_t;

/// \brief Set the parameters of the CAN frame
/// \param id_a 11-bit CAN ID
/// \param id_b 18-bit extension to CAN ID (used if ide=true)
/// \param rtr true if frame is remote
/// \param ide true if frame is extended
/// \param dlc DLC field; data length if a data frame, arbitrary 4-bit value if a remote frame
/// \param data pointer to up to 8 bytes of payload
/// \param frame the handle to the frame (see canhack_get_frame)
void canhack_set_frame(uint32_t id_a, uint32_t id_b, bool rtr, bool ide, uint32_t dlc, const uint8_t *data, canhack_frame_t *frame);



////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// CAN frame creator.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void add_raw_bit(uint8_t bit, bool stuff, canhack_frame_t *frame)
{
    // Record the status of the stuff bit for display purposes
    frame->stuff_bit[frame->tx_bits] = stuff;
    frame->tx_bitstream[frame->tx_bits++] = bit;
}

static void do_crc(uint8_t bitval, canhack_frame_t *frame)
{
    uint32_t bit_14 = (frame->crc_rg & (1U << 14U)) >> 14U;
    uint32_t crc_nxt = bitval ^ bit_14;
    frame->crc_rg <<= 1U;
    frame->crc_rg &= 0x7fffU;
    if (crc_nxt) {
        frame->crc_rg ^= 0x4599U;
    }
}

static void add_bit(uint8_t bit, canhack_frame_t *frame)
{
    if (frame->crcing) {
        do_crc(bit, frame);
    }
    add_raw_bit(bit, false, frame);
    if (bit) {
        frame->recessive_bits++;
        frame->dominant_bits = 0;
    } else {
        frame->dominant_bits++;
        frame->recessive_bits = 0;
    }
    if (frame->stuffing) {
        if (frame->dominant_bits >= 5U) {
            add_raw_bit(1U, true, frame);
            frame->dominant_bits = 0;
            frame->recessive_bits = 1U;
        }
        if (frame->recessive_bits >= 5U) {
            add_raw_bit(0, true, frame);
            frame->dominant_bits = 1U;
            frame->recessive_bits = 0;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// API to module
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void canhack_set_frame(uint32_t id_a, uint32_t id_b, bool rtr, bool ide, uint32_t dlc, const uint8_t *data, canhack_frame_t *frame)
{
    uint8_t len = rtr ? 0 : (dlc >= 8U ? 8U : dlc); // RTR frames have a DLC of any value but no data field

    frame->tx_bits = 0;
    frame->crc_rg = 0;
    frame->stuffing = true;
    frame->crcing = true;
    frame->dominant_bits = 0;
    frame->recessive_bits = 0;

    for (uint32_t i = 0; i < CANHACK_MAX_BITS; i++) {
        frame->tx_bitstream[i] = 1U;
    }

    // ID field is:
    // {SOF, ID A, RTR, IDE = 0, r0} [Standard]
    // {SOF, ID A, SRR = 1, IDE = 1, ID B, RTR, r1, r0) [Extended]

    // SOF
    add_bit(0, frame);

    // ID A
    id_a <<= 21U;
    for (uint32_t i = 0; i < 11U; i++) {
        if (id_a & 0x80000000U) {
            add_bit(1U, frame);
        }
        else {
            add_bit(0, frame);
        }
        id_a <<= 1U;
    }

    // RTR/SRR
    if (rtr || ide) {
        add_bit(1U, frame);
    }
    else {
        add_bit(0, frame);
    }

    // The last bit of the arbitration field is the RTR bit if a basic frame; this might be overwritten if IDE = 1
    frame->last_arbitration_bit = frame->tx_bits - 1U;

    // IDE
    if (ide) {
        add_bit(1U, frame);
    }
    else {
        add_bit(0, frame);
    }

    if (ide) {
        // ID B
        id_b <<= 14U;
        for (uint32_t i = 0; i < 18U; i++) {
            if (id_b & 0x80000000U) {
                add_bit(1U, frame);
            } else {
                add_bit(0, frame);
            }
            id_b <<= 1U;
        }
        // RTR
        if (rtr) {
            add_bit(1U, frame);
        }
        else {
            add_bit(0, frame);
        }
        // The RTR bit is the last bit in the arbitration field if an extended frame
        frame->last_arbitration_bit = frame->tx_bits - 1U;

        // r1
        add_bit(0, frame);
    }
    else {
        // If IDE = 0 then the last arbitration field bit is the RTR
    }
    // r0
    add_bit(0, frame);

    // DLC
    dlc <<= 28U;
    for (uint32_t i = 0; i < 4U; i++) {
        if (dlc & 0x80000000U) {
            add_bit(1U, frame);
        } else {
            add_bit(0, frame);
        }
        dlc <<= 1U;
    }
    frame->last_dlc_bit = frame->tx_bits - 1U;

    // Data
    for (uint32_t i = 0; i < len; i ++) {
        uint8_t byte = data[i];
        for (uint32_t j = 0; j < 8; j++) {
            if (byte & 0x80U) {
                add_bit(1U, frame);
            }
            else {
                add_bit(0, frame);
            }
            byte <<= 1U;
        }
    }
    // If the length is 0 then the last data bit is equal to the last DLC bit
    frame->last_data_bit = frame->tx_bits - 1U;

    // CRC
    frame->crcing = false;
    uint32_t crc_rg = frame->crc_rg << 17U;
    for (uint32_t i = 0; i < 15U; i++) {
        if (crc_rg & 0x80000000U) {
            add_bit(1U, frame);
        } else {
            add_bit(0, frame);
        }
        crc_rg <<= 1U;
    }
    frame->last_crc_bit = frame->tx_bits - 1U;

    // Bit stuffing is disabled at the end of the CRC field
    frame->stuffing = false;

    // CRC delimiter
    add_bit(1U, frame);

    // ACK; we transmit this as a dominant bit to ensure the state machines lock on to the right
    // EOF field; it's mostly moot since if there are no CAN controllers then there is not much
    // hacking to do.
    add_bit(0, frame);

    // ACK delimiter
    add_bit(1U, frame);

    // EOF
    add_bit(1U, frame);
    add_bit(1U, frame);
    add_bit(1U, frame);
    add_bit(1U, frame);
    add_bit(1U, frame);
    add_bit(1U, frame);
    add_bit(1U, frame);
    frame->last_eof_bit = frame->tx_bits - 1U;

    // IFS
    add_bit(1U, frame);
    add_bit(1U, frame);
    add_bit(1U, frame);

    // Set up the matching masks for this CAN frame
    frame->tx_arbitration_bits = frame->last_arbitration_bit + 1U;

    frame->frame_set = true;
}

void make_can_word( canhack_frame_t * pico_frame , uint32_t * can_buffer , uint32_t * can_buffer_count )
{
	for( int i = 0 ; i < 5 ; i++ )
	{
		can_buffer[i] = 0 ;
	}
	*can_buffer_count = 0 ;

	int temp_bits = 0 ;
	int bits_in_word = 0 ;
	int temp_word = 0 ;

	while( temp_bits < pico_frame->tx_bits )
	{
		can_buffer[temp_word] = can_buffer[temp_word] << 1 ;
		bits_in_word++ ;

		if( pico_frame->tx_bitstream[temp_bits] == 1 )
		{
			can_buffer[temp_word] = can_buffer[temp_word] + 1 ;
		}

		if( bits_in_word == 32 )
		{
			temp_word++ ;
			bits_in_word = 0 ;
		}
		temp_bits++ ;
	}

	for( int j = bits_in_word ; bits_in_word < 32 ; bits_in_word++ )
	{
		can_buffer[temp_word] = can_buffer[temp_word] << 1 ;
		can_buffer[temp_word] = can_buffer[temp_word] + 1 ;
	}
	temp_word++ ;
	*can_buffer_count = (uint32_t) temp_word ;
}


int main()
{
#ifndef PICO_DEFAULT_LED_PIN
#warning blink example requires a board with a regular LED
#else

    uint LED_PIN = PICO_DEFAULT_LED_PIN ;
	PIO pio = pio0 ;
	uint offset = pio_add_program( pio , &can_program ) ;
	uint sm = pio_claim_unused_sm( pio , true ) ;
	
	const uint TX_PIN = 7 ;
	const uint RX_PIN = 8 ;
    uint dma_chan ;
    dma_channel_claim( dma_chan ) ;


	can_program_init( pio , sm , offset , TX_PIN , RX_PIN , dma_chan ) ;


	uint32_t can_buffer[5]  ;
	uint32_t can_buffer_count ;

	canhack_frame_t pico_frame ;

	const uint8_t data[3] = { 0x31 , 0xCD , 0x0C } ;

	canhack_set_frame( 0x201 , 0x568 , false , false , 0x03 , data , &pico_frame ) ;

	make_can_word( &pico_frame , can_buffer , &can_buffer_count ) ;
while(true)
{
    for( int i = 0 ; i < can_buffer_count ; i++ )
        dma_channel_transfer_from_buffer_now( dma_chan , &can_buffer[i] , 1 ) ;
}




#endif

}
