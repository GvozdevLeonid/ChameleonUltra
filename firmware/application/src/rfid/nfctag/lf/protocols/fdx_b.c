#include "fdx_b.h"

#include <stdlib.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf_pwm.h"
#include "protocols.h"
#include "t55xx.h"
#include "crc_utils.h"
#include "tag_base_type.h"
#include "utils/diphasedemod.h"

#define FDX_B_RAW_SIZE      (128)
// 128 bits * 2 half-bits * = 256
#define FDX_B_PWM_SIZE      (FDX_B_RAW_SIZE * 2)
#define FDX_B_DATA_SIZE     (11)    // 88 bits
// 38 bits: National ID
// 10 bits: Country Code
// 1 bit: Extra data flag.
// 14 bits: Reserved / Application bits.
// 1 bit: Animal flag 
// 24 bits: Extra data
#define FDX_B_HEADER        (0x001)
#define FDX_B_T55XX_BLOCK_COUNT (5)

#define FDX_B_TIME_1    (32) // 1T
#define FDX_B_TIME_2    (48) // 1.5T
#define FDX_B_TIME_3    (64) // 2T
#define FDX_B_JITTER_TIME  (8)

#define NRF_LOG_MODULE_NAME fdx_b_protocol
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
NRF_LOG_MODULE_REGISTER();

static nrf_pwm_values_wave_form_t m_fdx_b_pwm_seq_vals[FDX_B_RAW_SIZE * 2] = {};

nrf_pwm_sequence_t const m_fdx_b_pwm_seq = {
    .values.p_wave_form = m_fdx_b_pwm_seq_vals,
    .length = NRF_PWM_VALUES_LENGTH(m_fdx_b_pwm_seq_vals),
    .repeats = 0,
    .end_delay = 0,
};

typedef struct {
    uint8_t data[FDX_B_DATA_SIZE];
    uint64_t raw[2];
    uint8_t raw_length;
    diphase *modem;
} fdx_b_codec;


void byte_to_bytebits(uint8_t src, uint8_t *dest) {
    for (size_t i = 0 ; i < 8 ; ++i) {
        dest[i] =  src & 1;
        src >>= 1;
    }
}

uint16_t fdx_b_crc(uint8_t *data) {
    uint16_t crc = 0x0000; 

    for (uint8_t i = 0; i < 8; i++) {
        crc ^= (uint16_t)(data[i] << 8);

        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }

    uint16_t reflected_crc = 0;
    for (uint8_t i = 0; i < 16; i++) {
        if (crc & (1U << i)) {
            reflected_crc |= (1U << (15 - i));
        }
    }

   return reflected_crc;
}

static inline bool fdx_b_get_bit(uint64_t raw0, uint64_t raw1, uint8_t idx) {
    if (idx < 64) {
        // bits from 0 to 63 in raw[0]
        return (raw0 >> (63 - idx)) & 1;
    } else {
        // bits from 64 to 127 in raw[1]
        return (raw1 >> (127 - idx)) & 1;
    }
}

static void fdx_b_raw_data(uint8_t *fdx_b_data, uint8_t *bits) {

    // preambule
    memset(bits, 0, FDX_B_RAW_SIZE);
    bits[10] = 1;

    uint16_t crc = fdx_b_crc(fdx_b_data);

    // every 9th bit is 0x01
    for (uint8_t bit_n = 19; bit_n < FDX_B_RAW_SIZE; bit_n+=9) {
        bits[bit_n] = 1;
    }

    // National ID + Country Code
    byte_to_bytebits(fdx_b_data[0], bits + 11);
    byte_to_bytebits(fdx_b_data[1], bits + 20);
    byte_to_bytebits(fdx_b_data[2], bits + 29);
    byte_to_bytebits(fdx_b_data[3], bits + 38);
    byte_to_bytebits(fdx_b_data[4], bits + 47);
    byte_to_bytebits(fdx_b_data[5], bits + 56);

    // flags
    byte_to_bytebits(fdx_b_data[6], bits + 65);
    byte_to_bytebits(fdx_b_data[7], bits + 74);

    // crc
    byte_to_bytebits(crc >> 0, bits + 83);
    byte_to_bytebits(crc >> 8, bits + 92);

    // Extra data
    byte_to_bytebits(fdx_b_data[8], bits + 101);
    byte_to_bytebits(fdx_b_data[9], bits + 110);
    byte_to_bytebits(fdx_b_data[10], bits + 119);
}

static bool fdx_b_get_time(uint8_t interval, uint8_t base) {
    return interval >= (base - FDX_B_JITTER_TIME) &&
           interval <= (base + FDX_B_JITTER_TIME);
}

static uint8_t fdx_b_period(uint8_t interval) {
    if (fdx_b_get_time(interval, FDX_B_TIME_1)) {
        return 0; // 1T
    }
    if (fdx_b_get_time(interval, FDX_B_TIME_2)) {
        return 1; // 1.5T
    }
    if (fdx_b_get_time(interval, FDX_B_TIME_3)) {
        return 2; // 2.0T
    }
    return 3; // Noise
}


static fdx_b_codec *fdx_b_alloc(void) {
    fdx_b_codec *d = malloc(sizeof(fdx_b_codec));
    d->modem = malloc(sizeof(diphase));
    d->modem->rp = fdx_b_period;
    return d;
};

static void fdx_b_free(fdx_b_codec *d) {
    if (d->modem) {
        free(d->modem);
        d->modem = NULL;
    }
    free(d);
};

static uint8_t *fdx_b_get_data(fdx_b_codec *d) { 
    return d->data;
};

static void fdx_b_decoder_start(fdx_b_codec *d, uint8_t format) {
    memset(d->data, 0, sizeof(d->data));
    memset(d->raw, 0, sizeof(d->raw));
    d->raw_length = 0;

    diphase_reset(d->modem);
};

static bool fdx_b_decode_bit(fdx_b_codec *d, bool bit) {

    d->raw[0] = (d->raw[0] << 1) | (d->raw[1] >> 63);
    d->raw[1] = (d->raw[1] << 1) | bit;

    if (d->raw_length < FDX_B_RAW_SIZE) {
        d->raw_length++;
        return false;
    }

    // check header
    if ((d->raw[0] >> 53) != FDX_B_HEADER) {
        return false; 
    }

    uint8_t bit_idx = 11;
    uint8_t current_byte = 0;
    uint16_t received_crc = 0;

    // extract National ID, Country Code, Application bits.
    for (int byte_n = 0; byte_n < 8; byte_n++) {
        current_byte = 0;
        for (int b = 0; b < 8; b++) {
            current_byte = (current_byte << 1) | fdx_b_get_bit(d->raw[0], d->raw[1], bit_idx);
            bit_idx++; 
        }

        bit_idx++;
        d->data[byte_n] = current_byte;
    }

    // extract Extra data
    bit_idx = 101;
    for (int byte_n = 0; byte_n < 3; byte_n++) {
        current_byte = 0;
        for (int b = 0; b < 8; b++) {
            current_byte = (current_byte << 1) | fdx_b_get_bit(d->raw[0], d->raw[1], bit_idx);
            bit_idx++; 
        }

        bit_idx++;
        d->data[byte_n + 8] = current_byte;      
    }

    // extract crc
    bit_idx = 83;
    for (int byte_n = 0; byte_n < 2; byte_n++) {
        current_byte = 0;
        for (int b = 0; b < 8; b++) {
            current_byte = (current_byte << 1) | fdx_b_get_bit(d->raw[0], d->raw[1], bit_idx);
            bit_idx++; 
        }

        bit_idx++;
        if (byte_n == 0) {
            received_crc |= current_byte;
        }
        else {
            received_crc |= (current_byte << 8);
        }
    }

    // compare crc
    // if (fdx_b_crc(d->data) != received_crc) {
    //     return false;
    // }

    d->raw[0] = 0;
    d->raw[1] = 0;
    d->raw_length = 0;

    return true;
}

static bool fdx_b_decoder_feed(fdx_b_codec *d, uint16_t interval) {
    bool bits[2] = {0};
    int8_t bitlen = 0;

    diphase_feed(d->modem, (uint8_t)interval, bits, &bitlen);

    if (bitlen == -1) {
        diphase_reset(d->modem);
        d->raw_length = 0;
        return false;
    }

    for (int i = 0; i < bitlen; i++) {
        if (fdx_b_decode_bit(d, bits[i])) {
            return true;
        }
    }

    return false;
}

static const nrf_pwm_sequence_t *fdx_b_modulator(fdx_b_codec *d, uint8_t *buf) {
    uint8_t bits[FDX_B_RAW_SIZE];

    fdx_b_raw_data(buf, bits);
    bool level = false;
    int out = 0;

    for (int i = 0; i < FDX_B_RAW_SIZE; i++) {
        bool bit = bits[i];

        level = !level;

        m_fdx_b_pwm_seq_vals[out].channel_0 = level ? 16 : 0;
        m_fdx_b_pwm_seq_vals[out].counter_top = 15;
        out++;

        if (bit == 0) {
            level = !level;
        }

        m_fdx_b_pwm_seq_vals[out].channel_0 = level ? 16 : 0;
        m_fdx_b_pwm_seq_vals[out].counter_top = 15;
        out++;
    }

    return &m_fdx_b_pwm_seq;
}

const protocol fdx_b = {
    .tag_type = TAG_TYPE_FDX_B, 
    .data_size = FDX_B_DATA_SIZE,
    .alloc = (codec_alloc)fdx_b_alloc,
    .free = (codec_free)fdx_b_free,
    .get_data = (codec_get_data)fdx_b_get_data,
    .modulator = (modulator)fdx_b_modulator,
    .decoder =
        {
            .start = (decoder_start)fdx_b_decoder_start,
            .feed = (decoder_feed)fdx_b_decoder_feed,
        },
};

uint8_t fdx_b_t55xx_writer(uint8_t *fdx_b_data, uint32_t *blks) {

    uint8_t bits[FDX_B_RAW_SIZE];
    uint8_t bit_to_write;
    fdx_b_raw_data(fdx_b_data, bits);

    blks[0] = T5577_FDX_B_CONFIG; 

    for (int block_idx = 0; block_idx < 4; block_idx++) {
        uint32_t block_data = 0;

        for (int bit_idx = 0; bit_idx < 32; bit_idx++) {
            int current_bit_pos = (block_idx * 32) + bit_idx;

            // invert bit  (!bits) because using biphase in config.
            bit_to_write = bits[current_bit_pos] ? 0 : 1;
            block_data = (block_data << 1) | bit_to_write;
        }

        blks[block_idx + 1] = block_data;
    }

    return FDX_B_T55XX_BLOCK_COUNT;
}
