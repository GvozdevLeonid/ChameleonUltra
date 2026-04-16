#include "lf_reader_data.h"

#include <string.h>

#include "bsp_delay.h"
#include "bsp_time.h"
#include "circular_buffer.h"
#include "lf_125khz_radio.h"
#include "protocols/fdx_b.h"
#include "protocols/protocols.h"

#define NRF_LOG_MODULE_NAME fdx_b_reader
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
NRF_LOG_MODULE_REGISTER();


#define FDX_B_BUFFER_SIZE (256)

static circular_buffer cb;

static void fdx_b_gpio_int0_cb(void) {
    uint32_t cntr = get_lf_counter_value();
    uint16_t val = 0;
    
    if (cntr > 0xff) {
        val = 0xff;
    } else {
        val = cntr & 0xff;
    }

    cb_push_back(&cb, &val);
    clear_lf_counter_value();
}

static void init_fdx_b_hw(void) {
    register_rio_callback(fdx_b_gpio_int0_cb);
    lf_125khz_radio_gpiote_toggle_enable();
}

static void uninit_fdx_b_hw(void) {
    lf_125khz_radio_gpiote_disable();
    unregister_rio_callback();
}

bool fdx_b_read(uint8_t *data, uint32_t timeout_ms) {
    void *codec = fdx_b.alloc();
    fdx_b.decoder.start(codec, 0);

    cb_init(&cb, FDX_B_BUFFER_SIZE, sizeof(uint16_t));
    init_fdx_b_hw();
    start_lf_125khz_radio();
   
    bool ok = false;
    autotimer *p_at = bsp_obtain_timer(0);
    
    while (!ok && NO_TIMEOUT_1MS(p_at, timeout_ms)) {
        uint16_t val = 0;
        
        while (!ok && NO_TIMEOUT_1MS(p_at, timeout_ms) && cb_pop_front(&cb, &val)) {
            
            if (fdx_b.decoder.feed(codec, val)) {
                memcpy(data, fdx_b.get_data(codec), fdx_b.data_size);
                ok = true;
                break;
            }
        }
    }

    bsp_return_timer(p_at);
    stop_lf_125khz_radio();
    uninit_fdx_b_hw();
    cb_free(&cb);

    fdx_b.free(codec);
    return ok;
}