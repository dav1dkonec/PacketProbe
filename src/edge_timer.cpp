#include "edge_timer.h"

#include "edge_timer.pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"

EdgeTimer::EdgeTimer(PIO pio, uint pin, uint32_t* ring, uint ring_bits)
    : pio_(pio), ring_(ring), ring_mask_((1u << ring_bits) / sizeof(uint32_t) - 1)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_down(pin);

    sm_ = pio_claim_unused_sm(pio, true);
    // the program is shared by all timers on this PIO block
    static int offset[2] = {-1, -1};
    const int idx = pio_get_index(pio);
    if (offset[idx] < 0) {
        offset[idx] = static_cast<int>(pio_add_program(pio, &edge_timer_program));
    }
    edge_timer_program_init(pio, sm_, static_cast<uint>(offset[idx]), pin);

    dma_ = static_cast<uint>(dma_claim_unused_channel(true));
    dma_channel_config c = dma_channel_get_default_config(dma_);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_ring(&c, true, ring_bits);  // write address wraps in the ring
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm_, false));
    dma_channel_configure(dma_, &c, ring_, &pio->rxf[sm_], kDmaCount, true);
}
