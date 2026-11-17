#ifndef PROBE_EDGE_TIMER_H
#define PROBE_EDGE_TIMER_H

#include <cstdint>

#include "hardware/dma.h"
#include "hardware/pio.h"

/// Edge timestamps of one input pin: a PIO state machine (edge_timer.pio)
/// writes raw counter values through DMA into a RAM ring, drain() decodes them
/// into sys clock cycles since the state machine started (resolution 2 cycles).
class EdgeTimer {
public:
    /// Claims a state machine and a DMA channel; the state machine stays
    /// disabled until started, so several timers can start in sync:
    /// pio_enable_sm_mask_in_sync(pio, a.sm_mask() | b.sm_mask()).
    /// @p ring: 2^@p ring_bits bytes, aligned to its size (DMA ring wrap).
    /// Not a member: an aligned member would pad the object to 3x its size.
    EdgeTimer(PIO pio, uint pin, uint32_t* ring, uint ring_bits);

    uint32_t sm_mask() const { return 1u << sm_; }

    /// Calls @p on_edge(uint64_t cycles, bool rising) for every pending edge.
    /// @p now_bound: cycles since start, not earlier than any pending edge
    /// (from time_us_64 with a margin); resolves wraps of X during long
    /// silence, so it must be less than ~68 s after the edges (drain often).
    template <typename F>
    void drain(uint64_t now_bound, F&& on_edge)
    {
        const uint32_t written = begin_batch();
        for (uint32_t r = read_; r != written; ++r) {
            const uint32_t x = ring_[r & ring_mask_];
            sync(x);
            uint64_t t = time_at(r);
            add_missed_wraps(t, now_bound);
            level_ = !level_;
            on_edge(t, level_);
        }
        read_ = written;
    }

    /// Burst mode for the wire channel: edges closer than @p gap_iters loop
    /// iterations (2 cycles each) form one burst; calls
    /// @p on_burst(uint64_t start, uint64_t end, uint32_t edges) for each burst
    /// that ended. Per edge only a load, a 32-bit subtraction and a compare
    /// (~10 cycles, up to ~10M edges/s): the edge index is the word position,
    /// the 64-bit time is computed at burst boundaries only.
    /// @p now_bound as for drain(). Do not mix with drain().
    template <typename F>
    void drain_bursts(uint32_t gap_iters, uint64_t now_bound, F&& on_burst)
    {
        const uint32_t written = begin_batch();
        if (lost_burst_) {
            lost_burst_ = false;
            burst_open_ = false;  // its edges are partly overwritten: drop it
        }
        // locals keep the loop in registers (Cortex-M0+: 8 low registers)
        const uint32_t* const ring = ring_;
        const uint32_t mask = ring_mask_;
        uint32_t last_x = last_x_;
        bool open = burst_open_;

        for (uint32_t r = read_; r != written; ++r) {
            const uint32_t x = ring[r & mask];
            const uint32_t d = last_x - x;  // decrements since the previous edge
            if (d > gap_iters || !open) [[unlikely]] {
                // a wrap of X (x > last_x) also lands here: d is then huge
                const bool wrapped = x > last_x;
                sync(x);
                uint64_t t = time_at(r);
                if (open) {
                    // previous edge: d + 1 iterations earlier (+1 cycle if X wrapped)
                    on_burst(burst_start_, t - 2 * (uint64_t{d} + 1) - wrapped, r - burst_first_);
                }
                add_missed_wraps(t, now_bound);
                open = true;
                burst_start_ = t;
                burst_first_ = r;
            }
            last_x = x;
        }

        read_ = written;
        last_x_ = last_x;
        burst_open_ = open;
    }

    /// Ends the open burst if its last edge is more than @p gap cycles before
    /// @p now (cycles, must not be ahead of the PIO: drain first).
    template <typename F>
    void close_idle_burst(uint64_t now, uint64_t gap, F&& on_burst)
    {
        if (!burst_open_) {
            return;
        }
        const uint32_t r = read_ - 1;  // word of the last edge
        const uint32_t d = ref_x_ - last_x_;
        const uint64_t last = 2 * (dec_ + d + (r - 1)) + wraps_ + (last_x_ > ref_x_);
        if (now > last + gap) {
            burst_open_ = false;
            on_burst(burst_start_, last, r + 1 - burst_first_);
        }
    }

    /// Start of the burst in progress, false if none.
    bool open_burst(uint64_t& start) const
    {
        start = burst_start_;
        return burst_open_;
    }

    /// Words lost because the CPU fell behind by more than the ring size.
    uint32_t overruns() const { return overruns_; }  // read by the other core

private:
    /// X passes 0 every 2^32 iterations (~68.7 s at 125 MHz), one extra cycle each.
    static constexpr uint64_t kWrapCycles = (uint64_t{1} << 33) + 1;
    static constexpr uint32_t kDmaCount = 0xffffffffu;  // ~1 h at 1.2M edges/s (10 Mb/s, 1 frame/ms)

    /// Words written by the DMA; handles the marker and an overrun.
    uint32_t begin_batch()
    {
        const uint32_t written = kDmaCount - dma_channel_hw_addr(dma_)->transfer_count;
        if (written - read_ > ring_mask_ + 1) {
            // the DMA overwrote unread words: skip them. Indexes stay right (word
            // positions) and so does the time (taken from the next X).
            const uint32_t skipped = written - read_ - (ring_mask_ + 1);
            overruns_ = overruns_ + skipped;
            read_ += skipped;
            level_ ^= (skipped & 1u) != 0;
            lost_burst_ = true;
        }
        if (read_ == 0 && written != 0) {
            level_ = ring_[0] != 0;  // marker: level before the first edge
            read_ = 1;
        }
        return written;
    }

    /// Moves the reference point (dec_, wraps_ at X = ref_x_) to X = @p x;
    /// right as long as X decreased by less than one full wrap since.
    void sync(uint32_t x)
    {
        dec_ += ref_x_ - x;  // modulo 2^32: also right across one wrap
        wraps_ += x > ref_x_;
        ref_x_ = x;
    }

    /// Time of the edge in word @p r, after sync() to its X.
    /// Word 0 is the marker, so word r holds edge r - 1; the iteration that
    /// pushes an edge does not decrement X: + the edges before it.
    uint64_t time_at(uint32_t r) const { return 2 * (dec_ + (r - 1)) + wraps_; }

    /// Two edges more than one wrap apart look like less than one: the time
    /// @p t is then a multiple of kWrapCycles too early. @p now_bound tells by
    /// how many, as long as it is less than one wrap after the real time.
    void add_missed_wraps(uint64_t& t, uint64_t now_bound)
    {
        if (now_bound > t + kWrapCycles) {
            const uint64_t k = (now_bound - t) / kWrapCycles;
            t += k * kWrapCycles;
            dec_ += k << 32;
            wraps_ += static_cast<uint32_t>(k);
        }
    }

    PIO pio_;
    uint sm_;
    uint dma_;
    uint32_t* ring_;
    uint32_t ring_mask_;  // words - 1

    uint32_t read_ = 0;  // words consumed, word 0 = marker
    volatile uint32_t overruns_ = 0;
    bool level_ = false;  // level after the last decoded edge

    // reference point of the time: decrements and wraps of X until X = ref_x_
    uint32_t ref_x_ = 0xffffffffu;
    uint64_t dec_ = 0;
    uint32_t wraps_ = 0;

    // burst mode
    uint32_t last_x_ = 0xffffffffu;
    bool burst_open_ = false;
    bool lost_burst_ = false;
    uint64_t burst_start_ = 0;
    uint32_t burst_first_ = 0;  // word of its first edge
};

#endif // PROBE_EDGE_TIMER_H
