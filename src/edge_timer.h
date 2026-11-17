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
    EdgeTimer(PIO pio, uint pin);

    uint32_t sm_mask() const { return 1u << sm_; }

    /// Calls @p on_edge(uint64_t cycles, bool rising) for every pending edge.
    /// Hot path (up to ~1.2M edges/s): one DMA register read per batch and a
    /// 32-bit subtraction per edge; inline so it runs from RAM with the caller.
    template <typename F>
    void drain(F&& on_edge)
    {
        const uint32_t written = kDmaCount - dma_channel_hw_addr(dma_)->transfer_count;
        uint32_t r = read_;
        if (written - r > kRingLen) {
            // the DMA overwrote unread words; their edges still count, so
            // the time stays exact (a wrap of X inside the gap is still seen)
            const uint32_t skipped = written - r - kRingLen;
            overruns_ = overruns_ + skipped;
            r += skipped;
            if (!have_marker_) {
                have_marker_ = true;  // lost with the skipped words: level unknown
            }
            edges_ += skipped;
            level_ ^= (skipped & 1u) != 0;
        }
        for (; r != written; ++r) {
            const uint32_t x = ring_[r & (kRingLen - 1)];
            if (!have_marker_) {
                have_marker_ = true;
                level_ = x != 0;
                continue;
            }
            if (x > last_x_) {
                ++wraps_;  // X passed 0 (needs an edge at least every ~68 s)
            }
            decrements_ += last_x_ - x;  // modulo 2^32: also right across a wrap
            last_x_ = x;
            // the iteration that pushes an edge does not decrement X: + edges
            const uint64_t cycles = 2 * (decrements_ + edges_) + wraps_;
            ++edges_;
            level_ = !level_;
            on_edge(cycles, level_);
        }
        read_ = r;
    }

    /// Burst mode for the wire channel: edges closer than @p gap_iters loop
    /// iterations (2 cycles each) form one burst; calls
    /// @p on_burst(uint64_t start, uint64_t end, uint32_t edges) for each burst
    /// that ended. Per edge only a 32-bit subtraction and a compare: the 64-bit
    /// time is computed at burst boundaries only. Do not mix with drain().
    template <typename F>
    void drain_bursts(uint32_t gap_iters, F&& on_burst)
    {
        const uint32_t written = kDmaCount - dma_channel_hw_addr(dma_)->transfer_count;
        uint32_t r = read_;
        if (r == written) {
            return;
        }
        if (written - r > kRingLen) {
            const uint32_t skipped = written - r - kRingLen;
            overruns_ = overruns_ + skipped;
            r += skipped;
            have_marker_ = true;
            edges_ += skipped;
            burst_open_ = false;  // its edges are partly lost: drop it
        }
        if (!have_marker_) {
            have_marker_ = true;
            ++r;
        }

        // state in locals: keeps the loop in registers
        uint32_t last_x = last_x_;
        uint64_t dec = decrements_;
        uint64_t edges = edges_;
        uint32_t wraps = wraps_;
        bool open = burst_open_;
        uint64_t start = burst_start_;
        uint32_t first_edge = burst_first_edge_;

        for (; r != written; ++r) {
            const uint32_t x = ring_[r & (kRingLen - 1)];
            const uint32_t dx = last_x - x;  // decrements since the previous edge
            const bool wrapped = x > last_x;
            last_x = x;
            dec += dx;
            wraps += wrapped;
            if (dx > gap_iters || !open) {
                const uint64_t t = 2 * (dec + edges) + wraps;
                if (open) {
                    // previous edge: dx + 1 iterations earlier (+1 cycle if X wrapped)
                    on_burst(start, t - 2 * (uint64_t{dx} + 1) - wrapped,
                             static_cast<uint32_t>(edges) - first_edge);
                }
                open = true;
                start = t;
                first_edge = static_cast<uint32_t>(edges);
            }
            ++edges;
        }

        read_ = r;
        last_x_ = last_x;
        decrements_ = dec;
        edges_ = edges;
        wraps_ = wraps;
        burst_open_ = open;
        burst_start_ = start;
        burst_first_edge_ = first_edge;
    }

    /// Ends the open burst if its last edge is more than @p gap cycles before
    /// @p now (cycles, must not be ahead of the PIO: drain first).
    template <typename F>
    void close_idle_burst(uint64_t now, uint64_t gap, F&& on_burst)
    {
        if (!burst_open_) {
            return;
        }
        const uint64_t last = 2 * (decrements_ + edges_ - 1) + wraps_;
        if (now > last + gap) {
            burst_open_ = false;
            on_burst(burst_start_, last, static_cast<uint32_t>(edges_) - burst_first_edge_);
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
    static constexpr uint32_t kDmaCount = 0xffffffffu;  // ~1 h at 1.2M edges/s (10 Mb/s, 1 frame/ms)
    static constexpr uint kRingBits = 15;  // 32 KB ring = 8192 edges, ~7 frames at 10 Mb/s
    static constexpr uint32_t kRingLen = (1u << kRingBits) / sizeof(uint32_t);

    PIO pio_;
    uint sm_;
    uint dma_;
    alignas(1u << kRingBits) uint32_t ring_[kRingLen];  // DMA ring must be size aligned

    uint32_t read_ = 0;  // words consumed
    volatile uint32_t overruns_ = 0;
    bool have_marker_ = false;
    bool level_ = false;  // level after the last decoded edge
    uint32_t last_x_ = 0xffffffffu;
    uint32_t wraps_ = 0;
    uint64_t decrements_ = 0;
    uint64_t edges_ = 0;

    bool burst_open_ = false;
    uint64_t burst_start_ = 0;
    uint32_t burst_first_edge_ = 0;  // edge index (low 32 bits) of its first edge
};

#endif // PROBE_EDGE_TIMER_H
