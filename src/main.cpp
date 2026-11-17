// PacketProbe: wire-to-GPIO latency.
// GP2 = wire activity (comparator), GP3 = Pi GPIO17. Core1 timestamps and
// pairs, core0 prints one CSV line per frame and a '#' summary every second.
// kSimulate: the Pico makes both signals itself (jumpers GP4 -> GP2, GP5 -> GP3).

#include <algorithm>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <initializer_list>

#include "edge_timer.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "matcher.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "sim.pio.h"

namespace {

constexpr uint kPinWire = 2;
constexpr uint kPinGpio = 3;

constexpr bool kSimulate = true;
constexpr uint kPinSimWire = 4;
constexpr uint kPinSimGpio = 5;
constexpr uint32_t kSimPeriods = 600;          // 600 x 12 cycles = 57.6 us, like 72 B at 10 Mb/s
constexpr uint32_t kSimFrameCycles = 125'000;  // 1 ms at 125 MHz
constexpr uint32_t kSimDelayCycles = 3125;     // 25 us after the frame end

// matcher limits
constexpr uint64_t kGapNs = 300;          // Manchester has an edge at least every 100 ns
constexpr uint64_t kMinFrameNs = 2'000;   // link pulses are ~100 ns
constexpr uint64_t kTimeoutNs = 10'000'000;
constexpr uint64_t kNowMarginUs = 5;      // keeps "now" behind the PIO timers
constexpr uint64_t kBoundMarginUs = 1000; // keeps the bound ahead of every edge

uint32_t g_sys_hz;
uint64_t g_start_us;
EdgeTimer* g_wire;
EdgeTimer* g_gpio;
Matcher* g_matcher;
queue_t g_results;
volatile uint32_t g_queue_full = 0;
uint64_t g_gap_cycles;

uint64_t ns_to_cycles(uint64_t ns) { return ns * g_sys_hz / 1'000'000'000; }
uint64_t cycles_to_ns(uint64_t c) { return c * 1'000'000'000 / g_sys_hz; }  // exact at 125 MHz

// in RAM: flash (XIP cache shared with core0 and USB) is too slow for ~1.2M edges/s
void __not_in_flash_func(core1_main)()
{
    for (;;) {
        // "now" first: every wire edge before it is in the DMA ring already
        const uint64_t us = time_us_64() - g_start_us;
        const uint64_t now = us > kNowMarginUs ? (us - kNowMarginUs) * (g_sys_hz / 1'000'000) : 0;
        const uint64_t bound = (us + kBoundMarginUs) * (g_sys_hz / 1'000'000);

        const auto on_frame = [](uint64_t start, uint64_t end, uint32_t edges) {
            g_matcher->frame(start, end, edges);
        };
        g_wire->drain_bursts(static_cast<uint32_t>(g_gap_cycles / 2), bound, on_frame);
        g_wire->close_idle_burst(now, g_gap_cycles, on_frame);
        g_gpio->drain(bound, [](uint64_t t, bool) { g_matcher->gpio_edge(t); });

        uint64_t safe;
        if (!g_wire->open_burst(safe)) {
            safe = now > g_gap_cycles ? now - g_gap_cycles : 0;
        }
        g_matcher->flush(safe, [](const Result& r) {
            if (!queue_try_add(&g_results, &r)) {
                g_queue_full = g_queue_full + 1;
            }
        });
    }
}

/// Drives a simulator output alone and checks that only its input follows;
/// @p followers gets the pins that follow it.
bool check_wiring(uint out, uint in, uint other_in, uint32_t& followers)
{
    gpio_init(out);
    gpio_set_dir(out, GPIO_OUT);
    gpio_put(out, true);
    sleep_us(10);
    const uint32_t high = gpio_get_all();
    gpio_put(out, false);
    sleep_us(10);
    const uint32_t low = gpio_get_all();
    followers = (high & ~low) & ~(1u << out);
    return (followers >> in & 1u) && !(followers >> other_in & 1u);
}

void start_simulation()
{
    const uint off_frame = pio_add_program(pio1, &sim_frame_program);
    const uint off_gpio = pio_add_program(pio1, &sim_gpio_program);
    const uint sm_frame = pio_claim_unused_sm(pio1, true);
    const uint sm_gpio = pio_claim_unused_sm(pio1, true);
    sim_frame_program_init(pio1, sm_frame, off_frame, kPinSimWire, kSimPeriods,
                           kSimFrameCycles - 12 * kSimPeriods - 4);
    sim_gpio_program_init(pio1, sm_gpio, off_gpio, kPinSimGpio, kSimDelayCycles - 10);
    pio_enable_sm_mask_in_sync(pio1, (1u << sm_frame) | (1u << sm_gpio));
}

struct Summary {
    uint32_t matched = 0;
    uint32_t unmatched = 0;
    uint64_t len_min = UINT64_MAX;
    uint64_t len_max = 0;
    uint64_t lat_min = UINT64_MAX;
    uint64_t lat_max = 0;
    uint64_t lat_sum = 0;
};

} // namespace

int main()
{
    stdio_init_all();
    g_sys_hz = clock_get_hz(clk_sys);

    bool wiring_ok = true;
    uint32_t follow_wire = 0;
    uint32_t follow_gpio = 0;
    if (kSimulate) {
        for (uint pin : {kPinWire, kPinGpio}) {
            gpio_init(pin);
            gpio_pull_down(pin);
        }
        const bool ok_wire = check_wiring(kPinSimWire, kPinWire, kPinGpio, follow_wire);
        const bool ok_gpio = check_wiring(kPinSimGpio, kPinGpio, kPinWire, follow_gpio);
        wiring_ok = ok_wire && ok_gpio;
    }

    static EdgeTimer wire(pio0, kPinWire);  // static: 32 KB DMA rings
    static EdgeTimer gpio(pio0, kPinGpio);
    g_gap_cycles = ns_to_cycles(kGapNs);
    static Matcher matcher({ns_to_cycles(kMinFrameNs), ns_to_cycles(kTimeoutNs)});
    g_wire = &wire;
    g_gpio = &gpio;
    g_matcher = &matcher;
    queue_init(&g_results, sizeof(Result), 256);

    // one time base for both channels
    pio_enable_sm_mask_in_sync(pio0, wire.sm_mask() | gpio.sm_mask());
    g_start_us = time_us_64();
    multicore_launch_core1(core1_main);

    if (kSimulate && wiring_ok) {
        start_simulation();
    }

    Summary s;
    uint64_t next_report = time_us_64() + 1'000'000;
    for (;;) {
        Result r;
        while (queue_try_remove(&g_results, &r)) {
            const uint64_t len = cycles_to_ns(r.end - r.start);
            s.len_min = std::min(s.len_min, len);
            s.len_max = std::max(s.len_max, len);
            if (r.matched) {
                const uint64_t lat = cycles_to_ns(r.gpio - r.end);
                ++s.matched;
                s.lat_min = std::min(s.lat_min, lat);
                s.lat_max = std::max(s.lat_max, lat);
                s.lat_sum += lat;
                std::printf("%" PRIu32 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64
                            ",%" PRIu32 "\n",
                            r.index, cycles_to_ns(r.start), cycles_to_ns(r.end), cycles_to_ns(r.gpio), len,
                            lat, r.edges);
            } else {
                ++s.unmatched;
                std::printf("%" PRIu32 ",%" PRIu64 ",%" PRIu64 ",,%" PRIu64 ",,%" PRIu32 "\n", r.index,
                            cycles_to_ns(r.start), cycles_to_ns(r.end), len, r.edges);
            }
        }

        if (time_us_64() >= next_report) {
            next_report += 1'000'000;
            std::printf("# index,start_ns,end_ns,gpio_ns,frame_ns,latency_ns,edges\n");
            if (kSimulate) {
                std::printf("# simulation, wiring GP%u->GP%u GP%u->GP%u %s (pins following GP%u: 0x%08lx, GP%u: 0x%08lx)\n",
                            kPinSimWire, kPinWire, kPinSimGpio, kPinGpio,
                            wiring_ok ? "ok" : "BAD, no signals generated", kPinSimWire,
                            static_cast<unsigned long>(follow_wire), kPinSimGpio,
                            static_cast<unsigned long>(follow_gpio));
            }
            std::printf("# uptime ns %" PRIu64 "\n", (time_us_64() - g_start_us) * 1000);
            std::printf("# frames %" PRIu32 " matched %" PRIu32 " unmatched %" PRIu32,
                        s.matched + s.unmatched, s.matched, s.unmatched);
            if (s.matched + s.unmatched > 0) {
                std::printf(", frame ns %" PRIu64 "-%" PRIu64, s.len_min, s.len_max);
            }
            if (s.matched > 0) {
                std::printf(", latency ns min %" PRIu64 " avg %" PRIu64 " max %" PRIu64, s.lat_min,
                            s.lat_sum / s.matched, s.lat_max);
            }
            std::printf(", link pulses %" PRIu32 ", stray gpio %" PRIu32 ", overruns %" PRIu32 "/%" PRIu32
                        ", dropped %" PRIu32 "/%" PRIu32 "\n",
                        static_cast<uint32_t>(matcher.link_pulses), static_cast<uint32_t>(matcher.stray_edges),
                        wire.overruns(), gpio.overruns(), static_cast<uint32_t>(matcher.dropped),
                        static_cast<uint32_t>(g_queue_full));
            s = {};
        }
    }
}
