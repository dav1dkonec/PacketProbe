// PacketProbe: wire-to-GPIO latency.
// GP2 = wire activity (comparator), GP3 = Pi GPIO17. Core1 timestamps and
// pairs; core0 handles USB: results as CSV and text commands (type "help").

#include <algorithm>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
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

// USB text protocol (README "Protocol"): bump on any change of commands,
// CSV columns or the '#' markers tools/probe_log.py waits for
constexpr int kProtocolVersion = 2;

constexpr uint kPinWire = 2;
constexpr uint kPinGpio = 3;

// simulator ("sim on"): jumpers GP4 -> GP2, GP5 -> GP3
constexpr uint kPinSimWire = 4;
constexpr uint kPinSimGpio = 5;
constexpr uint32_t kSimPeriods = 600;          // 600 x 12 cycles = 57.6 us, like 72 B at 10 Mb/s
constexpr uint32_t kSimFrameCycles = 125'000;  // 1 ms at 125 MHz
constexpr uint32_t kSimDelayCycles = 3125;     // 25 us after the frame end

// matcher limits
constexpr uint64_t kGapNs = 300;           // Manchester has an edge at least every 100 ns
constexpr uint64_t kMinFrameNs = 2'000;    // link pulses are ~100 ns
constexpr uint64_t kTimeoutNs = 10'000'000;
constexpr uint64_t kNowMarginUs = 5;       // keeps "now" behind the PIO timers
constexpr uint64_t kBoundMarginUs = 1000;  // keeps the bound ahead of every edge

// DMA rings: aligned to their size; the wire takes up to ~1.2M edges/s
constexpr uint kWireRingBits = 15;  // 32 KB = 8192 edges, ~7 frames
constexpr uint kGpioRingBits = 12;  // 4 KB = 1024 edges, ~1 s at 1000 packets/s
alignas(1u << kWireRingBits) uint32_t g_wire_ring[(1u << kWireRingBits) / 4];
alignas(1u << kGpioRingBits) uint32_t g_gpio_ring[(1u << kGpioRingBits) / 4];

/// Deferred mode: results stay here until "dump", nothing goes over USB
/// while measuring. 16 B each; start has us resolution, the rest is exact.
struct Record {
    uint32_t index;
    uint32_t start_us;
    uint32_t frame_cycles;
    uint32_t latency_cycles;  // kNoEdge: no GPIO edge
};
constexpr uint32_t kNoEdge = 0xffffffffu;
constexpr uint32_t kRecordCapacity = 10'000;
Record g_records[kRecordCapacity];

uint32_t g_sys_hz;
uint64_t g_start_us;
uint64_t g_gap_cycles;
EdgeTimer* g_wire;
EdgeTimer* g_gpio;
Matcher* g_matcher;
queue_t g_results;
volatile uint32_t g_queue_full = 0;
volatile bool g_reset_request = false;  // core0 -> core1
constexpr uint32_t kResetMarker = UINT32_MAX;  // Result::edges of the marker queued at a reset

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

        if (g_reset_request) {
            g_matcher->reset(now);
            // results queued before this one belong to the old run
            const Result marker{false, 0, 0, 0, 0, kResetMarker};
            queue_add_blocking(&g_results, &marker);
            g_reset_request = false;
        }

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

// ---------------------------------------------------------------------------
// simulator
// ---------------------------------------------------------------------------

struct Sim {
    bool loaded = false;
    bool running = false;
    uint off_frame = 0, off_gpio = 0, sm_frame = 0, sm_gpio = 0;
};
Sim g_sim;

/// Drives a simulator output alone; true if only its input follows.
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
    gpio_set_dir(out, GPIO_IN);
    followers = (high & ~low) & ~(1u << out);
    return (followers >> in & 1u) && !(followers >> other_in & 1u);
}

void sim_start()
{
    if (g_sim.running) {
        std::printf("# sim already on\n");
        return;
    }
    // refuses when something else drives the inputs (Pi, comparator)
    uint32_t follow_wire = 0;
    uint32_t follow_gpio = 0;
    const bool ok = check_wiring(kPinSimWire, kPinWire, kPinGpio, follow_wire) &&
                    check_wiring(kPinSimGpio, kPinGpio, kPinWire, follow_gpio);
    if (!ok) {
        std::printf("# sim: wiring BAD, need jumpers GP%u->GP%u and GP%u->GP%u only "
                    "(pins following GP%u: 0x%08lx, GP%u: 0x%08lx)\n",
                    kPinSimWire, kPinWire, kPinSimGpio, kPinGpio, kPinSimWire,
                    static_cast<unsigned long>(follow_wire), kPinSimGpio,
                    static_cast<unsigned long>(follow_gpio));
        return;
    }
    if (!g_sim.loaded) {
        g_sim.off_frame = pio_add_program(pio1, &sim_frame_program);
        g_sim.off_gpio = pio_add_program(pio1, &sim_gpio_program);
        g_sim.sm_frame = pio_claim_unused_sm(pio1, true);
        g_sim.sm_gpio = pio_claim_unused_sm(pio1, true);
        g_sim.loaded = true;
    }
    sim_frame_program_init(pio1, g_sim.sm_frame, g_sim.off_frame, kPinSimWire, kSimPeriods,
                           kSimFrameCycles - 12 * kSimPeriods - 4);
    sim_gpio_program_init(pio1, g_sim.sm_gpio, g_sim.off_gpio, kPinSimGpio, kSimDelayCycles - 10);
    pio_enable_sm_mask_in_sync(pio1, (1u << g_sim.sm_frame) | (1u << g_sim.sm_gpio));
    g_sim.running = true;
    std::printf("# sim on: frame 57552 ns every 1 ms, GPIO toggle ~25 us after its end\n");
}

void sim_stop()
{
    if (g_sim.running) {
        pio_set_sm_mask_enabled(pio1, (1u << g_sim.sm_frame) | (1u << g_sim.sm_gpio), false);
        for (uint pin : {kPinSimWire, kPinSimGpio}) {
            gpio_init(pin);  // back to a plain input: nothing driven
        }
        g_sim.running = false;
    }
    std::printf("# sim off\n");
}

// ---------------------------------------------------------------------------
// results
// ---------------------------------------------------------------------------

struct Summary {
    uint32_t matched = 0;
    uint32_t unmatched = 0;
    uint64_t len_min = UINT64_MAX;
    uint64_t len_max = 0;
    uint64_t lat_min = UINT64_MAX;
    uint64_t lat_max = 0;
    uint64_t lat_sum = 0;

    void add(uint64_t len_ns, bool matched_, uint64_t lat_ns)
    {
        len_min = std::min(len_min, len_ns);
        len_max = std::max(len_max, len_ns);
        if (!matched_) {
            ++unmatched;
            return;
        }
        ++matched;
        lat_min = std::min(lat_min, lat_ns);
        lat_max = std::max(lat_max, lat_ns);
        lat_sum += lat_ns;
    }
};

bool g_deferred = false;
uint32_t g_record_count = 0;
uint32_t g_record_lost = 0;
Summary g_interval;  // live mode: printed every second
Summary g_total;     // since "reset"

void print_csv_header()
{
    std::printf("index,start_ns,end_ns,gpio_ns,frame_ns,latency_ns,edges\n");
}

/// @p lat_ns / @p edges: UINT64_MAX = empty field
void print_csv(uint32_t index, uint64_t start_ns, uint64_t frame_ns, uint64_t lat_ns, uint64_t edges)
{
    const uint64_t end_ns = start_ns + frame_ns;
    std::printf("%" PRIu32 ",%" PRIu64 ",%" PRIu64 ",", index, start_ns, end_ns);
    if (lat_ns != UINT64_MAX) {
        std::printf("%" PRIu64, end_ns + lat_ns);
    }
    std::printf(",%" PRIu64 ",", frame_ns);
    if (lat_ns != UINT64_MAX) {
        std::printf("%" PRIu64, lat_ns);
    }
    std::printf(",");
    if (edges != UINT64_MAX) {
        std::printf("%" PRIu64, edges);
    }
    std::printf("\n");
}

void print_summary(const char* what, const Summary& s)
{
    std::printf("# %s: uptime ns %" PRIu64 ", frames %" PRIu32 " matched %" PRIu32 " unmatched %" PRIu32,
                what, (time_us_64() - g_start_us) * 1000, s.matched + s.unmatched, s.matched, s.unmatched);
    if (s.matched + s.unmatched > 0) {
        std::printf(", frame ns %" PRIu64 "-%" PRIu64, s.len_min, s.len_max);
    }
    if (s.matched > 0) {
        std::printf(", latency ns min %" PRIu64 " avg %" PRIu64 " max %" PRIu64, s.lat_min,
                    s.lat_sum / s.matched, s.lat_max);
    }
    std::printf(", other frames %" PRIu32 ", link pulses %" PRIu32 ", stray gpio %" PRIu32
                ", overruns %" PRIu32 "/%" PRIu32 ", dropped %" PRIu32 "/%" PRIu32 "\n",
                static_cast<uint32_t>(g_matcher->other_frames),
                static_cast<uint32_t>(g_matcher->link_pulses), static_cast<uint32_t>(g_matcher->stray_edges),
                g_wire->overruns(), g_gpio->overruns(), static_cast<uint32_t>(g_matcher->dropped),
                static_cast<uint32_t>(g_queue_full));
}

void handle_result(const Result& r)
{
    const uint64_t len_ns = cycles_to_ns(r.end - r.start);
    const uint64_t lat_ns = r.matched ? cycles_to_ns(r.gpio - r.end) : UINT64_MAX;
    g_interval.add(len_ns, r.matched, lat_ns);
    g_total.add(len_ns, r.matched, lat_ns);

    if (!g_deferred) {
        print_csv(r.index, cycles_to_ns(r.start), len_ns, lat_ns, r.edges);
        return;
    }
    if (g_record_count == kRecordCapacity) {
        ++g_record_lost;
        return;
    }
    g_records[g_record_count++] = Record{
        r.index, static_cast<uint32_t>(cycles_to_ns(r.start) / 1000),
        static_cast<uint32_t>(r.end - r.start),
        r.matched ? static_cast<uint32_t>(r.gpio - r.end) : kNoEdge};
}

void reset_results()
{
    g_reset_request = true;
    Result r;
    do {
        queue_remove_blocking(&g_results, &r);  // old run, up to the marker
    } while (r.edges != kResetMarker);
    g_record_count = 0;
    g_record_lost = 0;
    g_interval = {};
    g_total = {};
    std::printf("# reset: frame index starts at 0\n");
}

void drain_results()
{
    Result r;
    while (queue_try_remove(&g_results, &r)) {
        handle_result(r);
    }
}

void dump_records()
{
    print_csv_header();
    const uint32_t count = g_record_count;  // later results are kept for the next dump
    for (uint32_t i = 0; i < count; ++i) {
        if (i % 64 == 0) {
            drain_results();  // printing is slow, the queue must not overflow meanwhile
        }
        const Record& rec = g_records[i];
        print_csv(rec.index, uint64_t{rec.start_us} * 1000, cycles_to_ns(rec.frame_cycles),
                  rec.latency_cycles == kNoEdge ? UINT64_MAX : cycles_to_ns(rec.latency_cycles),
                  UINT64_MAX);
    }
    std::printf("# end dump: %" PRIu32 " records, %" PRIu32 " lost (buffer full)\n", count,
                g_record_lost);
}

// ---------------------------------------------------------------------------
// commands over USB, one per line
// ---------------------------------------------------------------------------

/// "frame MIN MAX" (ns) or "frame off"; without arguments prints the window.
void set_frame_window(const char* args)
{
    // strtoul, not sscanf: scanf from newlib doubles the firmware size
    char* end = nullptr;
    const unsigned long min_ns = std::strtoul(args, &end, 10);
    const bool got_min = end != args;
    const char* rest = end;
    const unsigned long max_ns = std::strtoul(rest, &end, 10);
    const bool parsed = got_min && end != rest && *end == '\0';
    if (std::strcmp(args, " off") == 0) {
        g_matcher->window_min = 0;
        g_matcher->window_max = UINT32_MAX;
    } else if (parsed && min_ns <= max_ns && max_ns <= 10'000'000) {
        // two stores: a frame decided in between sees one old and one new bound
        g_matcher->window_max = static_cast<uint32_t>(ns_to_cycles(max_ns));
        g_matcher->window_min = static_cast<uint32_t>(ns_to_cycles(min_ns));
    } else if (args[0] != '\0') {
        std::printf("# usage: frame MIN MAX (ns, MIN <= MAX <= 10000000) | frame off\n");
        return;
    }
    if (g_matcher->window_max == UINT32_MAX) {
        std::printf("# frame window off: all frames\n");
    } else {
        std::printf("# frame window %" PRIu64 "-%" PRIu64 " ns\n", cycles_to_ns(g_matcher->window_min),
                    cycles_to_ns(g_matcher->window_max));
    }
}

void print_help()
{
    std::printf("# commands:\n"
                "#   version         firmware name and protocol version\n"
                "#   status          totals since reset\n"
                "#   reset           clear results, frame index from 0\n"
                "#   live            CSV line per frame + summary every second (default)\n"
                "#   deferred        keep up to %lu results in RAM, no USB output while measuring\n"
                "#   dump            print the kept results as CSV, ends with '# end dump'\n"
                "#   (a run: deferred, reset, measure, dump)\n"
                "#   sim on|off      simulated frame + GPIO on GP%u/GP%u (jumpers to GP%u/GP%u)\n"
                "#   frame MIN MAX   match only frames MIN-MAX ns long (test packets), others\n"
                "#                   are counted as 'other frames'; 'frame off': all frames\n",
                static_cast<unsigned long>(kRecordCapacity), kPinSimWire, kPinSimGpio, kPinWire,
                kPinGpio);
}

void handle_command(const char* cmd)
{
    if (std::strcmp(cmd, "version") == 0) {
        std::printf("# version PacketProbe protocol %d\n", kProtocolVersion);
    } else if (std::strcmp(cmd, "status") == 0) {
        print_summary(g_deferred ? "deferred" : "live", g_total);
        if (g_deferred) {
            std::printf("# kept %" PRIu32 "/%lu, lost %" PRIu32 "\n", g_record_count,
                        static_cast<unsigned long>(kRecordCapacity), g_record_lost);
        }
    } else if (std::strcmp(cmd, "reset") == 0) {
        reset_results();
    } else if (std::strcmp(cmd, "live") == 0) {
        g_deferred = false;
        std::printf("# mode live\n");
        print_csv_header();
    } else if (std::strcmp(cmd, "deferred") == 0) {
        g_deferred = true;
        std::printf("# mode deferred\n");
    } else if (std::strcmp(cmd, "dump") == 0) {
        dump_records();
    } else if (std::strcmp(cmd, "sim on") == 0) {
        sim_start();
    } else if (std::strcmp(cmd, "sim off") == 0) {
        sim_stop();
    } else if (std::strncmp(cmd, "frame", 5) == 0) {
        set_frame_window(cmd + 5);
    } else if (std::strcmp(cmd, "help") == 0) {
        print_help();
    } else if (cmd[0] != '\0') {
        std::printf("# unknown command '%s', try help\n", cmd);
    }
}

/// Collects characters from USB without blocking; runs complete lines.
void poll_commands()
{
    static char line[32];
    static uint32_t len = 0;
    for (int c; (c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT;) {
        if (c == '\n' || c == '\r') {
            line[len] = '\0';
            handle_command(line);
            len = 0;
        } else if (len < sizeof line - 1) {
            line[len++] = static_cast<char>(c);
        }
    }
}

} // namespace

int main()
{
    stdio_init_all();
    g_sys_hz = clock_get_hz(clk_sys);
    g_gap_cycles = ns_to_cycles(kGapNs);

    static EdgeTimer wire(pio0, kPinWire, g_wire_ring, kWireRingBits);
    static EdgeTimer gpio(pio0, kPinGpio, g_gpio_ring, kGpioRingBits);
    static Matcher matcher({ns_to_cycles(kMinFrameNs), ns_to_cycles(kTimeoutNs)});
    g_wire = &wire;
    g_gpio = &gpio;
    g_matcher = &matcher;
    queue_init(&g_results, sizeof(Result), 256);

    // one time base for both channels
    pio_enable_sm_mask_in_sync(pio0, wire.sm_mask() | gpio.sm_mask());
    g_start_us = time_us_64();
    multicore_launch_core1(core1_main);

    uint64_t next_report = time_us_64() + 1'000'000;
    for (;;) {
        poll_commands();

        drain_results();

        if (time_us_64() >= next_report) {
            next_report += 1'000'000;
            if (!g_deferred) {
                print_summary("last second", g_interval);
            }
            g_interval = {};
        }
    }
}
