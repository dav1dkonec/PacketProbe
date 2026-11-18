# PacketProbe

Raspberry Pi Pico (RP2040) firmware measuring wire-to-GPIO latency of
[PacketReceiver](https://github.com/dav1dkonec/PacketReceiver): it timestamps each
Ethernet frame on the wire and the receiver's GPIO toggle with PIO
(16 ns resolution at 125 MHz), pairs them and prints one CSV line per frame over USB.

## Pins

| Pico | Signal |
|---|---|
| GP2 (pin 4) | wire activity: current transformer → TLV3501 comparator (10BASE-T) |
| GP3 (pin 5) | Pi GPIO17 (pin 11), through ~330 Ω |
| GND (pin 3) | Pi GND (pin 6) |

Simulation (`sim on`): the Pico generates both signals itself, jumpers
GP4 → GP2 and GP5 → GP3: a frame of 57.6 us every 1 ms, GPIO toggle 25 us after
its end. It refuses to start if anything else drives GP2/GP3.

## Build and flash

```sh
export PICO_SDK_PATH=<path>/pico-sdk
cmake -B build -G Ninja && cmake --build build
picotool load -f -x build/probe.uf2      # first time: hold BOOTSEL while plugging in
```

## Protocol (USB serial)

The Pico is a USB CDC serial port: `/dev/ttyACM0` on the Pi, `/dev/cu.usbmodem*`
on a Mac (the baud rate is ignored). Text, one line per command or message.
`tools/probe_log.py` relies on everything below; change both together and bump
`kProtocolVersion` (firmware) and `PROTOCOL` (script).

**Commands** (host → Pico, terminated by `\n`):

| command | reply |
|---|---|
| `version` | `# version PacketProbe protocol 2` |
| `status` | `# live: ...` or `# deferred: ...` totals since reset (+ `# kept N/10000, lost M`) |
| `reset` | `# reset: frame index starts at 0`; frames that started earlier are not reported |
| `live` | `# mode live`, the CSV header, then a CSV line per frame (default mode) |
| `deferred` | `# mode deferred`; results stay in RAM (up to 10 000), nothing is sent |
| `dump` | the CSV header, the kept results, `# end dump: N records, M lost (buffer full)` |
| `frame MIN MAX` / `frame off` | `# frame window MIN-MAX ns` / `# frame window off: all frames`; only frames of this length (ns) are paired, see below |
| `sim on` / `sim off` | `# sim on: ...` / `# sim off`, or `# sim: wiring BAD ...` |
| `help` | `# commands: ...` |

**Lines** (Pico → host), told apart by their first character:

- digit: a result, `index,start_ns,end_ns,gpio_ns,frame_ns,latency_ns,edges`
- `index,...`: the CSV header
- `#`: a status message; in live mode also `# last second: ...` once a second

Times are ns since the Pico started. `latency_ns` = GPIO edge − last edge of
the frame; `gpio_ns` and `latency_ns` are empty for a frame without a GPIO edge
within 10 ms. In deferred mode `start_ns` has us resolution and `edges` is empty.

## Logger

`tools/probe_log.py` (Python 3, no dependencies) on the machine the Pico is
plugged into, e.g. the Pi:

```sh
tools/probe_log.py live -o run.csv --seconds 15   # stream
tools/probe_log.py arm                            # deferred + reset
# ... run sender and receiver ...
tools/probe_log.py dump -o run.csv                # fetch the run
tools/probe_log.py cmd sim on                     # any command
```

On the Pi, keep the logger away from the receiver's CPU (`taskset -c 1`): USB
traffic of the probe runs on the device under test. `arm`/`dump` avoids it
during the measurement.

## Known limits

- One run lasts at most ~1 h at 1000 frames/s (DMA transfer count).
- Without a frame window, `index` counts all frames on the wire, including ARP.

## Test frames

The sender pads each packet to a fixed UDP payload (`-s`, default 100 B), so its
frames have a length no other traffic is likely to have. On 10BASE-T a frame
lasts (8 + max(64, 46 + payload)) × 0.8 us: 123.2 us for 100 B, 57.6 us for ARP.
`frame 122000 125000` pairs only test frames; the rest is counted as
`other frames` in `status`. The window stays set across `reset` until `frame off`
or a reboot (default: off).
