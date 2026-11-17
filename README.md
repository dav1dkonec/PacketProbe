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

Simulation (`kSimulate` in `src/main.cpp`): the Pico generates both signals
itself, jumpers GP4 → GP2 and GP5 → GP3. A frame of 57.6 us every 1 ms, GPIO
toggle 25 us after its end.

## Build and flash

```sh
export PICO_SDK_PATH=<path>/pico-sdk
cmake -B build -G Ninja && cmake --build build
picotool load -f -x build/probe.uf2      # first time: hold BOOTSEL while plugging in
```

## Output (USB serial, /dev/cu.usbmodem*)

```
index,start_ns,end_ns,gpio_ns,frame_ns,latency_ns,edges
3698,3698132000,3698189552,3698214560,57552,25008,1200
# frames 1000 matched 1000 unmatched 0, frame ns 57552-57552, latency ns min 25008 avg 25008 max 25008, ...
```

Times are ns since start. `latency_ns` = GPIO edge − last edge of the frame;
empty for a frame without a GPIO edge within 10 ms. Lines starting with `#` are
a summary once a second.

## Known limits

- The GPIO channel needs an edge at least every ~68 s (32-bit PIO counter).
- One run lasts at most ~1 h at 1000 frames/s (DMA transfer count).
- `index` counts all frames on the wire, including ARP.
