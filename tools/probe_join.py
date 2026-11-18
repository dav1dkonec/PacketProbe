#!/usr/bin/env python3
"""Joins a PacketProbe run with the PacketReceiver CSV of the same run.

  probe_join.py probe.csv rx.csv -o joined.csv [--size 100]

Pairing: the k-th matched probe row (a frame followed by a GPIO toggle) is the
k-th valid receiver row (each received datagram toggles the GPIO once).
Checked, not assumed:
  - seq - index is the same for every pair (the probe index counts frames in the
    length window, the sender's seq counts packets: both +1 per test packet);
    a shift means a frame or a toggle the other side did not see
  - unmatched probe frames (no toggle) have no receiver row: lost packets
  - frame_ns fits the payload size (10BASE-T: (8 + max(64, 46 + size)) x 800 ns)
Exit status 1 if a check fails.
"""

import argparse
import csv
import statistics
import sys

PICO_HEADER = ["index", "start_ns", "end_ns", "gpio_ns", "frame_ns", "latency_ns", "edges"]
RX_HEADER = ["index", "seq", "kernel_ns", "user_ns"]
OUT_HEADER = ["seq", "probe_index", "start_ns", "frame_ns", "latency_ns", "kernel_ns", "user_ns"]


def read_csv(path, header):
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    if not rows or rows[0] != header:
        sys.exit(f"{path}: expected header {','.join(header)}")
    return [dict(zip(header, r)) for r in rows[1:] if r]


def frame_ns(size):
    """Preamble start to FCS end on 10BASE-T: 8 B preamble + frame (min 64 B)."""
    return (8 + max(64, 14 + 20 + 8 + size + 4)) * 800  # Ethernet + IPv4 + UDP headers, FCS


def percentile(sorted_values, p):
    return sorted_values[min(len(sorted_values) - 1, int(p / 100 * len(sorted_values)))]


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("probe_csv", help="probe_log.py live/dump output")
    p.add_argument("rx_csv", help="latency_rx -o output")
    p.add_argument("-o", "--output", help="joined CSV, one row per pair")
    p.add_argument("--size", type=int, default=100, help="sender UDP payload (-s), default 100")
    p.add_argument("--tolerance", type=int, default=1000, help="allowed frame_ns deviation, ns (default 1000)")
    args = p.parse_args()

    probe = read_csv(args.probe_csv, PICO_HEADER)
    rx_all = read_csv(args.rx_csv, RX_HEADER)
    rx = [r for r in rx_all if int(r["seq"]) >= 0]  # seq -1: not a test packet (no frame in the window)
    invalid_rx = len(rx_all) - len(rx)
    matched = [r for r in probe if r["latency_ns"]]
    unmatched = [r for r in probe if not r["latency_ns"]]
    if not matched or not rx:
        sys.exit(f"nothing to pair: {len(matched)} matched probe rows, {len(rx)} valid receiver rows")

    failed = False
    pairs = list(zip(matched, rx))
    offset = int(rx[0]["seq"]) - int(matched[0]["index"])
    shifted = [(m, r) for m, r in pairs if int(r["seq"]) - int(m["index"]) != offset]
    if shifted:
        failed = True
        m, r = shifted[0]
        print(f"FAIL pairing: {len(shifted)} pairs off, first: probe index {m['index']} "
              f"with seq {r['seq']} (expected seq {int(m['index']) + offset})")
    if len(matched) != len(rx):
        print(f"note: {len(matched)} matched probe rows, {len(rx)} valid receiver rows: "
              f"the last {abs(len(matched) - len(rx))} are unpaired (run ends not aligned)")

    rx_seqs = {int(r["seq"]) for r in rx}
    seen_lost = [r for r in unmatched if int(r["index"]) + offset in rx_seqs]
    if seen_lost:
        failed = True
        print(f"FAIL: {len(seen_lost)} frames without a GPIO toggle were received anyway, "
              f"first probe index {seen_lost[0]['index']}")

    expected = frame_ns(args.size)
    bad_len = [m for m, _ in pairs if abs(int(m["frame_ns"]) - expected) > args.tolerance]
    if bad_len:
        failed = True
        print(f"FAIL frame length: {len(bad_len)} frames not {expected} +- {args.tolerance} ns, "
              f"first: index {bad_len[0]['index']}, {bad_len[0]['frame_ns']} ns")

    if args.output:
        with open(args.output, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(OUT_HEADER)
            for m, r in pairs:
                w.writerow([r["seq"], m["index"], m["start_ns"], m["frame_ns"], m["latency_ns"],
                            r["kernel_ns"], r["user_ns"]])

    lat = sorted(int(m["latency_ns"]) for m, _ in pairs)
    frames = [int(m["frame_ns"]) for m, _ in pairs]
    print(f"pairs {len(pairs)}, seq {rx[0]['seq']}..{rx[len(pairs) - 1]['seq']}, "
          f"lost (no toggle) {len(unmatched)}, invalid datagrams {invalid_rx}")
    print(f"frame ns {min(frames)}-{max(frames)} (expected {expected})")
    print(f"latency ns min {lat[0]} p50 {percentile(lat, 50)} p99 {percentile(lat, 99)} "
          f"p99.9 {percentile(lat, 99.9)} max {lat[-1]} mean {statistics.fmean(lat):.0f} "
          f"stdev {statistics.pstdev(lat):.0f}")
    if args.output:
        print(f"-> {args.output}")
    print("FAIL" if failed else "OK")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
