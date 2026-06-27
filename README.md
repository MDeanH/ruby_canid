# ruby_canid

**Offline decode + identify for almost any CAN bus.** A single-file Python tool
(`canid.py`) that takes a frame ID + data bytes (or a whole `candump` log) and tells
you what it most likely is — using car DBCs, embedded J1939/OBD-II tables, a set of
known robotics/motor protocols, and a heuristic classifier for buses it has never
seen before.

This is personal reverse-engineering tooling, built for my own projects (an MX-5
dash and a Segway-Ninebot ServeBot S1 chassis). It runs entirely offline and is
designed to stay memory-safe on a 1 GB Raspberry Pi.

See **[LESSONS.md](LESSONS.md)** for write-ups of what reverse-engineering these
two buses actually taught me (verified-on-hardware vs. spec, finding signals by
diffing, bus-limit gotchas, and a listen-only-first workflow).

## What it does

- **DBC-backed decode** — indexes a directory of `*.dbc` files (e.g. comma.ai
  [opendbc](https://github.com/commaai/opendbc) car databases) by frame ID and
  decodes against them on demand. The index is cached to `db/dbc_index.json` and
  DBCs are loaded lazily through a small LRU, so all databases never sit in RAM at
  once.
- **Embedded protocol tables** — a built-in J1939 common PGN/SPN table and OBD-II
  mode-01 PIDs, so common heavy-vehicle and diagnostic frames decode with no DBC.
- **Protocol profiles** — `auto`, `car`, `servebot`, `robomaster`, `cybergear`,
  `canopen`, `vesc`, `j1939`, `obd`. The `servebot` profile is a
  reverse-engineered telemetry map for the Ninebot ServeBot S1 chassis.
- **Unknown-bus analysis** — `analyze` runs a heuristic byte-classifier over a
  capture to characterise IDs whose meaning isn't in any database (cycle time,
  byte entropy, counters, likely signal layout).

## Usage

```
canid.py index                          # (re)build the DBC index from ~/canid/db
canid.py dbcs                           # list databases + coverage
canid.py decode <IDhex> <DATAhex> [--ext] [--profile P]
canid.py live   [--channel can0] [--seconds N] [--bitrate B] [--profile P]
canid.py analyze <candump.log>          # heuristic identify of an unknown bus
```

`--ext` selects 29-bit (extended) IDs; `--profile` forces a protocol instead of
auto-detect.

## Databases

DBC files live in `~/canid/db/` and are **not** bundled here — drop your own in.
opendbc DBCs are MIT-licensed and sourced separately; the embedded J1939/OBD-II
tables are in `canid.py` itself.

## Requirements

- Python 3
- [`cantools`](https://github.com/cantools/cantools) (DBC decode)
- [`python-can`](https://github.com/hardbyte/python-can) (only for `live` capture)

## Notes

Decode is read-only — this tool identifies and decodes traffic, it does not
transmit. Decoded signal meanings are best-effort: a DBC or spec is a starting
point, and anything safety-relevant should be confirmed against the physical
hardware before you rely on it.
