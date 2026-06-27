# Lessons from reverse-engineering CAN buses

Notes from reverse-engineering two very different CAN buses on hardware I own — a
Mazda MX-5 ND ("Ruby") and a Segway-Ninebot ServeBot S1 self-balancing chassis.
These are the things that actually cost time or changed how I work, written down so
they're reusable.

The claims below are grounded in on-hardware measurement or the actual project
code, and I've tried to be explicit about what is **verified** versus what is still
**pending a capture**. When a measurement contradicts a spec, the measurement wins —
that single habit is the through-line of everything here.

## 1. Treat the spec as a hypothesis, not ground truth

Treat a community DBC, datasheet, or README as a starting template — confirm every
decoded signal against an observable physical reality before you trust it. Concrete
case: the MX-5 ND fuel level (CAN `0x9E`, byte 5) is encoded at **0.2 L/bit** in the
community DBC, but on-car a full ~45 L tank decoded to only ~80% at that scale.
Re-measuring against a known-full tank gave **0.25 L/bit** (full byte5 = 181 →
45.25 L ≈ tank size → 100%), which is the value I shipped. The gap is ~20% — small
enough to look plausible on a gauge, large enough to be wrong.
*(On-car verified, shipped.)*

## 2. Diff a baseline against an action to find a signal

To learn which frames drive a function, record a static baseline, perform the
action, then diff for IDs/bytes that appear or change **only** during the action —
and time-align both captures to a frame you know is stable.

On the MX-5 I have the baseline half of this in hand: `0x472` (RoofGraphicStatus, a
4-bit dash status flag) sat completely static across a full drive with the roof
closed (~1.34M frames), so it doubles as an untouched reference and a time-alignment
clock for a future roof-actuation diff. Worth stating plainly: `0x472` is the stable
clock you *align to*, not the frame whose moving bytes reveal the command — and the
roof command frames themselves are **still unisolated**, because the switch-held
on-car capture hasn't been run yet. The method and tooling are ready; the result is
pending. *(Baseline on-car verified; command capture pending.)*

## 3. Never assume a bus's bitrate or ID width — scan for them

A wrong guess about bitrate or ID width silently wastes hours. The two buses in this
project sit at opposite ends: the MX-5 ND high-speed CAN runs at **500 kbit/s with
11-bit standard IDs** (every frame in the project DBC falls in `0x40`–`0x7E9`),
while the ServeBot S1 chassis's internal CAN runs at **1 Mbit/s with 29-bit extended
IDs** (`0x0CB003xx` telemetry, found only by scanning common rates until 1 Mbit
produced frames). Concretely: leaving a shared USB-CAN adapter configured at 1 Mbit
after a bench session once clobbered the car's 500k config, leaving the interface
ERROR-PASSIVE and the dashboard gauges blank despite live traffic on the wire.
*(On-hardware verified.)*

## 4. Inject where the signal lives — and confirm it's on the bus at all

Inject where the target signal actually lives, and first confirm the signal is on
the bus at all. On the ND MX-5 there is **no** roof open/close command frame on
HS-CAN — only a 4-bit status field (`0x472`) — and a single injected frame is
overwritten within milliseconds because the roof module continuously re-transmits.
So OBD-II command injection will not move the roof: the roof command path simply
isn't exposed on HS-CAN. Real actuation lives down at the roof-switch harness and
needs an on-harness hardware approach rather than an OBD inject (the precise
form — filtering bridge vs single-node filter, plus likely analog switch-signal
emulation — is still research-inferred and pending an on-car capture).

The power windows aren't on HS-CAN at all: zero window frames appear in the DBC —
they ride a LIN bus / physical switch contacts, so no CAN frame of any kind will
drive them. *(No-command-frame, injected-frame-overwritten, and windows-not-on-CAN
all verified; the harness bridge form is inferred, not yet captured.)*

## 5. Know the bus's limits for anything safety-related

Before treating a state as reachable over the bus, prove it — and accept that some
states aren't reachable at all. On the ServeBot S1 there is **no CAN command that
frees the wheels**: the self-balance loop always holds (across 280+ current samples
the motor current never fell below ~164, and every by-hand push netted zero motion),
so "stop sending" does **not** stop the motors. The only real freewheel is cutting
motor power in hardware — a GPIO-driven relay/MOSFET (BCM GPIO26 / Pi physical pin
37, rated ≥40A DC for the hub-motor stall peak) on the motor feed, fail-safed so an
un-driven Pi leaves the motors powered and balancing. *(On-hardware verified; the
only firmware-side cut is the chassis's own tilt/fall-angle cutout — a physical
trigger, not a commandable frame.)*

## 6. Bring the decoder up listen-only first

Build and exercise the decoder against a virtual bus before you touch the real
vehicle, and treat any frame drought as missing data rather than a frozen reading.
In the MX-5 stack a small simulator opens a Linux `vcan0` and replays a full driving
cycle at ~50 Hz; the data layer picks a sim-vs-car decode map by channel so the
bench sim and live car never cross-decode, and a 1.5 s stale timeout blanks all
vehicle fields to "NO DATA" after silence (sim killed, ECU asleep, harness
unplugged). On the car the bus is engaged **listen-only** (HS-CAN 500k via OBD-II)
with a `LISTEN-ONLY` flag shown on the HUD; transmitting is a wholly separate path,
disarmed by default — no TX socket exists until you explicitly arm it behind a
dead-man hold-to-fire control. *(vcan0 replay + listen-only live bus verified.)*

## 7. Build an offline identifier so you can fingerprint before you transmit

Fingerprint an unknown bus before you ever transmit on it. A single read-only tool
(this repo's `canid.py`) can combine `cantools` decoding against a directory of
opendbc car DBCs, a small embedded J1939 common-PGN/SPN table and OBD-II mode-01 PID
table for heavy-vehicle/diagnostic frames, and a heuristic per-byte classifier
(period, DLC, and const/counter/enum-flags/CRC-random/value per byte) for IDs that
aren't in any database. For example, the same engine decoded an OBD-II RPM response
(`7E8 04410C1AF8` → 1726 rpm) and a J1939 EEC1 frame (`0CF00400` → 1024 rpm), and
characterised an unknown robot capture purely from byte statistics (correct 20/50 ms
periods). Keeping identification read-only and separate from any injector means you
understand the bus first and inject second. *(On-hardware verified.)*

---

*Status & honesty: decoded signal meanings are best-effort; anything safety-relevant
here was confirmed against the physical hardware before being relied on. Where a
result is still pending an on-car capture (the MX-5 roof command frames), it's marked
as such rather than presented as done.*
