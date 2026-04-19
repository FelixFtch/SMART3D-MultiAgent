# DDSM115 / RS485 for Smart3D

Last updated: 2026-04-18

This report is the DDSM115 / RS485 motor-control handoff for the canonical Smart3D tree under [connection-guideline/smart3d](/Users/felixfang/Documents/Playground/connection-guideline/smart3d) and the deploy tree under [SMART3D-MultiAgent/deploy](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy).

The goal here is not to invent a new motor stack. It is to document what the hardware officially expects, what the Pi baseline is actually doing, how Jetson should replicate it, and what is most likely wrong with the current "port opens but no feedback" situation on `/dev/ttyUSB0`.

## 1. Hardware Identity

- Motor family: `DDSM115` direct-drive servo hub motor from Waveshare.
- Bus: `RS485`, shared multi-drop bus, one host serial adapter, multiple wheel IDs.
- Smart3D assumption today:
  - left front motor ID `1`
  - right front motor ID `2`
  - left rear motor ID `3`
  - right rear motor ID `4`
  - right side sign is inverted in software
- Smart3D adapter binding:
  - FTDI adapter serial `BG01LYTM`
  - bound to `/dev/ttyUSB0` by [mk_ddsm_dev.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/mk_ddsm_dev.sh)

Primary official identity sources:

- [Waveshare DDSM115 product page](https://www.waveshare.com/ddsm115.htm)
- [Waveshare DDSM115 wiki](https://www.waveshare.com/wiki/DDSM115)
- [Waveshare Chinese DDSM115 wiki](https://www.waveshare.net/wiki/DDSM115)

Key official specs I verified from the Waveshare docs:

- nominal voltage `18V DC`, allowed range `12V-24V`
- rated speed `115rpm`
- no-load speed about `200rpm`
- encoder resolution `4096`
- RS485 signal pins:
  - `GND`
  - `A = DATA+`
  - `B = DATA-`

## 2. Official Docs / Packages

### Official docs

High confidence:

- [Waveshare DDSM115 wiki](https://www.waveshare.com/wiki/DDSM115)
- [Waveshare Chinese DDSM115 wiki](https://www.waveshare.net/wiki/DDSM115)
- [Waveshare DDSM115 product page](https://www.waveshare.com/ddsm115.htm)

Also useful as a Waveshare-hosted English protocol snapshot:

- [Amazon-hosted DDSM115 PDF image export](https://m.media-amazon.com/images/I/913N5OyPPoL.pdf)

### Official packages / repos / tooling

I found one clearly official code repo:

- [waveshareteam/ddsm_example](https://github.com/waveshareteam/ddsm_example)

I cloned it only for inspection into:

- [tmp clone README](/tmp/waveshare_ddsm_example/README.md)
- [tmp clone Arduino driver header](/tmp/waveshare_ddsm_example/ddsm_ctrl/ddsm_ctrl.h)
- [tmp clone Arduino driver source](/tmp/waveshare_ddsm_example/ddsm_ctrl/ddsm_ctrl.cpp)
- [tmp clone Python serial demo](/tmp/waveshare_ddsm_example/ddsm_python/serial_simple_ctrl.py)

What it is:

- an official Arduino library for DDSM115 / DDSM210
- an official `DDSM Driver HAT (A)` JSON demo
- a tiny Python serial demo for talking to the `Driver HAT (A)` over JSON

What it is not:

- not a ROS package
- not a direct Linux raw-RS485 SDK for the exact Smart3D path
- not a drop-in replacement for our current `pyserial + raw 10-byte frame` approach

Related official board docs:

- [Waveshare DDSM Driver HAT (A) wiki](https://www.waveshare.com/wiki/DDSM_Driver_HAT_%28A%29)

Important conclusion:

- Waveshare's official software story is centered on either:
  1. direct raw DDSM115 RS485 bytes documented in the DDSM115 wiki, or
  2. JSON commands through the `DDSM Driver HAT (A)` bridge.
- Smart3D is using path `1`, not path `2`.
- Do not mix the HAT JSON path into the current Smart3D raw-RS485 stack unless you explicitly decide to redesign the stack.

## 3. Protocol / Usage Model

### 3.1 Official wiring model

From the DDSM115 official wiki:

- signal connector:
  - pin 1 `GND`
  - pin 2 `A = DATA+`
  - pin 3 `B = DATA-`
- power connector:
  - `VCC`
  - `GND`

Official recommendation:

- use a `USB to RS485` adapter for a USB host
- connect motor `DATA+ -> adapter A+`
- connect motor `DATA- -> adapter B-`
- connect signal `GND -> adapter GND` if available

That last part matters. Waveshare says GND is recommended even if some converters omit it.

### 3.2 Official serial settings

From the official protocol page:

- baud `115200`
- data bits `8`
- stop bits `1`
- parity `none`
- frame length `10 bytes`
- reply model: one request / one reply
- hardware theoretical rate up to `500Hz`
- official recommended working rate: closer to `100Hz-200Hz`

### 3.3 Official control modes

For DDSM115:

- `0x01` current loop
- `0x02` speed loop
- `0x03` position loop

Important official behavior:

- after power-up, default mode is speed loop
- switching to position mode should happen only when speed is very low or zero

### 3.4 Official raw frame model

#### Protocol 1: drive motor

Send:

- byte 0: `ID`
- byte 1: `0x64`
- byte 2-3: target value
- byte 4-5: `0`
- byte 6: acceleration time
- byte 7: brake flag
- byte 8: `0`
- byte 9: `CRC8/MAXIM` over bytes 0-8

Feedback:

- byte 0: `ID`
- byte 1: mode
- byte 2-3: torque/current
- byte 4-5: speed
- byte 6-7: position
- byte 8: fault code
- byte 9: CRC

#### Protocol 2: query other info

Send:

- `[ID, 0x74, 0, 0, 0, 0, 0, 0, 0, CRC]`

Feedback:

- byte 0: `ID`
- byte 1: mode
- byte 2-3: torque/current
- byte 4-5: speed
- byte 6: winding temperature
- byte 7: `U8` position
- byte 8: fault code
- byte 9: CRC

#### Mode switch frame

For DDSM115 the official frame is:

- `[ID, 0xA0, 0, 0, 0, 0, 0, 0, 0, MODE]`

This is slightly unusual: the last byte is the mode value, not a CRC.

#### ID set / query

Set ID:

- `[0xAA, 0x55, 0x53, ID, 0, 0, 0, 0, 0, 0]`
- must be sent `5` times
- only one motor should be on the bus
- only allowed once per power cycle

Query ID:

- `[0xC8, 0x64, 0, 0, 0, 0, 0, 0, 0, CRC]`
- again, only one motor should be on the bus

### 3.5 Official example commands

ID `1`, switch to speed loop:

```text
01 A0 00 00 00 00 00 00 00 02
```

ID `1`, query extra info:

```text
01 74 00 00 00 00 00 00 00 04
```

ID `1`, brake in speed loop:

```text
01 64 00 00 00 00 00 FF 00 D1
```

ID `1`, speed command `50 rpm`:

```text
01 64 00 32 00 00 00 00 00 D3
```

Broadcast-style ID query, single motor only:

```text
C8 64 00 00 00 00 00 00 00 DE
```

### 3.6 Official safety / protection points

From Waveshare docs:

- bus overcurrent protection threshold about `3A`
- phase current protection threshold about `4.6A`
- locked-rotor protection triggers after about `5s`
- overtemperature protection around `80C`

That means "wheel did not reply" and "wheel is alive but in protection" are not the same state.

## 4. Pi Baseline Files and Behavior

### 4.1 Files directly involved

Canonical local files:

- [mk_ddsm_dev.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/mk_ddsm_dev.sh)
- [smart3d_campus.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh)
- [robotcar_wasd_max.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/robotcar_wasd_max.sh)
- [ddsm_hard_stop.py](/Users/felixfang/Documents/Playground/ddsm_hard_stop.py)
- [ddsm_emergency_brake.py](/Users/felixfang/Documents/Playground/ddsm_emergency_brake.py)
- [robotcar_wasdqe_test.py](/Users/felixfang/Documents/Playground/robotics_helpers/robotcar_wasdqe_test.py)

Deploy-tree files:

- [deploy/cmd_vel_driver.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cmd_vel_driver.py)
- [deploy/pi5_reference/cmd_vel_driver.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_reference/cmd_vel_driver.py)
- [deploy/pi5_reference/ddsm115_odom.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_reference/ddsm115_odom.py)
- [deploy/pi5_client.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_client.py)
- [deploy/pi5_reference/start_full.sh](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_reference/start_full.sh)
- [deploy/cartographer_config/robotcar_2d_odom.lua](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/robotcar_2d_odom.lua)
- [deploy/cartographer_config/robotcar_2d.lua](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/robotcar_2d.lua)

Jetson-side replication files:

- [jetson/start_full_native.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/start_full_native.sh)
- [jetson/deploy_runtime_to_jetson.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/deploy_runtime_to_jetson.sh)

### 4.2 What Pi is actually doing

#### Device binding

[mk_ddsm_dev.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/pi/mk_ddsm_dev.sh) searches `/dev/ttyUSB*`, finds FTDI serial `BG01LYTM`, and forces that device to appear as `/dev/ttyUSB0`.

That is an important baseline assumption. Everything else hardcodes `/dev/ttyUSB0`.

#### `cmd_vel -> raw RS485`

[deploy/cmd_vel_driver.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cmd_vel_driver.py):

- opens `/dev/ttyUSB0` at `115200`
- subscribes to `/cmd_vel`
- converts differential drive linear/angular velocity to left/right RPM
- clamps to `MAX_RPM = 80`
- sends raw `0x64` frames
- writes:
  - left IDs `1`, `3` directly
  - right IDs `2`, `4` with negated sign
- uses a ROS-side deadman timeout of `0.5s`
- does not read serial feedback
- does not explicitly switch motor mode on startup

This matches the official wire protocol enough to be plausible. It is relying on:

- correct IDs already programmed
- motors already being in default speed loop
- no other process corrupting the shared serial conversation

#### `odom <- raw RS485`

[deploy/pi5_reference/ddsm115_odom.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_reference/ddsm115_odom.py):

- also opens `/dev/ttyUSB0` at `115200`
- sends `0x74` query frames to IDs `1`, `2`, `3`, `4`
- parses bytes `4-5` of the reply as signed RPM
- averages left and right side RPMs
- publishes `/odom`

This also matches the official protocol well enough.

#### Local emergency gating

[deploy/pi5_client.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_client.py):

- publishes `/cmd_vel`
- does local front-arc laser emergency stop before motion commands

This is not motor-protocol-specific, but it is part of the actual safety chain.

#### Startup pattern

[deploy/pi5_reference/start_full.sh](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_reference/start_full.sh) launches:

- `ddsm115_odom.py`
- Cartographer with [robotcar_2d_odom.lua](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/robotcar_2d_odom.lua)

[smart3d_campus.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/smart3d_campus.sh) later launches:

- `mk_ddsm_dev.sh`
- `cmd_vel_driver.py`

That means the current operational baseline can put both of these on the same RS485 bus:

- `ddsm115_odom.py`
- `cmd_vel_driver.py`

#### Direct manual tooling

[robotcar_wasdqe_test.py](/Users/felixfang/Documents/Playground/robotics_helpers/robotcar_wasdqe_test.py):

- can query IDs `1..4`
- can spin a single motor
- can drive left/right sides
- can send brake frames
- is the best direct raw serial debug tool already in-tree

[ddsm_hard_stop.py](/Users/felixfang/Documents/Playground/ddsm_hard_stop.py) and [ddsm_emergency_brake.py](/Users/felixfang/Documents/Playground/ddsm_emergency_brake.py):

- send repeated `0 rpm` and brake frames
- match the official `0x64` command layout

## 5. Compare Official Usage with Pi Baseline

### 5.1 What Pi does correctly

- uses `115200 8N1`
- uses `CRC-8/MAXIM`
- uses correct `0x64` motion command family
- uses correct `0x74` query family
- uses brake byte `0xFF` in the correct field
- assumes speed loop after power-up, which matches the official docs

### 5.2 What Pi is doing informally, not explicitly

- it never explicitly sends the speed-loop mode switch at startup
- it assumes the four wheels are already programmed to IDs `1,2,3,4`
- it assumes right-side inversion is correct for this chassis wiring
- it assumes the FTDI RS485 adapter direction handling is fine

### 5.3 Where Pi diverges from the official usage model

The biggest divergence is host-side ownership.

Official model:

- one host process sends one request and gets one reply on a half-duplex bus

Pi baseline in practice:

- `cmd_vel_driver.py` owns the write path
- `ddsm115_odom.py` owns a polling read/query path
- direct keyboard scripts also exist
- these can compete for the same `/dev/ttyUSB0`

That is the single biggest architectural risk in the current baseline.

### 5.4 Important note on Waveshare's official software path

Waveshare's current official software investment is mostly around the `DDSM Driver HAT (A)` JSON bridge, not around a raw-RS485 Linux SDK.

That matters because:

- the official HAT stack adds a higher-level controller
- the current Smart3D Pi stack bypasses that and talks raw bytes straight to the motors

So the right replication target for Jetson is the Pi behavior, not the HAT JSON flow.

## 6. Jetson Replication Checklist

The current Jetson deploy script already copies the important DDSM pieces:

- [jetson/deploy_runtime_to_jetson.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/deploy_runtime_to_jetson.sh)

It deploys:

- `cmd_vel_driver.py`
- `ddsm115_odom.py`
- `mk_ddsm_dev.sh`
- `ddsm_hard_stop.py`
- `ddsm_emergency_brake.py`
- `robotcar_wasdqe_test.py`

The current Jetson startup flow:

- [jetson/start_full_native.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/start_full_native.sh)

still preserves the Pi baseline behavior, including the warning that:

- `cmd_vel_driver.py` shares `/dev/ttyUSB0` with `ddsm115_odom.py`

For a clean Jetson replication, do this in order:

1. bind the correct FTDI adapter first
2. prove raw DDSM replies with only one host process
3. prove safe stop / brake
4. prove per-wheel manual motion at low RPM
5. prove `ddsm115_odom.py` alone
6. only then combine odom with the rest of the stack
7. only then start `cmd_vel_driver.py`
8. only after that reintroduce `pi5_client.py` and planner path following

Operational recommendation:

- until serial feedback is proven clean, Jetson should prefer the no-wheel-odom Cartographer config:
  - [robotcar_2d.lua](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/robotcar_2d.lua)
- after DDSM feedback is proven stable, move back to:
  - [robotcar_2d_odom.lua](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cartographer_config/robotcar_2d_odom.lua)

## 7. Exact Verification Commands

The safest commands are ordered from no-motion to motion.

### 7.1 Confirm the correct adapter

```bash
ls -l /dev/ttyUSB* /dev/serial/by-id 2>/dev/null
udevadm info -q property -n /dev/ttyUSB0 | rg 'ID_VENDOR|ID_MODEL|ID_SERIAL|DEVNAME'
```

Expected for this Smart3D setup:

- FTDI-based adapter
- serial `BG01LYTM`

### 7.2 Kill all competing RS485 owners first

```bash
pkill -f '[c]md_vel_driver.py|[d]dsm115_odom.py|[r]obotcar_wasdqe_test.py' || true
```

### 7.3 Safe brake before any experiment

Jetson:

```bash
python3 /home/jetson/SMART3D/jetson_runtime/ddsm_emergency_brake.py /dev/ttyUSB0
```

Pi:

```bash
python3 /root/ddsm_emergency_brake.py /dev/ttyUSB0
```

### 7.4 Multi-motor no-motion query test for IDs 1-4

This is the best first proof for the current four-wheel bus.

```bash
python3 - <<'PY'
import serial, time

def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc >> 1) ^ 0x8C) if (crc & 1) else (crc >> 1)
    return crc & 0xFF

def query(mid):
    body = bytes([mid, 0x74, 0, 0, 0, 0, 0, 0, 0])
    return body + bytes([crc8(body)])

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.05)
try:
    for mid in (1, 2, 3, 4):
        print(f'ID {mid}')
        for _ in range(5):
            ser.reset_input_buffer()
            ser.write(query(mid))
            resp = ser.read(10)
            print('  ', resp.hex() if resp else '<empty>')
            time.sleep(0.03)
finally:
    ser.close()
PY
```

Good result:

- at least some `10-byte` replies with correct CRC

Bad result:

- all empty
- random non-10-byte garbage
- CRC failures only

### 7.5 Single-motor-only ID query

Use this only when there is exactly one motor physically on the bus.

```bash
python3 - <<'PY'
import serial
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
try:
    ser.write(bytes.fromhex('c8 64 00 00 00 00 00 00 00 de'))
    print(ser.read(10).hex())
finally:
    ser.close()
PY
```

### 7.6 Single-motor-only explicit speed-loop command

Again: single isolated motor, mechanically secured, power-cut ready.

```bash
python3 - <<'PY'
import serial, time
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
try:
    ser.write(bytes.fromhex('01 a0 00 00 00 00 00 00 00 02'))
    time.sleep(0.05)
    ser.write(bytes.fromhex('01 64 00 14 00 00 00 00 00 8d'))  # 20 rpm
    time.sleep(0.3)
    ser.write(bytes.fromhex('01 64 00 00 00 00 00 ff 00 d1'))  # brake
finally:
    ser.close()
PY
```

### 7.7 Existing Smart3D manual tool

This is the best in-repo direct exerciser once the query test works:

```bash
python3 /home/jetson/SMART3D/jetson_runtime/robotcar_wasdqe_test.py \
  --port /dev/ttyUSB0 \
  --base-rpm 35 \
  --turn-rpm 28 \
  --arc-rpm 35 \
  --deadman-sec 0.35 \
  --max-rpm-limit 80
```

Useful keys:

- `r` query motor replies
- `1 2 3 4` single motor forward
- `! @ # $` single motor backward
- `x` stop
- `z` hard brake

### 7.8 Existing Pi wrapper

For the Pi baseline path:

```bash
PI_HOST='robotcar@<pi_ip>' \
PI_PASS='234123' \
bash /Users/felixfang/Documents/Playground/connection-guideline/smart3d/scripts/robotcar_wasd_max.sh
```

## 8. Likely Causes of the Current No-Feedback Issue on `/dev/ttyUSB0`

This section is ordered from most likely to less likely for the present Smart3D situation.

### 8.1 RS485 contention between host processes

Most likely software-side cause.

Why:

- [deploy/cmd_vel_driver.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/cmd_vel_driver.py) and [deploy/pi5_reference/ddsm115_odom.py](/Users/felixfang/Documents/private-repos/SMART3D-MultiAgent/deploy/pi5_reference/ddsm115_odom.py) both open the same half-duplex bus
- [jetson/start_full_native.sh](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/jetson/start_full_native.sh) explicitly preserves this pattern
- direct control tools can also join the fight

Symptom pattern:

- port opens fine
- writes succeed
- replies are empty or inconsistent

### 8.2 The adapter is present, but motor power is absent or unstable

Very likely hardware-side cause.

Opening `/dev/ttyUSB0` only proves:

- Linux sees the USB serial adapter

It does not prove:

- DDSM motors are powered
- the power ground is common enough for signaling
- protection state has cleared

### 8.3 Wrong or duplicated motor IDs

Very plausible.

Official constraints:

- default ID is `1`
- ID set should happen with only one motor on the bus

If all wheels are still ID `1`, then:

- querying ID `1` can cause multiple motors to answer simultaneously
- queries to IDs `2,3,4` will be empty
- the bus can look dead even though wheels are powered

### 8.4 A/B polarity or signal ground mismatch

Still plausible, especially after rewiring or moving between computers.

Official recommendation explicitly says:

- `DATA+ -> A+`
- `DATA- -> B-`
- GND recommended

Real-world problem:

- some RS485 adapters label A/B opposite ways
- some work without signal ground, some do not

### 8.5 Adapter direction / dongle behavior issue

Lower confidence, but possible.

The FTDI device proving up as a serial port does not by itself prove the RS485 transceiver path is healthy.

If this adapter is a marginal or odd USB-to-RS485 implementation, writes may appear successful while the bus never cleanly switches into receive.

### 8.6 Motor not in the expected mode

Possible, but not my first choice for the current symptom.

Why it is less likely:

- the official default after power-up is speed loop
- even if mode were wrong, `0x74` info queries should still usually reply if the bus is healthy

### 8.7 Protection state on one or more motors

Possible if the system recently stalled or overcurrented.

Less likely to explain a totally silent bus across all wheels, but worth checking once low-level comms are confirmed.

## 9. Concrete Next Experiments in Safest Order

### Step 1: make the chassis safe

- wheels off the ground, or drivetrain unloaded
- power switch reachable
- run brake script first

### Step 2: kill all RS485 owners

- no `cmd_vel_driver.py`
- no `ddsm115_odom.py`
- no manual controller still attached

### Step 3: verify adapter identity

- confirm FTDI serial `BG01LYTM`
- confirm the expected `/dev/ttyUSB0`

### Step 4: do the no-motion query test

- use the exact `0x74` multi-ID Python query above
- do not send motion yet

Interpretation:

- if one or more IDs reply, the bus is alive
- if none reply, stay below ROS and below motion until that is fixed

### Step 5: if zero replies, isolate one motor physically

This is the most important next experiment if the shared 4-motor bus stays silent.

With one motor only:

- run the single-motor ID query
- confirm the actual ID
- confirm the motor replies at all

This cleanly separates:

- bus wiring / power problems
- ID collision problems
- multi-drop contention problems

### Step 6: if isolated motor replies, program IDs deliberately

One wheel at a time:

- isolate wheel
- confirm current ID
- set desired ID
- power cycle
- re-check

Do not assume the chassis still has the historical `1,2,3,4` mapping unless you re-prove it.

### Step 7: reconnect four motors and query each ID again

No motion yet.

Only when the shared bus gives stable replies to `1,2,3,4` should you continue.

### Step 8: run `ddsm115_odom.py` alone

Expected good sign:

- query fail rate falls well below `100%`
- pose changes when a wheel is moved or gently driven

### Step 9: run manual low-RPM direct control

Use [robotcar_wasdqe_test.py](/Users/felixfang/Documents/Playground/robotics_helpers/robotcar_wasdqe_test.py) at very low RPM.

First do:

- single wheel forward / backward

Then:

- both left wheels
- both right wheels
- full chassis

### Step 10: only then restore ROS active driving

Reintroduce in this order:

1. `ddsm115_odom.py`
2. Cartographer
3. `cmd_vel_driver.py`
4. `pi5_client.py`
5. planner goal-test

If serial feedback regresses only after step 3, the main problem is host-side port ownership.

## 10. Bottom Line

The official DDSM115 protocol is clear, simple, and the current Smart3D raw frames mostly match it. The biggest weakness in the current Smart3D implementation is not the byte format; it is the fact that multiple host processes can compete for the same half-duplex RS485 bus.

The shortest safe path forward is:

1. treat `/dev/ttyUSB0` as single-owner during debugging
2. prove raw replies before any ROS integration
3. isolate one motor if the 4-wheel bus stays silent
4. re-prove IDs `1..4`
5. only then restore odom and active driving

## Brief Summary

What I found:

- official Waveshare docs exist and are good enough to verify the raw Smart3D frame format
- Smart3D's `cmd_vel_driver.py`, `ddsm115_odom.py`, and brake tools mostly match the official protocol
- Waveshare does have an official repo, but it is centered on Arduino and `DDSM Driver HAT (A)` JSON control, not ROS
- the most likely current failure mode is shared RS485 ownership, followed by motor power / ID / wiring issues

Files changed:

- [HARDWARE_DDSM115_RS485.md](/Users/felixfang/Documents/Playground/connection-guideline/smart3d/research/HARDWARE_DDSM115_RS485.md)
