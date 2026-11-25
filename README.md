
# ratgdo for ESPHome

[ESPHome](https://esphome.io/) component for [ratgdo](https://ratcloud.llc) — a WiFi control board for garage door openers that works over your local network. Compatible with most residential Chamberlain and LiftMaster openers, with dry contact support for other brands.

Purchase boards at [ratcloud.llc](https://ratcloud.llc).

## Web Installer

Flash the ESPHome based firmware using the [Web Installer](https://ratgdo.github.io/esphome-ratgdo/).

## First use after adding to Home Assistant

The ESPHome firmware will allow you to open the door to any position after calibration. To calibrate the door, open and close it once without stopping.

<img width="560" alt="position_demo" src="https://github.com/RATGDO/esphome-ratgdo/assets/663432/22a9873e-67bb-4b2f-bb32-70047cfe666d">

## ESPHome configs

| Board | Chip | Protocol | Config |
|-------|------|----------|--------|
| v2.0 | ESP8266 D1 Mini | Security+ 2.0 | [v2board_esp8266_d1_mini.yaml](static/v2board_esp8266_d1_mini.yaml) |
| v2.0 | ESP8266 D1 Mini lite | Security+ 2.0 | [v2board_esp8266_d1_mini_lite.yaml](static/v2board_esp8266_d1_mini_lite.yaml) |
| v2.0 | ESP32 D1 Mini | Security+ 2.0 | [v2board_esp32_d1_mini.yaml](static/v2board_esp32_d1_mini.yaml) |
| v2.0 | ESP32 Lolin S2 Mini | Security+ 2.0 | [v2board_esp32_lolin_s2_mini.yaml](static/v2board_esp32_lolin_s2_mini.yaml) |
| v2.5 | ESP8266 D1 Mini | Security+ 2.0 | [v25board_esp8266_d1_mini.yaml](static/v25board_esp8266_d1_mini.yaml) |
| v2.5 | ESP8266 D1 Mini | Security+ 1.0 | [v25board_esp8266_d1_mini_secplusv1.yaml](static/v25board_esp8266_d1_mini_secplusv1.yaml) |
| v2.5 | ESP8266 D1 Mini lite | Security+ 2.0 | [v25board_esp8266_d1_mini_lite.yaml](static/v25board_esp8266_d1_mini_lite.yaml) |
| v2.5 | ESP8266 D1 Mini lite | Security+ 1.0 | [v25board_esp8266_d1_mini_lite_secplusv1.yaml](static/v25board_esp8266_d1_mini_lite_secplusv1.yaml) |
| v2.5 | ESP32 D1 Mini | Security+ 2.0 | [v25board_esp32_d1_mini.yaml](static/v25board_esp32_d1_mini.yaml) |
| v2.5 | ESP32 D1 Mini | Security+ 1.0 | [v25board_esp32_d1_mini_secplusv1.yaml](static/v25board_esp32_d1_mini_secplusv1.yaml) |
| v2.5i/2.52i/2.53i | ESP32 | Security+ 2.0 | [v25iboard.yaml](static/v25iboard.yaml) |
| v2.5i/2.52i/2.53i | ESP32 | Security+ 1.0 | [v25iboard_secplusv1.yaml](static/v25iboard_secplusv1.yaml) |
| v2.5i/2.52i/2.53i | ESP32 | Dry Contact | [v25iboard_drycontact.yaml](static/v25iboard_drycontact.yaml) |
| v3.2 | ESP32 | Security+ 2.0 | [v32board.yaml](static/v32board.yaml) |
| v3.2 | ESP32 | Security+ 1.0 | [v32board_secplusv1.yaml](static/v32board_secplusv1.yaml) |
| v3.2 | ESP32 | Dry Contact | [v32board_drycontact.yaml](static/v32board_drycontact.yaml) |
| v3.2 Disco | ESP32 | Security+ 2.0 | [v32disco.yaml](static/v32disco.yaml) |
| v3.2 Disco | ESP32 | Security+ 1.0 | [v32disco_secplusv1.yaml](static/v32disco_secplusv1.yaml) |
| v3.2 Disco | ESP32 | Dry Contact | [v32disco_drycontact.yaml](static/v32disco_drycontact.yaml) |

![Home Assistant Screen Shot](static/hass.png)

## ESP32 Framework

Most ESP32 boards use the **ESP-IDF** framework. The project originally depended on Arduino, but PR [#577](https://github.com/ratgdo/esphome-ratgdo/pull/577) replaced the SoftwareSerial dependency with hardware UART and RMT peripherals, eliminating the need for Arduino on ESP32. Removing the Arduino layer saves ~1.5KB RAM and ~44KB flash since Arduino is built as an IDF component on top of ESP-IDF, and the smaller firmware means faster OTA updates.

The **v3.2 Disco** board uses the Arduino framework because its VL53L4CX distance sensor library and Wire I2C library require it.

ESP8266 boards continue to use the Arduino framework as ESPHome requires it on that platform.

## Troubleshooting

### Security+ 1.0 query read back as a command

On Security+ 1.0, ratgdo checks that each status query it sends reads back from
the bus unchanged. A query that reads back as a one-byte wall-panel command is
logged as a warning and counted by the `Door Query Read Back As Command` sensor
(`type: door_tx_echo_errors`). The opener received the same byte, so if it read
back as `0x30` the door may have moved. Such a byte is never reported as a
wall-panel button press. To limit how often this can happen, a settled door is
queried every 5 minutes.

### Security+ 1.0 checked query sender (ESP8266, opt-in)

A status query (`0x38`) differs from a door press (`0x30`) in one bit, and the
bus is wired-AND, so a short low pulse on that bit turns a query into a press.
With `-DRATGDO_SECPLUS1_CHECKED_QUERY_TX=1`, ratgdo sends queries bit by bit
with interrupts off and reads the bus back while it releases each data bit. If
the bus is low when it should be high, or an edge goes out late, it releases
the line and sends nothing more. Every panel command has bits 6 and 7 clear,
and those are the last bits a query drives, so what is left on the bus is not
a command. Such a query is counted by the `Door Queries Aborted` sensor
(`type: door_tx_query_aborts`) and sent again once.

```yaml
esphome:
  platformio_options:
    build_flags:
      - -DRATGDO_SECPLUS1_CHECKED_QUERY_TX=1
      - -DRATGDO_SECPLUS1_IDLE_PROBE_MS=10000
```

`RATGDO_SECPLUS1_IDLE_PROBE_MS` sets how often a settled door is queried
(default 300000). The sender assumes the bus reads idle within 250 µs of being
released; diagnostics builds log the measured time as `rise=`.

`-DRATGDO_SECPLUS1_ABORT_TEST=1` is for a supervised hardware test only. The
Query status button then sends 30 deliberately abandoned queries, to confirm
that the opener ignores them. Do not leave it flashed.

### Security+ 1.0 packet diagnostics

Additional packet, state-confirmation, transmit-queue, and timing logs are disabled
by default. To include them in a troubleshooting build, merge this into the device
YAML, preserving any existing build flags:

```yaml
esphome:
  platformio_options:
    build_flags:
      - -DRATGDO_SECPLUS1_DIAGNOSTICS=1

logger:
  level: DEBUG
```

Rebuild and flash to change this option. Set the flag to `0` or omit it to compile
out the added tracing and its argument evaluation. Ordinary logs and command
diagnostic counters remain available. Startup configuration reports whether packet
diagnostics are enabled. Runtime log-level changes cannot enable compiled-out tracing.

Extra logging can affect this timing-sensitive protocol. Capture logs externally
when investigating a failure; enabling the flag does not archive them. Commands
from a partially stopped door do not arm the recovery watch, so a missed command
from that state can leave all command counters unchanged.

Some Security+ 1.0 openers stop answering status queries for a few seconds at a
time and ignore a button press sent then. Before pressing for a stationary door,
ratgdo waits until two consecutive status queries are answered, for up to 10
seconds. A delayed press logs `Door press held …`; reaching the limit logs
`pressing anyway`.

Security+ 1.0 partial positions are travel-time estimates. Only opener endpoint
reports publish 0% or 100%; other states remain within 1–99%. Partial moves require
a valid starting estimate and calibrated travel durations.

### False obstruction events on ESP32 v2.5 boards

Some v2.5 boards using ESP32 D1 Mini controllers may report spurious obstruction events. This is caused by the obstruction sensor "asleep" detection logic being inverted on these boards.

To fix this, add the following to your ESPHome config:

```yaml
ratgdo:
  id: ratgdov25
  obst_sleep_low: true
```

The `id` must match the `id` of the existing `ratgdo` component in your config (e.g. `ratgdov25` for v2.5 boards). This setting tells the obstruction sensor to treat LOW as the "asleep" state instead of the ESP32 default of HIGH.
