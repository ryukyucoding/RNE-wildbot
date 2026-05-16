# wildbot_wheelctrl

ESP32 firmware for controlling DDSM115 hub motors over RS-485 via serial JSON commands.

## Hardware

- **MCU**: ESP32
- **Motor**: DDSM115 (RS-485, CRC-8/MAXIM)
- **Serial**: 115200 baud, 8N1
- **Pins**: WHEEL_RX = GPIO 18, WHEEL_TX = GPIO 19

## Communication

Send newline-terminated JSON strings over the ESP32 USB serial (115200 baud). Each command is identified by the `"T"` field.

### Wheel Commands

| Command | T | Example | Description |
|---|---|---|---|
| Stop | 10000 | `{"T":10000,"id":1}` | Stop a single wheel |
| Control | 10010 | `{"T":10010,"id":1,"cmd":50,"act":3}` | Control wheel speed/position/current |
| Change Mode | 10012 | `{"T":10012,"id":1,"mode":2}` | Change control mode |
| Get Info | 10032 | `{"T":10032,"id":1}` | Get wheel info (temp, voltage, error) |
| Heartbeat | 11001 | `{"T":11001,"time":2000}` | Set heartbeat timeout in ms (-1 to disable) |

### Control Modes (`"mode"`)

| Mode | Description |
|---|---|
| 1 | Current loop (cmd: -32767 ~ 32767 -> -8 ~ 8 A) |
| 2 | Speed loop (cmd: -200 ~ 200 rpm) |
| 3 | Position loop (cmd: 0 ~ 32767 -> 0 ~ 360 deg) |

### System Commands

| Command | T | Example | Description |
|---|---|---|---|
| Reboot | 600 | `{"T":600}` | Reboot ESP32 |

## Feedback

The ESP32 sends JSON responses over serial when wheel data is received.

### Motor Feedback (T: 20010)

Normal response:

```json
{"T":20010,"id":1,"mode":2,"tor":0,"spd":50,"pos":1234,"err":0}
```

Info response (after `CMD_WHEEL_INFO`):

```json
{"T":20010,"id":1,"mode":2,"tor":0,"spd":50,"temp":35,"u8":0,"err":0}
```

CRC error response:

```json
{"T":20010,"crc":0}
```

## Heartbeat

When enabled, if no command is received within the timeout period, all wheels (ID 1-4) are automatically stopped. Any new command resets the timer.

- Enable: `{"T":11001,"time":2000}` (2 second timeout)
- Disable: `{"T":11001,"time":-1}`
