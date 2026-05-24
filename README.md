# j4_display

Secondary display board for the Johnny 4 robot controller. Receives live control data from the main TTGO T-Display controller board over a wired UART link and renders a status dashboard on a 3.5" colour TFT.

## What this board does

- Receives 25 fps binary packets from the TTGO controller over UART (wired, 115200 baud)
- Displays all 7 potentiometer values (volume, eyes, spotlight, arms, neck, jaw)
- Shows the currently selected audio phrase
- Shows all three battery voltages
- Shows ESP-NOW link health (LINKED / NO LINK) between the controller and robot receiver
- Displays a "Waiting for controller…" message on startup until the first packet arrives

## Hardware

| Component | Details |
|-----------|---------|
| Microcontroller | SeeedStudio XIAO ESP32S3 |
| Display | 3.5" TFT LCD, 480×320, SPI, ILI9488 driver |

## Pin assignments

| XIAO Pin | GPIO | Function |
|----------|------|----------|
| D0 | GPIO1 | TFT backlight (HIGH = on) |
| D1 | GPIO2 | TFT RST |
| D2 | GPIO3 | TFT DC |
| D3 | GPIO4 | TFT CS |
| D4 | GPIO5 | Free (I2C SDA if needed later) |
| D5 | GPIO6 | Free (I2C SCL if needed later) |
| D6 | GPIO43 | UART TX → TTGO GPIO27 |
| D7 | GPIO44 | UART RX ← TTGO GPIO17 |
| D8 | GPIO7 | TFT SCLK |
| D9 | GPIO8 | TFT MISO |
| D10 | GPIO9 | TFT MOSI |
| GND | — | Shared ground with TTGO |

## Wiring to TTGO controller

3.3V logic on both sides — no level shifter needed.

| XIAO | | TTGO |
|------|-|------|
| D7 (GPIO44) RX | ← | GPIO17 TX |
| D6 (GPIO43) TX | → | GPIO27 RX |
| GND | — | GND |

## UART packet protocol

The TTGO sends a fixed 49-byte packed binary struct (`disp_pkt_t`) at 25 fps:

| Bytes | Field | Description |
|-------|-------|-------------|
| 2 | `magic` | Frame sync header: `0xAB`, `0xCD` |
| 1 | `volume` | Volume knob (0–100) |
| 1 | `eyes` | Eye position (0–255) |
| 1 | `spot` | Spotlight intensity (0–255) |
| 1 | `left_arm` | Left arm position (0–255) |
| 1 | `right_arm` | Right arm position (0–255) |
| 1 | `neck` | Neck position (0–255) |
| 1 | `jaw` | Jaw position (0–255) |
| 2 | `bat1_mv` | Controller battery (millivolts, uint16) |
| 2 | `bat2_raw` | Robot battery 2 raw value from receiver |
| 2 | `bat3_raw` | Robot battery 3 raw value from receiver |
| 1 | `connect_ok` | ESP-NOW link health (1 = ok, 0 = error) |
| 32 | `phrase` | Selected audio phrase, null-terminated |
| 1 | `checksum` | XOR of all preceding bytes |

The receiver re-syncs on the `0xAB 0xCD` magic header after any glitch. Packets that fail the checksum are silently dropped.

## Dependencies

Managed via PlatformIO (`platformio.ini`):

| Library | Version |
|---------|---------|
| Bodmer/TFT_eSPI | ^2.5.43 |

TFT_eSPI is configured entirely through `build_flags` in `platformio.ini` — no `User_Setup.h` file needed.

## Building and uploading

```bash
# Build
pio run

# Upload
pio run --target upload

# Monitor serial output (USB CDC debug)
pio device monitor
```

If the upload fails, hold the **BOOT** button on the XIAO, tap **RESET**, then release BOOT to force download mode.

## Related projects

- **[j4_controller](https://github.com/kevinkevinlangelange/j4_controller)** — the TTGO T-Display transmitter that sends data to this board
