# j4_display_left

Jukebox display board for the Johnny 4 robot controller -- the portrait display on the LEFT of the panel (renamed from j4_display when [j4_display_right](https://github.com/kevinkevinlangelange/j4_display_right) joined the controller). Receives live control data from the main TTGO T-Display controller board over a wired UART link and renders the jukebox UI on a 3.5" colour TFT.

## What this board does

- Receives 25 fps binary packets from the TTGO controller over UART (wired, 115200 baud)
- Displays all 7 potentiometer values (volume, eyes, spotlight, arms, neck, jaw)
- Shows the currently selected audio phrase
- Shows all three battery voltages
- Shows ESP-NOW link health (LINKED / NO LINK) between the controller and robot receiver
- Clears the now-playing highlight when a track stops or finishes (acts on empty-phrase packets too)
- Requests the file list with `LIST?` every 2 seconds until it arrives, so a display reboot recovers the list from the controller's cache
- Displays a "Waiting for controller..." message on startup until the first packet arrives
- Sends a `PING` heartbeat once per second so the controller's connection-status screen can show j4_display as connected

## Hardware

| Component | Details |
|-----------|---------|
| Microcontroller | SeeedStudio XIAO ESP32S3 |
| Display | 3.5" TFT LCD, 480x320, SPI, ILI9488 driver |

## Display power

Specs from the [official product page](https://www.lcdwiki.com/3.5inch_SPI_Module_ILI9488_SKU:MSP3520) (Hosyond MSP3520):

| Parameter | Value |
|-----------|-------|
| VCC supply range | 3.3V to 5V (onboard regulator on the module PCB) |
| Logic IO voltage | 3.3V TTL |
| Power consumption | 0.62W |
| Current at 3.3V | ~188 mA |
| Current at 5V | ~124 mA |

VCC connects to the XIAO's 3V3 pin. The 3.3V regulator handles the display (~188 mA) plus the ESP32S3 fine -- it's rated ~700 mA so there's plenty of headroom. The 5V pin works too and takes some load off the regulator, but it's only live when USB is connected.

The backlight (D0/GPIO1) supports PWM dimming if you need to cut power draw.

## Pin assignments

| XIAO Pin | GPIO | Function |
|----------|------|----------|
| 3V3 | n/a | Display VCC (3.3V power) |
| GND | n/a | Display GND, shared with TTGO GND |
| D0 | GPIO1 | TFT backlight (HIGH = on) |
| D1 | GPIO2 | TFT RST |
| D2 | GPIO3 | TFT DC |
| D3 | GPIO4 | TFT CS |
| D4 | GPIO5 | Free (I2C SDA if needed later) |
| D5 | GPIO6 | Free (I2C SCL if needed later) |
| D6 | GPIO43 | UART TX to TTGO GPIO27 |
| D7 | GPIO44 | UART RX from TTGO GPIO17 |
| D8 | GPIO7 | TFT SCLK |
| D9 | GPIO8 | TFT MISO |
| D10 | GPIO9 | TFT MOSI |

## Pin diagram

Physical layout, component side up, USB-C at the TOP. The two header rails
read top-to-bottom as shown.

```
            LEFT rail                              RIGHT rail
                            +--[ USB-C ]--+
     TFT BL  -- D0 (GP1) |  |             |  | 5V           (unused)
     TFT RST -- D1 (GP2) |  |    XIAO     |  | GND          ground
     TFT DC  -- D2 (GP3) |  |   ESP32-S3  |  | 3V3          TFT VCC
     TFT CS  -- D3 (GP4) |  |             |  | D10 (GP9)    TFT MOSI
      (free) -- D4 (GP5) |  |             |  | D9  (GP8)    TFT MISO
      (free) -- D5 (GP6) |  |             |  | D8  (GP7)    TFT SCLK
LINK TX -> TTGO 27 -- D6 |  |             |  | D7  (GP44)   LINK RX <- TTGO 17
              (GP43)        +-------------+
```

The ILI9488 TFT uses BL/RST/DC/CS (D0-D3) plus SCK/MISO/MOSI (D8/D9/D10);
D6/D7 are the UART link to the TTGO; D4/D5 are free.

## Wiring to TTGO controller

3.3V logic on both sides -- no level shifter needed.

| XIAO | TTGO |
|------|------|
| D7 (GPIO44) RX | GPIO17 TX |
| D6 (GPIO43) TX | GPIO27 RX |
| GND | GND |

## Wiring to ILI9488 display module

| XIAO | Display pin |
|------|-------------|
| 3V3 | VCC |
| GND | GND |
| D0 (GPIO1) | LED / BL (backlight) |
| D1 (GPIO2) | RST |
| D2 (GPIO3) | DC |
| D3 (GPIO4) | CS |
| D8 (GPIO7) | SCK |
| D9 (GPIO8) | MISO (SDO) |
| D10 (GPIO9) | MOSI (SDI) |

## UART packet protocol

The TTGO sends a fixed 49-byte packed binary struct (`disp_pkt_t`) at 25 fps:

| Bytes | Field | Description |
|-------|-------|-------------|
| 2 | `magic` | Frame sync header: 0xAB, 0xCD |
| 1 | `volume` | Volume knob (0-100) |
| 1 | `eyes` | Eye position (0-255) |
| 1 | `spot` | Spotlight intensity (0-255) |
| 1 | `left_arm` | Left arm position (0-255) |
| 1 | `right_arm` | Right arm position (0-255) |
| 1 | `neck` | Neck position (0-255) |
| 1 | `jaw` | Jaw position (0-255) |
| 2 | `bat1_mv` | Controller battery (millivolts, uint16) |
| 2 | `bat2_raw` | Robot battery 2 raw value from receiver |
| 2 | `bat3_raw` | Robot battery 3 raw value from receiver |
| 1 | `connect_ok` | ESP-NOW link health (1 = ok, 0 = error) |
| 32 | `phrase` | Selected audio phrase, null-terminated |
| 1 | `checksum` | XOR of all preceding bytes |

The receiver re-syncs on the 0xAB 0xCD magic header after any glitch. Packets that fail the checksum are silently dropped.

## Dependencies

Managed via PlatformIO (`platformio.ini`):

| Library | Version |
|---------|---------|
| Bodmer/TFT_eSPI | latest from GitHub |

TFT_eSPI is configured entirely through `build_flags` in `platformio.ini` -- no `User_Setup.h` file needed.

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

- **[j4_controller](https://github.com/kevinkevinlangelange/j4_controller)** -- the TTGO T-Display transmitter that sends data to this board
- **[j4_display_right](https://github.com/kevinkevinlangelange/j4_display_right)** -- the landscape display + pot-label board on the right of the panel
