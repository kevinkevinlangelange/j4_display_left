//******************************************************************************
//       file name:  j4_display
//     v0_1 created:  2026-05-22
//     last updated:  2026-05-23
//           author:  Kevin Lange
//      description:  Secondary display board for the Johnny 4 controller.
//                    Receives control data from the TTGO T-Display v1.1 over
//                    a wired UART link and renders a live status dashboard on
//                    a 3.5" ILI9488 480x320 SPI TFT display.
//
//
//      J4_DISPLAY — SeeedStudio XIAO ESP32S3 Pin Connections
//      ------------------------------------------------------------------
//      D0 / GPIO1   TFT backlight (HIGH = on)
//      D1 / GPIO2   TFT RST
//      D2 / GPIO3   TFT DC
//      D3 / GPIO4   TFT CS
//      D4 / GPIO5   (free — I2C SDA if needed later)
//      D5 / GPIO6   (free — I2C SCL if needed later)
//      D6 / GPIO43  LINK TX  →  TTGO GPIO27  (TTGO RX)
//      D7 / GPIO44  LINK RX  ←  TTGO GPIO17  (TTGO TX)
//      D8 / GPIO7   TFT SCLK
//      D9 / GPIO8   TFT MISO
//      D10 / GPIO9  TFT MOSI
//      GND          GND (shared with TTGO GND)
//      ------------------------------------------------------------------
//
//
//      UART LINK PROTOCOL
//      ------------------------------------------------------------------
//      The TTGO sends a 49-byte binary packet (disp_pkt_t) at 25 fps.
//      Each packet begins with a 2-byte magic header (0xAB, 0xCD) used
//      to re-sync if bytes are lost. The final byte is an XOR checksum
//      over all preceding bytes. Corrupted packets are silently dropped.
//      ------------------------------------------------------------------
//
//
//      DISPLAY LAYOUT  (480 × 320 landscape)
//      ------------------------------------------------------------------
//      ┌──────────────────────────────────────────────────────────────┐
//      │  J4 CONTROLLER                              LINKED / NO LINK  │  ← header bar (dark blue bg)
//      ├──────────────────────────────────────────────────────────────┤  ← green divider line
//      │  VOL:   ###  │                                               │
//      │  EYES:  ###  │  PLAYING:                                     │
//      │  SPOT:  ###  │  <phrase name>                                │
//      │  L-ARM: ###  │                                               │
//      │  R-ARM: ###  │                                               │
//      │  NECK:  ###  │                                               │
//      │  JAW:   ###  │                                               │
//      ├──────────────────────────────────────────────────────────────┤  ← green divider line
//      │  BAT1: #.##V         BAT2: ###         BAT3: ###            │  ← battery row
//      └──────────────────────────────────────────────────────────────┘
//      ------------------------------------------------------------------
//******************************************************************************


#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>


// -----------------------------------------------------------------------------
//  UART LINK  —  Serial1 on GPIO43 (TX) / GPIO44 (RX)
//  Serial (USB CDC) is kept free for debug output via the USB port.
// -----------------------------------------------------------------------------
#define LINK_RX    44      // D7 ← receives data from TTGO GPIO17
#define LINK_TX    43      // D6 → sends data to   TTGO GPIO27 (reserved for future use)
#define LINK_BAUD  115200


// -----------------------------------------------------------------------------
//  PACKET STRUCTURE
//  Must stay byte-for-byte identical to disp_pkt_t in j4_controller/src/main.cpp.
//  __attribute__((packed)) prevents the compiler from adding padding bytes.
// -----------------------------------------------------------------------------
typedef struct __attribute__((packed)) {
  uint8_t  magic[2];       // frame sync marker: always 0xAB, 0xCD
  uint8_t  volume;         // master volume knob         (0–100)
  uint8_t  eyes;           // eye position/brightness    (0–255)
  uint8_t  spot;           // spotlight intensity        (0–255)
  uint8_t  left_arm;       // left arm servo position    (0–255)
  uint8_t  right_arm;      // right arm servo position   (0–255)
  uint8_t  neck;           // neck servo position        (0–255)
  uint8_t  jaw;            // jaw servo position         (0–255)
  uint16_t bat1_mv;        // controller battery voltage (millivolts)
  int16_t  bat2_raw;       // robot battery 2 — raw value forwarded from receiver
  int16_t  bat3_raw;       // robot battery 3 — raw value forwarded from receiver
  uint8_t  connect_ok;     // ESP-NOW link health: 1 = linked, 0 = error
  char     phrase[32];     // currently selected audio phrase, null-terminated (e.g. "A12")
  uint8_t  checksum;       // XOR of every byte above — used to detect corrupt packets
} disp_pkt_t;


// -----------------------------------------------------------------------------
//  COLOR PALETTE  (RGB565)
// -----------------------------------------------------------------------------
#define C_BG       TFT_BLACK    // background
#define C_LABEL    0x07FF       // cyan  — pot/battery label text
#define C_VALUE    TFT_WHITE    // white — live numeric values
#define C_OK       TFT_GREEN    // green — "LINKED" status, divider lines
#define C_ERR      TFT_RED      // red   — "NO LINK" status
#define C_ACCENT   TFT_YELLOW   // yellow — "PLAYING:" label
#define C_DIM      0x4208       // dark grey — waiting message, divider lines
#define C_HEADER   0x1082       // very dark blue — header bar background


// -----------------------------------------------------------------------------
//  LAYOUT CONSTANTS  (all pixel positions for 480 × 320 landscape)
//
//  Columns:
//    COL_LABEL_1 / COL_VALUE_1  — left pot-value column
//    COL_LABEL_2 / COL_VALUE_2  — right "PLAYING" / BAT2 column
//    COL_LABEL_3 / COL_VALUE_3  — BAT3 column (battery row only)
//
//  Rows:
//    ROW_HEADER   — vertical centre of header text
//    ROW_DIVIDER  — top green line separating header from data area
//    ROW_DATA_1   — first pot-value row (VOL)
//    ROW_STEP     — pixel height of each pot-value row
//    ROW_DIVIDER2 — bottom green line separating data area from battery row
//    ROW_BAT      — battery label/value row
// -----------------------------------------------------------------------------
#define COL_LABEL_1   10
#define COL_VALUE_1   90
#define COL_LABEL_2  200
#define COL_VALUE_2  280
#define COL_LABEL_3  370
#define COL_VALUE_3  420
#define ROW_HEADER    8
#define ROW_DIVIDER  40
#define ROW_DATA_1   54
#define ROW_STEP     22
#define ROW_DIVIDER2 210
#define ROW_BAT      222


// -----------------------------------------------------------------------------
//  GLOBALS
// -----------------------------------------------------------------------------
TFT_eSPI tft = TFT_eSPI();

disp_pkt_t rxPkt;                  // most recently received and verified packet
bool firstPacketReceived = false;  // used to redraw labels cleanly on first data


// -----------------------------------------------------------------------------
//  FUNCTION PROTOTYPES
// -----------------------------------------------------------------------------
void drawStaticLayout();
bool receivePkt();
void drawValues(const disp_pkt_t &pkt);
void drawWaiting();


// -----------------------------------------------------------------------------
//  SETUP
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);   // USB CDC — use for debug prints; does not affect LINK_RX/TX

  // UART link to the TTGO controller board
  Serial1.begin(LINK_BAUD, SERIAL_8N1, LINK_RX, LINK_TX);

  // TFT backlight — TFT_eSPI also configures this in tft.init(), but being
  // explicit here makes the intent clear if the pin definition ever changes
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  tft.init();
  tft.setRotation(3);     // rotation 3 = landscape, 480 px wide × 320 px tall
  tft.fillScreen(C_BG);
  tft.setSwapBytes(true); // required for correct RGB byte order on ESP32 + TFT_eSPI

  drawStaticLayout();
  drawWaiting();
}


// -----------------------------------------------------------------------------
//  MAIN LOOP
//  Checks for incoming UART packets every iteration. When a valid packet
//  arrives, update only the value regions of the screen (labels never change).
// -----------------------------------------------------------------------------
void loop() {
  if (receivePkt()) {
    if (!firstPacketReceived) {
      firstPacketReceived = true;
      drawStaticLayout();   // wipe the "Waiting..." message before first draw
    }
    drawValues(rxPkt);
  }
}


// -----------------------------------------------------------------------------
//  receivePkt()
//  Non-blocking packet parser. Reads whatever bytes Serial1 has available,
//  accumulates them into a static buffer, and returns true only when a full
//  49-byte packet has been received AND its checksum matches.
//
//  Re-sync strategy: if byte 0 is not 0xAB or byte 1 is not 0xCD, the buffer
//  position resets and the parser hunts for the next magic header. This keeps
//  the display running cleanly if the UART glitches or the TTGO restarts.
// -----------------------------------------------------------------------------
bool receivePkt() {
  static uint8_t buf[sizeof(disp_pkt_t)];
  static size_t  pos = 0;

  while (Serial1.available()) {
    uint8_t b = (uint8_t)Serial1.read();

    // Enforce magic header to stay in sync with the transmitter
    if (pos == 0 && b != 0xAB) continue;
    if (pos == 1 && b != 0xCD) { pos = 0; continue; }

    buf[pos++] = b;

    if (pos == sizeof(disp_pkt_t)) {
      pos = 0;

      // XOR all bytes except the last one, compare against stored checksum
      uint8_t cs = 0;
      for (size_t i = 0; i < sizeof(disp_pkt_t) - 1; i++) cs ^= buf[i];
      if (cs != buf[sizeof(disp_pkt_t) - 1]) {
        Serial.println("XIAO: checksum error — packet dropped");
        return false;
      }

      memcpy(&rxPkt, buf, sizeof(disp_pkt_t));
      return true;
    }
  }
  return false;
}


// -----------------------------------------------------------------------------
//  drawStaticLayout()
//  Draws everything that never changes: header bar, divider lines, and all
//  text labels. Called once at startup and once again when the first real
//  packet arrives (to erase the "Waiting..." message cleanly).
// -----------------------------------------------------------------------------
void drawStaticLayout() {
  tft.fillScreen(C_BG);

  // Header bar — dark blue background with title text
  tft.fillRect(0, 0, 480, ROW_DIVIDER - 2, C_HEADER);
  tft.setTextColor(C_OK, C_HEADER);
  tft.drawString("J4 CONTROLLER", COL_LABEL_1, ROW_HEADER, 4);

  // Green divider line below header
  tft.drawFastHLine(0, ROW_DIVIDER, 480, C_OK);

  // Left column — potentiometer labels (cyan, font 2 ≈ 14px tall)
  tft.setTextColor(C_LABEL, C_BG);
  tft.drawString("VOL:",    COL_LABEL_1, ROW_DATA_1 + ROW_STEP * 0, 2);
  tft.drawString("EYES:",   COL_LABEL_1, ROW_DATA_1 + ROW_STEP * 1, 2);
  tft.drawString("SPOT:",   COL_LABEL_1, ROW_DATA_1 + ROW_STEP * 2, 2);
  tft.drawString("L-ARM:",  COL_LABEL_1, ROW_DATA_1 + ROW_STEP * 3, 2);
  tft.drawString("R-ARM:",  COL_LABEL_1, ROW_DATA_1 + ROW_STEP * 4, 2);
  tft.drawString("NECK:",   COL_LABEL_1, ROW_DATA_1 + ROW_STEP * 5, 2);
  tft.drawString("JAW:",    COL_LABEL_1, ROW_DATA_1 + ROW_STEP * 6, 2);

  // Vertical dimmed line separating pot column from phrase area
  tft.drawFastVLine(170, ROW_DIVIDER, ROW_DIVIDER2 - ROW_DIVIDER, C_DIM);

  // Right column — currently playing phrase label (yellow)
  tft.setTextColor(C_ACCENT, C_BG);
  tft.drawString("PLAYING:", COL_LABEL_2, ROW_DATA_1, 2);

  // Green divider line above battery row
  tft.drawFastHLine(0, ROW_DIVIDER2, 480, C_OK);

  // Battery row labels (cyan)
  tft.setTextColor(C_LABEL, C_BG);
  tft.drawString("BAT1:", COL_LABEL_1, ROW_BAT, 2);
  tft.drawString("BAT2:", COL_LABEL_2, ROW_BAT, 2);
  tft.drawString("BAT3:", COL_LABEL_3, ROW_BAT, 2);
}


// -----------------------------------------------------------------------------
//  drawWaiting()
//  Shown at startup before the first packet arrives. Drawn in dim grey so it
//  is visually distinct from live data and doesn't look like a frozen screen.
// -----------------------------------------------------------------------------
void drawWaiting() {
  tft.setTextColor(C_DIM, C_BG);
  tft.drawString("Waiting for controller...", COL_LABEL_2, ROW_DATA_1 + ROW_STEP * 2, 2);
}


// -----------------------------------------------------------------------------
//  drawValues()
//  Updates only the regions that contain live data. Each value area is erased
//  (filled black) before the new number is drawn to prevent digit ghosting
//  without having to redraw the entire screen.
// -----------------------------------------------------------------------------
void drawValues(const disp_pkt_t &pkt) {

  // --- Link status (top-right corner of header bar) ---
  // Green "LINKED" when ESP-NOW is healthy; red "NO LINK" when the TTGO
  // has lost contact with the robot receiver.
  tft.fillRect(340, ROW_HEADER - 2, 135, 28, C_HEADER);
  tft.setTextColor(pkt.connect_ok ? C_OK : C_ERR, C_HEADER);
  tft.drawString(pkt.connect_ok ? "LINKED" : "NO LINK", 345, ROW_HEADER, 4);

  // --- Potentiometer values (left column) ---
  // Erase the entire value column in one pass, then redraw all seven numbers.
  // This is faster than erasing each cell individually.
  tft.setTextColor(C_VALUE, C_BG);
  for (int row = 0; row < 7; row++) {
    tft.fillRect(COL_VALUE_1, ROW_DATA_1 + ROW_STEP * row, 70, ROW_STEP - 2, C_BG);
  }
  tft.drawString(String(pkt.volume),    COL_VALUE_1, ROW_DATA_1 + ROW_STEP * 0, 2);
  tft.drawString(String(pkt.eyes),      COL_VALUE_1, ROW_DATA_1 + ROW_STEP * 1, 2);
  tft.drawString(String(pkt.spot),      COL_VALUE_1, ROW_DATA_1 + ROW_STEP * 2, 2);
  tft.drawString(String(pkt.left_arm),  COL_VALUE_1, ROW_DATA_1 + ROW_STEP * 3, 2);
  tft.drawString(String(pkt.right_arm), COL_VALUE_1, ROW_DATA_1 + ROW_STEP * 4, 2);
  tft.drawString(String(pkt.neck),      COL_VALUE_1, ROW_DATA_1 + ROW_STEP * 5, 2);
  tft.drawString(String(pkt.jaw),       COL_VALUE_1, ROW_DATA_1 + ROW_STEP * 6, 2);

  // --- Currently playing phrase (right column, row below "PLAYING:" label) ---
  tft.fillRect(COL_LABEL_2, ROW_DATA_1 + ROW_STEP, 300, ROW_STEP - 2, C_BG);
  tft.setTextColor(C_VALUE, C_BG);
  tft.drawString(String(pkt.phrase), COL_LABEL_2, ROW_DATA_1 + ROW_STEP, 2);

  // --- Battery voltages (bottom row) ---
  // BAT1 is the controller's own LiPo, stored as millivolts and converted here.
  // BAT2 / BAT3 are forwarded raw from the robot receiver — units TBD.
  tft.fillRect(COL_VALUE_1, ROW_BAT, 60, 18, C_BG);
  tft.fillRect(COL_VALUE_2, ROW_BAT, 60, 18, C_BG);
  tft.fillRect(COL_VALUE_3, ROW_BAT, 55, 18, C_BG);

  char bat1_str[10];
  dtostrf(pkt.bat1_mv / 1000.0f, 4, 2, bat1_str);

  tft.setTextColor(C_VALUE, C_BG);
  tft.drawString(String(bat1_str) + "V", COL_VALUE_1, ROW_BAT, 2);
  tft.drawString(String(pkt.bat2_raw),   COL_VALUE_2, ROW_BAT, 2);
  tft.drawString(String(pkt.bat3_raw),   COL_VALUE_3, ROW_BAT, 2);
}
