//******************************************************************************
//       file name:  j4_display
//     v0_1 created:  2026-05-22
//     last updated:  2026-05-31 -- 1207 CDT
//     last updated:  2026-06-02 -- 1100 CDT
//           author:  Kevin Lange
//      description:  Secondary display board for the Johnny 4 controller.
//                    Receives control data from the TTGO T-Display v1.1 over
//                    a wired UART link and renders a live status dashboard on
//                    a 3.5" ILI9488 480x320 SPI TFT display.
//
//
//      J4_DISPLAY - SeeedStudio XIAO ESP32S3 Pin Connections
//      ------------------------------------------------------------------
//      D0 / GPIO1   TFT backlight (HIGH = on)
//      D1 / GPIO2   TFT RST
//      D2 / GPIO3   TFT DC
//      D3 / GPIO4   TFT CS
//      D4 / GPIO5   (free - I2C SDA if needed later)
//      D5 / GPIO6   (free - I2C SCL if needed later)
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
//  UART LINK  -  Serial1 on GPIO43 (TX) / GPIO44 (RX)
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
  uint8_t  volume;         // master volume knob         (0-100)
  uint8_t  eyes;           // eye position/brightness    (0-255)
  uint8_t  spot;           // spotlight intensity        (0-255)
  uint8_t  left_arm;       // left arm servo position    (0-255)
  uint8_t  right_arm;      // right arm servo position   (0-255)
  uint8_t  neck;           // neck servo position        (0-255)
  uint8_t  jaw;            // jaw servo position         (0-255)
  uint16_t bat1_mv;        // controller battery voltage (millivolts)
  int16_t  bat2_raw;       // robot battery 2 - raw value forwarded from receiver
  int16_t  bat3_raw;       // robot battery 3 - raw value forwarded from receiver
  uint8_t  connect_ok;     // ESP-NOW link health: 1 = linked, 0 = error
  char     phrase[32];     // currently selected audio phrase, null-terminated (e.g. "A12")
  uint8_t  checksum;       // XOR of every byte above - used to detect corrupt packets
} disp_pkt_t;


// -----------------------------------------------------------------------------
//  FILE LIST PACKET
//  Second packet type on the same UART link - magic 0xBE, 0xCD.
//  Must stay byte-for-byte identical to filelist_pkt_t in j4_controller.
// -----------------------------------------------------------------------------
#define MAX_FILES      50
#define FILE_ID_LEN     2
#define FILE_NAME_MAX  22

typedef struct __attribute__((packed)) {
  uint8_t magic[2];        // 0xBE, 0xCD
  uint8_t total_files;
  struct {
    char id[FILE_ID_LEN];
    char name[FILE_NAME_MAX];
  } files[MAX_FILES];
  uint8_t checksum;
} filelist_pkt_t;

// Banks: index 0 = numeric (IDs 00-09), 1-4 = A-D
#define NUM_BANKS       5
#define FILES_PER_BANK 10

struct BankEntry {
  bool  valid;
  char  id[FILE_ID_LEN + 1];
  char  name[FILE_NAME_MAX + 1];
};

// -----------------------------------------------------------------------------
//  COLOR PALETTE  (RGB565)
// -----------------------------------------------------------------------------
#define C_BG       TFT_BLACK    // background
#define C_LABEL    0x07FF       // cyan  - pot/battery label text
#define C_VALUE    TFT_WHITE    // white - live numeric values
#define C_OK       TFT_GREEN    // green - "LINKED" status, divider lines
#define C_ERR      TFT_RED      // red   - "NO LINK" status
#define C_ACCENT   TFT_YELLOW   // yellow - "PLAYING:" label
#define C_DIM      0x4208       // dark grey - waiting message, divider lines
#define C_HEADER   0x1082       // very dark blue - header bar background


// -----------------------------------------------------------------------------
//  LAYOUT CONSTANTS  (all pixel positions for 480 × 320 landscape)
//
//  Columns:
//    COL_LABEL_1 / COL_VALUE_1  - left pot-value column
//    COL_LABEL_2 / COL_VALUE_2  - right "PLAYING" / BAT2 column
//    COL_LABEL_3 / COL_VALUE_3  - BAT3 column (battery row only)
//
//  Rows:
//    ROW_HEADER   - vertical centre of header text
//    ROW_DIVIDER  - top green line separating header from data area
//    ROW_DATA_1   - first pot-value row (VOL)
//    ROW_STEP     - pixel height of each pot-value row
//    ROW_DIVIDER2 - bottom green line separating data area from battery row
//    ROW_BAT      - battery label/value row
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
//  JUKEBOX LAYOUT CONSTANTS
// -----------------------------------------------------------------------------
//  Jukebox panel occupies the right half of the screen (x >= 245).
//  Left half retains the existing pot-value column.
//
//  Bank tab row: y = ROW_DIVIDER..ROW_DIVIDER+22  (one tab per bank)
//  File list:    10 rows below that, ROW_STEP apart
//
#define JB_LEFT         245   // left edge of jukebox panel
#define JB_TAB_Y        (ROW_DIVIDER + 2)
#define JB_TAB_H        20
#define JB_TAB_W        45
#define JB_LIST_Y       (JB_TAB_Y + JB_TAB_H + 4)
#define JB_ROW_H        17
#define JB_ID_X         (JB_LEFT + 4)
#define JB_NAME_X       (JB_LEFT + 30)

// -----------------------------------------------------------------------------
//  GLOBALS
// -----------------------------------------------------------------------------
TFT_eSPI tft = TFT_eSPI();

disp_pkt_t rxPkt;                  // most recently received and verified packet
bool firstPacketReceived = false;  // used to redraw labels cleanly on first data

// Jukebox
BankEntry banks[NUM_BANKS][FILES_PER_BANK];
bool      jukeboxReceived = false;
uint8_t   activeBank      = 0;     // 0=numeric, 1=A, 2=B, 3=C, 4=D
char      prevPhrase[33]  = "";


// -----------------------------------------------------------------------------
//  FUNCTION PROTOTYPES
// -----------------------------------------------------------------------------
void drawStaticLayout();
int  readSerial();
bool receivePkt();
void drawValues(const disp_pkt_t &pkt);
void drawWaiting();
void loadFilelist(const filelist_pkt_t &pkt);
void drawJukeboxPanel();
void drawJukeboxTabs();
void drawJukeboxList();


// -----------------------------------------------------------------------------
//  SETUP
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);   // USB CDC - use for debug prints; does not affect LINK_RX/TX

  // UART link to the TTGO controller board
  Serial1.begin(LINK_BAUD, SERIAL_8N1, LINK_RX, LINK_TX);

  // TFT backlight - TFT_eSPI also configures this in tft.init(), but being
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
// Result codes from the unified serial reader
#define SERIAL_NONE      0
#define SERIAL_CONTROL   1
#define SERIAL_FILELIST  2

int readSerial();  // forward declaration - defined after receivePkt()

void loop() {
  int result = readSerial();

  if (result == SERIAL_FILELIST) {
    if (!jukeboxReceived) {
      jukeboxReceived = true;
      drawJukeboxPanel();
    }
  }

  if (result == SERIAL_CONTROL) {
    if (!firstPacketReceived) {
      firstPacketReceived = true;
      drawStaticLayout();
      if (jukeboxReceived) drawJukeboxPanel();
    }
    drawValues(rxPkt);
  }
}


// -----------------------------------------------------------------------------
//  readSerial()
//  Unified non-blocking parser for both packet types on Serial1.
//
//  State machine:
//    - Waits for 0xAB or 0xBE as the first magic byte.
//    - 0xAB -> expects 0xCD next -> control packet (disp_pkt_t)
//    - 0xBE -> expects 0xCD next -> file list packet (filelist_pkt_t)
//    - Any mismatch resets to hunting for the next 0xAB / 0xBE.
//
//  Returns SERIAL_CONTROL, SERIAL_FILELIST, or SERIAL_NONE.
// -----------------------------------------------------------------------------
int readSerial() {
  // Max packet size is filelist_pkt_t; control packets fit inside it.
  static uint8_t buf[sizeof(filelist_pkt_t)];
  static size_t  pos     = 0;
  static uint8_t pktType = 0;   // 0=unknown, 1=control, 2=filelist

  while (Serial1.available()) {
    uint8_t b = (uint8_t)Serial1.read();

    if (pos == 0) {
      if      (b == 0xAB) { pktType = 1; buf[pos++] = b; }
      else if (b == 0xBE) { pktType = 2; buf[pos++] = b; }
      continue;
    }

    if (pos == 1) {
      if (b != 0xCD) { pos = 0; pktType = 0; continue; }
      buf[pos++] = b;
      continue;
    }

    buf[pos++] = b;

    size_t expected = (pktType == 2) ? sizeof(filelist_pkt_t) : sizeof(disp_pkt_t);

    if (pos == expected) {
      pos    = 0;
      uint8_t cs = 0;
      for (size_t i = 0; i < expected - 1; i++) cs ^= buf[i];

      if (cs != buf[expected - 1]) {
        Serial.println("XIAO: checksum error - packet dropped");
        pktType = 0;
        return SERIAL_NONE;
      }

      if (pktType == 1) {
        memcpy(&rxPkt, buf, sizeof(disp_pkt_t));
        pktType = 0;
        return SERIAL_CONTROL;
      } else {
        filelist_pkt_t tmp;
        memcpy(&tmp, buf, sizeof(filelist_pkt_t));
        loadFilelist(tmp);
        pktType = 0;
        return SERIAL_FILELIST;
      }
    }
  }
  return SERIAL_NONE;
}

// Kept as a stub so existing call sites compile without changes.
bool receivePkt() { return false; }


// -----------------------------------------------------------------------------
//  drawStaticLayout()
//  Draws everything that never changes: header bar, divider lines, and all
//  text labels. Called once at startup and once again when the first real
//  packet arrives (to erase the "Waiting..." message cleanly).
// -----------------------------------------------------------------------------
void drawStaticLayout() {
  tft.fillScreen(C_BG);

  // Header bar - dark blue background with title text
  tft.fillRect(0, 0, 480, ROW_DIVIDER - 2, C_HEADER);
  tft.setTextColor(C_OK, C_HEADER);
  tft.drawString("J4 CONTROLLER", COL_LABEL_1, ROW_HEADER, 4);

  // Green divider line below header
  tft.drawFastHLine(0, ROW_DIVIDER, 480, C_OK);

  // Left column - potentiometer labels (cyan, font 2 ≈ 14px tall)
  tft.setTextColor(C_LABEL, C_BG);
  tft.drawString("VOL:",    COL_LABEL_1, ROW_DATA_1 + ROW_STEP * 0, 2);
  tft.drawString("EYES:",   COL_LABEL_1, ROW_DATA_1 + ROW_STEP * 1, 2);
  tft.drawString("SPOT:",   COL_LABEL_1, ROW_DATA_1 + ROW_STEP * 2, 2);
  tft.drawString("L-ARM:",  COL_LABEL_1, ROW_DATA_1 + ROW_STEP * 3, 2);
  tft.drawString("R-ARM:",  COL_LABEL_1, ROW_DATA_1 + ROW_STEP * 4, 2);
  tft.drawString("NK-L:",   COL_LABEL_1, ROW_DATA_1 + ROW_STEP * 5, 2);
  tft.drawString("NK-R:",   COL_LABEL_1, ROW_DATA_1 + ROW_STEP * 6, 2);

  // Vertical dimmed line separating pot column from middle area
  tft.drawFastVLine(170, ROW_DIVIDER, ROW_DIVIDER2 - ROW_DIVIDER, C_DIM);

  // Vertical dimmed line separating middle area from jukebox panel
  tft.drawFastVLine(JB_LEFT - 5, ROW_DIVIDER, ROW_DIVIDER2 - ROW_DIVIDER, C_DIM);

  // Middle column - currently playing phrase label (yellow)
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

  // setTextColor(fg, bg) makes TFT_eSPI draw foreground and background pixels
  // in a single SPI pass - no separate fillRect needed.
  // setTextPadding(n) extends that background fill to at least n pixels wide,
  // so a shorter new value (e.g. "9") always erases a longer old value (e.g. "100").
  // Result: no intermediate black frame, no flicker, and no sprite overhead.

  // --- Link status (top-right of header bar) ---
  // Redrawn only when the link state actually changes; one-time fillRect flicker
  // on a rare state change is acceptable.
  static uint8_t prev_connect_ok = 0xFF;
  if (pkt.connect_ok != prev_connect_ok) {
    prev_connect_ok = pkt.connect_ok;
    tft.fillRect(340, ROW_HEADER - 2, 135, 28, C_HEADER);
    tft.setTextColor(pkt.connect_ok ? C_OK : C_ERR, C_HEADER);
    tft.drawString(pkt.connect_ok ? "LINKED" : "NO LINK", 345, ROW_HEADER, 4);
  }

  // --- Potentiometer values (left column) ---
  // Padding of 45px covers the widest possible value ("255" or "100") in font 2.
  tft.setTextColor(C_VALUE, C_BG);
  tft.setTextPadding(45);
  tft.drawString(String(pkt.volume),    COL_VALUE_1, ROW_DATA_1 + ROW_STEP * 0, 2);
  tft.drawString(String(pkt.eyes),      COL_VALUE_1, ROW_DATA_1 + ROW_STEP * 1, 2);
  tft.drawString(String(pkt.spot),      COL_VALUE_1, ROW_DATA_1 + ROW_STEP * 2, 2);
  tft.drawString(String(pkt.left_arm),  COL_VALUE_1, ROW_DATA_1 + ROW_STEP * 3, 2);
  tft.drawString(String(pkt.right_arm), COL_VALUE_1, ROW_DATA_1 + ROW_STEP * 4, 2);
  tft.drawString(String(pkt.neck),      COL_VALUE_1, ROW_DATA_1 + ROW_STEP * 5, 2);
  tft.drawString(String(pkt.jaw),       COL_VALUE_1, ROW_DATA_1 + ROW_STEP * 6, 2);

  // --- Currently playing phrase (only redraw when it changes) ---
  if (strncmp(pkt.phrase, prevPhrase, sizeof(prevPhrase)) != 0) {
    strncpy(prevPhrase, pkt.phrase, sizeof(prevPhrase));
    tft.setTextPadding(0);
    tft.fillRect(COL_LABEL_2, ROW_DATA_1 + ROW_STEP, JB_LEFT - COL_LABEL_2 - 10, ROW_STEP - 2, C_BG);
    tft.setTextColor(C_VALUE, C_BG);
    tft.drawString(String(pkt.phrase), COL_LABEL_2, ROW_DATA_1 + ROW_STEP, 2);

    // Switch jukebox bank to match the playing track's prefix
    if (jukeboxReceived && strlen(pkt.phrase) >= 1) {
      char prefix = pkt.phrase[0];
      uint8_t bank = 0;
      if      (prefix == 'A' || prefix == 'a') bank = 1;
      else if (prefix == 'B' || prefix == 'b') bank = 2;
      else if (prefix == 'C' || prefix == 'c') bank = 3;
      else if (prefix == 'D' || prefix == 'd') bank = 4;
      if (bank != activeBank) {
        activeBank = bank;
        drawJukeboxPanel();
      }
    }
  }

  // --- Battery voltages (bottom row) ---
  // BAT1 is the controller LiPo converted from millivolts.
  // BAT2 / BAT3 are raw values forwarded from the robot receiver.
  char bat1_str[10];
  dtostrf(pkt.bat1_mv / 1000.0f, 4, 2, bat1_str);

  tft.setTextColor(C_VALUE, C_BG);
  tft.setTextPadding(60);  // covers up to 5 chars in font 2
  tft.drawString(String(bat1_str) + "V", COL_VALUE_1, ROW_BAT, 2);
  tft.drawString(String(pkt.bat2_raw),   COL_VALUE_2, ROW_BAT, 2);
  tft.drawString(String(pkt.bat3_raw),   COL_VALUE_3, ROW_BAT, 2);
  tft.setTextPadding(0);
}



void loadFilelist(const filelist_pkt_t &pkt) {
  memset(banks, 0, sizeof(banks));

  for (uint8_t i = 0; i < pkt.total_files && i < MAX_FILES; i++) {
    char id0 = pkt.files[i].id[0];
    char id1 = pkt.files[i].id[1];

    uint8_t bank  = 0;
    uint8_t slot  = 0;
    bool    valid = false;

    if (id0 >= '0' && id0 <= '9' && id1 >= '0' && id1 <= '9') {
      bank  = 0;
      slot  = id1 - '0';
      valid = true;
    } else {
      char uc = (id0 >= 'a' && id0 <= 'd') ? (id0 - 32) : id0;
      if (uc >= 'A' && uc <= 'D' && id1 >= '0' && id1 <= '9') {
        bank  = (uc - 'A') + 1;
        slot  = id1 - '0';
        valid = true;
      }
    }

    if (valid && slot < FILES_PER_BANK) {
      banks[bank][slot].valid = true;
      memcpy(banks[bank][slot].id, pkt.files[i].id, FILE_ID_LEN);
      banks[bank][slot].id[FILE_ID_LEN] = '\0';
      strncpy(banks[bank][slot].name, pkt.files[i].name, FILE_NAME_MAX);
      banks[bank][slot].name[FILE_NAME_MAX] = '\0';
    }
  }
}


void drawJukeboxPanel() {
  tft.fillRect(JB_LEFT, ROW_DIVIDER + 1, 480 - JB_LEFT, ROW_DIVIDER2 - ROW_DIVIDER - 2, C_BG);
  drawJukeboxTabs();
  drawJukeboxList();
}


void drawJukeboxTabs() {
  const char *labels[NUM_BANKS] = { "0-9", "A", "B", "C", "D" };

  for (uint8_t b = 0; b < NUM_BANKS; b++) {
    int16_t tx = JB_LEFT + b * (JB_TAB_W + 2);
    bool    sel = (b == activeBank);

    tft.fillRect(tx, JB_TAB_Y, JB_TAB_W, JB_TAB_H, sel ? C_OK : C_DIM);
    tft.setTextColor(sel ? C_BG : C_VALUE, sel ? C_OK : C_DIM);
    tft.drawString(labels[b], tx + 4, JB_TAB_Y + 3, 2);
  }
}


void drawJukeboxList() {
  for (uint8_t row = 0; row < FILES_PER_BANK; row++) {
    int16_t y = JB_LIST_Y + row * JB_ROW_H;
    const BankEntry &e = banks[activeBank][row];

    // Clear the row
    tft.fillRect(JB_LEFT, y, 480 - JB_LEFT, JB_ROW_H - 1, C_BG);

    if (e.valid) {
      // ID  (cyan, bold)
      tft.setTextColor(C_LABEL, C_BG);
      tft.drawString(String(e.id), JB_ID_X, y, 2);

      // Name (white, truncated to available width)
      tft.setTextColor(C_VALUE, C_BG);
      tft.setTextPadding(480 - JB_NAME_X - 4);
      tft.drawString(String(e.name), JB_NAME_X, y, 2);
      tft.setTextPadding(0);
    }
  }
}
