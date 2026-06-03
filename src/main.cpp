//******************************************************************************
//       file name:  j4_display
//     v0_1 created:  2026-05-22
//     last updated:  2026-05-31 -- 1207 CDT
//     last updated:  2026-06-02 -- 1100 CDT
//     last updated:  2026-06-02 -- 1145 CDT
//           author:  Kevin Lange
//      description:  Dedicated jukebox display for the Johnny 4 controller.
//                    Receives file list and now-playing data from the TTGO
//                    j4_controller over UART and renders a jukebox selection
//                    UI on a 3.5" ILI9488 320x480 SPI TFT display.
//
//
//      J4_DISPLAY - SeeedStudio XIAO ESP32S3 Pin Connections
//      ------------------------------------------------------------------
//      D0 / GPIO1   TFT backlight (HIGH = on)
//      D1 / GPIO2   TFT RST
//      D2 / GPIO3   TFT DC
//      D3 / GPIO4   TFT CS
//      D4 / GPIO5   (free)
//      D5 / GPIO6   (free)
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
//      Two binary packet types share Serial1:
//        0xAB 0xCD -- disp_pkt_t     (control data, 25 fps)
//        0xBE 0xCD -- filelist_pkt_t (file list, sent once at boot)
//      Both end with an XOR checksum. Corrupted packets are silently dropped.
//      ------------------------------------------------------------------
//
//
//      DISPLAY LAYOUT  (320 x 480 portrait)
//      ------------------------------------------------------------------
//      ┌──────────────────────┐  y=0
//      │        KEVCO         │  header (56px, dark blue bg)
//      ├──────────────────────┤  green line
//      │ [0-9] [A] [B] [C] [D]│  bank tabs (28px, 64px each)
//      ├──────────────────────┤  dim line
//      │ 01 | filename.wav    │
//      │ 02 | filename.wav    │
//      │ ...                  │  10 file rows (~36px each)
//      │ 09 | filename.wav    │
//      │ 00 | filename.wav    │
//      ├──────────────────────┤  dim line
//      │   # STOP  *  CLEAR   │  bottom bar (26px)
//      └──────────────────────┘  y=480
//      ------------------------------------------------------------------
//      Active bank tab: green background, black text
//      Currently playing row: green background, black text
//      ------------------------------------------------------------------
//******************************************************************************


#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>


// -----------------------------------------------------------------------------
//  UART LINK
// -----------------------------------------------------------------------------
#define LINK_RX    44      // D7 receives from TTGO GPIO17
#define LINK_TX    43      // D6 sends to   TTGO GPIO27
#define LINK_BAUD  115200


// -----------------------------------------------------------------------------
//  PACKET STRUCTURES
//  Must stay byte-for-byte identical to matching structs in j4_controller.
// -----------------------------------------------------------------------------
typedef struct __attribute__((packed)) {
  uint8_t  magic[2];
  uint8_t  volume;
  uint8_t  eyes;
  uint8_t  spot;
  uint8_t  left_arm;
  uint8_t  right_arm;
  uint8_t  neck;
  uint8_t  jaw;
  uint16_t bat1_mv;
  int16_t  bat2_raw;
  int16_t  bat3_raw;
  uint8_t  connect_ok;
  char     phrase[32];     // currently playing track ID, e.g. "A3"
  uint8_t  checksum;
} disp_pkt_t;

#define MAX_FILES      50
#define FILE_ID_LEN     2
#define FILE_NAME_MAX  22

typedef struct __attribute__((packed)) {
  uint8_t magic[2];
  uint8_t total_files;
  struct {
    char id[FILE_ID_LEN];
    char name[FILE_NAME_MAX];
  } files[MAX_FILES];
  uint8_t checksum;
} filelist_pkt_t;

#define NUM_BANKS       5
#define FILES_PER_BANK 10

struct BankEntry {
  bool valid;
  char id[FILE_ID_LEN + 1];
  char name[FILE_NAME_MAX + 1];
};


// -----------------------------------------------------------------------------
//  COLOR PALETTE
// -----------------------------------------------------------------------------
#define C_BG     TFT_BLACK
#define C_GREEN  TFT_GREEN
#define C_DIM    0x4208      // dark grey
#define C_HEADER 0x1082      // very dark blue


// -----------------------------------------------------------------------------
//  LAYOUT  (320 x 480 portrait)
//  If the display appears upside down, change setRotation(0) to setRotation(2).
// -----------------------------------------------------------------------------
#define SCREEN_W  320
#define SCREEN_H  480

#define HDR_Y       0
#define HDR_H      56

#define TAB_Y      (HDR_H + 2)
#define TAB_H      28
#define TAB_W      (SCREEN_W / NUM_BANKS)   // 64px each

#define LIST_Y     (TAB_Y + TAB_H + 3)
#define BOT_H      26
#define BOT_Y      (SCREEN_H - BOT_H)
#define ROW_H      ((BOT_Y - LIST_Y) / FILES_PER_BANK)

#define ID_X        6
#define SEP_X      34
#define NAME_X     40
#define NAME_W     (SCREEN_W - NAME_X - 4)

// Row order: slots 1-9 first, then slot 0 (matches keypad layout)
const uint8_t ROW_TO_SLOT[FILES_PER_BANK] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 0 };


// -----------------------------------------------------------------------------
//  GLOBALS
// -----------------------------------------------------------------------------
TFT_eSPI tft = TFT_eSPI();

disp_pkt_t rxPkt;

BankEntry banks[NUM_BANKS][FILES_PER_BANK];
bool      jukeboxReceived = false;
uint8_t   activeBank      = 0;
char      prevPhrase[33]  = "";
int8_t    playingBank     = -1;   // -1 = nothing playing
int8_t    playingSlot     = -1;


// -----------------------------------------------------------------------------
//  FUNCTION PROTOTYPES
// -----------------------------------------------------------------------------
#define SERIAL_NONE     0
#define SERIAL_CONTROL  1
#define SERIAL_FILELIST 2

int  readSerial();
void loadFilelist(const filelist_pkt_t &pkt);
void handlePhraseChange(const char *phrase);
void drawScreen();
void drawHeader();
void drawBankTabs();
void drawFileList();
void drawRow(uint8_t display_row, bool highlighted);
void drawBottomBar();


// -----------------------------------------------------------------------------
//  SETUP
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial1.begin(LINK_BAUD, SERIAL_8N1, LINK_RX, LINK_TX);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  tft.init();
  tft.setRotation(0);    // portrait 320x480; change to 2 if display is upside down
  tft.fillScreen(C_BG);
  tft.setSwapBytes(true);

  drawScreen();
}


// -----------------------------------------------------------------------------
//  MAIN LOOP
// -----------------------------------------------------------------------------
void loop() {
  int result = readSerial();

  if (result == SERIAL_FILELIST) {
    jukeboxReceived = true;
    drawBankTabs();
    drawFileList();
  }

  if (result == SERIAL_CONTROL && strlen(rxPkt.phrase) > 0) {
    if (strncmp(rxPkt.phrase, prevPhrase, sizeof(prevPhrase)) != 0) {
      strncpy(prevPhrase, rxPkt.phrase, sizeof(prevPhrase));
      handlePhraseChange(rxPkt.phrase);
    }
  }
}


// -----------------------------------------------------------------------------
//  readSerial()
//  Non-blocking unified parser. Detects packet type from the first magic byte:
//    0xAB -> disp_pkt_t (control)
//    0xBE -> filelist_pkt_t (file list)
//  Both must be followed by 0xCD or the buffer resets.
// -----------------------------------------------------------------------------
int readSerial() {
  static uint8_t buf[sizeof(filelist_pkt_t)];
  static size_t  pos     = 0;
  static uint8_t pktType = 0;

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
      pos = 0;
      uint8_t cs = 0;
      for (size_t i = 0; i < expected - 1; i++) cs ^= buf[i];

      if (cs != buf[expected - 1]) {
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


// -----------------------------------------------------------------------------
//  loadFilelist()
//  Sorts incoming files into the banks[] array by ID prefix.
// -----------------------------------------------------------------------------
void loadFilelist(const filelist_pkt_t &pkt) {
  memset(banks, 0, sizeof(banks));

  for (uint8_t i = 0; i < pkt.total_files && i < MAX_FILES; i++) {
    char id0 = pkt.files[i].id[0];
    char id1 = pkt.files[i].id[1];

    uint8_t bank = 0;
    uint8_t slot = 0;
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


// -----------------------------------------------------------------------------
//  handlePhraseChange()
//  Highlights the now-playing row. If the playing track is in a different bank,
//  switches the active bank first.
// -----------------------------------------------------------------------------
void handlePhraseChange(const char *phrase) {
  int8_t newBank = -1;
  int8_t newSlot = -1;

  if (strlen(phrase) >= 2) {
    char c0 = phrase[0];
    char c1 = phrase[1];

    if (c0 >= '0' && c0 <= '9' && c1 >= '0' && c1 <= '9') {
      newBank = 0;
      newSlot = c1 - '0';
    } else {
      char uc = (c0 >= 'a' && c0 <= 'd') ? (c0 - 32) : c0;
      if (uc >= 'A' && uc <= 'D' && c1 >= '0' && c1 <= '9') {
        newBank = (uc - 'A') + 1;
        newSlot = c1 - '0';
      }
    }
  }

  bool bankChanged = (newBank >= 0 && newBank != (int8_t)activeBank);

  // Remove highlight from old row if still in the same bank view
  if (!bankChanged && playingBank == (int8_t)activeBank && playingSlot >= 0) {
    for (uint8_t r = 0; r < FILES_PER_BANK; r++) {
      if (ROW_TO_SLOT[r] == (uint8_t)playingSlot) { drawRow(r, false); break; }
    }
  }

  playingBank = newBank;
  playingSlot = newSlot;

  if (bankChanged) {
    activeBank = (uint8_t)newBank;
    drawBankTabs();
    drawFileList();
  } else if (newBank == (int8_t)activeBank && newSlot >= 0) {
    for (uint8_t r = 0; r < FILES_PER_BANK; r++) {
      if (ROW_TO_SLOT[r] == (uint8_t)newSlot) { drawRow(r, true); break; }
    }
  }
}


// -----------------------------------------------------------------------------
//  DRAWING
// -----------------------------------------------------------------------------

void drawScreen() {
  tft.fillScreen(C_BG);
  drawHeader();
  tft.drawFastHLine(0, HDR_H, SCREEN_W, C_GREEN);
  drawBankTabs();
  tft.drawFastHLine(0, TAB_Y + TAB_H + 1, SCREEN_W, C_DIM);
  drawFileList();
  tft.drawFastHLine(0, BOT_Y - 1, SCREEN_W, C_DIM);
  drawBottomBar();
}


void drawHeader() {
  tft.fillRect(0, HDR_Y, SCREEN_W, HDR_H, C_HEADER);
  tft.setTextColor(C_GREEN, C_HEADER);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("KEVCO", SCREEN_W / 2, HDR_Y + HDR_H / 2, 4);
  tft.setTextDatum(TL_DATUM);
}


void drawBankTabs() {
  const char *labels[NUM_BANKS] = { "0-9", "A", "B", "C", "D" };

  for (uint8_t b = 0; b < NUM_BANKS; b++) {
    int16_t tx  = b * TAB_W;
    bool    sel = (b == activeBank);

    tft.fillRect(tx, TAB_Y, TAB_W - 1, TAB_H, sel ? C_GREEN : C_DIM);
    tft.setTextColor(sel ? C_BG : TFT_WHITE, sel ? C_GREEN : C_DIM);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(labels[b], tx + TAB_W / 2, TAB_Y + TAB_H / 2, 2);
    tft.setTextDatum(TL_DATUM);
  }
}


void drawRow(uint8_t display_row, bool highlighted) {
  uint8_t slot = ROW_TO_SLOT[display_row];
  int16_t y    = LIST_Y + display_row * ROW_H;
  const BankEntry &e = banks[activeBank][slot];

  uint32_t bg  = highlighted ? C_GREEN : C_BG;
  uint32_t fg  = highlighted ? C_BG    : C_GREEN;
  uint32_t sep = highlighted ? C_BG    : C_DIM;

  tft.fillRect(0, y, SCREEN_W, ROW_H - 1, bg);

  if (e.valid) {
    tft.setTextColor(fg, bg);
    tft.drawString(String(e.id), ID_X, y + 2, 2);

    tft.drawFastVLine(SEP_X, y + 4, ROW_H - 9, sep);

    tft.setTextPadding(NAME_W);
    tft.drawString(String(e.name), NAME_X, y + 2, 2);
    tft.setTextPadding(0);
  }
}


void drawFileList() {
  for (uint8_t r = 0; r < FILES_PER_BANK; r++) {
    uint8_t slot      = ROW_TO_SLOT[r];
    bool    highlight = (playingBank == (int8_t)activeBank && playingSlot == (int8_t)slot);
    drawRow(r, highlight);
  }
}


void drawBottomBar() {
  tft.fillRect(0, BOT_Y, SCREEN_W, BOT_H, C_BG);
  tft.setTextColor(C_DIM, C_BG);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("# STOP    *  CLEAR", SCREEN_W / 2, BOT_Y + BOT_H / 2, 2);
  tft.setTextDatum(TL_DATUM);
}
