//******************************************************************************
//       file name:  j4_display_v0_1
//     v0_1 created:  2026-05-22
//     last updated:  2026-05-22
//           author:  Kevin Lange
//      description:  Secondary display board for the Johnny 4 controller.
//                    Receives data from the TTGO T-Display v1.1 over UART
//                    and renders a status dashboard on a 3.5" ILI9488 TFT.
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
//******************************************************************************


#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>

// UART link to TTGO T-Display v1.1
#define LINK_RX    44      // D7 ← TTGO GPIO17
#define LINK_TX    43      // D6 → TTGO GPIO27
#define LINK_BAUD  115200

// Packet sent by the TTGO — must match disp_pkt_t in j4_controller/src/main.cpp exactly
typedef struct __attribute__((packed)) {
  uint8_t  magic[2];       // 0xAB, 0xCD
  uint8_t  volume;         // 0–100
  uint8_t  eyes;           // 0–255
  uint8_t  spot;           // 0–255
  uint8_t  left_arm;       // 0–255
  uint8_t  right_arm;      // 0–255
  uint8_t  neck;           // 0–255
  uint8_t  jaw;            // 0–255
  uint16_t bat1_mv;        // controller battery in millivolts
  int16_t  bat2_raw;       // robot battery 2 raw value from receiver
  int16_t  bat3_raw;       // robot battery 3 raw value from receiver
  uint8_t  connect_ok;     // 1 = ESP-NOW link healthy, 0 = error
  char     phrase[32];     // null-terminated phrase name
  uint8_t  checksum;       // XOR of all preceding bytes
} disp_pkt_t;

TFT_eSPI tft = TFT_eSPI();

disp_pkt_t rxPkt;
bool firstPacketReceived = false;

// Colors
#define C_BG       TFT_BLACK
#define C_LABEL    0x07FF   // cyan
#define C_VALUE    TFT_WHITE
#define C_OK       TFT_GREEN
#define C_ERR      TFT_RED
#define C_ACCENT   TFT_YELLOW
#define C_DIM      0x4208   // dark grey

// Layout constants (480×320 landscape)
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

void drawStaticLayout();
bool receivePkt();
void drawValues(const disp_pkt_t &pkt);
void drawWaiting();


void setup() {
  Serial.begin(115200);   // USB CDC for debug output

  // UART link — use Serial1 on GPIO43/44
  Serial1.begin(LINK_BAUD, SERIAL_8N1, LINK_RX, LINK_TX);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  tft.init();
  tft.setRotation(3);          // landscape, 480 wide × 320 tall
  tft.fillScreen(C_BG);
  tft.setSwapBytes(true);

  drawStaticLayout();
  drawWaiting();
}


void loop() {
  if (receivePkt()) {
    if (!firstPacketReceived) {
      firstPacketReceived = true;
      drawStaticLayout();    // redraw clean labels after waiting screen
    }
    drawValues(rxPkt);
  }
}


// Hunt for a valid packet in the Serial1 stream.
// Returns true when a complete, checksum-verified packet is ready in rxPkt.
bool receivePkt() {
  static uint8_t buf[sizeof(disp_pkt_t)];
  static size_t  pos = 0;

  while (Serial1.available()) {
    uint8_t b = (uint8_t)Serial1.read();

    // Re-sync on magic bytes
    if (pos == 0 && b != 0xAB) continue;
    if (pos == 1 && b != 0xCD) { pos = 0; continue; }

    buf[pos++] = b;

    if (pos == sizeof(disp_pkt_t)) {
      pos = 0;
      uint8_t cs = 0;
      for (size_t i = 0; i < sizeof(disp_pkt_t) - 1; i++) cs ^= buf[i];
      if (cs != buf[sizeof(disp_pkt_t) - 1]) {
        Serial.println("XIAO: checksum error");
        return false;
      }
      memcpy(&rxPkt, buf, sizeof(disp_pkt_t));
      return true;
    }
  }
  return false;
}


void drawStaticLayout() {
  tft.fillScreen(C_BG);

  // Header bar
  tft.fillRect(0, 0, 480, ROW_DIVIDER - 2, 0x1082);   // very dark blue
  tft.setTextColor(C_OK, 0x1082);
  tft.drawString("J4 CONTROLLER", COL_LABEL_1, ROW_HEADER, 4);

  tft.drawFastHLine(0, ROW_DIVIDER, 480, C_OK);

  // Column 1 labels
  tft.setTextColor(C_LABEL, C_BG);
  tft.drawString("VOL:",    COL_LABEL_1, ROW_DATA_1 + ROW_STEP * 0, 2);
  tft.drawString("EYES:",   COL_LABEL_1, ROW_DATA_1 + ROW_STEP * 1, 2);
  tft.drawString("SPOT:",   COL_LABEL_1, ROW_DATA_1 + ROW_STEP * 2, 2);
  tft.drawString("L-ARM:",  COL_LABEL_1, ROW_DATA_1 + ROW_STEP * 3, 2);
  tft.drawString("R-ARM:",  COL_LABEL_1, ROW_DATA_1 + ROW_STEP * 4, 2);
  tft.drawString("NECK:",   COL_LABEL_1, ROW_DATA_1 + ROW_STEP * 5, 2);
  tft.drawString("JAW:",    COL_LABEL_1, ROW_DATA_1 + ROW_STEP * 6, 2);

  // Right area label
  tft.drawFastVLine(170, ROW_DIVIDER, ROW_DIVIDER2 - ROW_DIVIDER, C_DIM);
  tft.setTextColor(C_ACCENT, C_BG);
  tft.drawString("PLAYING:", COL_LABEL_2, ROW_DATA_1, 2);

  // Battery row
  tft.drawFastHLine(0, ROW_DIVIDER2, 480, C_OK);
  tft.setTextColor(C_LABEL, C_BG);
  tft.drawString("BAT1:", COL_LABEL_1,  ROW_BAT, 2);
  tft.drawString("BAT2:", COL_LABEL_2,  ROW_BAT, 2);
  tft.drawString("BAT3:", COL_LABEL_3,  ROW_BAT, 2);
}


void drawWaiting() {
  tft.setTextColor(C_DIM, C_BG);
  tft.drawString("Waiting for controller...", COL_LABEL_2, ROW_DATA_1 + ROW_STEP * 2, 2);
}


void drawValues(const disp_pkt_t &pkt) {
  // Link status (top-right of header)
  tft.fillRect(340, ROW_HEADER - 2, 135, 28, 0x1082);
  tft.setTextColor(pkt.connect_ok ? C_OK : C_ERR, 0x1082);
  tft.drawString(pkt.connect_ok ? "LINKED" : "NO LINK", 345, ROW_HEADER, 4);

  // Pot values — erase then redraw to avoid ghosting
  int valueX = COL_VALUE_1;
  int clearW = 70;
  tft.setTextColor(C_VALUE, C_BG);

  for (int row = 0; row < 7; row++) {
    tft.fillRect(valueX, ROW_DATA_1 + ROW_STEP * row, clearW, ROW_STEP - 2, C_BG);
  }
  tft.drawString(String(pkt.volume),    valueX, ROW_DATA_1 + ROW_STEP * 0, 2);
  tft.drawString(String(pkt.eyes),      valueX, ROW_DATA_1 + ROW_STEP * 1, 2);
  tft.drawString(String(pkt.spot),      valueX, ROW_DATA_1 + ROW_STEP * 2, 2);
  tft.drawString(String(pkt.left_arm),  valueX, ROW_DATA_1 + ROW_STEP * 3, 2);
  tft.drawString(String(pkt.right_arm), valueX, ROW_DATA_1 + ROW_STEP * 4, 2);
  tft.drawString(String(pkt.neck),      valueX, ROW_DATA_1 + ROW_STEP * 5, 2);
  tft.drawString(String(pkt.jaw),       valueX, ROW_DATA_1 + ROW_STEP * 6, 2);

  // Phrase playing
  tft.fillRect(COL_LABEL_2, ROW_DATA_1 + ROW_STEP, 300, ROW_STEP - 2, C_BG);
  tft.setTextColor(C_VALUE, C_BG);
  tft.drawString(String(pkt.phrase), COL_LABEL_2, ROW_DATA_1 + ROW_STEP, 2);

  // Battery voltages
  tft.fillRect(COL_VALUE_1,  ROW_BAT, 60, 18, C_BG);
  tft.fillRect(COL_VALUE_2,  ROW_BAT, 60, 18, C_BG);
  tft.fillRect(COL_VALUE_3,  ROW_BAT, 55, 18, C_BG);

  char bat1_str[10];
  float bat1_v = pkt.bat1_mv / 1000.0f;
  dtostrf(bat1_v, 4, 2, bat1_str);

  tft.setTextColor(C_VALUE, C_BG);
  tft.drawString(String(bat1_str) + "V", COL_VALUE_1, ROW_BAT, 2);
  tft.drawString(String(pkt.bat2_raw),   COL_VALUE_2, ROW_BAT, 2);
  tft.drawString(String(pkt.bat3_raw),   COL_VALUE_3, ROW_BAT, 2);
}
