/*
  DW Client
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <M5Core2.h>

int status = WL_IDLE_STATUS;
char ssid[] = "";  //  your network SSID (name)
char pass[] = "";       // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key Index number (needed only for WEP)
IPAddress SendIP(10, 0, 0, 62);
uint16_t SendPort = 57345;
WiFiUDP Udp;
char ReplyBuffer[] = "acknowledged";
char packetBuffer[255] = { 0 };

#define NOT_CONNECTED_COLOR YELLOW
#define CONNECTED_COLOR DARKGREEN
#define BORDER_WIDTH 12
#define BUTTON_H 40
#define BUTTON_W 90
#define LCD_W 320
#define LCD_H 240
#define STATUS_1_X LCD_W / 4
#define STATUS_1_Y 75
#define STATUS_1_R 40
#define STATUS_1_5_X LCD_W / 4 * 2
#define STATUS_2_X LCD_W / 4 * 3
#define STATUS_2_Y 75
#define STATUS_2_R 40
#define STATUS_1_BAD_COLOR RED
#define STATUS_1_GOOD_COLOR GREEN
#define STATUS_1_UNKNOWN_COLOR LIGHTGREY
#define STATUS_2_BAD_COLOR RED
#define STATUS_2_GOOD_COLOR GREEN
#define STATUS_2_UNKNOWN_COLOR LIGHTGREY

#define STATUS_1_LOW_VAL 100.
#define STATUS_1_HIGH_VAL 120.
#define STATUS_2_LOW_VAL 40.
#define STATUS_2_HIGH_VAL 212.

// Defines the buttons. Colors in format {bg, text, outline}
ButtonColors on_clrs = { RED, WHITE, WHITE };
ButtonColors off_clrs = { BLACK, WHITE, WHITE };
Button connect_b(20, 0, 0, 0, false, " on", off_clrs, on_clrs, BC_DATUM);


bool g_is_connected = false;
float g_dist = 0.;
float g_temp = 0.;
uint32_t g_s1_c = STATUS_1_UNKNOWN_COLOR;
uint32_t g_s2_c = STATUS_2_UNKNOWN_COLOR;
uint32_t g_buzz_ctrl = 0;
uint8_t g_class = 0;
float g_conf = 0.;

void startHandler(Event& e) {
  Button& b = *e.button;
  Serial.println("Trying to connect");
  if (b != connect_b) return;
  Serial.println("Connecting");
  if (b != M5.background) {
    // Toggles the button color between black and blue
    b.off.bg = (b.off.bg == BLACK) ? BLUE : BLACK;
    b.draw();
  }

  //Send start data
  Udp.beginPacket(SendIP, SendPort);
  const char msg[] = "-1-0";
  auto r = Udp.write((const uint8_t*)msg, strlen(msg));
  Serial.println("write returned");
  Serial.println(r);
  Udp.endPacket();
}

void setup() {
  //Setup serial output
  Serial.begin(115200);
  while (!Serial)
    ;

  // check for the presence of the shield:
  Serial.println("start");

  // attempt to connect to Wifi network:
  WiFi.disconnect(true);
  Serial.println(status);

  // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {

    Serial.print("Attempting to connect to SSID: ");

    Serial.println(ssid);

    delay(10000);
  }

  M5.begin();
  M5.lcd.setTextColor(BLACK, WHITE);
  M5.Buttons.addHandler(startHandler, E_TAP);
  drawScreen();
}

void drawScreen() {
  Serial.printf("Sc %d %d\n", M5.lcd.width(), M5.lcd.height());
  M5.lcd.fillRect(0, 0, M5.lcd.width(), M5.lcd.height(), g_is_connected ? CONNECTED_COLOR : NOT_CONNECTED_COLOR);
  M5.lcd.fillRect(BORDER_WIDTH, BORDER_WIDTH, M5.lcd.width() - BORDER_WIDTH * 2, M5.lcd.height() - BORDER_WIDTH * 2, WHITE);
  doButtons();
  doStatus(g_class, g_conf, g_s1_c);
}

void doStatus(uint32_t detclass, float conf, uint32_t color) {
  static char s1_cv[120];
  static char s2_cv[120];
  static char class_cv[120];

  //if (detclass != 1 and detclass != 0  && conf < .8) return;
  sprintf(s1_cv, "%d", detclass);
  sprintf(s2_cv, "%f", conf);
  if (detclass == 0) {
    strcpy(class_cv, "    Empty   ");
    color = LIGHTGREY;
  }

  else if (detclass == 1) {
    strcpy(class_cv, "    Ball     ");
    color = RED;
  } else if (detclass == 2) {
    strcpy(class_cv, "Person Out");
    color = GREEN;
  } else if (detclass == 3) {
    strcpy(class_cv, " Person In ");
    color = YELLOW;
  }
  M5.lcd.drawString(class_cv, STATUS_1_5_X, STATUS_2_Y - STATUS_2_R - 2);
  M5.lcd.fillCircle(STATUS_1_5_X, STATUS_2_Y, STATUS_2_R, color);
  M5.lcd.drawString(s1_cv, STATUS_1_X, STATUS_1_Y + STATUS_1_R + 20);
  M5.lcd.drawString(s2_cv, STATUS_2_X, STATUS_2_Y + STATUS_2_R + 20);
}
void doButtons() {
  int16_t qw = M5.Lcd.width() / 2;
  int16_t h = M5.Lcd.height();
  connect_b.set(qw - BUTTON_W / 2, h - (BORDER_WIDTH + BUTTON_H + 10), BUTTON_W, BUTTON_H);

  M5.Buttons.draw();
}

void updateValues() {
  int packetSize = Udp.parsePacket();

  if (packetSize) {
    int len = Udp.read(packetBuffer, 255);
    Serial.printf("Class:%d Conf:%f\n", *(uint32_t*)&packetBuffer, *(float*)&packetBuffer[4]);
    g_class = *(uint32_t*)&packetBuffer;
    g_conf = *(float*)&packetBuffer[4];

    if (len > 0) {
      packetBuffer[len] = 0;
    }
  }
}
void udpConnect() {
  g_is_connected = true;

  drawScreen();
}

uint64_t app_s = 0;
void loop() {
  app_s++;
  if ((app_s % 10000) == 0) {
    updateValues();
    doStatus(g_class, g_conf, g_s1_c);
  }
  M5.update();
}

