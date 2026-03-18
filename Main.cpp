#include <Arduino.h>

// UART2 pins (typisch):
// ESP32 GPIO17 (TX2) -> Motor RX
// ESP32 GPIO16 (RX2) <- Motor TX
static const int DDSM_RX = 16; // ESP32 RX2
static const int DDSM_TX = 17; // ESP32 TX2

static const uint32_t BAUD = 115200;
static const uint8_t MOTOR_ID = 1;

// ---------- CRC-8/MAXIM (Dallas/Maxim) ----------
// Wiki: CRC8 über DATA[0]..DATA[8], Algorithmus CRC-8/MAXIM, Poly x^8+x^5+x^4+1  [oai_citation:2‡Waveshare](https://www.waveshare.com/wiki/DDSM210?srsltid=AfmBOoq7D7Z__qriczWc_kmWMWZZn5Kkax7JqxRCivi_0GvfEgxB4lKj)
uint8_t crc8_maxim(const uint8_t *data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      if (crc & 0x01) crc = (crc >> 1) ^ 0x8C; // reflected poly
      else crc >>= 1;
    }
  }
  return crc;
}

// Build a generic 10-byte frame (Protocol 1 / common layout in wiki)
void buildFrame(uint8_t out[10],
                uint8_t id,
                uint8_t cmd,
                int16_t value_s16,   // speed: 0.1 rpm units (signed 16-bit)  [oai_citation:3‡Waveshare](https://www.waveshare.com/wiki/DDSM210?srsltid=AfmBOoq7D7Z__qriczWc_kmWMWZZn5Kkax7JqxRCivi_0GvfEgxB4lKj)
                uint8_t fb1,
                uint8_t fb2,
                uint8_t accTime,
                uint8_t brake,
                uint8_t zero = 0x00) {
  out[0] = id;
  out[1] = cmd;
  out[2] = (uint8_t)((value_s16 >> 8) & 0xFF);
  out[3] = (uint8_t)(value_s16 & 0xFF);
  out[4] = fb1;
  out[5] = fb2;
  out[6] = accTime;
  out[7] = brake;
  out[8] = zero;
  out[9] = crc8_maxim(out, 9);
}

void printHex(const uint8_t *buf, size_t n) {
  for (size_t i = 0; i < n; i++) {
    if (buf[i] < 0x10) Serial.print('0');
    Serial.print(buf[i], HEX);
    if (i + 1 < n) Serial.print(' ');
  }
}

bool readReply10(uint8_t in[10], uint32_t timeoutMs = 200) {
  uint32_t start = millis();
  size_t got = 0;

  while ((millis() - start) < timeoutMs && got < 10) {
    if (Serial2.available()) {
      in[got++] = (uint8_t)Serial2.read();
    }
  }
  return (got == 10);
}

bool crcOk10(const uint8_t in[10]) {
  return crc8_maxim(in, 9) == in[9];
}

// ---------- High-level commands from wiki ----------

// 1) Switch to velocity loop (mode=0x02). Wiki says: no feedback.  [oai_citation:4‡Waveshare](https://www.waveshare.com/wiki/DDSM210?srsltid=AfmBOoq7D7Z__qriczWc_kmWMWZZn5Kkax7JqxRCivi_0GvfEgxB4lKj)
void setVelocityLoop(uint8_t id) {
  uint8_t frame[10];
  // In wiki example: 01 A0 02 00 00 00 00 00 00 E4
  // That means cmd=0xA0 and bytes[2]=0x02, [3]=0x00.
  // We can represent that as value_s16 = 0x0200.
  buildFrame(frame, id, 0xA0, (int16_t)0x0200, 0x00, 0x00, 0x00, 0x00, 0x00);

  Serial.print(">> setVelocityLoop: ");
  printHex(frame, 10);
  Serial.println();

  Serial2.write(frame, 10);
  Serial2.flush();
}

// 2) Query motor mode (Wiki: 01 75 ... 47)  [oai_citation:5‡Waveshare](https://www.waveshare.com/wiki/DDSM210?srsltid=AfmBOoq7D7Z__qriczWc_kmWMWZZn5Kkax7JqxRCivi_0GvfEgxB4lKj)
void queryMode(uint8_t id) {
  uint8_t frame[10];
  // Use cmd=0x75 and all zeros
  buildFrame(frame, id, 0x75, 0, 0, 0, 0, 0, 0);

  Serial.print(">> queryMode: ");
  printHex(frame, 10);
  Serial.println();

  Serial2.write(frame, 10);
  Serial2.flush();

  uint8_t rep[10];
  if (readReply10(rep)) {
    Serial.print("<< reply: ");
    printHex(rep, 10);
    Serial.print(" | CRC ");
    Serial.println(crcOk10(rep) ? "OK" : "BAD");
  } else {
    Serial.println("<< no reply");
  }
}

// 3) Set speed in velocity loop: value is 0.1 rpm units, signed 16-bit  [oai_citation:6‡Waveshare](https://www.waveshare.com/wiki/DDSM210?srsltid=AfmBOoq7D7Z__qriczWc_kmWMWZZn5Kkax7JqxRCivi_0GvfEgxB4lKj)
void setSpeedRpm(uint8_t id, float rpm, uint8_t accTime = 0) {
  int16_t v = (int16_t)lroundf(rpm * 10.0f); // 0.1 rpm units
  uint8_t frame[10];

  // Protocol 1: cmd=0x64, feedback selectors in [4],[5]. Feedback content: 0x01 speed, 0x02 current, 0x03 position  [oai_citation:7‡Waveshare](https://www.waveshare.com/wiki/DDSM210?srsltid=AfmBOoq7D7Z__qriczWc_kmWMWZZn5Kkax7JqxRCivi_0GvfEgxB4lKj)
  // Wir fordern speed feedback (fb1=0x01), fb2=0x00.
  buildFrame(frame, id, 0x64, v, 0x01, 0x00, accTime, 0x00, 0x00);

  Serial.print(">> setSpeed ");
  Serial.print(rpm, 1);
  Serial.print(" rpm: ");
  printHex(frame, 10);
  Serial.println();

  // Clear any stale bytes
  while (Serial2.available()) Serial2.read();

  Serial2.write(frame, 10);
  Serial2.flush();

  uint8_t rep[10];
  if (readReply10(rep)) {
    Serial.print("<< reply: ");
    printHex(rep, 10);
    Serial.print(" | CRC ");
    Serial.println(crcOk10(rep) ? "OK" : "BAD");

    if (crcOk10(rep) && rep[0] == id && rep[1] == 0x64) {
      int16_t fb1 = (int16_t)((rep[2] << 8) | rep[3]); // will be speed if fb1 requested
      int16_t fb2 = (int16_t)((rep[4] << 8) | rep[5]);
      uint8_t temp = rep[7];
      uint8_t err  = rep[8];

      Serial.print("   feedback1(s16)=");
      Serial.print(fb1);
      Serial.print(" => ");
      Serial.print(fb1 / 10.0f, 1);
      Serial.print(" rpm (if fb1=speed). Temp=");
      Serial.print(temp);
      Serial.print("C, Err=0x");
      if (err < 0x10) Serial.print('0');
      Serial.println(err, HEX);
    }
  } else {
    Serial.println("<< no reply (check RX/TX, GND, mode, ID, or level).");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("DDSM210 ESP32 UART demo (Protocol frames, CRC-8/MAXIM)");
  Serial2.begin(BAUD, SERIAL_8N1, DDSM_RX, DDSM_TX);

  delay(300);

  // According to wiki steps: set ID (optional), set mode, then send value  [oai_citation:8‡Waveshare](https://www.waveshare.com/wiki/DDSM210?srsltid=AfmBOoq7D7Z__qriczWc_kmWMWZZn5Kkax7JqxRCivi_0GvfEgxB4lKj)
  setVelocityLoop(MOTOR_ID);   // no feedback
  delay(100);

  queryMode(MOTOR_ID);         // optional: see if we get something
  delay(200);

  setSpeedRpm(MOTOR_ID, 10.0f); // 10 rpm
}

void loop() {
  // Example: toggle between +10 rpm and -10 rpm every 3 seconds
  static uint32_t t0 = 0;
  static bool dir = false;

  if (millis() - t0 > 3000) {
    t0 = millis();
    dir = !dir;
    setSpeedRpm(MOTOR_ID, dir ? 10.0f : -10.0f);
  }
}
