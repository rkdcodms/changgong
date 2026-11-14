// J12 (Serial2) loopback test for OpenRB-150.
// Checks whether the board's own Bluetooth UART pins still work, e.g. after a
// BT-210 was powered from 5 V and may have pushed 5 V into the J12 Rx pin.
//
// 1. Power OFF (battery AND USB unplugged). Unplug the BT-210 from J12.
// 2. Put ONE jumper wire between J12 "Tx" and J12 "Rx". Nothing on V or G.
// 3. Plug in USB, upload this sketch, open the Serial Monitor (any baud).
// 4. Every second it prints how many bytes came back correctly at 57600 bps
//    (and once at 115200 bps). "PASS" = the pins work. Then remove the jumper.
// Writes never wait: the sketch sends only what fits in the TX buffer.

#include <Arduino.h>

#if !defined(ARDUINO_OpenRB)
  #error "Select the OpenRB-150 board for this sketch."
#endif

constexpr uint32_t TEST_BAUDS[] = {57600, 115200};
constexpr uint32_t TEST_MS_PER_BAUD = 5000;

uint8_t nextSend = 0, nextExpect = 0;
uint32_t sent = 0, good = 0, wrong = 0, lastPrintMs = 0, phaseStartMs = 0;
uint8_t phase = 0;
bool firstWrongShown = false;

void startPhase(uint8_t p) {
  phase = p;
  Serial2.end();
  Serial2.begin(TEST_BAUDS[p]);
  delay(5);
  while (Serial2.available()) Serial2.read();
  nextSend = nextExpect = 0;
  sent = good = wrong = 0;
  firstWrongShown = false;
  phaseStartMs = lastPrintMs = millis();
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  startPhase(0);
}

void report(bool final) {
  if (!Serial.dtr()) return;
  Serial.print(final ? "[RESULT] " : "  ");
  Serial.print(TEST_BAUDS[phase]); Serial.print(" bps  sent="); Serial.print(sent);
  Serial.print(" back_ok="); Serial.print(good);
  Serial.print(" wrong="); Serial.print(wrong);
  Serial.print(" not_back_yet="); Serial.print(sent >= good + wrong ? sent - good - wrong : 0);
  if (final) {
    // up to ~256 bytes can still be in the TX ring / on the wire when the phase ends
    bool pass = sent > 1000 && wrong == 0 && good + 300 >= sent;
    Serial.println(pass ? "  -> PASS (J12 Tx/Rx work)" :
                   (good == 0 ? "  -> FAIL: nothing comes back (jumper? Rx pin damaged?)" :
                                "  -> FAIL: bytes come back wrong or missing (Rx/Tx pin damaged?)"));
    digitalWrite(LED_BUILTIN, pass ? HIGH : LOW);
  } else Serial.println();
}

void loop() {
  // send what fits, never block
  int room = Serial2.availableForWrite();
  while (room-- > 1) { Serial2.write(nextSend++); ++sent; }
  // check what came back
  while (Serial2.available()) {
    uint8_t b = (uint8_t)Serial2.read();
    if (b == nextExpect) { ++good; ++nextExpect; }
    else {
      ++wrong;
      if (!firstWrongShown && Serial.dtr()) {
        firstWrongShown = true;
        Serial.print("  first wrong byte: expected "); Serial.print(nextExpect);
        Serial.print(" got "); Serial.println(b);
      }
      nextExpect = b + 1;              // resync on the received byte
    }
  }
  uint32_t now = millis();
  if (now - lastPrintMs >= 1000) { lastPrintMs = now; report(false); }
  if (now - phaseStartMs >= TEST_MS_PER_BAUD) {
    report(true);
    startPhase((phase + 1) % (sizeof(TEST_BAUDS) / sizeof(TEST_BAUDS[0])));
  }
}
