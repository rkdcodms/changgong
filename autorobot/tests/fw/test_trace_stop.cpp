// Hardware-free harness for arduino/trace_stop/trace_stop.ino (the Mega firmware).
// It records the CURRENT behaviour; it does not change the firmware.
#include "Arduino.h"
#include <sstream>
#include "sources.h"
namespace T {
#include TRACE_STOP_SKETCH
}

Device board;
int checks = 0;
const char* scenario = "";
void expect(bool ok, const char* msg) {
  ++checks;
  if (!ok) { fprintf(stderr, "FAIL [%s t=%u]: %s\n", scenario, board.now, msg); exit(1); }
}
void mask(int bits) { for (int i = 0; i < 8; ++i) board.adc[i] = (bits & (1 << i)) ? 50 : 788; }
void reset(const char* name) {
  scenario = name; board = Device(); activeDevice = &board; mask(0x18);
  memset(T::white, 0, sizeof T::white); memset(T::sensorValues, 0, sizeof T::sensorValues);
  T::whiteCount = 0; T::error = 0; T::previousError = T::filteredDerivative = 0; T::pdReady = false;
  T::lastDirection = 0; T::leftCommand = T::rightCommand = 0;
  T::stateSince = T::lastControl = T::stableSince = 0; T::stableActive = false; T::lastSensorPrint = 0;
  T::mode = T::WAIT_VISION; T::stopSequence = T::lastStatePrint = 0; T::lastReportedMode = -1;
  T::visionSession = T::lastHeartbeat = T::lastAcceptedEvent = 0; T::lastVisionClass = -1; T::stopReason = 0;
  T::visionRxLength = 0; T::visionRxDropping = false; T::visionRxAt = 0;
  T::lastVisionPrint = T::reportedSession = T::reportedEvent = 0;
  T::reportedClass = -2; T::reportedReason = T::reportedLive = -1;
  T::orderedSession = T::releaseRequested = T::approachActive = false;
  T::searchSegmentAt = 0; T::priorSearchBaseMs = 0;
  T::setup();
}
void feed(const std::string& s) { board.rx += s; while (Serial.available()) { board.room = 63; T::loop(); } }
void tick(int bits = 0x18, int n = 1, bool hb = true) {
  for (int i = 0; i < n; ++i) {
    board.now += 10; mask(bits); board.room = 63;
    if (hb && T::visionSession)
      board.rx += std::string(T::orderedSession ? "@VC1,Q," : "@VC1,H,") + std::to_string(T::visionSession) + "\n";
    T::loop();
  }
}
bool braked() { return board.pwm[5] == 255 && board.pwm[6] == 255 && T::leftCommand == 0 && T::rightCommand == 0; }

int main() {
  reset("boot");
  tick(0x18, 200, false);
  expect(T::mode == T::WAIT_VISION && braked(), "no heartbeat: braked in WAIT_VISION");

  reset("robocon Q session starts at first heartbeat");
  feed("@VC1,Q,42\n"); tick(0x18, 30);
  expect(T::orderedSession && T::mode == T::FOLLOW && !braked(),
         "Q heartbeat alone starts driving (there is no separate start command)");

  reset("robocon hold until R");
  feed("@VC1,Q,42\n"); tick(0x18, 5); feed("@VC1,S,42,1,3\n");
  expect(T::mode == T::STOPPED && braked(), "S brakes");
  for (int i = 0; i < 2000; ++i) { if (i % 100 == 0) feed("@VC1,S,42,1,3\n"); tick(); }
  expect(T::mode == T::STOPPED && braked(), "20 s with heartbeats and duplicate S: still stopped");
  feed("@VC1,R,42,0\n@VC1,R,42,2\n"); tick();
  expect(T::mode == T::STOPPED, "R naming another event is ignored");
  feed("@VC1,R,42,1\n"); tick();
  expect(T::mode == T::FOLLOW, "R for this stop releases");
  feed("@VC1,S,42,2,4\n"); feed("@VC1,R,42,1\n"); tick(0x18, 20);
  expect(T::mode == T::STOPPED, "late R of the previous stop cannot release the next stop");
  feed("@VC1,S,42,1,3\n"); expect(T::lastAcceptedEvent == 2, "replayed old S ignored");

  reset("approach speed");
  feed("@VC1,Q,42\n"); tick(0x18, 100); const float cruise = T::leftCommand;
  feed("@VC1,A,42,1\n"); tick(0x18, 100); const float approach = T::leftCommand;
  feed("@VC1,A,42,0\n"); tick(0x18, 100);
  expect(cruise > approach * 1.4f && approach > 0, "A,1 slows to base speed (1/1.5 of cruise)");
  expect(T::leftCommand > approach * 1.4f, "A,0 restores cruise");
  feed("@VC1,A,42,7\n@VC1,A,99,1\n"); tick();
  expect(!T::approachActive, "invalid or foreign A ignored");

  reset("timed H session keeps 5 s behaviour");
  feed("@VC1,H,42\n"); tick(0x18, 3); feed("@VC1,S,42,1,3\n");
  tick(0x18, 499); expect(T::mode == T::STOPPED, "H session: stopped before 5 s");
  tick(0x18, 2); expect(T::mode == T::FOLLOW, "H session: resumes after 5 s");
  feed("@VC1,A,42,1\n"); tick(); expect(!T::approachActive, "A ignored in H session");

  reset("session type cannot change");
  feed("@VC1,Q,42\n"); const uint32_t hb = T::lastHeartbeat; tick(0x18, 1, false);
  feed("@VC1,H,42\n"); expect(T::lastHeartbeat == hb, "H cannot refresh a Q session");

  reset("link loss while held");
  feed("@VC1,Q,42\n"); tick(0x18, 3); feed("@VC1,S,42,1,3\n");
  tick(0x18, 150, false);
  expect(T::mode == T::FAULT && braked(), "1.5 s without heartbeat -> FAULT, braked");
  feed("@VC1,Q,42\n@VC1,R,42,1\n"); tick(0x18, 100);
  expect(T::mode == T::FAULT && braked(), "FAULT is latched; R and heartbeats do not restart");

  reset("link loss while driving");
  feed("@VC1,Q,42\n"); tick(0x18, 30); tick(0x18, 150, false);
  expect(T::mode == T::FAULT && braked(), "link loss while driving -> FAULT");

  // KNOWN BEHAVIOUR (field check): all eight sensors white => straight at
  // STRAIGHT_PWM with no time limit. The old EXIT_BAR faulted after ~3.3 s.
  reset("all white has no timeout");
  feed("@VC1,Q,42\n"); tick(0x18, 20); tick(0xFF, 1000);
  expect(T::mode == T::FOLLOW && T::leftCommand > 0 && T::leftCommand == T::rightCommand,
         "10 s all-white: still driving straight (no FAULT)");

  printf("PASS: %d assertions; trace_stop Q/R hold, A speed, H 5 s, link FAULT, all-white behaviour\n", checks);
}
