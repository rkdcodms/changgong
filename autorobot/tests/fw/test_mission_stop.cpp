// Hardware-free harness for arduino/mission_stop/mission_stop.ino (MS1).
// Compares MS1 against the deployed-baseline vision_stop.ino where the
// behaviour must be identical, and checks the new hold/GO/P semantics.
#include "Arduino.h"
#include <sstream>
#include <tuple>
#include "sources.h"
namespace Vision {
#include VISION_SKETCH
}
namespace Mission {
#include MISSION_SKETCH
}
#undef MS1_GAP_COAST_MS
#undef MS1_AUTO_RESUME_MS
#define MS1_GAP_COAST_MS 200
namespace Gap {
#include MISSION_SKETCH
}
#undef MS1_GAP_COAST_MS
#undef MS1_AUTO_RESUME_MS
#define MS1_AUTO_RESUME_MS 5000
namespace Compat {
#include MISSION_SKETCH
}
#undef MS1_AUTO_RESUME_MS

Device board, other;
int checks = 0;
const char* scenario = "initial";
void expect(bool ok, const char* msg) {
  ++checks;
  if (!ok) { fprintf(stderr, "FAIL [%s t=%u]: %s\n", scenario, board.now, msg); exit(1); }
}
void mask(Device& d, int bits) { for (int i = 0; i < 8; ++i) d.adc[i] = (bits & (1 << i)) ? 50 : 788; }

#define RESET_NS(ns) do { \
  memset(ns::white,0,sizeof(ns::white)); memset(ns::sensorValues,0,sizeof(ns::sensorValues)); \
  ns::whiteCount=ns::allWhiteStreak=0; ns::startupChecked=false; ns::error=0; \
  ns::previousError=ns::filteredDerivative=0; ns::pdReady=false; ns::lastDirection=0; \
  ns::leftCommand=ns::rightCommand=0; ns::stateSince=ns::lastControl=ns::stableSince=0; \
  ns::stableActive=false; ns::lastSensorPrint=0; \
  ns::stopSequence=ns::lastStatePrint=0; ns::lastReportedMode=-1; ns::mode=ns::WAIT_VISION; \
  ns::visionSession=ns::lastHeartbeat=ns::lastAcceptedEvent=0; ns::lastVisionClass=-1; ns::stopReason=0; \
  ns::visionRxLength=0; ns::visionRxDropping=false; ns::visionRxAt=0; \
  ns::lastVisionPrint=ns::reportedSession=ns::reportedEvent=0; \
  ns::reportedClass=-2; ns::reportedReason=ns::reportedLive=-1; } while (0)
#define RESET_MS(ns) do { RESET_NS(ns); ns::lastIdPrint=0; ns::gapSince=0; ns::gapActive=false; } while (0)
#define CONTROL_STATE(ns) std::make_tuple(int(ns::mode),ns::whiteCount,ns::allWhiteStreak,ns::startupChecked, \
  ns::error,ns::previousError,ns::filteredDerivative,ns::pdReady,ns::lastDirection, \
  ns::leftCommand,ns::rightCommand,ns::stateSince,ns::lastControl,ns::stableSince,ns::stableActive)

// The harness drives one namespace at a time through these function pointers.
struct Target { void (*loop)(); uint32_t (*session)(); };
#define TARGET(ns) Target{ [] { ns::loop(); }, [] { return ns::visionSession; } }
Target cur;

void reset(const char* name, Target t, void (*init)(), int bits = 0x18, uint32_t at = 0) {
  scenario = name; board = Device(); board.now = at; mask(board, bits); activeDevice = &board;
  cur = t; init();
}
void feed(const std::string& line) {
  activeDevice = &board; board.rx += line;
  while (Serial.available()) { board.room = 63; cur.loop(); }
}
void tick(int bits = 0x18, uint32_t dt = 10, bool heartbeat = true, int room = 63) {
  board.now += dt; mask(board, bits); activeDevice = &board; board.room = room;
  if (heartbeat && cur.session()) board.rx += "@VC1,H," + std::to_string(cur.session()) + "\n";
  cur.loop();
}
void ticks(int n, int bits = 0x18) { for (int i = 0; i < n; ++i) tick(bits); }
bool braked() {
  return board.pwm[5] == 255 && board.pwm[6] == 255 && board.level[30] == 0 && board.level[31] == 0 &&
         board.level[32] == 0 && board.level[33] == 0;
}
bool moving() { return board.pwm[5] != 255 || board.pwm[6] != 255; }

void initMission() { RESET_MS(Mission); Mission::setup(); }
void initGap() { RESET_MS(Gap); Gap::setup(); }
void initCompat() { RESET_MS(Compat); Compat::setup(); }

int main() {
  // ---------------- boot / arming ----------------
  reset("boot holds without heartbeat", TARGET(Mission), initMission);
  for (int i = 0; i < 200; ++i) tick(0x18, 10, false);
  expect(Mission::mode == Mission::WAIT_VISION && braked(), "no heartbeat never moves");
  expect(board.tx.find("@ID1,MS1,") != std::string::npos, "firmware identifies itself as MS1");

  reset("heartbeat binds but does not start", TARGET(Mission), initMission);
  feed("@VC1,H,42\n");
  expect(Mission::visionSession == 42 && Mission::mode == Mission::WAIT_VISION, "H binds session only");
  for (int i = 0; i < 300; ++i) tick();
  expect(Mission::mode == Mission::WAIT_VISION && braked(), "healthy link alone never moves the robot");
  feed("@VC1,S,42,1,3\n");
  expect(Mission::mode == Mission::WAIT_VISION && Mission::lastAcceptedEvent == 0, "S before GO ignored");
  feed("@VC1,G,99,1\n");
  expect(Mission::mode == Mission::WAIT_VISION, "foreign-session GO ignored");
  feed("@VC1,G,42,0\n"); feed("@VC1,G,42,x\n"); feed("@VC1,G,42\n"); feed("@VC1,G,42,1,2\n");
  expect(Mission::mode == Mission::WAIT_VISION, "malformed GO ignored");
  feed("@VC1,G,42,1\n");
  expect(Mission::mode == Mission::FOLLOW && Mission::lastAcceptedEvent == 1, "GO starts the robot");
  ticks(30); expect(moving(), "robot drives after GO");

  // ---------------- hold until GO ----------------
  reset("vision stop holds", TARGET(Mission), initMission);
  feed("@VC1,H,42\n@VC1,G,42,1\n"); ticks(5);
  feed("@VC1,S,42,2,4\n");
  expect(Mission::mode == Mission::STOPPED && Mission::stopReason == 2 && braked(), "S brakes");
  const uint32_t since = Mission::stateSince;
  for (int i = 0; i < 2000; ++i) { if (i % 50 == 0) feed("@VC1,S,42,2,4\n"); tick(); }
  expect(Mission::mode == Mission::STOPPED && Mission::stateSince == since && braked(),
         "20 s with heartbeat and duplicate S: no auto restart, no extension");
  expect(board.tx.find("@V1,42,2,4,2,1\n") != std::string::npos, "stop is acknowledged on V1");
  feed("@VC1,G,42,2\n");
  expect(Mission::mode == Mission::STOPPED, "GO reusing the stop's event id is ignored");
  feed("@VC1,G,42,3\n");
  expect(Mission::mode == Mission::FOLLOW && Mission::stopReason == 0, "fresh GO resumes on normal line");
  feed("@VC1,G,42,3\n"); feed("@VC1,S,42,2,4\n");
  expect(Mission::mode == Mission::FOLLOW, "replayed GO/S after resume do nothing");
  feed("@VC1,S,42,4,5\n");
  expect(Mission::mode == Mission::STOPPED && Mission::lastVisionClass == 5, "next target stop accepted");
  feed("@VC1,G,42,3\n");
  expect(Mission::mode == Mission::STOPPED, "delayed old GO cannot release the next stop");

  reset("GO before minimum stop time", TARGET(Mission), initMission);
  feed("@VC1,H,42\n@VC1,G,42,1\n"); ticks(5); feed("@VC1,S,42,2,1\n");
  tick(); feed("@VC1,G,42,3\n");
  expect(Mission::mode == Mission::STOPPED && Mission::lastAcceptedEvent == 2, "GO within 300 ms ignored");
  for (int i = 0; i < 30; ++i) tick();
  feed("@VC1,G,42,3\n");
  expect(Mission::mode == Mission::FOLLOW, "retransmitted GO after 300 ms accepted");

  reset("resume uses sensors", TARGET(Mission), initMission);
  feed("@VC1,H,42\n@VC1,G,42,1\n"); tick(0x03); feed("@VC1,S,42,2,1\n");
  ticks(40, 0); feed("@VC1,G,42,3\n");
  expect(Mission::mode == Mission::SEARCH && Mission::lastDirection == 1, "lost line after stop resumes SEARCH");
  reset("resume on bar", TARGET(Mission), initMission);
  feed("@VC1,H,42\n@VC1,G,42,1\n"); tick(); feed("@VC1,S,42,2,1\n");
  ticks(40, 255); feed("@VC1,G,42,3\n");
  expect(Mission::mode == Mission::EXIT_BAR, "all-white after stop resumes through EXIT_BAR");

  // ---------------- host hold (P) ----------------
  reset("host hold", TARGET(Mission), initMission);
  feed("@VC1,H,42\n@VC1,G,42,1\n"); ticks(5);
  feed("@VC1,P,42,2\n");
  expect(Mission::mode == Mission::STOPPED && Mission::stopReason == 4 && braked(), "P brakes with reason 4");
  feed("@VC1,P,42,3\n"); feed("@VC1,S,42,4,2\n");
  expect(Mission::stopReason == 4 && Mission::lastAcceptedEvent == 2, "P/S while stopped ignored");
  ticks(40); feed("@VC1,G,42,5\n");
  expect(Mission::mode == Mission::FOLLOW, "GO releases host hold");
  reset("P in WAIT ignored", TARGET(Mission), initMission);
  feed("@VC1,H,42\n@VC1,P,42,1\n");
  expect(Mission::mode == Mission::WAIT_VISION && Mission::lastAcceptedEvent == 0, "P cannot consume ids in WAIT");

  // ---------------- link loss / FAULT ----------------
  reset("link loss while holding", TARGET(Mission), initMission);
  feed("@VC1,H,42\n@VC1,G,42,1\n"); ticks(3); feed("@VC1,S,42,2,1\n");
  tick(0x18, 1500, false);
  expect(Mission::mode == Mission::FAULT && Mission::stopReason == 3 && braked(), "heartbeat loss latches FAULT");
  feed("@VC1,H,42\n@VC1,G,42,9\n"); ticks(100);
  expect(Mission::mode == Mission::FAULT && braked(), "GO and heartbeats never revive FAULT");
  reset("link loss while waiting", TARGET(Mission), initMission);
  feed("@VC1,H,42\n"); tick(0x18, 1500, false);
  expect(Mission::mode == Mission::FAULT, "link loss before GO also latches FAULT");
  reset("line search failure", TARGET(Mission), initMission);
  feed("@VC1,H,42\n@VC1,G,42,1\n"); tick(); for (int i = 0; i < 600; ++i) tick(0);
  expect(Mission::mode == Mission::FAULT && braked(), "unknown-direction line loss still FAULTs (original)");

  reset("millis wrap", TARGET(Mission), initMission, 0x18, UINT32_MAX - 300);
  feed("@VC1,H,42\n@VC1,G,42,1\n"); tick(); feed("@VC1,S,42,2,1\n"); ticks(60);
  feed("@VC1,G,42,3\n");
  expect(Mission::mode == Mission::FOLLOW, "MIN_STOP_MS works across millis wrap");

  // ---------------- telemetry ----------------
  reset("telemetry fits", TARGET(Mission), initMission);
  feed("@VC1,H,4294967295\n@VC1,G,4294967295,4294967294\n");
  for (int i = 0; i < 300; ++i) tick(0x18, 10, true, (i % 3 == 0) ? 0 : 63);
  std::istringstream rows(board.tx); std::string row; int ids = 0;
  while (std::getline(rows, row)) {
    if (row.rfind("@", 0) == 0) expect(row.size() + 1 <= 63, "every protocol row fits the TX buffer");
    if (row.rfind("@ID1,MS1,", 0) == 0) ++ids;
  }
  expect(ids >= 2, "ID line repeats under backpressure");
  expect(board.blockingPrints == 0, "telemetry never blocks");

  // ---------------- equivalence with deployed vision_stop.ino ----------------
  {
    scenario = "normal control equivalence";
    board = Device(); other = Device(); mask(board, 0x18); mask(other, 0x18);
    activeDevice = &other; RESET_NS(Vision); Vision::setup();
    activeDevice = &board; RESET_MS(Mission); Mission::setup(); cur = TARGET(Mission);
    activeDevice = &other; other.rx = "@VC1,H,42\n"; Vision::loop();
    activeDevice = &board; board.rx = "@VC1,H,42\n@VC1,G,42,1\n"; Mission::loop(); Mission::loop();
    expect(Vision::mode == Vision::FOLLOW && Mission::mode == Mission::FOLLOW, "both following");
    for (int i = 0; i < 1500; ++i) {
      int bits = (i % 200 < 50) ? 0x18 : ((i % 200 < 100) ? 0x03 : ((i % 200 < 115) ? 0 : 0x30));
      other.now += 10; mask(other, bits); other.room = 63; other.rx += "@VC1,H,42\n";
      activeDevice = &other; Vision::loop();
      tick(bits);
      expect(CONTROL_STATE(Vision) == CONTROL_STATE(Mission), "control state equals vision_stop.ino");
      expect(memcmp(other.pwm, board.pwm, sizeof(board.pwm)) == 0, "motor PWM equals vision_stop.ino");
      expect(memcmp(other.level, board.level, sizeof(board.level)) == 0, "motor direction equals vision_stop.ino");
    }
  }

  // ---------------- optional dark-gap coasting ----------------
  reset("gap coast bridges short centred gap", TARGET(Gap), initGap);
  feed("@VC1,H,42\n@VC1,G,42,1\n"); ticks(50);
  for (int i = 0; i < 15; ++i) tick(0);
  expect(Gap::mode == Gap::FOLLOW && moving(), "150 ms centred dark gap keeps FOLLOW");
  ticks(10);
  expect(Gap::mode == Gap::FOLLOW, "line regained after gap");
  for (int i = 0; i < 25; ++i) tick(0);
  expect(Gap::mode == Gap::SEARCH || Gap::mode == Gap::FAULT, "gap longer than 200 ms falls back to original SEARCH");
  reset("gap coast not used off-centre", TARGET(Gap), initGap);
  feed("@VC1,H,42\n@VC1,G,42,1\n"); ticks(20, 0x01); tick(0);
  expect(Gap::mode == Gap::SEARCH, "line lost at the edge searches immediately");

  // ---------------- 5-second compatibility build ----------------
  reset("compat 5 s build", TARGET(Compat), initCompat);
  feed("@VC1,H,42\n@VC1,G,42,1\n"); tick(); feed("@VC1,S,42,2,1\n");
  const uint32_t at = Compat::stateSince;
  while (Compat::mode == Compat::STOPPED && board.now - at < 6000) tick();
  expect(board.now - at == 5000 && Compat::mode == Compat::FOLLOW, "AUTO_RESUME_MS=5000 reproduces 5 s test");

  printf("PASS: %d assertions; MS1 hold/GO/P, link FAULT, telemetry, vision_stop equivalence, gap option\n", checks);
}
