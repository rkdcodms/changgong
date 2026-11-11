// OpenRB-150: PPM wheels + BT-210 follower arm + dual XL430 joint 2
// Leader protocol is UNCHANGED: p1,p2,p3,p4,gripper_delta\n (5 values).
// Physical arm IDs: 1,2,3,4,5,6. Joint 2 uses IDs 2 and 6, 1:1 coupled.
// ID2 Drive Mode=0, ID6 Drive Mode=1 (hardware Reverse). Goal RAWs move together.
// Wheels: 9/10. Gripper: XC330 ID 5, current-based position mode.
//
// Default: RUN mode using the user's ID2/ID6 limits, 540..3329 each.
// Boot: discover/configure motors -> move arm to 2048 -> calibrate gripper
// -> enable Leader following and wheel control. No fixed 300 ms boot wait.
// CALIBRATION_MODE=true is available for torque-OFF manual measurement.
// Support the arm mechanically before disabling torque.
//
// ID5 is optional: its setup/runtime failure disables ONLY the gripper.
// Required arm or pair safety faults latch the whole robot OFF. No auto-reboot.
// Clear the cause, support the arm, and power-cycle. A broken bus can prevent
// torque-off delivery; software is not a substitute for a physical power stop.

#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <stdint.h>
#include <string.h>

#define DXL_SERIAL Serial1
#define BLT_SERIAL Serial2
#define DEBUG_SERIAL Serial
#define DEBUG 1
#define FIRMWARE_LABEL "FOLLOWER_TRACK_20261112"

Dynamixel2Arduino dxl(DXL_SERIAL, -1);
using namespace ControlTableItem;

// ======================== USER SETTINGS =============================
constexpr bool CALIBRATION_MODE = false;
constexpr bool PAIR_LIMITS_CONFIRMED = true;
constexpr bool ARM_HOME_AT_STARTUP = true; // active in confirmed RUN mode
constexpr uint32_t DXL_READY_TIMEOUT_MS = 1000; // maximum, not a fixed boot delay

constexpr uint8_t LEADER_JOINT_COUNT = 5;
constexpr uint8_t ARM_MOTOR_COUNT = 6;
constexpr uint8_t ARM_IDS[ARM_MOTOR_COUNT] = {1, 2, 3, 4, 5, 6};
constexpr uint8_t PAIR_INDEX_2 = 1;
constexpr uint8_t PAIR_INDEX_6 = 5;
constexpr uint8_t GRIP_INDEX = 4;
constexpr uint8_t GRIPPER_IDS[] = {5};
constexpr uint8_t GRIPPER_NUM = sizeof(GRIPPER_IDS) / sizeof(GRIPPER_IDS[0]);
constexpr uint16_t XL430_MODEL_NUMBER = 1060; // XL430-W250

// Software zero correction. These are the actual readings at the SAME pose.
// The user confirmed both motors are aligned at 2048 in their reported RAW
// coordinates. Preserve existing EEPROM Homing Offset; do not add it twice.
constexpr int32_t LEADER_J2_ZERO = 2048;
constexpr int32_t LEADER_TO_FOLLOWER_J2_SIGN = -1; // Mechanical orientation of the entire follower joint
constexpr int32_t PAIR_ZERO_2 = 2048;
constexpr int32_t PAIR_ZERO_6 = 2048;

// Limits supplied by the user. ID6 is already reversed in its Drive Mode.
// With both zeros at 2048, q=-1508..1281 and each commanded motor=540..3329.
constexpr int32_t PAIR_2_MIN = 540;
constexpr int32_t PAIR_2_MAX = 3329;
constexpr int32_t PAIR_6_MIN = 540;
constexpr int32_t PAIR_6_MAX = 3329;

// Original limits for the OTHER joints are preserved.
//                         ID1  ID2          ID3   ID4   ID5  ID6
constexpr int32_t JOINT_MIN[ARM_MOTOR_COUNT] =
    {0, PAIR_2_MIN, 1325, 1700, 0, PAIR_6_MIN};
constexpr int32_t JOINT_MAX[ARM_MOTOR_COUNT] =
    {4095, PAIR_2_MAX, 3405, 3010, 4095, PAIR_6_MAX};
constexpr uint16_t ARM_PROFILE_VELOCITY = 20;
constexpr uint16_t ARM_PROFILE_ACCELERATION = 10;
constexpr uint32_t ARM_CONTROL_MS = 20;
constexpr uint32_t ARM_LINK_TIMEOUT_MS = 500;
constexpr int32_t PAIR_STEP_RAW = 6; // initial move: 6 pulses per 20 ms
constexpr int32_t FOLLOW_PROFILE_VELOCITY = 240;
constexpr int32_t FOLLOW_PROFILE_ACCELERATION = 60;
constexpr int32_t PAIR_FOLLOW_STEP_RAW = 60; // live tracking: 12 pulses per 20 ms

// Restore the original Follower initial pose before accepting Leader targets.
// ID1/3/4 -> 2048; ID2/6 -> their calibrated zeros (both currently 2048).
// ID5 retains contact-based closed-position homing, not an absolute 2048 goal.
constexpr int32_t ARM_INITIAL_POSITION_RAW = 2048;
constexpr uint32_t ARM_HOME_MIN_MS = 3000;
constexpr uint32_t ARM_HOME_TIMEOUT_MS = 15000;
constexpr int32_t ARM_HOME_TOLERANCE_RAW = 25;
constexpr uint8_t ARM_HOME_STABLE_SAMPLES = 3;
constexpr int32_t ARM_HOME_TRACK_ABORT_RAW = 240;
constexpr uint32_t ARM_HOME_TRACK_ABORT_MS = 600;

// XL430 has NO current sensor/current control. Use PWM and inferred load.
// These are commissioning starting values, not verified load ratings.
constexpr int32_t PAIR_PWM_CAP_RAW = 440; // 440 * 0.113% = ~49.7%
constexpr int32_t PAIR_LOAD_TRIP_RAW = 450; // |Present Load|, 0.1% units
constexpr uint32_t PAIR_LOAD_TRIP_MS = 200;
constexpr int32_t PAIR_SYNC_TRIP_RAW = 60; // zero-relative position disagreement
constexpr int32_t PAIR_SYNC_HARD_RAW = 180;
constexpr uint32_t PAIR_SYNC_TRIP_MS = 120;
constexpr int32_t PAIR_STARTUP_SYNC_RAW = 20;
constexpr int32_t PAIR_TRACK_TRIP_RAW = 180;
constexpr uint32_t PAIR_TRACK_TRIP_MS = 1000;
constexpr uint8_t PAIR_TEMP_TRIP_C = 65;
constexpr uint8_t PAIR_COMM_MISS_LIMIT = 3;
constexpr uint32_t PAIR_POLL_MS = 20;
constexpr uint32_t IO_TIMEOUT_MS = 5;
constexpr uint32_t CONFIG_TIMEOUT_MS = 100;

constexpr uint16_t GRIPPER_GOAL_CURRENT = 700; // original XC330 RAW values
constexpr uint16_t GRIP_HOME_CURRENT = 500;
constexpr int32_t GRIP_SPAN = 1010;
constexpr int GRIP_CLOSE_DIR = +1;
constexpr int LEADER_GRIP_SIGN = +1;
constexpr int32_t GRIP_CLOSE_MARGIN = 0;
constexpr int32_t GRIP_OPEN_MARGIN = 30;

constexpr uint8_t PPM_PIN = 0;
constexpr uint8_t LEFT_ID = 9;
constexpr uint8_t RIGHT_ID = 10;
constexpr int LEFT_SIGN = 1;
constexpr int RIGHT_SIGN = -1;
constexpr int FORWARD_SIGN = 1;
constexpr int TURN_SIGN = 1;
constexpr int TURN_CENTER = 1500;
constexpr int FWD_CENTER = 1500;
constexpr int DEADZONE = 40;
constexpr int MAX_FWD_SPEED = 300;
constexpr int MAX_TURN_SPEED = 150;
constexpr uint32_t PPM_TIMEOUT_US = 150000;

static_assert(ARM_MOTOR_COUNT < LEFT_ID, "Arm and wheel IDs must be distinct");
static_assert(GRIPPER_NUM == 1, "This build has one gripper, ID 5");
static_assert(PAIR_STEP_RAW > 0, "Pair slew rate must be positive");
static_assert(ARM_HOME_TIMEOUT_MS > ARM_HOME_MIN_MS && ARM_HOME_STABLE_SAMPLES > 0,
              "Invalid initial-pose timing");
static_assert(GRIP_SPAN > GRIP_OPEN_MARGIN + GRIP_CLOSE_MARGIN,
              "Gripper margins must leave positive travel");

// =========================== STATE ==================================
bool armReady = false, wheelsReady = false, systemFault = false;
bool gripperAvailable = true, gripperReady = false;
const char *gripperDisabledReason = "none";
bool configurationIO = true;
const char *setupStage = "motor discovery";
uint8_t setupMotor = 0;
bool setupCheckFailed = false;
int32_t setupActual = 0, setupLower = 0, setupUpper = 0;
int32_t hardwareMin[ARM_MOTOR_COUNT] = {0, 0, 0, 0, 0, 0};
int32_t hardwareMax[ARM_MOTOR_COUNT] = {4095, 4095, 4095, 4095, 4095, 4095};
uint16_t gripFollowCurrent = GRIPPER_GOAL_CURRENT;
uint16_t gripHomingCurrent = GRIP_HOME_CURRENT;
uint32_t dxlPowerOnMs = 0;
const char *faultReason = "none";
uint8_t faultId = 0;
bool armed = false, pairSampleOK = false;
uint8_t neutralCount = 0, pairCommMisses = 0;
uint16_t dbgTurn = 0, dbgFwd = 0;
uint32_t armPacketCount = 0, armRejectCount = 0, armWriteCount = 0;
uint32_t lastArmPacketMs = 0;
bool armLinkActive = false;
bool armHomeActive = false;
int32_t qMin = 0, qMax = 0, qCommand = 0, qTarget = 0;
int32_t pairFollowStep = PAIR_STEP_RAW;
int32_t targetGoal[ARM_MOTOR_COUNT] = {};
int32_t gripClosedPos = 0;
int32_t leaderGripClosed = 0, leaderGripOpen = 0;
bool leaderGripSeen = false;
int32_t leaderGripHist[3] = {};
uint8_t leaderGripHistN = 0;
uint32_t loadSince = 0, syncSince = 0, trackSince = 0;
bool loadTiming = false, syncTiming = false, trackTiming = false;

struct PairSample {
  int32_t pos2, pos6;
  int16_t load2, load6;
  uint8_t temp2, temp6;
};
PairSample pairSample = {};

struct IoFailure {
  bool valid;
  const char *operation;
  uint8_t id, item, status;
  uint16_t lib;
  int32_t value;
};
IoFailure lastIoFailure = {}, faultIoFailure = {};

struct __attribute__((packed)) GoalData { int32_t position; };
GoalData swData[ARM_MOTOR_COUNT] = {};
DYNAMIXEL::InfoSyncWriteInst_t swInfo = {};
DYNAMIXEL::XELInfoSyncWrite_t swXels[ARM_MOTOR_COUNT] = {};

volatile uint16_t received[8] = {};
volatile uint32_t lastFrameUs = 0;
volatile bool frameReceived = false;
uint32_t previousEdgeUs = 0;
uint16_t working[8] = {};
uint8_t channelCount = 0;
bool frameValid = true;

// =========================== HELPERS ================================
int32_t min32(int32_t a, int32_t b) { return a < b ? a : b; }
int32_t max32(int32_t a, int32_t b) { return a > b ? a : b; }
int32_t abs32(int32_t v) { return v < 0 ? -v : v; }
int32_t clamp32(int32_t v, int32_t lo, int32_t hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

int32_t jointMinimum(uint8_t index) { return max32(JOINT_MIN[index], hardwareMin[index]); }
int32_t jointMaximum(uint8_t index) { return min32(JOINT_MAX[index], hardwareMax[index]); }
bool armMotorEnabled(uint8_t index) { return index != GRIP_INDEX || gripperAvailable; }

const char *itemName(uint8_t item) {
  switch (item) {
    case TORQUE_ENABLE: return "TORQUE_ENABLE";
    case MODEL_NUMBER: return "MODEL_NUMBER";
    case FIRMWARE_VERSION: return "FIRMWARE_VERSION";
    case STARTUP_CONFIGURATION: return "STARTUP_CONFIGURATION";
    case STATUS_RETURN_LEVEL: return "STATUS_RETURN_LEVEL";
    case BUS_WATCHDOG: return "BUS_WATCHDOG";
    case OPERATING_MODE: return "OPERATING_MODE";
    case DRIVE_MODE: return "DRIVE_MODE";
    case HOMING_OFFSET: return "HOMING_OFFSET";
    case SECONDARY_ID: return "SECONDARY_ID";
    case MIN_POSITION_LIMIT: return "MIN_POSITION_LIMIT";
    case MAX_POSITION_LIMIT: return "MAX_POSITION_LIMIT";
    case HARDWARE_ERROR_STATUS: return "HARDWARE_ERROR_STATUS";
    case CURRENT_LIMIT: return "CURRENT_LIMIT";
    case PWM_LIMIT: return "PWM_LIMIT";
    case GOAL_CURRENT: return "GOAL_CURRENT";
    case GOAL_PWM: return "GOAL_PWM";
    case GOAL_POSITION: return "GOAL_POSITION";
    case PRESENT_POSITION: return "PRESENT_POSITION";
    case PROFILE_VELOCITY: return "PROFILE_VELOCITY";
    case PROFILE_ACCELERATION: return "PROFILE_ACCELERATION";
    default: return "CONTROL_ITEM";
  }
}

void printFaultDetails() {
  if (!DEBUG_SERIAL) return;
  if (faultIoFailure.valid) {
    DEBUG_SERIAL.print("[DXL] ID="); DEBUG_SERIAL.print(faultIoFailure.id);
    DEBUG_SERIAL.print(" op="); DEBUG_SERIAL.print(faultIoFailure.operation);
    DEBUG_SERIAL.print(" item="); DEBUG_SERIAL.print(itemName(faultIoFailure.item));
    DEBUG_SERIAL.print(" value="); DEBUG_SERIAL.print(faultIoFailure.value);
    DEBUG_SERIAL.print(" lib="); DEBUG_SERIAL.print(faultIoFailure.lib);
    DEBUG_SERIAL.print(" status=0x"); DEBUG_SERIAL.println(faultIoFailure.status, HEX);
  }
  if (setupCheckFailed) {
    DEBUG_SERIAL.print("[CHECK] actual="); DEBUG_SERIAL.print(setupActual);
    DEBUG_SERIAL.print(" allowed="); DEBUG_SERIAL.print(setupLower);
    DEBUG_SERIAL.print(".."); DEBUG_SERIAL.println(setupUpper);
  }
}

void captureIoFailure(const char *operation, uint8_t id, uint8_t item, int32_t value) {
  lastIoFailure.valid = true;
  lastIoFailure.operation = operation;
  lastIoFailure.id = id; lastIoFailure.item = item; lastIoFailure.value = value;
  lastIoFailure.lib = dxl.getLastLibErrCode();
  lastIoFailure.status = dxl.getLastStatusPacketError();
}

bool rejectSetup(const char *stage, uint8_t id, int32_t actual, int32_t lo, int32_t hi) {
  setupStage = stage; setupMotor = id;
  setupCheckFailed = true; setupActual = actual; setupLower = lo; setupUpper = hi;
  lastIoFailure.valid = false;
  return false;
}

// Both goals MUST be computed from one clipped joint coordinate q.
// ID6's firmware Reverse mode supplies the opposite PHYSICAL direction.
// Do not invert ID6 a second time in software.
void computePairBounds() {
  qMin = max32(PAIR_2_MIN - PAIR_ZERO_2, PAIR_6_MIN - PAIR_ZERO_6);
  qMax = min32(PAIR_2_MAX - PAIR_ZERO_2, PAIR_6_MAX - PAIR_ZERO_6);
}
void setPairGoals(int32_t q) {
  q = clamp32(q, qMin, qMax);
  targetGoal[PAIR_INDEX_2] = PAIR_ZERO_2 + q;
  targetGoal[PAIR_INDEX_6] = PAIR_ZERO_6 + q;
}

bool readItem(uint8_t item, uint8_t id, int32_t &value) {
  uint32_t timeout = configurationIO ? CONFIG_TIMEOUT_MS : IO_TIMEOUT_MS;
  bool received = true;
  if (item == BUS_WATCHDOG) {
    // D2A 0.8.1 incorrectly lists X-series addr98 as TWO bytes. It is ONE.
    uint8_t raw = 0;
    received = dxl.read(id, 98, 1, &raw, 1, timeout) == 1;
    value = (int8_t)raw;
  } else value = dxl.readControlTableItem(item, id, timeout);
  bool ok = received && dxl.getLastLibErrCode() == DXL_LIB_OK &&
            (dxl.getLastStatusPacketError() & 0x7F) == 0;
  if (!ok) captureIoFailure("read", id, item, value);
  else lastIoFailure.valid = false;
  return ok;
}
bool writeItem(uint8_t item, uint8_t id, int32_t value) {
  uint32_t timeout = configurationIO ? CONFIG_TIMEOUT_MS : IO_TIMEOUT_MS;
  bool sent;
  if (item == BUS_WATCHDOG) {
    uint8_t raw = (uint8_t)value;
    sent = dxl.write(id, 98, &raw, 1, timeout);
  } else sent = dxl.writeControlTableItem(item, id, value, timeout);
  bool ok = sent && (dxl.getLastStatusPacketError() & 0x7F) == 0;
  if (!ok) captureIoFailure("write", id, item, value);
  else lastIoFailure.valid = false;
  return ok;
}
bool setIfChanged(uint8_t item, uint8_t id, int32_t value) {
  int32_t oldValue;
  return readItem(item, id, oldValue) &&
         (oldValue == value || writeItem(item, id, value));
}

void settleUnacknowledgedWrite() {
  DXL_SERIAL.flush();
  delay(2);
  uint16_t budget = 256;
  while (budget-- > 0 && DXL_SERIAL.available()) DXL_SERIAL.read();
}

bool ensureStatusReturnLevel(uint8_t id) {
  int32_t level;
  if (readItem(STATUS_RETURN_LEVEL, id, level) && level == 2) return true;
  // Level 1 replies to reads but not writes; level 0 need not reply to either.
  // Set RAM addr68 without requiring the ACK that may currently be disabled.
  uint8_t fullStatus = 2;
  if (!dxl.writeNoResp(id, 68, &fullStatus, 1)) {
    captureIoFailure("writeNoResp", id, STATUS_RETURN_LEVEL, 2); return false;
  }
  settleUnacknowledgedWrite();
  if (!readItem(STATUS_RETURN_LEVEL, id, level)) return false;
  if (level != 2) return rejectSetup("status return level readback", id, level, 2, 2);
  return true;
}

bool disablePairStartupTorque(uint8_t id) {
  int32_t firmware;
  if (!readItem(FIRMWARE_VERSION, id, firmware)) return false;
  if (firmware < 45) return true; // register 60 exists from XL430 firmware V45
  // Library 0.8.1 omits this X-series item, so use its documented raw address.
  uint8_t startup = 0;
  uint32_t timeout = configurationIO ? CONFIG_TIMEOUT_MS : IO_TIMEOUT_MS;
  if (dxl.read(id, 60, 1, &startup, 1, timeout) != 1 ||
      dxl.getLastLibErrCode() != DXL_LIB_OK ||
      (dxl.getLastStatusPacketError() & 0x7F) != 0) {
    captureIoFailure("read raw60", id, STARTUP_CONFIGURATION, startup); return false;
  }
  uint8_t safeStartup = startup & ~0x03; // disable startup torque AND RAM restore
  if (startup == safeStartup) return true;
  if (!dxl.write(id, 60, &safeStartup, 1, timeout)) {
    captureIoFailure("write raw60", id, STARTUP_CONFIGURATION, safeStartup); return false;
  }
  return true;
}

// Torque-OFF is sent in one packet to both XL430s AND all other listed motors.
// Sync Write has no per-motor acknowledgement; repeat while latched/calibrating.
void allTorqueOff() {
  const uint8_t ids[] = {1, 2, 3, 4, 5, 6, 9, 10};
  uint8_t off = 0;
  DYNAMIXEL::InfoSyncWriteInst_t info = {};
  DYNAMIXEL::XELInfoSyncWrite_t xels[8] = {};
  info.addr = 64;
  info.addr_length = 1;
  info.p_xels = xels;
  info.xel_count = 8;
  info.is_info_changed = true;
  for (uint8_t i = 0; i < 8; ++i) {
    xels[i].id = ids[i];
    xels[i].p_data = &off;
  }
  dxl.syncWrite(&info);
}

void tripFault(const char *reason, uint8_t id) {
  if (systemFault) return;
  bool hadWheels = wheelsReady;
  faultIoFailure = lastIoFailure;
  systemFault = true;
  faultReason = reason; faultId = id;
  armReady = wheelsReady = armed = armLinkActive = armHomeActive = false;
  neutralCount = 0;
  // Stop the wheels before removing their torque (best effort on a bad bus).
  if (hadWheels) {
    writeItem(GOAL_VELOCITY, LEFT_ID, 0);
    writeItem(GOAL_VELOCITY, RIGHT_ID, 0);
  }
  allTorqueOff();
  if (DEBUG_SERIAL) {
    DEBUG_SERIAL.print("[FAULT] "); DEBUG_SERIAL.print(reason);
    DEBUG_SERIAL.print(" ID="); DEBUG_SERIAL.println(id);
    printFaultDetails();
    DEBUG_SERIAL.println("Support arm, remove cause, then power-cycle. No auto-restart.");
  }
}

bool waitForMotor(uint8_t id) {
  // Start configuring as soon as this motor answers instead of always waiting
  // 300 ms. Never energize an arm whose power-up/discovery has not completed.
  while (!dxl.ping(id)) {
    captureIoFailure("ping", id, MODEL_NUMBER, 0);
    allTorqueOff();
    if ((uint32_t)(millis() - dxlPowerOnMs) >= DXL_READY_TIMEOUT_MS) return false;
    uint8_t usbBudget = 16;
    while (usbBudget-- > 0 && DEBUG_SERIAL.available()) {
      if (DEBUG_SERIAL.read() == '!') { tripFault("USB stop during motor discovery", id); return false; }
    }
    delay(1);
  }
  lastIoFailure.valid = false;
  return true;
}

bool validPairConfiguration() {
  computePairBounds();
  return PAIR_LIMITS_CONFIRMED &&
      PAIR_2_MIN >= 0 && PAIR_2_MAX <= 4095 &&
      PAIR_6_MIN >= 0 && PAIR_6_MAX <= 4095 &&
      PAIR_2_MIN < PAIR_ZERO_2 && PAIR_ZERO_2 < PAIR_2_MAX &&
      PAIR_6_MIN < PAIR_ZERO_6 && PAIR_ZERO_6 < PAIR_6_MAX &&
      LEADER_J2_ZERO >= 0 && LEADER_J2_ZERO <= 4095 && qMin < 0 && qMax > 0;
}

// ======================== PPM / WHEELS ===============================
void readPPM() {
  uint32_t now = micros();
  uint32_t interval = now - previousEdgeUs;
  previousEdgeUs = now;
  if (interval > 3000) {
    if (frameValid && channelCount >= 8 && channelCount <= 16) {
      for (uint8_t i = 0; i < 8; ++i) received[i] = working[i];
      lastFrameUs = now;
      frameReceived = true;
    }
    channelCount = 0;
    frameValid = true;
    return;
  }
  if (interval < 900 || interval > 2100) frameValid = false;
  if (channelCount < 8) working[channelCount] = interval;
  if (channelCount < 255) ++channelCount;
}

float stickValue(int value, int center) {
  int offset = value - center;
  if (abs(offset) <= DEADZONE) return 0.0f;
  float r = (abs(offset) - DEADZONE) / float(500 - DEADZONE);
  r = constrain(r, 0.0f, 1.0f);
  return offset > 0 ? r : -r;
}
bool commandWheels(int left, int right) {
  bool l = writeItem(GOAL_VELOCITY, LEFT_ID, LEFT_SIGN * left);
  bool r = writeItem(GOAL_VELOCITY, RIGHT_ID, RIGHT_SIGN * right);
  return l && r;
}
bool setupWheel(uint8_t id) {
  if (!dxl.ping(id)) return false;
  if (!ensureStatusReturnLevel(id)) return false;
  if (!writeItem(TORQUE_ENABLE, id, 0)) return false;
  if (!writeItem(BUS_WATCHDOG, id, 0)) return false;
  if (!setIfChanged(OPERATING_MODE, id, OP_VELOCITY)) return false;
  if (!setIfChanged(DRIVE_MODE, id, 0)) return false;
  if (!writeItem(PROFILE_ACCELERATION, id, 0)) return false;
  if (!writeItem(GOAL_VELOCITY, id, 0)) return false;
  if (!writeItem(TORQUE_ENABLE, id, 1)) return false;
  return writeItem(BUS_WATCHDOG, id, 10); // 200 ms bus watchdog
}

void stopOnlyWheels() {
  // No model lookup or reply is required when a wheel is absent. Do not
  // deactivate the arm merely because wheels 9/10 are not connected.
  int32_t zero = 0;
  uint8_t off = 0;
  DYNAMIXEL::InfoSyncWriteInst_t info = {};
  DYNAMIXEL::XELInfoSyncWrite_t xels[2] = {};
  info.addr = 104; info.addr_length = 4; info.p_xels = xels;
  info.xel_count = 2; info.is_info_changed = true;
  xels[0].id = LEFT_ID; xels[1].id = RIGHT_ID;
  xels[0].p_data = xels[1].p_data = (uint8_t *)&zero;
  dxl.syncWrite(&info);
  info.addr = 64; info.addr_length = 1; info.is_info_changed = true;
  info.packet.is_completed = false;
  xels[0].p_data = xels[1].p_data = &off;
  dxl.syncWrite(&info);
}
void driveControl() {
  uint16_t chTurn, chFwd;
  uint32_t frameTime;
  bool seen;
  noInterrupts();
  chTurn = received[0]; chFwd = received[2];
  frameTime = lastFrameUs; seen = frameReceived;
  interrupts();
  dbgTurn = chTurn; dbgFwd = chFwd;
  bool signalOK = seen && (uint32_t)(micros() - frameTime) < PPM_TIMEOUT_US &&
      chTurn >= 900 && chTurn <= 2100 && chFwd >= 900 && chFwd <= 2100;
  if (!signalOK) {
    if (!commandWheels(0, 0)) tripFault("wheel stop write", 9);
    armed = false; neutralCount = 0;
    return;
  }
  float forward = FORWARD_SIGN * stickValue(chFwd, FWD_CENTER);
  float turn = TURN_SIGN * stickValue(chTurn, TURN_CENTER);
  if (!armed) {
    if (!commandWheels(0, 0)) { tripFault("wheel neutral write", 9); return; }
    if (forward == 0.0f && turn == 0.0f) {
      if (++neutralCount >= 25) armed = true;
    } else neutralCount = 0;
    return;
  }
  float left = forward * MAX_FWD_SPEED + turn * MAX_TURN_SPEED;
  float right = forward * MAX_FWD_SPEED - turn * MAX_TURN_SPEED;
  float peak = max(abs(left), abs(right));
  if (peak > MAX_FWD_SPEED) {
    float scale = MAX_FWD_SPEED / peak;
    left *= scale; right *= scale;
  }
  if (!commandWheels((int)left, (int)right)) tripFault("wheel command write", 9);
}

// ====================== ARM / GRIPPER SETUP ==========================
void prepareArmSyncWrite() {
  swInfo.addr = 116;
  swInfo.addr_length = 4;
  swInfo.p_xels = swXels;
  swInfo.xel_count = 0;
  swInfo.packet.is_completed = false;
  swInfo.is_info_changed = true;
  for (uint8_t i = 0; i < ARM_MOTOR_COUNT; ++i) {
    if (!armMotorEnabled(i)) continue;
    uint8_t slot = swInfo.xel_count++;
    swXels[slot].id = ARM_IDS[i];
    swXels[slot].p_data = (uint8_t *)&swData[i].position;
  }
}

void disableGripper(const char *reason) {
  // An absent/bad ID5 must not prevent the working arm joints from running.
  // Do not use allTorqueOff() here, and never automatically re-enable ID5.
  if (!gripperAvailable) return;
  gripperAvailable = gripperReady = false;
  gripperDisabledReason = reason;
  if (DEBUG_SERIAL) {
    DEBUG_SERIAL.print("[GRIP DISABLED] ID=5 reason="); DEBUG_SERIAL.println(reason);
    if (lastIoFailure.valid) {
      DEBUG_SERIAL.print("[GRIP DXL] op="); DEBUG_SERIAL.print(lastIoFailure.operation);
      DEBUG_SERIAL.print(" item="); DEBUG_SERIAL.print(itemName(lastIoFailure.item));
      DEBUG_SERIAL.print(" lib="); DEBUG_SERIAL.print(lastIoFailure.lib);
      DEBUG_SERIAL.print(" status=0x"); DEBUG_SERIAL.println(lastIoFailure.status, HEX);
    }
    if (setupCheckFailed) {
      DEBUG_SERIAL.print("[GRIP CHECK] actual="); DEBUG_SERIAL.println(setupActual);
    }
    DEBUG_SERIAL.println("Arm IDs 1/2/3/4/6 remain enabled. Power-cycle to retry ID5.");
  }
  uint8_t off = 0;
  dxl.writeNoResp(ARM_IDS[GRIP_INDEX], 64, &off, 1);
  settleUnacknowledgedWrite();
  prepareArmSyncWrite(); // remove ID5 from EVERY subsequent goal packet
  lastIoFailure.valid = false;
  setupCheckFailed = false;
}
bool sendArmGoals() {
  for (uint8_t i = 0; i < ARM_MOTOR_COUNT; ++i) swData[i].position = targetGoal[i];
  swInfo.is_info_changed = true;
  if (!dxl.syncWrite(&swInfo)) return false;
  ++armWriteCount;
  return true;
}

// Gripper is the ONLY actuator using current-based position control.
bool configureGripper() {
  int32_t currentLimit, pwmLimit;
  setupStage = "gripper current/PWM setup"; setupMotor = 5;
  if (!readItem(CURRENT_LIMIT, 5, currentLimit)) return false;
  if (currentLimit <= 0) return rejectSetup("gripper current limit", 5, currentLimit, 1, 32767);
  if (!readItem(PWM_LIMIT, 5, pwmLimit)) return false;
  if (pwmLimit <= 0) return rejectSetup("gripper PWM limit", 5, pwmLimit, 1, 885);
  gripFollowCurrent = (uint16_t)min32(currentLimit, GRIPPER_GOAL_CURRENT);
  gripHomingCurrent = (uint16_t)min32(currentLimit, GRIP_HOME_CURRENT);
  int32_t pwm = min32(pwmLimit, 796); // original 90% PWM, bounded by EEPROM cap
  return writeItem(GOAL_CURRENT, 5, gripHomingCurrent) &&
         writeItem(GOAL_PWM, 5, pwm) &&
         writeItem(PROFILE_VELOCITY, 5, 40);
}

bool homeGripper() {
  int32_t last, error, torque;
  if (!gripperAvailable || !armReady || systemFault ||
      !readItem(TORQUE_ENABLE, 5, torque) || torque != 1) return false;
  if (!readItem(PRESENT_POSITION, 5, last)) return false;
  if (!writeItem(GOAL_POSITION, 5, last)) return false;
  // Keep the original contact homing, with timeout/error checks.
  if (!readItem(PRESENT_POSITION, 5, last)) return false;
  int64_t closeGoal = (int64_t)last + GRIP_CLOSE_DIR * (GRIP_SPAN + 500);
  if (closeGoal < -1048575 || closeGoal > 1048575) return false;
  if (!writeItem(GOAL_POSITION, 5, (int32_t)closeGoal)) return false;
  uint32_t started = millis(), lastSample = started;
  uint8_t still = 0;
  bool found = false;
  while ((uint32_t)(millis() - started) < 4000) {
    uint8_t usbBudget = 16;
    while (usbBudget-- > 0 && DEBUG_SERIAL.available()) {
      if (DEBUG_SERIAL.read() == '!') { tripFault("USB stop during gripper homing", 5); return false; }
    }
    // Arm joints are already at their initial pose. Keep safety feedback and
    // the pair's watchdog alive while the gripper finds its closed position.
    servicePairSafety();
    serviceOtherArmSafety();
    if (systemFault || !gripperAvailable) return false;
    uint32_t now = millis();
    if ((uint32_t)(now - lastSample) >= 100) {
      lastSample = now;
      int32_t position;
      if (!readItem(HARDWARE_ERROR_STATUS, 5, error) || error != 0) return false;
      if (!readItem(PRESENT_POSITION, 5, position) ||
          position < -1047000 || position > 1047000) return false;
      if ((uint32_t)(now - started) > 500 && abs32(position - last) < 3) {
        if (++still >= 3) { found = true; last = position; break; }
      } else still = 0;
      last = position;
    }
    delay(1);
  }
  if (!found) return false;
  gripClosedPos = last;
  targetGoal[GRIP_INDEX] = last - GRIP_CLOSE_DIR * GRIP_CLOSE_MARGIN;
  if (!writeItem(GOAL_POSITION, 5, targetGoal[GRIP_INDEX])) return false;
  if (!writeItem(GOAL_CURRENT, 5, gripFollowCurrent)) return false;
  if (!writeItem(PROFILE_VELOCITY, 5, ARM_PROFILE_VELOCITY)) return false;
  if (DEBUG_SERIAL) {
    DEBUG_SERIAL.print("[GRIP] closed="); DEBUG_SERIAL.println(gripClosedPos);
  }
  return true;
}

bool discoverArmMotor(uint8_t i) {
  uint8_t id = ARM_IDS[i];
  setupMotor = id; setupStage = "arm ping/model discovery";
  if (!waitForMotor(id)) return false;
  setupStage = "arm response level setup";
  if (!ensureStatusReturnLevel(id)) return false;
  setupStage = "arm torque OFF";
  if (!writeItem(TORQUE_ENABLE, id, 0)) return false;
  DYNAMIXEL::ControlTableItemInfo_t item =
      DYNAMIXEL::getControlTableItemInfo(dxl.getModelNumber(id), GOAL_POSITION);
  if (item.addr != 116 || item.addr_length != 4)
    return rejectSetup("unsupported goal position table", id, item.addr, 116, 116);

  return true;
}

bool configureArmMotor(uint8_t i) {
  uint8_t id = ARM_IDS[i];
  setupMotor = id;
  bool paired = i == PAIR_INDEX_2 || i == PAIR_INDEX_6;
  int32_t mode, hardwareError;
  setupStage = "hardware status";
  if (!readItem(HARDWARE_ERROR_STATUS, id, hardwareError)) return false;
  if (hardwareError) return rejectSetup("hardware error before torque ON", id, hardwareError, 0, 0);
  setupStage = "clear BUS_WATCHDOG";
  if (!writeItem(BUS_WATCHDOG, id, 0)) return false;
  setupStage = "operating/drive mode";
  if (!setIfChanged(OPERATING_MODE, id,
                   i == GRIP_INDEX ? OP_CURRENT_BASED_POSITION : OP_POSITION)) return false;
  if (!readItem(DRIVE_MODE, id, mode)) return false;
  // Same velocity-based profile; disable torque-on-by-goal (bit 3).
  // Keep the user's hardware reversal: ID2 normal, ID6 Reverse.
  mode &= ~0x0C;
  if (paired) mode = (i == PAIR_INDEX_6) ? 1 : 0;
  if (!setIfChanged(DRIVE_MODE, id, mode)) return false;
  setupStage = "position profile";
  if (!writeItem(PROFILE_VELOCITY, id, ARM_PROFILE_VELOCITY) ||
      !writeItem(PROFILE_ACCELERATION, id, ARM_PROFILE_ACCELERATION)) return false;
  if (paired) {
    int32_t offset, shutdown, pwmLimit, secondary;
    setupStage = "paired EEPROM/status setup";
    if (!readItem(HOMING_OFFSET, id, offset)) return false;
    if (!readItem(SECONDARY_ID, id, secondary)) return false;
    if ((secondary & 0xFF) != 255)
      return rejectSetup("paired secondary ID must be unused", id, secondary & 0xFF, 255, 255);
    if (!disablePairStartupTorque(id)) return false;
    if (!readItem(SHUTDOWN, id, shutdown)) return false;
    if (!setIfChanged(SHUTDOWN, id, shutdown | 0x3D)) return false;
    if (!readItem(PWM_LIMIT, id, pwmLimit)) return false;
    if (pwmLimit <= 0) return rejectSetup("paired PWM limit", id, pwmLimit, 1, 885);
    if (!writeItem(GOAL_PWM, id, min32(pwmLimit, PAIR_PWM_CAP_RAW))) return false;
    // Persist measured position limits in the actuator as a second guard.
    // Only update EEPROM when values actually differ.
    setupStage = "paired position limits";
    int32_t minimum = i == PAIR_INDEX_2 ? PAIR_2_MIN : PAIR_6_MIN;
    int32_t maximum = i == PAIR_INDEX_2 ? PAIR_2_MAX : PAIR_6_MAX;
    if (!setIfChanged(MIN_POSITION_LIMIT, id, minimum) ||
        !setIfChanged(MAX_POSITION_LIMIT, id, maximum)) return false;
  }
  if (i != GRIP_INDEX) {
    int32_t position, offset;
    setupStage = "read position/hardware range";
    int32_t hwMin, hwMax;
    if (!readItem(MIN_POSITION_LIMIT, id, hwMin) ||
        !readItem(MAX_POSITION_LIMIT, id, hwMax)) return false;
    hardwareMin[i] = hwMin; hardwareMax[i] = hwMax;
    if (paired && hwMin != (i == PAIR_INDEX_2 ? PAIR_2_MIN : PAIR_6_MIN))
      return rejectSetup("paired minimum readback", id, hwMin, JOINT_MIN[i], JOINT_MIN[i]);
    if (paired && hwMax != (i == PAIR_INDEX_2 ? PAIR_2_MAX : PAIR_6_MAX))
      return rejectSetup("paired maximum readback", id, hwMax, JOINT_MAX[i], JOINT_MAX[i]);
    if (hwMin < 0 || hwMax > 4095 || hwMin > hwMax)
      return rejectSetup("invalid hardware position range", id, hwMin, 0, hwMax);
    if (jointMinimum(i) > jointMaximum(i))
      return rejectSetup("no shared hardware/software position range", id, jointMinimum(i), 0, jointMaximum(i));
    int32_t initial = i == PAIR_INDEX_2 ? PAIR_ZERO_2 :
                     (i == PAIR_INDEX_6 ? PAIR_ZERO_6 : ARM_INITIAL_POSITION_RAW);
    if (ARM_HOME_AT_STARTUP && (initial < jointMinimum(i) || initial > jointMaximum(i)))
      return rejectSetup("initial target outside effective range", id, initial, jointMinimum(i), jointMaximum(i));
    if (!readItem(PRESENT_POSITION, id, position) ||
        !readItem(HOMING_OFFSET, id, offset)) return false;
    // Torque OFF can report multiple turns; OP_POSITION torque ON resets
    // to one turn, plus the existing Homing Offset.
    if (offset < -1024 || offset > 1024) offset = 0; // ignored by firmware in OP_POSITION
    int64_t singleTurn = ((int64_t)position - offset) % 4096;
    if (singleTurn < 0) singleTurn += 4096;
    position = (int32_t)(singleTurn + offset);
    if (position < jointMinimum(i) || position > jointMaximum(i))
      return rejectSetup("starting position outside effective range", id, position, jointMinimum(i), jointMaximum(i));
    targetGoal[i] = position; // preload present pose before slow initial move
  }

  return true;
}

bool setupArm() {
  // Required joints are discovered before optional ID5. A missing gripper
  // cannot use up the discovery deadline before required paired ID6.
  const uint8_t discoveryOrder[] = {0, 1, 2, 3, 5, 4};
  for (uint8_t slot = 0; slot < ARM_MOTOR_COUNT; ++slot) {
    uint8_t i = discoveryOrder[slot];
    if (!discoverArmMotor(i)) {
      if (systemFault || i != GRIP_INDEX) return false;
      disableGripper(setupStage);
    }
  }
  for (uint8_t id = 2; id <= 6; id += 4) {
    int32_t model;
    setupStage = "paired model read"; setupMotor = id;
    if (!readItem(MODEL_NUMBER, id, model)) return false;
    if (model != XL430_MODEL_NUMBER)
      return rejectSetup("paired model mismatch", id, model, XL430_MODEL_NUMBER, XL430_MODEL_NUMBER);
  }

  for (uint8_t i = 0; i < ARM_MOTOR_COUNT; ++i) {
    if (!armMotorEnabled(i)) continue;
    if (!configureArmMotor(i)) {
      if (i != GRIP_INDEX) return false;
      disableGripper(setupStage);
    }
  }
  int32_t q2 = targetGoal[PAIR_INDEX_2] - PAIR_ZERO_2;
  int32_t q6 = targetGoal[PAIR_INDEX_6] - PAIR_ZERO_6;
  if (abs32(q2 - q6) > PAIR_STARTUP_SYNC_RAW)
    return rejectSetup("paired starting alignment", 6, q2 - q6, -PAIR_STARTUP_SYNC_RAW, PAIR_STARTUP_SYNC_RAW);
  qCommand = qTarget = clamp32((q2 + q6) / 2, qMin, qMax);
  setPairGoals(qCommand);
  if (gripperAvailable && !configureGripper()) disableGripper(setupStage);
  // Refresh positions after configuration. Hold the gripper here; contact
  // homing is deferred until AFTER the arm has moved to its initial pose.
  for (uint8_t i = 0; i < ARM_MOTOR_COUNT; ++i) {
    if (!armMotorEnabled(i)) continue;
    setupMotor = ARM_IDS[i]; setupStage = "refresh pose before torque ON";
    if (i == GRIP_INDEX) {
      int32_t position;
      if (!readItem(PRESENT_POSITION, ARM_IDS[i], position)) {
        disableGripper(setupStage); continue;
      }
      if (position < -1047000 || position > 1047000) {
        disableGripper("gripper starting position range"); continue;
      }
      targetGoal[i] = position;
      continue;
    }
    int32_t position, offset;
    if (!readItem(PRESENT_POSITION, ARM_IDS[i], position) ||
        !readItem(HOMING_OFFSET, ARM_IDS[i], offset)) return false;
    if (offset < -1024 || offset > 1024) offset = 0;
    int64_t oneTurn = ((int64_t)position - offset) % 4096;
    if (oneTurn < 0) oneTurn += 4096;
    position = (int32_t)(oneTurn + offset);
    if (position < jointMinimum(i) || position > jointMaximum(i))
      return rejectSetup("pre-enable position outside effective range", ARM_IDS[i], position, jointMinimum(i), jointMaximum(i));
    targetGoal[i] = position;
  }
  q2 = targetGoal[PAIR_INDEX_2] - PAIR_ZERO_2;
  q6 = targetGoal[PAIR_INDEX_6] - PAIR_ZERO_6;
  if (abs32(q2 - q6) > PAIR_STARTUP_SYNC_RAW)
    return rejectSetup("paired pre-enable alignment", 6, q2 - q6, -PAIR_STARTUP_SYNC_RAW, PAIR_STARTUP_SYNC_RAW);
  qCommand = qTarget = clamp32((q2 + q6) / 2, qMin, qMax);
  setPairGoals(qCommand);
  prepareArmSyncWrite();
  // A broadcast alone cannot confirm that each motor received its hold goal.
  // Preload with acknowledged unicast writes before the shared torque-ON.
  for (uint8_t i = 0; i < ARM_MOTOR_COUNT; ++i) {
    if (!armMotorEnabled(i)) continue;
    setupMotor = ARM_IDS[i]; setupStage = "preload hold goal";
    if (!writeItem(GOAL_POSITION, ARM_IDS[i], targetGoal[i])) {
      if (i != GRIP_INDEX) return false;
      disableGripper(setupStage);
    }
  }
  if (!sendArmGoals()) return false; // preload every goal BEFORE torque ON

  uint8_t on = 1;
  setupStage = "arm torque ON packet";
  DYNAMIXEL::InfoSyncWriteInst_t info = {};
  DYNAMIXEL::XELInfoSyncWrite_t xels[ARM_MOTOR_COUNT] = {};
  info.addr = 64; info.addr_length = 1; info.p_xels = xels;
  info.xel_count = 0; info.is_info_changed = true;
  for (uint8_t i = 0; i < ARM_MOTOR_COUNT; ++i) {
    if (!armMotorEnabled(i)) continue;
    uint8_t slot = info.xel_count++;
    xels[slot].id = ARM_IDS[i]; xels[slot].p_data = &on;
  }
  if (!dxl.syncWrite(&info)) return false;
  for (uint8_t i = 0; i < ARM_MOTOR_COUNT; ++i) {
    if (!armMotorEnabled(i)) continue;
    int32_t torque;
    setupMotor = ARM_IDS[i]; setupStage = "verify torque ON";
    if (!readItem(TORQUE_ENABLE, ARM_IDS[i], torque)) {
      if (i != GRIP_INDEX) return false;
      disableGripper(setupStage); continue;
    }
    if (torque != 1) {
      rejectSetup("torque did not enable", ARM_IDS[i], torque, 1, 1);
      if (i != GRIP_INDEX) return false;
      disableGripper(setupStage);
    }
  }
  return true;
}

// ====================== PAIR SAFETY MONITOR ==========================
// One contiguous read per XL430, 64..146. At 1 Mbps this avoids many small
// transactions. 5 ms timeout per read bounds the impact on the wheel loop.
bool readPairMotor(uint8_t id, int32_t &position, int16_t &load, uint8_t &temp) {
  uint8_t data[83] = {};
  int32_t count = dxl.read(id, 64, sizeof(data), data, sizeof(data), IO_TIMEOUT_MS);
  if (count != (int32_t)sizeof(data) ||
      dxl.getLastLibErrCode() != DXL_LIB_OK ||
      (dxl.getLastStatusPacketError() & 0x7F) != 0) return false;
  if (data[70 - 64] != 0) { tripFault("pair hardware error", id); return false; }
  if (data[64 - 64] != 1) { tripFault("pair torque lost", id); return false; }
  if ((int8_t)data[98 - 64] < 0) { tripFault("pair bus watchdog", id); return false; }
  memcpy(&load, &data[126 - 64], sizeof(load));
  memcpy(&position, &data[132 - 64], sizeof(position));
  temp = data[146 - 64];
  return true;
}

bool heldFor(bool condition, uint32_t now, uint32_t duration,
             uint32_t &since, bool &timing) {
  if (!condition) { timing = false; return false; }
  if (!timing) { since = now; timing = true; }
  return (uint32_t)(now - since) >= duration;
}

// 1=temperature, 2=position limit, 3=large alignment error.
// Bounds are checked before subtraction, so corrupt INT32 positions are safe.
uint8_t instantaneousPairFault() {
  if (pairSample.temp2 >= PAIR_TEMP_TRIP_C || pairSample.temp6 >= PAIR_TEMP_TRIP_C)
    return 1;
  if (pairSample.pos2 < PAIR_2_MIN || pairSample.pos2 > PAIR_2_MAX ||
      pairSample.pos6 < PAIR_6_MIN || pairSample.pos6 > PAIR_6_MAX) return 2;
  int32_t mismatch = abs32((pairSample.pos2 - PAIR_ZERO_2) -
                           (pairSample.pos6 - PAIR_ZERO_6));
  if (mismatch > PAIR_SYNC_HARD_RAW) return 3;
  return 0;
}

void servicePairSafety() {
  static uint32_t lastPoll = 0;
  static bool polled = false;
  uint32_t now = millis();
  if (!armReady || systemFault || (polled && (uint32_t)(now - lastPoll) < PAIR_POLL_MS)) return;
  polled = true;
  lastPoll = now;
  PairSample sample = {};
  bool ok2 = readPairMotor(2, sample.pos2, sample.load2, sample.temp2);
  bool ok6 = false;
  if (!systemFault) ok6 = readPairMotor(6, sample.pos6, sample.load6, sample.temp6);
  pairSampleOK = ok2 && ok6;
  if (systemFault) return;
  if (!pairSampleOK) {
    if (++pairCommMisses >= PAIR_COMM_MISS_LIMIT) tripFault("pair read timeout", 2);
    return; // Never issue a NEW target while either motor is unverified.
  }
  pairCommMisses = 0;
  pairSample = sample;
  uint8_t fault = instantaneousPairFault();
  if (fault != 0) {
    tripFault(fault == 1 ? "pair temperature" :
              (fault == 2 ? "pair position limit" : "pair alignment hard limit"), 2);
    return;
  }
  int32_t mismatch = abs32((sample.pos2 - PAIR_ZERO_2) - (sample.pos6 - PAIR_ZERO_6));
  int32_t following = max32(abs32(sample.pos2 - (PAIR_ZERO_2 + qCommand)),
                            abs32(sample.pos6 - (PAIR_ZERO_6 + qCommand)));
  int32_t peakLoad = max32(abs32(sample.load2), abs32(sample.load6));
  now = millis();
  if (heldFor(peakLoad >= PAIR_LOAD_TRIP_RAW, now, PAIR_LOAD_TRIP_MS,
              loadSince, loadTiming)) { tripFault("pair sustained load", 2); return; }
  if (heldFor(mismatch > PAIR_SYNC_TRIP_RAW, now, PAIR_SYNC_TRIP_MS,
              syncSince, syncTiming)) { tripFault("pair alignment", 2); return; }
  if (heldFor(following > PAIR_TRACK_TRIP_RAW, now, PAIR_TRACK_TRIP_MS,
              trackSince, trackTiming)) { tripFault("pair stalled/following error", 2); return; }
}

void serviceOtherArmSafety() {
  static uint32_t lastPoll = 0;
  static uint8_t index = 0;
  const uint8_t ids[] = {1, 3, 4, 5};
  uint32_t now = millis();
  if (!armReady || systemFault || (uint32_t)(now - lastPoll) < 50) return;
  lastPoll = now;
  uint8_t id = ids[index];
  index = (index + 1) % 4;
  if (id == ARM_IDS[GRIP_INDEX] && !gripperAvailable) return;
  int32_t error, torque;
  if (!readItem(HARDWARE_ERROR_STATUS, id, error) ||
      !readItem(TORQUE_ENABLE, id, torque)) {
    if (id == ARM_IDS[GRIP_INDEX]) disableGripper("gripper status read");
    else tripFault("arm status read", id);
    return;
  }
  if (error != 0 || torque != 1) {
    if (id == ARM_IDS[GRIP_INDEX]) disableGripper("gripper hardware/torque");
    else tripFault("arm hardware/torque", id);
  }
}

// ====================== FOLLOWER INITIAL POSE ========================
int32_t armInitialGoal(uint8_t index) {
  if (index == PAIR_INDEX_2) return PAIR_ZERO_2;
  if (index == PAIR_INDEX_6) return PAIR_ZERO_6;
  if (index == GRIP_INDEX) return targetGoal[GRIP_INDEX];
  return ARM_INITIAL_POSITION_RAW;
}

bool armInitialPoseAllowed() {
  if (qMin > 0 || qMax < 0) return false;
  for (uint8_t i = 0; i < ARM_MOTOR_COUNT; ++i) {
    if (i == GRIP_INDEX) continue;
    int32_t goal = armInitialGoal(i);
    if (goal < jointMinimum(i) || goal > jointMaximum(i)) return false;
  }
  return true;
}

int32_t armInitialGoalAt(int32_t start, int32_t goal, uint32_t progress, uint32_t duration) {
  if (progress >= duration) return goal;
  return start + (int32_t)(((int64_t)goal - start) * progress / duration);
}

bool readArmInitialPose(int32_t *pose) {
  if (!pairSampleOK) return false;
  pose[PAIR_INDEX_2] = pairSample.pos2;
  pose[PAIR_INDEX_6] = pairSample.pos6;
  pose[GRIP_INDEX] = targetGoal[GRIP_INDEX];
  for (uint8_t i = 0; i < ARM_MOTOR_COUNT; ++i) {
    if (i == GRIP_INDEX) continue;
    if (i != PAIR_INDEX_2 && i != PAIR_INDEX_6) {
      int32_t position;
      if (!readItem(PRESENT_POSITION, ARM_IDS[i], position)) return false;
      pose[i] = position;
    }
    if (pose[i] < jointMinimum(i) || pose[i] > jointMaximum(i)) return false;
  }
  return true;
}

bool armInitialPoseReached(const int32_t *pose) {
  for (uint8_t i = 0; i < ARM_MOTOR_COUNT; ++i) {
    if (i == GRIP_INDEX) continue;
    int32_t error = pose[i] - armInitialGoal(i);
    if (error < -ARM_HOME_TOLERANCE_RAW || error > ARM_HOME_TOLERANCE_RAW) return false;
  }
  return true;
}

bool moveArmToInitialPose() {
  // Calibration exits setup BEFORE this function. It cannot enable motion.
  if (CALIBRATION_MODE || !validPairConfiguration() || !armInitialPoseAllowed() ||
      !armReady || !pairSampleOK || wheelsReady) {
    tripFault("initial pose not ready/allowed", 0); return false;
  }
  int32_t start[ARM_MOTOR_COUNT] = {};
  if (!readArmInitialPose(start)) { tripFault("initial pose read failed", 0); return false; }
  int32_t q2 = start[PAIR_INDEX_2] - PAIR_ZERO_2;
  int32_t q6 = start[PAIR_INDEX_6] - PAIR_ZERO_6;
  if (abs32(q2 - q6) > PAIR_STARTUP_SYNC_RAW) {
    tripFault("initial pair alignment", 2); return false;
  }
  int32_t startQ = clamp32((q2 + q6) / 2, qMin, qMax);
  qCommand = qTarget = startQ;
  for (uint8_t i = 0; i < ARM_MOTOR_COUNT; ++i)
    if (i != GRIP_INDEX) targetGoal[i] = start[i];
  setPairGoals(startQ);
  int32_t distance = 0;
  for (uint8_t i = 0; i < ARM_MOTOR_COUNT; ++i) {
    start[i] = targetGoal[i];
    if (i != GRIP_INDEX)
      distance = max32(distance, abs32(armInitialGoal(i) - start[i]));
  }
  // At most PAIR_STEP_RAW per nominal control tick for EVERY moving motor.
  // Use the same duration for all joints and one q trajectory for ID2 + ID6.
  uint32_t duration = ((distance + PAIR_STEP_RAW - 1) / PAIR_STEP_RAW) * ARM_CONTROL_MS;
  // If already within the arrival tolerance, just verify the pose promptly.
  // The three-second plan is for an actual move, not an idle boot wait.
  if (distance > ARM_HOME_TOLERANCE_RAW && duration < ARM_HOME_MIN_MS)
    duration = ARM_HOME_MIN_MS;
  if (duration + ARM_CONTROL_MS * ARM_HOME_STABLE_SAMPLES >= ARM_HOME_TIMEOUT_MS) {
    tripFault("initial pose duration exceeds timeout", 0); return false;
  }
  armHomeActive = true; armLinkActive = false;
  if (!sendArmGoals()) { tripFault("initial hold write failed", 0); return false; }
  if (DEBUG_SERIAL) DEBUG_SERIAL.println("[ARM HOME] Moving to 2048/paired zeros. Wheels OFF.");
  uint32_t started = millis(), lastTick = started, progress = 0;
  uint32_t lagSince = 0;
  bool lagTiming = false;
  uint8_t stable = 0;
  while ((uint32_t)(millis() - started) < ARM_HOME_TIMEOUT_MS) {
    uint8_t usbBudget = 16;
    while (usbBudget-- > 0 && DEBUG_SERIAL.available()) {
      if (DEBUG_SERIAL.read() == '!') { tripFault("USB stop during initial move", 0); return false; }
    }
    servicePairSafety();
    serviceOtherArmSafety();
    if (systemFault) return false;
    uint32_t now = millis();
    if ((uint32_t)(now - lastTick) >= ARM_CONTROL_MS) {
      lastTick = now;
      if (pairSampleOK) {
        int32_t actual[ARM_MOTOR_COUNT] = {};
        if (!readArmInitialPose(actual)) { tripFault("initial position read/limit", 0); return false; }
        int32_t error = 0;
        for (uint8_t i = 0; i < ARM_MOTOR_COUNT; ++i)
          if (i != GRIP_INDEX) error = max32(error, abs32(actual[i] - targetGoal[i]));
        if (heldFor(error > ARM_HOME_TRACK_ABORT_RAW, now, ARM_HOME_TRACK_ABORT_MS,
                    lagSince, lagTiming)) { tripFault("initial move stalled", 0); return false; }
        if (progress >= duration && armInitialPoseReached(actual)) {
          if (++stable >= ARM_HOME_STABLE_SAMPLES) {
            armHomeActive = false;
            if (DEBUG_SERIAL) DEBUG_SERIAL.println("[ARM HOME] Reached. Holding pose; Leader follow next.");
            return true; // keep Follower holding torque ON
          }
        } else stable = 0;
        // Failed pair reads pause progression; they never cause a catch-up jump.
        if (progress < duration) progress += ARM_CONTROL_MS;
        if (progress > duration) progress = duration;
        for (uint8_t i = 0; i < ARM_MOTOR_COUNT; ++i)
          if (i != GRIP_INDEX && i != PAIR_INDEX_2 && i != PAIR_INDEX_6)
            targetGoal[i] = armInitialGoalAt(start[i], armInitialGoal(i), progress, duration);
        qCommand = qTarget = armInitialGoalAt(startQ, 0, progress, duration);
        setPairGoals(qCommand);
        if (!sendArmGoals()) { tripFault("initial goal write failed", 0); return false; }
      }
    }
    delay(1); // setup only; safety polling and USB stop remain active
  }
  tripFault("initial pose arrival timeout", 0);
  return false;
}

// ======================= BLUETOOTH / GRIPPER =========================
// Parse and validate the ENTIRE frame before touching any targets/history.
// No strtol overflow, no extra/missing fields, no state changes on rejection.
bool parseLeaderLine(const char *line, int32_t *values) {
  uint8_t pos = 0;
  for (uint8_t field = 0; field < LEADER_JOINT_COUNT; ++field) {
    bool negative = false;
    if (line[pos] == '-') { negative = true; ++pos; }
    if (line[pos] < '0' || line[pos] > '9') return false;
    int32_t value = 0;
    while (line[pos] >= '0' && line[pos] <= '9') {
      value = value * 10 + (line[pos] - '0');
      if (value > 4095) return false; // bounded BEFORE another multiplication
      ++pos;
    }
    if (negative) value = -value;
    if (field == GRIP_INDEX) {
      if (value < -3000 || value > 3000) return false;
    } else if (value < 0 || value > 4095) return false;
    values[field] = value;
    if (field + 1 < LEADER_JOINT_COUNT) {
      if (line[pos] != ',') return false;
      ++pos;
    } else if (line[pos] != '\0') return false;
  }
  return true;
}

int32_t gripperGoal(int32_t delta) {
  int32_t c = GRIP_CLOSE_DIR * LEADER_GRIP_SIGN * delta;
  leaderGripHist[0] = leaderGripHist[1];
  leaderGripHist[1] = leaderGripHist[2]; leaderGripHist[2] = c;
  if (leaderGripHistN < 3) ++leaderGripHistN;
  if (!leaderGripSeen) {
    leaderGripClosed = leaderGripOpen = c;
    leaderGripSeen = true;
  } else if (leaderGripHistN >= 3) {
    int32_t lo = min32(leaderGripHist[0], min32(leaderGripHist[1], leaderGripHist[2]));
    int32_t hi = max32(leaderGripHist[0], max32(leaderGripHist[1], leaderGripHist[2]));
    if (lo > leaderGripClosed) leaderGripClosed = lo;
    if (hi < leaderGripOpen) leaderGripOpen = hi;
  }
  int32_t span = leaderGripClosed - leaderGripOpen;
  float fraction = span > 30 ? float(leaderGripClosed - c) / span : 0.0f;
  fraction = constrain(fraction, 0.0f, 1.0f);
  int32_t open = (int32_t)(fraction * (GRIP_SPAN - GRIP_OPEN_MARGIN - GRIP_CLOSE_MARGIN));
  return gripClosedPos - GRIP_CLOSE_DIR * (GRIP_CLOSE_MARGIN + open);
}

void applyFollowProfile() {
  // X-series RAM: acceleration (108, four bytes), then velocity (112, four bytes).
  // One broadcast sets the two coupled motors' profiles at the same time.
  int32_t profile[2] = {FOLLOW_PROFILE_ACCELERATION, FOLLOW_PROFILE_VELOCITY};
  DYNAMIXEL::InfoSyncWriteInst_t info = {};
  DYNAMIXEL::XELInfoSyncWrite_t xels[ARM_MOTOR_COUNT] = {};
  info.addr = 108; info.addr_length = sizeof(profile); info.p_xels = xels;
  info.is_info_changed = true;
  for (uint8_t i = 0; i < ARM_MOTOR_COUNT; ++i) {
    if (i == GRIP_INDEX) continue;
    uint8_t slot = info.xel_count++;
    xels[slot].id = ARM_IDS[i];
    xels[slot].p_data = (uint8_t *)profile;
  }
  if (dxl.syncWrite(&info)) pairFollowStep = PAIR_FOLLOW_STEP_RAW;
  else if (DEBUG_SERIAL) DEBUG_SERIAL.println("[ARM] Follow profile TX failed; retaining initial step limit.");
}

void applyArmValues(const int32_t *values) {
  if (!armReady || systemFault || armHomeActive) return;
  for (uint8_t i = 0; i < LEADER_JOINT_COUNT; ++i) {
    if (i == PAIR_INDEX_2) {
      qTarget = clamp32(LEADER_TO_FOLLOWER_J2_SIGN * (values[i] - LEADER_J2_ZERO), qMin, qMax);
    } else if (i == GRIP_INDEX) {
      if (gripperReady) targetGoal[i] = gripperGoal(values[i]);
    } else targetGoal[i] = clamp32(values[i], jointMinimum(i), jointMaximum(i));
  }
  lastArmPacketMs = millis();
  armLinkActive = true;
  ++armPacketCount;
}

void pollArmLink() {
  static char buffer[96];
  static uint8_t length = 0;
  static bool discard = true; // discard any partial frame left from setup
  int32_t latest[LEADER_JOINT_COUNT] = {};
  bool haveValues = false;
  uint16_t budget = 192; // bounded even if a sender streams without pausing
  while (budget-- > 0 && BLT_SERIAL.available()) {
    char c = (char)BLT_SERIAL.read();
    if (c == '\n') {
      if (discard) ++armRejectCount;
      else if (length > 0) {
        buffer[length] = '\0';
        int32_t parsed[LEADER_JOINT_COUNT];
        if (parseLeaderLine(buffer, parsed)) {
          memcpy(latest, parsed, sizeof(latest)); haveValues = true;
        } else ++armRejectCount;
      }
      length = 0; discard = false;
    } else if (c == '\r') {
      continue; // accept the original CRLF/LF formats
    } else if (!discard) {
      if ((uint8_t)c < 32 || (uint8_t)c > 126 || length >= sizeof(buffer) - 1) {
        discard = true; // discard THROUGH newline, never accept a long-line tail
      } else buffer[length++] = c;
    }
  }
  if (haveValues) applyArmValues(latest); // latest complete VALID frame
}

bool holdArmAtPresent() {
  if (!pairSampleOK) return false;
  // A brief link dropout stops the trajectory at the measured position while
  // retaining holding torque. A hardware fault instead latches torque OFF.
  int32_t q2 = pairSample.pos2 - PAIR_ZERO_2;
  int32_t q6 = pairSample.pos6 - PAIR_ZERO_6;
  if (abs32(q2 - q6) > PAIR_SYNC_TRIP_RAW) return false;
  qCommand = qTarget = clamp32((q2 + q6) / 2, qMin, qMax);
  setPairGoals(qCommand);
  for (uint8_t i = 0; i < LEADER_JOINT_COUNT; ++i) {
    if (i == PAIR_INDEX_2) continue;
    if (i == GRIP_INDEX && !gripperReady) continue;
    int32_t position;
    if (!readItem(PRESENT_POSITION, ARM_IDS[i], position)) {
      if (i != GRIP_INDEX) return false;
      disableGripper("gripper Bluetooth hold read"); continue;
    }
    if (i == GRIP_INDEX) {
      int64_t closed = (int64_t)gripClosedPos - GRIP_CLOSE_DIR * GRIP_CLOSE_MARGIN;
      int64_t open = (int64_t)gripClosedPos - GRIP_CLOSE_DIR * (GRIP_SPAN - GRIP_OPEN_MARGIN);
      bool outside = GRIP_CLOSE_DIR > 0 ? position < open || position > closed :
                                        position < closed || position > open;
      if (outside) { disableGripper("gripper Bluetooth hold range"); continue; }
    } else if (position < jointMinimum(i) || position > jointMaximum(i)) return false;
    targetGoal[i] = position;
  }
  return sendArmGoals();
}

void serviceArmMotion() {
  static uint32_t lastControl = 0;
  uint32_t now = millis();
  if (!armReady || systemFault || armHomeActive || !pairSampleOK ||
      (uint32_t)(now - lastControl) < ARM_CONTROL_MS) return;
  lastControl = now;
  if (armLinkActive && (uint32_t)(now - lastArmPacketMs) >= ARM_LINK_TIMEOUT_MS) {
    armLinkActive = false;
    if (!holdArmAtPresent()) { tripFault("Bluetooth hold failed", 2); return; }
    if (DEBUG_SERIAL) DEBUG_SERIAL.println("[BT] timeout: holding measured pose.");
    return;
  }
  int32_t step = clamp32(qTarget - qCommand, -pairFollowStep, pairFollowStep);
  qCommand += step;
  setPairGoals(qCommand);
  if (!sendArmGoals()) tripFault("arm sync write", 2);
}

// ======================= CALIBRATION / MAIN =========================
void calibrationMonitor() {
  static uint32_t lastOff = 0, lastPrint = 0;
  uint32_t now = millis();
  if ((uint32_t)(now - lastOff) >= 250) { lastOff = now; allTorqueOff(); }
  if (!DEBUG_SERIAL || (uint32_t)(now - lastPrint) < 500) return;
  lastPrint = now;
  for (uint8_t i = 0; i < 2; ++i) {
    uint8_t id = i == 0 ? 2 : 6;
    int32_t raw, offset, mode, torque;
    if (!readItem(PRESENT_POSITION, id, raw) ||
        !readItem(HOMING_OFFSET, id, offset) ||
        !readItem(DRIVE_MODE, id, mode) ||
        !readItem(TORQUE_ENABLE, id, torque)) {
      DEBUG_SERIAL.print("[CAL] no valid read ID="); DEBUG_SERIAL.println(id);
      continue;
    }
    int64_t absolute = ((int64_t)raw - offset) % 4096;
    if (absolute < 0) absolute += 4096;
    DEBUG_SERIAL.print("[CAL] ID="); DEBUG_SERIAL.print(id);
    DEBUG_SERIAL.print(" pos="); DEBUG_SERIAL.print(raw);
    DEBUG_SERIAL.print(" single="); DEBUG_SERIAL.print((int32_t)absolute + offset);
    DEBUG_SERIAL.print(" offset="); DEBUG_SERIAL.print(offset);
    DEBUG_SERIAL.print(" drive="); DEBUG_SERIAL.print(mode);
    DEBUG_SERIAL.print(" torque="); DEBUG_SERIAL.println(torque);
  }
}

void setup() {
  DEBUG_SERIAL.begin(115200);
  if (DEBUG_SERIAL) DEBUG_SERIAL.println("[BUILD] " FIRMWARE_LABEL);
  BLT_SERIAL.begin(57600);
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), readPPM, RISING);
  pinMode(31, OUTPUT);
  digitalWrite(31, HIGH);
  dxlPowerOnMs = millis();
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(2.0);
  allTorqueOff();
  if (CALIBRATION_MODE) {
    // Make calibration readings use the same coordinates as RUN mode:
    // ID2 normal, ID6 Reverse. Change settings only with torque confirmed OFF.
    for (uint8_t i = 0; i < 2; ++i) {
      uint8_t id = i == 0 ? 2 : 6;
      if (!waitForMotor(id) || !ensureStatusReturnLevel(id) || dxl.getModelNumber(id) != XL430_MODEL_NUMBER ||
          !writeItem(TORQUE_ENABLE, id, 0) ||
          !writeItem(BUS_WATCHDOG, id, 0) ||
          !setIfChanged(DRIVE_MODE, id, id == 6 ? 1 : 0) ||
          !setIfChanged(OPERATING_MODE, id, OP_POSITION) ||
          !disablePairStartupTorque(id)) {
        tripFault("calibration XL430 setup", id); return;
      }
    }
    if (DEBUG_SERIAL) DEBUG_SERIAL.println("[CAL] ALL motors OFF. Support arm; measure travel manually.");
    return;
  }
  if (!validPairConfiguration()) { tripFault("unconfirmed/invalid pair limits", 2); return; }
  if (ARM_HOME_AT_STARTUP && !armInitialPoseAllowed()) {
    tripFault("initial target outside allowed limits", 0); return;
  }
  if (!setupArm()) { tripFault(setupStage, setupMotor); return; }
  configurationIO = false;
  setupCheckFailed = false;
  // From here onward safety feedback keeps watchdogs alive during both the
  // initial arm move and the subsequent gripper calibration.
  if (!writeItem(BUS_WATCHDOG, 2, 10) || !writeItem(BUS_WATCHDOG, 6, 10)) {
    tripFault("pair watchdog setup", 2); return;
  }
  armReady = true;
  servicePairSafety();
  if (systemFault || !pairSampleOK) { tripFault("pair startup verification", 2); return; }
  if (ARM_HOME_AT_STARTUP && !moveArmToInitialPose()) return;
  if (gripperAvailable) {
    bool homed = homeGripper();
    if (systemFault) return; // pair/core fault or USB stop still stops everything
    if (!homed) disableGripper("gripper homing after initial pose failed");
    else gripperReady = gripperAvailable;
  }
  applyFollowProfile();
  bool leftOK = setupWheel(LEFT_ID);
  bool rightOK = setupWheel(RIGHT_ID);
  wheelsReady = leftOK && rightOK;
  if (wheelsReady) {
    if (!commandWheels(0, 0)) { tripFault("wheel startup stop", 9); return; }
  } else {
    stopOnlyWheels();
    lastIoFailure.valid = false;
    if (DEBUG_SERIAL) DEBUG_SERIAL.println("[DRIVE] Wheel setup unavailable; wheels OFF, arm follow remains enabled.");
  }
  // Drop complete AND partial Bluetooth frames accumulated during setup.
  uint16_t budget = 256;
  while (budget-- > 0 && BLT_SERIAL.available()) BLT_SERIAL.read();
  if (DEBUG_SERIAL) {
    DEBUG_SERIAL.println("[READY] Arm IDs 1/2/3/4/6 enabled. Leader still sends FIVE values.");
    DEBUG_SERIAL.print("[GRIP] ");
    DEBUG_SERIAL.println(gripperReady ? "ready" : "disabled; other arm joints remain enabled");
  }
}

void loop() {
  if (systemFault) {
    static uint32_t lastOff = 0, lastFaultPrint = 0;
    if ((uint32_t)(millis() - lastOff) >= 100) { lastOff = millis(); allTorqueOff(); }
    if (DEBUG_SERIAL && (uint32_t)(millis() - lastFaultPrint) >= 1000) {
      lastFaultPrint = millis();
      DEBUG_SERIAL.print("[LATCHED] "); DEBUG_SERIAL.print(faultReason);
      DEBUG_SERIAL.print(" ID="); DEBUG_SERIAL.print(faultId);
      DEBUG_SERIAL.print(" build="); DEBUG_SERIAL.println(FIRMWARE_LABEL);
      printFaultDetails();
    }
    return;
  }
  if (CALIBRATION_MODE) { calibrationMonitor(); return; }
  // A USB '!' is a latched software stop, not a physical emergency stop.
  uint8_t usbBudget = 16;
  while (usbBudget-- > 0 && DEBUG_SERIAL.available()) {
    if (DEBUG_SERIAL.read() == '!') { tripFault("USB stop", 0); return; }
  }
  static uint32_t lastDrive = 0;
  if (wheelsReady && (uint32_t)(millis() - lastDrive) >= 20) {
    lastDrive = millis(); driveControl();
  }
  if (systemFault) return;
  servicePairSafety();
  serviceOtherArmSafety();
  if (systemFault) return;
  pollArmLink();
  serviceArmMotion();
#if DEBUG
  static uint32_t lastPrint = 0;
  if (DEBUG_SERIAL && (uint32_t)(millis() - lastPrint) >= 250) {
    lastPrint = millis();
    DEBUG_SERIAL.print("CH1="); DEBUG_SERIAL.print(dbgTurn);
    DEBUG_SERIAL.print(" CH3="); DEBUG_SERIAL.print(dbgFwd);
    DEBUG_SERIAL.print(" armed="); DEBUG_SERIAL.print(armed);
    DEBUG_SERIAL.print(" packets="); DEBUG_SERIAL.print(armPacketCount);
    DEBUG_SERIAL.print(" reject="); DEBUG_SERIAL.print(armRejectCount);
    DEBUG_SERIAL.print(" grip="); DEBUG_SERIAL.print(gripperReady ? "ready" : "disabled");
    DEBUG_SERIAL.print(" p2="); DEBUG_SERIAL.print(pairSample.pos2);
    DEBUG_SERIAL.print(" p6="); DEBUG_SERIAL.print(pairSample.pos6);
    DEBUG_SERIAL.print(" L2="); DEBUG_SERIAL.print(pairSample.load2);
    DEBUG_SERIAL.print(" L6="); DEBUG_SERIAL.println(pairSample.load6);
  }
#endif
}
