// OpenRB-150 Leader with startup homing, compatible with ManualRobot_DualXL430.ino.
// Bluetooth format NEVER changes: p1,p2,p3,p4,gripper_delta\n (FIVE values).
// The Follower computes its ID6 goal from the logical joint-2 value.
// Original boot flow: command 2048 with a slow motor profile, wait 3 seconds,
// then torque OFF and stream ACTUAL measured positions. No exact-arrival gate.
//
// Confirmed hardware: Leader IDs 1..5, Follower IDs 1..6 plus wheels 9/10.
// ID6 exists ONLY on the Follower. Never query or configure ID6 here.

#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#if !defined(ARDUINO_OpenRB)
  #error "Select the OpenRB-150 board for this sketch."
#endif

#define DXL_SERIAL Serial1
#define DEBUG_SERIAL Serial
#define BLT_SERIAL Serial2
#define FIRMWARE_LABEL "LEADER_REV6_FIX2_20261112"

using namespace ControlTableItem;
Dynamixel2Arduino dxl(DXL_SERIAL, -1);

// ======================= USER SETTINGS ==============================
constexpr uint8_t LOGICAL_JOINT_COUNT = 5;
constexpr uint8_t LEADER_MOTOR_COUNT = 5;
constexpr uint8_t DXL_IDS[LEADER_MOTOR_COUNT] = {1, 2, 3, 4, 5};
constexpr uint8_t JOINT2_INDEX = 1;
constexpr uint8_t GRIPPER_INDEX = 4;

// The Follower expects its logical joint-2 reference at 2048.
// Set the Leader raw zero to its reading at the SAME reference pose.
constexpr int32_t LEADER_J2_ZERO_RAW = 2048;
constexpr int32_t FOLLOWER_J2_REFERENCE = 2048;
constexpr int LEADER_J2_SIGN = +1;

// Keep the original Leader gripper delta convention (current position - 2048).
constexpr int32_t GRIPPER_REFERENCE_RAW = 2048;
constexpr int32_t GRIPPER_DELTA_LIMIT = 3000;
constexpr uint32_t SEND_INTERVAL_US = 25000; // original 25 ms / 40 Hz
constexpr uint32_t READ_TIMEOUT_MS = 5;     // per expected status packet
constexpr uint32_t CONFIG_TIMEOUT_MS = 100;
constexpr uint32_t TORQUE_OFF_INTERVAL_MS = 100;
constexpr size_t LINE_CAPACITY = 64;

// Startup homing is enabled by default, restoring the original boot behavior.
// Default targets are all 2048; ID2 and gripper follow their configured zeros.
constexpr bool HOME_AT_STARTUP = true;
constexpr int32_t HOME_TARGET_RAW[LEADER_MOTOR_COUNT] =
    {2048, LEADER_J2_ZERO_RAW, 2048, 2048, GRIPPER_REFERENCE_RAW};
constexpr uint32_t HOME_DURATION_MS = 3000; // original powered initial-move wait
constexpr uint32_t HOME_TICK_MS = 20;
constexpr int32_t HOME_PROFILE_VELOCITY = 50; // velocity-based profile, RAW
constexpr int32_t HOME_PROFILE_ACCELERATION = 20;
constexpr int32_t HOME_PWM_CAP_RAW = 440; // ~49.7%; obey a lower EEPROM limit
constexpr int32_t HOME_GRIP_CURRENT_RAW = 300; // ID5 ONLY, bounded by CURRENT_LIMIT

static_assert(LEADER_J2_SIGN == 1 || LEADER_J2_SIGN == -1, "Direction must be +1/-1");
static_assert(LEADER_J2_ZERO_RAW >= 0 && LEADER_J2_ZERO_RAW <= 4095, "Invalid ID2 zero");
static_assert(FOLLOWER_J2_REFERENCE == 2048, "Must match the supplied Follower");
static_assert(LOGICAL_JOINT_COUNT == 5, "Follower accepts FIVE values");
static_assert(HOME_DURATION_MS > 0, "Initial-move wait must be positive");

// Read torque, hardware status and position in the SAME Sync Read cycle.
// X-series addresses 64..135 (72 bytes), including Present Position at 132.
struct __attribute__((packed)) LeaderReadData {
  uint8_t torque;             // 64
  uint8_t reserved65to69[5];
  uint8_t hardwareError;      // 70
  uint8_t reserved71to131[61];
  int32_t position;           // 132..135
};
static_assert(sizeof(LeaderReadData) == 72, "Read layout size mismatch");
static_assert(offsetof(LeaderReadData, hardwareError) == 6, "Hardware status offset");
static_assert(offsetof(LeaderReadData, position) == 68, "Position offset");

LeaderReadData samples[LEADER_MOTOR_COUNT] = {};
DYNAMIXEL::InfoSyncReadInst_t readInfo = {};
DYNAMIXEL::XELInfoSyncRead_t readXels[LEADER_MOTOR_COUNT] = {};
uint8_t readPacketBuffer[128] = {};
bool ready = false, latched = false;
bool configurationIO = true, gripperAvailable = true;
uint8_t readMotorIndex[LEADER_MOTOR_COUNT] = {};
const char *statusReason = "starting";
uint8_t faultId = 0;
uint32_t sentFrames = 0, rejectedFrames = 0, busyFrames = 0;
bool homeActive = false;
uint32_t homeStartedMs = 0, homeLastTickMs = 0;
int32_t homeStart[LEADER_MOTOR_COUNT] = {};
int32_t homeGoals[LEADER_MOTOR_COUNT] = {};
DYNAMIXEL::InfoSyncWriteInst_t homeWriteInfo = {};
DYNAMIXEL::XELInfoSyncWrite_t homeWriteXels[LEADER_MOTOR_COUNT] = {};

struct IoFailure {
  bool valid;
  const char *operation;
  uint8_t id, item, status;
  uint16_t lib;
  int32_t value;
};
IoFailure lastIoFailure = {}, faultIoFailure = {};

bool motorEnabled(uint8_t index) { return index != GRIPPER_INDEX || gripperAvailable; }

void captureIoFailure(const char *operation, uint8_t id, uint8_t item, int32_t value) {
  lastIoFailure.valid = true;
  lastIoFailure.operation = operation;
  lastIoFailure.id = id; lastIoFailure.item = item; lastIoFailure.value = value;
  lastIoFailure.lib = dxl.getLastLibErrCode();
  lastIoFailure.status = dxl.getLastStatusPacketError();
}

void printIoFailure(const IoFailure &failure) {
  if (!failure.valid || !DEBUG_SERIAL) return;
  DEBUG_SERIAL.print("[DXL] ID="); DEBUG_SERIAL.print(failure.id);
  DEBUG_SERIAL.print(" op="); DEBUG_SERIAL.print(failure.operation);
  DEBUG_SERIAL.print(" item="); DEBUG_SERIAL.print(failure.item);
  DEBUG_SERIAL.print(" value="); DEBUG_SERIAL.print(failure.value);
  DEBUG_SERIAL.print(" lib="); DEBUG_SERIAL.print(failure.lib);
  DEBUG_SERIAL.print(" status=0x"); DEBUG_SERIAL.println(failure.status, HEX);
}

bool readItem(uint8_t item, uint8_t id, int32_t &value) {
  uint32_t timeout = configurationIO ? CONFIG_TIMEOUT_MS : READ_TIMEOUT_MS;
  bool received = true;
  if (item == BUS_WATCHDOG) {
    // D2A 0.8.1 lists this X-series register as 2 bytes; it is one byte.
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
  uint32_t timeout = configurationIO ? CONFIG_TIMEOUT_MS : READ_TIMEOUT_MS;
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

void settleUnacknowledgedWrite() {
  DXL_SERIAL.flush();
  delay(2);
  uint16_t budget = 256;
  while (budget-- > 0 && DXL_SERIAL.available()) DXL_SERIAL.read();
}

bool ensureStatusReturnLevel(uint8_t id) {
  int32_t level;
  if (readItem(STATUS_RETURN_LEVEL, id, level) && level == 2) return true;
  uint8_t fullStatus = 2;
  if (!dxl.writeNoResp(id, 68, &fullStatus, 1)) {
    captureIoFailure("writeNoResp", id, STATUS_RETURN_LEVEL, 2); return false;
  }
  settleUnacknowledgedWrite();
  if (!readItem(STATUS_RETURN_LEVEL, id, level)) return false;
  if (level != 2) {
    captureIoFailure("readback", id, STATUS_RETURN_LEVEL, level); return false;
  }
  return true;
}
bool setIfChanged(uint8_t item, uint8_t id, int32_t value) {
  int32_t oldValue;
  return readItem(item, id, oldValue) &&
         (oldValue == value || writeItem(item, id, value));
}

void torqueOffLeader() {
  uint8_t off = 0;
  DYNAMIXEL::InfoSyncWriteInst_t info = {};
  DYNAMIXEL::XELInfoSyncWrite_t xels[LEADER_MOTOR_COUNT] = {};
  info.addr = 64; info.addr_length = 1;
  info.p_xels = xels; info.xel_count = LEADER_MOTOR_COUNT;
  info.is_info_changed = true;
  for (uint8_t i = 0; i < LEADER_MOTOR_COUNT; ++i) {
    xels[i].id = DXL_IDS[i]; xels[i].p_data = &off;
  }
  dxl.syncWrite(&info); // repeated in passive mode and on every latched fault
}

void latchLeader(const char *reason, uint8_t id) {
  ready = false; latched = true; homeActive = false;
  faultIoFailure = lastIoFailure;
  statusReason = reason; faultId = id;
  torqueOffLeader();
  digitalWrite(LED_BUILTIN, LOW);
}

void prepareHomeWrite() {
  homeWriteInfo.addr = 116; homeWriteInfo.addr_length = 4;
  homeWriteInfo.p_xels = homeWriteXels;
  homeWriteInfo.xel_count = 0;
  homeWriteInfo.packet.is_completed = false;
  homeWriteInfo.is_info_changed = true;
  for (uint8_t i = 0; i < LEADER_MOTOR_COUNT; ++i) {
    if (!motorEnabled(i)) continue;
    uint8_t slot = homeWriteInfo.xel_count++;
    homeWriteXels[slot].id = DXL_IDS[i];
    homeWriteXels[slot].p_data = (uint8_t *)&homeGoals[i];
  }
}

void disableGripper(const char *reason) {
  if (!gripperAvailable) return;
  if (DEBUG_SERIAL) {
    DEBUG_SERIAL.print("[GRIP DISABLED] Leader ID5: "); DEBUG_SERIAL.println(reason);
    printIoFailure(lastIoFailure);
  }
  uint8_t off = 0;
  dxl.writeNoResp(DXL_IDS[GRIPPER_INDEX], 64, &off, 1);
  settleUnacknowledgedWrite();
  gripperAvailable = false;
  prepareSyncRead();
  prepareHomeWrite();
  lastIoFailure.valid = false;
}

bool configureMotor(uint8_t i) {
    uint8_t id = DXL_IDS[i];
    faultId = id;
    if (!dxl.ping(id)) {
      captureIoFailure("ping", id, MODEL_NUMBER, 0);
      statusReason = "missing Leader motor"; return false;
    }
    if (!ensureStatusReturnLevel(id)) { statusReason = "Leader response level setup failed"; return false; }
    uint16_t model = dxl.getModelNumber(id);
    const DYNAMIXEL::ControlTableItemInfo_t pos =
        DYNAMIXEL::getControlTableItemInfo(model, PRESENT_POSITION);
    const DYNAMIXEL::ControlTableItemInfo_t torque =
        DYNAMIXEL::getControlTableItemInfo(model, TORQUE_ENABLE);
    const DYNAMIXEL::ControlTableItemInfo_t error =
        DYNAMIXEL::getControlTableItemInfo(model, HARDWARE_ERROR_STATUS);
    if (pos.addr != 132 || pos.addr_length != 4 || torque.addr != 64 ||
        torque.addr_length != 1 || error.addr != 70 || error.addr_length != 1) {
      statusReason = "unsupported Leader control table"; return false;
    }
    int32_t hardwareError, drive;
    if (!writeItem(TORQUE_ENABLE, id, 0) || !readItem(HARDWARE_ERROR_STATUS, id, hardwareError)) {
      statusReason = "Leader torque/status setup failed"; return false;
    }
    if (hardwareError != 0) { statusReason = "Leader hardware error"; return false; }
    if (!writeItem(BUS_WATCHDOG, id, 0) ||
        !setIfChanged(OPERATING_MODE, id, i == GRIPPER_INDEX ? OP_CURRENT_BASED_POSITION : OP_POSITION) ||
        !readItem(DRIVE_MODE, id, drive)) {
      statusReason = "Leader mode setup failed"; return false;
    }
    drive &= ~0x0C; // velocity-based profile, no torque-on-by-goal
    if (i == JOINT2_INDEX) drive = 0;
    if (!setIfChanged(DRIVE_MODE, id, drive)) {
      statusReason = "Leader direction setup failed"; return false;
    }
    int32_t torqueEnabled;
    if (!readItem(TORQUE_ENABLE, id, torqueEnabled) || torqueEnabled != 0) {
      statusReason = "Leader torque is not OFF"; return false;
    }
  return true;
}

bool configureLeader() {
  for (uint8_t i = 0; i < LEADER_MOTOR_COUNT; ++i) {
    if (!configureMotor(i)) {
      if (i != GRIPPER_INDEX) return false;
      disableGripper(statusReason);
    }
  }
  faultId = 0;
  return true;
}

void prepareSyncRead() {
  readInfo.packet.p_buf = readPacketBuffer;
  readInfo.packet.buf_capacity = sizeof(readPacketBuffer);
  readInfo.packet.is_completed = false;
  readInfo.addr = 64;
  readInfo.addr_length = sizeof(LeaderReadData);
  readInfo.p_xels = readXels;
  readInfo.xel_count = 0;
  readInfo.is_info_changed = true;
  for (uint8_t i = 0; i < LEADER_MOTOR_COUNT; ++i) {
    if (!motorEnabled(i)) continue;
    uint8_t slot = readInfo.xel_count++;
    readMotorIndex[slot] = i;
    readXels[slot].id = DXL_IDS[i];
    readXels[slot].p_recv_buf = (uint8_t *)&samples[i];
  }
}

bool readPoseWithTorque(int32_t *raw, uint8_t expectedTorque) {
  // A missing or short response must not reuse a previous joint value.
  raw[GRIPPER_INDEX] = GRIPPER_REFERENCE_RAW; // absent ID5 -> fifth CSV value 0
  uint8_t expectedCount = readInfo.xel_count;
  for (uint8_t slot = 0; slot < expectedCount; ++slot) {
    uint8_t i = readMotorIndex[slot];
    memset(&samples[i], 0xFF, sizeof(samples[i]));
    samples[i].position = INT32_MIN;
    readXels[slot].error = 0xFF;
  }
  uint8_t receivedCount = dxl.syncRead(&readInfo,
      configurationIO ? CONFIG_TIMEOUT_MS : READ_TIMEOUT_MS);
  uint16_t readError = dxl.getLastLibErrCode();
  bool droppedOptional = false;
  for (uint8_t slot = 0; slot < expectedCount; ++slot) {
    uint8_t i = readMotorIndex[slot];
    bool missing = samples[i].position == INT32_MIN || readXels[slot].error == 0xFF;
    bool statusBad = readXels[slot].error != 0 || samples[i].hardwareError != 0;
    bool torqueBad = samples[i].torque != expectedTorque;
    bool homeRangeBad = expectedTorque == 1 &&
        (samples[i].position < 0 || samples[i].position > 4095);
    if (i == GRIPPER_INDEX && (missing || statusBad || torqueBad || homeRangeBad)) {
      captureIoFailure("syncRead", DXL_IDS[i], PRESENT_POSITION, samples[i].position);
      disableGripper(missing ? "missing position response" :
          (homeRangeBad ? "gripper homing position range" : "gripper status/torque error"));
      droppedOptional = true;
      continue;
    }
    if (missing) {
      captureIoFailure("syncRead", DXL_IDS[i], PRESENT_POSITION, samples[i].position);
      faultId = DXL_IDS[i]; statusReason = "incomplete required position; not sending";
      return false;
    }
    if (statusBad) {
      captureIoFailure("syncRead", DXL_IDS[i], HARDWARE_ERROR_STATUS, samples[i].hardwareError);
      latchLeader("Leader hardware/status error", DXL_IDS[i]); return false;
    }
    if (torqueBad) {
      captureIoFailure("syncRead", DXL_IDS[i], TORQUE_ENABLE, samples[i].torque);
      latchLeader("unexpected Leader torque state", DXL_IDS[i]); return false;
    }
    raw[i] = samples[i].position;
  }
  if (receivedCount < GRIPPER_INDEX ||
      (!droppedOptional && (receivedCount != expectedCount || readError != DXL_LIB_OK))) {
    statusReason = "incomplete Sync Read; not sending"; return false;
  }
  lastIoFailure.valid = false;
  faultId = 0;
  return true;
}

bool readFreshPose(int32_t *raw) {
  return readPoseWithTorque(raw, 0);
}

bool startupPoseInRange(const int32_t *raw) {
  for (uint8_t i = 0; i < LEADER_MOTOR_COUNT; ++i)
    if (motorEnabled(i) && (raw[i] < 0 || raw[i] > 4095)) return false;
  return true;
}

bool writeStartupGoals() {
  homeWriteInfo.is_info_changed = true;
  return dxl.syncWrite(&homeWriteInfo);
}

bool configureStartupMotor(uint8_t i, const int32_t *raw) {
  uint8_t id = DXL_IDS[i];
  faultId = id;
  int32_t pwmLimit, velocityLimit;
  if (HOME_TARGET_RAW[i] < 0 || HOME_TARGET_RAW[i] > 4095) {
    statusReason = "invalid home target"; return false;
  }
  if (i != GRIPPER_INDEX) {
    int32_t minimum, maximum;
    if (!readItem(MIN_POSITION_LIMIT, id, minimum) ||
        !readItem(MAX_POSITION_LIMIT, id, maximum) || minimum > maximum ||
        raw[i] < minimum || raw[i] > maximum ||
        HOME_TARGET_RAW[i] < minimum || HOME_TARGET_RAW[i] > maximum) {
      statusReason = "home pose outside hardware position limits"; return false;
    }
  }
  if (!readItem(PWM_LIMIT, id, pwmLimit) || pwmLimit <= 0 ||
      !readItem(VELOCITY_LIMIT, id, velocityLimit) || velocityLimit <= 0) {
    statusReason = "invalid homing PWM/velocity limit"; return false;
  }
  int32_t pwm = pwmLimit < HOME_PWM_CAP_RAW ? pwmLimit : HOME_PWM_CAP_RAW;
  int32_t velocity = velocityLimit < HOME_PROFILE_VELOCITY ? velocityLimit : HOME_PROFILE_VELOCITY;
  if (!writeItem(GOAL_PWM, id, pwm) ||
      !writeItem(PROFILE_VELOCITY, id, velocity) ||
      !writeItem(PROFILE_ACCELERATION, id, HOME_PROFILE_ACCELERATION)) {
    statusReason = "homing profile setup failed"; return false;
  }
  if (i == GRIPPER_INDEX) {
    int32_t currentLimit;
    if (!readItem(CURRENT_LIMIT, id, currentLimit) || currentLimit <= 0) {
      statusReason = "invalid gripper current limit"; return false;
    }
    int32_t current = currentLimit < HOME_GRIP_CURRENT_RAW ? currentLimit : HOME_GRIP_CURRENT_RAW;
    if (!writeItem(GOAL_CURRENT, id, current)) {
      statusReason = "gripper homing current setup failed"; return false;
    }
  }

  return true;
}

bool enableStartupMotor(uint8_t i) {
  faultId = DXL_IDS[i];
  int32_t current;
  // Re-read each joint immediately before enabling it. Other setup writes
  // must not leave us using an old pose after an unsupported arm has moved.
  if (!readItem(PRESENT_POSITION, DXL_IDS[i], current) || current < 0 || current > 4095) {
    statusReason = "homing pre-enable position read failed"; return false;
  }
  homeStart[i] = homeGoals[i] = current;
  if (!writeItem(GOAL_POSITION, DXL_IDS[i], current) ||
      !writeItem(BUS_WATCHDOG, DXL_IDS[i], 10)) {
    statusReason = "homing hold/watchdog setup failed"; return false;
  }
  if (!writeItem(TORQUE_ENABLE, DXL_IDS[i], 1)) {
    statusReason = "homing torque enable failed"; faultId = DXL_IDS[i]; return false;
  }

  return true;
}

bool beginStartupHome() {
  ready = false;
  int32_t raw[LEADER_MOTOR_COUNT] = {};
  if (!readFreshPose(raw)) {
    if (!latched) statusReason = "invalid starting pose for homing";
    return false;
  }
  if (gripperAvailable && (raw[GRIPPER_INDEX] < 0 || raw[GRIPPER_INDEX] > 4095))
    disableGripper("gripper starting position outside homing range");
  if (!startupPoseInRange(raw)) {
    statusReason = "invalid starting pose for homing"; return false;
  }
  // Finish validation/configuration with torque OFF before enabling any ID.
  for (uint8_t i = 0; i < LEADER_MOTOR_COUNT; ++i) {
    if (!motorEnabled(i)) continue;
    if (!configureStartupMotor(i, raw)) {
      if (i != GRIPPER_INDEX) return false;
      disableGripper(statusReason);
    }
  }
  prepareHomeWrite();
  for (uint8_t i = 0; i < LEADER_MOTOR_COUNT; ++i) {
    if (!motorEnabled(i)) continue;
    if (!enableStartupMotor(i)) {
      if (i != GRIPPER_INDEX) return false;
      disableGripper(statusReason);
    }
  }
  if (!readPoseWithTorque(raw, 1) || !startupPoseInRange(raw)) {
    if (!latched) statusReason = "homing torque/pose verification failed";
    return false;
  }
  // The motor's velocity/acceleration profile performs the slow move.
  // Preserve the original fixed wait; do not gate streaming on exact arrival.
  for (uint8_t i = 0; i < LEADER_MOTOR_COUNT; ++i)
    if (motorEnabled(i)) homeGoals[i] = HOME_TARGET_RAW[i];
  if (!writeStartupGoals()) {
    statusReason = "initial goal write failed"; faultId = 0; return false;
  }
  homeStartedMs = homeLastTickMs = millis();
  homeActive = true; faultId = 0;
  statusReason = "initial 2048 move; Bluetooth paused for 3 seconds";
  if (DEBUG_SERIAL) DEBUG_SERIAL.println("[HOME] 2048 command sent. Waiting 3 seconds, then torque OFF.");
  return true;
}

bool finishStartupHome() {
  torqueOffLeader();
  for (uint8_t i = 0; i < LEADER_MOTOR_COUNT; ++i) {
    if (!motorEnabled(i)) continue;
    int32_t torque = -1;
    if (!writeItem(TORQUE_ENABLE, DXL_IDS[i], 0) ||
        !readItem(TORQUE_ENABLE, DXL_IDS[i], torque) || torque != 0 ||
        !writeItem(BUS_WATCHDOG, DXL_IDS[i], 0)) {
      if (!lastIoFailure.valid) captureIoFailure("readback", DXL_IDS[i], TORQUE_ENABLE, torque);
      if (i == GRIPPER_INDEX) { disableGripper("homing torque OFF verification failed"); continue; }
      latchLeader("homing torque OFF verification failed", DXL_IDS[i]); return false;
    }
  }
  homeActive = false; ready = true;
  statusReason = "initial move completed; passive Leader streaming enabled";
  if (DEBUG_SERIAL) DEBUG_SERIAL.println("[HOME] Initial move interval completed. Torque OFF. Streaming measured positions.");
  return true;
}

void serviceStartupHome() {
  if (!homeActive || latched) return;
  uint32_t now = millis();
  uint32_t elapsed = now - homeStartedMs;
  if ((uint32_t)(now - homeLastTickMs) < HOME_TICK_MS) return;
  homeLastTickMs = now;
  int32_t raw[LEADER_MOTOR_COUNT] = {};
  if (!readPoseWithTorque(raw, 1) || !startupPoseInRange(raw)) {
    if (!latched) latchLeader("homing status/position read failed", faultId);
    return;
  }
  if (elapsed < HOME_DURATION_MS) return;
  // Endpoint error is diagnostic only. Send fresh measurements after torque
  // OFF; never substitute the commanded 2048 or reuse this homing sample.
  if (DEBUG_SERIAL) {
    for (uint8_t i = 0; i < LEADER_MOTOR_COUNT; ++i) {
      if (!motorEnabled(i)) continue;
      DEBUG_SERIAL.print("[HOME POS] ID="); DEBUG_SERIAL.print(DXL_IDS[i]);
      DEBUG_SERIAL.print(" actual="); DEBUG_SERIAL.print(raw[i]);
      DEBUG_SERIAL.print(" target="); DEBUG_SERIAL.print(HOME_TARGET_RAW[i]);
      DEBUG_SERIAL.print(" error="); DEBUG_SERIAL.println(raw[i] - HOME_TARGET_RAW[i]);
    }
  }
  finishStartupHome();
}

bool makeFollowerValues(const int32_t *raw, int32_t *values) {
  for (uint8_t i = 0; i < LEADER_MOTOR_COUNT; ++i) {
    if (i != GRIPPER_INDEX && (raw[i] < 0 || raw[i] > 4095)) return false;
  }
  int32_t joint2 = FOLLOWER_J2_REFERENCE + LEADER_J2_SIGN * (raw[JOINT2_INDEX] - LEADER_J2_ZERO_RAW);
  if (joint2 < 0 || joint2 > 4095) return false;
  int64_t gripperDelta = gripperAvailable ? (int64_t)raw[GRIPPER_INDEX] - GRIPPER_REFERENCE_RAW : 0;
  if (gripperDelta < -GRIPPER_DELTA_LIMIT || gripperDelta > GRIPPER_DELTA_LIMIT) {
    disableGripper("gripper delta outside valid transmission range"); gripperDelta = 0;
  }
  for (uint8_t i = 0; i < LOGICAL_JOINT_COUNT; ++i) values[i] = raw[i];
  values[JOINT2_INDEX] = joint2;
  values[GRIPPER_INDEX] = (int32_t)gripperDelta;
  return true;
}

int formatFollowerLine(const int32_t *values, char *line, size_t capacity) {
  int length = snprintf(line, capacity, "%ld,%ld,%ld,%ld,%ld\n",
      (long)values[0], (long)values[1], (long)values[2], (long)values[3], (long)values[4]);
  if (length <= 0 || (size_t)length >= capacity) return -1;
  return length;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  DEBUG_SERIAL.begin(57600); // original USB monitor baudrate
  if (DEBUG_SERIAL) DEBUG_SERIAL.println("[BUILD] " FIRMWARE_LABEL);
  BLT_SERIAL.begin(57600);
  pinMode(31, OUTPUT); digitalWrite(31, HIGH);
  delay(300);
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(2.0);
  torqueOffLeader();
  if (!configureLeader()) { latchLeader(statusReason, faultId); return; }
  prepareSyncRead();
  if (HOME_AT_STARTUP) {
    if (!beginStartupHome() && !latched) latchLeader(statusReason, faultId);
  } else {
    ready = true;
    statusReason = "ready: powered homing disabled";
  }
  configurationIO = false;
  digitalWrite(LED_BUILTIN, latched ? LOW : HIGH);
}

void loop() {
  static uint32_t lastOffMs = 0, lastAttemptUs = 0, lastStatusMs = 0;
  uint32_t nowMs = millis();
  if (!homeActive && (uint32_t)(nowMs - lastOffMs) >= TORQUE_OFF_INTERVAL_MS) {
    lastOffMs = nowMs; torqueOffLeader();
  }
  uint8_t usbBudget = 16;
  while (usbBudget-- > 0 && DEBUG_SERIAL.available()) {
    if (DEBUG_SERIAL.read() == '!') latchLeader("USB stop: transmission latched OFF", 0);
  }
  if (DEBUG_SERIAL && (uint32_t)(nowMs - lastStatusMs) >= 1000) {
    lastStatusMs = nowMs;
    DEBUG_SERIAL.print("[LEADER] sent="); DEBUG_SERIAL.print(sentFrames);
    DEBUG_SERIAL.print(" rejected="); DEBUG_SERIAL.print(rejectedFrames);
    DEBUG_SERIAL.print(" busy="); DEBUG_SERIAL.print(busyFrames);
    DEBUG_SERIAL.print(" ID="); DEBUG_SERIAL.print(faultId);
    DEBUG_SERIAL.print(" grip="); DEBUG_SERIAL.print(gripperAvailable ? "enabled" : "absent/disabled");
    DEBUG_SERIAL.print(" build="); DEBUG_SERIAL.print(FIRMWARE_LABEL);
    DEBUG_SERIAL.print(" "); DEBUG_SERIAL.println(statusReason);
    if (latched) printIoFailure(faultIoFailure);
    else printIoFailure(lastIoFailure);
  }
  if (homeActive) { serviceStartupHome(); return; }
  if (!ready || latched) return;
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastAttemptUs) < SEND_INTERVAL_US) return;
  lastAttemptUs = nowUs;
  int32_t raw[LEADER_MOTOR_COUNT] = {};
  int32_t values[LOGICAL_JOINT_COUNT] = {};
  if (!readFreshPose(raw)) { ++rejectedFrames; digitalWrite(LED_BUILTIN, LOW); return; }
  if (!makeFollowerValues(raw, values)) {
    ++rejectedFrames; statusReason = "position/gripper delta invalid; not sending";
    digitalWrite(LED_BUILTIN, LOW); return;
  }
  char line[LINE_CAPACITY];
  int length = formatFollowerLine(values, line, sizeof(line));
  if (length < 0) { latchLeader("CSV buffer failure", 0); return; }
  // Whole frames fit the TX ring. Skip instead of accumulating old poses.
  if (BLT_SERIAL.availableForWrite() < length) { ++busyFrames; return; }
  if (BLT_SERIAL.write((const uint8_t *)line, (size_t)length) != (size_t)length) {
    latchLeader("Bluetooth TX failure", 0); return;
  }
  if (DEBUG_SERIAL && DEBUG_SERIAL.availableForWrite() >= length)
    DEBUG_SERIAL.write((const uint8_t *)line, (size_t)length);
  ++sentFrames; statusReason = "streaming FIVE fields";
  digitalWrite(LED_BUILTIN, HIGH);
}
