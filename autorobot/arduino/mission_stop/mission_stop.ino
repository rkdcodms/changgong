/*
  Mission-stop firmware (MS1), derived from vision_stop.ino (VC1 5-second test).
  Difference from vision_stop.ino — everything else is kept byte-for-byte:
   * A heartbeat only binds the session. The robot stays braked in WAIT_VISION
     until the Jetson sends an explicit GO (@VC1,G,...). It never starts merely
     because a camera/serial link became healthy.
   * A vision stop (S) or host hold (P) brakes and HOLDS. There is no 5-second
     auto-restart (AUTO_RESUME_MS = 0). Only an explicit GO resumes. The GO is
     sent by the Jetson mission controller after its departure condition; the
     firmware does not know whether crew transfer is complete.
   * @ID1,MS1,<build> is printed every second so the Jetson can refuse to run
     the mission program against the old 5-second firmware.
   * GAP_COAST_MS (default 0 = original behaviour) can bridge short dark gaps
     such as a dark tape crossing the white line. Enable only after field test.
  Heartbeat loss (1500ms) still latches FAULT; physical RESET is required.
  The original vision_stop.ino header follows.
  line_tracer_ver2 — Arduino Mega 2560 + L298N, 흰색 선 추종
  ver2: 전체 이동 명령에 SPEED_SCALE=0.60 적용 (PWM 목표 약 40% 감소).
  같은 조향 계산에서 좌우 목표 비율을 유지하도록 제한 후 함께 감속한다.
  실제 속도와 PWM은 정확히 비례하지 않으며, 주행궤적 불변을 보장하지 않는다.
  저속에서 모터가 멈칫하면 SPEED_SCALE을 0.70~0.80으로 높여 시험한다.
  제동, 3회 연속 정지 판정, 5초 정지는 감속 배율을 적용하지 않는다.
  센서 원시값은 주행 중에도 100ms 간격으로 출력한다 (115200 baud).
  센서: 로봇 전진 방향 기준 오른쪽 A0 -> 왼쪽 A7
  센서 위치 가중치: +4,+3,+2,+1,-1,-2,-3,-4.
  analogRead <= 500: 흰색 / > 500: 검정

  사용자 배선: ENA=5, ENB=6, IN1=30, IN2=31, IN3=32, IN4=33.
  L298N의 ENA/ENB 점퍼를 제거하고 PWM 핀에 연결한다.
  Arduino, 센서, 모터 드라이버의 GND를 공통으로 연결한다.
  다른 드라이버를 쓰면 writeMotor()/brakeNow()를 수정해야 한다.

  FOLLOW: PD + 코너 감속 + PWM 변화 제한
  STOPPED: 유효한 카메라 정지 명령을 받으면 제동, 5초 유지
  EXIT_BAR: 저속 직진, 일반 선이 80ms 연속 보이면 정지 감지 재개
  SEARCH: 마지막 선 방향으로 완만히 돌고, 실패하면 제자리 탐색
  FAULT: 탐색/정지선 탈출 실패 또는 heartbeat 끊김이면 제동, RESET 필요.
  WAIT_VISION: 최초 heartbeat 대기 중 제동.

  센서가 차축보다 5cm 앞에 있어 코너를 미리 감지한다.
  5cm만큼 무조건 전진하거나 고정 시간 동안 90도 회전하지 않는다.
  코너 통과는 센서 피드백으로 판단한다. 날카로운 직각에서는 탐색
  회전이 필요할 수 있으며, 차체/센서 간격/마찰에 맞게 튜닝해야 한다.
  바퀴 지름 6.5cm, 좌우 바퀴 중심 사이 거리 19cm를 사용한다.
  0.448초/회전이 바퀴의 회전주기라면 PWM 180에서 약 45.6cm/s.
  탐색 시간은 이 값과 감속 후 PWM-속도의 비례 근사로 설정한다. 부하, 마찰,
  저속 모터 정지구간 때문에 오차가 있으므로 정확한 거리/각도 제어는 아니다.
  최초 측정부터 모두 흰색이면 출발선으로 보고 먼저 빠져나온다.
  이 시험에서는 흰색 바 자체로 5초 정지하지 않고 EXIT_BAR로 통과한다.
  주행 중에는 10ms 간격으로 측정한다.
  아래 PWM, PD 상수, 탐색시간은 실측 최적값이 아닌 시작값이다.
*/

#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// ===================== 사용자 설정 =====================
const uint8_t SENSOR_PINS[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
const int8_t SENSOR_WEIGHTS[8] = {4, 3, 2, 1, -1, -2, -3, -4};
const int THRESHOLD = 500;

// L298N 채널 A = 왼쪽 모터, 채널 B = 오른쪽 모터
const byte ENA = 5;
const byte ENB = 6;
const byte IN1 = 30;
const byte IN2 = 31;
const byte IN3 = 32;
const byte IN4 = 33;
const bool LEFT_REVERSED = false;   // 전진 명령인데 역회전하면 true
const bool RIGHT_REVERSED = false;

// true이면 모터를 제동한 채 센서값만 출력한다 (115200 baud).
const bool SENSOR_TEST_ONLY = false;
const bool PRINT_SENSOR_VALUES = true;
const uint32_t SENSOR_PRINT_MS = 100;
// 첫 측정부터 8개 모두 흰색이면 출발선 탈출 후 정지 감지 시작.
// 전원을 켠 위치에서도 정지 판정이 필요하면 false로 변경한다.
const bool IGNORE_INITIAL_WHITE = true;
const bool STOP_ON_WHITE_BAR = false; // camera-triggered stop test

// 모든 모드의 이동 출력을 drive()에서 한 번만 줄인다.
// 아래 PWM 상수는 감속 전 값이다. 개별적으로0.6배하면 이중 감속된다.
constexpr float SPEED_SCALE = 0.60f;
static_assert(SPEED_SCALE > 0.0f && SPEED_SCALE <= 1.0f,
              "SPEED_SCALE must be greater than 0 and at most 1");
const int STRAIGHT_PWM = 145;       // 감속 후 직선 목표는87
const int CURVE_PWM = 85;           // 감속 후 큰 편차에서 기본 목표는51
const int MAX_PWM = 180;            // 감속 전 상한, 최종 이동 출력 상한은108
const int EXIT_PWM = 110;           // 감속 후 정지선 탈출 목표는66
const int SEARCH_OUTER_PWM = 115;   // 감속 후 원호 바깥쪽 목표는69
const int SEARCH_INNER_PWM = 25;    // 감속 후 원호 안쪽 목표는15 (실제 회전은 모터에 의존)
const int PIVOT_PWM = 100;          // 감속 후 제자리 탐색 목표는 ±60

// 위치 편차 범위 -4(왼쪽) ~ +4(오른쪽).
// 가중치 변경에 따른 조향 반응을 볼 수 있도록 PD 게인은 기존 값을 유지한다.
const float KP = 18.0f;
const float KD = 0.12f;             // 미분항은 초 단위 dt로 계산
// 기존처럼 A1/A6(두 번째 바깥 센서)까지 치우치면 기본 PWM을85로 낮춘다.
const float CORNER_ERROR_SCALE = 3.0f;
const float DERIVATIVE_TAU_S = 0.035f;
const float PWM_SLEW_PER_S = 700.0f; // drive()에서 변화량도 SPEED_SCALE만큼 축소

const float WHEEL_DIAMETER_CM = 6.5f;
const float WHEEL_TRACK_CM = 19.0f; // 좌우 바퀴 중심 사이 거리
const float SENSOR_AHEAD_CM = 5.0f;
const float REFERENCE_PWM = 180.0f;
const float REFERENCE_REV_TIME_S = 0.448f;
const float PI_F = 3.14159265f;
const float NOMINAL_CM_PER_S = PI_F * WHEEL_DIAMETER_CM / REFERENCE_REV_TIME_S;
// 아래 두 값은 탐색시간의 초기 추정에만 사용한다.
const float ARC_CM_PER_S = NOMINAL_CM_PER_S *
    ((SEARCH_OUTER_PWM + SEARCH_INNER_PWM) / 2.0f) * SPEED_SCALE / REFERENCE_PWM;
const float PIVOT_WHEEL_CM_PER_S =
    NOMINAL_CM_PER_S * PIVOT_PWM * SPEED_SCALE / REFERENCE_PWM;

const uint32_t CONTROL_MS = 10;     // 100Hz; 3회 판정의 첫째~셋째 간격은 20ms
const uint8_t STOP_CONFIRM_SAMPLES = 3;
// STOP_MS (5 s) of vision_stop.ino is replaced by AUTO_RESUME_MS below.
const uint32_t CLEAR_STABLE_MS = 80;
// 이동 관련 제한시간만 감속 배율에 맞춰 늘린다. 판정시간/5초 정지는 그대로다.
const uint32_t EXIT_TIMEOUT_MS = (uint32_t)(2000.0f / SPEED_SCALE + 0.5f);
const uint32_t UNKNOWN_SEARCH_MS = (uint32_t)(100.0f / SPEED_SCALE + 0.5f);
const uint32_t SEARCH_RAMP_MARGIN_MS = (uint32_t)(400.0f / SPEED_SCALE + 0.5f);
// 선 이탈 시 약 5cm의 주행거리에 해당하는 원호 탐색 (0.60에서 약470ms).
const uint32_t SEARCH_ARC_MS = (uint32_t)(1000.0f * SENSOR_AHEAD_CM / ARC_CM_PER_S + 0.5f);
// 제자리 90도 회전의 이론 시간 (0.60에서 약982ms). 실제 회전은 센서로 종료.
const uint32_t NOMINAL_PIVOT_90_MS =
    (uint32_t)(1000.0f * PI_F * WHEEL_TRACK_CM / (4.0f * PIVOT_WHEEL_CM_PER_S) + 0.5f);
// 최대 탐색시간: 원호 + 이론 180도 회전 + 가감속 여유 (0.60에서 약3.1초).
const uint32_t SEARCH_TIMEOUT_MS =
    SEARCH_ARC_MS + 2 * NOMINAL_PIVOT_90_MS + SEARCH_RAMP_MARGIN_MS;
const uint32_t REACQUIRE_MS = 15;
// ======================================================

enum Mode { FOLLOW, STOPPED, EXIT_BAR, SEARCH, FAULT, WAIT_VISION };

// ---- MS1 mission settings ----
#define MISSION_BUILD "MS1-20261112a"
// 0 = hold until GO. 5000 reproduces vision_stop.ino (test only, never contest).
#ifndef MS1_AUTO_RESUME_MS
#define MS1_AUTO_RESUME_MS 0
#endif
const uint32_t AUTO_RESUME_MS = MS1_AUTO_RESUME_MS;
// GO is ignored until the robot has been braked at least this long.
const uint32_t MIN_STOP_MS = 300;
// 0 = original: any all-dark sample in FOLLOW enters SEARCH immediately.
// >0: keep the current (slew-limited) command for up to this long when the
// line vanishes while it was centred, then fall back to SEARCH as before.
#ifndef MS1_GAP_COAST_MS
#define MS1_GAP_COAST_MS 0
#endif
const uint32_t GAP_COAST_MS = MS1_GAP_COAST_MS;
const float GAP_COAST_MAX_ERROR = 1.5f;
const uint32_t ID_PRINT_MS = 1000;
uint32_t lastIdPrint = 0;
uint32_t gapSince = 0;
bool gapActive = false;
void changeMode(Mode next, uint32_t now);
Mode mode = WAIT_VISION;

bool white[8];
int sensorValues[8];
uint8_t whiteCount = 0;
uint8_t allWhiteStreak = 0;
bool startupChecked = false;
float error = 0.0f;
float previousError = 0.0f, filteredDerivative = 0.0f;
bool pdReady = false;
int8_t lastDirection = 0;           // +1 오른쪽, -1 왼쪽, 0 미확인
float leftCommand = 0.0f, rightCommand = 0.0f;
uint32_t stateSince = 0, lastControl = 0;
uint32_t stableSince = 0;
bool stableActive = false;
uint32_t lastSensorPrint = 0;

// Existing mode values remain 0..4; WAIT_VISION is 5.
const uint32_t STATE_PRINT_MS = 100;
uint32_t stopSequence = 0, lastStatePrint = 0;
int8_t lastReportedMode = -1;


// @VC1,H,<session> / @VC1,S,<session>,<event>,<class>, LF terminated.
// The first heartbeat fixes the nonzero session until hardware RESET.
// Events must increase within that session; replayed IDs never restart a stop.
const uint32_t VISION_TIMEOUT_MS = 1500;
const uint32_t RX_PARTIAL_TIMEOUT_MS = 250;
const uint8_t RX_BUDGET_PER_LOOP = 32;
uint32_t visionSession = 0, lastHeartbeat = 0, lastAcceptedEvent = 0;
int8_t lastVisionClass = -1;
uint8_t stopReason = 0; // 0 none, 1 white bar, 2 vision, 3 heartbeat loss, 4 host hold
char visionRx[63]; // at most 62 characters + LF = 63 wire bytes
uint8_t visionRxLength = 0;
bool visionRxDropping = false;
uint32_t visionRxAt = 0;
uint32_t lastVisionPrint = 0, reportedSession = 0, reportedEvent = 0;
int8_t reportedClass = -2, reportedReason = -1, reportedLive = -1;

void brakeNow();
void resumeFromStop(uint32_t now);

bool visionLive(uint32_t now) {
  return visionSession != 0 && (uint32_t)(now - lastHeartbeat) < VISION_TIMEOUT_MS;
}

void printVisionState(uint32_t now) {
  const int live = visionLive(now) ? 1 : 0;
  if ((uint32_t)(now - lastVisionPrint) < STATE_PRINT_MS &&
      reportedSession == visionSession && reportedEvent == lastAcceptedEvent &&
      reportedClass == lastVisionClass && reportedReason == stopReason && reportedLive == live) return;
  char line[48];
  int length = snprintf(line, sizeof(line), "@V1,%lu,%lu,%d,%u,%u\n",
                        (unsigned long)visionSession, (unsigned long)lastAcceptedEvent,
                        (int)lastVisionClass, (unsigned)stopReason, (unsigned)live);
  if (length > 0 && length < (int)sizeof(line) && Serial.availableForWrite() >= length) {
    Serial.write((const uint8_t*)line, (size_t)length);
    lastVisionPrint = now; reportedSession = visionSession; reportedEvent = lastAcceptedEvent;
    reportedClass = lastVisionClass; reportedReason = stopReason; reportedLive = live;
  }
}

bool parseU32(const char* value, uint32_t &out) {
  if (!*value) return false;
  uint32_t result = 0;
  uint8_t digits = 0;
  for (const char* p = value; *p; ++p) {
    if (*p < '0' || *p > '9' || ++digits > 10) return false;
    uint8_t digit = (uint8_t)(*p - '0');
    if (result > (UINT32_MAX - digit) / 10U) return false;
    result = result * 10U + digit;
  }
  out = result;
  return true;
}

void handleVisionLine(char* line, uint32_t now) {
  char* fields[5] = {line, 0, 0, 0, 0};
  uint8_t count = 1;
  for (char* p = line; *p; ++p) {
    if (*p == ',') {
      if (count == 5) return;
      *p = '\0'; fields[count++] = p + 1;
    }
  }
  if (strcmp(fields[0], "@VC1") != 0) return;
  uint32_t session = 0;
  if (count < 3 || !parseU32(fields[2], session) || session == 0) return;
  if (count == 3 && strcmp(fields[1], "H") == 0) {
    if (visionSession == 0) {
      if (mode != WAIT_VISION) return;
      visionSession = session; lastHeartbeat = now;
      stopReason = 0; // MS1: stay braked in WAIT_VISION until GO
    } else if (session == visionSession) {
      lastHeartbeat = now; // refresh may report link live, but never clears FAULT
    }
    return;
  }
  if (session != visionSession || !visionLive(now) || SENSOR_TEST_ONLY) return;
  if (count == 4 && (strcmp(fields[1], "G") == 0 || strcmp(fields[1], "P") == 0)) {
    uint32_t hostEvent = 0;
    if (!parseU32(fields[3], hostEvent) || hostEvent == 0 || hostEvent <= lastAcceptedEvent) return;
    if (fields[1][0] == 'G') {
      if (mode == WAIT_VISION) {
        lastAcceptedEvent = hostEvent;
        changeMode(FOLLOW, now);   // startupChecked still handles an all-white start
      } else if (mode == STOPPED && (uint32_t)(now - stateSince) >= MIN_STOP_MS &&
                 stopReason != 3) {
        lastAcceptedEvent = hostEvent;
        resumeFromStop(now);
      }
      return;                      // otherwise ignored; Jetson times out and faults
    }
    if (mode != FOLLOW && mode != SEARCH && mode != EXIT_BAR) return;
    lastAcceptedEvent = hostEvent; stopReason = 4;
    brakeNow();
    changeMode(STOPPED, now);
    digitalWrite(LED_BUILTIN, HIGH);
    return;
  }
  if (count != 5 || strcmp(fields[1], "S") != 0) return;
  uint32_t event = 0, classId = 0;
  if (!parseU32(fields[3], event) || event == 0 || event <= lastAcceptedEvent ||
      !parseU32(fields[4], classId) || classId > 7) return;
  if (mode != FOLLOW && mode != SEARCH && mode != EXIT_BAR) return;
  lastAcceptedEvent = event; lastVisionClass = (int8_t)classId; stopReason = 2;
  brakeNow();
  changeMode(STOPPED, now);
  digitalWrite(LED_BUILTIN, HIGH);
}

void pollVisionCommands(uint32_t now) {
  if ((visionRxLength || visionRxDropping) &&
      (uint32_t)(now - visionRxAt) >= RX_PARTIAL_TIMEOUT_MS) {
    visionRxLength = 0; visionRxDropping = false;
  }
  for (uint8_t consumed = 0; consumed < RX_BUDGET_PER_LOOP && Serial.available() > 0; ++consumed) {
    const int value = Serial.read();
    if (value < 0) break;
    const char c = (char)value;
    visionRxAt = now;
    if (c == '\n') {
      if (!visionRxDropping && visionRxLength) {
        visionRx[visionRxLength] = '\0'; handleVisionLine(visionRx, now);
      }
      visionRxLength = 0; visionRxDropping = false;
    } else if (!visionRxDropping) {
      if (value < 32 || value > 126 || visionRxLength >= sizeof(visionRx) - 1) {
        visionRxLength = 0; visionRxDropping = true;
      } else visionRx[visionRxLength++] = c;
    }
  }
}

void checkVisionTimeout(uint32_t now) {
  if (visionSession != 0 && !visionLive(now) && mode != FAULT) {
    stopReason = 3;
    brakeNow();
    changeMode(FAULT, now);
    digitalWrite(LED_BUILTIN, HIGH);
  }
}

void printMissionId(uint32_t now) {
  if (lastIdPrint != 0 && (uint32_t)(now - lastIdPrint) < ID_PRINT_MS) return;
  const char line[] = "@ID1,MS1," MISSION_BUILD "\n";
  if (Serial.availableForWrite() >= (int)(sizeof(line) - 1)) {
    Serial.write((const uint8_t*)line, sizeof(line) - 1);
    lastIdPrint = now ? now : 1;
  }
}

void printJetsonState(uint32_t now) {
  if (lastReportedMode == (int8_t)mode &&
      (uint32_t)(now - lastStatePrint) < STATE_PRINT_MS) return;
  // Maximum row is 42 bytes, including LF, even with three UINT32_MAX values.
  char line[64];
  int length = snprintf(line, sizeof(line), "@LT1,%u,%lu,%lu,%lu,%u\n",
                        (unsigned int)mode, (unsigned long)stopSequence,
                        (unsigned long)(uint32_t)(now - stateSince),
                        (unsigned long)now, SENSOR_TEST_ONLY ? 1U : 0U);
  if (length > 0 && length < (int)sizeof(line) && Serial.availableForWrite() >= length) {
    Serial.write((const uint8_t*)line, (size_t)length);
    lastStatePrint = now;
    lastReportedMode = (int8_t)mode;
  }
}

float limitFloat(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

void resetPD() {
  pdReady = false;
  filteredDerivative = 0.0f;
}

void readSensors() {
  int sum = 0;
  whiteCount = 0;
  for (uint8_t i = 0; i < 8; ++i) {
    sensorValues[i] = analogRead(SENSOR_PINS[i]);
    white[i] = sensorValues[i] <= THRESHOLD;
    if (white[i]) {
      ++whiteCount;
      sum += SENSOR_WEIGHTS[i];   // A0:+4, A1:+3, ... A7:-4
    }
  }
  if (whiteCount > 0) error = (float)sum / whiteCount;
}

void printSensors(uint32_t now) {
  if (!PRINT_SENSOR_VALUES && !SENSOR_TEST_ONLY) return;
  if ((uint32_t)(now - lastSensorPrint) < SENSOR_PRINT_MS) return;

  // 버퍼가 부족하면 그 회차는 건너뛰어 제어 루프가 출력 때문에 기다리지 않게 한다.
  // A0(오른쪽)부터 A7(왼쪽)까지, 위에서 읽은 원시값을 그대로 사용한다.
  char line[48];
  int length = snprintf(line, sizeof(line), "%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
                        sensorValues[0], sensorValues[1], sensorValues[2], sensorValues[3],
                        sensorValues[4], sensorValues[5], sensorValues[6], sensorValues[7]);
  if (length > 0 && length < (int)sizeof(line) && Serial.availableForWrite() >= length) {
    Serial.write((const uint8_t*)line, (size_t)length);
    lastSensorPrint = now;
  }
}

// L298N: EN=HIGH, IN1=IN2=LOW -> 동적 제동.
// brakeNow()는 관성 없는 순간 정지를 보장하지 않는다.
void writeMotor(uint8_t en, uint8_t inA, uint8_t inB,
                int pwm, bool reversed) {
  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);
  if (reversed) pwm = -pwm;
  analogWrite(en, 0);             // 방향 핀 변경 중 드라이버 비활성
  digitalWrite(inA, pwm > 0 ? HIGH : LOW);
  digitalWrite(inB, pwm < 0 ? HIGH : LOW);
  analogWrite(en, pwm == 0 ? 255 : abs(pwm));
}

void brakeNow() {
  leftCommand = rightCommand = 0.0f;
  writeMotor(ENA, IN1, IN2, 0, LEFT_REVERSED);
  writeMotor(ENB, IN3, IN4, 0, RIGHT_REVERSED);
}

float ramp(float current, float target, float step) {
  return current + limitFloat(target - current, -step, step);
}

void drive(float left, float right, float dt) {
  // 전체 모드 공통 감속: 기존 제한을 적용한 뒤 좌우 모두 같은 배율을 곱한다.
  // FOLLOW/EXIT_BAR/SEARCH 모두 이 함수를 사용하며, 다른 곳에서는 재차 곱하지 않는다.
  const float leftTarget = limitFloat(left, -MAX_PWM, MAX_PWM) * SPEED_SCALE;
  const float rightTarget = limitFloat(right, -MAX_PWM, MAX_PWM) * SPEED_SCALE;
  const float step = PWM_SLEW_PER_S * SPEED_SCALE * dt;
  leftCommand = ramp(leftCommand, leftTarget, step);
  rightCommand = ramp(rightCommand, rightTarget, step);
  writeMotor(ENA, IN1, IN2, (int)roundf(leftCommand), LEFT_REVERSED);
  writeMotor(ENB, IN3, IN4, (int)roundf(rightCommand), RIGHT_REVERSED);
}

void followLine(float dt) {
  float derivative = pdReady ? (error - previousError) / dt : 0.0f;
  float alpha = dt / (DERIVATIVE_TAU_S + dt);
  filteredDerivative += alpha * (derivative - filteredDerivative);
  previousError = error;
  pdReady = true;

  if (error > 0.7f) lastDirection = 1;
  else if (error < -0.7f) lastDirection = -1;

  float turn = KP * error + KD * filteredDerivative;
  float severity = limitFloat(fabsf(error) / CORNER_ERROR_SCALE, 0.0f, 1.0f);
  float base = STRAIGHT_PWM - (STRAIGHT_PWM - CURVE_PWM) * severity;
  turn = limitFloat(turn, -base, base); // 일반 추종에서는 안쪽 바퀴 역회전 금지

  // 선이 오른쪽(error>0)이면 왼쪽 바퀴를 빠르게, 오른쪽을 느리게.
  drive(limitFloat(base + turn, 0, MAX_PWM),
        limitFloat(base - turn, 0, MAX_PWM), dt);
}

void changeMode(Mode next, uint32_t now) {
  if (next == STOPPED && mode != STOPPED) ++stopSequence;
  mode = next;
  stateSince = now;
  gapActive = false;
  stableActive = false;
  allWhiteStreak = 0;
  resetPD();
  printJetsonState(now);
}

void resumeFromStop(uint32_t now) {
  const bool visionStop = stopReason == 2 || stopReason == 4;
  stopReason = 0;
  if (visionStop) {
    // Use current sensors. Preserve the last turn direction if the line is lost.
    changeMode(whiteCount == 8 ? EXIT_BAR : (whiteCount == 0 ? SEARCH : FOLLOW), now);
  } else {
    changeMode(EXIT_BAR, now);
    lastDirection = 0;
  }
  digitalWrite(LED_BUILTIN, LOW);
}

bool heldFor(bool condition, uint32_t now, uint32_t duration) {
  if (!condition) {
    stableActive = false;
    return false;
  }
  if (!stableActive) {
    stableActive = true;
    stableSince = now;
  }
  return (uint32_t)(now - stableSince) >= duration;
}

void failStop() {
  mode = FAULT;
  brakeNow();
  digitalWrite(LED_BUILTIN, HIGH);
  const char message[] = "FAULT: reposition robot, then press RESET\n";
  if (Serial.availableForWrite() >= (int)(sizeof(message) - 1))
    Serial.write((const uint8_t*)message, sizeof(message) - 1);
}

void searchLine(uint32_t now, float dt) {
  // 넓은 흰색 영역보다 중앙에 돌아온 일반 선을 우선 확인한다.
  bool centered = (white[3] || white[4]) && !white[0] && !white[7];
  if (heldFor(centered, now, REACQUIRE_MS)) {
    changeMode(FOLLOW, now);
    followLine(dt);
    return;
  }

  uint32_t elapsed = (uint32_t)(now - stateSince);
  if (elapsed >= SEARCH_TIMEOUT_MS) {
    failStop();
    return;
  }
  if (lastDirection == 0) {
    // 방향을 모르면 짧은 틈만 저속 직진으로 확인한다.
    if (elapsed >= UNKNOWN_SEARCH_MS) failStop();
    else drive(CURVE_PWM, CURVE_PWM, dt);
    return;
  }

  int outer = elapsed < SEARCH_ARC_MS ? SEARCH_OUTER_PWM : PIVOT_PWM;
  int inner = elapsed < SEARCH_ARC_MS ? SEARCH_INNER_PWM : -PIVOT_PWM;
  if (lastDirection > 0) drive(outer, inner, dt);
  else drive(inner, outer, dt);
}

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  for (uint8_t i = 0; i < 8; ++i) pinMode(SENSOR_PINS[i], INPUT);
  Serial.begin(115200);
  if (PRINT_SENSOR_VALUES || SENSOR_TEST_ONLY) {
    Serial.println(F("A0\tA1\tA2\tA3\tA4\tA5\tA6\tA7"));
  }
  brakeNow();
  lastControl = stateSince = millis();
  printJetsonState(lastControl);
  printVisionState(lastControl);
  printMissionId(lastControl);
}

void loop() {
  uint32_t now = millis();
  checkVisionTimeout(now); // do not let an overdue buffered heartbeat revive motion
  pollVisionCommands(now);
  printVisionState(now);
  printJetsonState(now);
  printMissionId(now);
  if (SENSOR_TEST_ONLY) {
    if ((uint32_t)(now - lastControl) < CONTROL_MS) return;
    lastControl = now;
    readSensors();
    printJetsonState(millis());
    printSensors(millis());
    return;
  }

  uint32_t elapsed = (uint32_t)(now - lastControl);
  if (elapsed < CONTROL_MS) return;
  lastControl = now;
  float dt = elapsed * 0.001f;
  readSensors();
  now = millis();                 // 8개 센서 측정 후 시각
  printJetsonState(now);
  printSensors(now);

  if (mode == FAULT || mode == WAIT_VISION) return;

  // 출발선 위에서 전원을 켜도 정지 판정을 누적하지 않고 먼저 빠져나온다.
  if (!startupChecked && mode != STOPPED) {
    startupChecked = true;
    if (IGNORE_INITIAL_WHITE && whiteCount == 8) changeMode(EXIT_BAR, now);
  }

  if (mode == FOLLOW || mode == SEARCH) {
    if (whiteCount == 8) {
      if (!STOP_ON_WHITE_BAR) {
        changeMode(EXIT_BAR, now);
        drive(EXIT_PWM, EXIT_PWM, dt);
        return;
      }
      if (allWhiteStreak < STOP_CONFIRM_SAMPLES) ++allWhiteStreak;
      if (allWhiteStreak >= STOP_CONFIRM_SAMPLES) {
        stopReason = 1;
        brakeNow();              // 세 번째 확인 즉시 제동, 여기서부터 5초
        changeMode(STOPPED, millis());
        digitalWrite(LED_BUILTIN, HIGH);
      } else {
        // 모두 흰색일 때는 선의 위치를 알 수 없으므로 기존 PWM을 유지.
        // 확인 전 감속하면 얇은 선에 머무는 시간이 길어질 수 있다.
        stableActive = false;
        resetPD();
      }
      return;
    }
    // 하나라도 검정이면 연속 횟수를 초기화한다. 누적 합계가 아니다.
    allWhiteStreak = 0;
  } else {
    allWhiteStreak = 0;
  }

  if (mode == STOPPED) {
    // MS1: hold until GO unless the 5-second test behaviour is compiled in.
    if (AUTO_RESUME_MS == 0 || (uint32_t)(now - stateSince) < AUTO_RESUME_MS) return;
    resumeFromStop(now);
  }

  if (mode == EXIT_BAR) {
    // 사진처럼 직선 위의 정지선을 통과할 때 사용한다.
    // 단순 타이머 차단이 아니라 정상 선으로 돌아온 것을 확인한다.
    bool normalLine = (white[3] || white[4]) && !white[0] && !white[7];
    if (heldFor(normalLine, now, CLEAR_STABLE_MS)) {
      changeMode(FOLLOW, now);
      followLine(dt);
    } else if ((uint32_t)(now - stateSince) >= EXIT_TIMEOUT_MS) {
      failStop();
    } else {
      drive(EXIT_PWM, EXIT_PWM, dt);
    }
    return;
  }

  if (mode == FOLLOW && whiteCount == 0) {
    if (GAP_COAST_MS > 0 && fabsf(error) <= GAP_COAST_MAX_ERROR) {
      if (!gapActive) { gapActive = true; gapSince = now; }
      if ((uint32_t)(now - gapSince) < GAP_COAST_MS) {
        drive(leftCommand / SPEED_SCALE, rightCommand / SPEED_SCALE, dt);
        return;                    // error/PD keep their last line-based values
      }
    }
    gapActive = false;
    changeMode(SEARCH, now);
  } else if (whiteCount > 0) gapActive = false;
  if (mode == SEARCH) searchLine(now, dt);
  else followLine(dt);
}
