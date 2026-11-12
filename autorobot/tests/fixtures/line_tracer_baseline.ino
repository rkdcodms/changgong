/*
  Jetson observation-only addition: @LT1 state telemetry, no command receiver.
  Original pins, control, search, automatic start and 5-second restart are unchanged.
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
  STOPPED: 8개 모두 흰색인 측정이 3회 연속이면 제동, 5초 유지
  EXIT_BAR: 저속 직진, 일반 선이 80ms 연속 보이면 정지 감지 재개
  SEARCH: 마지막 선 방향으로 완만히 돌고, 실패하면 제자리 탐색
  FAULT: 탐색/정지선 탈출 실패 시 제동. 위치를 바로잡고 RESET.

  센서가 차축보다 5cm 앞에 있어 코너를 미리 감지한다.
  5cm만큼 무조건 전진하거나 고정 시간 동안 90도 회전하지 않는다.
  코너 통과는 센서 피드백으로 판단한다. 날카로운 직각에서는 탐색
  회전이 필요할 수 있으며, 차체/센서 간격/마찰에 맞게 튜닝해야 한다.
  바퀴 지름 6.5cm, 좌우 바퀴 중심 사이 거리 19cm를 사용한다.
  0.448초/회전이 바퀴의 회전주기라면 PWM 180에서 약 45.6cm/s.
  탐색 시간은 이 값과 감속 후 PWM-속도의 비례 근사로 설정한다. 부하, 마찰,
  저속 모터 정지구간 때문에 오차가 있으므로 정확한 거리/각도 제어는 아니다.
  최초 측정부터 모두 흰색이면 출발선으로 보고 먼저 빠져나온다.
  이후 일반 선을 확인하면 3회 연속 정지 판정을 활성화한다.
  주행 중에는 10ms 간격으로 측정한다. 1~2회만 모두 흰색이면 통과.
  얇은 선도 저속에서 3회 연속 감지되면 정지할 수 있다.
  아래 PWM, PD 상수, 탐색시간은 실측 최적값이 아닌 시작값이다.
*/

#include <Arduino.h>
#include <math.h>
#include <stdio.h>

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
const uint32_t STOP_MS = 5000;
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

enum Mode { FOLLOW, STOPPED, EXIT_BAR, SEARCH, FAULT };
void changeMode(Mode next, uint32_t now);
Mode mode = FOLLOW;

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

// Observation only: mode values remain FOLLOW=0, STOPPED=1, EXIT_BAR=2, SEARCH=3, FAULT=4.
const uint32_t STATE_PRINT_MS = 100;
uint32_t stopSequence = 0, lastStatePrint = 0;
int8_t lastReportedMode = -1;

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
  stableActive = false;
  allWhiteStreak = 0;
  resetPD();
  printJetsonState(now);
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
  Serial.println(F("FAULT: reposition robot, then press RESET"));
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
  lastControl = millis();
}

void loop() {
  uint32_t now = millis();
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

  if (mode == FAULT) return;

  // 출발선 위에서 전원을 켜도 정지 판정을 누적하지 않고 먼저 빠져나온다.
  if (!startupChecked) {
    startupChecked = true;
    if (IGNORE_INITIAL_WHITE && whiteCount == 8) changeMode(EXIT_BAR, now);
  }

  if (mode == FOLLOW || mode == SEARCH) {
    if (whiteCount == 8) {
      if (allWhiteStreak < STOP_CONFIRM_SAMPLES) ++allWhiteStreak;
      if (allWhiteStreak >= STOP_CONFIRM_SAMPLES) {
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
    if ((uint32_t)(now - stateSince) < STOP_MS) return;
    changeMode(EXIT_BAR, now);
    lastDirection = 0;
    digitalWrite(LED_BUILTIN, LOW);
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

  if (mode == FOLLOW && whiteCount == 0) changeMode(SEARCH, now);
  if (mode == SEARCH) searchLine(now, dt);
  else followLine(dt);
}
