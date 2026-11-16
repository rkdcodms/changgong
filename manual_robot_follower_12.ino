// OpenRB-150: PPM wheels + BT-210 follower arm + dual XL430 joint 2
// + 트럭 개방(CH8) + 자동 투입(CH7)   [Follower_Grip_Track 팔 코드 기반으로 합침]
//
// [주행] 오른쪽 스틱 상하(CH2) = 앞뒤, 왼쪽 스틱 좌우(CH4) = 좌우 회전.
//        최대속도 225 + 모터 Velocity Limit 자동 반영, 대각선 입력 시 비율 축소 (휘는 문제 해결)
// [CH7]  왼쪽 뒤 버튼: 팔 세우기 → ID1 뒤로 → 통 위에서 놓기 → 세우기 → ID1 정면 → Leader 자세로 복귀
//        동작 중 다시 누르면 중단하고 Leader 자세로 복귀
// [CH8]  오른쪽 뒤 버튼: 누르고 있으면 트럭 열림, 떼면 닫힘 (RC 끊기면 닫힘)
// [복구] 팔 모터 하나가 Overload/토크 꺼짐 → 그 모터만 2초 뒤 재부팅해서 복구 (어깨는 ID2+ID6 같이)
//        나머지 모터·주행은 계속 동작. 전체 정지는 어깨 정렬/위치한계/온도/통신 이상일 때만
// [그리퍼] 부팅 때 끝까지 확 닫아서 그 위치를 '닫힘'으로 기억 → 거기서 3684만큼이 '열림'
//          Leader 그리퍼 전체 범위 ↔ Follower 전체 범위 비례. 물체를 쥐고 막히면 힘을 줄여 Overload 방지
// 아래 원본 설명은 팔(Follower) 부분에 해당
//
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
#define FIRMWARE_LABEL "FOLLOWER_GRIP_ZERO_WRAP_20261112+GRIP_HOME+DRIVE_DUMP_AUTO+ANY_POSE_BOOT+SLOW_CATCHUP+FAST_DUMP_OPEN+REVIVE_ANY_POSE+WHEEL_TOLERANT+POWER_CYCLE+RX_DRAIN+PAIR_LIFT_BOOST+HOME_HANDOFF+REVIVE_1S+WHEEL_DEATH_DETECT"

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
constexpr int32_t GRIP_NO_SAMPLE = INT32_MIN; // fifth field only: keep the last gripper target
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
// 11/13 실측 범위: ID2·ID6 1350~3300 (q = -698..1252). 코드가 EEPROM 위치 한계도 이 값으로 맞춤
constexpr int32_t PAIR_2_MIN = 1350;
constexpr int32_t PAIR_2_MAX = 3300;
constexpr int32_t PAIR_6_MIN = 1350;
constexpr int32_t PAIR_6_MAX = 3300;

// Original limits for the OTHER joints are preserved.
//                         ID1  ID2          ID3   ID4   ID5  ID6
// 11/13 실측: ID3 1196~3432, ID4 761~3390 (EEPROM 위치 한계도 이 값으로 맞춤)
constexpr int32_t JOINT_MIN[ARM_MOTOR_COUNT] =
    {0, PAIR_2_MIN, 1196, 761, -4095, PAIR_6_MIN};
constexpr int32_t JOINT_MAX[ARM_MOTOR_COUNT] =
    {4095, PAIR_2_MAX, 3432, 3390, 8190, PAIR_6_MAX};
constexpr uint16_t ARM_PROFILE_VELOCITY = 20;
constexpr uint16_t ARM_PROFILE_ACCELERATION = 10;
constexpr uint32_t ARM_CONTROL_MS = 20;
constexpr uint32_t ARM_LINK_TIMEOUT_MS = 500;
constexpr int32_t PAIR_STEP_RAW = 6; // initial move: 6 pulses per 20 ms
constexpr int32_t FOLLOW_PROFILE_VELOCITY = 240;
constexpr int32_t FOLLOW_PROFILE_ACCELERATION = 60;
constexpr int32_t PAIR_FOLLOW_STEP_RAW = 60; // live tracking: 12 pulses per 20 ms
// 목표가 멀 때 천천히 따라잡기 (관절별). 20 ms마다 최대 이동량(raw, 4096 = 1바퀴)
constexpr int32_t NORMAL_STEP_RAW  = 60;  // 평소 추종: 60/20ms ≈ 264°/s (리더 손동작 그대로)
constexpr int32_t CATCHUP_STEP_RAW = 5;   // 멀 때: 5/20ms ≈ 22°/s
constexpr int32_t CATCHUP_ENTER_RAW = 250; // 목표와 이만큼(≈22°) 넘게 떨어지면 느린 모드
constexpr int32_t CATCHUP_EXIT_RAW  = 20;  // 이만큼(≈2°) 안으로 들어오면 평소 속도로 복귀
constexpr int32_t BOOT_LIMIT_MARGIN = 20;  // 범위 밖에서 켰을 때 EEPROM 한계를 현재 위치 + 여유까지 임시로 넓힘

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
constexpr uint32_t HOME_LAG_HANDOFF_MS = 3000;
constexpr uint32_t REVIVE_WAIT_MS = 1000;   // 모터가 죽은 뒤 재부팅까지 기다리는 시간 (관절·어깨·트럭 공통)

// XL430 has NO current sensor/current control. Use PWM and inferred load.
// These are commissioning starting values, not verified load ratings.
// 어깨 최대 출력. 원래 440(~50%)이었는데 새 그리퍼로 팔이 무거워져 부하가 상한(49%)에 붙어
// 팔을 못 들고 'stalled/following error'로 전체 정지 → 750(~85%)로 올림. 885 = 100%
// (두 모터가 서로 비트는 상황은 정렬 감시 PAIR_SYNC_TRIP_RAW가 계속 막아 줌)
constexpr int32_t PAIR_PWM_CAP_RAW = 750;
// 처진 자세에서 들어 올릴 때(복구 직후·부팅 홈 이동)만 어깨 출력 100%. 목표 근처에 실제로 도착하면 원래 값으로
constexpr int32_t PAIR_LIFT_PWM_RAW = 885;
constexpr int32_t PAIR_LIFT_DONE_RAW = 40;      // 실제 위치가 목표와 이만큼(≈3.5°) 안이면 도착
constexpr uint32_t PAIR_LIFT_MAX_MS = 10000;    // 10초 넘게 못 올라가도 원래 출력으로 (과열 방지)
constexpr int32_t PAIR_LOAD_TRIP_RAW = 800; // 경고 출력 기준 |Present Load| (0.1% 단위)
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

// ---------- 그리퍼 (ID5, XC330, Current-based Position) ----------
constexpr uint16_t GRIPPER_GOAL_CURRENT = 400; // 쥐는 힘 상한 (RAW). 700이면 오래 쥘 때 Overload 위험
constexpr uint16_t GRIP_HOME_CURRENT = 500;    // 부팅 때 닫는 힘. 끝까지 안 닫히면 올리기
constexpr int32_t GRIP_HOME_VELOCITY = 200;    // 부팅 때 닫는 속도 (Profile Velocity, 확 닫기)
constexpr uint32_t GRIP_HOME_TIMEOUT_MS = 4000;
constexpr int32_t GRIP_SPAN = 3684;            // 완전 닫힘 → 완전 열림 거리
constexpr int GRIP_CLOSE_DIR = +1;             // 닫힐 때 위치값 증가
// Leader 그리퍼 범위: 3072(닫힘) ~ 1024(열림). Leader가 보내는 값 = 현재 - 2048
// → +1024(닫힘) ~ -1024(열림)을 Follower 닫힘 ~ 닫힘-3684에 '고정 비례'로 대응
constexpr int32_t LEADER_GRIP_CLOSED_RAW = 3072;
constexpr int32_t LEADER_GRIP_OPEN_RAW   = 1024;
// 쥐고 있을 때 Overload 방지: 닫으라는 명령보다 GRIP_STALL_ERR 이상 덜 닫힌 채로
// GRIP_STALL_MS 동안 멈춰 있으면(= 물체에 막힘) 목표를 '현재 위치 + GRIP_HOLD_SQUEEZE'로 낮춤
constexpr int32_t GRIP_STALL_ERR = 80;
constexpr uint32_t GRIP_STALL_MS = 300;
constexpr int32_t GRIP_HOLD_SQUEEZE = 60;      // 막힌 뒤 더 미는 양 (클수록 세게 쥠)
constexpr int LEADER_GRIP_SIGN = +1;
constexpr int32_t GRIP_CLOSE_MARGIN = 0;
constexpr int32_t GRIP_OPEN_MARGIN = 20;

// ---------- RC 채널 (received[] 인덱스 = CH번호 - 1) ----------
constexpr uint8_t CH_DRIVE_V = 1;   // CH2 오른쪽 스틱 상하 (앞 2000) = 앞뒤
constexpr uint8_t CH_DRIVE_W = 3;   // CH4 왼쪽 스틱 좌우 (우 2000) = 좌우 회전
constexpr uint8_t CH_AUTO    = 6;   // CH7 왼쪽 뒤 버튼 (누름 ≈2000)
constexpr uint8_t CH_DUMP    = 7;   // CH8 오른쪽 뒤 버튼 (누름 ≈2000)

// ---------- 주행 ----------
constexpr uint8_t PPM_PIN = 0;
constexpr uint8_t LEFT_ID = 9;
constexpr uint8_t RIGHT_ID = 10;
constexpr int LEFT_SIGN = -1;       // 앞뒤·좌우 모두 반대였음 → 두 바퀴 부호 반전
constexpr int RIGHT_SIGN = 1;
constexpr int FORWARD_SIGN = 1;     // 앞으로 밀었는데 후진하면 -1
constexpr int TURN_SIGN = 1;        // 오른쪽으로 밀었는데 반시계로 돌면 -1
constexpr int TURN_CENTER = 1500;
constexpr int FWD_CENTER = 1500;
constexpr int DEADZONE = 40;
// XL430 실제 최고속도(11.1 V 무부하 ≈ 249) 근처까지 쓰면 두 모터가 포화돼 경로가 휨 → 225
constexpr int MAX_FWD_SPEED = 225;
constexpr int MAX_TURN_SPEED = 112; // 병진의 절반
constexpr uint32_t PPM_TIMEOUT_US = 150000;

// ---------- 트럭 개방 (CH8) ----------
constexpr uint8_t  DUMP_ID         = 7;      // TODO: 트럭 모터 ID 확인
constexpr int32_t  DUMP_CLOSED_POS = 1662;
constexpr int32_t  DUMP_OPEN_POS   = 722;
constexpr uint16_t DUMP_MOVE_TIME  = 500;    // Time-based일 때 열고 닫는 시간 [ms]
constexpr uint16_t DUMP_MOVE_VEL   = 60;     // Velocity-based일 때 속도
constexpr uint16_t DUMP_P_GAIN     = 1200;   // 덮개 무게로 목표 직전에서 멈추면 올리기 (0 = 기본값)
constexpr uint16_t DUMP_I_GAIN     = 0;      // 기어 유격 안에서 헌팅 생겨서 끔
// 트럭 모터가 문을 누르다 Overload → 순간 큰 전류로 전압이 떨어져 다른 모터까지 멈추던 문제 대책
constexpr uint16_t DUMP_OPEN_PWM   = 885;    // 열 때: 최대 출력 100% + 속도·가속 제한 없음 (최대한 빠르게)
constexpr uint16_t DUMP_PWM_CAP    = 620;    // 최대 출력 ~70% 로 제한 (885 = 100%). 문이 안 닫히면 올리기
constexpr uint8_t  DUMP_SHUTDOWN   = 0x14;   // 과열·전기충격 때만 꺼짐 (Overload로는 안 꺼짐, 과열 보호는 유지)

// ---------- 자동 투입 (CH7) ----------
// 놓는 위치 (도면 좌표, mm): x 옆, y 정면 +, z 차체 윗면 위 +
constexpr float DROP_X = -40.0f;     // 차체 구조 때문에 옆으로 (x: 왼쪽 +, 즉 -40 = 오른쪽 → ID1 시계 방향)
constexpr float DROP_Y = -80.0f;
constexpr float DROP_Z = 240.0f;     // 200은 새 관절 범위(ID2 1350~)에서 한계에 딱 붙어서 불가 → 240 (여유 47 tick)
                                    // 대안: y=-100, z=200 (여유 51 tick)
constexpr int32_t AUTO_LIMIT_MARGIN = 30;  // 놓는 자세는 관절 한계에서 이만큼 이상 떨어져야 함
constexpr float AUTO_Y_MIN = -135.0f;     // 팔 어느 부분도 이보다 뒤로(y 더 작게) 가면 안 됨
constexpr float GRIP_TIP_EXTRA = 30.0f;   // 잡는 지점에서 그리퍼 끝까지 추가 길이 (여유)
constexpr float AUTO_SPEED_DEG_S = 120.0f;   // 갈 때 관절 속도 (빠르면 pair 부하 감시에 걸릴 수 있음)
constexpr float AUTO_RETURN_SPEED_DEG_S = 50.0f;  // 마지막에 Leader 자세로 내려갈 때 속도 (차체 흔들림 방지)
constexpr uint32_t AUTO_SETTLE_MS = 200;     // 놓기 전 자세 안정 대기
constexpr uint32_t RELEASE_WAIT_MS = 300;    // 그리퍼 연 뒤 대기
// 팔 치수 (2048 자세: ID2 (0,68), ID3 (25,180), ID4 (157,180), 잡는 지점 (365.64,151))
constexpr float SH_Z  = 68.0f;
constexpr float L1    = 114.7563f;
constexpr float A1_0  = 1.3511822f;          // atan2(112, 25)
constexpr float L2    = 132.0f;
constexpr float L3    = 210.6458f;
constexpr float DELTA = -0.1381105f;         // atan2(-29, 208.64)
constexpr float C_MAX = 0.995f;
// tick이 커지는 방향 (11/13 실측): ID1 = 반시계로 돌 때 증가 → +1
//   ID2·ID6 = 숙일 때 증가 → 젖히면 감소 → -1,  ID3·ID4 = 숙일 때 감소 → 젖히면 증가 → +1
constexpr int FOLLOWER_JOINT_DIR[4] = { 1, -1, 1, 1 };
constexpr float DEG_RAD = 0.01745329252f;

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
uint32_t faultAtMs = 0;
constexpr uint32_t AUTO_RESTART_DELAY_MS = 3000;   // 잠금 상태가 되면 이 시간 뒤 보드 전체 자동 재시작
uint8_t faultId = 0;
bool armed = false, pairSampleOK = false;
uint8_t neutralCount = 0, pairCommMisses = 0;
uint16_t dbgTurn = 0, dbgFwd = 0;
uint32_t armPacketCount = 0, armRejectCount = 0, armWriteCount = 0;
uint32_t lastArmPacketMs = 0;
bool armLinkActive = false;
bool armHomeActive = false;
int32_t qMin = 0, qMax = 0, qCommand = 0, qTarget = 0;
bool pairLiftBoost = false;
bool homeHandoff = false;   // 홈 이동이 중간에 넘겨졌으면 armWant를 덮어쓰지 않음
uint32_t pairLiftStartMs = 0;
int32_t pairFollowStep = PAIR_STEP_RAW;
int32_t targetGoal[ARM_MOTOR_COUNT] = {};   // 실제로 모터에 보내는 목표
int32_t armWant[ARM_MOTOR_COUNT] = {};      // ID1/3/4: 리더·자동이 원하는 목표 (속도 제한 거쳐 targetGoal로)
bool    jointSlow[4] = {};                  // 0=ID1, 1=어깨(2+6), 2=ID3, 3=ID4 느린 따라잡기 모드
int32_t bootMin[ARM_MOTOR_COUNT] = {}, bootMax[ARM_MOTOR_COUNT] = {};
int32_t gripClosedPos = 0;          // 부팅 때 찾은 완전 닫힘 위치 (모터 RAW)
int32_t gripDesired = 0;            // Leader가 원하는 그리퍼 목표
bool gripHoldActive = false;        // 물체에 막혀서 힘 줄인 상태
int32_t gripHoldPos = 0;
int32_t leaderGripClosed = 0, leaderGripOpen = 0;
bool leaderGripSeen = false;
int32_t leaderGripHist[3] = {};
uint8_t leaderGripHistN = 0;
uint32_t loadSince = 0, syncSince = 0, trackSince = 0;
bool loadTiming = false, syncTiming = false, trackTiming = false;

// 주행·트럭·자동 투입 상태
int wheelMax = MAX_FWD_SPEED;     // min(MAX_FWD_SPEED, 바퀴 Velocity Limit)
bool dumpReady = false, dumpOpen = false;
bool dumpTimeBased = false;
int32_t dumpPwmLimit = 885;
enum AutoState : uint8_t { AUTO_IDLE, AUTO_UP, AUTO_TURN, AUTO_DROP, AUTO_SETTLE, AUTO_RELEASE,
                           AUTO_UP2, AUTO_TURN2, AUTO_RETURN };
AutoState autoState = AUTO_IDLE;
float tStandFront[4], tStandBack[4], tDrop[4];   // 관절 tick: ID1, ID2(pair), ID3, ID4
bool dropPoseOK = false;
float autoFrom[4], autoTo[4], autoProgress = 0, autoDur = 1;
bool autoToLeader = false;
uint32_t autoWaitStart = 0;
int32_t leaderVals[LEADER_JOINT_COUNT] = {};
bool leaderValsSeen = false;
bool gripLatchOpen = false;       // 자동 투입 후: Leader 그리퍼를 절반 이상 열 때까지 열린 채 유지

// 개별 모터 복구 상태 (index = ID). 어깨는 pairRev로 둘을 같이
enum RevState : uint8_t { RV_NONE, RV_WAIT, RV_BOOT, RV_EASE };
RevState revState[11] = {};
uint32_t revT0[11] = {};
uint8_t revTries[11] = {};
uint8_t revSilentCycles[11] = {};   // 재부팅해도 대답이 없는 복구 사이클 수
uint8_t pairSilentCycles = 0;
// 재부팅 명령으로도 안 살아나는(핑에도 대답 없는) 모터는 전원을 껐다 켜야만 살아남.
// 이 횟수만큼 복구 사이클(약 4.6 s)이 연속 실패하면 DXL 전원을 1초 끊었다가 보드 전체 재시작.
constexpr uint8_t SILENT_CYCLES_FOR_POWER_CYCLE = 2;
constexpr uint32_t DXL_POWER_OFF_MS = 1000;
RevState pairRev = RV_NONE;
uint32_t pairRevT0 = 0;
uint8_t pairRevTries = 0;
int32_t gripPosBeforeReboot = 0;
uint32_t reviveCount = 0;
void startRevive(uint8_t id, const char *why, int32_t err);
void startPairRevive(uint8_t id, const char *why, int32_t err);

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
// 부팅 홈 이동 전용: 소프트웨어 범위 밖(EEPROM 임시 확장 범위 안)에서도 그대로 출발
void setPairGoalsRaw(int32_t q) {
  int32_t lo = max32(hardwareMin[PAIR_INDEX_2] - PAIR_ZERO_2, hardwareMin[PAIR_INDEX_6] - PAIR_ZERO_6);
  int32_t hi = min32(hardwareMax[PAIR_INDEX_2] - PAIR_ZERO_2, hardwareMax[PAIR_INDEX_6] - PAIR_ZERO_6);
  q = clamp32(q, lo, hi);
  targetGoal[PAIR_INDEX_2] = PAIR_ZERO_2 + q;
  targetGoal[PAIR_INDEX_6] = PAIR_ZERO_6 + q;
}
void drainDxlRx();
void setPairPwm(int32_t pwm) {          // 두 어깨 Goal PWM (PWM Limit보다 크면 거부되므로 각자 한계로 자름)
  for (uint8_t id = 2; id <= 6; id += 4) {
    drainDxlRx();
    int32_t lim = dxl.readControlTableItem(PWM_LIMIT, id, IO_TIMEOUT_MS);
    drainDxlRx();
    dxl.writeControlTableItem(GOAL_PWM, id, min32(pwm, lim > 0 ? lim : PAIR_PWM_CAP_RAW), IO_TIMEOUT_MS);
  }
}
void startPairLift() { pairLiftBoost = true; pairLiftStartMs = millis(); }
void endPairLift() {
  if (!pairLiftBoost) return;
  pairLiftBoost = false;
  setPairPwm(PAIR_PWM_CAP_RAW);
  if (DEBUG_SERIAL) DEBUG_SERIAL.println("[LIFT] shoulder back to normal power");
}
void setPairGoals(int32_t q) {
  q = clamp32(q, qMin, qMax);
  targetGoal[PAIR_INDEX_2] = PAIR_ZERO_2 + q;
  targetGoal[PAIR_INDEX_6] = PAIR_ZERO_6 + q;
}

// 이 라이브러리는 응답의 ID를 확인하지 않고 버퍼에 먼저 와 있는 응답을 그대로 읽음.
// 늦게 온 응답·writeNoResp에 대한 응답이 남아 있으면 이후 모든 통신이 한 칸씩 밀려 실패/엉뚱한 값이 됨
// → 모든 요청 전에 수신 버퍼를 비움 (통신은 한 번에 하나씩이라 비워도 잃는 응답 없음)
void drainDxlRx() { uint16_t b = 128; while (b-- > 0 && DXL_SERIAL.available()) DXL_SERIAL.read(); }

bool readItem(uint8_t item, uint8_t id, int32_t &value) {
  drainDxlRx();
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
  drainDxlRx();
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
  faultAtMs = millis();
  faultReason = reason; faultId = id;
  armReady = wheelsReady = armed = armLinkActive = armHomeActive = false;
  neutralCount = 0;
  // Stop the wheels before removing their torque (best effort on a bad bus).
  if (hadWheels) {
    writeItem(GOAL_VELOCITY, LEFT_ID, 0);
    writeItem(GOAL_VELOCITY, RIGHT_ID, 0);
  }
  allTorqueOff();
  autoState = AUTO_IDLE;
  if (dumpReady) dxl.writeControlTableItem(GOAL_POSITION, DUMP_ID, DUMP_CLOSED_POS, IO_TIMEOUT_MS);
  if (DEBUG_SERIAL) {
    DEBUG_SERIAL.print("[FAULT] "); DEBUG_SERIAL.print(reason);
    DEBUG_SERIAL.print(" ID="); DEBUG_SERIAL.println(id);
    printFaultDetails();
    DEBUG_SERIAL.println("Support arm! Auto-restart (full reboot) in 3 s.");
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
// 다른 모터(복구 중인 관절 등)의 늦게 도착한 응답이 버퍼에 남아 있으면 바퀴 명령 응답으로 잘못 읽혀
// '바퀴 실패'가 됨 → 보내기 전에 버퍼 비우기 + 실패하면 한 번 더 + 연속 3번(60 ms) 실패해야만 진짜 실패로 처리.
// (바퀴 Bus Watchdog은 200 ms라 60 ms 동안 명령이 안 가도 안전)
constexpr uint8_t WHEEL_FAIL_LIMIT = 3;
uint8_t wheelFailStreak = 0;
// 바퀴가 과부하로 죽어도(토크 OFF) 명령 쓰기는 '성공'으로 응답함 → 예전엔 죽은 걸 몰라서 복구를 안 했음.
// 응답의 Alert 비트(0x80 = 하드웨어 오류 있음)를 보고 바로 복구 시작.
bool wheelAlert = false;
bool wheelWrite(uint8_t id, int32_t v) {
  bool ok = writeItem(GOAL_VELOCITY, id, v);
  if (!ok) ok = writeItem(GOAL_VELOCITY, id, v);
  if (ok && (dxl.getLastStatusPacketError() & 0x80)) wheelAlert = true;
  return ok;
}
bool commandWheels(int left, int right) {
  left = constrain(left, -wheelMax, wheelMax);     // Velocity Limit 넘는 명령은 모터가 무시함
  right = constrain(right, -wheelMax, wheelMax);
  bool l = wheelWrite(LEFT_ID, LEFT_SIGN * left);
  bool r = wheelWrite(RIGHT_ID, RIGHT_SIGN * right);
  if (wheelAlert) { wheelAlert = false; wheelFailStreak = 0; return false; }   // 하드웨어 오류: 즉시 복구
  if (l && r) { wheelFailStreak = 0; lastIoFailure.valid = false; return true; }
  if (++wheelFailStreak < WHEEL_FAIL_LIMIT) { lastIoFailure.valid = false; return true; }  // 일시적: 무시
  wheelFailStreak = 0;
  return false;
}
bool setupWheel(uint8_t id) {
  if (!dxl.ping(id)) return false;
  if (!ensureStatusReturnLevel(id)) return false;
  if (!writeItem(TORQUE_ENABLE, id, 0)) return false;
  if (!writeItem(BUS_WATCHDOG, id, 0)) return false;
  if (!setIfChanged(OPERATING_MODE, id, OP_VELOCITY)) return false;
  if (!setIfChanged(DRIVE_MODE, id, 0)) return false;
  if (!writeItem(PROFILE_ACCELERATION, id, 0)) return false;
  int32_t vlim;
  if (readItem(VELOCITY_LIMIT, id, vlim) && vlim > 0 && vlim < wheelMax) wheelMax = vlim;
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
void setDump(bool open);
void onAutoButton();

// 바퀴 명령 실패 → 전체 정지 대신 바퀴만 복구 (Watchdog 해제 / 하드웨어 오류면 재부팅)
bool wheelRecovering = false;
uint32_t wheelRecoverMs = 0;
uint32_t wheelFailCount = 0;

void reportWheelFailure(const char *where) {
  ++wheelFailCount;
  if (!wheelRecovering && DEBUG_SERIAL) {
    DEBUG_SERIAL.print("[WHEEL] "); DEBUG_SERIAL.print(where);
    DEBUG_SERIAL.print(" failed: lib="); DEBUG_SERIAL.print((int)dxl.getLastLibErrCode());
    DEBUG_SERIAL.print(" status=0x"); DEBUG_SERIAL.print(dxl.getLastStatusPacketError(), HEX);
    for (uint8_t id = LEFT_ID; id <= RIGHT_ID; ++id) {
      DEBUG_SERIAL.print(" | ID"); DEBUG_SERIAL.print(id);
      DEBUG_SERIAL.print(" hw=0x"); DEBUG_SERIAL.print(dxl.readControlTableItem(HARDWARE_ERROR_STATUS, id, IO_TIMEOUT_MS), HEX);
      DEBUG_SERIAL.print(" wd="); DEBUG_SERIAL.print(dxl.readControlTableItem(BUS_WATCHDOG, id, IO_TIMEOUT_MS));
      DEBUG_SERIAL.print(" V="); DEBUG_SERIAL.print(dxl.readControlTableItem(PRESENT_INPUT_VOLTAGE, id, IO_TIMEOUT_MS) / 10.0f, 1);
    }
    DEBUG_SERIAL.println(" -> recovering wheels only");
  }
  lastIoFailure.valid = false;
  wheelRecovering = true;
}

// 오류 없이 토크만 꺼진 경우(전원 순간 끊김 등)도 잡기: 250 ms마다 바퀴 하나씩 토크 상태 확인
void serviceWheelHealth() {
  static uint32_t last = 0;
  static uint8_t which = 0;
  if (!wheelsReady || wheelRecovering) return;
  uint32_t now = millis();
  if ((uint32_t)(now - last) < 250) return;
  last = now;
  uint8_t id = which ? RIGHT_ID : LEFT_ID;
  which ^= 1;
  int32_t torque, hw;
  if (!readItem(TORQUE_ENABLE, id, torque) || !readItem(HARDWARE_ERROR_STATUS, id, hw)) { lastIoFailure.valid = false; return; }
  if (torque != 1 || hw != 0) reportWheelFailure(hw != 0 ? "hardware error" : "torque lost");
}

// 200 ms마다 한 번 시도 (루프를 막지 않음)
void serviceWheelRecovery() {
  if (!wheelRecovering || !wheelsReady) return;
  uint32_t now = millis();
  if ((uint32_t)(now - wheelRecoverMs) < 200) return;
  wheelRecoverMs = now;
  bool ok = true;
  for (uint8_t id = LEFT_ID; id <= RIGHT_ID; ++id) {
    drainDxlRx();
    int32_t hw = dxl.readControlTableItem(HARDWARE_ERROR_STATUS, id, IO_TIMEOUT_MS);
    if (dxl.getLastLibErrCode() != DXL_LIB_OK) { ok = false; continue; }   // 재부팅 중이면 다음에
    if (hw > 0) { drainDxlRx(); dxl.reboot(id, IO_TIMEOUT_MS); drainDxlRx(); ok = false; continue; }   // 다음 시도 때 재설정
    // writeItem: 버퍼 비우기 + Bus Watchdog을 올바른 1바이트로 씀 (라이브러리 기본 함수는 2바이트로 써서 거부될 수 있음)
    ok = writeItem(BUS_WATCHDOG, id, 0) && ok;            // Watchdog 오류 해제
    ok = writeItem(TORQUE_ENABLE, id, 0) && ok;
    ok = writeItem(PROFILE_ACCELERATION, id, 0) && ok;    // 재부팅 대비
    ok = writeItem(GOAL_VELOCITY, id, 0) && ok;
    ok = writeItem(TORQUE_ENABLE, id, 1) && ok;
    ok = writeItem(BUS_WATCHDOG, id, 10) && ok;
  }
  lastIoFailure.valid = false;
  if (!ok && DEBUG_SERIAL) {
    static uint32_t lastNote = 0;
    if ((uint32_t)(now - lastNote) >= 1000) { lastNote = now; DEBUG_SERIAL.println("[WHEEL] recovering... (retry)"); }
  }
  if (ok) {
    wheelRecovering = false;
    armed = false; neutralCount = 0;          // 스틱 중앙 확인 후 다시 출발
    if (DEBUG_SERIAL) DEBUG_SERIAL.println("[WHEEL] recovered (center left/right sticks to drive)");
  }
}

// RC: 주행(왼쪽 스틱) + CH8 트럭 + CH7 자동 투입. 20 ms마다 호출
void rcControl() {
  uint16_t ch[8];
  uint32_t frameTime;
  bool seen;
  noInterrupts();
  for (uint8_t i = 0; i < 8; ++i) ch[i] = received[i];
  frameTime = lastFrameUs; seen = frameReceived;
  interrupts();
  dbgTurn = ch[CH_DRIVE_W]; dbgFwd = ch[CH_DRIVE_V];
  bool signalOK = seen && (uint32_t)(micros() - frameTime) < PPM_TIMEOUT_US;
  for (uint8_t i = 0; i < 8 && signalOK; ++i) if (ch[i] < 900 || ch[i] > 2100) signalOK = false;

  static bool autoBtnPrev = true;   // 켤 때 이미 눌려 있으면 무시
  if (!signalOK) {
    if (wheelsReady && !wheelRecovering && !commandWheels(0, 0)) reportWheelFailure("stop write");
    setDump(false);
    armed = false; neutralCount = 0; autoBtnPrev = true;
    return;
  }
  float forward = FORWARD_SIGN * stickValue(ch[CH_DRIVE_V], FWD_CENTER);
  float turn = TURN_SIGN * stickValue(ch[CH_DRIVE_W], TURN_CENTER);
  if (!armed) {
    if (wheelsReady && !wheelRecovering && !commandWheels(0, 0)) { reportWheelFailure("neutral write"); return; }
    if (forward == 0.0f && turn == 0.0f) {
      if (++neutralCount >= 25) armed = true;
    } else neutralCount = 0;
    return;
  }
  if (wheelsReady && !wheelRecovering) {
    float turnMax = (float)MAX_TURN_SPEED * wheelMax / MAX_FWD_SPEED;
    float left = forward * wheelMax + turn * turnMax;
    float right = forward * wheelMax - turn * turnMax;
    float peak = max(fabsf(left), fabsf(right));     // 넘으면 두 바퀴 같은 비율로 축소 (곡률 유지)
    if (peak > wheelMax) { float scale = wheelMax / peak; left *= scale; right *= scale; }
    if (!commandWheels((int)left, (int)right)) { reportWheelFailure("command write"); return; }
  }
  // CH8: 누르고 있으면 트럭 열림
  if (ch[CH_DUMP] > 1750) setDump(true);
  else if (ch[CH_DUMP] < 1250) setDump(false);
  // CH7: 눌리는 순간 자동 투입 (동작 중이면 중단)
  bool autoBtn = ch[CH_AUTO] > 1750;
  if (autoBtn && !autoBtnPrev) onAutoButton();
  autoBtnPrev = autoBtn;
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
  if (!writeItem(PROFILE_VELOCITY, 5, GRIP_HOME_VELOCITY)) return false;   // 확 닫기
  int64_t closeGoal = (int64_t)last + GRIP_CLOSE_DIR * (GRIP_SPAN + 500);
  if (closeGoal < -1048575 || closeGoal > 1048575) return false;
  if (!writeItem(GOAL_POSITION, 5, (int32_t)closeGoal)) return false;
  uint32_t started = millis(), lastSample = started;
  uint8_t still = 0;
  bool found = false;
  while ((uint32_t)(millis() - started) < GRIP_HOME_TIMEOUT_MS) {
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
      if ((uint32_t)(now - started) > 300 && abs32(position - last) < 3) {
        if (++still >= 3) { found = true; last = position; break; }
      } else still = 0;
      last = position;
    }
    delay(1);
  }
  if (!found) {
    if (DEBUG_SERIAL) DEBUG_SERIAL.println("[GRIP] homing timeout: did not stop closing (raise GRIP_HOME_CURRENT?)");
    return false;
  }
  gripClosedPos = last;
  targetGoal[GRIP_INDEX] = last - GRIP_CLOSE_DIR * GRIP_CLOSE_MARGIN;
  gripDesired = targetGoal[GRIP_INDEX];
  gripHoldActive = false;
  if (!writeItem(GOAL_POSITION, 5, targetGoal[GRIP_INDEX])) return false;
  if (!writeItem(GOAL_CURRENT, 5, gripFollowCurrent)) return false;
  if (!writeItem(PROFILE_VELOCITY, 5, 360)) return false;
  if (!writeItem(PROFILE_ACCELERATION, 5, 60)) return false;
  if (DEBUG_SERIAL) {
    DEBUG_SERIAL.print("[GRIP] closed="); DEBUG_SERIAL.print(gripClosedPos);
    DEBUG_SERIAL.print(" open="); DEBUG_SERIAL.println(gripClosedPos - GRIP_CLOSE_DIR * GRIP_SPAN);
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

// 현재 위치(한 바퀴 기준)가 범위 밖이면 EEPROM 위치 한계를 거기까지 임시로 넓힘.
// 소프트웨어 범위(JOINT_MIN/MAX)는 그대로라 리더·자동 목표는 계속 원래 범위로 제한됨.
// 다음 부팅 때 다시 계산되므로 범위 안에서 켜면 원래 값으로 돌아감.
bool bootWidenedLimits(uint8_t i, uint8_t id, int32_t baseMin, int32_t baseMax) {
  int32_t pos, off;
  if (!readItem(PRESENT_POSITION, id, pos) || !readItem(HOMING_OFFSET, id, off)) return false;
  if (off < -1024 || off > 1024) off = 0;
  int64_t t = ((int64_t)pos - off) % 4096;
  if (t < 0) t += 4096;
  pos = (int32_t)(t + off);
  bootMin[i] = max32(0, min32(baseMin, pos - BOOT_LIMIT_MARGIN));
  bootMax[i] = min32(4095, max32(baseMax, pos + BOOT_LIMIT_MARGIN));
  if ((bootMin[i] != baseMin || bootMax[i] != baseMax) && DEBUG_SERIAL) {
    DEBUG_SERIAL.print("[BOOT] ID "); DEBUG_SERIAL.print(id); DEBUG_SERIAL.print(" outside range at ");
    DEBUG_SERIAL.print(pos); DEBUG_SERIAL.println(" -> limits widened, homing slowly back");
  }
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
    // 입력 전압 오류(0x01)로는 꺼지지 않게: 다른 모터(트럭 등)가 순간 전류를 많이 먹어
    // 전압이 잠깐 떨어질 때마다 어깨가 꺼져 전체가 멈추던 문제 방지. 과부하·과열·전기충격은 유지
    if (!setIfChanged(SHUTDOWN, id, (shutdown | 0x3C) & ~0x01)) return false;
    if (!readItem(PWM_LIMIT, id, pwmLimit)) return false;
    if (pwmLimit <= 0) return rejectSetup("paired PWM limit", id, pwmLimit, 1, 885);
    if (!writeItem(GOAL_PWM, id, min32(pwmLimit, PAIR_PWM_CAP_RAW))) return false;
    // Persist measured position limits in the actuator as a second guard.
    // Only update EEPROM when values actually differ.
    setupStage = "paired position limits";
    int32_t minimum = i == PAIR_INDEX_2 ? PAIR_2_MIN : PAIR_6_MIN;
    int32_t maximum = i == PAIR_INDEX_2 ? PAIR_2_MAX : PAIR_6_MAX;
    if (!bootWidenedLimits(i, id, minimum, maximum)) return false;
    if (!setIfChanged(MIN_POSITION_LIMIT, id, bootMin[i]) ||
        !setIfChanged(MAX_POSITION_LIMIT, id, bootMax[i])) return false;
  }
  if (i == 2 || i == 3) {                  // ID3, ID4: EEPROM 위치 한계를 실측 범위로 (범위 밖에서 켜면 임시 확장)
    setupStage = "ID3/ID4 position limits";
    if (!bootWidenedLimits(i, id, JOINT_MIN[i], JOINT_MAX[i])) return false;
    if (!setIfChanged(MIN_POSITION_LIMIT, id, bootMin[i]) ||
        !setIfChanged(MAX_POSITION_LIMIT, id, bootMax[i])) return false;
  }
  if (i != GRIP_INDEX) {
    int32_t position, offset;
    setupStage = "read position/hardware range";
    int32_t hwMin, hwMax;
    if (!readItem(MIN_POSITION_LIMIT, id, hwMin) ||
        !readItem(MAX_POSITION_LIMIT, id, hwMax)) return false;
    hardwareMin[i] = hwMin; hardwareMax[i] = hwMax;
    if (paired && hwMin != bootMin[i])
      return rejectSetup("paired minimum readback", id, hwMin, bootMin[i], bootMin[i]);
    if (paired && hwMax != bootMax[i])
      return rejectSetup("paired maximum readback", id, hwMax, bootMax[i], bootMax[i]);
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
    // 어떤 자세로 재부팅해도 시작: 현재 위치 그대로 목표로 (튀지 않음). 홈 이동이 천천히 범위 안으로 데려감
    position = clamp32(position, hwMin, hwMax);
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
  if (abs32(q2 - q6) > PAIR_STARTUP_SYNC_RAW && DEBUG_SERIAL) {   // 거부 대신 경고: 두 모터 평균으로 맞춤
    DEBUG_SERIAL.print("[BOOT] pair start mismatch "); DEBUG_SERIAL.println(q2 - q6);
  }
  qCommand = qTarget = (q2 + q6) / 2;      // 범위 밖이어도 현재 자세 그대로
  setPairGoalsRaw(qCommand);
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
    position = clamp32(position, hardwareMin[i], hardwareMax[i]); // 현재 위치 그대로 (튀지 않음)
    targetGoal[i] = position;
  }
  q2 = targetGoal[PAIR_INDEX_2] - PAIR_ZERO_2;
  q6 = targetGoal[PAIR_INDEX_6] - PAIR_ZERO_6;
  // (부팅 정렬 검사 제거: 평균 자세로 토크 ON 하면 두 모터가 스스로 맞춰짐)
  qCommand = qTarget = (q2 + q6) / 2;
  setPairGoalsRaw(qCommand);
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
  if (data[70 - 64] != 0) { startPairRevive(id, "pair hardware error", data[70 - 64]); return false; }
  if (data[64 - 64] != 1) { startPairRevive(id, "pair torque lost", 0); return false; }
  if ((int8_t)data[98 - 64] < 0) { startPairRevive(id, "pair bus watchdog", data[98 - 64]); return false; }
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
  // 위치 한계 초과는 더 이상 정지 사유 아님 (목표가 범위 안에 잘려 있어 스스로 돌아옴)
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
  if (pairRev != RV_NONE) return;          // 어깨 복구 중: 감시 잠시 멈춤 (복구가 끝나면 재개)
  polled = true;
  lastPoll = now;
  PairSample sample = {};
  bool ok2 = readPairMotor(2, sample.pos2, sample.load2, sample.temp2);
  bool ok6 = false;
  if (!systemFault && pairRev == RV_NONE) ok6 = readPairMotor(6, sample.pos6, sample.load6, sample.temp6);
  if (pairRev != RV_NONE) return;          // 방금 오류 감지 → 복구 시작됨
  pairSampleOK = ok2 && ok6;
  if (systemFault) return;
  if (!pairSampleOK) {
    if (++pairCommMisses >= PAIR_COMM_MISS_LIMIT) {
      pairCommMisses = 0;
      if (armHomeActive) { if (DEBUG_SERIAL) DEBUG_SERIAL.println("[BOOT] pair read timeout (retry)"); }
      else startPairRevive(2, "pair read timeout", 0);
    }
    return; // Never issue a NEW target while either motor is unverified.
  }
  pairCommMisses = 0;
  pairSample = sample;
  uint8_t fault = instantaneousPairFault();
  if (fault == 3 && armHomeActive) fault = 0;  // 부팅 홈 이동 중엔 큰 정렬 오차도 정지 안 함 (같은 목표로 당겨 맞춤)
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
  // 지속 부하: 전체 정지 대신 경고만 (실제 과부하는 모터 Overload 보호 → 개별 복구가 처리)
  if (heldFor(peakLoad >= PAIR_LOAD_TRIP_RAW, now, PAIR_LOAD_TRIP_MS, loadSince, loadTiming)) {
    static uint32_t lastWarn = 0;
    if (DEBUG_SERIAL && (uint32_t)(now - lastWarn) >= 1000) {
      lastWarn = now;
      DEBUG_SERIAL.print("[WARN] pair high load "); DEBUG_SERIAL.println(peakLoad);
    }
  }
  if (heldFor(mismatch > PAIR_SYNC_TRIP_RAW, now, PAIR_SYNC_TRIP_MS,
              syncSince, syncTiming)) {
    if (armHomeActive) { syncTiming = false; return; }   // 홈 이동 중엔 정지 안 함
    syncTiming = false; startPairRevive(2, "pair alignment", mismatch); return;
  }
  if (heldFor(following > PAIR_TRACK_TRIP_RAW, now, PAIR_TRACK_TRIP_MS,
              trackSince, trackTiming)) {
    if (armHomeActive) { trackTiming = false; return; }  // 홈 이동 루프가 진행을 멈춰 기다려 줌
    trackTiming = false; startPairRevive(2, "pair stalled/following error", following); return;
  }
}

void serviceOtherArmSafety() {
  static uint32_t lastPoll = 0;
  static uint8_t index = 0;
  const uint8_t ids[] = {1, 3, 4, 5};
  uint32_t now = millis();
  if (!armReady || systemFault || (uint32_t)(now - lastPoll) < 50) return;
  lastPoll = now;
  static uint8_t misses[11] = {};
  uint8_t id = ids[index];
  index = (index + 1) % 4;
  if (id == ARM_IDS[GRIP_INDEX] && !gripperAvailable) return;
  if (revState[id] != RV_NONE) return;     // 복구 중인 모터는 건너뜀
  int32_t error, torque;
  if (!readItem(HARDWARE_ERROR_STATUS, id, error) ||
      !readItem(TORQUE_ENABLE, id, torque)) {
    lastIoFailure.valid = false;
    if (++misses[id] < 3) return;          // 한 번 못 읽은 걸로는 멈추지 않음
    if (id == ARM_IDS[GRIP_INDEX]) disableGripper("gripper status read");
    else { misses[id] = 0; startRevive(id, "no response", 0); }   // 전체 정지 대신 그 관절만 복구 시도
    return;
  }
  misses[id] = 0;
  if (error != 0 || torque != 1) startRevive(id, error != 0 ? "hardware error" : "torque lost", error);
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
    // 범위 밖 측정값도 허용 (목표는 항상 범위 안으로 잘려 있으므로 끌려 들어옴)
  }
  return true;
}

bool armInitialPoseReached(const int32_t *pose) {
  for (uint8_t i = 0; i < ARM_MOTOR_COUNT; ++i) {
    if (i == GRIP_INDEX) continue;
    int32_t error = pose[i] - armInitialGoal(i);
    if (error < -100 || error > 100) return false;
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
  // (초기 정렬 검사 제거: 어떤 자세에서도 홈 이동 시작)
  int32_t startQ = (q2 + q6) / 2;           // 범위 밖이어도 현재 자세에서 출발 (튀지 않음)
  qCommand = qTarget = startQ;
  for (uint8_t i = 0; i < ARM_MOTOR_COUNT; ++i)
    if (i != GRIP_INDEX) targetGoal[i] = start[i];
  setPairGoalsRaw(startQ);
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
  uint32_t homeTimeout = duration + ARM_HOME_TIMEOUT_MS;   // 먼 자세여도 시간 부족으로 거부 안 함
  armHomeActive = true; armLinkActive = false;
  setPairPwm(PAIR_LIFT_PWM_RAW); startPairLift();   // 처진 자세에서 켜도 들어 올릴 수 있게 (도착하면 원래 출력)
  if (!sendArmGoals()) { tripFault("initial hold write failed", 0); return false; }
  if (DEBUG_SERIAL) DEBUG_SERIAL.println("[ARM HOME] Moving to 2048/paired zeros. Wheels OFF.");
  uint32_t started = millis(), lastTick = started, progress = 0;
  uint32_t lagSince = 0;
  bool lagTiming = false;
  uint8_t stable = 0;
  while ((uint32_t)(millis() - started) < homeTimeout) {
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
        if (!readArmInitialPose(actual)) continue;   // 한 번 못 읽으면 다음 틱에 재시도
        int32_t error = 0;
        for (uint8_t i = 0; i < ARM_MOTOR_COUNT; ++i)
          if (i != GRIP_INDEX) error = max32(error, abs32(actual[i] - targetGoal[i]));
        // 못 따라오면(중력·걸림) 정지하지 않고 궤적 진행만 멈춰서 기다림
        bool lagging = error > ARM_HOME_TRACK_ABORT_RAW;
        // 3초 넘게 못 따라오면 홈 이동 루프를 끝내고 일반 제어로 넘김. 예전엔 여기서 궤적을 멈춘 채
        // 그 중간 자세를 '목표'로 굳혀 버려서, ID1까지 중간에 멈춘 채 힘을 주고 버텼음.
        if (heldFor(lagging, now, HOME_LAG_HANDOFF_MS, lagSince, lagTiming)) break;
        if (progress >= duration && armInitialPoseReached(actual)) {
          if (++stable >= ARM_HOME_STABLE_SAMPLES) {
            armHomeActive = false;
            if (DEBUG_SERIAL) DEBUG_SERIAL.println("[ARM HOME] Reached. Holding pose; Leader follow next.");
            return true; // keep Follower holding torque ON
          }
        } else stable = 0;
        // Failed pair reads pause progression; they never cause a catch-up jump.
        if (!lagging && progress < duration) progress += ARM_CONTROL_MS;
        if (progress > duration) progress = duration;
        for (uint8_t i = 0; i < ARM_MOTOR_COUNT; ++i)
          if (i != GRIP_INDEX && i != PAIR_INDEX_2 && i != PAIR_INDEX_6)
            targetGoal[i] = armInitialGoalAt(start[i], armInitialGoal(i), progress, duration);
        qCommand = qTarget = armInitialGoalAt(startQ, 0, progress, duration);
        setPairGoalsRaw(qCommand);
        if (!sendArmGoals()) { tripFault("initial goal write failed", 0); return false; }
      }
    }
    delay(1); // setup only; safety polling and USB stop remain active
  }
  // 못 따라와도 정지하지 않음. 목표는 '중간 자세'가 아니라 최종 홈 자세(2048)로 두고,
  // 일반 제어(관절별 느린 따라잡기 + 어깨 100% 출력)가 계속 끝까지 데려가게 넘김
  armHomeActive = false;
  for (uint8_t i = 0; i < ARM_MOTOR_COUNT; ++i) {
    if (i == GRIP_INDEX || i == PAIR_INDEX_2 || i == PAIR_INDEX_6) continue;
    armWant[i] = armInitialGoal(i);
  }
  qTarget = 0;
  for (uint8_t j = 0; j < 4; ++j) jointSlow[j] = true;
  homeHandoff = true;
  if (DEBUG_SERIAL) DEBUG_SERIAL.println("[ARM HOME] slow/blocked -> handing off; still moving to 2048 slowly.");
  return true;
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
    const int64_t maxMagnitude = field == GRIP_INDEX ? (int64_t)INT32_MAX + (negative ? 1 : 0) : 4095;
    int64_t value = 0;
    while (line[pos] >= '0' && line[pos] <= '9') {
      value = value * 10 + (line[pos] - '0');
      if (value > maxMagnitude) return false;
      ++pos;
    }
    if (negative) value = -value;
    if (field != GRIP_INDEX && value < 0) return false;
    values[field] = (int32_t)value;
    if (field + 1 < LEADER_JOINT_COUNT) {
      if (line[pos] != ',') return false;
      ++pos;
    } else if (line[pos] != '\0') return false;
  }
  return true;
}

// Leader 그리퍼 열림 정도 0(닫힘) ~ 1(열림). 고정 범위라 처음/나중 속도가 같음
float leaderGripFraction(int32_t delta) {
  float leaderRaw = 2048.0f + delta;
  float f = (LEADER_GRIP_CLOSED_RAW - leaderRaw) / (float)(LEADER_GRIP_CLOSED_RAW - LEADER_GRIP_OPEN_RAW);
  return f < 0 ? 0.0f : (f > 1 ? 1.0f : f);
}

int32_t gripperGoal(int32_t delta) {
  int32_t maxOpen = GRIP_SPAN - GRIP_OPEN_MARGIN - GRIP_CLOSE_MARGIN;
  int32_t open = (int32_t)(leaderGripFraction(delta) * maxOpen + 0.5f);
  return gripClosedPos - GRIP_CLOSE_DIR * (GRIP_CLOSE_MARGIN + open);
}

// 그리퍼 목표 적용 (물체에 막혀 힘을 줄인 상태면 그 이상 세게 쥐지 않음)
void setGripTarget(int32_t desired) {
  gripDesired = desired;
  if (gripHoldActive) {
    int32_t holdGoal = gripHoldPos + GRIP_CLOSE_DIR * GRIP_HOLD_SQUEEZE;
    if (GRIP_CLOSE_DIR * (desired - holdGoal) > 0) { targetGoal[GRIP_INDEX] = holdGoal; return; }
    gripHoldActive = false;          // Leader가 그 지점보다 더 열면 해제
  }
  targetGoal[GRIP_INDEX] = desired;
}

// 50 ms마다: 닫으라는데 막혀서 안 닫히고 멈춰 있으면 → 쥐는 힘을 낮춤
void serviceGripHold() {
  static uint32_t lastPoll = 0, stillSince = 0;
  static int32_t lastPos = 0;
  static bool timing = false, havePos = false;
  uint32_t now = millis();
  if (!gripperReady || systemFault || (uint32_t)(now - lastPoll) < 50) return;
  lastPoll = now;
  int32_t p;
  if (!readItem(PRESENT_POSITION, 5, p)) { lastIoFailure.valid = false; return; }
  bool still = havePos && abs32(p - lastPos) < 4;
  lastPos = p; havePos = true;
  bool blocked = !gripHoldActive && GRIP_CLOSE_DIR * (gripDesired - p) > GRIP_STALL_ERR;
  if (blocked && still) {
    if (!timing) { timing = true; stillSince = now; }
    else if ((uint32_t)(now - stillSince) >= GRIP_STALL_MS) {
      gripHoldActive = true;
      gripHoldPos = p;
      targetGoal[GRIP_INDEX] = p + GRIP_CLOSE_DIR * GRIP_HOLD_SQUEEZE;
      timing = false;
      if (DEBUG_SERIAL) { DEBUG_SERIAL.print("[GRIP] holding object at "); DEBUG_SERIAL.println(p); }
    }
  } else timing = false;
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
  memcpy(leaderVals, values, sizeof(leaderVals));   // 자동 투입 복귀 목표용으로 항상 기억
  leaderValsSeen = true;
  lastArmPacketMs = millis();
  armLinkActive = true;
  ++armPacketCount;
  if (autoState != AUTO_IDLE) {                     // 자동 투입 중: 그리퍼 학습만 하고 팔은 안 따라감
    if (gripperReady && values[GRIP_INDEX] != GRIP_NO_SAMPLE) gripperGoal(values[GRIP_INDEX]);
    return;
  }
  for (uint8_t i = 0; i < LEADER_JOINT_COUNT; ++i) {
    if (i == PAIR_INDEX_2) {
      qTarget = clamp32(LEADER_TO_FOLLOWER_J2_SIGN * (values[i] - LEADER_J2_ZERO), qMin, qMax);
    } else if (i == GRIP_INDEX) {
      if (gripperReady && values[i] != GRIP_NO_SAMPLE) {
        int32_t g = gripperGoal(values[i]);
        if (gripLatchOpen) {                        // 놓은 직후: Leader를 절반 이상 열면 다시 따라감
          if (leaderGripFraction(values[i]) > 0.5f) { gripLatchOpen = false; setGripTarget(g); }
        } else setGripTarget(g);
      }
    } else armWant[i] = clamp32(values[i], jointMinimum(i), jointMaximum(i));
  }
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
  if (!pairSampleOK && pairRev == RV_NONE) return false;
  // A brief link dropout stops the trajectory at the measured position while
  // retaining holding torque. A hardware fault instead latches torque OFF.
  int32_t q2 = pairSample.pos2 - PAIR_ZERO_2;
  int32_t q6 = pairSample.pos6 - PAIR_ZERO_6;
  if (pairRev == RV_NONE) {                          // 어깨 복구 중이면 어깨는 복구 루틴에 맡김
    if (abs32(q2 - q6) > PAIR_SYNC_TRIP_RAW) return false;
    qCommand = (q2 + q6) / 2;
    qTarget = clamp32(qCommand, qMin, qMax);
    setPairGoalsRaw(qCommand);
  }
  for (uint8_t i = 0; i < LEADER_JOINT_COUNT; ++i) {
    if (i == PAIR_INDEX_2) continue;
    if (i == GRIP_INDEX && !gripperReady) continue;
    if (revState[ARM_IDS[i]] != RV_NONE) continue;   // 복구 중인 관절은 읽지도 건드리지도 않음 (죽은 모터 때문에 전체 정지 방지)
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
    } else position = clamp32(position, hardwareMin[i], hardwareMax[i]);  // 범위 밖이어도 정지 안 함
    targetGoal[i] = position;
    if (i != GRIP_INDEX) armWant[i] = position;
    if (i == GRIP_INDEX) { gripDesired = position; gripHoldActive = false; }
  }
  return sendArmGoals();
}

void runAuto(float dt);

// 한 주기(20 ms) 동안 cur → want로 이동. 멀면 느린 모드로 들어가서 거의 도착할 때까지 유지
int32_t catchupStep(int32_t cur, int32_t want, uint8_t j, bool autoOn) {
  int32_t err = want - cur;
  int32_t a = abs32(err);
  if (autoOn) jointSlow[j] = false;               // 자동 투입은 자체 속도 프로파일 사용
  else if (!jointSlow[j] && a > CATCHUP_ENTER_RAW) {
    jointSlow[j] = true;
    if (DEBUG_SERIAL) { DEBUG_SERIAL.print("[SLOW] joint "); DEBUG_SERIAL.print(j); DEBUG_SERIAL.print(" far "); DEBUG_SERIAL.println(a); }
  } else if (jointSlow[j] && a <= CATCHUP_EXIT_RAW) jointSlow[j] = false;
  int32_t lim = jointSlow[j] ? CATCHUP_STEP_RAW : NORMAL_STEP_RAW;
  return cur + clamp32(err, -lim, lim);
}

void serviceArmMotion() {
  static uint32_t lastControl = 0;
  uint32_t now = millis();
  if (!armReady || systemFault || armHomeActive || !pairSampleOK ||
      (uint32_t)(now - lastControl) < ARM_CONTROL_MS) return;
  lastControl = now;
  if (autoState != AUTO_IDLE) {
    runAuto(ARM_CONTROL_MS / 1000.0f);   // 목표만 정함 → 아래 pair 속도 제한·전송은 공통
    if (systemFault) return;
  } else if (armLinkActive && (uint32_t)(now - lastArmPacketMs) >= ARM_LINK_TIMEOUT_MS) {
    armLinkActive = false;
    if (!holdArmAtPresent()) { tripFault("Bluetooth hold failed", 2); return; }
    if (DEBUG_SERIAL) DEBUG_SERIAL.println("[BT] timeout: holding measured pose.");
    return;
  }
  // 어깨 복구 중에는 목표를 앞서 보내지 않음 (복구 직후 추종 오차로 전체 정지 방지)
  // 관절별 속도 제한: 목표가 멀면(재부팅·복구 직후, 리더가 멀리 있을 때 등) 느리게, 가까우면 평소 속도
  bool autoOn = autoState != AUTO_IDLE;
  const uint8_t idx[3] = {0, 2, 3};
  for (uint8_t k = 0; k < 3; ++k) {
    uint8_t i = idx[k];
    uint8_t id = ARM_IDS[i];
    if (revState[id] != RV_NONE) continue;         // 복구 중인 모터는 목표 고정
    targetGoal[i] = catchupStep(targetGoal[i], armWant[i], k == 0 ? 0 : k + 1, autoOn);
  }
  if (pairRev == RV_NONE) {
    int32_t next = catchupStep(qCommand, qTarget, 1, autoOn);
    qCommand = clamp32(next, qCommand - pairFollowStep, qCommand + pairFollowStep);
  }
  setPairGoalsRaw(qCommand);   // 범위 밖에서 복구된 경우에도 튀지 않고 천천히 범위 안으로
  if (pairLiftBoost && pairRev == RV_NONE) {
    int32_t actualErr = max32(abs32(pairSample.pos2 - (PAIR_ZERO_2 + qTarget)),
                              abs32(pairSample.pos6 - (PAIR_ZERO_6 + qTarget)));
    if ((!jointSlow[1] && actualErr <= PAIR_LIFT_DONE_RAW) ||
        (uint32_t)(now - pairLiftStartMs) >= PAIR_LIFT_MAX_MS) endPairLift();
  }
  if (!sendArmGoals()) tripFault("arm sync write", 2);
}

// ======================= 개별 모터 자동 복구 ==========================
// 모두 루프를 막지 않음 (바퀴·어깨 Bus Watchdog 200 ms 때문). 통신은 짧은 타임아웃 한 번씩.
uint8_t armIndexOfId(uint8_t id) { for (uint8_t i = 0; i < ARM_MOTOR_COUNT; ++i) if (ARM_IDS[i] == id) return i; return 0; }

void startRevive(uint8_t id, const char *why, int32_t err) {
  if (id > 10 || revState[id] != RV_NONE) return;
  revState[id] = RV_WAIT; revT0[id] = millis(); revTries[id] = 0;
  ++reviveCount;
  if (DEBUG_SERIAL) {
    DEBUG_SERIAL.print("[REVIVE] ID "); DEBUG_SERIAL.print(id);
    DEBUG_SERIAL.print(" "); DEBUG_SERIAL.print(why);
    DEBUG_SERIAL.print(" err=0x"); DEBUG_SERIAL.print(err, HEX);
    DEBUG_SERIAL.println(" -> reboot in 1 s (others keep running)");
  }
}

void startPairRevive(uint8_t id, const char *why, int32_t err) {
  if (pairRev != RV_NONE) return;
  pairRev = RV_WAIT; pairRevT0 = millis(); pairRevTries = 0;
  ++reviveCount;
  if (DEBUG_SERIAL) {
    DEBUG_SERIAL.print("[REVIVE] shoulder ID2+ID6 ("); DEBUG_SERIAL.print(why);
    DEBUG_SERIAL.print(" at ID "); DEBUG_SERIAL.print(id);
    DEBUG_SERIAL.print(", err=0x"); DEBUG_SERIAL.print(err, HEX);
    DEBUG_SERIAL.println(") -> reboot both in 1 s. Shoulder may sag!");
  }
}

// 재부팅 직후 공통 설정 (RAM 값은 재부팅하면 초기화되므로 다시 씀)
bool reviveBasic(uint8_t id) {
  bool alive = false;
  for (uint8_t k = 0; k < 3 && !alive; ++k) { drainDxlRx(); alive = dxl.ping(id); }   // 핑 3번까지
  if (!alive) { reviveFailNote(id, "ping (no answer)"); return false; }
  uint8_t full = 2, zero = 0;
  drainDxlRx();
  dxl.write(id, 68, &full, 1, IO_TIMEOUT_MS);       // Status Return Level (응답까지 받아서 버퍼에 안 남김)
  drainDxlRx();
  if (!dxl.writeControlTableItem(TORQUE_ENABLE, id, 0, IO_TIMEOUT_MS)) { reviveFailNote(id, "torque off"); return false; }
  drainDxlRx();
  dxl.write(id, 98, &zero, 1, IO_TIMEOUT_MS);        // Bus Watchdog 해제
  drainDxlRx();
  return true;
}

// 죽은 관절이 토크가 꺼진 채 처져서 EEPROM 위치 한계 밖으로 나가 있으면, '현재 위치를 목표로' 쓰는 게
// 모터에게 거부돼(Data Limit) 복구가 영원히 실패했음 → 토크 OFF 상태(EEPROM 쓰기 가능)에서 한계를 거기까지 넓힘.
// 소프트웨어 범위는 그대로라 이후 천천히 원래 범위로 돌아감. 넓힌 한계는 다음 부팅 때 원래대로.
bool reviveFitLimits(uint8_t id, int32_t &pos) {
  uint8_t i = armIndexOfId(id);
  drainDxlRx();
  int32_t off = dxl.readControlTableItem(HOMING_OFFSET, id, IO_TIMEOUT_MS);
  if (dxl.getLastLibErrCode() != DXL_LIB_OK) return false;
  if (off < -1024 || off > 1024) off = 0;
  int64_t t = ((int64_t)pos - off) % 4096;
  if (t < 0) t += 4096;
  pos = (int32_t)(t + off);                          // 한 바퀴 기준으로 정리 (토크 ON 후 좌표와 같게)
  if (pos >= hardwareMin[i] && pos <= hardwareMax[i]) return true;
  int32_t lo = max32(0, min32(hardwareMin[i], pos - BOOT_LIMIT_MARGIN));
  int32_t hi = min32(4095, max32(hardwareMax[i], pos + BOOT_LIMIT_MARGIN));
  if (lo != hardwareMin[i] && !dxl.writeControlTableItem(MIN_POSITION_LIMIT, id, lo, (uint32_t)20)) return false;
  if (hi != hardwareMax[i] && !dxl.writeControlTableItem(MAX_POSITION_LIMIT, id, hi, (uint32_t)20)) return false;
  hardwareMin[i] = lo; hardwareMax[i] = hi;
  drainDxlRx();
  if (DEBUG_SERIAL) {
    DEBUG_SERIAL.print("[REVIVE] ID "); DEBUG_SERIAL.print(id); DEBUG_SERIAL.print(" sagged outside limits at ");
    DEBUG_SERIAL.print(pos); DEBUG_SERIAL.println(" -> limits widened, returning slowly");
  }
  return true;
}
void reviveFailNote(uint8_t id, const char *step) {
  static uint32_t last[11] = {};
  uint32_t now = millis();
  if (!DEBUG_SERIAL || id > 10 || (uint32_t)(now - last[id]) < 1000) return;
  last[id] = now;
  DEBUG_SERIAL.print("[REVIVE] ID "); DEBUG_SERIAL.print(id); DEBUG_SERIAL.print(" retry: "); DEBUG_SERIAL.print(step);
  DEBUG_SERIAL.print(" lib="); DEBUG_SERIAL.print((int)dxl.getLastLibErrCode());
  DEBUG_SERIAL.print(" status=0x"); DEBUG_SERIAL.println(dxl.getLastStatusPacketError(), HEX);
}

// ID1/3/4/5 하나 복구: 현재 위치에서 토크 ON → 1초간 천천히 목표로 → 정상 속도
bool reviveRestoreSingle(uint8_t id) {
  if (!reviveBasic(id)) return false;
  int32_t now = dxl.readControlTableItem(PRESENT_POSITION, id, IO_TIMEOUT_MS);
  if (dxl.getLastLibErrCode() != DXL_LIB_OK) { reviveFailNote(id, "read position"); return false; }  // 0으로 튀는 것 방지
  if (id != 5 && !reviveFitLimits(id, now)) { reviveFailNote(id, "widen limits"); return false; }
  if (id == 5) {
    // 재부팅하면 위치 기준이 4096 단위로 바뀔 수 있음 → 닫힘 기준도 같이 이동
    int32_t shift = (int32_t)lroundf((now - gripPosBeforeReboot) / 4096.0f) * 4096;
    gripClosedPos += shift; gripDesired += shift; gripHoldPos += shift;
    targetGoal[GRIP_INDEX] += shift;
    int32_t pwmLimit = dxl.readControlTableItem(PWM_LIMIT, id, IO_TIMEOUT_MS);
    dxl.writeControlTableItem(GOAL_CURRENT, id, gripFollowCurrent, IO_TIMEOUT_MS);
    dxl.writeControlTableItem(GOAL_PWM, id, pwmLimit > 0 ? min32(pwmLimit, 796) : 796, IO_TIMEOUT_MS);
  }
  dxl.writeControlTableItem(PROFILE_ACCELERATION, id, 20, IO_TIMEOUT_MS);
  dxl.writeControlTableItem(PROFILE_VELOCITY, id, 40, IO_TIMEOUT_MS);   // 천천히 합류
  if (!dxl.writeControlTableItem(GOAL_POSITION, id, now, IO_TIMEOUT_MS)) { reviveFailNote(id, "goal write"); return false; }
  if (!dxl.writeControlTableItem(TORQUE_ENABLE, id, 1, IO_TIMEOUT_MS)) { reviveFailNote(id, "torque on"); return false; }
  if (id != 5) {                                     // 현재 위치부터 느린 따라잡기로 합류
    uint8_t i = armIndexOfId(id);
    targetGoal[i] = now;
    jointSlow[i] = true;                             // 느린 모드 슬롯: ID1=0, ID3=2, ID4=3
  }
  return true;
}

void reviveNormalProfile(uint8_t id) {
  if (id == 5) {
    dxl.writeControlTableItem(PROFILE_VELOCITY, id, 360, IO_TIMEOUT_MS);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, id, 60, IO_TIMEOUT_MS);
  } else {
    dxl.writeControlTableItem(PROFILE_ACCELERATION, id, FOLLOW_PROFILE_ACCELERATION, IO_TIMEOUT_MS);
    dxl.writeControlTableItem(PROFILE_VELOCITY, id, FOLLOW_PROFILE_VELOCITY, IO_TIMEOUT_MS);
  }
}

// 어깨 둘 복구: 두 위치가 맞는지 확인 → 현재 위치를 목표로 → 동시에 토크 ON
bool reviveRestorePair() {
  if (!reviveBasic(2) || !reviveBasic(6)) return false;
  drainDxlRx();
  int32_t p2 = dxl.readControlTableItem(PRESENT_POSITION, 2, IO_TIMEOUT_MS);
  if (dxl.getLastLibErrCode() != DXL_LIB_OK) { reviveFailNote(2, "read position"); return false; }
  drainDxlRx();
  int32_t p6 = dxl.readControlTableItem(PRESENT_POSITION, 6, IO_TIMEOUT_MS);
  if (dxl.getLastLibErrCode() != DXL_LIB_OK) { reviveFailNote(6, "read position"); return false; }
  if (!reviveFitLimits(2, p2) || !reviveFitLimits(6, p6)) { reviveFailNote(2, "widen limits"); return false; }
  int32_t q2 = p2 - PAIR_ZERO_2, q6 = p6 - PAIR_ZERO_6;
  if (abs32(q2 - q6) > PAIR_SYNC_HARD_RAW) { tripFault("pair alignment after revive", 2); return false; }
  qCommand = (q2 + q6) / 2;                          // 처진 위치 그대로 (튀지 않음) → 느린 모드로 복귀
  setPairGoalsRaw(qCommand);
  jointSlow[1] = true;
  startPairLift();
  for (uint8_t id = 2; id <= 6; id += 4) {
    int32_t pwmLimit = dxl.readControlTableItem(PWM_LIMIT, id, IO_TIMEOUT_MS);
    dxl.writeControlTableItem(GOAL_PWM, id, min32(pwmLimit > 0 ? pwmLimit : PAIR_PWM_CAP_RAW, PAIR_LIFT_PWM_RAW), IO_TIMEOUT_MS);  // 들어 올리는 동안 100%
    dxl.writeControlTableItem(PROFILE_ACCELERATION, id, 20, IO_TIMEOUT_MS);
    dxl.writeControlTableItem(PROFILE_VELOCITY, id, 40, IO_TIMEOUT_MS);
    if (!dxl.writeControlTableItem(GOAL_POSITION, id, targetGoal[id == 2 ? PAIR_INDEX_2 : PAIR_INDEX_6], IO_TIMEOUT_MS)) { reviveFailNote(id, "goal write"); return false; }
  }
  uint8_t on = 1;                                    // 두 모터 한 패킷으로 동시에 토크 ON
  DYNAMIXEL::InfoSyncWriteInst_t info = {};
  DYNAMIXEL::XELInfoSyncWrite_t xels[2] = {};
  info.addr = 64; info.addr_length = 1; info.p_xels = xels; info.xel_count = 2; info.is_info_changed = true;
  xels[0].id = 2; xels[1].id = 6; xels[0].p_data = xels[1].p_data = &on;
  if (!dxl.syncWrite(&info)) return false;
  uint8_t wd = 10;
  dxl.write(2, 98, &wd, 1, IO_TIMEOUT_MS);
  dxl.write(6, 98, &wd, 1, IO_TIMEOUT_MS);
  return true;
}

void powerCycleRestart(const char *why, uint8_t id) {
  if (DEBUG_SERIAL) {
    DEBUG_SERIAL.print("[POWER CYCLE] ID "); DEBUG_SERIAL.print(id); DEBUG_SERIAL.print(" "); DEBUG_SERIAL.print(why);
    DEBUG_SERIAL.println(" -> DXL power OFF 1 s, then full restart (any-pose boot)");
    DEBUG_SERIAL.flush();
  }
  allTorqueOff();
  digitalWrite(31, LOW);              // OpenRB-150 DXL 전원 차단 = 모든 모터 전원 껐다 켜기
  delay(DXL_POWER_OFF_MS);
  NVIC_SystemReset();                 // setup()이 전원을 다시 켜고 어떤 자세에서든 다시 시작
}

void serviceRevive() {
  static uint32_t lastPoll = 0;
  uint32_t now = millis();
  if (systemFault || (uint32_t)(now - lastPoll) < 50) return;
  lastPoll = now;

  const uint8_t ids[] = {1, 3, 4, 5};
  for (uint8_t k = 0; k < 4; ++k) {
    uint8_t id = ids[k];
    switch (revState[id]) {
      case RV_WAIT:
        if ((uint32_t)(now - revT0[id]) >= REVIVE_WAIT_MS) {
          if (id == 5) gripPosBeforeReboot = dxl.readControlTableItem(PRESENT_POSITION, id, IO_TIMEOUT_MS);
          drainDxlRx(); dxl.reboot(id, IO_TIMEOUT_MS); drainDxlRx();
          revState[id] = RV_BOOT; revT0[id] = now; revTries[id] = 0;
        }
        break;
      case RV_BOOT:
        if ((uint32_t)(now - revT0[id]) < 600) break;
        if (reviveRestoreSingle(id)) { revState[id] = RV_EASE; revT0[id] = now; revSilentCycles[id] = 0; }
        else if (++revTries[id] >= 40) {
          revState[id] = RV_WAIT; revT0[id] = now;
          if (++revSilentCycles[id] >= SILENT_CYCLES_FOR_POWER_CYCLE) powerCycleRestart("no answer even after reboot", id);
        }
        lastIoFailure.valid = false;
        break;
      case RV_EASE:
        if ((uint32_t)(now - revT0[id]) >= 1000) {
          reviveNormalProfile(id);
          revState[id] = RV_NONE;
          if (DEBUG_SERIAL) { DEBUG_SERIAL.print("[REVIVE] ID "); DEBUG_SERIAL.print(id); DEBUG_SERIAL.println(" back"); }
        }
        break;
      default: break;
    }
  }

  switch (pairRev) {
    case RV_WAIT:
      if ((uint32_t)(now - pairRevT0) >= REVIVE_WAIT_MS) {
        drainDxlRx(); dxl.reboot(2, IO_TIMEOUT_MS);
        drainDxlRx(); dxl.reboot(6, IO_TIMEOUT_MS); drainDxlRx();
        pairRev = RV_BOOT; pairRevT0 = now; pairRevTries = 0;
      }
      break;
    case RV_BOOT:
      if ((uint32_t)(now - pairRevT0) < 600) break;
      if (systemFault) break;
      if (reviveRestorePair()) { pairRev = RV_EASE; pairRevT0 = now; pairSilentCycles = 0; }
      else if (!systemFault && ++pairRevTries >= 40) {
        pairRev = RV_WAIT; pairRevT0 = now;
        if (++pairSilentCycles >= SILENT_CYCLES_FOR_POWER_CYCLE) powerCycleRestart("shoulder no answer even after reboot", 2);
      }
      lastIoFailure.valid = false;
      break;
    case RV_EASE:
      if ((uint32_t)(now - pairRevT0) >= 1000) {
        reviveNormalProfile(2); reviveNormalProfile(6);
        loadTiming = syncTiming = trackTiming = false;   // 감시 타이머 초기화 후 재개
        pairCommMisses = 0;
        pairRev = RV_NONE;
        if (DEBUG_SERIAL) DEBUG_SERIAL.println("[REVIVE] shoulder back");
      }
      break;
    default: break;
  }
}

// ======================= 트럭 개방 ===================================
void setupDump() {
  uint32_t t0 = millis();
  while (!dxl.ping(DUMP_ID)) {
    if ((uint32_t)(millis() - t0) >= DXL_READY_TIMEOUT_MS) {
      if (DEBUG_SERIAL) { DEBUG_SERIAL.print("[DUMP] ID "); DEBUG_SERIAL.print(DUMP_ID); DEBUG_SERIAL.println(" not found -> disabled"); }
      lastIoFailure.valid = false;
      return;
    }
    delay(2);
  }
  int32_t mode = 0, lo = 0, hi = 4095;
  if (!ensureStatusReturnLevel(DUMP_ID) ||
      !writeItem(TORQUE_ENABLE, DUMP_ID, 0) ||
      !setIfChanged(OPERATING_MODE, DUMP_ID, OP_POSITION) ||
      !readItem(DRIVE_MODE, DUMP_ID, mode)) {
    if (DEBUG_SERIAL) DEBUG_SERIAL.println("[DUMP] setup failed -> disabled");
    lastIoFailure.valid = false;
    return;
  }
  bool timeBased = mode & 0x04;
  dumpTimeBased = timeBased;
  setIfChanged(SHUTDOWN, DUMP_ID, DUMP_SHUTDOWN);
  writeItem(GOAL_PWM, DUMP_ID, DUMP_PWM_CAP);
  { int32_t lim; if (readItem(PWM_LIMIT, DUMP_ID, lim) && lim > 0) dumpPwmLimit = lim; }
  writeItem(PROFILE_VELOCITY, DUMP_ID, timeBased ? DUMP_MOVE_TIME : DUMP_MOVE_VEL);
  if (DUMP_P_GAIN > 0) writeItem(POSITION_P_GAIN, DUMP_ID, DUMP_P_GAIN);
  writeItem(POSITION_I_GAIN, DUMP_ID, DUMP_I_GAIN);
  if (readItem(MIN_POSITION_LIMIT, DUMP_ID, lo) && readItem(MAX_POSITION_LIMIT, DUMP_ID, hi) &&
      (DUMP_CLOSED_POS < lo || DUMP_CLOSED_POS > hi || DUMP_OPEN_POS < lo || DUMP_OPEN_POS > hi) && DEBUG_SERIAL)
    DEBUG_SERIAL.println("[DUMP] WARNING: open/closed pos outside motor position limit");
  writeItem(GOAL_POSITION, DUMP_ID, DUMP_CLOSED_POS);
  writeItem(TORQUE_ENABLE, DUMP_ID, 1);
  lastIoFailure.valid = false;
  dumpReady = true; dumpOpen = false;
  if (DEBUG_SERIAL) DEBUG_SERIAL.println("[DUMP] Ready (closed).");
}

// 트럭 모터 Overload 등 하드웨어 오류 → 2초 뒤 재부팅하고 다시 설정 (원래 위치로)
// ※ 절대 루프를 막지 않음: 바퀴·어깨는 200 ms 동안 명령이 없으면 Bus Watchdog으로 멈추기 때문.
//   모든 통신은 짧은 타임아웃(IO_TIMEOUT_MS) 한 번씩, 기다리는 건 millis()로.
bool dumpQuickConfigure() {
  uint8_t id = DUMP_ID;
  if (!dxl.ping(id)) return false;
  uint8_t full = 2;
  drainDxlRx();
  dxl.write(id, 68, &full, 1, IO_TIMEOUT_MS);              // Status Return Level = 2 (응답 소비)
  drainDxlRx();
  int32_t mode = dxl.readControlTableItem(DRIVE_MODE, id, IO_TIMEOUT_MS);
  bool timeBased = mode & 0x04;
  dumpTimeBased = timeBased;
  bool ok = dxl.writeControlTableItem(TORQUE_ENABLE, id, 0, IO_TIMEOUT_MS);
  ok = ok && dxl.writeControlTableItem(GOAL_PWM, id, DUMP_PWM_CAP, IO_TIMEOUT_MS);   // RAM → 재부팅 후 다시
  ok = ok && dxl.writeControlTableItem(PROFILE_VELOCITY, id, timeBased ? DUMP_MOVE_TIME : DUMP_MOVE_VEL, IO_TIMEOUT_MS);
  if (DUMP_P_GAIN > 0) ok = ok && dxl.writeControlTableItem(POSITION_P_GAIN, id, DUMP_P_GAIN, IO_TIMEOUT_MS);
  ok = ok && dxl.writeControlTableItem(POSITION_I_GAIN, id, DUMP_I_GAIN, IO_TIMEOUT_MS);
  ok = ok && dxl.writeControlTableItem(GOAL_POSITION, id, dumpOpen ? DUMP_OPEN_POS : DUMP_CLOSED_POS, IO_TIMEOUT_MS);
  ok = ok && dxl.writeControlTableItem(TORQUE_ENABLE, id, 1, IO_TIMEOUT_MS);
  return ok;
}

void serviceDump() {
  enum : uint8_t { D_OK, D_WAIT, D_BOOT };
  static uint8_t state = D_OK;
  static uint32_t lastPoll = 0, t0 = 0;
  static uint8_t tries = 0;
  if (!dumpReady) return;
  uint32_t now = millis();
  if ((uint32_t)(now - lastPoll) < 100) return;
  lastPoll = now;

  if (state == D_WAIT) {                       // 2초 식히기
    if ((uint32_t)(now - t0) < REVIVE_WAIT_MS) return;
    dxl.reboot(DUMP_ID, IO_TIMEOUT_MS);
    state = D_BOOT; t0 = now; tries = 0;
    return;
  }
  if (state == D_BOOT) {                       // 재부팅 끝날 때까지 (막지 않고) 기다렸다가 짧게 재설정
    if ((uint32_t)(now - t0) < 600) return;
    if (dumpQuickConfigure()) {
      state = D_OK;
      if (DEBUG_SERIAL) DEBUG_SERIAL.println("[DUMP] revived");
    } else if (++tries >= 20) {                // 2초 동안 실패 → 처음부터 다시
      state = D_WAIT; t0 = now;
      if (DEBUG_SERIAL) DEBUG_SERIAL.println("[DUMP] revive failed, retrying");
    }
    lastIoFailure.valid = false;
    return;
  }
  int32_t err = dxl.readControlTableItem(HARDWARE_ERROR_STATUS, DUMP_ID, IO_TIMEOUT_MS);
  if (err > 0) {
    state = D_WAIT; t0 = now;
    if (DEBUG_SERIAL) {
      DEBUG_SERIAL.print("[DUMP] HW error 0x"); DEBUG_SERIAL.print(err, HEX);
      DEBUG_SERIAL.print(" auto="); DEBUG_SERIAL.print((int)autoState);
      DEBUG_SERIAL.println(" -> revive in 1 s");
    }
  }
}

void setDump(bool open) {
  if (!dumpReady || open == dumpOpen) return;
  dumpOpen = open;
  if (open) {
    // 열기: 프로파일 속도 0 / 가속 0 = 제한 없음, PWM 최대 → 모터가 낼 수 있는 최고 속도
    dxl.writeControlTableItem(PROFILE_ACCELERATION, DUMP_ID, 0, IO_TIMEOUT_MS);
    dxl.writeControlTableItem(PROFILE_VELOCITY, DUMP_ID, 0, IO_TIMEOUT_MS);
    dxl.writeControlTableItem(GOAL_PWM, DUMP_ID, min32(DUMP_OPEN_PWM, dumpPwmLimit), IO_TIMEOUT_MS);
  } else {
    // 닫기: 기존 설정 그대로 (천천히·출력 제한 → 끝에서 부딪혀 과부하 나는 것 방지)
    dxl.writeControlTableItem(GOAL_PWM, DUMP_ID, min32(DUMP_PWM_CAP, dumpPwmLimit), IO_TIMEOUT_MS);
    dxl.writeControlTableItem(PROFILE_VELOCITY, DUMP_ID, dumpTimeBased ? DUMP_MOVE_TIME : DUMP_MOVE_VEL, IO_TIMEOUT_MS);
  }
  dxl.writeControlTableItem(GOAL_POSITION, DUMP_ID, open ? DUMP_OPEN_POS : DUMP_CLOSED_POS, IO_TIMEOUT_MS);
}

// ======================= 자동 투입 ===================================
// 관절 tick 공간: j=0 ID1, j=1 ID2(=PAIR_ZERO_2+q, ID6 동일), j=2 ID3, j=3 ID4
int32_t autoTickMin(uint8_t j) { return j == 1 ? PAIR_ZERO_2 + qMin : jointMinimum(j); }
int32_t autoTickMax(uint8_t j) { return j == 1 ? PAIR_ZERO_2 + qMax : jointMaximum(j); }

float autoWrapPi(float a) {
  while (a >  3.14159265f) a -= 6.2831853f;
  while (a < -3.14159265f) a += 6.2831853f;
  return a;
}
float autoQToTick(uint8_t j, float q) { return 2048.0f + FOLLOWER_JOINT_DIR[j] * q * (4096.0f / 6.2831853f); }

bool autoTicksOK(const float t[4]) {
  for (uint8_t j = 0; j < 4; ++j) if (t[j] < autoTickMin(j) || t[j] > autoTickMax(j)) return false;
  return true;
}
bool autoTicksOKMargin(const float t[4]) {
  for (uint8_t j = 0; j < 4; ++j) {
    float m = j == 0 ? 0 : AUTO_LIMIT_MARGIN;          // ID1(회전)은 여유 불필요
    if (t[j] < autoTickMin(j) + m || t[j] > autoTickMax(j) - m) return false;
  }
  return true;
}

// ID1을 psi로 돌린 평면에서 잡는 지점 (r, z), 그리퍼 각 phi → 관절 tick
bool autoIK(float psi, float r, float z, float phi, float t[4]) {
  float wr = r - L3 * cosf(phi + DELTA);
  float wz = z - SH_Z - L3 * sinf(phi + DELTA);
  float c = (wr * wr + wz * wz - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
  if (c > C_MAX || c < -C_MAX) return false;
  float beta = -acosf(c);
  float a1 = atan2f(wz, wr) - atan2f(L2 * sinf(beta), L1 + L2 * cosf(beta));
  float a2 = a1 + beta;
  float q[4] = { psi, autoWrapPi(a1 - A1_0), autoWrapPi(beta + A1_0), autoWrapPi(phi - a2) };
  for (uint8_t j = 0; j < 4; ++j) t[j] = autoQToTick(j, q[j]);
  return autoTicksOK(t);
}

// 관절 tick 자세에서 팔꿈치·손목·잡는 지점·그리퍼 끝 중 가장 뒤쪽(y 최소) 값
float autoMinY(const float t[4]) {
  float q[4];
  for (uint8_t j = 0; j < 4; ++j) q[j] = (t[j] - 2048.0f) / FOLLOWER_JOINT_DIR[j] * (6.2831853f / 4096.0f);
  float a1 = A1_0 + q[1], a2 = a1 - A1_0 + q[2], phi = a2 + q[3];
  float rE = L1 * cosf(a1);
  float rW = rE + L2 * cosf(a2);
  float rT = rW + L3 * cosf(phi + DELTA);
  float rTip = rT + GRIP_TIP_EXTRA * cosf(phi);
  float c = cosf(q[0]);                  // 도면 y = 수평거리 × cos(ID1 각)
  float m = 0;                           // ID2 축(원점) 자체는 y=0
  float r[4] = { rE, rW, rT, rTip };
  for (uint8_t i = 0; i < 4; ++i) m = min(m, r[i] * c);
  return m;
}

// a → b 관절 보간 경로 전체가 y 제한을 지키는지 (20등분 검사)
bool autoPathOK(const float a[4], const float b[4]) {
  for (uint8_t k = 0; k <= 20; ++k) {
    float s = k / 20.0f, t[4];
    for (uint8_t j = 0; j < 4; ++j) t[j] = a[j] + (b[j] - a[j]) * s;
    if (autoMinY(t) < AUTO_Y_MIN) return false;
  }
  return true;
}

void computeAutoPoses() {
  float qs[4] = { 0.0f, 1.5707963f - A1_0, A1_0, 0.0f };   // 세운 자세: L1·L2 수직, 그리퍼 위
  for (uint8_t j = 0; j < 4; ++j) tStandFront[j] = autoQToTick(j, qs[j]);
  dropPoseOK = false;
  if (!autoTicksOK(tStandFront)) {
    if (DEBUG_SERIAL) DEBUG_SERIAL.println("[AUTO] stand pose outside joint limits -> auto disabled");
    return;
  }
  // ID1을 놓는 지점 쪽으로. 모터 범위 안에 들어가는 각도 표현 선택 (뒤쪽 ±180° 처리)
  float fwd = DROP_Y, side = DROP_X;            // 코드 좌표: 정면 +, 왼쪽 + (도면 x와 같은 방향, 11/13 확인)
  float r = sqrtf(fwd * fwd + side * side);
  float psi = atan2f(side, fwd);
  float cand[3] = { psi, psi - 6.2831853f, psi + 6.2831853f };
  bool yawOK = false;
  for (uint8_t i = 0; i < 3 && !yawOK; ++i) {
    float t = autoQToTick(0, cand[i]);
    if (t >= autoTickMin(0) && t <= autoTickMax(0)) { psi = cand[i]; yawOK = true; }
  }
  if (!yawOK) {                                   // 한계에 걸리면 가능한 끝까지만 돌림
    float t = constrain(autoQToTick(0, psi), (float)autoTickMin(0), (float)autoTickMax(0));
    psi = (t - 2048.0f) / FOLLOWER_JOINT_DIR[0] * (6.2831853f / 4096.0f);
  }
  for (uint8_t j = 0; j < 4; ++j) tStandBack[j] = tStandFront[j];
  tStandBack[0] = autoQToTick(0, psi);

  // 1단계: 5° 간격, 2단계: 찾은 범위 ±5°를 1° 간격 (계산량 줄이기)
  // ※ 계산 중에도 어깨 상태를 읽어 통신 유지: 200 ms 넘게 통신이 없으면 Bus Watchdog으로 멈춤
  float best = 1e9f, t[4];
  int coarseLo = 999, coarseHi = -999;
  for (int pass = 0; pass < 2; ++pass) {
    int from = pass == 0 ? -180 : coarseLo - 5, to = pass == 0 ? 179 : coarseHi + 5;
    int stepDeg = pass == 0 ? 5 : 1;
    if (pass == 1 && coarseLo > coarseHi) { from = -180; to = 179; }   // 5°로 못 찾으면 전체를 1°로
    for (int d = from; d <= to; d += stepDeg) {
      servicePairSafety();                       // 통신 유지 (자체적으로 주기 제한됨)
      if (systemFault) return;
      if (!autoIK(psi, r, DROP_Z, d * DEG_RAD, t)) continue;
      if (!autoTicksOKMargin(t)) continue;       // 한계에 붙은 자세는 제외
      if (!autoPathOK(tStandBack, t)) continue;  // 놓는 자세 + 가는 경로 모두 y ≥ AUTO_Y_MIN
      if (pass == 0) { coarseLo = min(coarseLo, d); coarseHi = max(coarseHi, d); }
      float cost = 0;
      for (uint8_t j = 1; j < 4; ++j) cost += fabsf(t[j] - tStandBack[j]);
      if (cost < best) { best = cost; for (uint8_t j = 0; j < 4; ++j) tDrop[j] = t[j]; dropPoseOK = true; }
    }
  }
  if (!DEBUG_SERIAL) return;
  if (!dropPoseOK) { DEBUG_SERIAL.println("[AUTO] drop point unreachable within limits / y limit -> auto disabled"); return; }
  DEBUG_SERIAL.print("[AUTO] stand ");
  for (uint8_t j = 0; j < 4; ++j) { DEBUG_SERIAL.print((int32_t)lroundf(tStandFront[j])); DEBUG_SERIAL.print(j < 3 ? "," : "  back-yaw "); }
  DEBUG_SERIAL.print((int32_t)lroundf(tStandBack[0]));
  DEBUG_SERIAL.print("  drop ");
  for (uint8_t j = 0; j < 4; ++j) { DEBUG_SERIAL.print((int32_t)lroundf(tDrop[j])); DEBUG_SERIAL.print(j < 3 ? "," : ""); }
  DEBUG_SERIAL.print("  min y "); DEBUG_SERIAL.println(autoMinY(tDrop), 1);
}

void autoCurrentTicks(float t[4]) {
  t[0] = targetGoal[0]; t[1] = PAIR_ZERO_2 + qCommand; t[2] = targetGoal[2]; t[3] = targetGoal[3];
}
void autoLeaderTicks(float t[4]) {
  if (!leaderValsSeen) { autoCurrentTicks(t); return; }
  t[0] = clamp32(leaderVals[0], jointMinimum(0), jointMaximum(0));
  t[1] = PAIR_ZERO_2 + clamp32(LEADER_TO_FOLLOWER_J2_SIGN * (leaderVals[1] - LEADER_J2_ZERO), qMin, qMax);
  t[2] = clamp32(leaderVals[2], jointMinimum(2), jointMaximum(2));
  t[3] = clamp32(leaderVals[3], jointMinimum(3), jointMaximum(3));
}

float autoSpeed = AUTO_SPEED_DEG_S;

void autoMoveTo(const float to[4], bool toLeader) {
  float now[4];
  autoCurrentTicks(now);
  float maxDeg = 0;
  for (uint8_t j = 0; j < 4; ++j) {
    autoFrom[j] = now[j];
    autoTo[j] = to[j];
    maxDeg = max(maxDeg, fabsf(to[j] - now[j]) * 0.08789f);
  }
  autoToLeader = toLeader;
  autoDur = max(0.3f, maxDeg / autoSpeed);
  autoProgress = 0;
}

// 보간한 목표를 targetGoal/qTarget에 넣음 (pair는 serviceArmMotion의 속도 제한을 거쳐 전송)
bool autoMoveStep(float dt) {
  autoProgress = min(1.0f, autoProgress + dt / autoDur);
  float s = autoProgress * autoProgress * (3.0f - 2.0f * autoProgress);
  float lead[4];
  if (autoToLeader) autoLeaderTicks(lead);       // 복귀 중 Leader가 움직여도 따라감
  float d[4];
  for (uint8_t j = 0; j < 4; ++j) d[j] = autoFrom[j] + ((autoToLeader ? lead[j] : autoTo[j]) - autoFrom[j]) * s;
  armWant[0] = clamp32(lroundf(d[0]), jointMinimum(0), jointMaximum(0));
  qTarget    = clamp32(lroundf(d[1]) - PAIR_ZERO_2, qMin, qMax);
  armWant[2] = clamp32(lroundf(d[2]), jointMinimum(2), jointMaximum(2));
  armWant[3] = clamp32(lroundf(d[3]), jointMinimum(3), jointMaximum(3));
  // 보간 끝 + pair가 이번 주기 안에 목표에 닿을 만큼 가까움 (정확히 같을 필요 없음:
  // Leader 값은 계속 미세하게 떨려서 '정확히 같음'을 기다리면 영원히 안 끝남)
  return autoProgress >= 1.0f && abs32(qTarget - qCommand) <= pairFollowStep;
}

void runAuto(float dt) {
  switch (autoState) {
    case AUTO_UP:
      if (autoMoveStep(dt)) {
        autoMoveTo(tStandBack, false); autoState = AUTO_TURN;
        if (DEBUG_SERIAL) DEBUG_SERIAL.println("[AUTO] turn back");
      }
      break;
    case AUTO_TURN:
      if (autoMoveStep(dt)) {
        autoMoveTo(tDrop, false); autoState = AUTO_DROP;
        if (DEBUG_SERIAL) DEBUG_SERIAL.println("[AUTO] lower to drop");
      }
      break;
    case AUTO_DROP:   if (autoMoveStep(dt)) { autoWaitStart = millis(); autoState = AUTO_SETTLE; } break;
    case AUTO_SETTLE:
      autoMoveStep(dt);
      if ((uint32_t)(millis() - autoWaitStart) >= AUTO_SETTLE_MS) {
        gripHoldActive = false;
        if (gripperReady) setGripTarget(gripClosedPos - GRIP_CLOSE_DIR * (GRIP_SPAN - GRIP_OPEN_MARGIN));
        gripLatchOpen = true;
        autoWaitStart = millis();
        autoState = AUTO_RELEASE;
        if (DEBUG_SERIAL) DEBUG_SERIAL.println("[AUTO] released");
      }
      break;
    case AUTO_RELEASE:
      autoMoveStep(dt);
      if ((uint32_t)(millis() - autoWaitStart) >= RELEASE_WAIT_MS) {
        autoSpeed = AUTO_SPEED_DEG_S;            // 다시 세우기·정면 회전은 빠르게
        autoMoveTo(tStandBack, false); autoState = AUTO_UP2;
        if (DEBUG_SERIAL) DEBUG_SERIAL.println("[AUTO] stand up");
      }
      break;
    case AUTO_UP2:
      if (autoMoveStep(dt)) {
        autoMoveTo(tStandFront, false); autoState = AUTO_TURN2;
        if (DEBUG_SERIAL) DEBUG_SERIAL.println("[AUTO] turn front");
      }
      break;
    case AUTO_TURN2:
      if (autoMoveStep(dt)) {
        float l[4]; autoLeaderTicks(l);
        autoSpeed = AUTO_RETURN_SPEED_DEG_S;     // Leader 자세로 내려가는 것만 천천히
        autoMoveTo(l, true); autoState = AUTO_RETURN;
        if (DEBUG_SERIAL) DEBUG_SERIAL.println("[AUTO] return to leader (slow)");
      }
      break;
    case AUTO_RETURN:
      autoMoveStep(dt);
      if (autoProgress >= 1.0f) {                 // 보간 끝나면 종료 → 텔레오퍼레이션이 이어받음
        autoState = AUTO_IDLE;
        lastArmPacketMs = millis();              // 복귀 직후 Bluetooth 타임아웃 오판 방지
        if (DEBUG_SERIAL) DEBUG_SERIAL.println("[AUTO] done -> following leader");
      }
      break;
    default: break;
  }
}

void onAutoButton() {
  if (autoState != AUTO_IDLE) {                  // 동작 중 → 중단하고 Leader 자세로 복귀
    bool back = autoState == AUTO_TURN || autoState == AUTO_DROP ||
                autoState == AUTO_SETTLE || autoState == AUTO_RELEASE;
    if (back) { autoSpeed = AUTO_SPEED_DEG_S; autoMoveTo(tStandBack, false); autoState = AUTO_UP2; }
    else if (autoState == AUTO_UP || autoState == AUTO_TURN2) {
      float l[4]; autoLeaderTicks(l);
      autoSpeed = AUTO_RETURN_SPEED_DEG_S; autoMoveTo(l, true); autoState = AUTO_RETURN;
    }
    if (DEBUG_SERIAL) DEBUG_SERIAL.println("[AUTO] aborted -> returning to leader");
    return;
  }
  if (!armReady || systemFault || armHomeActive || !pairSampleOK) return;
  if (!dropPoseOK) { if (DEBUG_SERIAL) DEBUG_SERIAL.println("[AUTO] drop pose unavailable -> ignored"); return; }
  autoSpeed = AUTO_SPEED_DEG_S;
  autoMoveTo(tStandFront, false);
  autoState = AUTO_UP;
  if (DEBUG_SERIAL) DEBUG_SERIAL.println("[AUTO] start");
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
  if (!CALIBRATION_MODE) setupDump();   // 트럭은 켜자마자 닫기 (ID7은 allTorqueOff 대상 아님)
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
  if (!homeHandoff)
    for (uint8_t i = 0; i < ARM_MOTOR_COUNT; ++i) armWant[i] = targetGoal[i];   // 첫 리더 값 전까지 현재 목표 유지
  computeAutoPoses();                   // 실제 관절 한계(EEPROM 포함)를 읽은 뒤에 계산
  bool leftOK = setupWheel(LEFT_ID);
  bool rightOK = setupWheel(RIGHT_ID);
  wheelsReady = leftOK && rightOK;
  if (wheelsReady) {
    if (!commandWheels(0, 0)) { tripFault("wheel startup stop", 9); return; }
    if (DEBUG_SERIAL) { DEBUG_SERIAL.print("[DRIVE] wheel max speed = "); DEBUG_SERIAL.println(wheelMax); }
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
    // 강제 복구: 잠금 3초 뒤 보드 전체 재시작 (전원 껐다 켠 것과 같음 → 홈 이동부터 다시)
    if ((uint32_t)(millis() - faultAtMs) >= AUTO_RESTART_DELAY_MS) {
      if (DEBUG_SERIAL) { DEBUG_SERIAL.println("[RESTART] rebooting OpenRB..."); DEBUG_SERIAL.flush(); }
      allTorqueOff();
      delay(50);
      NVIC_SystemReset();
    }
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
  if ((uint32_t)(millis() - lastDrive) >= 20) {   // 바퀴가 없어도 트럭·자동 투입 버튼은 동작
    lastDrive = millis(); rcControl();
  }
  if (systemFault) return;
  servicePairSafety();
  serviceOtherArmSafety();
  if (systemFault) return;
  pollArmLink();
  serviceGripHold();
  serviceDump();
  serviceRevive();
  serviceWheelHealth();
  serviceWheelRecovery();
  serviceArmMotion();
#if DEBUG
  static uint32_t lastPrint = 0;
  if (DEBUG_SERIAL && (uint32_t)(millis() - lastPrint) >= 250) {
    lastPrint = millis();
    DEBUG_SERIAL.print("turnCH4="); DEBUG_SERIAL.print(dbgTurn);
    DEBUG_SERIAL.print(" fwdCH2="); DEBUG_SERIAL.print(dbgFwd);
    DEBUG_SERIAL.print(" armed="); DEBUG_SERIAL.print(armed);
    DEBUG_SERIAL.print(" packets="); DEBUG_SERIAL.print(armPacketCount);
    DEBUG_SERIAL.print(" reject="); DEBUG_SERIAL.print(armRejectCount);
    DEBUG_SERIAL.print(" grip="); DEBUG_SERIAL.print(gripperReady ? "ready" : "disabled");
    DEBUG_SERIAL.print(" p2="); DEBUG_SERIAL.print(pairSample.pos2);
    DEBUG_SERIAL.print(" p6="); DEBUG_SERIAL.print(pairSample.pos6);
    DEBUG_SERIAL.print(" L2="); DEBUG_SERIAL.print(pairSample.load2);
    DEBUG_SERIAL.print(" L6="); DEBUG_SERIAL.print(pairSample.load6);
    DEBUG_SERIAL.print(" dump="); DEBUG_SERIAL.print(dumpReady ? (dumpOpen ? "OPEN" : "CLOSED") : "off");
    if (dumpReady) {
      DEBUG_SERIAL.print(" dpos="); DEBUG_SERIAL.print((int32_t)dxl.readControlTableItem(PRESENT_POSITION, DUMP_ID, IO_TIMEOUT_MS));
      DEBUG_SERIAL.print(" dload="); DEBUG_SERIAL.print((int32_t)dxl.readControlTableItem(PRESENT_CURRENT, DUMP_ID, IO_TIMEOUT_MS));
    }
    DEBUG_SERIAL.print(" auto="); DEBUG_SERIAL.print((int)autoState);
    DEBUG_SERIAL.print(" revives="); DEBUG_SERIAL.print(reviveCount);
    DEBUG_SERIAL.print(" wheelFails="); DEBUG_SERIAL.println(wheelFailCount);
  }
#endif
}
