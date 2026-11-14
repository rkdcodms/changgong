// ============================================================================
//  LinkTest_Robot.ino  -  BT-210 링크 시험 (로봇 쪽)  /  보드: OpenRB-150
//  SNU 창의공학설계 2026 ROBOCON · 블루투스 링크 시험 도구
// ----------------------------------------------------------------------------
//  하는 일
//    1) 조종기 쪽(LinkTest_Leader)이 20 ms마다 보내는 시험 줄을 BT-210(슬레이브)으로 받는다.
//         T,ssssss,mmmmmmmm,pad*CC\n      (ssssss = 줄 번호, mmmmmmmm = 조종기가 보낸 시각, CC = XOR 체크섬)
//    2) 체크섬이 맞으면 곧바로 응답 줄을 돌려보낸다(조종기가 왕복 시간과 손실을 잰다).
//         R,ssssss,mmmmmmmm,gggggg*CC\n   (gggggg = 이 보드가 지금까지 제대로 받은 줄 수 → 조종기가 손실을 '가는 길/오는 길'로 나눈다)
//    3) 이 보드 혼자서도 '가는 길(조종기 → 로봇)'을 잰다. 돌아가는 길이 나빠도 이쪽 숫자는 맞다.
//         - 1초에 받은 줄 수, 빠진 번호(손실), 체크섬 틀림, 형식 틀림(잘린 줄·이어 붙은 줄), 중복, 순서 뒤바뀜
//         - 제대로 된 줄과 줄 사이의 가장 긴 공백, 0.3초 넘는 공백 횟수(= 실제 로봇이라면 팔이 멈추는 횟수)
//         - 묵은 줄: 평소보다 0.3초 넘게 늦게 도착한 줄(끊긴 동안 모듈에 쌓였다가 한꺼번에 나온 줄. 실제 로봇이라면 지난 자세로 움직인다)
//         - 늦게 도착한 정도: 줄에 적힌 조종기 시각과 이 보드 시각의 차이가 '가장 빨리 온 줄'보다 얼마나 큰가(5초마다 새로 잰다)
//    4) 5초마다 한글 요약(이번 5초 + 누적)과 판정(합격 / 주의 / 불합격)을 USB 시리얼 모니터에 찍는다.
//
//  절대 멈추지 않는다
//    - 응답을 보낼 자리가 블루투스 송신 버퍼에 없으면 기다리지 않고 그 응답을 건너뛰고 센다('응답 건너뜀').
//    - 받은 글자는 정해진 크기의 버퍼에만 모은다(String 을 쓰지 않는다). 줄 시작 글자 'T' 가 오면 앞의 조각은 버리고
//      새 줄을 시작한다 → 다시 연결된 직후의 잘린 줄·깨진 글자는 '형식 틀림'으로 세고 다음 줄부터 정상으로 읽는다.
//    - USB 출력은 큐에 쌓았다가 PC가 받아 갈 수 있을 때만 내보낸다(시리얼 모니터 없이 로봇에 올려 두어도 된다).
//
//  연결
//    Serial2 = BT-210 슬레이브 57600 bps (보드의 4핀 UART 커넥터. 로봇 펌웨어·수업 예제의 BLT_Serial 과 같은 포트)
//    USB     = 결과 표시(시리얼 모니터). 없어도 동작한다(LED 로 링크 상태를 본다).
//    ★ 다이나믹셀은 쓰지 않는다. 다이나믹셀 전원을 켜지 않으므로 팔 모터에 힘이 없다(팔이 처진다).
//      시험 전에 팔을 접어 두거나 받쳐 둔다. 바퀴도 돌지 않는다.
//
//  USB 시리얼 모니터에서 보내는 명령 (한 글자)
//    r       = 누적을 지운다
//    0 ~ 9   = 시험 단계 번호를 정하고 누적을 지운다 (읽어보기.md 5장의 단계 번호)
//
//  고칠 곳은 바로 아래 "조정값" 블록 하나뿐이다. 조종기 쪽 LinkTest_Leader.ino 의 같은 이름 값과 맞춘다.
// ============================================================================

#include <Arduino.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#if !defined(ARDUINO_OpenRB)
  #error "보드가 OpenRB-150 이 아닙니다. 도구 > 보드 > OpenRB-150 을 고르세요."
#endif

// 포트 이름 (배선이 정해져 있어 바꾸지 않는다)
#define BT_SERIAL  Serial2   // BT-210 블루투스 모듈 (슬레이브)
#define USB_SERIAL Serial    // PC 시리얼 모니터

#define FIRMWARE_LABEL "LINKTEST_ROBOT_20261113"

// ============================================================================
//  ★★★ 조정값 ★★★   (한 줄에 값 하나, 숫자만 바꾼다)
// ============================================================================

// ---- 1. 통신 ----
constexpr uint32_t BT_BAUD            = 57600;  // BT-210 속도(bps). 모듈 기본값. 조종기 쪽 시험 스케치와 같아야 한다
constexpr uint32_t EXPECTED_PERIOD_MS = 20;     // 조종기 쪽 FRAME_PERIOD_MS 와 같은 값(초당 기대 줄 수를 보여 줄 때만 쓴다)
constexpr bool     ECHO_ENABLED       = true;   // false = 응답을 보내지 않는다(실제 로봇처럼 받기만 한다. 이때 조종기 쪽은 모두 '손실'로 나온다)

// ---- 2. 판정 기준 (우리가 정한 목표값이다. 공식 규격이 아니다. 읽어보기.md 4장) ----
constexpr uint32_t LOSS_LIMIT_X100  = 100;      // 빠진 번호가 이 값(0.01 % 단위) 이상이면 불합격. 100 = 1 %
constexpr uint32_t SILENCE_LIMIT_MS = 300;      // 제대로 된 줄이 이 시간(ms) 넘게 끊기면 불합격. 로봇 펌웨어의 LEADER_TIMEOUT_MS 와 같은 값
constexpr uint32_t STALE_LIMIT_MS   = 300;      // 평소보다 이 시간(ms) 넘게 늦게 도착한 줄은 '묵은 줄'(하나라도 있으면 불합격)

// ---- 3. 시간 ----
constexpr uint32_t REPORT_PERIOD_MS = 5000;     // 요약을 찍는 주기(ms). 5000 = 5초

// ---- 4. 표시와 보드 ----
constexpr bool PRINT_CSV       = false;         // true = 요약마다 엑셀에 붙여 넣기 좋은 "CSV,..." 줄도 찍는다(읽어보기.md 3장)
constexpr bool LED_ACTIVE_HIGH = true;          // 보드 LED가 반대로 보이면 false
constexpr bool USB_TX_CHECK    = true;          // 시리얼 모니터에 아무것도 안 나오면 false 로 (USB 송신 상태 확인을 끈다)

// ============================================================================
//  컴파일할 때 자동으로 하는 검사
// ============================================================================
static_assert(BT_BAUD >= 9600 && BT_BAUD <= 1000000, "BT_BAUD 는 9600 ~ 1000000 이어야 합니다 (BT-210 기본값 57600)");
static_assert(EXPECTED_PERIOD_MS >= 10 && EXPECTED_PERIOD_MS <= 200, "EXPECTED_PERIOD_MS 는 10 ~ 200 ms 여야 합니다");
static_assert(LOSS_LIMIT_X100 >= 1 && LOSS_LIMIT_X100 <= 5000, "LOSS_LIMIT_X100 은 1 ~ 5000 (0.01 % ~ 50 %) 이어야 합니다");
static_assert(SILENCE_LIMIT_MS >= 2 * EXPECTED_PERIOD_MS && SILENCE_LIMIT_MS <= 10000,
              "SILENCE_LIMIT_MS 는 전송 주기의 2배 이상, 10000 ms 이하여야 합니다");
static_assert(STALE_LIMIT_MS >= 50 && STALE_LIMIT_MS <= 5000, "STALE_LIMIT_MS 는 50 ~ 5000 ms 여야 합니다");
static_assert(REPORT_PERIOD_MS >= 1000 && REPORT_PERIOD_MS <= 10000,
              "REPORT_PERIOD_MS 는 1000 ~ 10000 ms 여야 합니다 (5초 구간의 도착 시각을 담는 칸 수가 정해져 있다)");

// 아두이노 IDE는 .ino 안의 함수 선언을 자동으로 파일 위쪽에 끼워 넣는데, 그러면 아래 구조체보다
// 선언이 먼저 나와 컴파일이 깨질 수 있다. 그래서 전체를 한 묶음(namespace)으로 감싼다. 지우지 말 것.
namespace linkrobot {

// ---------------------------------------------------------------------------
//  줄 형식 (조종기 쪽 LinkTest_Leader.ino 와 같은 약속. 한쪽을 고치면 다른 쪽도 고친다)
// ---------------------------------------------------------------------------
constexpr uint32_t SEQ_MOD        = 1000000UL;    // 줄 번호는 0 ~ 999999 를 돈다(6자리)
constexpr char     TEST_START     = 'T';          // 시험 줄의 첫 글자 (16진 숫자 A~F 와 겹치지 않는 글자)
constexpr char     REPLY_START    = 'R';          // 응답 줄의 첫 글자
constexpr uint8_t  HEAD_LEN       = 18;           // "T,ssssss,mmmmmmmm," 의 길이
constexpr uint8_t  LINE_BUF_SIZE  = 64;           // 한 줄 버퍼 크기
constexpr uint8_t  RX_LINE_MAX    = 48;           // 받는 줄의 최대 글자 수('\n' 제외). 넘으면 그 줄을 버린다

constexpr uint8_t PARSE_OK     = 0;   // 맞는 줄
constexpr uint8_t PARSE_SUM    = 1;   // 꼴은 맞는데 체크섬이 틀림
constexpr uint8_t PARSE_FORMAT = 2;   // 꼴이 틀림(잘린 줄, 이어 붙은 줄, 이상한 글자)

// 번호 판단
constexpr uint32_t REORDER_MAX     = 50;     // 마지막 번호보다 이만큼 이하로 앞선 번호가 오면 '순서 뒤바뀜', 더 앞서면 조종기 재시작으로 본다
constexpr uint32_t RESTART_MS_BACK = 2000;   // 줄에 적힌 조종기 시각이 이만큼(ms) 넘게 거꾸로 가면 조종기 보드가 다시 켜진 것으로 본다

// 늦게 도착한 정도
constexpr uint16_t OFF_CAP  = 600;                     // 5초 구간 동안 도착 시각 차이를 담아 두는 칸 수(50 Hz x 10초 + 여유)
constexpr uint16_t JIT_BINS = 1001;                    // 막대그래프 칸 수(1 ms 한 칸, 0 ~ 1000 ms. 넘으면 마지막 칸)
static_assert((uint32_t)OFF_CAP * EXPECTED_PERIOD_MS >= REPORT_PERIOD_MS + 1000,
              "OFF_CAP 이 5초 구간의 줄 수보다 작습니다");

// ---------------------------------------------------------------------------
//  글자 다루기 (String 과 printf 를 쓰지 않고 정해진 버퍼에 직접 적는다)
// ---------------------------------------------------------------------------

// 글자 하나를 적는다. 자리가 모자라면 cap 을 돌려준다.
static uint8_t appendChar(char *out, uint8_t cap, uint8_t pos, char c) {
  if (pos >= cap) return cap;
  out[pos] = c;
  return (uint8_t)(pos + 1);
}

// 0 이상 정수를 width 자리 10진수(앞을 0으로 채움)로 적는다. 넘치는 윗자리는 버린다.
static uint8_t appendFixed(char *out, uint8_t cap, uint8_t pos, uint32_t value, uint8_t width) {
  if ((uint16_t)pos + width > cap) return cap;
  for (uint8_t i = 0; i < width; ++i) {
    out[pos + width - 1 - i] = (char)('0' + (value % 10));
    value /= 10;
  }
  return (uint8_t)(pos + width);
}

// 0 ~ 15 → '0'~'9', 'A'~'F'
static char hexDigit(uint8_t v) {
  return (char)((v < 10) ? ('0' + v) : ('A' + (v - 10)));
}

// '0'~'9', 'A'~'F', 'a'~'f' → 0 ~ 15. 16진 글자가 아니면 -1.
static int8_t hexValue(char c) {
  if (c >= '0' && c <= '9') return (int8_t)(c - '0');
  if (c >= 'A' && c <= 'F') return (int8_t)(c - 'A' + 10);
  if (c >= 'a' && c <= 'f') return (int8_t)(c - 'a' + 10);
  return -1;
}

// 앞에서부터 length 글자를 모두 XOR 한 값(체크섬). 조종기 펌웨어의 xorChecksum 과 같다.
static uint8_t xorChecksum(const char *text, uint8_t length) {
  uint8_t sum = 0;
  for (uint8_t i = 0; i < length; ++i) {
    sum = (uint8_t)(sum ^ (uint8_t)text[i]);
  }
  return sum;
}

// text[pos] 부터 width 자리 10진수를 읽는다. 숫자가 아닌 글자가 있으면 false.
static bool readFixed(const char *text, uint8_t pos, uint8_t width, uint32_t &value) {
  uint32_t v = 0;
  for (uint8_t i = 0; i < width; ++i) {
    const char c = text[pos + i];
    if (c < '0' || c > '9') return false;
    v = v * 10 + (uint32_t)(c - '0');
  }
  value = v;
  return true;
}

// 줄 끝의 "*CC" 를 확인한다. ('\n' 은 이미 떼어 낸 줄)
static uint8_t checkTail(const char *line, uint8_t len) {
  if (len < 4 || line[len - 3] != '*') return PARSE_FORMAT;
  const int8_t hi = hexValue(line[len - 2]);
  const int8_t lo = hexValue(line[len - 1]);
  if (hi < 0 || lo < 0) return PARSE_FORMAT;
  if (xorChecksum(line, (uint8_t)(len - 3)) != (uint8_t)((hi << 4) | lo)) return PARSE_SUM;
  return PARSE_OK;
}

// 시험 줄 "T,ssssss,mmmmmmmm,pad*CC" 을 해석한다. (pad 길이는 정해 두지 않는다: 조종기 쪽 LINE_BYTES 와 상관없이 읽는다)
static uint8_t parseTestLine(const char *line, uint8_t len, uint32_t &seq, uint32_t &ms) {
  const uint8_t tail = checkTail(line, len);
  if (tail != PARSE_OK) return tail;
  const uint8_t body = (uint8_t)(len - 3);              // '*' 앞까지의 글자 수
  if (body < HEAD_LEN) return PARSE_FORMAT;
  if (line[0] != TEST_START || line[1] != ',' || line[8] != ',' || line[17] != ',') return PARSE_FORMAT;
  if (!readFixed(line, 2, 6, seq) || !readFixed(line, 9, 8, ms)) return PARSE_FORMAT;
  for (uint8_t i = HEAD_LEN; i < body; ++i) {
    if (line[i] < 'a' || line[i] > 'z') return PARSE_FORMAT;
  }
  return PARSE_OK;
}

// 응답 줄 "R,ssssss,mmmmmmmm,gggggg*CC\n" 을 만든다. 길이를 돌려준다(실패하면 0). 끝에 '\0' 을 붙인다.
static uint8_t buildReply(char *out, uint8_t cap, uint32_t seq, uint32_t ms, uint32_t good) {
  uint8_t pos = 0;
  pos = appendChar(out, cap, pos, REPLY_START);
  pos = appendChar(out, cap, pos, ',');
  pos = appendFixed(out, cap, pos, seq, 6);
  pos = appendChar(out, cap, pos, ',');
  pos = appendFixed(out, cap, pos, ms, 8);
  pos = appendChar(out, cap, pos, ',');
  pos = appendFixed(out, cap, pos, good % SEQ_MOD, 6);
  if (pos >= cap) return 0;
  const uint8_t checksum = xorChecksum(out, pos);
  pos = appendChar(out, cap, pos, '*');
  pos = appendChar(out, cap, pos, hexDigit((uint8_t)(checksum >> 4)));
  pos = appendChar(out, cap, pos, hexDigit((uint8_t)(checksum & 0x0F)));
  pos = appendChar(out, cap, pos, '\n');
  if (pos >= cap) return 0;
  out[pos] = '\0';
  return pos;
}

// ---------------------------------------------------------------------------
//  받은 글자를 한 줄로 모으기 (조종기 쪽과 같은 방식. 시작 글자만 다르다)
// ---------------------------------------------------------------------------
constexpr uint8_t RX_NONE     = 0;   // 아직 줄이 끝나지 않았다
constexpr uint8_t RX_READY    = 1;   // buf[0..lineLen-1] 에 한 줄이 완성됐다
constexpr uint8_t RX_FRAGMENT = 2;   // 모으던 줄을 버렸다(시작 글자가 다시 옴 / 이상한 글자 / 너무 긴 줄). frag 에 그 조각이 있다
constexpr uint8_t RX_JUNK     = 3;   // 줄 밖의 글자(시작 글자 전에 온 글자)

struct LineRx {
  char    buf[LINE_BUF_SIZE];
  char    frag[LINE_BUF_SIZE];   // 방금 버린 조각(표시용)
  uint8_t len;                   // 지금 모으는 중인 글자 수(0 = 줄 밖)
  uint8_t lineLen;               // 방금 완성된 줄의 길이(RX_READY 일 때)
};

static void rxKeepFragment(LineRx &r) {
  memcpy(r.frag, r.buf, r.len);
  r.frag[r.len] = '\0';
}

static uint8_t rxFeed(LineRx &r, char c, char startChar) {
  if (c == startChar) {
    const bool hadPart = r.len > 0;
    if (hadPart) rxKeepFragment(r);
    r.buf[0] = c;
    r.len = 1;
    return hadPart ? RX_FRAGMENT : RX_NONE;
  }
  if (r.len == 0) {
    return (c == '\r' || c == '\n') ? RX_NONE : RX_JUNK;   // 줄 밖
  }
  if (c == '\n') {
    r.buf[r.len] = '\0';
    r.lineLen = r.len;
    r.len = 0;
    return RX_READY;
  }
  if (c == '\r') return RX_NONE;                            // CR 은 무시
  if (c < 32 || c > 126 || r.len >= RX_LINE_MAX) {         // 이상한 글자 또는 너무 긴 줄 → 버리고 줄 밖으로
    rxKeepFragment(r);
    r.len = 0;
    return RX_FRAGMENT;
  }
  r.buf[r.len] = c;
  r.len = (uint8_t)(r.len + 1);
  return RX_NONE;
}

// ---------------------------------------------------------------------------
//  통계 (5초 구간 하나, 누적 하나)
// ---------------------------------------------------------------------------
struct Stats {
  uint32_t startMs;        // 이 통계를 시작한 시각
  uint32_t good;           // 체크섬이 맞은 시험 줄(중복 포함)
  uint32_t missing;        // 빠진 번호(가는 길 손실)
  uint32_t dup;            // 같은 번호가 또 옴
  uint32_t reorder;        // 앞선 번호가 늦게 옴
  uint32_t badSum;         // 체크섬이 틀린 줄
  uint32_t badFormat;      // 꼴이 틀린 줄(잘린 줄, 이어 붙은 줄 포함)
  uint32_t junk;           // 줄 밖 글자 수(바이트)
  uint32_t echoSkipped;    // 송신 버퍼가 모자라 보내지 못한 응답
  uint32_t stale;          // 묵은 줄(평소보다 STALE_LIMIT_MS 넘게 늦게 도착)
  uint32_t staleMax;       // 묵은 줄이 평소보다 늦은 가장 큰 값(ms)
  uint32_t maxGap;         // 제대로 된 줄 사이의 가장 긴 공백(ms)
  uint32_t silenceCount;   // SILENCE_LIMIT_MS 넘는 공백 횟수
  uint32_t silenceMs;      // 그런 공백의 합(ms)
  uint32_t restarts;       // 조종기 보드가 다시 켜진 것으로 보인 횟수
  uint32_t jitCount;       // 늦게 도착한 정도 막대그래프에 든 줄 수
  uint32_t jitMax;         // 늦게 도착한 정도의 최대(ms)
  uint32_t jitP95;         // (5초 구간만) 늦게 도착한 정도의 95 % 값(ms)
  uint32_t windows;        // (누적만) 5초 구간 수
  uint32_t failedWindows;  // (누적만) 불합격 구간 수
};

constexpr uint8_t VERDICT_PASS = 0;   // 합격
constexpr uint8_t VERDICT_WARN = 1;   // 주의
constexpr uint8_t VERDICT_FAIL = 2;   // 불합격

// ---- 상태 변수 ----
static Stats    win;                       // 이번 5초
static Stats    tot;                       // 누적('r' 이나 단계 번호를 보내면 지운다)
static LineRx   rx;
static char     txBuf[LINE_BUF_SIZE];
static uint32_t goodEver = 0;              // 부팅 뒤 제대로 받은 줄 수(응답 줄에 실어 보낸다. 지우지 않는다)

static bool     haveSeq = false;           // 번호 기준이 있다
static uint32_t lastSeq = 0;               // 가장 나중에 받은 번호
static uint32_t lastLeaderMs = 0;          // 그 줄에 적힌 조종기 시각

static int32_t  winOff[OFF_CAP];           // 이번 5초 동안 받은 줄의 (이 보드 시각 - 조종기 시각)
static uint16_t winOffCount = 0;
static int32_t  winMinOff = INT32_MAX;     // 그 가운데 가장 작은 값(= 가장 빨리 온 줄)
static bool     baseValid = false;         // 지난 5초의 가장 작은 값이 있다
static int32_t  baseOff = 0;
static uint16_t jitHist[JIT_BINS];         // 5초 구간 계산용(요약할 때만 채운다)
static uint32_t totJitHist[JIT_BINS];      // 누적

static bool     everGood = false;          // 부팅 뒤 제대로 된 줄을 한 번이라도 받았다
static uint32_t bootMs = 0;
static uint32_t firstGoodMs = 0;           // 부팅 뒤 첫 줄까지 걸린 시간(ms)
static uint32_t lastGoodMs = 0;            // 마지막으로 제대로 된 줄을 받은 시각
static bool     inOutage = false;          // 지금 SILENCE_LIMIT_MS 넘게 줄이 없다
static uint32_t longestOutageMs = 0;       // 부팅 뒤 가장 길었던 끊김(지우지 않는다)

static uint32_t lastReportMs = 0;
static uint32_t reportCount = 0;
static uint8_t  testStep = 0;

static bool     staleWarnPending = false;
static uint32_t staleWarnCount = 0;
static uint32_t staleWarnMax = 0;
static uint32_t lastStaleWarnMs = 0;
static char     lastBadLine[LINE_BUF_SIZE];
static bool     badLineNew = false;
static char     junkTail[25];
static uint8_t  junkLen = 0;
static bool     junkNew = false;

static bool     usbWasOpen = false;
static bool     bannerPending = false;
static bool     ledIsOn = false;

// ============================================================================
//  USB 표시 - 절대 멈추지 않게 만든 출력 (조종기 펌웨어와 같은 방식)
// ============================================================================
constexpr uint16_t DBG_QUEUE_SIZE = 4096;   // 2의 거듭제곱
constexpr uint16_t DBG_LINE_MAX   = 900;    // 한 줄 최대 바이트(한글은 글자당 3바이트)
constexpr uint8_t  USB_CHUNK_MAX  = 63;     // USB로 한 번에 내보내는 최대 바이트(64 미만이면 기다림이 없다)

static_assert((DBG_QUEUE_SIZE & (DBG_QUEUE_SIZE - 1)) == 0, "DBG_QUEUE_SIZE 는 2의 거듭제곱이어야 합니다");

static char     dbgQueue[DBG_QUEUE_SIZE];
static uint16_t dbgHead = 0;
static uint16_t dbgTail = 0;
static char     lineBuf[DBG_LINE_MAX];
static uint16_t lineLen = 0;
static uint32_t lastUsbWriteMs = 0;

// PC에서 시리얼 모니터를 열었는가. (if (Serial) 과 같은 뜻이지만 10 ms 지연이 없다)
static bool usbTerminalOpen() {
  return USB_SERIAL.dtr() || USB_SERIAL.rts();
}

// 앞서 내보낸 USB 데이터를 PC가 아직 가져가지 않았는가.
static bool usbTxBusy(uint32_t now) {
  if (!USB_TX_CHECK) return (uint32_t)(now - lastUsbWriteMs) < 3;
  for (uint8_t ep = 1; ep < 8; ++ep) {
    if (USB->DEVICE.DeviceEndpoint[ep].EPCFG.bit.EPTYPE1 == 3) {        // 3 = PC 쪽으로 보내는 통로(USB 시리얼 송신)
      return USB->DEVICE.DeviceEndpoint[ep].EPSTATUS.bit.BK1RDY != 0;   // 1 = 아직 PC가 안 가져갔다
    }
  }
  return true;
}

static uint16_t dbgUsed() {
  return (uint16_t)((dbgHead - dbgTail) & (DBG_QUEUE_SIZE - 1));
}

static void dbgPush(const char *text, uint16_t length) {
  if (!usbTerminalOpen()) return;
  if (length > (uint16_t)(DBG_QUEUE_SIZE - 1 - dbgUsed())) return;
  for (uint16_t i = 0; i < length; ++i) {
    dbgQueue[dbgHead] = text[i];
    dbgHead = (uint16_t)((dbgHead + 1) & (DBG_QUEUE_SIZE - 1));
  }
}

static void dbgPump(uint32_t now) {
  if (dbgHead == dbgTail) return;
  if (!usbTerminalOpen()) {
    dbgHead = 0;
    dbgTail = 0;
    return;
  }
  if (usbTxBusy(now)) return;
  const uint16_t available = dbgUsed();
  uint16_t count = (available < USB_CHUNK_MAX) ? available : USB_CHUNK_MAX;
  if (count < available) {
    // 한글을 중간에서 자르지 않는다
    while (count > 1 && (((uint8_t)dbgQueue[(dbgTail + count) & (DBG_QUEUE_SIZE - 1)]) & 0xC0) == 0x80) {
      count = (uint16_t)(count - 1);
    }
  }
  uint8_t chunk[USB_CHUNK_MAX];
  for (uint16_t i = 0; i < count; ++i) {
    chunk[i] = (uint8_t)dbgQueue[(dbgTail + i) & (DBG_QUEUE_SIZE - 1)];
  }
  const size_t written = USB_SERIAL.write(chunk, count);
  lastUsbWriteMs = now;
  if (written != count) return;
  dbgTail = (uint16_t)((dbgTail + count) & (DBG_QUEUE_SIZE - 1));
}

static void lineStart() {
  lineLen = 0;
  lineBuf[0] = '\0';
}

static void lineAddV(const char *format, va_list args) {
  if (lineLen >= DBG_LINE_MAX - 3) return;
  const uint16_t room = (uint16_t)(DBG_LINE_MAX - 3 - lineLen);
  const int n = vsnprintf(lineBuf + lineLen, (size_t)room + 1, format, args);
  if (n <= 0) return;
  lineLen = (uint16_t)(lineLen + (((uint16_t)n > room) ? room : (uint16_t)n));
}

static void lineAdd(const char *format, ...) __attribute__((format(printf, 1, 2)));
static void lineAdd(const char *format, ...) {
  va_list args;
  va_start(args, format);
  lineAddV(format, args);
  va_end(args);
}

static void lineEnd() {
  lineBuf[lineLen] = '\r';
  lineBuf[lineLen + 1] = '\n';
  dbgPush(lineBuf, (uint16_t)(lineLen + 2));
  lineLen = 0;
}

static void dbgEvent(const char *format, ...) __attribute__((format(printf, 1, 2)));
static void dbgEvent(const char *format, ...) {
  va_list args;
  lineStart();
  va_start(args, format);
  lineAddV(format, args);
  va_end(args);
  lineEnd();
}

static void lineAddPct(uint32_t num, uint32_t den) {
  if (den == 0) {
    lineAdd("-");
    return;
  }
  const uint32_t x100 = (uint32_t)(((uint64_t)num * 10000u + den / 2) / den);
  lineAdd("%lu.%02lu%%", (unsigned long)(x100 / 100), (unsigned long)(x100 % 100));
}

static void lineAddSec(uint32_t ms) {
  lineAdd("%lu.%lu초", (unsigned long)(ms / 1000), (unsigned long)((ms % 1000) / 100));
}

static void lineAddElapsed(uint32_t ms) {
  const uint32_t s = ms / 1000;
  lineAdd("%lu분%02lu초", (unsigned long)(s / 60), (unsigned long)(s % 60));
}

// ============================================================================
//  통계 계산
// ============================================================================
static void statsClear(Stats &s, uint32_t now) {
  memset(&s, 0, sizeof(s));
  s.startMs = now;
}

// 막대그래프에서 pct % 값(올림 순위)을 찾는다.
static uint32_t percentile32(const uint32_t *hist, uint32_t count, uint32_t pct) {
  if (count == 0) return 0;
  uint32_t rank = (uint32_t)(((uint64_t)count * pct + 99) / 100);
  if (rank < 1) rank = 1;
  uint32_t cum = 0;
  for (uint16_t i = 0; i < JIT_BINS; ++i) {
    cum += hist[i];
    if (cum >= rank) return i;
  }
  return JIT_BINS - 1;
}

static uint32_t percentile16(const uint16_t *hist, uint32_t count, uint32_t pct) {
  if (count == 0) return 0;
  uint32_t rank = (uint32_t)(((uint64_t)count * pct + 99) / 100);
  if (rank < 1) rank = 1;
  uint32_t cum = 0;
  for (uint16_t i = 0; i < JIT_BINS; ++i) {
    cum += hist[i];
    if (cum >= rank) return i;
  }
  return JIT_BINS - 1;
}

// 받은 줄 수(중복 뺌)와 빠진 번호로 손실 비율의 분모를 만든다.
static uint32_t lossDen(const Stats &s) {
  const uint32_t unique = (s.good > s.dup) ? (s.good - s.dup) : 0;
  return unique + s.missing;
}

// 판정. 까닭을 why 에 적는다(합격이면 빈 글).
static uint8_t judge(const Stats &s, char *why, size_t cap) {
  uint8_t verdict = VERDICT_PASS;
  size_t used = 0;
  why[0] = '\0';
  #define WHY_ADD(...) do { if (used < cap) { const int n_ = snprintf(why + used, cap - used, __VA_ARGS__); \
                            if (n_ > 0) used += (size_t)n_; if (used >= cap) used = cap - 1; } } while (0)
  if (s.good == 0) {
    WHY_ADD("받은 줄 없음");
    verdict = VERDICT_FAIL;
  } else {
    const uint32_t den = lossDen(s);
    if (den > 0 && (uint64_t)s.missing * 10000u >= (uint64_t)LOSS_LIMIT_X100 * den) {
      const uint32_t x100 = (uint32_t)(((uint64_t)s.missing * 10000u + den / 2) / den);
      WHY_ADD("%s빠진 번호 %lu.%02lu%%", used ? ", " : "", (unsigned long)(x100 / 100), (unsigned long)(x100 % 100));
      verdict = VERDICT_FAIL;
    }
  }
  if (s.maxGap > SILENCE_LIMIT_MS) {
    WHY_ADD("%s공백 %lu ms", used ? ", " : "", (unsigned long)s.maxGap);
    verdict = VERDICT_FAIL;
  }
  if (s.stale > 0) {
    WHY_ADD("%s묵은 줄 %lu", used ? ", " : "", (unsigned long)s.stale);
    verdict = VERDICT_FAIL;
  }
  if (verdict == VERDICT_PASS) {
    if (s.badSum > 0) WHY_ADD("%s체크섬 틀림 %lu", used ? ", " : "", (unsigned long)s.badSum);
    if (s.badFormat > 0) WHY_ADD("%s형식 틀림 %lu", used ? ", " : "", (unsigned long)s.badFormat);
    if (s.reorder > 0) WHY_ADD("%s순서 뒤바뀜 %lu", used ? ", " : "", (unsigned long)s.reorder);
    if (s.dup > 0) WHY_ADD("%s중복 %lu", used ? ", " : "", (unsigned long)s.dup);
    if (s.echoSkipped > 0) WHY_ADD("%s응답 건너뜀 %lu", used ? ", " : "", (unsigned long)s.echoSkipped);
    if (used > 0) verdict = VERDICT_WARN;
  }
  #undef WHY_ADD
  return verdict;
}

static const char *verdictText(uint8_t v) {
  return (v == VERDICT_PASS) ? "합격" : (v == VERDICT_WARN) ? "주의" : "불합격";
}

// ============================================================================
//  블루투스: 받고 곧바로 응답하기
// ============================================================================

// 응답 줄을 보낸다. 송신 버퍼에 자리가 없으면 그 응답은 건너뛴다(절대 기다리지 않는다).
static void sendReply(uint32_t seq, uint32_t leaderMs) {
  const uint8_t len = buildReply(txBuf, LINE_BUF_SIZE, seq, leaderMs, goodEver);
  if (len == 0) return;
  if (BT_SERIAL.availableForWrite() < (int)len) {
    ++win.echoSkipped;
    ++tot.echoSkipped;
    return;
  }
  BT_SERIAL.write((const uint8_t *)txBuf, len);
}

// 줄 번호를 확인한다. 이번 줄 앞에서 빠진 번호 수를 돌려준다. restart = 조종기 보드가 다시 켜진 것으로 보인다.
static uint32_t checkSequence(uint32_t seq, uint32_t leaderMs, bool &restart) {
  restart = false;
  if (!haveSeq) {                                       // 첫 줄: 기준만 잡는다
    haveSeq = true;
    lastSeq = seq;
    lastLeaderMs = leaderMs;
    return 0;
  }
  const bool msBack = leaderMs + RESTART_MS_BACK < lastLeaderMs;   // 조종기 시각이 크게 거꾸로 갔다
  const uint32_t d = (seq + SEQ_MOD - lastSeq) % SEQ_MOD;          // 마지막 번호에서 몇 번 앞으로 갔나
  uint32_t missed = 0;
  if (!msBack && d == 0) {
    ++win.dup;
    ++tot.dup;
  } else if (!msBack && d < SEQ_MOD / 2) {
    missed = d - 1;                                     // 그 사이 번호는 오지 않았다
    win.missing += missed;
    tot.missing += missed;
    lastSeq = seq;
    lastLeaderMs = leaderMs;
  } else if (!msBack && (SEQ_MOD - d) <= REORDER_MAX) {
    ++win.reorder;                                      // 앞선 번호가 늦게 왔다: 빠진 번호로 셌던 것을 하나 되돌린다
    ++tot.reorder;
    if (win.missing > 0) --win.missing;
    if (tot.missing > 0) --tot.missing;
  } else {
    restart = true;                                     // 번호·시각이 처음부터 다시 시작했다
    ++win.restarts;
    ++tot.restarts;
    lastSeq = seq;
    lastLeaderMs = leaderMs;
  }
  return missed;
}

// 늦게 도착한 정도를 잰다. (이 보드 시각 - 줄에 적힌 조종기 시각)은 두 보드 시계 차이 + 전달 시간이다.
//   시계 차이는 거의 그대로이므로, '가장 빨리 온 줄'보다 큰 만큼이 그 줄이 더 늦게 온 시간이다.
static void noteDelay(uint32_t leaderMs, uint32_t now, bool restart) {
  const int32_t off = (int32_t)(now - leaderMs);
  if (restart) {                                        // 조종기 시계가 처음부터 다시 시작했다 → 기준을 버린다
    winOffCount = 0;
    winMinOff = INT32_MAX;
    baseValid = false;
  }
  if (off < winMinOff) winMinOff = off;
  if (winOffCount < OFF_CAP) {
    winOff[winOffCount] = off;
    winOffCount = (uint16_t)(winOffCount + 1);
  }
  const int32_t ref = (baseValid && baseOff < winMinOff) ? baseOff : winMinOff;
  const uint32_t extra = (uint32_t)(off - ref);
  if (extra > STALE_LIMIT_MS) {
    ++win.stale;
    ++tot.stale;
    if (extra > win.staleMax) win.staleMax = extra;
    if (extra > tot.staleMax) tot.staleMax = extra;
    staleWarnPending = true;
    ++staleWarnCount;
    if (extra > staleWarnMax) staleWarnMax = extra;
  }
}

// 제대로 된 줄 사이의 공백을 잰다 / 끊김 복구를 알린다.
static void noteArrival(uint32_t now, uint32_t seq, uint32_t missed, bool restart) {
  if (everGood) {
    const uint32_t gap = (uint32_t)(now - lastGoodMs);
    if (gap > win.maxGap) win.maxGap = gap;
    if (gap > tot.maxGap) tot.maxGap = gap;
    if (inOutage) {
      inOutage = false;
      win.silenceMs += gap;
      tot.silenceMs += gap;
      if (gap > longestOutageMs) longestOutageMs = gap;
      if (restart) {
        dbgEvent("[복구] 조종기 줄 다시 옴: %lu.%lu초 끊겼음 (조종기 보드가 다시 켜졌음, 지금 번호 %06lu)",
                 (unsigned long)(gap / 1000), (unsigned long)((gap % 1000) / 100), (unsigned long)seq);
      } else {
        dbgEvent("[복구] 조종기 줄 다시 옴: %lu.%lu초 끊겼음 (그동안 빠진 번호 %lu개)",
                 (unsigned long)(gap / 1000), (unsigned long)((gap % 1000) / 100), (unsigned long)missed);
      }
    } else if (restart) {
      dbgEvent("[알림] 조종기 보드가 다시 켜진 것으로 보입니다 (줄 번호·시각이 처음부터 다시 시작, 지금 번호 %06lu)",
               (unsigned long)seq);
    }
  } else {
    everGood = true;
    firstGoodMs = (uint32_t)(now - bootMs);
    dbgEvent("[알림] 첫 줄 받음: 부팅 뒤 %lu.%lu초 (번호 %06lu) → 링크가 이어졌습니다",
             (unsigned long)(firstGoodMs / 1000), (unsigned long)((firstGoodMs % 1000) / 100), (unsigned long)seq);
  }
  lastGoodMs = now;
}

// 체크섬이 맞은 시험 줄 하나를 처리한다.
static void handleTestLine(uint32_t seq, uint32_t leaderMs, uint32_t now) {
  ++win.good;
  ++tot.good;
  goodEver = (goodEver + 1) % SEQ_MOD;
  if (ECHO_ENABLED) sendReply(seq, leaderMs);           // 1) 가장 먼저 응답한다(왕복 시간에 이 보드의 계산 시간이 끼지 않게)
  bool restart = false;
  const uint32_t missed = checkSequence(seq, leaderMs, restart);   // 2) 번호
  noteDelay(leaderMs, now, restart);                    // 3) 늦게 도착한 정도
  noteArrival(now, seq, missed, restart);               // 4) 공백
}

static void keepBadLine(const char *text) {
  strncpy(lastBadLine, text, sizeof(lastBadLine) - 1);
  lastBadLine[sizeof(lastBadLine) - 1] = '\0';
  badLineNew = true;
}

static void keepJunk(char c) {
  const char shown = (c >= 32 && c <= 126) ? c : '.';
  if (junkLen < sizeof(junkTail) - 1) {
    junkTail[junkLen] = shown;
    junkLen = (uint8_t)(junkLen + 1);
  } else {
    memmove(junkTail, junkTail + 1, sizeof(junkTail) - 2);
    junkTail[sizeof(junkTail) - 2] = shown;
  }
  junkTail[junkLen] = '\0';
  junkNew = true;
}

// 블루투스로 들어온 글자를 읽는다. 한 번에 읽는 글자 수를 정해 두어 루프가 오래 붙잡히지 않게 한다.
static void btPoll() {
  uint8_t budget = 128;
  while (budget > 0 && BT_SERIAL.available() > 0) {
    budget = (uint8_t)(budget - 1);
    const char c = (char)BT_SERIAL.read();
    const uint8_t r = rxFeed(rx, c, TEST_START);
    if (r == RX_JUNK) {
      ++win.junk;
      ++tot.junk;
      keepJunk(c);
      continue;
    }
    if (r == RX_FRAGMENT) {
      ++win.badFormat;
      ++tot.badFormat;
      keepBadLine(rx.frag);
      continue;
    }
    if (r != RX_READY) continue;
    uint32_t seq = 0;
    uint32_t ms = 0;
    const uint8_t p = parseTestLine(rx.buf, rx.lineLen, seq, ms);
    if (p == PARSE_SUM) {
      ++win.badSum;
      ++tot.badSum;
      keepBadLine(rx.buf);
      continue;
    }
    if (p != PARSE_OK) {
      ++win.badFormat;
      ++tot.badFormat;
      keepBadLine(rx.buf);
      continue;
    }
    handleTestLine(seq, ms, millis());
  }
}

// 제대로 된 줄이 SILENCE_LIMIT_MS 넘게 없으면 '끊김'으로 본다.
static void silenceTask(uint32_t now) {
  if (!everGood || inOutage) return;
  if ((uint32_t)(now - lastGoodMs) > SILENCE_LIMIT_MS) {
    inOutage = true;
    ++win.silenceCount;
    ++tot.silenceCount;
    dbgEvent("[경고] 조종기 줄 끊김: %lu ms 넘게 제대로 된 줄이 없습니다 (마지막 번호 %06lu) → 실제 로봇이라면 팔이 제자리에 멈춘다",
             (unsigned long)SILENCE_LIMIT_MS, (unsigned long)lastSeq);
  }
}

// 묵은 줄이 왔으면 1초에 한 번만 알린다.
static void staleWarnTask(uint32_t now) {
  if (!staleWarnPending || (uint32_t)(now - lastStaleWarnMs) < 1000) return;
  staleWarnPending = false;
  lastStaleWarnMs = now;
  dbgEvent("[경고] 묵은 줄 %lu개: 평소보다 최대 %lu ms 늦게 도착 (끊긴 동안 모듈에 쌓였다가 나온 줄로 보임 → 실제 로봇이라면 지난 자세로 움직인다)",
           (unsigned long)staleWarnCount, (unsigned long)staleWarnMax);
  staleWarnCount = 0;
  staleWarnMax = 0;
}

// ============================================================================
//  요약 찍기
// ============================================================================

// 이번 5초의 '늦게 도착한 정도'를 계산하고 누적 막대그래프에 더한다. 다음 5초의 기준(가장 작은 값)도 넘겨 둔다.
//   묵은 줄도 여기서 다시 센다: 줄이 도착할 때는 그때까지의 '가장 빨리 온 줄'로만 판단할 수 있다(부팅 직후에
//   묵은 줄부터 받으면 그 줄이 기준이 되어 못 잡는다). 5초가 끝나면 구간 전체의 가장 작은 값을 알므로 빠짐없이 센다.
//   (기준은 작아지기만 하므로, 도착할 때 묵은 줄로 센 줄은 여기서도 반드시 묵은 줄이다 → 늘어난 만큼만 더한다)
static void finishDelayWindow() {
  if (winOffCount == 0) return;                         // 받은 줄이 없으면 기준을 그대로 둔다
  memset(jitHist, 0, sizeof(jitHist));
  const int32_t ref = (baseValid && baseOff < winMinOff) ? baseOff : winMinOff;
  uint32_t mx = 0;
  uint32_t staleNow = 0;
  uint32_t staleMaxNow = 0;
  for (uint16_t i = 0; i < winOffCount; ++i) {
    const uint32_t rel = (uint32_t)(winOff[i] - winMinOff);
    const uint16_t bin = (rel >= JIT_BINS) ? (uint16_t)(JIT_BINS - 1) : (uint16_t)rel;
    if (jitHist[bin] < 0xFFFF) jitHist[bin] = (uint16_t)(jitHist[bin] + 1);
    totJitHist[bin] = totJitHist[bin] + 1;
    if (rel > mx) mx = rel;
    const uint32_t extra = (uint32_t)(winOff[i] - ref);
    if (extra > STALE_LIMIT_MS) {
      ++staleNow;
      if (extra > staleMaxNow) staleMaxNow = extra;
    }
  }
  if (staleNow > win.stale) {                           // 도착할 때 못 잡은 묵은 줄
    tot.stale += staleNow - win.stale;
    staleWarnCount += staleNow - win.stale;             // [경고] 알림에도 넣는다(부팅 직후 묵은 줄도 알림이 나오게)
    if (staleMaxNow > staleWarnMax) staleWarnMax = staleMaxNow;
    staleWarnPending = true;
    win.stale = staleNow;
  }
  if (staleMaxNow > win.staleMax) win.staleMax = staleMaxNow;
  if (staleMaxNow > tot.staleMax) tot.staleMax = staleMaxNow;
  win.jitCount = winOffCount;
  win.jitMax = mx;
  win.jitP95 = percentile16(jitHist, winOffCount, 95);
  tot.jitCount += winOffCount;
  if (mx > tot.jitMax) tot.jitMax = mx;
  baseOff = winMinOff;
  baseValid = true;
  winOffCount = 0;
  winMinOff = INT32_MAX;
}

static uint8_t printStatsLine(const Stats &s, bool isTotal, uint32_t now) {
  char why[200];
  const uint8_t verdict = judge(s, why, sizeof(why));
  const uint32_t elapsed = (uint32_t)(now - s.startMs);
  lineStart();
  if (isTotal) {
    lineAdd("[로봇 누적 ");
    lineAddElapsed(elapsed);
  } else {
    lineAdd("[로봇 %lu초 #%lu", (unsigned long)(REPORT_PERIOD_MS / 1000), (unsigned long)reportCount);
  }
  if (testStep > 0) lineAdd(" 단계%u", (unsigned)testStep);
  lineAdd(" %s%s%s]", verdictText(verdict), why[0] ? ": " : "", why);
  const uint32_t rateX10 = elapsed ? (uint32_t)(((uint64_t)s.good * 10000u + elapsed / 2) / elapsed) : 0;
  lineAdd(" 받음 %lu줄(초당 %lu.%lu, 기대 %lu) 빠진번호 %lu(", (unsigned long)s.good,
          (unsigned long)(rateX10 / 10), (unsigned long)(rateX10 % 10), (unsigned long)(1000 / EXPECTED_PERIOD_MS),
          (unsigned long)s.missing);
  lineAddPct(s.missing, lossDen(s));
  lineAdd(") | 가장 긴 공백 %lu ms, %lu ms 넘는 공백 %lu번", (unsigned long)s.maxGap,
          (unsigned long)SILENCE_LIMIT_MS, (unsigned long)s.silenceCount);
  if (s.silenceMs > 0) {
    lineAdd("(합 ");
    lineAddSec(s.silenceMs);
    lineAdd(")");
  }
  lineAdd(" | 묵은 줄 %lu", (unsigned long)s.stale);
  if (s.stale > 0) lineAdd("(최대 %lu ms 늦음)", (unsigned long)s.staleMax);
  if (s.jitCount > 0) {
    const uint32_t p95 = isTotal ? percentile32(totJitHist, s.jitCount, 95) : s.jitP95;
    lineAdd(" | 늦게 도착(가장 빠른 줄 대비) 95%% %lu ms 최대 %lu ms", (unsigned long)p95, (unsigned long)s.jitMax);
  }
  lineAdd(" | 체크섬틀림 %lu 형식틀림 %lu 순서뒤바뀜 %lu 중복 %lu 줄밖글자 %lu 응답건너뜀 %lu 조종기재시작 %lu",
          (unsigned long)s.badSum, (unsigned long)s.badFormat, (unsigned long)s.reorder, (unsigned long)s.dup,
          (unsigned long)s.junk, (unsigned long)s.echoSkipped, (unsigned long)s.restarts);
  if (isTotal) {
    lineAdd(" | 불합격 구간 %lu/%lu", (unsigned long)s.failedWindows, (unsigned long)s.windows);
    if (everGood) {
      lineAdd(" | 첫 줄: 부팅 뒤 ");
      lineAddSec(firstGoodMs);
    } else {
      lineAdd(" | 아직 받은 줄 없음");
    }
    if (longestOutageMs > 0) {
      lineAdd(" | 부팅 뒤 가장 긴 끊김 ");
      lineAddSec(longestOutageMs);
    }
  }
  lineEnd();
  return verdict;
}

static void printCsvLine(const Stats &s, uint8_t verdict, uint32_t now) {
  const uint32_t den = lossDen(s);
  const uint32_t loss = den ? (uint32_t)(((uint64_t)s.missing * 10000u + den / 2) / den) : 0;
  lineStart();
  lineAdd("CSV,robot,%u,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%u",
          (unsigned)testStep, (unsigned long)((now - tot.startMs) / 1000), (unsigned long)s.good,
          (unsigned long)s.missing, (unsigned long)loss, (unsigned long)s.maxGap, (unsigned long)s.silenceCount,
          (unsigned long)s.stale, (unsigned long)s.jitP95, (unsigned long)s.jitMax, (unsigned long)s.badSum,
          (unsigned long)s.badFormat, (unsigned long)s.reorder, (unsigned long)s.dup, (unsigned long)s.junk,
          (unsigned long)s.echoSkipped, (unsigned long)s.restarts, (unsigned)verdict);
  lineEnd();
}

// 5초마다: 이번 5초 요약 + 누적 요약을 찍고 5초 통계를 지운다.
static void reportTask(uint32_t now) {
  if ((uint32_t)(now - lastReportMs) < REPORT_PERIOD_MS) return;
  lastReportMs = now;
  ++reportCount;
  if (everGood && inOutage) {                           // 지금 끊겨 있으면 그 시간도 '가장 긴 공백'에 넣는다
    const uint32_t gap = (uint32_t)(now - lastGoodMs);
    if (gap > win.maxGap) win.maxGap = gap;
    if (gap > tot.maxGap) tot.maxGap = gap;
  }
  finishDelayWindow();
  const uint8_t winVerdict = printStatsLine(win, false, now);
  ++tot.windows;
  if (winVerdict == VERDICT_FAIL) ++tot.failedWindows;
  (void)printStatsLine(tot, true, now);
  if (PRINT_CSV) printCsvLine(win, winVerdict, now);
  if (inOutage) {
    lineStart();
    lineAdd("[참고] 지금 끊겨 있음: ");
    lineAddSec((uint32_t)(now - lastGoodMs));
    lineAdd("째 조종기 줄 없음");
    lineEnd();
  }
  if (badLineNew || junkNew) {
    lineStart();
    lineAdd("[참고]");
    if (badLineNew) lineAdd(" 마지막으로 버린 줄: \"%s\"", lastBadLine);
    if (junkNew) lineAdd(" 줄 밖 글자(마지막 %u개): \"%s\"", (unsigned)junkLen, junkTail);
    lineEnd();
    badLineNew = false;
    junkNew = false;
  }
  statsClear(win, now);
}

// 누적을 지운다('r' 또는 단계 번호). 링크 상태(마지막 줄 시각, 끊김 중인지, 번호 기준)는 그대로 둔다.
static void resetTotals(uint32_t now) {
  statsClear(tot, now);
  statsClear(win, now);
  memset(totJitHist, 0, sizeof(totJitHist));
  winOffCount = 0;
  winMinOff = INT32_MAX;
  if (inOutage) {
    tot.silenceCount = 1;
    win.silenceCount = 1;
  }
  lastReportMs = now;
  reportCount = 0;
  badLineNew = false;
  junkNew = false;
}

// ============================================================================
//  USB: 안내문과 명령
// ============================================================================
static void printBanner() {
  dbgEvent("==== BT-210 링크 시험 · 로봇 쪽(슬레이브) · %s ====", FIRMWARE_LABEL);
  dbgEvent("  BT: Serial2 %lu bps | 응답 %s | %lu초마다 요약 | 다이나믹셀 전원 꺼짐(팔 모터에 힘 없음)",
           (unsigned long)BT_BAUD, ECHO_ENABLED ? "보냄" : "안 보냄(ECHO_ENABLED = false)",
           (unsigned long)(REPORT_PERIOD_MS / 1000));
  dbgEvent("  판정 기준(우리가 정한 목표값): 빠진 번호 %lu.%02lu%% 미만, %lu ms 넘는 공백 없음, 묵은 줄(%lu ms 넘게 늦음) 없음",
           (unsigned long)(LOSS_LIMIT_X100 / 100), (unsigned long)(LOSS_LIMIT_X100 % 100),
           (unsigned long)SILENCE_LIMIT_MS, (unsigned long)STALE_LIMIT_MS);
  dbgEvent("  명령: r = 누적 지우기, 0~9 = 시험 단계 번호 정하기(누적도 지움). 조종기 보드에는 LinkTest_Leader 를 올린다");
  if (PRINT_CSV) {
    dbgEvent("CSV,side,step,elapsed_s,good,missing,loss_x100,max_gap_ms,silence_count,stale,late_p95_ms,late_max_ms,"
             "bad_sum,bad_format,reorder,dup,junk,echo_skipped,leader_restarts,verdict(0pass/1warn/2fail)");
  }
  lineStart();
  lineAdd("  지금: 받은 줄 %lu", (unsigned long)goodEver);
  if (everGood) {
    lineAdd(", 첫 줄 부팅 뒤 ");
    lineAddSec(firstGoodMs);
    lineAdd(inOutage ? ", 지금 끊겨 있음" : ", 받는 중");
  } else {
    lineAdd(", 아직 받은 줄 없음 (조종기 보드·BT-210 짝·전원 확인)");
  }
  lineEnd();
}

static void usbTask() {
  const bool open = usbTerminalOpen();
  if (open && !usbWasOpen) {
    dbgHead = 0;
    dbgTail = 0;
    bannerPending = true;
  }
  usbWasOpen = open;
  if (open && bannerPending) {
    bannerPending = false;
    printBanner();
  }
}

static void usbCommandTask(uint32_t now) {
  if (!usbTerminalOpen()) return;
  uint8_t budget = 8;
  while (budget > 0 && USB_SERIAL.available() > 0) {
    budget = (uint8_t)(budget - 1);
    const int c = USB_SERIAL.read();
    if (c == 'r' || c == 'R') {
      resetTotals(now);
      dbgEvent("[명령] 누적을 지웠습니다%s", (testStep > 0) ? " (단계 번호는 그대로)" : "");
    } else if (c >= '0' && c <= '9') {
      testStep = (uint8_t)(c - '0');
      resetTotals(now);
      dbgEvent("[명령] 시험 단계 %u 시작: 누적을 지웠습니다", (unsigned)testStep);
    }
  }
}

// ============================================================================
//  LED (시리얼 모니터 없이 로봇에 올려 두었을 때 링크 상태를 본다)
//    켜짐             = 조종기 줄을 제대로 받는 중(정상)
//    빠른 깜빡임      = 조종기 줄이 SILENCE_LIMIT_MS 넘게 끊겼다 (1초에 5번)
//    1초에 한 번 짧게 = 부팅 뒤 아직 한 줄도 못 받았다
// ============================================================================
static void ledTask(uint32_t now) {
  bool on;
  if (!everGood) {
    on = (now % 1000) < 60;
  } else if (inOutage) {
    on = (now % 200) < 100;
  } else {
    on = true;
  }
  if (on != ledIsOn) {
    ledIsOn = on;
    digitalWrite(LED_BUILTIN, (on == LED_ACTIVE_HIGH) ? HIGH : LOW);
  }
}

// ============================================================================
//  시작과 반복
// ============================================================================
static void setupRobot() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LED_ACTIVE_HIGH ? LOW : HIGH);   // LED 끔
  USB_SERIAL.begin(115200);                // USB 라서 이 숫자는 의미가 없다
  BT_SERIAL.begin(BT_BAUD);                // BT-210 (슬레이브)
  // 다이나믹셀은 쓰지 않는다(dxl.begin 을 부르지 않으므로 다이나믹셀 전원 스위치도 켜지 않는다)

  const uint32_t now = millis();
  bootMs = now;
  memset(&rx, 0, sizeof(rx));
  lastBadLine[0] = '\0';
  junkTail[0] = '\0';
  statsClear(win, now);
  statsClear(tot, now);
  memset(totJitHist, 0, sizeof(totJitHist));
  lastReportMs = now;
}

static void loopRobot() {
  uint32_t now = millis();
  usbTask();
  usbCommandTask(now);

  btPoll();

  now = millis();
  silenceTask(now);
  staleWarnTask(now);
  reportTask(now);
  ledTask(now);
  dbgPump(now);
}

}  // namespace linkrobot

void setup() {
  linkrobot::setupRobot();
}

void loop() {
  linkrobot::loopRobot();
}
