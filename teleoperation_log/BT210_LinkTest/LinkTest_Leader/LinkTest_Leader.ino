// ============================================================================
//  LinkTest_Leader.ino  -  BT-210 링크 시험 (조종기 쪽)  /  보드: OpenRB-150
//  SNU 창의공학설계 2026 ROBOCON · 블루투스 링크 시험 도구
// ----------------------------------------------------------------------------
//  하는 일
//    1) 20 ms마다 실제 조종기 줄과 길이가 같은 "시험 줄"을 BT-210(마스터)으로 보낸다.
//         T,ssssss,mmmmmmmm,pad*CC\n      (기본 25바이트. 실제 조종기 줄 "p1,p2,p3,p4,g*CC" 은 평균 약 25바이트)
//           ssssss   = 줄 번호(6자리, 0 ~ 999999 를 돈다)
//           mmmmmmmm = 보낸 시각(이 보드의 millis, 8자리)
//           pad      = 길이를 맞추는 소문자(a ~ z)
//           CC       = '*' 앞 글자 전부를 XOR 한 값(16진 대문자 2자리). 조종기 펌웨어와 같은 방식
//    2) 로봇 쪽(LinkTest_Robot)이 체크섬을 확인하고 곧바로 돌려보내는 "응답 줄"을 받는다.
//         R,ssssss,mmmmmmmm,gggggg*CC\n   (28바이트. ssssss·mmmmmmmm = 받은 시험 줄 그대로, gggggg = 로봇이 지금까지 제대로 받은 줄 수)
//    3) 줄마다 왕복 시간을 재고, 응답이 1초 안에 안 온 줄(손실), 체크섬 틀림, 형식 틀림(잘린 줄·이어 붙은 줄),
//       순서 뒤바뀜, 중복, 늦게 온 응답, 가장 긴 무응답 시간을 센다.
//       응답에 실린 '로봇이 받은 줄 수'로 손실을 '가는 길(조종기 → 로봇)'과 '오는 길(로봇 → 조종기)'로 나눈다.
//    4) 5초마다 한글 요약(이번 5초 + 누적)과 판정(합격 / 주의 / 불합격)을 USB 시리얼 모니터에 찍는다.
//
//  절대 멈추지 않는다
//    - 블루투스 송신 버퍼에 한 줄이 들어갈 자리가 없으면 기다리지 않고 그 줄을 건너뛰고 센다('BT 건너뜀').
//    - 받은 글자는 정해진 크기의 버퍼에만 모은다(String 을 쓰지 않는다). 줄 시작 글자 'R' 이 오면 앞의 조각은 버리고
//      새 줄을 시작한다 → 다시 연결된 직후의 잘린 줄·깨진 글자는 '형식 틀림'으로 세고 다음 줄부터 정상으로 읽는다.
//    - USB 출력은 큐에 쌓았다가 PC가 받아 갈 수 있을 때만 내보낸다(시리얼 모니터를 닫아도 시험은 그대로 돈다).
//
//  연결
//    Serial2 = BT-210 마스터 57600 bps (보드의 4핀 UART 커넥터. 조종기 펌웨어·수업 예제의 BLT_Serial 과 같은 포트)
//    USB     = 전원 + 결과 표시(시리얼 모니터)
//    다이나믹셀은 쓰지 않는다(다이나믹셀 전원도 켜지 않는다).
//
//  USB 시리얼 모니터에서 보내는 명령 (한 글자. 줄 끝 설정은 아무것이나)
//    r       = 누적을 지운다 (보드를 껐다 켜지 않고 다음 시험을 시작할 때)
//    0 ~ 9   = 시험 단계 번호를 정하고 누적을 지운다 (읽어보기.md 5장의 단계 번호)
//
//  고칠 곳은 바로 아래 "조정값" 블록 하나뿐이다. 로봇 쪽 LinkTest_Robot.ino 의 같은 이름 값과 맞춘다.
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
#define BT_SERIAL  Serial2   // BT-210 블루투스 모듈 (마스터)
#define USB_SERIAL Serial    // PC 시리얼 모니터

#define FIRMWARE_LABEL "LINKTEST_LEADER_20261113"

// ============================================================================
//  ★★★ 조정값 ★★★   (한 줄에 값 하나, 숫자만 바꾼다)
// ============================================================================

// ---- 1. 보내는 줄 ----
constexpr uint32_t BT_BAUD         = 57600;  // BT-210 속도(bps). 모듈 기본값. 로봇 쪽 시험 스케치와 같아야 한다
constexpr uint32_t FRAME_PERIOD_MS = 20;     // 한 줄을 보내는 주기(ms). 20 = 1초에 50번 (실제 조종기와 같다)
constexpr uint8_t  LINE_BYTES      = 25;     // 한 줄의 길이(바이트, 줄 끝 '\n' 포함). 22 ~ 48. 실제 조종기 줄은 13 ~ 32바이트(평균 약 25)

// ---- 2. 판정 기준 (우리가 정한 목표값이다. 공식 규격이 아니다. 읽어보기.md 4장) ----
constexpr uint32_t LOSS_LIMIT_X100  = 100;   // 손실이 이 값(0.01 % 단위) 이상이면 불합격. 100 = 1 %
constexpr uint32_t RTT_P95_LIMIT_MS = 60;    // 왕복 시간 95 % 값이 이 값(ms) 이상이면 불합격
constexpr uint32_t SILENCE_LIMIT_MS = 300;   // 응답이 이 시간(ms) 넘게 끊기면 불합격. 로봇 펌웨어가 팔을 멈추는 시간(LEADER_TIMEOUT_MS)과 같은 값

// ---- 3. 시간 ----
constexpr uint32_t REPLY_TIMEOUT_MS = 1000;  // 이 시간 안에 응답이 안 온 줄은 '손실'로 센다(ms). 그 뒤에 온 응답은 '늦게 옴'. 200 ~ 2000
constexpr uint32_t REPORT_PERIOD_MS = 5000;  // 요약을 찍는 주기(ms). 5000 = 5초

// ---- 4. 표시와 보드 ----
constexpr bool PRINT_CSV       = false;      // true = 요약마다 엑셀에 붙여 넣기 좋은 "CSV,..." 줄도 찍는다(읽어보기.md 3장)
constexpr bool LED_ACTIVE_HIGH = true;       // 보드 LED가 반대로 보이면 false
constexpr bool USB_TX_CHECK    = true;       // 시리얼 모니터에 아무것도 안 나오면 false 로 (USB 송신 상태 확인을 끈다)

// ============================================================================
//  컴파일할 때 자동으로 하는 검사 (조정값을 잘못 적으면 업로드 전에 여기서 멈춘다)
// ============================================================================
static_assert(LINE_BYTES >= 22 && LINE_BYTES <= 48,
              "LINE_BYTES 는 22 ~ 48 이어야 합니다 (22 = 덧붙이는 글자 없음, 48 = 로봇 쪽이 받을 수 있는 가장 긴 줄)");
static_assert(FRAME_PERIOD_MS >= 10 && FRAME_PERIOD_MS <= 200, "FRAME_PERIOD_MS 는 10 ~ 200 ms 여야 합니다");
static_assert((uint64_t)(LINE_BYTES > 28 ? LINE_BYTES : 28) * 10u * 1000u * 10u <= (uint64_t)BT_BAUD * 7u * FRAME_PERIOD_MS,
              "이 주기와 길이로는 블루투스 속도(BT_BAUD)의 70 % 를 넘습니다. FRAME_PERIOD_MS 를 늘리거나 LINE_BYTES 를 줄이세요");
static_assert(BT_BAUD >= 9600 && BT_BAUD <= 1000000, "BT_BAUD 는 9600 ~ 1000000 이어야 합니다 (BT-210 기본값 57600)");
static_assert(LOSS_LIMIT_X100 >= 1 && LOSS_LIMIT_X100 <= 5000, "LOSS_LIMIT_X100 은 1 ~ 5000 (0.01 % ~ 50 %) 이어야 합니다");
// 위쪽 끝 2000: 왕복 시간 막대그래프(1 ms 한 칸 x 2개)가 REPLY_TIMEOUT_MS 에 비례해 커진다. 3000 이면 보드 메모리(32 KB)를 넘어 링크가 깨진다.
static_assert(REPLY_TIMEOUT_MS >= 200 && REPLY_TIMEOUT_MS <= 2000, "REPLY_TIMEOUT_MS 는 200 ~ 2000 ms 여야 합니다 (더 길면 메모리가 모자랍니다)");
static_assert(RTT_P95_LIMIT_MS >= 5 && RTT_P95_LIMIT_MS < REPLY_TIMEOUT_MS, "RTT_P95_LIMIT_MS 는 5 ms 이상, REPLY_TIMEOUT_MS 보다 작아야 합니다");
static_assert(SILENCE_LIMIT_MS >= 2 * FRAME_PERIOD_MS && SILENCE_LIMIT_MS <= 10000,
              "SILENCE_LIMIT_MS 는 전송 주기의 2배 이상, 10000 ms 이하여야 합니다");
static_assert(REPORT_PERIOD_MS >= 1000 && REPORT_PERIOD_MS <= 60000, "REPORT_PERIOD_MS 는 1000 ~ 60000 ms 여야 합니다");

// 아두이노 IDE는 .ino 안의 함수 선언을 자동으로 파일 위쪽에 끼워 넣는데, 그러면 아래 구조체보다
// 선언이 먼저 나와 컴파일이 깨질 수 있다. 그래서 전체를 한 묶음(namespace)으로 감싼다. 지우지 말 것.
namespace linkleader {

// ---------------------------------------------------------------------------
//  줄 형식 (로봇 쪽 LinkTest_Robot.ino 와 같은 약속. 한쪽을 고치면 다른 쪽도 고친다)
// ---------------------------------------------------------------------------
constexpr uint32_t SEQ_MOD        = 1000000UL;    // 줄 번호는 0 ~ 999999 를 돈다(6자리)
constexpr uint32_t MS_MOD         = 100000000UL;  // 보낸 시각 = millis 를 1억으로 나눈 나머지(8자리, 약 27시간)
constexpr char     TEST_START     = 'T';          // 시험 줄의 첫 글자 (16진 숫자 A~F 와 겹치지 않는 글자)
constexpr char     REPLY_START    = 'R';          // 응답 줄의 첫 글자
constexpr uint8_t  HEAD_LEN       = 18;           // "T,ssssss,mmmmmmmm," 의 길이
constexpr uint8_t  TEST_FIXED     = 22;           // 덧붙이는 글자를 뺀 시험 줄 길이 = HEAD_LEN + "*CC\n"
constexpr uint8_t  PAD_LEN        = (uint8_t)(LINE_BYTES - TEST_FIXED);   // 덧붙이는 소문자 수
constexpr uint8_t  REPLY_BODY_LEN = 24;           // 응답 줄에서 '*' 앞까지의 길이 = HEAD_LEN + 6
constexpr uint8_t  REPLY_BYTES    = 28;           // 응답 줄 전체 길이('\n' 포함)
constexpr uint8_t  LINE_BUF_SIZE  = 64;           // 한 줄 버퍼 크기
constexpr uint8_t  RX_LINE_MAX    = 48;           // 받는 줄의 최대 글자 수('\n' 제외). 넘으면 그 줄을 버린다

// 받은 줄을 해석한 결과
constexpr uint8_t PARSE_OK     = 0;   // 맞는 줄
constexpr uint8_t PARSE_SUM    = 1;   // 꼴은 맞는데 체크섬이 틀림
constexpr uint8_t PARSE_FORMAT = 2;   // 꼴이 틀림(잘린 줄, 이어 붙은 줄, 이상한 글자)

// ---------------------------------------------------------------------------
//  응답을 기다리는 줄 목록
// ---------------------------------------------------------------------------
constexpr uint16_t TABLE_SIZE    = 250;    // 보낸 줄을 적어 두는 칸 수(50 Hz 에서 5초 분량). 1000000 을 나누어떨어지게 하는 수
constexpr uint8_t  SLOT_EMPTY    = 0;      // 빈 칸
constexpr uint8_t  SLOT_PENDING  = 1;      // 보냈고 응답을 기다리는 중
constexpr uint8_t  SLOT_ANSWERED = 2;      // 제때 응답이 왔다
constexpr uint8_t  SLOT_LOST     = 3;      // REPLY_TIMEOUT_MS 안에 응답이 안 와서 손실로 셌다
constexpr uint8_t  SLOT_LATE     = 4;      // 손실로 센 뒤에 늦게 응답이 왔다
constexpr uint16_t RTT_BINS      = (uint16_t)(REPLY_TIMEOUT_MS + 1);   // 왕복 시간 막대그래프 칸 수(1 ms 한 칸)
constexpr uint32_t SPLIT_MAX_GAP = 5000;   // 손실 나누기: 두 응답 사이 번호 차이가 이보다 크면(약 100초) 기준을 다시 잡는다
constexpr uint32_t ROBOT_RESTART_DROP = 1000;  // 로봇이 받은 줄 수가 이만큼 넘게 줄면 로봇이 다시 켜진 것으로 본다

static_assert(SEQ_MOD % TABLE_SIZE == 0, "TABLE_SIZE 는 1000000 을 나누어떨어지게 해야 합니다");
static_assert(REPLY_TIMEOUT_MS + 3 * FRAME_PERIOD_MS < (uint32_t)TABLE_SIZE * FRAME_PERIOD_MS,
              "REPLY_TIMEOUT_MS 가 너무 깁니다 (응답을 기다리는 줄 목록이 모자랍니다)");

// ---------------------------------------------------------------------------
//  글자 다루기 (String 과 printf 를 쓰지 않고 정해진 버퍼에 직접 적는다)
//    함수들은 '다음에 적을 위치'를 돌려준다. 자리가 모자라면 cap(버퍼 크기)을 돌려준다.
// ---------------------------------------------------------------------------

// 글자 하나를 적는다.
static uint8_t appendChar(char *out, uint8_t cap, uint8_t pos, char c) {
  if (pos >= cap) return cap;
  out[pos] = c;
  return (uint8_t)(pos + 1);
}

// 0 이상 정수를 width 자리 10진수(앞을 0으로 채움)로 적는다. 넘치는 윗자리는 버린다.
static uint8_t appendFixed(char *out, uint8_t cap, uint8_t pos, uint32_t value, uint8_t width) {
  if ((uint16_t)pos + width > cap) return cap;
  for (uint8_t i = 0; i < width; ++i) {
    out[pos + width - 1 - i] = (char)('0' + (value % 10));   // 낮은 자리부터 뒤에서 앞으로 적는다
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

// 시험 줄 "T,ssssss,mmmmmmmm,pad*CC\n" 을 만든다. 길이를 돌려준다(실패하면 0). 끝에 '\0' 을 붙인다.
static uint8_t buildTestLine(char *out, uint8_t cap, uint32_t seq, uint32_t ms) {
  uint8_t pos = 0;
  pos = appendChar(out, cap, pos, TEST_START);
  pos = appendChar(out, cap, pos, ',');
  pos = appendFixed(out, cap, pos, seq % SEQ_MOD, 6);
  pos = appendChar(out, cap, pos, ',');
  pos = appendFixed(out, cap, pos, ms % MS_MOD, 8);
  pos = appendChar(out, cap, pos, ',');
  for (uint8_t i = 0; i < PAD_LEN; ++i) {
    pos = appendChar(out, cap, pos, (char)('a' + ((seq + i) % 26)));   // 줄마다 다른 소문자(밀린 글자를 잡기 쉽게)
  }
  if (pos >= cap) return 0;
  const uint8_t checksum = xorChecksum(out, pos);       // '*' 앞 글자 전부를 XOR
  pos = appendChar(out, cap, pos, '*');
  pos = appendChar(out, cap, pos, hexDigit((uint8_t)(checksum >> 4)));
  pos = appendChar(out, cap, pos, hexDigit((uint8_t)(checksum & 0x0F)));
  pos = appendChar(out, cap, pos, '\n');
  if (pos >= cap) return 0;
  out[pos] = '\0';
  return pos;
}

// 응답 줄 "R,ssssss,mmmmmmmm,gggggg*CC" 을 해석한다.
static uint8_t parseReply(const char *line, uint8_t len, uint32_t &seq, uint32_t &ms, uint32_t &good) {
  const uint8_t tail = checkTail(line, len);
  if (tail != PARSE_OK) return tail;
  if ((uint8_t)(len - 3) != REPLY_BODY_LEN) return PARSE_FORMAT;
  if (line[0] != REPLY_START || line[1] != ',' || line[8] != ',' || line[17] != ',') return PARSE_FORMAT;
  if (!readFixed(line, 2, 6, seq) || !readFixed(line, 9, 8, ms) || !readFixed(line, HEAD_LEN, 6, good)) return PARSE_FORMAT;
  return PARSE_OK;
}

static_assert(TEST_FIXED + PAD_LEN == LINE_BYTES && LINE_BYTES < LINE_BUF_SIZE, "줄 길이 계산이 맞지 않습니다");

// ---------------------------------------------------------------------------
//  받은 글자를 한 줄로 모으기
//    줄 시작 글자(REPLY_START)가 와야 모으기 시작한다. 모으는 도중에 시작 글자가 다시 오면 앞의 조각은 버리고
//    새로 시작한다(끊겼다 이어진 직후의 잘린 줄 때문에 다음 줄까지 잃지 않는다). 줄 밖의 글자는 '줄 밖 글자'로 센다.
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
  uint32_t sent;           // 보낸 줄
  uint32_t skipped;        // 블루투스 송신 버퍼가 모자라 건너뛴 줄 (보내지 않았으므로 번호도 쓰지 않는다)
  uint32_t answered;       // 제때(REPLY_TIMEOUT_MS 안) 응답이 온 줄
  uint32_t lost;           // REPLY_TIMEOUT_MS 안에 응답이 안 온 줄
  uint32_t late;           // 손실로 센 뒤에 늦게 온 응답
  uint32_t dup;            // 같은 번호의 응답이 또 옴
  uint32_t reorder;        // 더 나중에 보낸 줄의 응답보다 늦게 도착한 응답
  uint32_t unknown;        // 보낸 적 없는(또는 너무 오래된) 번호의 응답
  uint32_t badSum;         // 체크섬이 틀린 응답 줄
  uint32_t badFormat;      // 꼴이 틀린 응답 줄(잘린 줄, 이어 붙은 줄 포함)
  uint32_t junk;           // 줄 밖 글자 수(바이트)
  uint32_t rttMin;         // 왕복 시간 최소(ms)
  uint32_t rttMax;         // 왕복 시간 최대(ms)
  uint32_t maxSilence;     // 가장 긴 무응답(제때 온 응답과 응답 사이, ms)
  uint32_t silenceCount;   // SILENCE_LIMIT_MS 넘게 끊긴 횟수
  uint32_t silenceMs;      // 그렇게 끊긴 시간의 합(ms)
  uint32_t fwdSent;        // 손실 나누기: 가는 길로 보낸 줄
  uint32_t fwdLost;        //              가는 길에서 사라진 줄(로봇이 못 받음)
  uint32_t retSent;        //              오는 길로 보낸 응답
  uint32_t retLost;        //              오는 길에서 사라진 응답(로봇은 받았는데 조종기가 응답을 못 받음)
  uint32_t windows;        // (누적만) 5초 구간 수
  uint32_t failedWindows;  // (누적만) 불합격 구간 수
};

constexpr uint8_t VERDICT_PASS = 0;   // 합격
constexpr uint8_t VERDICT_WARN = 1;   // 주의 (기준은 넘지 않았지만 늦은 응답·순서 뒤바뀜 같은 이상한 일이 있었다)
constexpr uint8_t VERDICT_FAIL = 2;   // 불합격

struct Slot {
  uint32_t seq;      // 줄 번호
  uint32_t sendMs;   // 보낸 시각
  uint8_t  state;    // SLOT_*
};

// ---- 상태 변수 ----
static Slot     table[TABLE_SIZE];
static Stats    win;                       // 이번 5초
static Stats    tot;                       // 누적('r' 이나 단계 번호를 보내면 지운다)
static uint32_t winHist[RTT_BINS];         // 이번 5초의 왕복 시간 막대그래프(1 ms 한 칸)
static uint32_t totHist[RTT_BINS];         // 누적 왕복 시간 막대그래프
static LineRx   rx;
static char     txBuf[LINE_BUF_SIZE];

static uint32_t seqNext = 0;               // 다음에 보낼 줄 번호
static uint32_t sentEver = 0;              // 부팅 뒤 보낸 줄 수(지우지 않는다)
static uint32_t lastFrameMs = 0;
static uint32_t lastReportMs = 0;
static uint32_t reportCount = 0;
static uint8_t  testStep = 0;              // 시험 단계 번호(0 = 안 정함)

static bool     everAnswered = false;      // 부팅 뒤 제때 온 응답이 한 번이라도 있었다
static uint32_t bootMs = 0;
static uint32_t firstAnswerMs = 0;         // 부팅 뒤 첫 응답까지 걸린 시간(ms)
static uint32_t lastAnswerMs = 0;          // 마지막으로 제때 온 응답을 받은 시각
static uint32_t lastAnsweredSeq = 0;       // 제때 온 응답 가운데 가장 나중 번호(순서 뒤바뀜 판단)
static bool     inOutage = false;          // 지금 SILENCE_LIMIT_MS 넘게 응답이 없다
static uint32_t outageSentMark = 0;        // 끊김이 시작될 때의 sentEver
static uint32_t longestOutageMs = 0;       // 부팅 뒤 가장 길었던 끊김(지우지 않는다)

static bool     splitValid = false;        // 손실 나누기의 기준이 있다
static uint32_t splitSeq = 0;              // 기준 응답의 번호
static uint32_t splitGood = 0;             // 기준 응답에 실린 '로봇이 받은 줄 수'

static uint32_t lastSkipWarnMs = 0;
static bool     skipWarnPending = false;
static char     lastBadLine[LINE_BUF_SIZE];   // 마지막으로 버린 응답 줄(표시용)
static bool     badLineNew = false;           // 이번 5초에 버린 줄이 있었다
static char     junkTail[25];                 // 마지막 줄 밖 글자 24개(표시용)
static uint8_t  junkLen = 0;
static bool     junkNew = false;

static bool     usbWasOpen = false;
static bool     bannerPending = false;
static bool     ledIsOn = false;

// ============================================================================
//  USB 표시 - 절대 멈추지 않게 만든 출력 (조종기 펌웨어와 같은 방식)
//    글자를 먼저 메모리(큐)에 쌓아 두고, PC가 받아 갈 수 있을 때만 조금씩 내보낸다.
//    PC가 읽지 않으면 버린다. (Serial.print 를 직접 부르면 PC가 안 읽을 때 오래 멈출 수 있다)
//    루프 안에서 if (Serial) 을 쓰지 않는다: OpenRB 코어에서는 그 한 줄이 매번 10 ms 를 잡아먹는다.
// ============================================================================
constexpr uint16_t DBG_QUEUE_SIZE = 4096;   // 2의 거듭제곱
constexpr uint16_t DBG_LINE_MAX   = 900;    // 한 줄 최대 바이트(한글은 글자당 3바이트)
constexpr uint8_t  USB_CHUNK_MAX  = 63;     // USB로 한 번에 내보내는 최대 바이트(64 미만이면 기다림이 없다)

static_assert((DBG_QUEUE_SIZE & (DBG_QUEUE_SIZE - 1)) == 0, "DBG_QUEUE_SIZE 는 2의 거듭제곱이어야 합니다");

static char     dbgQueue[DBG_QUEUE_SIZE];
static uint16_t dbgHead = 0;                // 다음에 쌓을 자리
static uint16_t dbgTail = 0;                // 다음에 내보낼 자리
static char     lineBuf[DBG_LINE_MAX];
static uint16_t lineLen = 0;
static uint32_t lastUsbWriteMs = 0;

// PC에서 시리얼 모니터를 열었는가. (if (Serial) 과 같은 뜻이지만 10 ms 지연이 없다)
static bool usbTerminalOpen() {
  return USB_SERIAL.dtr() || USB_SERIAL.rts();
}

// 앞서 내보낸 USB 데이터를 PC가 아직 가져가지 않았는가. (이때 또 쓰면 멈출 수 있으므로 쓰지 않는다)
static bool usbTxBusy(uint32_t now) {
  if (!USB_TX_CHECK) return (uint32_t)(now - lastUsbWriteMs) < 3;       // 확인을 끄면 3 ms 에 한 번만 쓴다
  for (uint8_t ep = 1; ep < 8; ++ep) {
    if (USB->DEVICE.DeviceEndpoint[ep].EPCFG.bit.EPTYPE1 == 3) {        // 3 = PC 쪽으로 보내는 통로(USB 시리얼 송신)
      return USB->DEVICE.DeviceEndpoint[ep].EPSTATUS.bit.BK1RDY != 0;   // 1 = 아직 PC가 안 가져갔다
    }
  }
  return true;                                                           // USB가 아직 연결되지 않았다
}

static uint16_t dbgUsed() {
  return (uint16_t)((dbgHead - dbgTail) & (DBG_QUEUE_SIZE - 1));
}

// 한 줄을 통째로 큐에 넣는다. 모니터가 닫혀 있거나 자리가 모자라면 그 줄은 버린다.
static void dbgPush(const char *text, uint16_t length) {
  if (!usbTerminalOpen()) return;
  if (length > (uint16_t)(DBG_QUEUE_SIZE - 1 - dbgUsed())) return;
  for (uint16_t i = 0; i < length; ++i) {
    dbgQueue[dbgHead] = text[i];
    dbgHead = (uint16_t)((dbgHead + 1) & (DBG_QUEUE_SIZE - 1));
  }
}

// 큐에 쌓인 글자를 조금 내보낸다. 루프를 돌 때마다 부른다. 기다리지 않는다.
static void dbgPump(uint32_t now) {
  if (dbgHead == dbgTail) return;
  if (!usbTerminalOpen()) {             // 듣는 쪽이 없으면 비운다
    dbgHead = 0;
    dbgTail = 0;
    return;
  }
  if (usbTxBusy(now)) return;
  const uint16_t available = dbgUsed();
  uint16_t count = (available < USB_CHUNK_MAX) ? available : USB_CHUNK_MAX;
  if (count < available) {
    // 한글(여러 바이트로 된 글자)을 중간에서 자르지 않는다: 다음 바이트가 '이어지는 바이트'면 한 칸씩 물러난다
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
  if (written != count) return;         // USB가 준비되지 않아 못 보냈다(이 코어는 실패하면 아주 큰 수를 돌려준다) → 다음에 다시
  dbgTail = (uint16_t)((dbgTail + count) & (DBG_QUEUE_SIZE - 1));
}

// ---- 한 줄 조립 ----
static void lineStart() {
  lineLen = 0;
  lineBuf[0] = '\0';
}

static void lineAddV(const char *format, va_list args) {
  if (lineLen >= DBG_LINE_MAX - 3) return;                                 // 줄 끝 "\r\n" 자리를 남긴다
  const uint16_t room = (uint16_t)(DBG_LINE_MAX - 3 - lineLen);            // 더 넣을 수 있는 글자 수
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

// 한 줄짜리 알림.
static void dbgEvent(const char *format, ...) __attribute__((format(printf, 1, 2)));
static void dbgEvent(const char *format, ...) {
  va_list args;
  lineStart();
  va_start(args, format);
  lineAddV(format, args);
  va_end(args);
  lineEnd();
}

// 비율을 "0.40%" 처럼 적는다. 분모가 0 이면 "-".
static void lineAddPct(uint32_t num, uint32_t den) {
  if (den == 0) {
    lineAdd("-");
    return;
  }
  const uint32_t x100 = (uint32_t)(((uint64_t)num * 10000u + den / 2) / den);
  lineAdd("%lu.%02lu%%", (unsigned long)(x100 / 100), (unsigned long)(x100 % 100));
}

// ms 를 "1.2초" 처럼 적는다.
static void lineAddSec(uint32_t ms) {
  lineAdd("%lu.%lu초", (unsigned long)(ms / 1000), (unsigned long)((ms % 1000) / 100));
}

// ============================================================================
//  통계 계산
// ============================================================================
static void statsClear(Stats &s, uint32_t now) {
  memset(&s, 0, sizeof(s));
  s.startMs = now;
  s.rttMin = 0xFFFFFFFFUL;
}

// 막대그래프에서 pct % 값(올림 순위)을 찾는다. count = 그래프에 들어 있는 개수.
static uint32_t histPercentile(const uint32_t *hist, uint32_t count, uint32_t pct) {
  if (count == 0) return 0;
  uint32_t rank = (uint32_t)(((uint64_t)count * pct + 99) / 100);
  if (rank < 1) rank = 1;
  uint32_t cum = 0;
  for (uint16_t i = 0; i < RTT_BINS; ++i) {
    cum += hist[i];
    if (cum >= rank) return i;
  }
  return RTT_BINS - 1;
}

// 왕복 시간 하나를 두 통계에 넣는다.
static void addRtt(uint32_t rtt) {
  const uint16_t bin = (rtt >= RTT_BINS) ? (uint16_t)(RTT_BINS - 1) : (uint16_t)rtt;
  winHist[bin] = winHist[bin] + 1;
  totHist[bin] = totHist[bin] + 1;
  if (rtt < win.rttMin) win.rttMin = rtt;
  if (rtt > win.rttMax) win.rttMax = rtt;
  if (rtt < tot.rttMin) tot.rttMin = rtt;
  if (rtt > tot.rttMax) tot.rttMax = rtt;
}

static void noteLost() {
  ++win.lost;
  ++tot.lost;
}

// 판정. 까닭을 why 에 적는다(합격이면 빈 글).
static uint8_t judge(const Stats &s, const uint32_t *hist, char *why, size_t cap) {
  uint8_t verdict = VERDICT_PASS;
  size_t used = 0;
  why[0] = '\0';
  // 까닭 하나를 덧붙인다
  #define WHY_ADD(...) do { if (used < cap) { const int n_ = snprintf(why + used, cap - used, __VA_ARGS__); \
                            if (n_ > 0) used += (size_t)n_; if (used >= cap) used = cap - 1; } } while (0)
  const uint32_t resolved = s.answered + s.lost;
  if (s.sent == 0 && resolved == 0) {
    WHY_ADD("보낸 줄 없음");
    verdict = VERDICT_FAIL;
  } else if (s.answered == 0) {
    WHY_ADD("응답 없음");
    verdict = VERDICT_FAIL;
  } else {
    if ((uint64_t)s.lost * 10000u >= (uint64_t)LOSS_LIMIT_X100 * resolved) {
      const uint32_t x100 = (uint32_t)(((uint64_t)s.lost * 10000u + resolved / 2) / resolved);
      WHY_ADD("%s손실 %lu.%02lu%%", used ? ", " : "", (unsigned long)(x100 / 100), (unsigned long)(x100 % 100));
      verdict = VERDICT_FAIL;
    }
    const uint32_t p95 = histPercentile(hist, s.answered, 95);
    if (p95 >= RTT_P95_LIMIT_MS) {
      WHY_ADD("%s왕복 95%% %lu ms", used ? ", " : "", (unsigned long)p95);
      verdict = VERDICT_FAIL;
    }
  }
  if (s.maxSilence > SILENCE_LIMIT_MS) {
    WHY_ADD("%s무응답 %lu ms", used ? ", " : "", (unsigned long)s.maxSilence);
    verdict = VERDICT_FAIL;
  }
  if (s.skipped > 0) {
    WHY_ADD("%sBT 건너뜀 %lu", used ? ", " : "", (unsigned long)s.skipped);
    verdict = VERDICT_FAIL;
  }
  if (verdict == VERDICT_PASS) {
    if (s.late > 0) WHY_ADD("%s늦게 옴 %lu", used ? ", " : "", (unsigned long)s.late);
    if (s.reorder > 0) WHY_ADD("%s순서 뒤바뀜 %lu", used ? ", " : "", (unsigned long)s.reorder);
    if (s.dup > 0) WHY_ADD("%s중복 %lu", used ? ", " : "", (unsigned long)s.dup);
    if (s.unknown > 0) WHY_ADD("%s모르는 번호 %lu", used ? ", " : "", (unsigned long)s.unknown);
    if (used > 0) verdict = VERDICT_WARN;
  }
  #undef WHY_ADD
  return verdict;
}

static const char *verdictText(uint8_t v) {
  return (v == VERDICT_PASS) ? "합격" : (v == VERDICT_WARN) ? "주의" : "불합격";
}

// ============================================================================
//  블루투스: 보내기
// ============================================================================

// 시험 줄 하나를 보낸다. 송신 버퍼에 자리가 없으면 그 줄은 건너뛴다(절대 기다리지 않는다).
static void sendTestLine(uint32_t now) {
  const uint8_t len = buildTestLine(txBuf, LINE_BUF_SIZE, seqNext, now);
  if (len == 0) return;
  if (BT_SERIAL.availableForWrite() < (int)len) {
    ++win.skipped;
    ++tot.skipped;
    skipWarnPending = true;
    return;                                             // 번호는 쓰지 않는다(로봇 쪽에서 '빠진 번호'로 보이지 않게)
  }
  Slot &slot = table[seqNext % TABLE_SIZE];
  if (slot.state == SLOT_PENDING) noteLost();           // (안전망) 아직 기다리던 칸을 덮어쓰면 그 줄은 손실
  BT_SERIAL.write((const uint8_t *)txBuf, len);
  slot.seq = seqNext;
  slot.sendMs = now;
  slot.state = SLOT_PENDING;
  ++win.sent;
  ++tot.sent;
  ++sentEver;
  seqNext = (seqNext + 1) % SEQ_MOD;
}

// REPLY_TIMEOUT_MS 가 지나도록 응답이 없는 줄을 손실로 센다.
static void sweepTimeouts(uint32_t now) {
  for (uint16_t i = 0; i < TABLE_SIZE; ++i) {
    Slot &slot = table[i];
    if (slot.state == SLOT_PENDING && (uint32_t)(now - slot.sendMs) >= REPLY_TIMEOUT_MS) {
      slot.state = SLOT_LOST;
      noteLost();
    }
  }
}

// ============================================================================
//  블루투스: 응답 받기
// ============================================================================

// 응답에 실린 '로봇이 받은 줄 수'로 손실을 가는 길 / 오는 길로 나눈다.
//   앞선 응답(번호 s0, 로봇 수 g0)과 이번 응답(s1, g1) 사이에 조종기는 d = s1 - s0 줄을 보냈고,
//   로봇은 그 가운데 gd = g1 - g0 줄을 받았다 → 가는 길에서 d - gd 줄이 사라졌다.
//   로봇이 보낸 gd 개의 응답 가운데 조종기가 받은 것은 이번 것 하나 → 오는 길에서 gd - 1 개가 사라졌다.
static void splitLoss(uint32_t seq, uint32_t good) {
  if (!splitValid) {
    splitValid = true;
    splitSeq = seq;
    splitGood = good;
    return;
  }
  const uint32_t d = (seq + SEQ_MOD - splitSeq) % SEQ_MOD;
  if (d == 0 || d >= SEQ_MOD / 2) return;               // 같거나 더 앞선 번호(순서 뒤바뀜)는 쓰지 않는다
  const uint32_t gd = (good + SEQ_MOD - splitGood) % SEQ_MOD;
  if (good < splitGood && splitGood - good > ROBOT_RESTART_DROP) {
    dbgEvent("[알림] 로봇 보드가 다시 켜진 것으로 보입니다 (로봇이 받은 줄 수가 처음부터 다시 시작: %lu → %lu)",
             (unsigned long)splitGood, (unsigned long)good);
  } else if (gd >= 1 && gd <= d && d <= SPLIT_MAX_GAP) {
    win.fwdSent += d;
    tot.fwdSent += d;
    win.fwdLost += d - gd;
    tot.fwdLost += d - gd;
    win.retSent += gd;
    tot.retSent += gd;
    win.retLost += gd - 1;
    tot.retLost += gd - 1;
  }
  splitSeq = seq;                                       // 맞지 않는 값이면(로봇 재시작 등) 기준만 다시 잡는다
  splitGood = good;
}

// 제때 온 응답 하나를 처리한다: 무응답 시간 / 끊김 복구.
static void noteAnswerTiming(uint32_t now, uint32_t rtt) {
  if (everAnswered) {
    const uint32_t gap = (uint32_t)(now - lastAnswerMs);
    if (gap > win.maxSilence) win.maxSilence = gap;
    if (gap > tot.maxSilence) tot.maxSilence = gap;
    if (inOutage) {
      inOutage = false;
      win.silenceMs += gap;
      tot.silenceMs += gap;
      if (gap > longestOutageMs) longestOutageMs = gap;
      dbgEvent("[복구] 응답 다시 옴: %lu.%lu초 동안 끊겼음 (그동안 보낸 줄 %lu개, 지금 왕복 %lu ms)",
               (unsigned long)(gap / 1000), (unsigned long)((gap % 1000) / 100),
               (unsigned long)(sentEver - outageSentMark), (unsigned long)rtt);
    }
  } else {
    everAnswered = true;
    firstAnswerMs = (uint32_t)(now - bootMs);
    dbgEvent("[알림] 첫 응답 받음: 부팅 뒤 %lu.%lu초 (왕복 %lu ms) → 링크가 이어졌습니다",
             (unsigned long)(firstAnswerMs / 1000), (unsigned long)((firstAnswerMs % 1000) / 100), (unsigned long)rtt);
  }
  lastAnswerMs = now;
}

// 맞는 응답 줄 하나를 처리한다.
static void handleReply(uint32_t seq, uint32_t good, uint32_t now) {
  Slot &slot = table[seq % TABLE_SIZE];
  if (slot.state == SLOT_EMPTY || slot.seq != seq) {   // 보낸 적 없는 번호(또는 5초 넘게 지난 줄)
    ++win.unknown;
    ++tot.unknown;
    return;
  }
  if (slot.state == SLOT_ANSWERED || slot.state == SLOT_LATE) {
    ++win.dup;
    ++tot.dup;
    return;
  }
  splitLoss(seq, good);
  const uint32_t rtt = (uint32_t)(now - slot.sendMs);
  if (slot.state == SLOT_PENDING && rtt >= REPLY_TIMEOUT_MS) {   // 시간이 지났지만 sweepTimeouts(20 ms마다)가 아직 못 본 줄:
    slot.state = SLOT_LOST;                             // 기준을 똑같이 맞춘다(손실 + 늦게 옴)
    noteLost();
  }
  if (slot.state == SLOT_LOST) {                        // 손실로 센 뒤에 왔다(REPLY_TIMEOUT_MS 넘게 묵은 응답)
    slot.state = SLOT_LATE;
    ++win.late;
    ++tot.late;
    return;
  }
  // 제때 온 응답
  slot.state = SLOT_ANSWERED;
  ++win.answered;
  ++tot.answered;
  addRtt(rtt);
  if (everAnswered) {
    const uint32_t back = (lastAnsweredSeq + SEQ_MOD - seq) % SEQ_MOD;   // 마지막 번호보다 얼마나 앞선 번호인가
    if (back > 0 && back < SEQ_MOD / 2) {
      ++win.reorder;
      ++tot.reorder;
    } else {
      lastAnsweredSeq = seq;
    }
  } else {
    lastAnsweredSeq = seq;
  }
  noteAnswerTiming(now, rtt);
}

// 버린 줄을 표시용으로 남긴다.
static void keepBadLine(const char *text) {
  strncpy(lastBadLine, text, sizeof(lastBadLine) - 1);
  lastBadLine[sizeof(lastBadLine) - 1] = '\0';
  badLineNew = true;
}

// 줄 밖 글자를 표시용으로 남긴다(마지막 24글자. 보이지 않는 글자는 '.').
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
    const uint8_t r = rxFeed(rx, c, REPLY_START);
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
    uint32_t good = 0;
    const uint8_t p = parseReply(rx.buf, rx.lineLen, seq, ms, good);
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
    handleReply(seq, good, millis());
  }
}

// 응답이 SILENCE_LIMIT_MS 넘게 없으면 '끊김'으로 본다.
static void silenceTask(uint32_t now) {
  if (!everAnswered || inOutage) return;
  if ((uint32_t)(now - lastAnswerMs) > SILENCE_LIMIT_MS) {
    inOutage = true;
    outageSentMark = sentEver;
    ++win.silenceCount;
    ++tot.silenceCount;
    dbgEvent("[경고] 응답 끊김: %lu ms 넘게 응답이 없습니다 (마지막으로 제때 온 응답 번호 %06lu) → 가는 길이 끊긴 것이면 실제 경기에서 로봇 팔이 멈춘다(로봇 쪽 화면으로 확인)",
             (unsigned long)SILENCE_LIMIT_MS, (unsigned long)lastAnsweredSeq);
  }
}

// ============================================================================
//  요약 찍기
// ============================================================================

// "3분20초" 처럼 적는다.
static void lineAddElapsed(uint32_t ms) {
  const uint32_t s = ms / 1000;
  lineAdd("%lu분%02lu초", (unsigned long)(s / 60), (unsigned long)(s % 60));
}

// 통계 한 줄. title = "5초 #12" 또는 "누적 3분20초" 앞부분.
static uint8_t printStatsLine(const Stats &s, const uint32_t *hist, bool isTotal, uint32_t now) {
  char why[200];
  const uint8_t verdict = judge(s, hist, why, sizeof(why));
  lineStart();
  if (isTotal) {
    lineAdd("[누적 ");
    lineAddElapsed((uint32_t)(now - s.startMs));
  } else {
    lineAdd("[%lu초 #%lu", (unsigned long)(REPORT_PERIOD_MS / 1000), (unsigned long)reportCount);
  }
  if (testStep > 0) lineAdd(" 단계%u", (unsigned)testStep);
  lineAdd(" %s%s%s]", verdictText(verdict), why[0] ? ": " : "", why);
  lineAdd(" 보냄 %lu 응답 %lu 손실 %lu(", (unsigned long)s.sent, (unsigned long)s.answered, (unsigned long)s.lost);
  lineAddPct(s.lost, s.answered + s.lost);
  lineAdd(") 가는길 ");
  lineAddPct(s.fwdLost, s.fwdSent);
  lineAdd(" 오는길 ");
  lineAddPct(s.retLost, s.retSent);
  if (s.answered > 0) {
    lineAdd(" | 왕복 ms 최소 %lu 중간 %lu 95%% %lu 최대 %lu", (unsigned long)s.rttMin,
            (unsigned long)histPercentile(hist, s.answered, 50), (unsigned long)histPercentile(hist, s.answered, 95),
            (unsigned long)s.rttMax);
  } else {
    lineAdd(" | 왕복 -");
  }
  lineAdd(" | 가장 긴 무응답 %lu ms, %lu ms 넘는 끊김 %lu번", (unsigned long)s.maxSilence,
          (unsigned long)SILENCE_LIMIT_MS, (unsigned long)s.silenceCount);
  if (s.silenceMs > 0) {
    lineAdd("(합 ");
    lineAddSec(s.silenceMs);
    lineAdd(")");
  }
  lineAdd(" | 체크섬틀림 %lu 형식틀림 %lu 순서뒤바뀜 %lu 중복 %lu 늦게옴 %lu 모르는번호 %lu BT건너뜀 %lu",
          (unsigned long)s.badSum, (unsigned long)s.badFormat, (unsigned long)s.reorder, (unsigned long)s.dup,
          (unsigned long)s.late, (unsigned long)s.unknown, (unsigned long)s.skipped);
  if (isTotal) {
    lineAdd(" | 불합격 구간 %lu/%lu", (unsigned long)s.failedWindows, (unsigned long)s.windows);
    if (everAnswered) {
      lineAdd(" | 첫 응답: 부팅 뒤 ");
      lineAddSec(firstAnswerMs);
    } else {
      lineAdd(" | 아직 응답 없음");
    }
    if (longestOutageMs > 0) {
      lineAdd(" | 부팅 뒤 가장 긴 끊김 ");
      lineAddSec(longestOutageMs);
    }
  }
  lineEnd();
  return verdict;
}

// 엑셀용 한 줄 (PRINT_CSV = true 일 때)
static void printCsvLine(const Stats &s, const uint32_t *hist, uint8_t verdict, uint32_t now) {
  const uint32_t resolved = s.answered + s.lost;
  const uint32_t loss = resolved ? (uint32_t)(((uint64_t)s.lost * 10000u + resolved / 2) / resolved) : 0;
  const uint32_t fwd = s.fwdSent ? (uint32_t)(((uint64_t)s.fwdLost * 10000u + s.fwdSent / 2) / s.fwdSent) : 0;
  const uint32_t ret = s.retSent ? (uint32_t)(((uint64_t)s.retLost * 10000u + s.retSent / 2) / s.retSent) : 0;
  lineStart();
  lineAdd("CSV,leader,%u,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%u",
          (unsigned)testStep, (unsigned long)((now - tot.startMs) / 1000), (unsigned long)s.sent,
          (unsigned long)s.answered, (unsigned long)s.lost, (unsigned long)loss, (unsigned long)fwd, (unsigned long)ret,
          (unsigned long)(s.answered ? s.rttMin : 0), (unsigned long)histPercentile(hist, s.answered, 50),
          (unsigned long)histPercentile(hist, s.answered, 95), (unsigned long)s.rttMax, (unsigned long)s.maxSilence,
          (unsigned long)s.silenceCount, (unsigned long)s.badSum, (unsigned long)s.badFormat, (unsigned long)s.reorder,
          (unsigned long)s.dup, (unsigned long)s.late, (unsigned long)s.unknown, (unsigned long)s.skipped,
          (unsigned)verdict);
  lineEnd();
}

// 5초마다: 이번 5초 요약 + 누적 요약을 찍고 5초 통계를 지운다.
static void reportTask(uint32_t now) {
  if ((uint32_t)(now - lastReportMs) < REPORT_PERIOD_MS) return;
  lastReportMs = now;
  ++reportCount;
  // 지금 끊겨 있으면 그 시간도 '가장 긴 무응답'에 넣는다(끊김이 끝날 때 다시 정확히 잰다)
  if (everAnswered && inOutage) {
    const uint32_t gap = (uint32_t)(now - lastAnswerMs);
    if (gap > win.maxSilence) win.maxSilence = gap;
    if (gap > tot.maxSilence) tot.maxSilence = gap;
  }
  const uint8_t winVerdict = printStatsLine(win, winHist, false, now);
  ++tot.windows;                                        // 누적 줄에 '불합격 구간 n/m' 으로 나온다
  if (winVerdict == VERDICT_FAIL) ++tot.failedWindows;
  (void)printStatsLine(tot, totHist, true, now);
  if (PRINT_CSV) printCsvLine(win, winHist, winVerdict, now);
  if (inOutage) {
    lineStart();
    lineAdd("[참고] 지금 끊겨 있음: ");
    lineAddSec((uint32_t)(now - lastAnswerMs));
    lineAdd("째 응답 없음");
    lineEnd();
  }
  if (badLineNew || junkNew) {
    lineStart();
    lineAdd("[참고]");
    if (badLineNew) lineAdd(" 마지막으로 버린 응답 줄: \"%s\"", lastBadLine);
    if (junkNew) lineAdd(" 줄 밖 글자(마지막 %u개): \"%s\"", (unsigned)junkLen, junkTail);
    lineEnd();
    badLineNew = false;
    junkNew = false;
  }
  statsClear(win, now);
  memset(winHist, 0, sizeof(winHist));
}

// 블루투스 송신 버퍼가 모자라 줄을 건너뛰었으면 1초에 한 번만 알린다.
static void skipWarnTask(uint32_t now) {
  if (!skipWarnPending || (uint32_t)(now - lastSkipWarnMs) < 1000) return;
  skipWarnPending = false;
  lastSkipWarnMs = now;
  dbgEvent("[경고] 블루투스 송신 버퍼에 자리가 없어 줄을 건너뛰었습니다 (누적 %lu줄). BT_BAUD·FRAME_PERIOD_MS·LINE_BYTES 확인",
           (unsigned long)tot.skipped);
}

// 누적을 지운다('r' 또는 단계 번호). 링크 상태(마지막 응답 시각, 끊김 중인지)는 그대로 둔다.
static void resetTotals(uint32_t now) {
  statsClear(tot, now);
  statsClear(win, now);
  memset(totHist, 0, sizeof(totHist));
  memset(winHist, 0, sizeof(winHist));
  if (inOutage) {                                       // 끊긴 채로 지웠으면 그 끊김을 새 누적에 넣는다
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
  dbgEvent("==== BT-210 링크 시험 · 조종기 쪽(마스터) · %s ====", FIRMWARE_LABEL);
  dbgEvent("  BT: Serial2 %lu bps | %lu ms마다 %u바이트 시험 줄 | 응답을 %lu ms 기다림 | %lu초마다 요약",
           (unsigned long)BT_BAUD, (unsigned long)FRAME_PERIOD_MS, (unsigned)LINE_BYTES,
           (unsigned long)REPLY_TIMEOUT_MS, (unsigned long)(REPORT_PERIOD_MS / 1000));
  dbgEvent("  판정 기준(우리가 정한 목표값): 손실 %lu.%02lu%% 미만, 왕복 95%% 값 %lu ms 미만, %lu ms 넘는 무응답 없음, BT 건너뜀 없음",
           (unsigned long)(LOSS_LIMIT_X100 / 100), (unsigned long)(LOSS_LIMIT_X100 % 100),
           (unsigned long)RTT_P95_LIMIT_MS, (unsigned long)SILENCE_LIMIT_MS);
  dbgEvent("  명령: r = 누적 지우기, 0~9 = 시험 단계 번호 정하기(누적도 지움). 로봇 보드에는 LinkTest_Robot 을 올린다");
  if (PRINT_CSV) {
    dbgEvent("CSV,side,step,elapsed_s,sent,answered,lost,loss_x100,fwd_loss_x100,ret_loss_x100,rtt_min,rtt_med,rtt_p95,rtt_max,"
             "max_silence_ms,silence_count,bad_sum,bad_format,reorder,dup,late,unknown,skipped,verdict(0pass/1warn/2fail)");
  }
  lineStart();
  lineAdd("  지금: 보낸 줄 %lu", (unsigned long)sentEver);
  if (everAnswered) {
    lineAdd(", 첫 응답 부팅 뒤 ");
    lineAddSec(firstAnswerMs);
    lineAdd(inOutage ? ", 지금 끊겨 있음" : ", 응답 받는 중");
  } else {
    lineAdd(", 아직 응답 없음 (로봇 보드·BT-210 짝·전원 확인)");
  }
  lineEnd();
}

// 시리얼 모니터가 방금 열렸으면 안내문을 찍는다.
static void usbTask() {
  const bool open = usbTerminalOpen();
  if (open && !usbWasOpen) {
    dbgHead = 0;                      // 예전에 쌓인 글은 버린다
    dbgTail = 0;
    bannerPending = true;
  }
  usbWasOpen = open;
  if (open && bannerPending) {
    bannerPending = false;
    printBanner();
  }
}

// 시리얼 모니터에서 보낸 한 글자 명령을 읽는다. (모니터가 열려 있을 때만, 한 번에 몇 글자만)
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
//  LED
//    켜짐             = 응답을 제때 받는 중(정상)
//    빠른 깜빡임      = 응답이 SILENCE_LIMIT_MS 넘게 끊겼다 (1초에 5번)
//    1초에 한 번 짧게 = 부팅 뒤 아직 응답을 한 번도 못 받았다(로봇 보드·BT-210 짝·전원 확인)
// ============================================================================
static void ledTask(uint32_t now) {
  bool on;
  if (!everAnswered) {
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
static void setupLeader() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LED_ACTIVE_HIGH ? LOW : HIGH);   // LED 끔
  USB_SERIAL.begin(115200);                // USB 라서 이 숫자는 의미가 없다
  BT_SERIAL.begin(BT_BAUD);                // BT-210 (마스터)

  const uint32_t now = millis();
  bootMs = now;
  for (uint16_t i = 0; i < TABLE_SIZE; ++i) {
    table[i].state = SLOT_EMPTY;
    table[i].seq = 0;
    table[i].sendMs = now;
  }
  memset(&rx, 0, sizeof(rx));
  lastBadLine[0] = '\0';
  junkTail[0] = '\0';
  statsClear(win, now);
  statsClear(tot, now);
  memset(winHist, 0, sizeof(winHist));
  memset(totHist, 0, sizeof(totHist));
  lastFrameMs = now;
  lastReportMs = now;
}

static void loopLeader() {
  uint32_t now = millis();
  usbTask();
  usbCommandTask(now);

  if ((uint32_t)(now - lastFrameMs) >= FRAME_PERIOD_MS) {
    lastFrameMs += FRAME_PERIOD_MS;                                           // 평균 주기를 정확히 지킨다
    if ((uint32_t)(now - lastFrameMs) >= FRAME_PERIOD_MS) lastFrameMs = now;  // 많이 밀렸으면 지금부터 다시 센다
    sendTestLine(now);
    sweepTimeouts(now);
  }

  btPoll();

  now = millis();
  silenceTask(now);
  skipWarnTask(now);
  reportTask(now);
  ledTask(now);
  dbgPump(now);
}

}  // namespace linkleader

void setup() {
  linkleader::setupLeader();
}

void loop() {
  linkleader::loopLeader();
}
