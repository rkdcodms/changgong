# AutoRobot 경기 프로그램 (MS1 미션 통합)

PR #2의 카메라 인식·Mega 통신 기반 위에, 최신 룰북(2026-11-10)과 Classum 답변(2026-11-12)을 반영한 경기용 상태 관리를 추가했다. 기존 5초 시험판(`vision_stop`)은 그대로 남겨 두었다.

> **검증 범위:** 아래 모든 결과는 PC에서의 컴파일·모의 시험이다. Jetson, Mega, 카메라, 모터에서는 아직 한 번도 실행하지 않았다. 펌웨어 업로드와 모터 구동은 현장 담당자 확인과 명시적 승인 후에만 한다.

## 1. 무엇이 어디에서 돌아가는가

| 구성 | 저장소 위치 | 상태 |
|---|---|---|
| Mega 단독 라인트레이서 (전원 투입 즉시 주행, 5초 재출발) | `arduino/line_tracer_ver3_updated/` (main) | PR #1로 병합. 컴파일만 기록됨 |
| Mega 5초 정지 시험판 VC1 | `autorobot/arduino/vision_stop/`, `firmware/vision_stop.ino.hex` (PR #2) | PR #2 작성자가 "기존 로봇 Mega에 승인된 펌웨어가 이미 설치"라고 기록. **이번 작업에서 확인하지 않음** |
| Jetson 5초 정지 프로그램 | `autorobot/jetson/vision_stop_server.py` (PR #2) | PR #2 기준 기존 Jetson 실행본. 이번 작업에서 동작 변경 없음 |
| **Mega 경기판 MS1** | `autorobot/arduino/mission_stop/mission_stop.ino` (신규) | 호스트 하네스만 통과. 업로드 안 함 |
| **Jetson 경기 프로그램** | `autorobot/jetson/mission_server.py` 외 (신규) | 모의 시험만 통과 |
| YOLO 모델 | `add-yolo-best-pt` 브랜치 `9f9d66d:best.pt` | 해시 대조 완료(아래) |
| 수동 로봇 리더·팔로워 | `teleoperation /` (폴더명 끝 공백 주의) | [수동 로봇 검토](../docs/software/MANUAL_ROBOT_REVIEW.md) |

연결 관계: 카메라 → Jetson(YOLO, `mission_server.py`) ⇄ USB 115200bps ⇄ Mega(MS1) → L298N → 바퀴. 바닥 센서 8개는 Mega가 직접 읽는다. Jetson은 정지·출발 명령만 보내고 라인 추종에는 관여하지 않는다.

### 실제 장비에 무엇이 올라가 있는지 확인하는 방법 (읽기 전용)

현장에서 아래 결과를 기록해야 "Git의 코드 = 장비의 코드"라고 말할 수 있다.

```bash
cd ~/changgong && git rev-parse HEAD && git status --short
```

```bash
sha256sum autorobot/models/best.pt autorobot/jetson/*.py
```

- Mega: 시리얼 모니터 115200bps에 `@ID1,MS1,MS1-20261112a`가 1초마다 나오면 MS1, `@LT1`/`@V1`만 나오면 VC1(5초판), 센서값만 나오면 단독 라인트레이서다. **시리얼을 열면 보드가 리셋될 수 있으니 단독 라인트레이서가 올라가 있을 가능성이 있으면 바퀴를 띄우거나 모터 전원을 끈다.**

| 파일 | SHA-256 (이 브랜치) |
|---|---|
| `best.pt` (5,507,098 B, `9f9d66d`) | `e67df5f04399513f8578a6948843902ea31b346c558375af88e919834283bd84` |
| `vision_stop.ino` / `.hex` (PR #2 그대로) | `b3b0ad12…2049` / `e3e292a4…216b` |
| `mission_stop.ino` | `8fdfcfb5989f2b46ab74f4c7f634731394116d12844c7bba579c8d9fddd63ac9` |

모델 확인 사항: 체크포인트 내부의 클래스 이름은 `Calypso, Cattle, Cyclopes, Laestrygonians, Poseidon, Scylla, Siren, TrojanHorse`로 제어 코드와 일치한다(룰북 표기 Cyclops/Sirens/Cattle of Helios/Poseidon's Wrath/Trojan Horse는 입력 시 자동 변환). 학습 메타데이터상 Ultralytics **8.4.190**으로 학습됐고 PR #2에 기록된 Jetson 환경은 **8.4.144**다. 기존 Jetson에서 이 모델이 정상 추론된 기록이 있다면 문제없지만, 새로 설치할 때 버전을 맞춰 확인한다.

## 2. 무엇이 바뀌었나 (5초판 대비)

| 항목 | VC1 5초판 | MS1 경기판 |
|---|---|---|
| 출발 | heartbeat가 들어오면 즉시 주행 | heartbeat는 세션만 묶음. **`start` 명령 전에는 제동 상태 유지** |
| 정지 대상 | 8종 중 어떤 동상이든 | **현재 시련의 동상만** (다른 동상은 기록만) |
| 정지 후 | 5초 뒤 자동 재출발 | **명시적 GO 전까지 정지 유지**. GO는 출발 조건이 충족됐을 때만 |
| 시련 완료 | 개념 없음 | 시련별 `stop / crew / altar` 상태를 따로 기록. 로봇 이동과 분리 |
| 펌웨어 확인 | 없음 | Jetson이 `@ID1,MS1`을 보기 전에는 heartbeat를 보내지 않음 → 구 펌웨어를 실수로 움직이지 않음 |
| 정지 테이프 | 고려 없음 | 선택 옵션 `GAP_COAST_MS`(기본 0=기존과 동일), 센서 기록 도구 |

라인 추종·탐색·제동·PWM은 `vision_stop.ino`와 비트 단위로 같다(하네스에서 1,500주기 PWM·방향·상태 비교).

## 3. 명령 프로토콜 (USB 115200, LF 종료, 한 줄 ≤ 62자)

Jetson → Mega

| 명령 | 의미 | Mega가 받아들이는 조건 |
|---|---|---|
| `@VC1,H,<session>` | heartbeat (200ms마다) | 최초 1회 세션 고정. 이후 같은 세션만 갱신 |
| `@VC1,S,<session>,<event>,<class>` | 동상 정지 | 주행 중(FOLLOW/EXIT_BAR/SEARCH), event > 마지막 event |
| `@VC1,G,<session>,<event>` | 출발/재출발 | WAIT_VISION이거나, STOPPED 후 0.3초 이상 경과(링크 FAULT 제외) |
| `@VC1,P,<session>,<event>` | 현재 위치 정지(호스트 hold) | 주행 중 |

event 번호는 S/G/P가 하나의 증가 카운터를 공유한다. 지연·재전송된 옛 명령은 번호가 작아 무시된다. Jetson은 응답이 올 때까지 100ms마다 같은 명령을 재전송하고 1초 안에 응답이 없으면 정지 신호(heartbeat)를 끊는다.

Mega → Jetson: `@LT1,<mode>,<stopSeq>,<age>,<uptime>,<test>` · `@V1,<session>,<event>,<class>,<reason>,<live>` · `@ID1,MS1,<build>` · 센서 8개 탭 구분 값. mode: 0 FOLLOW, 1 STOPPED, 2 EXIT_BAR, 3 SEARCH, 4 FAULT, 5 WAIT_VISION. reason: 0 없음, 2 동상, 3 링크 끊김, 4 호스트 hold.

## 4. 상태 전이

### 로봇 이동 상태 (Jetson `mission.py`)

| 현재 | 조건 | 다음 | 동작 |
|---|---|---|---|
| DISARMED | 카메라·모델·시리얼 정상, MS1 확인, Mega WAIT_VISION, 세션 live | READY | heartbeat 시작 (Mega는 계속 제동) |
| READY | `start` 이벤트 | STARTING | GO 전송 |
| STARTING | GO 응답 | SEEKING(1) | 목표 = 1번 시련 동상 |
| SEEKING(n) | 목표 동상 3프레임·0.1초 이상, 화면 중앙 40~60% | STOPPED(n) | S 전송 → 응답 확인 |
| STOPPED(n) | 최소 체류 + 출발 조건(아래) | DEPARTING | GO 전송, 목표 = n+1 |
| DEPARTING | GO 응답 | SEEKING(n+1) | |
| STOPPED(마지막) | 출발 조건 | FINAL_HOLD | 기본값: 그 자리에서 정지 유지(오디세우스 이송) |
| SEEKING/FINAL_HOLD(continue) | `hold` 또는 continue 시간 경과 | HOST_HOLD / FINAL_HOLD | P 전송 |
| HOST_HOLD | `resume` | DEPARTING → 이전 상태 | GO 전송 |
| 모든 상태 | 링크·카메라 1초 이상 비정상, Mega FAULT, 응답 시간초과, GO 없이 이동, 펌웨어 변경 | FAULT | 이후 명령 전송 안 함 → Mega 1.5초 watchdog FAULT |

### 시련 상태 (시련마다 독립)

| 필드 | 값 |
|---|---|
| stop | PENDING → STOPPED (재시작 시 앞 시련은 SKIPPED_BEFORE_RESTART) |
| crew | PENDING → DONE / NOT_REQUIRED(선원 0) / **UNVERIFIED**(시간 기반 출발, 완료 아님) |
| altar | PENDING → DONE (crew가 충족된 시련만 — 룰북 30쪽 Trial Rule) |
| complete | crew ∈ {DONE, NOT_REQUIRED} **그리고** altar = DONE |

예: 1번 선원 이송 후 로봇은 2번으로 이동 중(SEEKING 2)이고, 1번은 `crew=DONE, altar=PENDING, complete=false`. 수동 로봇이 1번 석상을 제단에 올리면 그때 `altar_done 1` → 1번 complete. 2번 도착 시 1번 제단 미완료이면 `previous_altar_pending_on_arrival=true`로 기록한다(조교 답변은 "다음 석상으로 미리 이동 가능"까지이며, 그 이상의 병렬 작업 허용 여부는 확인되지 않았다).

### 출발 조건 정책 (`config/mission.example.json`)

- `events`(기본): `crew_done n`과 `departure_clear n`이 모두 들어와야 출발. 선원 0명인 시련은 `departure_clear`만 필요.
- `timed`: 정지 후 `timed_base_s + timed_per_crew_s × 선원 수`초가 지나면 출발. crew는 **UNVERIFIED**로 남고 완료로 집계하지 않는다. 실측 작업 시간 없이 이 값을 정하지 않는다.

## 5. 완료·출발 공간을 실제로 어떻게 판단할 것인가 — **미정**

룰북 7쪽: AutoRobot은 "fully autonomous"여야 한다. 현재 설치된 장비(옆을 보는 카메라 1대, 바닥 IR 8개, Jetson-Mega USB)로는 다음을 직접 감지할 수 없다.

| 신호 | 현재 장비로 가능? | 후보 (확인 필요) |
|---|---|---|
| crew_done (갑판의 병사가 지정 수만큼 빠짐) | 불가 — 카메라가 갑판을 보지 않음 | 갑판 하중 센서(로드셀/FSR) 또는 갑판용 두 번째 카메라. 병사 1개 무게·갑판 고정 방식 실측 필요. 센서는 2만원 한도 구매 허용 품목 |
| departure_clear (수동 로봇이 경로를 비움) | 불가 | 전·측면 거리 센서(초음파/IR) |
| altar_done | 불필요 — 이동 결정에 쓰지 않음 | 점수·기록용으로만 사용 |

그래서 지금 구현은 이 신호를 `source`가 붙은 이벤트로 받는다. `scripts/mission_event.py`로 보내는 입력은 `source=operator`로 기록되고 상태에 `autonomous_run=false`가 표시된다. **사람이 누르는 입력을 경기에서 써도 되는지, 수동 로봇 쪽에서 신호를 보내는 것이 허용되는지 조교 확인 전에는 경기용으로 쓰지 않는다.** 확인 전 경기에서 가능한 자율 방식은 `timed` 정책뿐이다.

## 6. 실행 절차 (Jetson, `autorobot` 폴더)

준비: PR #2 README의 1~3단계(모델 다운로드, `config/robot.local.json`, `--check`)를 먼저 한다.

1. **펌웨어 (승인 후에만):** Arduino IDE에서 `arduino/mission_stop/mission_stop.ino`를 Mega 2560에 업로드. 업로드 전 기존 펌웨어 종류를 위 방법으로 기록한다. 업로드 중에는 모터 전원을 끈다.

```bash
arduino-cli compile --fqbn arduino:avr:mega arduino/mission_stop
```

2. **화면에 시련 순서가 나오면** (룰북 10쪽, 코드 수정 시간 제공):

```bash
/opt/hw2/venv/bin/python scripts/mission_plan.py --order "Scylla, Cyclops, Sirens" --crew "2,0,3"
```

   출력된 n·동상·선원 수를 화면과 한 줄씩 대조한다. 잘못된 이름, 3개 미만/8개 초과, 0~3 범위 밖, 중복 동상은 거부된다.

3. **점검 (장치 열지 않음):**

```bash
/opt/hw2/venv/bin/python scripts/run_mission.py --config config/robot.local.json --mission-config config/mission.example.json --check
```

4. **시작:** 모터 전원을 끈 상태로 실행 → 브라우저 `/status.json`의 `mission.motion`이 `READY`인지 확인 → 로봇을 출발 위치에 놓고 모터 전원 ON → 출발 신호에 맞춰 start.

```bash
/opt/hw2/venv/bin/python scripts/run_mission.py --config config/robot.local.json --mission-config config/mission.example.json --start
```

```bash
/opt/hw2/venv/bin/python scripts/mission_event.py start
```

5. **개발 시험용 이벤트** (경기 사용 여부 미확인):

```bash
/opt/hw2/venv/bin/python scripts/mission_event.py crew_done 1
```

```bash
/opt/hw2/venv/bin/python scripts/mission_event.py departure_clear 1
```

```bash
/opt/hw2/venv/bin/python scripts/mission_event.py altar_done 1
```

### 고장과 복구

| 상황 | 프로그램 동작 | 복구 |
|---|---|---|
| USB 케이블 빠짐 / Jetson 멈춤 | heartbeat 중단 → Mega 1.5초 후 FAULT 제동 | 모터 OFF → 프로그램 종료 → Mega RESET → 재시작 |
| 카메라 프레임 끊김·모델 오류 | 0.5초 후 heartbeat 중단 → 최대 약 2초 후 Mega FAULT | 동일 |
| 명령 응답 1초 없음 | Jetson FAULT, heartbeat 중단 | 동일 |
| Mega 리셋 (uptime 역행) | Jetson FAULT | 동일. 리셋 후 Mega는 WAIT_VISION에서 제동 |
| 선 탐색 실패 | Mega FAULT (기존 동작) | 위치 바로잡고 RESET |
| 중복·지연 이벤트 | 같은 id·5초 초과·다른 시련 번호는 거부, 응답에 이유 표시 | 필요하면 올바른 이벤트를 다시 보냄 |
| 경기 중 리셋(−5점, 출발 위치로) | — | `config`의 `start_index`를 남은 첫 시련(0부터)으로 바꿔 재시작 |

FAULT는 자동으로 풀리지 않는다. 원인을 확인하지 않고 반복 재시작하지 않는다.

## 7. 현장 시험 순서와 기록

모든 결과는 `tools/field_log.py`로 남긴다(Git 제외 `runtime/field_tests.jsonl`). 미션 로그의 소요 시간은 `import-mission`으로 가져온다.

1. **스모크 (바퀴 띄움, 모터 OFF→ON):** MS1 ID 확인 → 프로그램 시작 후 READY에서 바퀴가 안 도는지 → start 후 회전 → 목표 동상을 카메라 앞에 두면 정지 → 5초 넘게 정지 유지 → `crew_done`·`departure_clear` 후 재회전 → USB를 뽑아 1.5초 내 제동·FAULT.
2. **센서·정지 테이프:** `tools/sensor_capture.py record`로 흰 선, 어두운 테이프, 방지턱 위를 각각 천천히 통과하며 기록 → `analyze`로 채널별 여유와 전부-검정 구간 길이 확인. 구간이 생기면 기존 로직은 SEARCH로 들어가 방향을 모르면 약 0.17초 후 FAULT, 방향을 알면 원호 탐색으로 직선에서 벗어날 수 있으므로 `GAP_COAST_MS`를 그 값보다 약간 크게 정해 별도 승인 후 시험한다.
3. **정지 위치:** 동상 8종 × 각 5회 이상, 정지 표식 위에 로봇 일부가 올라갔는지(`--on-mark`)와 표식 중심까지 거리(`--error-mm`)를 기록. 화면 40~60% 구간과 실제 표식의 관계는 카메라 각도·속도·제동거리에 따라 달라 실측으로만 정한다.
4. **적재 방지턱:** 갑판(350g)+병사 3명 적재, 5mm 방지턱 쌍 통과 10회 연속 → 선 이탈·정지·FAULT·물체 낙하를 `bump`로 기록.
5. **5분 리허설:** 실제 순서 입력부터 귀환까지. 시간, 오디세우스 성공 여부, 점수(`tools/score.py`), 리셋 원인 기록.

## 8. 검증 결과 (2026-11-12, 이 브랜치, PC)

| 항목 | 결과 | 종류 |
|---|---|---|
| Python 테스트 (PR #2 기존 84 + 신규 28) | 112 passed | 모의 시험 |
| 웹 화면 assertion (PR #2) | 42 passed | 모의 시험 |
| `vision_stop.ino` 호스트 하네스 (PR #2) | 4,594 assertions PASS | 모의 시험 |
| `mission_stop.ino` 호스트 하네스 (신규) | 4,602 assertions PASS. 자동 재출발을 넣은 변이본은 FAIL로 검출 | 모의 시험 |
| `run_mission.py --check` (실제 `best.pt` 해시) | 통과 | 설정 점검 |
| AVR `arduino-cli compile` | **미수행** (이 환경에서 툴체인 다운로드 불가) | — |
| Jetson·Mega·카메라·모터 실물 | **미수행** | — |

재현:

```bash
python3 -m pytest -q tests
```

```bash
python3 tests/fw/run_mission_tests.py --work-dir /tmp/fw-mission
```

```bash
python3 tests/fw/run_tests.py --cxx g++ --work-dir /tmp/fw-vision
```

```bash
node tests/test_preview.js
```

## 9. 현장·조교 확인이 필요한 것

1. crew_done·departure_clear를 어떤 센서로 감지할지, 사람 입력이나 수동 로봇→자동 로봇 신호가 허용되는지 (조교).
2. 같은 동상이 한 경기에 두 번 나올 수 있는지 (현재 거부).
3. 어두운 정지 테이프가 흰 선을 가로지르는지, 선 옆에서 끝나는지, 센서에 어떻게 보이는지 (현장 측정).
4. 마지막 시련 후 자동 로봇이 어디에서 오디세우스를 넘겨야 유리한지, 추가 주회를 할지 (`after_last_trial`).
5. Mega에 현재 올라간 펌웨어 종류와 Jetson 실행 커밋.
