# AutoRobot 미션 후속 작업 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** PR #4(MS1 미션 통합) 위에서, 실물 없이 끝낼 수 있는 남은 소프트웨어 공백 세 가지를 메운다. ① GitHub에서 Mega 스케치를 실제 AVR로 컴파일하고 모든 모의 시험 실행 ② 경기 중 리셋 후 재시작 시련을 명령줄로 지정 ③ 웹 화면에 시련 상태 표시, 미션 모드에서 "5초 정지" 오표시 제거.

**Architecture:** 기존 파일 구조를 따른다. CI는 `.github/workflows/` 한 파일, 재시작 지정은 `scripts/run_mission.py`→`jetson/mission_server.py` 인자 전달, 화면은 `jetson/preview.html`의 `show(s)`에 `s.mission` 분기를 추가한다. 펌웨어·제어 로직(`mission.py`, `mission_link.py`, `.ino`)은 바꾸지 않는다.

**Tech Stack:** Python 3 + unittest/pytest, Node 22 (`node:assert`, `vm`), g++ C++17 호스트 하네스, arduino-cli + `arduino:avr` 1.8.8, GitHub Actions.

**Spec:** 사용자 요청(2026-11-12 세션 첫 메시지), `docs/eval2-robocon-20261112/SOFTWARE_AGENT_HANDOFF.txt`(PR #3), `autorobot/MISSION.md` §6·§8·§9.

## Global Constraints

- 펌웨어 업로드, 모터 구동, 장비 배포는 하지 않는다. CI는 컴파일·모의 시험만 한다.
- 컴파일·모의 시험·실물 결과를 구분해 보고한다. CI 통과를 "장비 검증"이라고 쓰지 않는다.
- `vision_stop.ino`, `vision_stop.ino.hex`, `vision_stop_server.py`(PR #2 작동본)는 수정하지 않는다.
- 비밀번호·SSH 키·개인 장치 주소·영상은 커밋하지 않는다. 테스트 주소는 `100.64.0.10`, `127.0.0.1`만 쓴다.
- 커밋은 `claude/exciting-cannon-tclnua`에만 하고 PR #4에 쌓는다. 메시지 끝에 세션 attribution 두 줄을 붙인다.
- 시련 번호는 사용자에게 1부터(n), 내부 `MissionConfig.start_index`는 0부터다.
- 화면 문구는 한국어, 기존 `preview.html` 어조("현재 미확인", "확인 중")를 따른다.

## Review Focus

1. **미션 모드 STOPPED가 "5초 정지 중"으로 보임** — 사람이 로봇이 곧 출발한다고 오해한다. 기대: `s.mission`이 있으면 "석상 정지 · 출발 명령 대기". → Task 3 테스트 `mission STOPPED label`.
2. **start 이벤트 중복(엔터 두 번)** — 두 번째 start가 GO를 또 보내면 안 된다. 기대: `(False, 'not_ready:STARTING')`. → Task 2 테스트 `test_double_start_rejected`.
3. **재시작 시련 번호가 범위 밖이거나 0** — 조용히 1번부터 시작하면 이미 끝난 시련을 다시 찾는다. 기대: `--check` 단계에서 거부, 종료 코드 1. → Task 2 테스트.
4. **구 VC1 서버의 status.json(`mission` 키 없음)** — 화면이 미션 패널 때문에 깨지면 안 된다. 기대: 패널 숨김, 기존 문구 그대로. → Task 3 테스트 `legacy status keeps 5-second label`.
5. **CI에서 AVR 코어 다운로드 실패** — 컴파일 단계를 건너뛰고 초록으로 끝나면 안 된다. 기대: job 실패. → Task 1 Step 3(확인 방법 고정, `continue-on-error` 금지).

---

### Task 1: CI — AVR 컴파일과 모의 시험

**Files:**
- Create: `.github/workflows/autorobot.yml`
- Create: `autorobot/requirements-dev.txt`
- Modify: `autorobot/MISSION.md` §8 (CI 행 추가)

**Interfaces:**
- Consumes: `autorobot/tests/fw/run_tests.py --cxx g++ --work-dir <dir>`, `autorobot/tests/fw/run_mission_tests.py --work-dir <dir>`, `node autorobot/tests/test_preview.js`, `python -m pytest -q autorobot/tests`
- Produces: PR #4의 check 이름 `autorobot / sim` 과 `autorobot / avr-compile`

- [ ] **Step 1: `requirements-dev.txt`** — `pytest==8.3.3`, `pyserial==3.5` 두 줄. Jetson 런타임 의존성(torch, ultralytics, opencv)은 넣지 않는다(테스트가 import하지 않음).
- [ ] **Step 2: workflow 작성** — 트리거 `pull_request`(paths `autorobot/**`, `arduino/**`, `.github/workflows/autorobot.yml`) + `workflow_dispatch`. `permissions: contents: read`.
  - job `sim` (ubuntu-24.04): `actions/setup-python@v5` python `3.12`, `actions/setup-node@v4` node `22`, pip install `-r autorobot/requirements-dev.txt`, 위 Consumes 네 명령을 각각 별도 step으로 실행.
  - job `avr-compile` (ubuntu-24.04): `arduino/setup-arduino-cli@v2`, `arduino-cli core update-index`, `arduino-cli core install arduino:avr@1.8.8`, 그리고 `--fqbn arduino:avr:mega`로 세 디렉터리 컴파일: `arduino/line_tracer_ver3_updated`, `autorobot/arduino/vision_stop`, `autorobot/arduino/mission_stop`. 각 compile에 `--warnings default`. 텔레오퍼레이션(OpenRB-150 코어 필요)은 이 Task에 넣지 않는다.
- [ ] **Step 3: 실패가 실패로 보이는지 확인** — workflow에 `continue-on-error`, `|| true`가 없음을 확인: `grep -nE "continue-on-error|\|\| true" .github/workflows/autorobot.yml` → 출력 없음.
- [ ] **Step 4: YAML 문법 확인** — `python3 -c "import yaml,sys;yaml.safe_load(open('.github/workflows/autorobot.yml'))"` → 오류 없음 (PyYAML 없으면 `pip install pyyaml` 후).
- [ ] **Step 5: 커밋·푸시** — `git add .github/workflows/autorobot.yml autorobot/requirements-dev.txt` → commit `ci: compile Mega sketches with arduino:avr 1.8.8 and run hardware-free tests` → push. 푸시가 workflow 권한으로 거부되면 파일을 되돌리지 말고 그 사실을 보고한다(사용자가 GitHub 앱 권한을 결정).
- [ ] **Step 6: CI 결과 확인** — PR #4 check runs에서 두 job 결과를 읽는다. 기대: `sim` PASS, `avr-compile` PASS와 각 스케치의 Flash/RAM 사용량 로그. 실패하면 로그 원인을 고친 뒤 재푸시. 결과를 `MISSION.md` §8 표에 "AVR 컴파일 (GitHub Actions, run 번호)" 행으로 기록하고 "실물 미수행" 행은 유지.

### Task 2: 리셋 후 재시작 시련 지정

**Files:**
- Modify: `autorobot/jetson/mission_server.py` (`parse_args`, `main`)
- Modify: `autorobot/scripts/run_mission.py` (`command`, `main`)
- Modify: `autorobot/MISSION.md` §6 고장과 복구 표 마지막 행
- Test: `autorobot/tests/test_mission_runtime.py`, `autorobot/tests/test_mission.py`

**Interfaces:**
- Consumes: `MissionConfig.start_index: int` (0-based), `load_plan(path) -> MissionPlan`, `mission_server.load_config(path|None) -> MissionConfig`
- Produces: CLI `--start-trial N` (1-based, 두 스크립트 공통); `mission_server.apply_start_trial(config: MissionConfig, start_trial: int | None, trial_count: int) -> MissionConfig` — None이면 config 그대로, 아니면 `dataclasses.replace(config, start_index=start_trial-1)`, 범위 밖이면 `ValueError('start trial must be 1..<count>')`.

- [ ] **Step 1: 실패 테스트 작성** (`test_mission_runtime.py`의 `LauncherTests`)

```python
def test_start_trial_override(self):
    cfg = mission_server.apply_start_trial(MissionConfig(), 3, 5)
    self.assertEqual(cfg.start_index, 2)
    self.assertIs(mission_server.apply_start_trial(MissionConfig(), None, 5).start_index, 0)
    for bad in (0, 6, -1):
        with self.assertRaises(ValueError):
            mission_server.apply_start_trial(MissionConfig(), bad, 5)

def test_start_trial_passed_through(self):
    cmd = run_mission.command(self.config(), Path('/r/plan.json'), None, executable='py', start_trial=2)
    self.assertEqual(cmd[cmd.index('--start-trial') + 1], '2')
    self.assertNotIn('--start-trial', run_mission.command(self.config(), Path('/r/plan.json'), None, executable='py'))
```

  그리고 `test_mission.py`의 `MissionTests`에:

```python
def test_double_start_rejected(self):
    b = Bench(); b.until(lambda: b.runner.motion == 'READY')
    self.assertTrue(b.event('start')[0])
    self.assertEqual(b.event('start'), (False, 'not_ready:STARTING'))
    b.until(lambda: b.runner.motion == 'SEEKING')
    self.assertEqual(sum(c.startswith(b'@VC1,G') for c in set(b.sent)), 1)
```

- [ ] **Step 2: 실패 확인** — `cd autorobot && python3 -m pytest -q tests/test_mission_runtime.py tests/test_mission.py -k "start_trial or double_start"` → `apply_start_trial` 없음/`start_trial` 인자 없음으로 FAIL (double_start는 이미 PASS일 수 있음 — 그 경우 기존 동작이 맞다는 증거로 남긴다).
- [ ] **Step 3: 구현** — `mission_server.parse_args`에 `--start-trial` (`type=int`, 기본 None). `main`에서 `config = apply_start_trial(load_config(...), args.start_trial, len(plan.trials))`. `run_mission.command(..., start_trial=None)`가 값이 있을 때 `['--start-trial', str(n)]`을 덧붙이고, `main`에 같은 인자를 추가해 `--check` 단계에서 `apply_start_trial`로 검증한다(오류 시 기존처럼 `Cannot start:` + 종료 코드 1). `--check` 출력에 `RESTART AT TRIAL n: <statue>` 한 줄.
- [ ] **Step 4: 통과 확인** — `python3 -m pytest -q tests` → 전체 PASS, 개수 = 이전 112 + 3.
- [ ] **Step 5: 문서** — §6 표의 리셋 행을 `--start-trial n`(남은 첫 시련 번호, 1부터) 사용으로 바꾼다.
- [ ] **Step 6: 커밋** — `git add autorobot/jetson/mission_server.py autorobot/scripts/run_mission.py autorobot/tests autorobot/MISSION.md` → `autorobot: --start-trial for restart after a reset`.

### Task 3: 웹 화면 시련 상태 패널과 정지 문구

**Files:**
- Modify: `autorobot/jetson/preview.html` (카드 영역에 패널 1개, `show(s)` 분기)
- Test: `autorobot/tests/test_preview.js`

**Interfaces:**
- Consumes: `/status.json`의 `s.mission` = `MissionRunner.status()` 반환값 — 키 `motion, fault, current_trial, target, game_elapsed_s, game_seconds, departure_policy, autonomous_run, operator_input_used, trials[{n, statue, crew_required, stop, crew, altar, complete}]`
- Produces: DOM id `mission`(카드 전체, `hidden` 토글), `mission-motion`, `mission-clock`, `mission-trials`(시련당 자식 1개), `mission-autonomy`

- [ ] **Step 1: 실패 테스트 작성** (`test_preview.js` 끝, 기존 assertion 뒤)

```js
const mission = {motion:'STOPPED', fault:null, current_trial:2, target:null, game_elapsed_s:61.2, game_seconds:300,
  departure_policy:'events', autonomous_run:false, operator_input_used:true,
  trials:[{n:1,statue:'Scylla',crew_required:2,stop:'STOPPED',crew:'DONE',altar:'PENDING',complete:false},
          {n:2,statue:'Cyclopes',crew_required:0,stop:'STOPPED',crew:'NOT_REQUIRED',altar:'PENDING',complete:false}]};
show({...state, mission});
assert.equal(nodes.get('mission').hidden, false);
assert.equal(nodes.get('mode').textContent, '석상 정지 · 출발 명령 대기');          // mission STOPPED label
assert.doesNotMatch(nodes.get('objects').children[0].textContent, /5초/);
assert.match(nodes.get('mission-motion').textContent, /2번 시련/);
assert.match(nodes.get('mission-clock').textContent, /1:01 \/ 5:00/);
assert.equal(nodes.get('mission-trials').children.length, 2);
assert.match(nodes.get('mission-trials').children[0].textContent, /1 Scylla · 정지 ✓ · 선원 ✓ · 제단 대기/);
assert.match(nodes.get('mission-trials').children[1].textContent, /선원 없음/);
assert.match(nodes.get('mission-autonomy').textContent, /개발 입력 사용 · 자율 경기 아님/);
show({...state, mission:{...mission, motion:'FAULT', fault:'LINK_OR_CAMERA_STALE'}});
assert.match(nodes.get('mission-motion').textContent, /LINK_OR_CAMERA_STALE/);
show({...state, mission:{...mission, trials:[{...mission.trials[0], crew:'UNVERIFIED'}]}});
assert.match(nodes.get('mission-trials').children[0].textContent, /선원 미확인/);
show(state);                                                                    // legacy status keeps 5-second label
assert.equal(nodes.get('mission').hidden, true);
assert.equal(nodes.get('mode').textContent, '5초 정지 중');
```

- [ ] **Step 2: 실패 확인** — `node autorobot/tests/test_preview.js` → `mission` 노드 `hidden` 값 불일치로 AssertionError.
- [ ] **Step 3: 구현** — `show(s)` 시작에 `const m=s.mission;`. Mega STOPPED 문구와 detection 문구의 "5초 정지 중"은 `m`이 있을 때 `'석상 정지 · 출발 명령 대기'`로 바꾼다(기존 `detectionState`에 `s`가 이미 전달됨). 패널 문구 매핑: stop `STOPPED→정지 ✓`, `PENDING→정지 대기`, `SKIPPED_BEFORE_RESTART→재시작 전`; crew `DONE→선원 ✓`, `NOT_REQUIRED→선원 없음`, `UNVERIFIED→선원 미확인`, `PENDING→선원 대기`; altar `DONE→제단 ✓`, `PENDING→제단 대기`. 시계는 `m:ss / m:ss`(경기 시작 전 `—`). `mission-motion`은 `motion`과 `current_trial`("n번 시련"), FAULT면 `fault` 문자열. DOM은 `textContent`만 사용(서버 문자열을 HTML로 넣지 않음). `화면 v3` 표기는 `화면 v4`로.
- [ ] **Step 4: 통과 확인** — `test_preview.js` 74·78행의 `화면 v3` 기대값과 88행 출력 문구를 `v4`로 바꾸고 개수를 실제 assertion 수로 갱신한 뒤 `node autorobot/tests/test_preview.js` → `Preview v4: <N> assertions passed`. `python3 -m pytest -q autorobot/tests` → `test_ui_identifies_confidence_and_display_only_pause` 포함 전체 PASS.
- [ ] **Step 5: 커밋** — `git add autorobot/jetson/preview.html autorobot/tests/test_preview.js` → `autorobot: show trial progress on the viewer; no 5-second label in mission mode`.

### Task 4: 최종 확인과 PR 갱신

**Files:** 없음(검증·보고만)

- [ ] **Step 1: 전체 로컬 시험** — `cd autorobot && python3 -m pytest -q tests && node tests/test_preview.js && python3 tests/fw/run_tests.py --cxx g++ --work-dir /tmp/fw-v && python3 tests/fw/run_mission_tests.py --work-dir /tmp/fw-m` → 모두 PASS, 펌웨어 하네스 4,594 / 4,602.
- [ ] **Step 2: 작동본 보존 확인** — `git diff e7ff2d7 -- autorobot/arduino/vision_stop autorobot/firmware autorobot/jetson/vision_stop_server.py "teleoperation /"` → 출력 없음.
- [ ] **Step 3: PR #4 본문 갱신** — 검증 표에 CI run 링크와 AVR 사용량을 추가하고, "실물 미수행" 줄은 유지.

## 이 계획에 넣지 않은 것 (현장·조교 답변이 필요해 막혀 있음)

- **선원 이송 완료·출발 공간 자동 감지:** 센서 종류(갑판 하중 센서/거리 센서)와 허용 여부가 정해져야 `source='sensor'` 이벤트 생성기를 설계할 수 있다. 답이 나오면 별도 계획을 쓴다.
- **수동 로봇 모터 모델 확인 코드:** ID1·3·4·5의 실제 모델과 5V/12V 배선이 확인돼야 기대 모델 번호를 정할 수 있다. 별도 계획을 쓴다(텔레오퍼레이션은 독립 하위 시스템).
- **`GAP_COAST_MS` 값, 정지 구역 40~60% 조정:** 센서 기록과 정지 오차 실측 후 결정한다.
