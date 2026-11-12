# AutoRobot: 라인트레이싱과 동상 인식 정지

> 경기용 시련 순서·지정 석상 정지·정지 유지·시련별 상태 관리는 [MISSION.md](MISSION.md)를 본다. 이 문서는 5초 정지 시험판(VC1) 안내이며 그대로 유지한다.

Mega가 바닥의 흰 선을 따라 주행하고, Jetson이 옆쪽 카메라로 동상을 인식해 정지를 요청한다. 정지 후 5초가 지나면 다시 출발한다. Chrome에서는 전체 영상, 인식 박스·신뢰도, 실제 정지 상태를 확인한다.

이 폴더는 팀원이 코드를 받아 설정을 바꾸고 같은 Jetson에서 실행하기 위한 자료다. GitHub에 코드를 올리거나 내려받는 것만으로 현재 로봇의 프로그램이 바뀌지는 않는다.

## 카메라 화면만 볼 때

팀원 PC가 이미 Tailscale에 연결돼 있다면, Jetson과 웹캠 프로그램이 켜진 상태에서 Chrome으로 다음 주소를 연다.

```text
http://JETSON_TAILSCALE_IP:8765/
```

`JETSON_TAILSCALE_IP`는 팀원 Tailscale에 표시되는 Jetson의 IPv4 주소로 바꾼다. Jetson 터미널에서는 `tailscale ip -4`로 확인한다.

- 팀 설정의 `allow_tailscale: true`는 Jetson에 접속할 수 있는 Tailscale 기기를 웹서버에서도 허용한다. PC별 IP를 추가하지 않아도 된다.
- Tailscale에 로그인했다는 이유만으로 모든 사람이 접근할 수 있는 것은 아니다. Jetson 공유 여부와 Tailscale 접근 정책이 먼저 적용된다.
- 동시 영상 시청은 최대 4개 연결이다. 안 쓰는 웹캠 탭은 닫는다.
- `max_seconds: 0`은 영상 서버의 시간 제한을 없앤다. Jetson 전원·네트워크·프로그램이 계속 실행돼 있어야 한다.

이 페이지는 영상과 상태를 보는 화면이다. 운전·설정 변경 버튼은 없으며, 설정 변경은 아래 SSH 절차로 한다. HTML 파일을 PC에서 직접 열거나 GitHub Pages에 올려도 Jetson의 카메라·추론·Mega 연결을 대신 실행할 수 없다.

## 현재 동작 기준

| 항목 | 기준 |
|---|---|
| 정지할 인식 | 같은 동상이 신뢰도 55% 이상으로 3프레임 이상, 최소 0.1초 동안 이어짐 |
| 정지할 위치 | 확인된 동상 박스의 중심이 화면 가로 40~60% 구간에 들어옴 |
| 재출발 | Mega에서 5초 정지 후 재출발 |
| 같은 동상 재인식 | 같은 종류가 실제 추론에서 1.5초 이상 보이지 않아야 다시 대기 상태가 됨 |
| 화면의 확률 | 모델의 신뢰도이며, 측정된 정확도나 오차율은 아님 |

화면 가장자리에서 먼저 인식한 뒤 중앙에 들어올 때 정지를 요청한다. 화면에 박스가 있다는 것만으로 정지 조건이 모두 충족된 것은 아니다.

## 같은 Jetson에서 설정하고 실행하기

실행과 재시작은 현장에서 로봇을 확인하는 한 명이 맡는다. **코드를 바꾸거나 프로그램을 시작하기 전에 모터 전원을 끈다.** Jetson·카메라·센서·Mega는 연결을 유지한다. Mega 포트를 열거나 RESET을 누르면 보드가 재시작될 수 있고, 정상 통신이 시작되면 주행 상태로 전환될 수 있다.

### 1. SSH로 접속하고 실행 폴더로 이동

팀원 PC 터미널에서 아래의 `USER`와 `JETSON_TAILSCALE_IP`를 실제 값으로 바꾼다.

```sh
ssh USER@JETSON_TAILSCALE_IP
```

기존 Jetson 계정의 SSH 인증이 필요하다. Tailscale 접속 권한과 Jetson 로그인 권한은 별개다. 이 안내 때문에 Tailscale SSH 기능을 새로 켤 필요는 없다.

Jetson에서 저장소를 받은 뒤, 그 안의 `autorobot` 폴더로 이동한다. 아래 명령은 모두 이 폴더에서 실행한다. 이 Jetson에는 기존에 구성한 `/opt/hw2/venv/bin/python`을 우선 사용한다.

### 2. 학습 모델과 기기별 설정 준비

```sh
/opt/hw2/venv/bin/python scripts/download_model.py
cp -n config/robot.example.json config/robot.local.json
```

모델 다운로드는 지정된 버전의 `best.pt`를 받고 SHA-256을 확인한다. `cp -n`은 이미 있는 개인 설정을 덮어쓰지 않는다. `config/robot.local.json`을 열어 실제 Jetson 주소와 Mega 장치를 맞춘다. 이 파일은 Git 추적에서 제외된다.

| 설정 | 의미·시작값 |
|---|---|
| `bind` | 이 Jetson의 Tailscale IPv4 주소. `0.0.0.0`으로 바꾸지 않는다. |
| `port` | 웹캠 주소의 포트, `8765` |
| `allow_tailscale` | Tailscale 접속 기기 허용, `true` |
| `allowed_clients` | 개별 IP 허용 목록. 팀 설정에서는 빈 배열 `[]` |
| `camera` | 카메라 장치, `/dev/video0` |
| `rotate_ccw` | 영상 반시계 회전 각도, `90` |
| `serial` | 실제 Mega의 `/dev/serial/by-id/` 경로 |
| `weights` | 학습 모델, `models/best.pt` |
| `confidence` | 정지 판단 신뢰도, `0.55` |
| `stop_zone_left`, `stop_zone_right` | 정지 구역의 좌우 경계, `0.4`, `0.6` |
| `max_seconds` | 서버 실행 시간 제한, `0`이면 무제한 |

Mega 장치 경로는 `ls -l /dev/serial/by-id/`로 확인한다. USB를 꽂은 순서에 따라 바뀔 수 있는 `/dev/ttyACM0`보다 이 경로를 사용한다. 계정에 Mega 포트 접근 권한이 없으면 관리자가 `dialout` 그룹을 설정한 뒤 새로 로그인해야 한다.

### 3. 장치를 열지 않는 점검

```sh
/opt/hw2/venv/bin/python scripts/run_robot.py --config config/robot.local.json --check
```

이 점검은 카메라와 Mega를 열거나 모터를 움직이지 않는다. 설정·파일 점검을 통과해도 실제 카메라, 추론 속도, 바닥 센서, 정지 동작이 검증됐다는 뜻은 아니다.

### 4. 기존 프로그램을 종료한 뒤 시작

모터 전원이 꺼져 있는지 확인하고, 이전 웹캠·주행 프로그램을 실행했던 터미널이나 서비스에서 먼저 정상 종료한다. 같은 카메라·Mega·8765 포트를 두 프로그램이 동시에 사용하게 두지 않는다. 이 실행기는 기존 프로그램을 자동으로 찾아 강제 종료하지 않는다.

```sh
/opt/hw2/venv/bin/python scripts/run_robot.py --config config/robot.local.json --start
```

이 명령은 현재 터미널에서 프로그램을 계속 실행한다. 터미널을 닫거나 `Ctrl+C`를 누르면 계속 운영된다고 가정하면 안 된다. 부팅 자동 시작이나 백그라운드 서비스 설치는 이 절차에 포함하지 않는다. 실행 중 생성되는 파일은 쓰기 가능한 실행 폴더에 기록한다.

Chrome에서 새로고침하고 영상, 신뢰도 박스, Mega 상태를 확인한다. 모터를 끈 채 동상을 중앙 구역에 보여서 `5초 정지 중`과 이후 상태 변화가 나타나는지 확인한다. 로봇을 정상 주행선 위에 놓고, 현장 담당자가 준비 상태를 확인한 뒤 짧게 주행 시험한다.

## 설정을 바꿀 때와 문제가 생겼을 때

- 신뢰도·정지 구역·카메라 방향은 `config/robot.local.json`에서 바꾸고 위 점검·종료·시작 순서로 적용한다. 영상 설정을 바꾸려고 Mega를 다시 업로드할 필요는 없다.
- 화면이 이전 모습이면 Chrome에서 `Ctrl+F5`로 새로고침한다. `403`이면 새 팀 접속 설정이 실제 실행 중인 프로그램에 적용됐는지 확인한다.
- 연결 자체가 안 되면 Jetson 전원, Tailscale 연결·접근 정책, 실행 프로그램, 주소·포트 순으로 확인한다. GitHub 저장소가 열리는 것과 Jetson 접속은 별개다.
- `FAULT`나 세션 불일치가 남으면 모터를 끄고 프로그램을 종료한다. 바닥 센서가 정상 흰 선 위치를 보는지 확인하고 Mega의 물리적 RESET을 누른 뒤 다시 시작한다. FAULT를 자동으로 지우거나 주행을 강제로 재개하는 기능은 없다.
- 카메라·Mega가 사용 중이라는 오류가 나면 이전 실행이 남아 있는지 확인한다. 확인 없이 다른 프로세스를 종료하거나 포트를 재설정하지 않는다.

## 개발 환경과 파일 위치

기존 Jetson에서 기록한 환경은 PyTorch `2.10.0`, CUDA `12.6`, Ultralytics `8.4.144`이다. NumPy, OpenCV(`cv2`), pyserial도 사용한다. 새 Jetson을 구성할 때는 JetPack과 GPU용 PyTorch 호환을 확인해야 한다. 일반 PC용 설치 명령으로 기존 GPU 환경을 덮어쓰지 않는다.

| 경로 | 수정·확인할 내용 |
|---|---|
| `jetson/vision_stop_server.py` | 카메라, 추론, 정지 요청, 웹 서버 연결 |
| `jetson/vision_control.py` | 연속 인식·정지·재인식·통신 상태 판단 |
| `jetson/preview.html` | Chrome 영상과 상태 표시 |
| `jetson/web_preview.py` | 영상 송출과 접속 허용 |
| `arduino/vision_stop/vision_stop.ino` | Mega 라인트레이싱·5초 정지 펌웨어 소스 |
| `firmware/vision_stop.ino.hex` | 같은 기준의 Mega 펌웨어 빌드 파일 |
| `tests/` | 제어 로직·화면·펌웨어 검사 |

기존 로봇의 Mega에는 승인된 펌웨어가 이미 설치돼 있다. 이 패키지에 소스와 HEX가 있다는 이유로 다시 업로드하지 않는다. 펌웨어 변경과 업로드는 별도 작업으로 다룬다.

코드를 공유할 때는 변경 내용과 검사 결과를 함께 남긴다. 로그인 비밀번호, SSH 키, `robot.local.json`, 실행 로그·영상은 Git에 넣지 않는다. GitHub에 수정본을 올린 뒤에도 현장 담당자가 Jetson에서 해당 버전을 받아 점검·재시작해야 실제 동작이 바뀐다.
