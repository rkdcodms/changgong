# 빌드 파일 (이전 버전) — 업로드하지 않음

| 파일 | 출처 |
|---|---|
| `vision_stop.ino.hex` | PR #2의 5초 정지 시험판과 같은 파일 (SHA-256 `e3e292a4…`) |
| `vision_stop_robocon.ino.hex` | Jetson에 있던 로보콘 빌드. 바이트 분석상 센서 가중치가 4~−4라서 `../arduino/vision_stop/vision_stop.ino`에서 빌드된 것으로 추정 (확정 아님) |

현재 Mega용 소스는 `../arduino/trace_stop/trace_stop.ino`이며, Arduino IDE에서 직접 컴파일해 올린다.
