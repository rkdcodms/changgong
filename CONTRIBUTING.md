# 팀 Git 작업 방식

AutoRobot·수동 로봇·문서를 여러 명이 동시에 고치면서도, **지금 로봇에 들어간 코드가 무엇인지** 언제나 답할 수 있게 하는 규칙이다. 저장소는 **공개(public)** 다.

## 1. 한눈에

```text
main  ← 리뷰된 것만. 직접 push·웹 업로드 금지
 ├─ auto/<주제>     AutoRobot (Jetson, Mega)
 ├─ manual/<주제>   수동 로봇 (OpenRB 리더·팔로워)
 ├─ docs/<주제>     룰북·답변·보고서
 └─ field/<날짜>    현장에서 바꾼 상수·튜닝 기록
로봇에 올린 순간 → 태그 robot-auto-YYYYMMDD-a / robot-manual-YYYYMMDD-a
```

## 2. 영역과 담당

| 영역 | 경로 | 담당(리뷰어) |
|---|---|---|
| AutoRobot | `autorobot/`, `arduino/` | AutoRobot 담당 |
| 수동 로봇 | `teleoperation /` (끝에 공백) | 수동 로봇 담당 |
| 규칙·문서 | `docs/`, `README.md` | 문서 담당 |

- 남의 영역 파일을 고쳐야 하면 그 담당을 PR 리뷰어로 지정한다.
- 현장 실행·재시작·업로드는 **현장 담당 1명**만 한다. 다른 사람은 PR로만 바꾼다.

## 3. 작업 순서

```bash
git switch main
```

```bash
git pull --ff-only
```

```bash
git switch -c auto/statue-lifted
```

1. 작게 고치고 자주 커밋한다. 한 PR = 한 목적(예: "석상 들어 올림 전 출발 금지"). 300줄을 넘으면 나눌 수 있는지 본다.
2. 시험을 돌리고 결과를 PR에 붙인다(`autorobot`이면 `python3 -m pytest -q tests`).
3. 일찍 **Draft PR**을 연다. 다른 사람이 같은 일을 하지 않게 하는 가장 싼 방법이다.
4. PR 템플릿의 "시험 종류"를 정확히 체크한다. **모의 시험 통과 ≠ 실물 검증.**
5. 리뷰 1명 승인 후 병합. 병합은 **Squash** 기본, 여러 커밋이 의미 있으면 Merge.
6. 병합 후 브랜치 삭제.

다른 사람 PR 위에 쌓아야 하면(예: #4가 #2 위) PR 본문 첫 줄에 "#2 다음에 병합"이라고 쓰고, 아래쪽 PR이 먼저 병합되면 위쪽을 `main` 기준으로 rebase한다.

## 4. 로봇에 올릴 때 (작동본 보존)

```bash
git tag -a robot-auto-20261114-a -m "Mega MS1 + Jetson mission, Eval2 rehearsal" <커밋>
```

```bash
git push origin robot-auto-20261114-a
```

- 태그가 없는 커밋은 로봇에 올리지 않는다. 롤백 = 직전 태그 checkout.
- Jetson의 작업 폴더에서 코드를 직접 고치지 않는다. 현장에서 상수를 바꿨다면 그날 `field/<날짜>` 브랜치로 커밋·PR한다.
- `tools/field_log.py` 기록에는 `--commit <태그>`를 붙인다.

## 5. 질문·결정은 Issue로

조교 질문, 실물 확인(모터 모델·전압, 정지 테이프 반응 등), 결정 사항을 채팅에만 두지 말고 Issue로 남긴다. 라벨: `rule-question`, `hardware-check`, `auto`, `manual`, `eval2-blocker`. 답이 나오면 Issue에 출처(공지 번호·날짜)를 적고 닫는다.

## 6. 올리지 않는 것

비밀번호, SSH 키, Tailscale 주소, `config/*.local.json`, 실행 로그·영상·사진, 학생 개인정보와 연락처. 수업 원본 자료(룰북 PDF, Classum 원문)는 공개 저장소에 올려도 되는지 팀에서 먼저 정한다. 큰 바이너리(모델 `best.pt`)는 브랜치 대신 GitHub Release에 올리고 SHA-256을 문서에 적는다.

## 7. 저장소 관리자 설정 (1회)

GitHub → Settings → Branches → `main` 보호 규칙: PR 필수, 승인 1명, (CI가 생기면) 상태 검사 통과 필수, force push 금지. 이렇게 하면 웹 "Add files via upload"로 main에 바로 들어가는 일을 막는다.
