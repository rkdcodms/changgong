# 학습 모델

기존 `add-yolo-best-pt` 브랜치의 검증된 모델을 그대로 사용한다. `autorobot` 폴더에서 다음 명령으로 받는다.

```sh
python3 scripts/download_model.py
```

다운로드는 아래 커밋의 파일을 사용하며, 크기와 SHA-256이 맞을 때만 `models/best.pt`에 저장한다. 모델을 실행하거나 다시 학습하지 않는다.

- 저장소: `rkdcodms/changgong`
- 커밋: `9f9d66dfd971276d2fbb0ab79bd2d233a6898df9`
- 파일: 저장소 루트의 `best.pt` (5,507,098 bytes)
- SHA-256: `e67df5f04399513f8578a6948843902ea31b346c558375af88e919834283bd84`

이미 설치된 같은 모델을 사용하려면 로컬 설정의 `weights`에 실제 경로를 적는다. 실행기는 그 파일의 해시도 확인한다. 새 모델을 학습했다면 클래스 순서와 제어 코드의 모델 검증 값을 함께 검토해야 한다.
