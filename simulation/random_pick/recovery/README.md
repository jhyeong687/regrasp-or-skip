# SO-ARM101 촉각 유무 복구 비교

직전 RGB-D 파지 파일럿에서 사용한 물체 종류, 초기 위치 및 평면 회전 12개를 읽는다. 각 물체에서 두 개는 첫 TCP 위치를 X 방향으로 치우치게 하고 두 개는 패드 접촉의 마찰계수를 낮춘다. 원래 12개 물체 배치를 모두 보존한다. 편심은 파지 지점의 편심으로, 질량중심에는 변화를 주지 않는다.

각 배치는 촉각 제공과 촉각 제외를 한 번씩 물리 초기화부터 재실행한다. 관측 이미지 SHA-256, TCP·그리퍼 간격 및 진단 구간의 양쪽 패드 샘플 요약을 비교한다. 동일한 초기 관측을 확인하고 촉각 제공 모드에만 PaXini 법선력·전단력·압력 중심을 전달한다. 두 모드 모두 접촉 전의 좌우·손목 RGB와 손목 깊이, 파지 및 시험 리프트 후의 좌우 RGB를 받는다. 첫 TCP의 실제 편심과 마찰계수는 진단 모델의 입력에 포함하지 않는다. 각 결함 조건에서 먼저 실행하는 관측 모드를 번갈아 배정한다.

Astra는 시험 리프트를 관측한 후 회차당 한 번 위치 보정 또는 그립 보강을 선택한다. 위치 보정은 그리퍼를 열고 원래 RGB-D 파일럿에서 얻은 시각적 중심을 향해 재파지한다. 그립 보강은 시험 리프트에서 내려온 위치를 유지하고 더 단단히 닫는다. 동작은 기존 IK를 사용한다. 성공은 후퇴 후 트레이 바닥과 접촉하고, 안쪽에 있고, 속도가 충분히 낮고, 열린 손가락과 접촉하지 않는 상태다.

같은 12개 평가 배치를 반복해 마찰을 선택하지 않기 위해 별도 시드로 물체마다 네 개의 보정용 위치를 만들었다. 양쪽 동작을 물리적으로 실행해 안착 성공이 가장 크게 갈리는 후보를 먼저 택한다. 동률에서는 보강 성공이 많은 값과 시험 리프트 성공이 적은 값을 우선한다.

```sh
python simulation/random_pick/recovery/calibrate.py --scene /path/to/scene.xml \
  --out /path/to/comparison/calibration.json
python simulation/random_pick/recovery/run.py --scene /path/to/scene.xml \
  --source /path/to/previous-pilot/results.json \
  --calibration /path/to/comparison/calibration.json --out /path/to/comparison
python simulation/random_pick/serve.py --out /path/to/comparison \
  --page simulation/random_pick/recovery/index.html --port 62654
```

출력 디렉터리는 미리 만들고 두 스크립트에서 같은 디렉터리를 지정한다. 실행에는 기존 Codex CLI 로그인, MuJoCo, NumPy 및 Pillow가 필요하다. 대시보드는 로컬에서 동작한다.

`total_s`는 초기화부터 시험 관측, 이번 복구 판단의 Astra 호출, 실행, 안착 판정과 기본 로그 저장까지의 실제 경과 시간이다. 모델 대기 중에 멈추는 `sim_s` 및 호출에만 걸린 `inference_s`도 별도로 남긴다. 이전 RGB-D 파일럿에서 수행한 초기 파지 선택의 시간과 회차별 영상 인코딩은 `total_s`에 포함하지 않는다. `runs.csv`는 회차별 결과를, `results.json`은 전체·결함별·배치별 비교를 담는다. 원래 모델 응답과 실제 입력은 회차별 JSON으로 남긴다.

## 접촉 이력 추가 실험

`--contact-history`는 촉각 제공 모드에 닫힘·시험 리프트·유지 구간 전체의 접촉 이력을 추가한다. 기존 PaXini 입력에 쓰인 0.15 N 법선력 접촉 임계값을 사용한다. 각 구간의 패드별 접촉 샘플 수, 양쪽 동시 접촉 샘플 수, 최대 법선력과 중간 압력 중심을 기록한다. 실제 그리퍼 간격과 명령 간격의 차이를 함께 제공한다. 접촉 이력의 분류는 센서와 그리퍼 명령으로 계산한다. GPT-6.2 Astra `xhigh`가 각 회차에서 복구 동작을 결정한다.

촉각 제외 모드는 이전 실험과 동일한 카메라 영상, 명령문, 관절 관측을 사용한다. `--baseline`에 이전 결과를 주면 같은 설정인지 확인하고 이전과 이번 회차의 실제 입력 영상, 초기 물리와 시험 리프트 판정을 대조한다. 대시보드에는 이전과 개선 후의 안착률, 총 시간 및 호출 시간 중앙값이 표시된다.

```sh
python simulation/random_pick/recovery/run.py --scene /path/to/scene.xml \
  --source /path/to/previous-pilot/results.json \
  --calibration /path/to/previous-comparison/calibration.json \
  --baseline /path/to/previous-comparison/results.json --contact-history \
  --out /path/to/contact-history-comparison
```

## 빈 파지 후 그립 보강 동작

`--clearance-recovery`는 Astra가 그립 보강을 선택한 뒤 시험 리프트 유지 구간의 실제 턱 간격이 명령 간격에 1 mm 이내로 도달했으면 다른 경로로 실행한다. 파지 위치 위에서 먼저 턱을 연다. 열린 채로 원래 파지 위치까지 내려가 14 mm 명령으로 조이고, 짧게 안정화한 다음 같은 손목 피치로 8 mm를 들어 올린다. 이후 기존 궤적으로 트레이에 놓는다. 이미 물체가 턱 간격을 벌린 채 유지되어 있으면 기존 그립 보강을 수행한다.

이 분기는 두 모드 모두에 주어지는 그리퍼 간격을 사용한다. 기본 파지 위치, 힘 제한, 그립 보강의 명령 간격, 마찰계수 및 Astra 입력은 직전 접촉 이력 실험과 같다. `recovery_motion`에 분기 여부와 관측된 유지 간격을 기록한다. 이 경로는 회차의 MuJoCo 물리 시간을 2.36초 늘린다.

```sh
python simulation/random_pick/recovery/run.py --scene /path/to/scene.xml \
  --source /path/to/previous-pick-pilot/results.json \
  --calibration /path/to/original-recovery/calibration.json \
  --baseline /path/to/contact-history-comparison/results.json --contact-history \
  --clearance-recovery --out /path/to/clearance-comparison
```

## RGB-D 정렬 관측과 원본 센서 저장

`--vision-alignment`는 MuJoCo 작업대에 정면 RGB-D 관측과 양쪽 턱의 시각 표식을 추가한다. 표식은 접촉하지 않는 시각 요소다. 파지 직전 손목 깊이에서 주황색 물체 상단 윤곽을 구하고, 파지 시점에 정면 색과 깊이로 양쪽 표식의 위치를 복원한다. 같은 좌표계에서 턱 중심과 물체 윤곽의 상대 거리를 계산한다. 형상 중심과 표식 좌표에는 렌더한 RGB와 깊이 및 보정 행렬을 사용한다. 표식이 가려지면 정렬 수치를 제공하지 않는다. 두 모드 모두 동일한 정렬 영상과 수치를 받고, 해당 모드의 이전 입력도 유지한다. 복구 선택은 Astra가 수행한다.

회차마다 `*-rgbd.npz`에 단계별 RGB, 손목·정면의 깊이(m), 손목 물체 마스크, 유효 범위와 카메라 보정값을 기록한다. `*-paxini.npz`는 40 ms 간격의 `forces_N[t, pad, 77, xyz]`, 위치, 좌·우 이름, 위상과 시간을 담는다. 물리 패드 샘플은 두 모드에서 기록해 비교할 수 있다. 촉각 제외 모드의 모델 입력에는 이 데이터가 포함되지 않는다. 기존 RGB-D 몽타주와 새 정렬 패널이 포함된 모델 입력을 각각 저장한다. 센서 파일 압축과 재생 파일 인코딩은 회차별 실제 경과 시간 측정이 끝난 뒤 실행한다.

## 새 물체와 시야 가림 비교

`occlusion.py`는 육각기둥과 타원기둥을 이전 평가에 쓰지 않은 네 좌표에 놓는다. 각 좌표와 같은 마찰·그리퍼 오차를 무가림, 정면 표식 가림, 정면과 양쪽 RGB의 파지 구역 가림에서 각각 실행한다. 따라서 물리 배치 네 개를 시야 조건마다 한 번씩 사용하며, 시야별 촉각 제공·제외의 페어는 같은 물리와 관측에서 시작한다.

파지 접근 전 손목 RGB-D는 물체 위치 추정에 사용한다. 시야 가림은 이후 카메라 앞에 비충돌 패널을 렌더링한다. 정면 가림에서는 기존 양쪽 RGB가 남고, 강한 가림에서는 시험 파지·리프트 시점의 양쪽 RGB 작업 구역도 가려진다. 두 조건 모두 Astra가 원래 관측으로부터 파지 복구를 고른다. 각 시야 안에서는 촉각 데이터와 이에 대한 설명만 두 모드 사이에서 다르다. 순서를 조건별로 번갈아 배정한다. 두 새 물체의 저마찰계수 0.125는 별도의 보정 좌표에서 확인하고 고정했다. 최종 배치를 평가한 결과는 원래 물리 행동의 검증과 함께 기록할 수 있다.

```sh
python simulation/random_pick/recovery/occlusion.py --scene /path/to/scene.xml \
  --out /path/to/novel-occlusion
python simulation/random_pick/serve.py --out /path/to/novel-occlusion \
  --page simulation/random_pick/recovery/index.html --port 62658
```

## 30쌍 가림 강도 실험

`--suite graded`는 모서리 절삭 블록, 둥근 직사각기둥, 테이퍼 블록 세 개에서 편심과 저마찰을 한 배치씩 사용한다. 여섯 배치를 모두 다섯 단계의 시야에서 반복한다. 단계는 추가 가림 없음, 정면 표식 가림, 정면과 오른쪽 카메라 가림, 왼쪽 카메라의 좁은 수평 틈만 유지, 양쪽 측면 가림이다. 손목 RGB-D의 최초 배치 확인은 전 단계에 있다. 여섯 배치는 기존 가림 실험과 마찰 보정에 쓰지 않은 자세다.

마찰 보정은 별도 자세에서 짧은 리프트가 실패하고 그립 보강으로 안착 가능한 계수에 한해 가장 높은 값을 고른다. 보정 뒤 물리 설정과 다섯 개의 패널 배치를 고정하고 Astra 결과를 수집한다. 화면에서는 안착률, 경과 시간 중앙값, 추론만 걸린 시간 중앙값, 촉각 제공과 제외 가운데 한쪽만 성공한 페어를 표시한다. 추론이 기술 오류를 반환하면 같은 관측으로 한 번 다시 호출한다. 같은 회차의 시간에는 실제 대기가 모두 포함된다.

```sh
python simulation/random_pick/recovery/occlusion.py --scene /path/to/scene.xml \
  --out /path/to/calibration-folder --calibrate-only
python simulation/random_pick/recovery/occlusion.py --scene /path/to/scene.xml \
  --suite graded --calibration /path/to/calibration-folder/calibration.json --out /path/to/graded-results
python simulation/random_pick/serve.py --out /path/to/graded-results \
  --page simulation/random_pick/recovery/index.html --port 62659
```
