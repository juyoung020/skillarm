# 02. skillarm 설계 (C++ / CUDA / Rust)

ASPIRE(`docs/01_aspire_analysis.md`)의 학습 루프를 SO-101 하나에 맞춰 다시 만든다.
쓰는 언어는 **C++20, CUDA, Rust** 셋뿐이다. 원본에서 파이썬이 하던 일은 모두 C++ 로 옮긴다.

## 1. 바뀌는 것과 그대로인 것

| 원본 ASPIRE | skillarm |
|---|---|
| 에이전트가 쓰는 프로그램: 제한 Python(`import math` 만, AST 검사) | **제한 C++**: 에이전트가 `program.cpp` 한 파일을 쓰면 하네스가 MSVC 로 DLL 컴파일(1~3 s) → 불러서 `run(Api&)` 실행. 허용 헤더는 `skillarm/api.hpp` 와 `<cmath>` 만, 헤더 검사로 강제 |
| 도구 24개 (`api.py:1533-1558`) | 같은 이름·같은 의미의 C++ 함수. 단위 m·rad, 쿼터니언 wxyz, 카메라 OpenCV 관례, 집게 0=닫힘 1=열림 |
| 실행 기록 `run.json`·`outcome.json`·`trace/events.jsonl`·`episode/steps.jsonl`·`media/keyframes` | **키 이름·디렉터리 그대로** — 스킬 md 를 고치지 않고 옮기기 위해 |
| 스킬 md, 코디네이터/서브에이전트/검증 에이전트 프롬프트 | 그대로 옮기고 Python 부분만 C++ 로 바꿈 |
| 시뮬레이터 LIBERO·Robosuite·BEHAVIOR | **MuJoCo 4.0.1 C API** + 공식 SO-101 모델(`models/so101`) |
| SAM3 (HTTP 서비스, PyTorch) | SAM 계열 → ONNX → **TensorRT** (C++), 같은 프로세스 |
| pyroki IK (JAX) | **DLS IK** (MuJoCo 자코비안, 시드 24개, FK 로 수렴 확인) — CPU 한 번 0.25 ms 라 GPU 로 옮기면 커널 실행 준비 비용 때문에 더 느림 |
| cuRobo | 관절 공간 보간 + 경유점마다 MuJoCo 책상 간섭 검사(CPU) |
| 실물 YAM | **SO-101**: 라즈베리파이 4 위 서보 서버(C++, Feetech 프로토콜, 30 Hz 연속 추종·막힘 정지·고장 잠금) ↔ PC 하네스(TCP) |
| 카메라 | 시뮬: MuJoCo RGB-D 렌더("scene" 고정, "wrist" 손목). 실물: 손목 C270(파이 `skillarm-camera` 가 MJPEG 전송, 자세는 순기구학+마운트 CAD) + **단안 깊이 모델**(MoGe-2, TensorRT). 축척은 책상이 화면 5 % 이상이면 책상 평면, 아니면 신경망 값. 그리퍼 자기 픽셀은 렌더로 가림 |

## 2. 구성 (CMake)

```
skillarm/
  include/skillarm/     공개 헤더: api.hpp(에이전트용 도구), types.hpp
  src/core/             캘리브레이션(스텝 ↔ rad), STL 로더
  src/sim/              SO-101 모델, 관절 이동 경로 책상 간섭 검사
  src/kin/              FK, DLS IK
  src/env/              EnvironmentAdapter 와 시뮬 과제(pick_cube)
  src/api/              에이전트 도구 24개 구현(계획·충돌 검사·파지 후보 포함)
  src/record/           ASPIRE 형식 실행 기록기
  src/perceive/         SAM3·MoGe-2 깊이(TensorRT, CUDA 전·후처리), ONNX 입력 상수화
  src/sim/sim_camera    MuJoCo 오프스크린 RGB + 미터 깊이(숨은 WGL 창)
  src/runner/           program.cpp → DLL 컴파일·로드·시드 반복 실행
  src/real/             실물 어댑터(서보 서버 TCP 클라이언트 + RealSo101)
  servo/                파이 서보 서버(C++, 리눅스) — Feetech 버스, 안전장치
  tools/                CLI(skillarm)
  skills/               스킬 md (원본에서 옮기고 고침)
  models/so101/         MJCF, STL
  config/               캘리브레이션
```

## 3. 성능 원칙
- 시뮬·IK·충돌은 **같은 프로세스 메모리**에서 — 원본의 HTTP 서비스 왕복(수 ms~수십 ms)을 없앤다.
- CUDA 는 측정으로 CPU 가 병목일 때만 쓴다. 지금은 인식 신경망(SAM3·깊이)뿐이다.
- GPU 는 공중전 강화학습과 같이 쓰므로, GPU 를 쓰는 실행 전에 그쪽에 알린다. CPU 경로로도 전부 동작해야 한다.

## 4. 단계 (각 단계는 실제로 돌려 본 결과로 끝낸다)

| 단계 | 내용 | 끝났다는 증거 |
|---|---|---|
| M0 | CMake 뼈대, MuJoCo 로 SO-101 불러 FK, 관절 한계, 책상 간섭 검사(`ref_path_check.py` 이식) | 검사 결과가 파이썬 참고본과 같은 값 |
| M1 | 파이 서보 서버(C++): 프로토콜, 캘리브레이션, 위치 스트림, 충돌 감지 이동 | 실제 팔 0° 자세 → 쉬는 자세 왕복 |
| M2 | IK(CPU) | 무작위 목표 500개 중 99.6 % 성공, 오차 < 0.5 mm, 0.25 ms (완료) |
| M3 | 시뮬 RGB-D 렌더, 장면에 물체 놓기, 과제 정의(집어 옮기기) | 640×480 한 장 1.5 ms, 역투영 바닥 오차 0.25 mm (완료) |
| M4 | 도구 24개 + 실행 기록기 + program.cpp DLL 러너 | 사람이 쓴 프로그램이 과제 성공 |
| M5 | SAM3 분할 + MoGe-2 깊이(TensorRT, CUDA 전·후처리) + 파지 후보 | 시뮬: `skillarm sim-perceive` 8/8, 카메라만으로 집기 7/10 (완료). 실물: 손목 C270 `skillarm real-observe` 65 ms, 평면 RMS 1.5–2.3 mm. 책상을 보는 자세에서 축척 확인 남음 |
| M6 | 스킬 md·프롬프트 이식, 코디네이터/서브에이전트/검증 루프 | 에이전트가 실패 → 수정 → 스킬 1개 등록 |
| M7 | 실물 어댑터(`src/real/real_so101`) + 서버 연속 추종 모드, `run --real` 은 작업자 확인 후 실행 | 같은 프로그램이 실물에서 동작 |
| M8 | 장애물 있는 과제: 충돌 검사를 장애물 상자까지 넓힘(CPU, 필요 측정되면 그때 가속) | 장애물 있는 과제 |
