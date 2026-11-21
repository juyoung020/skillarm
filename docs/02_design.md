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
| pyroki IK (JAX) | **CUDA 배치 IK**: 시드 수천 개를 한 번에 DLS 로 풀고 FK 로 수렴 확인 |
| cuRobo | **CUDA 충돌 검사 + 궤적 최적화**: MJCF 메시 점·책상 평면·장애물 상자 |
| 실물 YAM | **SO-101**: 라즈베리파이 4 위 서보 서버(C++, Feetech 프로토콜) ↔ PC 하네스(TCP) |
| 카메라 | 시뮬: MuJoCo RGB-D 렌더. 실물: RealSense D435(깊이) |

## 2. 구성 (CMake)

```
skillarm/
  include/skillarm/     공개 헤더: api.hpp(에이전트용 도구), types.hpp
  src/core/             좌표·단위·계약 타입, JSON 직렬화, 실행 기록기
  src/sim/              MuJoCo 환경 어댑터(EnvironmentAdapter 9개 메서드)
  src/kin/              FK, IK(CPU 기준 구현 + CUDA 배치)
  src/plan/             경로 계획, 충돌 검사(CPU 기준 구현 + CUDA)
  src/perceive/         분할(TensorRT), 점군, 파지 후보
  src/runner/           program.cpp → DLL 컴파일·로드·시드 반복 실행
  src/real/             실물 어댑터(TCP 클라이언트)
  servo/                파이 서보 서버(C++, 리눅스) — Feetech 버스, 안전장치
  tools/                CLI(skillarm)
  skills/               스킬 md (원본에서 옮기고 고침)
  models/so101/         MJCF, STL
  config/               캘리브레이션
```

## 3. 성능 원칙
- 시뮬·IK·충돌은 **같은 프로세스 메모리**에서 — 원본의 HTTP 서비스 왕복(수 ms~수십 ms)을 없앤다.
- CPU 기준 구현을 먼저 두고, CUDA 판은 결과를 기준 구현과 비교해 같을 때만 기본으로 쓴다.
- GPU 는 공중전 강화학습과 같이 쓰므로, GPU 를 쓰는 실행 전에 그쪽에 알린다. CPU 경로로도 전부 동작해야 한다.

## 4. 단계 (각 단계는 실제로 돌려 본 결과로 끝낸다)

| 단계 | 내용 | 끝났다는 증거 |
|---|---|---|
| M0 | CMake 뼈대, MuJoCo 로 SO-101 불러 FK, 관절 한계, 책상 간섭 검사(`ref_path_check.py` 이식) | 검사 결과가 파이썬 참고본과 같은 값 |
| M1 | 파이 서보 서버(C++): 프로토콜, 캘리브레이션, 위치 스트림, 충돌 감지 이동 | 실제 팔 0° 자세 → 쉬는 자세 왕복 |
| M2 | IK(CPU) + CUDA 배치 IK | 무작위 목표 1000개 오차 < 1 mm, CPU·GPU 결과 일치 |
| M3 | 시뮬 RGB-D 렌더, 장면에 물체 놓기, 과제 정의(집어 옮기기) | 키프레임 이미지 |
| M4 | 도구 24개 + 실행 기록기 + program.cpp DLL 러너 | 사람이 쓴 프로그램이 과제 성공 |
| M5 | 분할(TensorRT) + 파지 후보 | 시뮬 물체 마스크 |
| M6 | 스킬 md·프롬프트 이식, 코디네이터/서브에이전트/검증 루프 | 에이전트가 실패 → 수정 → 스킬 1개 등록 |
| M7 | 실물 어댑터 + D435 | 같은 프로그램이 실물에서 동작 |
| M8 | CUDA 충돌·궤적 최적화 | 장애물 있는 과제 |
