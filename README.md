# skillarm

**A coding agent teaches a $150 robot arm new skills — and writes down what it learned.**
C++/CUDA, native Windows, SO-101.

[한국어 ↓](#한국어)

> **Status (2026-11-22):** the simulation loop works end to end on the CPU — an agent program (`program.cpp`) is
> checked, compiled to a DLL in ~1 s, run per seed in its own process, and leaves an ASPIRE-format run directory.
> A scripted pick-up in MuJoCo succeeds on 10/10 seeds; IK solves 99.6 % of random 6-DoF targets at 0.25 ms each.
> The real arm runs behind a C++ servo server on the Pi (continuous 30 Hz servo mode, blockage/overload stop with a
> latched fault). Perception runs on TensorRT: SAM3 (fp16) finds the cube in 55 ms per new frame (7 ms when only the
> text changes), MoGe-2 depth anchored to the desk plane is within 2.3–3.4 % of the rendered depth on 8/8 sim seeds, and a
> vision-only pick (`examples/pick_cube_vision.cpp`) succeeds on 7/10 seeds. ASPIRE's skill helper scripts (campaign setup,
> progress, held-out validation, skill-report and skill-library lint, watchdog, evolutionary-search evaluation) are
> `skillarm` subcommands in C++ ([`docs/03_skill_scripts_spec.md`](docs/03_skill_scripts_spec.md)).
> Milestones: [`docs/02_design.md`](docs/02_design.md).

## Idea

[NVIDIA ASPIRE](https://github.com/NVlabs/ASPIRE) showed that "training" a robot can be *skill refinement instead of
gradient descent*: a coding agent (Claude Code, Codex) writes a robot program against a perception / planning / control
API, runs it, reads a structured execution trace, finds why it failed, fixes it — and stores every verified fix as a
markdown skill that later tasks reuse. The trained model is a folder of `.md` files, not weights.

ASPIRE targets Linux, Python, big GPUs and research robots. **skillarm** rebuilds the same loop for a hobby arm:

| Piece | ASPIRE | skillarm |
|---|---|---|
| Robot | YAM bimanual, LIBERO / Robosuite / BEHAVIOR sims | **SO-101** (6× Feetech STS3215), real + MuJoCo |
| Language / OS | Python, Linux x86_64 | **C++20 + CUDA, Windows** (Linux later) |
| Segmentation | SAM3 (PyTorch service) | SAM3 → ONNX → **TensorRT** |
| IK | pyroki (JAX) | damped least squares on the MuJoCo Jacobian, 24 seeds, FK-verified (0.25 ms on the CPU) |
| Motion / collision | cuRobo | joint-space interpolation + MuJoCo collision check of every waypoint against the desk |
| Simulator | LIBERO, Robosuite, Isaac (BEHAVIOR) | **MuJoCo C API** with the official SO-101 model |
| Real hardware | YAM driver | Raspberry Pi 4 + C++ servo server (`servo/`) with path checks, rate limit and collision stop |
| Camera / depth | multiple RGB-D | one Logitech C270 + monocular metric depth network (MoGe-2 on TensorRT) |
| Agent loop, trace, skills | `.claude/skills/*/SKILL.md`, verifier | **same formats** — Claude Code reads and writes them unchanged |

## Plan

1. **Analysis** of ASPIRE's harness API, trace format and skill files → `docs/`.
2. **MVP in simulation:** SO-101 in MuJoCo, one pick-and-place task; the agent writes a program, runs it, reads the trace,
   fixes it, and records one verified skill.
3. **GPU pieces:** TensorRT SAM3 and monocular depth with CUDA pre/post-processing. (IK and collision checks stay on the CPU: at 0.25 ms per solve a GPU launch would be slower.)
4. **Real arm:** same programs on the physical SO-101 through the safety layer.
5. Publish measured success rates, cost and time — or the reasons it did not work.

## Related

- [feetech-id-split](https://github.com/juyoung020/feetech-id-split) — split same-ID Feetech servos without unplugging
  (built while setting up the arm this project uses).

## License

Apache-2.0. Inspired by and following the design of NVIDIA ASPIRE (Apache-2.0); see [NOTICE](NOTICE).
Not affiliated with NVIDIA.

---

## 한국어

**코딩 에이전트가 약 16만 원짜리 로봇팔에게 새 기술을 가르치고, 배운 것을 md 파일로 적어 둡니다.**
C++/CUDA, 윈도우 네이티브, SO-101.

> **상태 (2026-11-22):** 시뮬레이션 루프가 CPU만으로 처음부터 끝까지 돕니다 — 에이전트 프로그램(`program.cpp`)을 검사해
> 약 1초에 DLL로 컴파일하고, 시드마다 별도 프로세스로 돌려 ASPIRE 형식 실행 기록을 남깁니다. MuJoCo 스크립트 집기
> 10/10 시드 성공, IK는 무작위 6자유도 목표 99.6 %를 한 번에 0.25 ms로 풉니다. 실제 팔은 파이의 C++ 서보 서버 뒤에서
> 움직입니다(30 Hz 연속 추종, 막힘·과부하 시 정지하고 잠금). 인식은 TensorRT로 돕니다: SAM3(fp16)가 새 화면 하나에
> 55 ms(글자만 바뀌면 7 ms)로 큐브를 찾고, 책상 평면으로 축척을 맞춘 MoGe-2 깊이는 시뮬 시드 8개 모두 렌더 깊이와
> 2.3–3.4 % 차이, 카메라만 쓰는 집기 예제(`examples/pick_cube_vision.cpp`)는 10개 중 7개 성공. ASPIRE 스킬 보조
> 스크립트(캠페인 생성, 진행표, 보류 시드 검증, 스킬 보고서·스킬 라이브러리 검사, 감시, 진화 탐색 평가)는 C++ `skillarm`
> 하위 명령입니다([`docs/03_skill_scripts_spec.md`](docs/03_skill_scripts_spec.md)).
> 단계 표: [`docs/02_design.md`](docs/02_design.md).

- NVIDIA [ASPIRE](https://github.com/NVlabs/ASPIRE)는 로봇 "학습"을 가중치 대신 **스킬 md를 다듬는 일**로 바꿨습니다.
  에이전트가 인식·계획·제어 API로 프로그램을 짜고, 실행 기록을 보고 실패를 고치고, 검증된 수정을 스킬로 쌓습니다.
- skillarm은 같은 루프를 **SO-101 + C++/CUDA + 윈도우**로 새로 만듭니다.
  SAM3와 단안 깊이 모델은 TensorRT로(깊이 카메라 없이 C270 한 대), 역기구학·충돌 검사는 CPU로(한 번 0.25 ms), 시뮬레이션은 MuJoCo C API로, 실제 팔은 라즈베리파이의 안전장치 달린 서보 서버로.
- 계획: 분석 → 시뮬레이션 MVP(집어 옮기기 하나, 스킬 하나) → GPU 부품 → 실제 팔 → 측정 결과 공개.

라이선스 Apache-2.0. ASPIRE(Apache-2.0)의 설계를 참고했습니다([NOTICE](NOTICE)). NVIDIA와 관계없습니다.
