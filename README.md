# skillarm

**A coding agent teaches a $150 robot arm new skills — and writes down what it learned.**
C++/CUDA, native Windows, SO-101.

[한국어 ↓](#한국어)

> **Status (2026-11-21): M0 done** — SO-101 loads in MuJoCo 4.0.1 from C++, and `skillarm path-check` simulates a
> joint-by-joint move with the wrist camera mount and refuses paths that come within 10 mm of the desk
> (same verdicts and millimetres as the Python reference on 5 recorded moves). Next: M1 servo server on the Pi.
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
| IK | pyroki (JAX) | **batched CUDA IK** (thousands of seeds at once) |
| Motion / collision | cuRobo | **CUDA collision checking + trajectory optimisation** against MJCF geometry |
| Simulator | LIBERO, Robosuite, Isaac (BEHAVIOR) | **MuJoCo C API** with the official SO-101 model |
| Real hardware | YAM driver | Raspberry Pi 4 + `so101-mcp` servo server (Rust, to be published here) with path checks and collision stop |
| Camera | multiple | one Logitech C270 |
| Agent loop, trace, skills | `.claude/skills/*/SKILL.md`, verifier | **same formats** — Claude Code reads and writes them unchanged |

## Plan

1. **Analysis** of ASPIRE's harness API, trace format and skill files → `docs/`.
2. **MVP in simulation:** SO-101 in MuJoCo, one pick-and-place task; the agent writes a program, runs it, reads the trace,
   fixes it, and records one verified skill.
3. **GPU pieces:** TensorRT SAM3, CUDA IK, CUDA collision/trajectory.
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

> **상태 (2026-11-21): M0 완료** — C++에서 MuJoCo 4.0.1로 SO-101을 불러, 손목 카메라 고정대까지 넣어 관절 이동 경로를
> 시뮬레이션하고 책상과 10 mm 안으로 가까워지는 경로를 거부합니다(기록해 둔 이동 5개에서 파이썬 참고본과 판정·mm 값 일치).
> 다음: M1 파이 서보 서버. 단계 표: [`docs/02_design.md`](docs/02_design.md).

- NVIDIA [ASPIRE](https://github.com/NVlabs/ASPIRE)는 로봇 "학습"을 가중치 대신 **스킬 md를 다듬는 일**로 바꿨습니다.
  에이전트가 인식·계획·제어 API로 프로그램을 짜고, 실행 기록을 보고 실패를 고치고, 검증된 수정을 스킬로 쌓습니다.
- skillarm은 같은 루프를 **SO-101 + C++/CUDA + 윈도우**로 새로 만듭니다.
  SAM3는 TensorRT로, 역기구학과 충돌 검사는 CUDA로, 시뮬레이션은 MuJoCo C API로, 실제 팔은 라즈베리파이의 안전장치 달린 서보 서버로.
- 계획: 분석 → 시뮬레이션 MVP(집어 옮기기 하나, 스킬 하나) → GPU 부품 → 실제 팔 → 측정 결과 공개.

라이선스 Apache-2.0. ASPIRE(Apache-2.0)의 설계를 참고했습니다([NOTICE](NOTICE)). NVIDIA와 관계없습니다.
