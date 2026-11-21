# 01. NVIDIA ASPIRE 소스 분석 — SO-101 Rust 포팅용

- 대상: `ref/ASPIRE` (NVlabs/ASPIRE main, `cap_harness` 0.1.0). 경로는 모두 이 클론 기준 상대 경로.
- 규모 실측: `src/` Python 30,133줄, `tests/` 14,947줄, `.claude/skills` 약 9,100줄(md+py). `third_party/*` 서브모듈(LIBERO-PRO, robosuite, sam3, curobo, contact_graspnet_pytorch, BEHAVIOR-1K)은 **비어 있음**(클론 안 됨) — 서비스 내부는 래퍼 코드만 보고 판단했다.
- 포팅 목표: SO-101 팔(Feetech, Pi4의 기존 Rust 도구로 구동) + MuJoCo `so101_new_calib.xml` + Logitech C270 1대 + Windows 11 / RTX 5070 Ti + Claude Code.

---

## 0. 한 줄 요약

ASPIRE = **"코딩 에이전트가 제한된 Python 로봇 프로그램(Code-as-Policy)을 쓰고 → 시뮬에서 여러 seed로 돌려 → 기록된 trace·keyframe을 읽고 → 고치고 → 일반화되는 패턴을 markdown skill 라이브러리에 쌓는다. 단, 독립 verifier가 그 skill만 보고 재현해야 라이브러리에 들어간다."**
코드(harness)는 이 루프의 **실행·기록·도구 계층**이고, 학습 알고리즘 자체는 전부 `.claude/skills/*/` 프롬프트 문서에 있다. 즉 포팅 대상은 (a) harness를 Rust로, (b) 프롬프트·skill 문서를 SO-101에 맞게 옮기는 것 두 갈래다.

---

## 1. 전체 구조

### 1.1 디렉터리별 역할

| 경로 | 역할 | 포팅 |
|---|---|---|
| `src/cap_harness/contracts.py` (1516줄) | 모든 타입 계약: Pose, CameraObservation, RobotState, Trajectory, 결과 타입, 민감 diagnostics 필터 | **코어** |
| `src/cap_harness/protocols.py` (218줄) | Provider/Adapter 경계(Protocol) 6종 | **코어** |
| `src/cap_harness/api.py` (2086줄) | `CapApi`: 프로그램에 주입되는 공개 도구 24개 구현 + 조합(localize, move_to_pose 등) | **코어** |
| `src/cap_harness/geometry.py` (430줄) | 쿼터니언/변환, mask→point cloud 역투영, PCA OBB, top-down grasp 선택 | **코어** |
| `src/cap_harness/runtime.py` (534줄) | 생성 프로그램 실행기: AST 검증, 안전 builtins, 도구 래핑·호출 trace | **코어** |
| `src/cap_harness/registry.py` (375줄) | `ToolRegistry`: 공개 도구 allowlist, embodiment 확장 접두사(`libero.` 등) | **코어** |
| `src/cap_harness/artifacts.py` (841줄) | `RunRecorder`: run 디렉터리, span trace, steps.jsonl, keyframe/overlay/video, manifest | **코어** |
| `src/cap_harness/run.py` (453줄) | 한 에피소드 오케스트레이션(adapter 생성→reset→provider 연결→실행→check_success→finalize) | 코어(벤치 분기만 교체) |
| `src/cap_harness/batch.py` (461줄) | `run-batch`: spawn 워커로 seed 다수 실행 | 코어 |
| `src/cap_harness/cli.py` (499줄) | `cap-harness doctor/manifest/validate/run/run-batch` | 코어 |
| `src/cap_harness/doctor.py` (1077줄) | 환경·서비스 진단 | 축소 |
| `src/cap_harness/environments.py` + `configs/environments.json`, `configs/profiles/*.json` | provider 포트·venv·GPU 배치·동시성 | 단순화 |
| `src/cap_harness/errors.py` | `ErrorCode` 23종 + `ApiError` | **코어** |
| `src/cap_harness/providers/` | sam3 / graspnet / pyroki / curobo HTTP 클라이언트+FastAPI 서비스, yam_kinematics(mink, in-process) | 교체 |
| `src/cap_harness/libero/`, `robosuite/`, `behavior/` | 벤치마크 어댑터·태스크 레지스트리·codec | 버림(참조만) |
| `src/cap_harness/yam_real/` (6389줄) | 실물 YAM 양팔: station 프로필, 캘리브레이션, RealSense, 레벨1 컨트롤러, arm server, MuJoCo 대역(sim.py) | **실물 SO-101 설계의 견본** |
| `src/cap_harness/agent/` | System 2: 실물용 MCP 서버(`mcp.py`), 지속 세션·job(`session.py`), 검토된 고정 프로그램 카탈로그 | 코어(실물 모드) |
| `src/cap_harness/validation/` | LIBERO/robosuite/behavior 회귀 검증, protocol evaluator | 버림 |
| `.claude/skills/iterative-debugging/` | **핵심 학습 루프**: coordinator/subagent/verifier 프롬프트, skill 라이브러리 `skills/*.md`, 캠페인 스크립트 | **이식(문서) + 스크립트 Rust화** |
| `.claude/skills/evosearch/` | K=8 후보 진화 탐색 변형 | 이식(후순위) |
| `.agents/skills/`, `AGENTS.md` | 실물 System 2 에이전트 실행 규칙, MCP 도구 사용법, 과제 레시피 | 이식(실물 모드) |
| `.codex/skills/` | Codex용 동일 skill 포인터 | 불필요 |
| `examples/` | 정답 프로그램 예시(verifier는 읽기 금지) | 참조 |
| `docs/` | api-reference, run-artifacts, 벤치별 가이드, system2 runbook | 참조 |
| `scripts/` | bootstrap(uv venv)·supervise_services·start_system2 등 bash | Windows용 재작성 |

### 1.2 핵심 모듈 의존 그래프

```
                 cli.py ─┬─ run.py ──────────────┬─ batch.py (spawn 워커가 run_program 반복)
                         │                       │
     ┌───────────────────┼───────────────┐       │
     ▼                   ▼               ▼       ▼
 *_adapter.py      providers/*/client  artifacts.RunRecorder ◄── TracedProvider(span 기록)
 (Environment       (HTTP → FastAPI         ▲
  Adapter)           service, 포트)         │ keyframe/span/step
     │                   │                  │
     └────────┬──────────┘                  │
              ▼                             │
          api.CapApi ──── geometry.py       │
              │  register_tools()           │
              ▼                             │
       registry.ToolRegistry                │
              ▼                             │
       runtime.ProgramExecutor ─────────────┘  (AST 검증 → exec(program.py))
              ▲
   contracts.py / errors.py / protocols.py  ← 모든 모듈이 의존하는 바닥층

 agent/session.LiveAgentSession ─ ProgramExecutor/CapApi 공유
 agent/mcp.CapMcpBridge ─ session (MCP 도구 노출, 실물 System 2)
```

근거: `run.py:9-18`(import), `run.py:289-364`(provider 래핑과 CapApi 생성, `ProgramExecutor(tools, recorder=recorder).execute_program(source)` at 364), `api.py:1531-1588`(register_tools).

### 1.3 실행 흐름 — 에피소드 1회 (`run.py:66-453`)

1. 벤치 선택 → 태스크 레지스트리 `resolve(suite, task_id)` → horizon으로 `max_steps` 결정 (`run.py:100-137`).
2. `RunRecorder` 생성: run 디렉터리와 `run.json(status=in_progress)` 기록 (`run.py:155-174`, `artifacts.py:286-394`).
3. adapter 생성·`reset(metadata, seed)` (`run.py:243-265`). seed가 장면 랜덤화를 결정.
4. provider 연결: SAM3(8114), Contact-GraspNet(8115), PyRoki(8116), cuRobo(8118)를 `TracedProvider`로 감싸 `CapApi`에 주입 (`run.py:292-361`).
5. `api.register_tools(ToolRegistry(...))` → `ProgramExecutor.execute_program(source)` (`run.py:361-364`).
6. 프로그램 종료 후 **host 전용** `adapter.check_success()` (특권 성공 판정, 프로그램엔 비노출) (`run.py:368`).
7. `termination_reason` 산출 → `recorder.finalize()` → `outcome.json`, `manifest.json` (`run.py:369-394`).

### 1.4 학습 루프 — 에이전트 관점 (iterative-debugging)

```
coordinator(메인 에이전트)                     subagent(태스크당 1개)                     verifier
────────────────────────                    ───────────────────────                 ─────────────
init_run.py --tasks → RUN_ROOT              Stage0: scene_snapshot 실행 → keyframe·
curriculum.md에 순서 결정 기록               overlay 보고 task_analysis.md
dispatch(subagent, GPU 할당) ─────────────►  v00_initial.py 작성 → run-batch(dev seeds)
GO IDLE                                     Stage1: 실패를 "모드"별 그룹화
                                             (perception/IK/execution/grasp-retention…)
                                             → v01,v02… 각 버전마다 dev seed 전체 재실행
                                             예산: 모드당 3버전, 총 8버전
                                             Step3 최다 성공 버전 → fix_code.py
                                             Step4 unseen seed 1회 측정(튜닝 금지)
                                             Step5 findings.md / Step6 skill_report.json
◄──────────────────────────── 요약(GPU, 성공률, audit PASS)
Stage2: run_validation.py로 held-out 1..N 평가
skill_report 처리: wrong→즉시 수정, misleading→trigger 좁힘/precondition
proposed_* 있으면 dispatch(verifier) ───────────────────────────────────────────────► skill+제안만 보고
                                                                                      재해결(5 seed,
◄─────────────────────────────────────────────────────────────── verified/refuted/void  2회 시도, ≥4/5)
verified만 skills/*.md에 승격, lint_skills.py
모든 태스크 done → phase3 rerun sweep(라이브러리 바뀐 저성과 태스크만 rerun_02/)
```

근거: `.claude/skills/iterative-debugging/SKILL.md:30-65`, `main-agent-prompt.md:36-66(상태기계)`, `:143-158(Loop)`, `:160-186(Coordinator Rules)`, `subagent-prompt.md:231-536(Stage0/1)`, `verifier-prompt.md:102-200`.

**구성 요약**
- **coordinator** = 사용자가 띄운 Claude Code 메인 세션. 직접 디버깅 금지, 배정·평가·라이브러리 큐레이션·GPU ledger 관리만 (`main-agent-prompt.md:160-166`).
- **actor(subagent)** = `Agent(subagent_type="general-purpose", run_in_background=True)` 로 태스크당 1개 (`main-agent-prompt.md:238-264`).
- **verifier** = 제안자의 코드·findings·run artifact·`examples/` 열람 금지, skill 문서와 `proposed_skills.md`만으로 재해결 (`verifier-prompt.md:48-86`).
- BEHAVIOR 변형은 "프로그램을 한 번에 쓰지 말고 code block을 하나씩 키운다" — 장기 과제에선 산출물이 프로그램이 아니라 라이브러리 (`behavior/SKILL.md:8-27`). **실물 SO-101 장기 과제에 참고할 프로토콜.**
- Single-GPU 모드: coordinator가 모든 `cap-harness run`을 직접 실행하고 subagent는 읽기 전용 분석 (`main-agent-prompt.md:593-606`). RTX 5070 Ti 1장이면 이 모드가 기본이 될 수 있으나, SO-101 MuJoCo는 CPU 시뮬이라 GPU 제약은 분할 모델에만 걸린다(8장).

### 1.5 실물 System 2 흐름 (YAM)

- `agent/server.py`가 FastAPI + `/mcp` 마운트, `/steer`(운영자 지시) 제공 (`agent/server.py:16-30`).
- MCP 도구는 작게 유지: `get_capability_graph`, `observe`, `get_job`, `read_operator_messages` + 직접 도구 `cap_get_robot_state`, `cap_localize_object`, `cap_move_to_joints`, `cap_move_to_pose`, `cap_move_synchronized`, `cap_set_gripper`, 확장 메타 + 검토된 고정 프로그램 `cap_program_go_home/observe` (`agent/mcp.py:196-205`, `:248-295`, `agent/program_catalog.py:36-48`).
- 동작 도구는 job으로 실행되고 **종료 결과 + 카메라 JPEG**를 같이 반환 (`agent/mcp.py:351-411`).
- 원격 호출자는 코드·경로를 보낼 수 없다("remote callers cannot supply code or paths", `program_catalog.py:35`). 실물에선 프로그램 대신 **도구 호출이 정책**이다.
- 실물엔 성공 판정이 없다: `YamRealAdapter.check_success()`는 항상 False ("this embodiment cannot tell", `yam_real/adapter.py:461-468`).

---

## 2. cap_harness 공개 API 전체

### 2.1 좌표·단위 규약 (전 API 공통)

- 위치 **m**, 관절 **rad**, 쿼터니언 **wxyz** 단위 길이(허용오차 1e-5) (`contracts.py:242-260`). 내부 일부(mink, LIBERO)는 xyzw라 경계에서 변환(`yam_real/kinematics.py:13`, `providers/yam_kinematics/client.py:89-92`).
- RPY 입력은 **도(deg)**, 고정축 XYZ: `R = Rz(yaw)·Ry(pitch)·Rx(roll)` (`docs/api-reference.md` "Cartesian pose inputs", `geometry.py:125`).
- 모든 Pose는 `frame` 문자열을 가지며 프레임 불일치는 즉시 오류(자동 변환 없음). EEF pose는 `RobotState.base_frame` 기준이어야 함(`contracts.py:378-384`).
- 카메라 프레임은 **OpenCV 관례(+X 오른쪽, +Y 아래, +Z 앞)**. MuJoCo(OpenGL) 카메라는 `diag(1,-1,-1)`로 뒤집어 게시 (`libero/adapter.py:567-571`). `camera_pose`는 base←camera 변환(`CameraObservation.camera_pose.frame == base_frame`).
- 그리퍼는 정규화 **0=닫힘, 1=열림** (`docs/api-reference.md` set_gripper).
- 관절 차원은 embodiment별 고정: `{"behavior":7,"libero":7,"robosuite":7,"yam_real":6}` (`contracts.py:15-22`). **SO-101 팔은 5관절(+그리퍼)** 이라 이 표·`home_joint_positions` 검증(`api.py:142-146`, 6 또는 7만 허용)을 바꿔야 한다.

### 2.2 프로그램에 주입되는 도구 (24개, `api.py:1533-1558`, `registry.py:11-38`)

| 계층 | 시그니처 | 반환 | tick 소비 | Provider 의존 |
|---|---|---|---|---|
| atomic | `get_task_context()` | TaskContext (+`episode_max_steps/used/remaining` 메타, `api.py:172-203`) | 0 | adapter |
| atomic | `get_observation(camera_names=None)` | Observation | 0 | adapter |
| atomic | `get_robot_state()` | RobotState | 0 | adapter |
| atomic | `segment_text(camera_name, text)` | SegmentationSet(최대 5, score순) | 0 | Segmentation(SAM3) |
| atomic | `segment_points(camera_name, points_px)` | SegmentationSet | 0 | Segmentation |
| atomic | `mask_to_point_cloud(mask, camera_name, target_frame="robot_base")` | PointCloud | 0 | 없음(depth 필요) |
| atomic | `crop_point_cloud(pc, lower_xyz, upper_xyz)` | PointCloud | 0 | 없음 |
| atomic | `estimate_geometry(pc)` | ObjectGeometry(PCA OBB) | 0 | 없음 |
| atomic | `generate_grasps(camera_name, mask, backend="contact-graspnet", max_candidates=5)` | GraspSet | 0 | Grasp |
| atomic | `solve_ik(target_pose, seed_joints=None, *, arm="primary", backend=None)` | IKResult | 0 | IK |
| atomic | `plan_motion(goal: Pose|joints, start_joints=None, *, arm, strategy=None)` | PlanResult | 0 | IK + (선택)Trajectory/Integrated planner |
| atomic | `plan_synchronized_motion(targets, strategy=None)` | SynchronizedPlanResult | 0 | Integrated(cuRobo) |
| atomic | `step(action: RobotAction)` | StepResult(reward 제거) | 1 | adapter |
| atomic | `execute_trajectory(traj, stop_on_termination=True)` | ExecutionResult | 다수 | adapter |
| atomic | `set_gripper(position, *, arm)` / `set_grippers(positions)` | ExecutionResult | 다수/1 | adapter |
| high | `localize_object(query, *, camera_name=None, target_frame=None)` | LocalizationResult | 0 | segment→project→OBB (`api.py:1147-1186`) |
| high | `select_grasp(grasps, strategy="top_down"|"highest_score")` | GraspCandidate|None | 0 | 없음 |
| high | `move_to_joints(target, tolerance=0.01, max_steps=120, *, arm)` | ExecutionResult | ≤max_steps | adapter (직접 servo 루프, `api.py:1201-1303`) |
| high | `move_to_pose(target_pose, tolerance=0.01, max_steps=120, *, arm, strategy=None)` | ExecutionResult | 다수 | plan_motion→execute→**도착 검증**(위치 ≤tol, 자세 ≤10°, `api.py:1305-1424`, 1391) |
| high | `move_synchronized(targets, ...)` | ExecutionResult | 다수 | 양팔용 |
| high | `open_gripper()` / `close_gripper()` | ExecutionResult | 다수 | close는 adapter의 contact-aware 구현 우선 (`api.py:1430-1448`) |
| high | `go_home(*, arm)` | ExecutionResult | 다수 | adapter.go_home 우선, 없으면 최초 상태로 최대 4회 재시도 (`api.py:1450-1529`) |
| ext | `<embodiment>.get_task_metadata()` / `.get_controller_metadata()` 외 BEHAVIOR 이동 확장 | dict | — | adapter (`api.py:1568-1588`, `registry.py:41-76`) |

주입 생성자 6개: `ArmCommand(mode="joint_position", target, gripper_position)`(mode는 joint_position만 허용, `contracts.py:486`), `MotionStrategy(ik_solver∈{pyroki,curobo,mink}, trajectory_planner∈{interpolation,curobo}, pose_planner∈{composed,curobo-integrated}, time_dilation_factor, interpolation_dt_s, maximum_trajectory_dt_s)` (`contracts.py:979-1036`), `Pose(position, quaternion_wxyz, frame)`, `RobotAction(arms)`, `Trajectory(...)`, `SynchronizedTrajectory(...)` (`runtime.py:76-85`).

### 2.3 주요 계약 타입 (`contracts.py`)

| 타입 | 필드 | 비고 |
|---|---|---|
| `Pose` (242) | position(3), quaternion_wxyz(4), frame | `as_matrix()` 4x4 |
| `CameraObservation` (283) | rgb(H,W,3 uint8 또는 [0,1] float), **depth_m(H,W) m**, intrinsics 3x3, frame, camera_pose(Pose), timestamp_s | depth 필수 |
| `RobotState` (336) | arm별 joint_positions/velocities, end_effector_poses, gripper_positions, base_frame, joint_names, timestamp_s, embodiment | 모든 arm 매핑 키 일치 강제 |
| `TaskContext` (424) | suite, task_id, task_name, language, family, metadata | language가 유일한 목표 출처 |
| `Observation` (450) | cameras{name→CameraObservation}, robot_state, task_context, timestamp_s | |
| `Trajectory` (528) | joint_positions(N,dof), dt_s, joint_names, planner, collision_aware, **expected_start(RobotState)**, arm, gripper_positions(N), embodiment | 실행 전 stale 검사(관절 0.02rad, 그리퍼 0.25, `api.py:63-80`) |
| `Segmentation`/`SegmentationSet` (675/726) | mask(bool HxW), label, score, camera_name, frame, box_xyxy | |
| `PointCloud` (866) | points(N,3), frame, colors | |
| `PlanningScene` (903) | point_cloud, voxel_size_m=0.01 | 공개 RGB-D만으로 구성(`api.py:1601-1651`) |
| `RobotPlanningContext` (932) | embodiment, model, joint_names, base_transforms, end_effector_links | 없으면 Panda로 추정(`api.py:1723-1743`) — **SO-101 adapter가 반드시 제공** |
| `ObjectGeometry` (1039) | pose, extents(3), point_count | |
| `GraspCandidate`/`GraspSet` (1070/1110) | pose, score, width_m, approach_distance_m | approach = pose 회전 z열(`geometry.py:358-399`) |
| `IKResult`/`PlanResult`/`SynchronizedPlanResult`/`LocalizationResult` | ok + 값 + error(ApiError) + diagnostics | |
| `StepResult` (1371) | ok, observation, terminated, truncated, reward(공개 시 제거) | |
| `ExecutionResult` (1410) | ok, steps_executed, status(ExecutionStatus 11종), final_observation, final_errors{arm→m or rad}, diagnostics | `public_view()`가 민감 키 제거 |

**결과 규약**: 예상 가능한 실패는 예외가 아니라 `ok=False + ApiError(code, message, recoverable, details)`로 반환, 계약 위반은 `ValueError` (`errors.py:1-6`). 민감 diagnostics 키(`mujoco`, `qpos`, `qvel`, `reward`, `success`, `object_pose`, `ground_truth`…)는 공개 결과에서 재귀 제거 (`contracts.py:1259-1319`).

### 2.4 geometry.py 핵심 알고리즘

- `mask_to_point_cloud` (259-313): 핀홀 역투영 `x=(u-cx)·d/fx, y=(v-cy)·d/fy, z=d`, depth>0만, 색 동반, `camera_pose`로 base 프레임 변환.
- `estimate_oriented_bounding_box` (316-355): 공분산 고유분해(PCA), 고유벡터 부호를 지배 성분 양수로 고정 + 오른손 기저 → 결정론적 OBB.
- `select_top_down_grasp` (358-399): approach축(z열)과 `-up` 내적 ≥0.8 중 최고 score.
- 그 외: `quaternion_wxyz_to_matrix`, `matrix_to_quaternion_wxyz`, `rpy_deg_to_quaternion_wxyz`, `pose_from_mapping`, `transform_points/point_cloud/pose`, `crop_point_cloud`, `invert_transform` (22-258).

### 2.5 plan_motion 기본 경로 (충돌 무시 보간) — `api.py:502-682`

```
goal이 Pose이고 pose_planner != composed → integrated planner(cuRobo) 위임
goal이 Pose → solve_ik(goal, start, backend=strategy.ik_solver)
trajectory_planner != interpolation → planner.plan_to_joints(..., scene=_planning_scene())
그 외: step_limits = min(액션bound/주파수, 0.05 rad)   (api.py:2019-2065)
       waypoint_count = ceil(max|Δq|/step_limits)       (api.py:655)
       선형 보간, collision_aware=False, dt=제어주기
```
subagent 프롬프트는 이 기본값이 **충돌 무시**임을 강하게 경고한다(`subagent-prompt.md:157-176`). 포팅 MVP는 이 경로만으로 시작 가능.

### 2.6 생성 프로그램 샌드박스 (`runtime.py`)

- 허용 import: `math` 하나, 별칭 금지 (`runtime.py:35-42`, 181-190).
- AST 금지 노드: `AsyncFunctionDef, Await, ClassDef, Delete, Global, ImportFrom, Nonlocal, With`; `_`로 시작하는 이름·속성 금지 (`runtime.py:159-200`). `def`, `lambda`, `for/while`, `try`는 허용.
- builtins 24개 allowlist(`abs…zip, print, Exception, ValueError, TypeError`) (`runtime.py:45-74`).
- 프로그램은 top-level `result`에 결과를 넣고, `print`는 `logs/program.stdout`으로 (`runtime.py:233-277`, `run.py:365-366`).
- 모든 도구 호출을 `CallTrace(index, tool, ok, duration_s, inputs_summary, outputs_summary, provider, frame, failure_code…)`로 기록, 동작 도구 앞뒤로 keyframe (`runtime.py:105-122`, `287-318`).
- README 경고: "trial process와 watchdog은 security sandbox가 아니다" — 격리 호스트 필요.

### 2.7 에이전트가 쓰는 프로그램 실례 (그대로 인용)

**(1) `examples/robosuite/cube_lifting_seed1.py` 전문** — 인식→접근→하강→파지→들어올림의 최소형:

```python
report = {}
opened = open_gripper()
cube = localize_object("red cube", camera_name="agentview")
report["localized"] = cube.ok

if cube.ok:
    points = cube.point_cloud.points
    xs = sorted([float(point[0]) for point in points])
    ys = sorted([float(point[1]) for point in points])
    zs = sorted([float(point[2]) for point in points])
    count = len(points)
    center = (xs[count // 2], ys[count // 2], zs[count // 2])
    state = get_robot_state()
    quat = state.end_effector_poses["primary"].quaternion_wxyz
    above = Pose(
        position=(center[0], center[1], center[2] + 0.12),
        quaternion_wxyz=quat,
        frame=state.base_frame,
    )
    approached = move_to_pose(above, tolerance=0.02, max_steps=300)
    report["approached"] = approached.ok
else:
    approached = None

if approached is not None and approached.ok:
    grasp = Pose(
        position=(center[0], center[1], center[2] - 0.005),
        quaternion_wxyz=quat,
        frame=state.base_frame,
    )
    descended = move_to_pose(grasp, tolerance=0.015, max_steps=250)
    report["descended"] = descended.ok
else:
    descended = None

if descended is not None and descended.ok:
    closed = close_gripper()
    report["closed"] = closed.ok
    pose = get_robot_state().end_effector_poses["primary"]
    lifted = move_to_pose(
        Pose(
            position=pose.position + (0.0, 0.0, 0.18),
            quaternion_wxyz=pose.quaternion_wxyz,
            frame=pose.frame,
        ),
        tolerance=0.02,
        max_steps=350,
    )
    report["lifted"] = lifted.ok

result = report
```

**(2) `examples/robosuite/cube_lifting_curobo_seed1.py` 앞부분** — 자유공간은 cuRobo, 접촉 구간은 보간으로 전략을 나눔:

```python
motion = MotionStrategy(ik_solver="pyroki", trajectory_planner="curobo")
contact_motion = MotionStrategy()
...
    above = Pose(
        # Keep the entire hand outside the cube's RGB-D collision surface.
        position=(center[0], center[1], center[2] + 0.30),
        quaternion_wxyz=quat,
        frame=state.base_frame,
    )
    approached = move_to_pose(above, tolerance=0.02, max_steps=300, strategy=motion)
...
    descended = move_to_pose(grasp, tolerance=0.015, max_steps=250, strategy=contact_motion)
```

**(3) `examples/libero/close_microwave.py` 앞부분** — 좌표 후보를 훑는 탐색형(주의: 고정 좌표 사용이라 subagent 프롬프트의 provenance 규칙(`subagent-prompt.md:109-128`)에는 어긋나는 "예시"):

```python
report = {"task": libero.get_task_metadata()["task_ref"], "attempts": []}
state = get_robot_state()
frame = state.base_frame
downward = state.end_effector_poses["primary"].quaternion_wxyz
for handle_x in (0.34, 0.39, 0.44, 0.48):
    attempt = {"handle_x": handle_x}
    go_home()
    open_gripper()
    handle = (handle_x, 0.04, 0.104)
    above = move_to_pose(
        Pose((handle[0], handle[1], handle[2] + 0.15), downward, frame),
        tolerance=0.05, max_steps=400,
    )
```

실물 YAM 예시는 3~5줄짜리(`examples/yam_real/joint_motion.py`: `go_home()` → 현재 관절+0.03rad → `move_to_joints(target, arm="right")`).

---

## 3. Providers (`src/cap_harness/providers/`)

공통: 각 provider는 **별도 venv의 FastAPI(uvicorn) 서비스** + harness 쪽 얇은 HTTP 클라이언트. JSON 본문, 배열은 `np.save`→base64 (`providers/wire.py:12-27`), 이미지는 PNG base64. `HttpProviderClient`는 408/425/429/5xx 재시도(기본 2회), 연결 실패는 `PROVIDER_UNAVAILABLE` (`providers/http.py:17-120`). 서비스는 세마포어로 동시 요청 제한(`CAP_HARNESS_SERVICE_CONCURRENCY`) + `/healthz` (`providers/service_runtime.py:40-61`). 포트·GPU는 `configs/environments.json`.

| Provider | 포트 / 경로 | 입력 → 출력 | GPU | 역할·특이점 | SO-101 대체 |
|---|---|---|---|---|---|
| **SAM3** (`sam3/`) | 8114, `POST /segment`, `/segment_point` (`sam3/wire.py`) | `{image_base64(PNG), text_prompt}` → `results[{mask_base64(raw bool), shape, box, score, label}]` 최대 5개 score순 / 점 프롬프트 → multimask | 필수(bf16 autocast, gated HF 가중치) | 텍스트 open-vocab 인스턴스 분할. confidence_threshold=0, 추론 후 `torch.cuda.empty_cache()` (`sam3/service.py:40-170`) | ort로 **GroundingDINO/OWLv2/YOLO-World(text→box) + SAM2/EfficientSAM/MobileSAM(box→mask)** 체인, 또는 class-agnostic 분할 + SigLIP2 텍스트 유사도 선택. SAM3 ONNX 공개 여부 미확인 |
| **Contact-GraspNet** (`graspnet/`) | 8115, `POST /plan_point_clouds` | 물체 cloud + 장면 cloud(npy b64) → 4x4 grasp 변환·score·contact | 필수 | **Panda 그리퍼 기준**(default width 0.08 m, `graspnet/client.py:60`), 로컬 z 90° 보정 행렬(`client.py:20-28`) | SO-101 그리퍼와 기하가 달라 부적합. OBB 기반 기하 grasp(top-down/측면, 폭≤그리퍼 개구) 직접 구현 |
| **PyRoki** (`pyroki/`) | 8116, `POST /ik` | `{target_pose_wxyz_xyz[7], prev_cfg}` → `{joint_positions, position_error_m, orientation_error_rad}` | 불필요(JAX CPU) | Panda URDF 고정(`--robot panda_description --target-link panda_hand`), 관절 한계 0.15rad 여유, pos 가중 50/ori 10 LM (`pyroki/service.py:45-152`). 클라이언트는 `libero/robosuite`만 허용하고 7관절 강제(`pyroki/client.py`) | MuJoCo `mj_jacSite` 기반 DLS/LM IK(Rust) + SO-101 해석적 IK |
| **cuRobo** (`curobo/`) | 8118, `POST /ik`, `/plan`, `/plan_synchronized` | 상태+목표(joint/pose)+`PlanningScene` 점군 → 시간 매개 궤적 | 필수(CUDA, 소스 빌드) | 점군→voxel mesh, 로봇 자기 점 carve(sphere), 첫 요청 ~50 s 워밍업, 422는 "진단"(어느 constraint 실패) (`curobo/service.py`, `curobo/client.py:31-57`, `main-agent-prompt.md:130-138`) | Windows 미지원 사실상. 관절공간 **RRT-Connect + MuJoCo 충돌검사**(점군 voxel → box geom 또는 occupancy grid) + 시간 매개화 |
| **YAM mink IK** (`yam_kinematics/`) | 서비스 없음(in-process) | Pose+RobotState → IKResult | 불필요 | MuJoCo 모델 위 mink QP IK, `seed()` 후 반복 20회, **FK로 수렴 검증 후에만 ok** (tol 0.01 m), 실패는 `IK_UNREACHABLE` (`yam_kinematics/client.py:41-188`, `yam_real/kinematics.py:125-175`) | **SO-101 IK의 직접 견본**. 같은 seam·같은 FK 검증 규칙으로 Rust 구현 |

규칙: 요청한 backend가 없으면 **조용한 대체 없이** `UNSUPPORTED` 반환 (`api.py:306-319, 413-425`; `docs/run-artifacts.md` "never substitutes a backend silently").

---

## 4. 트레이스 / 아티팩트 형식

### 4.1 디렉터리 (`artifacts.py:313-349`, `docs/run-artifacts.md`)

```
outputs/<benchmark>/<suite>/<task_id:02d>-<slug>/<seed:04d>/run-<UTCstamp>-<hex8>/   (기본)
<output-root>/<seed:04d>/<run-id>/                                                (--flat-run-dir)
├── run.json              identity·capture·providers·status(in_progress→finalized)
├── outcome.json          program_ok, task_success, protocol_success, cumulative_reward,
│                         steps_executed, simulator_duration_s, wall_duration_s, timing{buckets},
│                         termination_reason(+description), environment_terminated/truncated,
│                         success_observed_step, error, finalization_errors   (artifacts.py:679-703)
├── manifest.json         모든 파일 상대경로·size·sha256 (770-780)
├── source/               program.py 원본, program-result.json(공개 result), provenance.json(git), dependency-lock.json
├── episode/reset.json    reset 메타+관측
├── episode/steps.jsonl   tick당 1줄: step_index, simulator_time_s, action, robot_state, ok, terminated, truncated (486-511)
├── evaluation/protocol.json  (선택) host 전용 요약 판정
├── trace/events.jsonl    span_start/span_end(span_id, parent_id, name, category=program|api|provider, ok, duration_s) (437-470, 731-746)
├── trace/calls/call-<NNNNNN>-<slug>/input.json, output.json, error.json
├── trace/model/turns.jsonl   (선택) LLM 대화 trace 가져오기(비밀 redaction)
├── media/videos/<camera>.mp4 (H.264, tick당 1프레임), media/video-index.json(fps)
├── media/keyframes/<NNNNNN>-before-<tool>/ <camera>.png (+첫 1회 <camera>-depth.npy), metadata.json  (523-549)
├── media/overlays/       SAM top-1 마스크 오버레이 PNG + json (551-640)
├── payloads/<sha256>.npy 64원소 초과 배열 내용주소 저장
└── logs/program.stdout, program.stderr
```

### 4.2 JSON 직렬화 규칙 (`Serializer.encode`, `artifacts.py:149-261`)

- dataclass → `{"$type": 이름, 필드...}`; 64원소 이하 배열 → `{"$array_inline", dtype, shape}`; 큰 배열 → `{"$array": "payloads/<sha>.npy", dtype, shape, sha256}`; 2D bool 마스크 → `{"$type":"MaskSummary", shape, true_pixels}`(원본 미보존); PointCloud → frame·point_count·has_colors만(XYZ 미보존); CameraObservation → shape·intrinsics·camera_pose만(이미지 중복 저장 안 함); 비유한 float → `{"$float":"nan"}`; secret 키 `[REDACTED]`.
- JSON은 원자적 교체, JSONL은 매 줄 flush, 비디오는 `.<cam>.partial.mp4` → 종료 시 rename.
- `timing.buckets`: `adapter_step, setup, reset, record_step, video_encode, keyframe, overlay, trace_write, provider.<name>, finalize` 서로소, 나머지는 `unattributed_s`(프로그램 자체 Python) (`docs/run-artifacts.md` "Where the time went").

### 4.3 termination_reason (`artifacts.py:36-46`)

`program_completed, task_succeeded, environment_terminated, environment_truncated, step_limit, program_error, provider_error, harness_error, user_interrupt`. `task_success`와 별개 축.

### 4.4 subagent가 읽는 순서 (`subagent-prompt.md:296-305`)

`outcome.json` → `source/program-result.json` → `trace/events.jsonl`(실패 span, 빈 segmentation) → `media/keyframes/` → `media/overlays/` → `media/videos/` + `episode/steps.jsonl`. 이 순서·파일명이 skill 문서에 박혀 있으므로 **Rust 레코더는 같은 레이아웃·키 이름을 유지**하는 것이 skill 이식 비용을 최소화한다.

---

## 5. 스킬·프롬프트 파일

### 5.1 iterative-debugging (`.claude/skills/iterative-debugging/`)

| 파일 | 줄 | 핵심 |
|---|---|---|
| `SKILL.md` | 65 | 진입점. Setup(서비스·env) → Run Order 6단계(위 1.4). BEHAVIOR는 별도 프로토콜 |
| `main-agent-prompt.md` | 618 | coordinator: 상태기계(pending→stage1-done→done), skill 생명주기(proposed→verified/refuted), GPU ledger("One job per GPU, ever"), 서비스 확인, Loop, 규칙, 스킬 갱신 6a~6f, verifier 디스패치, curriculum, phase3 rerun, Stage2 명령, Single-GPU 모드 |
| `subagent-prompt.md` | 570 | 템플릿(TEMPLATE START/END). 범위·금지·provenance·허용 API·Benchmark Profiles·디렉터리 레이아웃·Stage0/1 Step1~6·반환 형식 |
| `verifier-prompt.md` | 219 | 읽기 규칙, 예산(5 seed, 2회, ≥4/5), v01은 skill을 "문자 그대로", 보고 JSON |
| `clean-task-slate.md` | 20 | 재실행 전 정리 |
| `skills/README.md` | 45 | 라이브러리 진입 경로(제안→verifier→coordinator 승격) |
| `skills/grasp.md` | 192 | 표준 pick&place 골격, top-down/원통 측면 grasp, 단계적 접근, 들어올린 뒤 재-localize로 grasp 검증 |
| `skills/localize.md` | 134 | SAM3 프롬프트·disambiguation·3D 위치 헬퍼 |
| `skills/transport.md` | 105 | waypoint, 충돌 없는 transit, placement 접근 |
| `skills/manipulation.md` | 65 | 서랍·노브·스위치·밀기 |
| `skills/task-exploration.md` | 140 | Stage0 scene snapshot 절차, `task_analysis.md` 양식 |
| `robosuite/*_fix_code.py` + `SUCCESS_RATES.md` | — | 과거 캠페인 결과 프로그램 |
| `behavior/` (SKILL, 3 프롬프트, skills 5개) | — | block-by-block 성장 프로토콜, 공유 라이브러리 |
| `scripts/` | — | `init_run.py`(campaign.json: dev/unseen/heldout seed), `gen_progress.py`(디스크 상태로 progress.md), `campaign.py`(skill_library_sha, skill_report 검증), `dispatch.py`(worker/verifier 패키지 스테이징), `lint_skills.py`, `run_validation.py`(Stage2), `watchdog.py`, `trace_stats.py`, `audit_seeding.py`, `scene_snapshot.py`(관측 전용 탐색 프로그램) |

**skill 항목 형식** (`main-agent-prompt.md:361-375`): 자유 서술 소절 = **Trigger**(증상/장면 조건) + **Code**(fix_code.py에서 뽑은 5~20줄 실코드) + **Why it works + evidence**(태스크·seed). 표는 한 줄 조회용만. "Merge, don't append". 항목은 **현재 참인 것만**, 이력은 `<file>-changelog.md` (`:377-396`). ~400단어 초과는 서술 과잉 신호.

**skill_report.json 스키마** (`subagent-prompt.md:437-470`): `schema_version, benchmark, suite, task_id, skill_library_sha, consulted[{skill, section, verdict∈useful|misleading|wrong|not-applicable, evidence}], proposed_edits[{skill, section, change, why}], proposed_new[{skill, title, trigger, code, evidence}]`.

**verification_report.json** (`verifier-prompt.md:156-180`): `seeds, attempts_used, trials, passes, verdict∈verified|refuted|void, skills_applied, proposals[{title, disposition∈sound|defective|untested, note}], gaps, notes`.

**중요 규칙 인용**

- "**Dispatch subagents — never debug yourself.**" / "**Go idle after dispatching.**" (`main-agent-prompt.md:162-165`)
- "**Only verified skills enter the library.** … An unverified proposal that reads well is exactly the failure this pipeline exists to prevent." (`main-agent-prompt.md:177-179`)
- "Debug by **failure mode, not by seed**" / "**Every version you write runs on all assigned development seeds**" (`subagent-prompt.md:266-273`)
- "**Budget: 3 versions per mode, 8 in total.**" (`subagent-prompt.md:349`)
- unseen seed: "**Do not fix anything against these seeds and do not run them twice**" (`subagent-prompt.md:386`)
- provenance: "Every task-dependent object choice, grasp or target pose, object dimension, and pixel location must be derived at runtime through the allowed public perception and CaP APIs." (`subagent-prompt.md:109-122`)
- "**Do NOT read any other task's directory**" — 공유 라이브러리만이 유일한 교차 태스크 채널 (`subagent-prompt.md:57-64`)
- "**Never run `rm`** … a command awaiting approval does not fail, it *waits*" (14시간 18분 블록 사례) (`subagent-prompt.md:73-79`) — Claude Code 권한 설정과 직결.
- "**A precondition is code or it is nothing.**" (`main-agent-prompt.md:347`)
- "A verifier passing … means the skill **transmits** … It does **not** mean the skill generalizes" (`main-agent-prompt.md:476-479`)
- 제안 체크리스트 7규칙: 모든 스칼라 유도, 프롬프트 문자열 명시, approach(stand-off) 명시, **tick 비용** 명시, 그리퍼 상태 명시, selector 양끝 제한·search는 None으로 시작, 측정 방법 공개 (`subagent-prompt.md:517-536`)
- `lint_skills.py` 오류 3종: sandbox가 거부하는 fence, 정의 안 된 helper, 빈 경우 없는 search (`scripts/lint_skills.py:1-30`).

### 5.2 evosearch (`.claude/skills/evosearch/`)

`SKILL.md`(53), `main-agent-prompt.md`(170), `subagent-prompt.md`(454), `skills/evosearch-iteration.md`(610), `scripts/evosearch_eval.py`(후보×seed 행렬), `analyze_evosearch_traces.py`. 반복마다 **K=8 후보**(각 docstring: Hypothesis / Differs from prior / Expected failure), 상위 3개를 씨앗으로 다음 세대, 중단: 최고 ≥80% 또는 5 iteration, candidate_A = 기존 fix_code.py 그대로 (`evosearch/subagent-prompt.md:216-300`). subagent가 Stage2까지 직접 수행(iterative-debugging과 다름).

### 5.3 실물 System 2 문서

- `AGENTS.md`: 우선순위(사용자 > skill), 시작 시 Home 1회, 실행 정책(짧은 과제는 즉시 MCP 시퀀스, 다단계는 계획 1회), **객체 중심 작업 상태** 유지, "Robot pose, gripper closure, and a bare success flag … do not by themselves prove grasp, placement", 비동기 job 처리, 복구 규칙.
- `.agents/skills/cap-tool-usage/SKILL.md`(102): MCP 액션 계약표, "Never disable collision checking to force a failed plan through", localize→move_to_pose→observe 체인.
- `.agents/skills/manipulation-tasks/` + `examples/yam_real/*/TASK_CONTEXT.md`: 과제 레시피(루빅스, 병뚜껑, 알약, 컵).

---

## 6. 환경 추상화와 새 로봇 추가 최소 인터페이스

### 6.1 연결 방식

`CapApi`는 `EnvironmentAdapter` Protocol(`protocols.py:144-194`)만 필수로 쓰고, 나머지는 **`getattr`로 feature-detect** 하는 선택 훅이다. 벤치 분기는 `run.py:100-137`(어댑터·레지스트리·확장 allowlist)와 `run.py:289-361`(provider 구성)에 하드코딩. `environments.py`/`configs/`는 서비스 토폴로지·GPU 배치 설정일 뿐 로봇 추상화가 아니다.

### 6.2 필수 메서드 (Protocol)

| 메서드 | 의미 |
|---|---|
| `embodiment` (property) | 식별자. `JOINT_DIMENSIONS`에 등록 필요 |
| `reset(task_ref, seed) -> Observation` | 결정론적 초기화(seed→장면) |
| `get_task_context() -> TaskContext` | language 포함 |
| `get_observation() -> Observation` | 보정된 RGB-D + 상태, tick 진행 없음 |
| `get_robot_state() -> RobotState` | |
| `step(RobotAction) -> StepResult` | **정확히 1 제어주기** |
| `execute_trajectory(Trajectory) -> ExecutionResult` | stale 검사, 종료 시 중단, dt=제어주기 |
| `set_gripper(pos, *, arm)` / `set_grippers(map)` | 정규화 그리퍼 |
| `close()` | idempotent |

### 6.3 선택 훅 (feature-detected)

| 훅 | 사용처 | SO-101 필요성 |
|---|---|---|
| `check_success()` | `run.py:368` host 판정 | 시뮬: 필수(MuJoCo 상태로), 실물: False 또는 별도 판정 |
| `get_planning_context()` | `api.py:1723` | **필수**(없으면 Panda로 추정) |
| `get_controller_metadata()` (`action_lower/upper_bounds`, `control_frequency_hz`, `control_period_s`, `arm_names`) | 보간 step 한계 `api.py:2019` | 필수 |
| `get_task_metadata()` | 확장 도구 | 권장 |
| `resolve_arm(arm)` | `"primary"` 별칭 해석 `api.py:370-396` | 단팔이면 불필요("primary" 하나) |
| `ik_request(pose, state, arm)` | EEF 프레임 오프셋 보정(robosuite `adapter.py:789`) | grasp site 정의에 따라 |
| `close_gripper(arm)` | contact-aware 닫기(정지 감지) | 실물에서 부하/전류로 구현 권장 |
| `go_home(arm)` | 보정 home으로 한 번에 | 권장 |
| `planning_workspace()` | 점군 crop `api.py:1666` | 권장 |
| `planning_scene_request(scene, arm)` | planner base 변환 | 불필요 |
| `monitor_snapshot()` | System 2 라이브 캐시 | 실물 MCP 시 |
| `native_env`, `native_step`, `compute_reward` | 검증 전용, 비주입 | — |
| `control_frequency`, `control_period_s`, `finish_episode_recording`, `bind_protocol_evaluator` | recorder/run | 권장 |

### 6.4 SO-101 MuJoCo 어댑터가 해야 할 일

1. `so101_new_calib.xml` + 테이블·물체를 포함한 scene MJCF 로드, seed로 물체 pose 랜덤화(태스크 레지스트리: `task_id → 장면 생성기 + 성공 술어 + horizon + language`).
2. 제어: `ArmCommand(joint_position)` → position actuator ctrl, 제어주기(예: 20~30 Hz) 동안 `mj_step` n회.
3. 카메라: C270 위치·FOV와 맞춘 MJCF 카메라에서 RGB + depth 렌더(`mjr_readPixels`의 비선형 depth → `near/(1-d·(1-near/far))` 형태로 m 변환, 확인 필요), 내부 파라미터 = fovy로부터 `f = 0.5·H/tan(fovy/2)` (`libero/adapter.py:559-565`), OpenGL→OpenCV 축 뒤집기.
4. EEF pose: MJCF의 그리퍼 site(예: gripper 사이 TCP site)를 base 프레임으로.
5. 그리퍼 정규화: gripper 관절 범위 → [0,1](0=닫힘).
6. `check_success`: 물체 body 위치 등 특권 상태(프로그램 비노출).

### 6.5 실물 SO-101 어댑터 (YAM 견본 대응)

YAM 구조(`yam_real/control/controller.py:1-22`): **레벨1**(PC, waypoint를 wall clock으로 재샘플) → **레벨2**(arm server, ~100 Hz 최신 목표 재송신, 버스 소유) → **레벨3**(모터 드라이버). SO-101 대응: 레벨2 = Pi4의 기존 Rust 도구(Feetech 버스 소유, hold loop), PC ↔ Pi4는 LAN RPC. YAM `transport.py`는 portal RPC로 `get_observations / command_joint_pos / command_joint_state / get_health`만 노출(`yam_real/transport.py:24-52`) — **Pi4 도구에 필요한 최소 RPC 면**이 이것이다. 그 밖에:
- station 프로필(`station.yaml`: base_frame, home/ready joints, joint limits, 카메라 serial, calibration bundle sha 검증) (`configs/yam_real/yam-example/station.yaml`).
- 카메라 extrinsics `T_base_from_camera` (calibration.json) — C270용 ChArUco/AprilTag 보정 필요.
- `allow_physical_motion` 게이트 없으면 모든 동작 거부(`yam_real/adapter.py:147-170`).
- MuJoCo 대역 plant(`yam_real/sim.py`): 실물 제어 경로를 그대로 MuJoCo에 물려 검증 — SO-101도 같은 방식(동일 Rust 컨트롤러가 Pi4 대신 MuJoCo를 구동)이 유용.

---

## 7. 로봇/벤치 특화 vs 범용 코어

| 범주 | 범용 코어 (포팅) | 로봇·벤치 특화 (대체/삭제) |
|---|---|---|
| 타입 | contracts 전부(단 `JOINT_DIMENSIONS`, `DEFAULT_EMBODIMENT="robosuite"`) | embodiment 표 값 |
| API | `CapApi` 24개 도구, stale 검사, 도착 검증, 민감정보 필터 | `_planning_context` Panda fallback(1723), `mask_to_point_cloud` 기본 프레임 `"robot_base"`(LIBERO), `default_camera="agentview"`, 그리퍼 0/1만 허용 로직(LIBERO 속도형 그리퍼) |
| 기하 | geometry.py 전부 | — |
| 실행기 | runtime 검증·trace·keyframe | Python AST 자체(언어 선택에 따라 재구현) |
| 기록 | artifacts 레이아웃·직렬화·timing | BEHAVIOR video stride, `_CLOSE_AFTER_RECORDING` |
| 오케스트레이션 | run/batch 골격, termination 체계 | `run.py` 벤치 if-분기, protocol evaluator(robosuite bimanual, behavior pickup) |
| Provider | Protocol 5종, HTTP 클라이언트 재시도·동시성 패턴, TracedProvider | SAM3/PyRoki(Panda)/cuRobo(franka.yml, yam yml)/Contact-GraspNet(Panda) 구현 |
| 어댑터 | Protocol + 선택 훅 목록 | libero/robosuite/behavior 전부, yam_real(DaMiao CAN, RealSense D405, 양팔) |
| 실물 | agent/session(job·steer), mcp(도구 축소·이미지 동봉), program catalog, station 프로필 개념, 3단 제어 구조 | YAM 모터·캘리브레이션 구현 |
| 학습 루프 | iterative-debugging 프로토콜·verifier·skill 형식·curriculum·rerun | LIBERO seed 51–65/66–80/1–50, `.venv-*` 경로, EGL 환경변수, 포트 표, `libero_*_swap/_task` 경고, skill 본문의 Panda·LIBERO 수치(tick, 높이) |

---

## 8. Rust 포팅 설계안

### 8.1 크레이트 구성 (cargo workspace)

```
aspire-rs/
├─ crates/
│  ├─ aspire-core       contracts.rs, errors.rs, geometry.rs(nalgebra), registry.rs, traits(Provider/Adapter)
│  ├─ aspire-api        CapApi (도구 24개 + 조합, stale/도착 검증, 민감 키 필터)
│  ├─ aspire-record     RunRecorder (serde_json, JSONL, PNG(image), npy(npyz), mp4(ffmpeg 파이프 또는 openh264))
│  ├─ aspire-script     프로그램 런타임(8.3 선택안) + 검증 + CallTrace + 도구 바인딩
│  ├─ aspire-mujoco     MuJoCo C API FFI(bindgen 또는 mujoco-rs), 오프스크린 RGB-D 렌더
│  ├─ aspire-kin        SO-101 FK/IK: 해석적 + DLS(mj_jacSite), FK 수렴 검증
│  ├─ aspire-plan       joint 보간(기본), RRT-Connect + MuJoCo/voxel 충돌검사, 시간 매개화
│  ├─ aspire-perceive   ort(ONNX Runtime) 분할·검출, mask→cloud, OBB, 기하 grasp
│  ├─ aspire-sim-so101  MuJoCo 어댑터 + 태스크 레지스트리(seed 랜덤화·성공 술어)
│  ├─ aspire-real-so101 실물 어댑터: Pi4 RPC 클라이언트, 레벨1 컨트롤러, C270 캡처(nokhwa/MediaFoundation), station 프로필·보정
│  ├─ aspire-mcp        System 2 MCP 서버(rmcp), job/steer 세션
│  └─ aspire-cli        aspire run / run-batch / doctor / campaign(init|progress|dispatch|lint|validate|watchdog)
├─ models/              MJCF(so101_new_calib.xml + scene), ONNX 가중치
├─ .claude/skills/      iterative-debugging·evosearch 이식본(SO-101 수치·명령으로 교체)
└─ docs/
```

### 8.2 Python 모듈 → Rust 대응

| Python | Rust | 메모 |
|---|---|---|
| `contracts.py` dataclass + `__post_init__` 검증 | `struct` + 생성자 `fn new() -> Result<_, ContractError>` (불변, `Arc<[f64]>`) | 검증 로직 1:1 이식, serde Serialize로 `$type` 태그 |
| `errors.py` | `enum ErrorCode`(serde snake_case) + `struct ApiError` | 결과 타입은 `struct XResult { ok, value: Option<_>, error: Option<ApiError>, diagnostics }` 유지(스크립트 측 표현을 위해 Result보다 이 형태) |
| `protocols.py` | `trait SegmentationProvider`, `GraspProvider`, `IkProvider`, `TrajectoryPlanner`, `PosePlanner`, `EnvironmentAdapter` + 선택 훅은 trait 기본 메서드 `fn planning_context(&self) -> Option<_> { None }` | getattr feature-detect → 기본 구현 None |
| `api.py` | `CapApi<A: EnvironmentAdapter>` + `HashMap<String, Box<dyn IkProvider>>` | `_resolve_arm` 등 그대로 |
| `geometry.py` | nalgebra `Isometry3`, `UnitQuaternion`, `SymmetricEigen`(PCA) | 고유벡터 부호 고정 규칙 그대로(결정론) |
| `runtime.py` | `aspire-script` | 8.3 |
| `registry.py` | 도구 이름·layer·capability 정적 테이블 + 문서 시그니처 문자열 | MCP 스키마 생성에도 재사용 |
| `artifacts.py` | `aspire-record` | 레이아웃·키 이름 동일 유지(skill 호환) |
| `run.py`/`batch.py` | `aspire-cli run`, `run-batch`(rayon 또는 프로세스 풀; MuJoCo는 스레드별 MjData로 프로세스 내 병렬 가능, GL 컨텍스트만 스레드당 1개) | spawn 격리 규칙은 seed별 MjData 새로 생성으로 대체 |
| `providers/sam3` | `aspire-perceive::seg` (ort, CUDA 또는 DirectML EP) | 프로세스 내 호출 → HTTP 서비스 불필요(동시성은 Mutex) |
| `providers/pyroki`, `yam_kinematics` | `aspire-kin` | |
| `providers/curobo` | `aspire-plan` | |
| `providers/graspnet` | `aspire-perceive::grasp` 기하식 | |
| `yam_real/*` | `aspire-real-so101` | 3단 제어, station.yaml, allow_motion 게이트 |
| `agent/*` | `aspire-mcp` | rmcp(공식 Rust MCP SDK) streamable HTTP |
| `.claude/skills/*/scripts/*.py` | `aspire-cli campaign ...` | Python 금지 방침에 맞춰 Rust 서브커맨드 |

### 8.3 MuJoCo 바인딩

- Windows용 MuJoCo 공식 배포에 `mujoco.dll` + 헤더 포함 → `bindgen`으로 FFI 직접 생성하거나 `mujoco-rs` 크레이트 사용(버전 3.x 지원·오프스크린 렌더러 지원 범위는 확인 필요).
- 필요한 C API: `mj_loadXML`, `mj_makeData`, `mj_step`, `mj_forward`, `mj_name2id`, `mj_jacSite`, `mj_collision`/`d->contact`, `mjv_updateScene`, `mjr_render`, `mjr_readPixels`(RGB+depth). 오프스크린 GL 컨텍스트는 Windows에 EGL이 없으므로 숨김 GLFW 창(WGL)으로.
- 시뮬은 CPU라 seed 병렬은 CPU 코어 수만큼, GPU는 분할 모델 전용.

### 8.4 분할(SAM3 대체) — ort

| 후보 | text 프롬프트 | 장점 | 위험 |
|---|---|---|---|
| GroundingDINO(또는 OWLv2) ONNX → SAM2/EfficientSAM 디코더 ONNX | 가능 | API `segment_text` 의미 그대로, 공개 ONNX 사례 다수 | 2모델 VRAM, 텍스트 인코더 tokenizer(BERT/CLIP) Rust 구현(`tokenizers` 크레이트) |
| YOLO-World ONNX(어휘 사전 임베딩 고정) + 마스크 | 제한적 | 빠름 | 어휘를 export 시점에 고정 |
| class-agnostic 분할(SAM 자동 마스크) + SigLIP2 crop 임베딩과 텍스트 유사도 | 가능 | 사용자 다른 프로젝트에서 쓰는 조합과 같은 계열 | 마스크 수 많으면 느림 |
| SAM3 ONNX | 가능 | 원본과 동일 동작 | export 가능 여부·라이선스 미확인 |

`segment_points`는 SAM 계열 point prompt 디코더로 그대로 제공 가능. RTX 5070 Ti(sm_120)용 ort CUDA EP는 CUDA 12.8+ 빌드 필요 — 안 되면 DirectML EP.

### 8.5 IK 직접 구현 (SO-101 = 5 arm 관절 + 그리퍼)

- 5자유도라 **임의 6D pose는 일반적으로 도달 불가**. 접근방향(approach 축) + 위치 3 + wrist_roll로 정의되는 5D 목표가 자연스럽다.
- 1차: 해석적 IK — shoulder_pan = atan2(y, x)(수직 평면 결정), shoulder_lift/elbow_flex/wrist_flex = 평면 3링크(목표 pitch 지정 시 2링크 닫힌 해 + wrist 보정), wrist_roll = 목표 yaw의 나머지. 팔꿈치 위/아래 2해 중 seed에 가까운 해.
- 2차: DLS/LM 정제 — MuJoCo `mj_jacSite`, 위치 가중 크게·자세 가중 작게(PyRoki 50/10, `pyroki/service.py:59-64`와 같은 철학), 관절 한계 클램프(PyRoki의 0.15 rad 여유 참고).
- **반드시 FK로 검증 후 ok** (YAM 규칙, `yam_kinematics/client.py:131-176`), 실패는 `IK_UNREACHABLE` + 잔차.
- API 영향: `move_to_pose` 도착 검증의 자세 허용 10°(`api.py:1391`)는 5DoF에서 자주 실패 → `Pose`에 "축 자유(free yaw/roll)" 옵션 또는 `orientation_tolerance` 파라미터를 추가하고, skill 문서에 명시.

### 8.6 경로계획 (cuRobo 대체)

1. MVP: ASPIRE 기본과 동일한 관절 보간(`api.py:653-682`) + 보간 경로 위 충돌 사전검사(통과 못 하면 `PLANNING_FAILED`로 알림만).
2. 2단계: RRT-Connect(관절공간) — 충돌검사는 (a) 시뮬: MuJoCo 모델 복사본에 로봇 qpos를 넣고 `mj_collision`, (b) 관측 점군: voxel(1 cm) occupancy grid vs 링크 capsule/sphere 근사(cuRobo의 sphere 방식). 대상 물체 cloud 제외 규칙은 `api.py:1653-1664` 그대로.
3. shortcut smoothing + 속도/가속 제한 시간 매개화(trapezoid/TOPP 단순판). 첫 호출 워밍업 문제 없음.

### 8.7 에이전트가 쓰는 "프로그램" 언어 선택

| 안 | 방식 | 장점 | 단점 |
|---|---|---|---|
| A. Rust 프로그램 매번 컴파일 | 에이전트가 `program.rs` 작성 → `cargo build`(cdylib/bin) → 실행 | 언어 하나, 빠른 실행 | 증분 빌드도 수 초~수십 초 × (dev seed 15 × 8버전), **샌드박스 없음**(파일·네트워크 자유), borrow checker로 반복 비용↑, ASPIRE skill 스니펫(Python) 전부 재작성 |
| B. 내장 스크립트 — **Starlark(starlark-rust)** | Python 방언, `def/for/if/lambda/comprehension`, import 없음, 결정론적 | **ASPIRE 샌드박스(제한 Python)와 의미가 거의 같음**, skill 스니펫 대부분 그대로 이식, LLM 작성 품질 높음, 즉시 실행·자원 제한 | `while`·`try`·클래스·generator 식 없음(대체: 범위 있는 `for`, ok 결과 검사), 일부 스니펫 수정 필요(`while` 사용은 skill/예시에 소수: localize.md 1, transport.md 1 등) |
| B'. 내장 스크립트 — Rhai | Rust/JS풍 문법 | while/try 지원, Rust 타입 등록 쉬움 | Python 스니펫 전면 변환, LLM 숙련도 낮음 |
| B''. RustPython 내장 | 순수 Rust Python 인터프리터 + ASPIRE와 같은 AST 검증 | skill·예시 거의 무수정 | 크고 느림, 호환성 편차, "Python 금지" 방침과 충돌 소지 |
| C. MCP 도구 호출만 | Claude Code가 `aspire-mcp` 도구를 직접 호출(System 2 방식) | 컴파일·인터프리터 불필요, Claude Code 기본 기능, 실물에 적합(ASPIRE도 실물은 이 방식) | **seed sweep 불가**(같은 정책을 15~50 장면에 재생 못 함 → iterative-debugging의 핵심 측정이 사라짐), 토큰·지연 큼 |

**권장: B(Starlark) + C(MCP) 병행.**
- 시뮬 학습 루프 = Starlark 프로그램(ASPIRE `program.py` 역할). `aspire run --program x.star` / `run-batch --seeds`. 검증 규칙: `_` 접두 이름 금지·도구 allowlist·step 상한·실행 시간 제한·`result` 전역.
- 실물 = MCP 도구 + 검토된 고정 프로그램(go_home/observe) + "프로그램 실행" 도구(시뮬에서 verified된 Starlark만 해시로 허용).
- starlark-rust의 `while` 지원 여부·확장 방식은 미확인 → 착수 첫 단계에서 확인, 없으면 `for _ in range(N)` 관례를 skill에 명시.

### 8.8 최소 기능 단계 (MVP 순서)

| 단계 | 내용 | 완료 조건 |
|---|---|---|
| M0 | `aspire-core`(contracts·errors·geometry) + 단위 테스트(쿼터니언·역투영·OBB를 Python 원본과 수치 비교) | 원본 대비 수치 일치 |
| M1 | `aspire-mujoco` + `aspire-sim-so101`: so101 scene 로드, step, RGB-D 렌더(C270 FOV), `get_robot_state` | 깊이 → 점군이 MuJoCo 실제 물체 위치와 mm 단위 일치 |
| M2 | `aspire-kin`(해석적+DLS, FK 검증) + 보간 `plan_motion` + `move_to_joints/move_to_pose/open/close/go_home` | 테이블 위 격자 목표 도달률 측정 |
| M3 | `aspire-record`(run 레이아웃 동일) + `aspire-script`(Starlark) + `aspire run` | ASPIRE `cube_lifting_seed1.py`를 Starlark로 옮겨 실행·기록 |
| M4 | `aspire-perceive` 분할(ort) → `segment_text`, `localize_object`, overlay | 시뮬 장면에서 "red cube" 분할·위치 오차 측정 |
| M5 | 태스크 레지스트리 3~5개(cube lift, cube→bowl, push) + seed 랜덤화 + `check_success` + `run-batch` | dev/unseen/held-out seed 분할 동작 |
| M6 | skill 이식: iterative-debugging 프롬프트 3종·skills/*.md를 SO-101·Starlark·Windows 명령으로 수정, `aspire campaign` 서브커맨드 | Claude Code로 1개 태스크 Stage0~2 완주 |
| M7 | verifier·curriculum·rerun 전체 루프 | 1회 이상 verified 승격 |
| M8 | `aspire-real-so101`: Pi4 RPC, 레벨1 컨트롤러, C270 캡처·보정, allow_motion 게이트, MuJoCo 대역 plant로 같은 경로 검증 | 실물 go_home/move_to_joints |
| M9 | `aspire-mcp` System 2 + 실물 깊이 해결(9장) → localize→pick | 실물 pick 1회 |
| M10 | RRT-Connect 충돌회피, 기하 grasp 고도화, evosearch | — |

---

## 9. 위험 요소와 미확인 사항

### 9.1 위험

1. **C270은 깊이가 없다.** ASPIRE의 인식·grasp·planning scene 전부가 metric depth를 전제한다(`CameraObservation.depth_m` 필수, `contracts.py:283-333`; `_planning_scene` `api.py:1601-1651`; YAM은 RealSense D405, `yam_real/cameras.py:95-176`). 대안: (a) 테이블 평면 가정 + 보정 extrinsics로 마스크 바닥 픽셀 광선-평면 교차(높이는 사전값/옆 실루엣), (b) 단안 metric depth ONNX(Depth Anything V2 metric 등, cm급 오차·스케일 드리프트), (c) 깊이 카메라 추가(이미 보유한 D435를 쓰는 것이 가장 확실). 시뮬은 깊이 렌더로 문제없으나 **sim(깊이 있음)과 real(없음)의 인식 경로가 갈라지면 skill 전이가 깨진다** — 시뮬에서도 실물과 같은 깊이 경로(평면 가정 또는 단안 추정)를 쓰도록 맞추는 것이 중요.
2. **SO-101은 5자유도 팔.** 계약의 6/7관절 가정(`contracts.py:15-22`, `api.py:142-146`), 자세 10° 도착 검증(`api.py:1391`), skill의 "임의 자세 grasp" 전제가 깨짐. API에 축-자유 목표를 추가해야 한다.
3. **cuRobo·Contact-GraspNet·PyRoki는 Panda/YAM 전용·Linux CUDA 전제**라 그대로 쓸 수 없다. 충돌 인지 계획 품질이 떨어지면 subagent 프롬프트의 "자유공간은 cuRobo" 지침을 SO-101용으로 다시 써야 함.
4. **skill 라이브러리는 LIBERO/Panda에서 얻은 수치**(tick 비용, 높이, 그리퍼 폭 0.08 m)로 가득하다. 이식 시 내용은 비우고 형식·규칙(Trigger/Code/Evidence, 7규칙, lint)만 가져가는 것이 안전.
5. **실물 리셋 불가.** iterative-debugging은 seed로 같은 장면을 재현하는 것이 전제. 실물은 System 2(MCP)·block-by-block(BEHAVIOR식) 프로토콜로, 학습은 시뮬에서 → skill 전이(README의 sim-to-real 사례)가 현실적.
6. **실물 성공 판정 부재**(`yam_real/adapter.py:461-468`). verifier의 ≥4/5 기준을 실물에 적용하려면 사람 판정 또는 VLM/인식 기반 판정기가 필요.
7. **Feetech 위치제어 특성**: 그리퍼 contact-aware close를 위해 STS 계열 present load/current 읽기 필요, 버스 지연·Pi4↔PC 네트워크 지연이 레벨1 재샘플·stale 검사(0.02 rad)와 충돌할 수 있음. YAM 그리퍼 허용치 0.25를 둔 이유(`api.py:65-80`) 참고.
8. **Windows 특이점**: EGL 없음(MuJoCo 오프스크린은 WGL/GLFW), ASPIRE 스크립트 전부 bash·`/bin/` venv 경로, 사용자 경로에 공백(`user one`) — 빌드 도구·DLL 경로 깨짐 위험(공백 없는 경로/정션 권장).
9. **RTX 5070 Ti(sm_120) + ort CUDA EP 호환**(CUDA 12.8+ 빌드 필요), GPU를 다른 학습 작업과 공유하면 분할 모델 VRAM 상주 비용(SAM3 서비스가 empty_cache를 하는 이유, `sam3/service.py:73-91`).
10. **샌드박스**: README 경고대로 생성 코드는 신뢰 불가. Starlark는 언어 차원 격리가 되지만, 실물 모션 권한은 별도 게이트(allow_motion)·관절/속도 한계·비상정지로 막아야 함.
11. **에이전트 운영 비용**: 승인 대기로 수십 시간 정지 사례(`main-agent-prompt.md:202-210`) — Claude Code 권한 allowlist를 캠페인 명령에 맞춰 미리 설정 필요.

### 9.2 미확인

- `third_party/*` 서브모듈이 비어 있어 SAM3 모델 구조·입출력 해상도, cuRobo 설정(`franka.yml`), Contact-GraspNet 체크포인트 상세는 미확인.
- SAM3의 ONNX export 가능성·라이선스.
- `mujoco-rs` 크레이트의 Windows 오프스크린 depth 렌더 지원 범위(아니면 bindgen 직접).
- starlark-rust의 `while`/재귀 허용 옵션, 실행 step 제한 API.
- `so101_new_calib.xml`의 TCP site 존재 여부·actuator 종류(position actuator kp)·관절 한계 — 저장소에 없어 미확인.
- Pi4 기존 Rust 도구의 RPC 면(읽기 주기, 목표 버퍼링, health, 부하/전류 노출 여부).
- C270 고정 초점·자동노출이 분할 안정성에 주는 영향, 보정 방식(ChArUco vs AprilTag).
- ASPIRE 성공률은 Claude Opus·GPT 계열 대형 모델 기준(README "Model Scaling") — 같은 루프의 토큰 비용은 미측정.
