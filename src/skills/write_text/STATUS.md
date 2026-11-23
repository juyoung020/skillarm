# write_text — status (2026-11-24, paused)

Pick up a pencil and write text on paper with the SO-101. C++ only, standalone build:

    cmake -S src/skills/write_text -B build/write_text -DCMAKE_BUILD_TYPE=Release && cmake --build build/write_text -j
    build/write_text/skillarm-write font "Hello 안녕하세요" --size 25 --out font.png
    build/write_text/skillarm-write plan '{"text":"Hi 안녕","size_mm":25,"paper":{...},"pencil":{...}}'
    build/write_text/skillarm-write demo "Hi 안녕"          # plan + full twin run (runs/write_text/<plan_id>/)

Works:
- Stroke font (`font.cpp`): Latin A-Z a-z, digits, punctuation; Hangul syllables composed from jamo templates
  (initial / medial / final boxes by vowel shape, double consonants and final clusters), layout in mm on the paper.
- Scene (`scene.cpp`, mjSpec): SO-101 + desk (z = -0.015) + A4 paper on a compliant underlay + 7 mm hexagonal pencil
  + C270 wrist-mount hull.
- Planner (`planner.cpp`): top-down grasp near the back end, writing-pose search (tilt 0-15 deg or 30-60 deg, 5-DoF
  tip + axis IK), regrasp fallback (stage the pencil), 30 Hz trajectory with contact-gated descents, per-stroke
  probes, place-back and home; collision sweep (desk, self, pencil-arm). "Hi 안녕" plans feasible in ~0.35-0.6 s.
- Executor (`executor.cpp`) over `ArmIO`; twin (`twin.cpp`) emulates the servo server (45 steps / 20 ms, gripper
  lead 60, 45 % gripper torque, load readings with noise); real IO (`real_io.cpp`) disabled unless enabled + confirmed.
- C ABI (`write_text.h`, libskillarm_write.so) and HTTP server (`skillarm-write-server`, :7757) — not yet wired to PAM.

Open (next steps):
1. Physics grasp fails in the twin: the contact gate on the grasp descent fires on motion load (the arm's loads
   saturate while moving), so the jaws close ~7 mm above the pencil. Fix: gate only below the nominal height minus a
   margin / use a slow constant-speed final descent with its own baseline, then re-run `demo`.
2. Then: full sim write metrics (shape error, force, slip, time), trace image, frames; precheck timing
   (now ~150 ms sweep + ~280 ms physics: subsample the sweep, shorten the physics phases).
3. Real-world readiness checklist and PAM `write_text` tool hookup (on hold by user decision).
