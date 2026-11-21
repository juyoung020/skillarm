"""실제 팔에 보낼 자세 이동(한 관절씩 순서대로)을 공식 모델 + 손목 고정대로 미리 검사.
사용: path_check.py "<지금 위치 6개>" "[(관절번호, 목표위치), ...]"  (위치는 서보 값, 2047 = 0°)
책상 면(z=0)보다 낮아지는 순간이 있으면 '위험' 과 처음 닿는 지점을 출력. 종료 코드 0 = 안전, 1 = 위험."""
import sys, ast, struct
import mujoco, numpy as np

m = mujoco.MjModel.from_xml_path(r"C:\tmp\so101sim\scene.xml")
d = mujoco.MjData(m)
NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
adr = [m.joint(n).qposadr[0] for n in NAMES]
rad = lambda p: (p - 2047) * 2 * np.pi / 4096

def stl(p):
    b = open(p, "rb").read(); n = struct.unpack("<I", b[80:84])[0]
    a = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return a["v"].reshape(-1, 3)[::7] / 1000.0  # mm → m, 점 일부만
mount = stl(r"C:\Users\user one\Desktop\so101-rl\viewer\assets\c270_mount_p.stl")
grip = m.body("gripper").id
base = m.body("base").id
arm_geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] not in (0, base) and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH]

def lowest():
    low, where = 1e9, ""
    for g in arm_geoms:
        mid = m.geom_dataid[g]
        v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
        z = (v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g])[:, 2].min()
        if z < low: low, where = z, m.body(m.geom_bodyid[g]).name
    z = (mount @ d.xmat[grip].reshape(3, 3).T + d.xpos[grip])[:, 2].min()
    if z < low: low, where = z, "손목 고정대"
    return low, where

start = [int(x) for x in sys.argv[1].split()]
steps = ast.literal_eval(sys.argv[2])
q = [rad(p) for p in start]
q[5] = 0.0
worst = (1e9, "", "")
for jid, target in steps:
    k = jid - 1
    a, b = q[k], rad(target)
    for t in np.linspace(0, 1, 30):
        q[k] = a + (b - a) * t
        for i in range(6): d.qpos[adr[i]] = q[i]
        mujoco.mj_forward(m, d)
        z, w = lowest()
        if z < worst[0]: worst = (z, w, f"{NAMES[k]} {np.degrees(q[k]):+.0f}°")
        if z < 0.010:
            print(f"위험: {NAMES[k]} 를 {np.degrees(q[k]):+.0f}° 로 옮기는 중 {w} 가 책상 면 {z*1000:+.0f} mm")
            sys.exit(1)
print(f"안전: 가장 낮은 점 {worst[0]*1000:+.0f} mm ({worst[1]}, {worst[2]})")
