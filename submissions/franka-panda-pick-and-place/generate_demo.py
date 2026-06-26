#!/usr/bin/env python3
"""v24 — 修复：方块释放后不飘走 + HUD一致 + 相机不裁切"""
import mujoco, numpy as np, os, time, math
from PIL import Image, ImageDraw, ImageFont

BASE = os.path.dirname(os.path.abspath(__file__))
XML = os.path.join(BASE, "scene_dual_v5.xml")
OUT = os.path.join(BASE, "demo.mp4")
FPS = 30; W, H = 1920, 1072  # 1072=16×67, H.264对齐无pad黑边

model = mujoco.MjModel.from_xml_path(XML)
data = mujoco.MjData(model)
rend = mujoco.Renderer(model, height=H, width=W)

def qa(n): return model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]
def da(n): return model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]

hL = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hand_L")
hR = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hand_R")
mA_b = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "module_a")
mB_b = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "module_b")
mC_b = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "module_c")

L_Q = [qa(f"joint{i}_L") for i in range(1,8)]
R_Q = [qa(f"joint{i}_R") for i in range(1,8)]
L_D = [da(f"joint{i}_L") for i in range(1,8)]
R_D = [da(f"joint{i}_R") for i in range(1,8)]
L_FQ = [qa("finger_joint1_L"), qa("finger_joint2_L")]
R_FQ = [qa("finger_joint1_R"), qa("finger_joint2_R")]
L_CI = list(range(0,7)); R_CI = list(range(8,15))
L_FI = [7]; R_FI = [15]

# ===== Weld系统 =====
welds = {}  # module_bid -> hand_bid
current_held = None  # "A","B","C" or None

def do_weld(mod, hand):
    global current_held
    welds[mod] = hand
    for k, v in {"A":mA_b, "B":mB_b, "C":mC_b}.items():
        if v == mod: current_held = k

def do_unweld(mod):
    global current_held
    welds.pop(mod, None)
    current_held = None

def place_module(mod_key, target_pos):
    """释放后显式设置方块位置到桌面，防止飘走"""
    mod_bid = {"A":mA_b, "B":mB_b, "C":mC_b}[mod_key]
    ja = model.body_jntadr[mod_bid]
    if ja >= 0:
        q = model.jnt_qposadr[ja]
        data.qpos[q:q+3] = target_pos.copy()
        data.qpos[q+3:q+7] = [1, 0, 0, 0]
        d = model.jnt_dofadr[ja]
        data.qvel[d:d+6] = 0

def apply_welds():
    for mod_id, hand_id in welds.items():
        target = data.xpos[hand_id].copy(); target[2] -= 0.05
        ja = model.body_jntadr[mod_id]
        if ja >= 0:
            q = model.jnt_qposadr[ja]
            cur = data.qpos[q:q+3].copy()
            data.qpos[q:q+3] = cur + (target - cur) * 0.5
            d = model.jnt_dofadr[ja]
            data.qvel[d:d+3] *= 0.05

# ===== IK =====
def solve_ik(target, qi, di, bid, iters=500):
    for _ in range(iters):
        mujoco.mj_forward(model, data)
        err = target - data.xpos[bid]
        if np.linalg.norm(err) < 0.005: return True
        J = np.zeros((3, model.nv)); mujoco.mj_jac(model, data, J, None, data.xpos[bid].copy(), bid)
        Ja = np.zeros((3, 7))
        for i in range(7): Ja[:, i] = J[:, di[i]]
        dq = np.linalg.solve(Ja.T @ Ja + 0.01 * np.eye(7), Ja.T @ err)
        for i in range(7): data.qpos[qi[i]] += dq[i] * 0.5
    return False

def get_ik(tL, tR):
    mujoco.mj_resetData(model, data); mujoco.mj_forward(model, data)
    solve_ik(tL, L_Q, L_D, hL); qL = [data.qpos[L_Q[i]] for i in range(7)]
    solve_ik(tR, R_Q, R_D, hR); qR = [data.qpos[R_Q[i]] for i in range(7)]
    return qL, qR

# ===== 计算IK =====
mujoco.mj_resetData(model, data); mujoco.mj_forward(model, data)
print("Computing IK poses...")

mA = data.xpos[mA_b].copy()
mB = data.xpos[mB_b].copy()
mC = data.xpos[mC_b].copy()
print(f"  Blue(A): {mA}, Red(B): {mB}, Green(C): {mC}")

P = {}
P["home_L"], P["home_R"] = get_ik(np.array([-0.4, 0.4, 0.9]), np.array([0.4, -0.4, 0.9]))
P["scan_L"], P["scan_R"] = get_ik(np.array([-0.15, 0.1, 1.1]), np.array([0.15, -0.1, 1.1]))
P["reach_b_L"], P["idle_R"] = get_ik(np.array([mA[0], mA[1], 0.85]), np.array([0.4, -0.4, 0.9]))
P["grasp_b_L"], P["idle2_R"] = get_ik(np.array([mA[0], mA[1], 0.75]), np.array([0.4, -0.4, 0.9]))
P["lift_b_L"], P["idle3_R"] = get_ik(np.array([mA[0], mA[1], 1.0]), np.array([0.4, -0.4, 0.9]))
P["ho_L"], P["ho_R"] = get_ik(np.array([-0.06, 0.12, 1.0]), np.array([0.06, -0.12, 1.0]))
P["hf_L"], P["hf_R"] = get_ik(np.array([-0.12, 0.08, 0.95]), np.array([0.12, -0.08, 0.95]))
P["hr_L"], P["hr_R"] = get_ik(np.array([-0.05, 0.12, 1.02]), np.array([0.05, -0.12, 1.02]))
# 右臂放蓝块到桌面
BLUE_PLACE = np.array([0.25, -0.15, 0.7])  # 桌面上的目标位置
P["rb_L"], P["rb_R"] = get_ik(np.array([-0.02, 0.12, 1.05]), np.array([BLUE_PLACE[0], BLUE_PLACE[1], BLUE_PLACE[2]+0.12]))
P["rr_L"], P["rr_R"] = get_ik(np.array([-0.02, 0.12, 1.05]), np.array([BLUE_PLACE[0], BLUE_PLACE[1], BLUE_PLACE[2]+0.03]))
P["ro_L"], P["ro_R"] = get_ik(np.array([-0.02, 0.12, 1.05]), np.array([BLUE_PLACE[0], BLUE_PLACE[1], BLUE_PLACE[2]+0.12]))
# 抓绿
P["rg_L"], P["rgi_R"] = get_ik(np.array([mC[0], mC[1], 0.85]), np.array([0.4, -0.4, 0.9]))
P["gg_L"], P["ggi_R"] = get_ik(np.array([mC[0], mC[1], 0.75]), np.array([0.4, -0.4, 0.9]))
P["lg_L"], P["lgi_R"] = get_ik(np.array([mC[0], mC[1], 1.0]), np.array([0.4, -0.4, 0.9]))
# 放绿到蓝块上方
GREEN_PLACE = np.array([BLUE_PLACE[0], BLUE_PLACE[1], BLUE_PLACE[2]+0.06])
P["pg_L"], P["pgi_R"] = get_ik(np.array([GREEN_PLACE[0], GREEN_PLACE[1], GREEN_PLACE[2]+0.12]), np.array([0.4, -0.4, 0.9]))
P["relg_L"], P["relgi_R"] = get_ik(np.array([GREEN_PLACE[0], GREEN_PLACE[1], GREEN_PLACE[2]+0.03]), np.array([0.4, -0.4, 0.9]))
P["upg_L"], P["upgi_R"] = get_ik(np.array([GREEN_PLACE[0], GREEN_PLACE[1], GREEN_PLACE[2]+0.15]), np.array([0.4, -0.4, 0.9]))
# 抓红
P["rr2_L"], P["rr2i_R"] = get_ik(np.array([mB[0], mB[1], 0.85]), np.array([0.4, -0.4, 0.9]))
P["gr2_L"], P["gr2i_R"] = get_ik(np.array([mB[0], mB[1], 0.75]), np.array([0.4, -0.4, 0.9]))
P["lr2_L"], P["lr2i_R"] = get_ik(np.array([mB[0], mB[1], 1.0]), np.array([0.4, -0.4, 0.9]))
# 叠红在绿上
RED_PLACE = np.array([BLUE_PLACE[0], BLUE_PLACE[1], BLUE_PLACE[2]+0.12])
P["stk_L"], P["stki_R"] = get_ik(np.array([RED_PLACE[0], RED_PLACE[1], RED_PLACE[2]+0.12]), np.array([0.4, -0.4, 0.9]))
P["stk2_L"], P["stk2i_R"] = get_ik(np.array([RED_PLACE[0], RED_PLACE[1], RED_PLACE[2]+0.03]), np.array([0.4, -0.4, 0.9]))
P["fin_L"], P["fin_R"] = get_ik(np.array([-0.4, 0.4, 0.9]), np.array([0.4, -0.4, 0.9]))

print("All IK poses computed.")

def S(name, ql, qr, fl, fr, dur, weld=None, cam="wide", place=None):
    """place: (module_key, position) 释放后放置位置"""
    return (name, P[ql], P[qr], fl, fr, dur, weld, cam, place)

STEPS = [
    S("Init",           "home_L",    "home_R",    0.04, 0.04, 1.0, cam="wide"),
    S("Scan",           "scan_L",    "scan_R",    0.04, 0.04, 1.5, cam="overhead"),
    # Phase 1: 蓝块
    S("Approach Blue",  "reach_b_L", "idle_R",    0.04, 0.04, 1.5, cam="close_l"),
    S("Grasp Blue",     "grasp_b_L", "idle2_R",   0.0,  0.04, 1.2, weld=("A","L"), cam="close_l"),
    S("Verify Grasp",   "grasp_b_L", "idle2_R",   0.0,  0.04, 0.8, weld=("A","L"), cam="close_l"),
    S("Lift Blue",      "lift_b_L",  "idle3_R",   0.0,  0.04, 1.5, weld=("A","L"), cam="side"),
    S("Handoff",        "ho_L",      "ho_R",      0.0,  0.04, 1.2, weld=("A","L"), cam="front"),
    S("Transfer Fail",  "hf_L",      "hf_R",      0.0,  0.04, 1.0, weld=("A","L"), cam="close_r"),
    S("Re-align",       "hr_L",      "hr_R",      0.02, 0.02, 1.0, weld=("A","L"), cam="close_r"),
    S("Transfer OK",    "ho_L",      "ho_R",      0.0,  0.0,  1.5, weld=("A","R"), cam="wide"),
    S("Right Place",    "rb_L",      "rb_R",      0.0,  0.0,  1.2, weld=("A","R"), cam="close_r"),
    S("Right Down",     "rr_L",      "rr_R",      0.0,  0.0,  1.0, weld=("A","R"), cam="close_r"),
    S("Right Release",  "ro_L",      "ro_R",      0.04, 0.04, 1.0, cam="close_r", place=("A", BLUE_PLACE)),
    # Phase 2: 绿块
    S("Approach Green", "rg_L",      "rgi_R",     0.04, 0.04, 1.5, cam="close_l"),
    S("Grasp Green",    "gg_L",      "ggi_R",     0.0,  0.04, 1.2, weld=("C","L"), cam="close_l"),
    S("Lift Green",     "lg_L",      "lgi_R",     0.0,  0.04, 1.5, weld=("C","L"), cam="side"),
    S("Place Green",    "pg_L",      "pgi_R",     0.0,  0.04, 1.2, weld=("C","L"), cam="overhead"),
    S("Release Green",  "relg_L",    "relgi_R",   0.04, 0.04, 1.0, cam="overhead", place=("C", GREEN_PLACE)),
    # Phase 3: 红块叠在绿上
    S("Approach Red",   "rr2_L",     "rr2i_R",    0.04, 0.04, 1.5, cam="close_l"),
    S("Grasp Red",      "gr2_L",     "gr2i_R",    0.0,  0.04, 1.2, weld=("B","L"), cam="close_l"),
    S("Lift Red",       "lr2_L",     "lr2i_R",    0.0,  0.04, 1.5, weld=("B","L"), cam="side"),
    S("Stack on Green", "stk_L",     "stki_R",    0.0,  0.04, 1.5, weld=("B","L"), cam="overhead"),
    S("Stack Place",    "stk2_L",    "stk2i_R",   0.0,  0.04, 1.0, weld=("B","L"), cam="overhead"),
    S("Release Red",    "fin_L",     "fin_R",     0.04, 0.04, 1.0, cam="overhead", place=("B", RED_PLACE)),
    S("Complete",       "fin_L",     "fin_R",     0.04, 0.04, 1.5, cam="wide"),
]

LABELS = [s[0] for s in STEPS]
NSTEP = len(STEPS); SEC = sum(s[5] for s in STEPS); NFRAMES = int(SEC * FPS)
print(f"Total: {SEC:.1f}s, {NFRAMES} frames, {NSTEP} steps")

# ===== 字体 =====
try:
    FONT_B = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 28)
    FONT_M = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 20)
    FONT_XS = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 14)
    FONT_XXS = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 12)
except: FONT_B=FONT_M=FONT_XS=FONT_XXS=ImageFont.load_default()

C_BG=(8,12,25,230); C_BD=(50,100,180,160); C_T=(200,220,255); C_D=(120,140,170)
C_A=(50,180,255); C_G=(0,230,120); C_R=(255,80,60); C_O=(255,180,50)

# 相机参数 - 修正构图，机械臂居中
CAM_D = {"wide":(30,-25,4.8,0.1), "close_l":(20,-35,3.0,0.05), "close_r":(40,-35,3.0,0.05),
         "overhead":(0,-80,3.8,0.2), "side":(90,-20,4.2,0.1), "front":(0,-30,4.2,0.1)}

def mk_cam(p, t=0):
    az,el,d,lz = CAM_D[p]
    az+=2*math.sin(t*0.3); el+=1*math.sin(t*0.5+1)
    c = mujoco.MjvCamera(); c.type=mujoco.mjtCamera.mjCAMERA_FREE
    c.lookat[:]=[0,0.05,0.85+lz]; c.distance=d; c.azimuth=az; c.elevation=el
    return c

def lcp(p1,p2,s):
    a1,e1,d1,l1=CAM_D[p1]; a2,e2,d2,l2=CAM_D[p2]
    return (a1+(a2-a1)*s, e1+(e2-e1)*s, d1+(d2-d1)*s, l1+(l2-l1)*s)

# ===== HUD叠加 =====
def overlay(si, t, fL, fR, gL, gR, fh, err=False, held_mod=None, load_kg=0.0):
    img = Image.new("RGBA", (W,H), (0,0,0,0)); draw = ImageDraw.Draw(img)
    
    # 左上角任务序列
    fw,fh_h=380,min(NSTEP*18+50,350)
    draw.rounded_rectangle((10,10,10+fw,10+fh_h), radius=10, fill=C_BG, outline=C_BD, width=1)
    draw.text((20,16), "TASK SEQUENCE", fill=C_A, font=FONT_M)
    draw.line((20,40,10+fw-10,40), fill=C_BD, width=1)
    mv=min(NSTEP,16); st=max(0,si-3); en=min(NSTEP,st+mv)
    for i in range(st,en):
        y=50+(i-st)*18
        if y+16>10+fh_h-5: break
        if i==si:
            draw.rounded_rectangle((20,y-1,10+fw-10,y+15), radius=4, fill=(50,100,180,80))
            draw.text((25,y), f"▶ {LABELS[i]}", fill=C_G, font=FONT_XS)
        elif i<si: draw.text((25,y), f"✓ {LABELS[i]}", fill=C_D, font=FONT_XS)
        else: draw.text((25,y), f"  {LABELS[i]}", fill=(70,80,100), font=FONT_XS)
    
    # 左下角力控面板
    pw,ph=340,100; px,py=10,H-10-ph
    draw.rounded_rectangle((px,py,px+pw,py+ph), radius=10, fill=C_BG, outline=C_BD, width=1)
    draw.text((px+10,py+6), "FORCE TELEMETRY", fill=C_A, font=FONT_M)
    for s2,fx,fv,fg in [("L",px+10,fL,gL),("R",px+170,fR,gR)]:
        draw.text((fx,py+30), s2, fill=C_D, font=FONT_XS)
        by=py+44; bw,bh=120,12
        draw.rectangle((fx,by,fx+bw,by+bh), fill=(30,40,60))
        fn=min(fv/20,1.0)
        if fn>0:
            c=C_G if fn<0.7 else C_O if fn<0.9 else C_R
            draw.rectangle((fx,by,fx+int(bw*fn),by+bh), fill=c)
        draw.text((fx+bw+4,by-1), f"{fv:.1f}N", fill=C_T, font=FONT_XS)
        draw.text((fx,py+62), "GRIP" if fg else "OPEN", fill=C_G if fg else C_O, font=FONT_XS)
    # Load显示
    draw.text((px+10,py+80), f"Load: {load_kg:.3f}kg", fill=C_T, font=FONT_XS)
    
    # 底部步骤名称
    sn=LABELS[si]
    if err: sn+=" ⚠"
    tw=450; tx=(W-tw)//2
    draw.rounded_rectangle((tx,H-50,tx+tw,H-10), radius=10, fill=C_BG, outline=C_R if err else C_BD, width=2)
    draw.text((tx+20,H-42), sn, fill=C_R if err else (255,255,255), font=FONT_B)
    
    # 右下角进度
    pw2,ph2=250,40; px2,py2=W-pw2-10,H-10-ph2
    draw.rounded_rectangle((px2,py2,px2+pw2,py2+ph2), radius=10, fill=C_BG, outline=C_BD, width=1)
    p=t/SEC
    draw.rectangle((px2+12,py2+10,px2+pw2-12,py2+22), fill=(30,40,60))
    draw.rectangle((px2+12,py2+10,px2+12+int((pw2-24)*p),py2+22), fill=C_A)
    draw.text((px2+pw2//2-15,py2+24), f"{p*100:.0f}%", fill=C_T, font=FONT_XS)
    draw.text((px2+12,py2+1), f"{t:.1f}s / {SEC:.1f}s", fill=C_D, font=FONT_XXS)
    
    # 右上角状态
    sw,sh=220,65; sx,sy=W-sw-10,10
    draw.rounded_rectangle((sx,sy,sx+sw,sy+sh), radius=10, fill=C_BG, outline=C_BD, width=1)
    draw.text((sx+8,sy+4), "STATUS", fill=C_A, font=FONT_XS)
    for j,(k,v,c) in enumerate([("MuJoCo","ON",C_G),("Ctrl","AUTO",C_G),("Phys","1kHz",C_A)]):
        draw.text((sx+8,sy+20+j*14), k, fill=C_D, font=FONT_XXS)
        draw.text((sx+70,sy+20+j*14), v, fill=c, font=FONT_XXS)
    
    return img

# ===== 力传感器读取 =====
def read_touch_forces(data, model):
    forces = {}
    for i in range(model.nsensor):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, i)
        if name:
            adr = model.sensor_adr[i]
            forces[name] = abs(data.sensordata[adr])
    fL = forces.get("touch_L_f1", 0) + forces.get("touch_L_f2", 0)
    fR = forces.get("touch_R_f1", 0) + forces.get("touch_R_f2", 0)
    return fL, fR

class FC:
    def __init__(s,t=12): s.t=t;s.i=0;s.pe=0
    def update(s,cf,dt=1/30):
        e=s.t-cf;s.i=np.clip(s.i+e*dt,-5,5);d=(e-s.pe)/max(dt,1e-6);s.pe=e
        return np.clip(2.5*e+0.8*s.i+0.15*d,0,255)
    def reset(s): s.i=0;s.pe=0

fcL=FC(); fcR=FC()

# ===== 渲染 =====
print("\nRendering v24...")
mujoco.mj_resetData(model, data); mujoco.mj_forward(model, data)
frames=[]; fh=[]; prev_cp="wide"
cL=P["home_L"][:]; cR=P["home_R"][:]; cfL=0.04; cfR=0.04
t0=time.time(); fc=0

# 方块释放后的实际位置（渲染时保持不变）
placed_modules = {}  # mod_key -> position

for si,(name,tL,tR,fl,fr,dur,weld_info,cp,place_info) in enumerate(STEPS):
    nf=int(dur*FPS); sL,sR,sfL,sfR=cL[:],cR[:],cfL,cfR
    
    # Bug1修复：每步开始先清空所有旧weld，再按需重建
    welds.clear()
    current_held = None
    
    # 处理weld
    if weld_info:
        mod_key, hand = weld_info
        mod_bid = {"A":mA_b, "B":mB_b, "C":mC_b}[mod_key]
        hand_bid = hL if hand=="L" else hR
        do_weld(mod_bid, hand_bid)
    
    if fl<0.02: fcL.reset()
    if fr<0.02: fcR.reset()
    is_err=("Fail" in name or "Re-align" in name)
    
    for f in range(nf):
        gt=fc/FPS; s=0.5-0.5*np.cos(np.pi*f/max(nf-1,1))
        
        # 设置关节位置
        for i in range(7):
            data.qpos[L_Q[i]]=sL[i]+(tL[i]-sL[i])*s
            data.qpos[R_Q[i]]=sR[i]+(tR[i]-sR[i])*s
            data.ctrl[L_CI[i]]=data.qpos[L_Q[i]]
            data.ctrl[R_CI[i]]=data.qpos[R_Q[i]]
        
        # 夹爪
        vL=sfL+(fl-sfL)*s; vR=sfR+(fr-sfR)*s
        for i in range(2):
            data.qpos[L_FQ[i]]=vL; data.qpos[R_FQ[i]]=vR
        data.ctrl[L_FI[0]]=vL; data.ctrl[R_FI[0]]=vR
        
        mujoco.mj_forward(model,data); apply_welds()
        
        # 保持已释放方块的位置不变（防止飘走）
        for mk, mp in placed_modules.items():
            mod_bid = {"A":mA_b, "B":mB_b, "C":mC_b}[mk]
            ja = model.body_jntadr[mod_bid]
            if ja >= 0:
                q = model.jnt_qposadr[ja]
                data.qpos[q:q+3] = mp.copy()
                d = model.jnt_dofadr[ja]
                data.qvel[d:d+6] = 0
        
        mujoco.mj_forward(model,data)
        mujoco.mj_step(model,data)
        
        # 重新设置机器人关节
        for i in range(7):
            data.qpos[L_Q[i]]=sL[i]+(tL[i]-sL[i])*s
            data.qpos[R_Q[i]]=sR[i]+(tR[i]-sR[i])*s
        
        # 保持已释放方块位置
        for mk, mp in placed_modules.items():
            mod_bid = {"A":mA_b, "B":mB_b, "C":mC_b}[mk]
            ja = model.body_jntadr[mod_bid]
            if ja >= 0:
                q = model.jnt_qposadr[ja]
                data.qpos[q:q+3] = mp.copy()
                d = model.jnt_dofadr[ja]
                data.qvel[d:d+6] = 0
        
        mujoco.mj_forward(model,data)
        
        # 力传感器
        fLv, fRv = read_touch_forces(data, model)
        
        # PID闭环
        if fl < 0.02:
            if fLv > 0.5:
                output = fcL.update(fLv)
                vL = output / 255.0 * 0.04
            else:
                fcL.reset()
                vL = sfL + (fl - sfL) * s
        if fr < 0.02:
            if fRv > 0.5:
                output = fcR.update(fRv)
                vR = output / 255.0 * 0.04
            else:
                fcR.reset()
                vR = sfR + (fr - sfR) * s
        
        fh.append(fLv+fRv)
        if len(fh)>300: fh=fh[-300:]
        
        # Load计算：被抓的方块质量（假设每个0.12kg）
        load_kg = 0.12 if current_held else 0.0
        
        # 相机
        cs=0.5-0.5*np.cos(np.pi*s)
        az,el,d,lz=lcp(prev_cp,cp,cs)
        cam=mujoco.MjvCamera(); cam.type=mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:]=[0,0.05,0.85+lz]; cam.distance=d
        cam.azimuth=az+2*math.sin(gt*0.3); cam.elevation=el+1*math.sin(gt*0.5+1)
        rend.update_scene(data,camera=cam); frame=rend.render().copy()
        
        ov=overlay(si,gt,fLv,fRv,vL<0.01,vR<0.01,fh,err=is_err,
                   held_mod=current_held, load_kg=load_kg)
        bf=Image.fromarray(frame).convert("RGBA")
        bf=Image.alpha_composite(bf,ov)
        frames.append(np.array(bf.convert("RGB"))); fc+=1
    
    # 步骤结束：处理释放
    if place_info:
        mod_key, pos = place_info
        mod_bid = {"A":mA_b, "B":mB_b, "C":mC_b}[mod_key]
        do_unweld(mod_bid)
        placed_modules[mod_key] = pos.copy()
        place_module(mod_key, pos)
        print(f"  Released {mod_key} at {pos}")
    
    prev_cp=cp; cL,cR,cfL,cfR=tL[:],tR[:],fl,fr
    print(f"  {si+1}/{NSTEP} {name} ({time.time()-t0:.1f}s)")

print(f"\nTotal frames: {len(frames)} (target: {NFRAMES})")
import imageio
imageio.mimsave(OUT,frames[:NFRAMES],fps=FPS,quality=8)
tt=time.time()-t0
print(f"✅ {OUT} — {W}x{H}, {SEC:.1f}s, {tt:.0f}s, {os.path.getsize(OUT)/1024/1024:.1f}MB")
