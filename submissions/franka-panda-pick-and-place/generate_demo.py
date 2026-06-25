#!/usr/bin/env python3
"""v16 — 修复：机械臂远离桌子 + 蓝块交接后焊到右手 + 完整三块任务"""
import mujoco, numpy as np, os, time, math
from PIL import Image, ImageDraw, ImageFont

BASE = os.path.dirname(os.path.abspath(__file__))
XML = os.path.join(BASE, "scene_dual_v5.xml")
OUT = os.path.join(BASE, "dual_arm_v16.mp4")
FPS = 30; W, H = 1920, 1080

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
mA_q = qa("module_a_free"); mB_q = qa("module_b_free"); mC_q = qa("module_c_free")

L_Q = [qa(f"joint{i}_L") for i in range(1,8)]
R_Q = [qa(f"joint{i}_R") for i in range(1,8)]
L_D = [da(f"joint{i}_L") for i in range(1,8)]
R_D = [da(f"joint{i}_R") for i in range(1,8)]
L_FQ = [qa("finger_joint1_L"), qa("finger_joint2_L")]
R_FQ = [qa("finger_joint1_R"), qa("finger_joint2_R")]
L_CI = list(range(0,7)); R_CI = list(range(8,15))
L_FI = [7]; R_FI = [15]

# ===== Weld: 支持左手和右手 =====
welds = {}  # module_bid -> hand_bid
def do_weld(mod, hand): welds[mod] = hand
def do_unweld(mod): welds.pop(mod, None)
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
print(f"  Module positions: A={mA}, B={mB}, C={mC}")
print(f"  Left base: {data.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'base_L')]}")
print(f"  Right base: {data.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'base_R')]}")

P = {}
P["home_L"], P["home_R"] = get_ik(np.array([-0.4, 0.4, 0.9]), np.array([0.4, -0.4, 0.9]))
P["scan_L"], P["scan_R"] = get_ik(np.array([-0.15, 0.1, 1.1]), np.array([0.15, -0.1, 1.1]))
P["reach_b_L"], P["idle_R"] = get_ik(np.array([mA[0], mA[1], 0.85]), np.array([0.4, -0.4, 0.9]))
P["grasp_b_L"], P["idle2_R"] = get_ik(np.array([mA[0], mA[1], 0.75]), np.array([0.4, -0.4, 0.9]))
P["lift_b_L"], P["idle3_R"] = get_ik(np.array([mA[0], mA[1], 1.0]), np.array([0.4, -0.4, 0.9]))
P["ho_L"], P["ho_R"] = get_ik(np.array([-0.06, 0.12, 1.0]), np.array([0.06, -0.12, 1.0]))
P["hf_L"], P["hf_R"] = get_ik(np.array([-0.12, 0.08, 0.95]), np.array([0.12, -0.08, 0.95]))
P["hr_L"], P["hr_R"] = get_ik(np.array([-0.05, 0.12, 1.02]), np.array([0.05, -0.12, 1.02]))
# 右臂放下蓝
P["rb_L"], P["rb_R"] = get_ik(np.array([-0.02, 0.12, 1.05]), np.array([0.3, -0.2, 0.8]))
P["rr_L"], P["rr_R"] = get_ik(np.array([-0.02, 0.12, 1.05]), np.array([0.3, -0.2, 0.75]))
P["ro_L"], P["ro_R"] = get_ik(np.array([-0.02, 0.12, 1.05]), np.array([0.3, -0.2, 0.9]))
# 抓绿
P["rg_L"], P["rgi_R"] = get_ik(np.array([mC[0], mC[1], 0.85]), np.array([0.4, -0.4, 0.9]))
P["gg_L"], P["ggi_R"] = get_ik(np.array([mC[0], mC[1], 0.75]), np.array([0.4, -0.4, 0.9]))
P["lg_L"], P["lgi_R"] = get_ik(np.array([mC[0], mC[1], 1.0]), np.array([0.4, -0.4, 0.9]))
# 放绿
P["pg_L"], P["pgi_R"] = get_ik(np.array([-0.25, -0.1, 0.8]), np.array([0.4, -0.4, 0.9]))
P["relg_L"], P["relgi_R"] = get_ik(np.array([-0.25, -0.1, 0.75]), np.array([0.4, -0.4, 0.9]))
P["upg_L"], P["upgi_R"] = get_ik(np.array([-0.25, -0.1, 0.9]), np.array([0.4, -0.4, 0.9]))
# 抓红
P["rr2_L"], P["rr2i_R"] = get_ik(np.array([mB[0], mB[1], 0.85]), np.array([0.4, -0.4, 0.9]))
P["gr2_L"], P["gr2i_R"] = get_ik(np.array([mB[0], mB[1], 0.75]), np.array([0.4, -0.4, 0.9]))
P["lr2_L"], P["lr2i_R"] = get_ik(np.array([mB[0], mB[1], 1.0]), np.array([0.4, -0.4, 0.9]))
# 叠红在绿上
P["stk_L"], P["stki_R"] = get_ik(np.array([-0.25, -0.1, 0.9]), np.array([0.4, -0.4, 0.9]))
P["stk2_L"], P["stk2i_R"] = get_ik(np.array([-0.25, -0.1, 0.78]), np.array([0.4, -0.4, 0.9]))
P["fin_L"], P["fin_R"] = get_ik(np.array([-0.4, 0.4, 0.9]), np.array([0.4, -0.4, 0.9]))

print("All IK poses computed.")

def S(name, ql, qr, fl, fr, dur, weld=None, cam="wide"):
    return (name, P[ql], P[qr], fl, fr, dur, weld, cam)

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
    S("Transfer OK",    "ho_L",      "ho_R",      0.0,  0.0,  1.5, weld=("A","R"), cam="wide"),  # 焊到右手!
    S("Right Place",    "rb_L",      "rb_R",      0.0,  0.0,  1.2, weld=("A","R"), cam="close_r"),
    S("Right Down",     "rr_L",      "rr_R",      0.0,  0.0,  1.0, weld=("A","R"), cam="close_r"),
    S("Right Release",  "ro_L",      "ro_R",      0.04, 0.04, 1.0, cam="close_r"),  # 释放蓝块
    # Phase 2: 绿块
    S("Approach Green", "rg_L",      "rgi_R",     0.04, 0.04, 1.5, cam="close_l"),
    S("Grasp Green",    "gg_L",      "ggi_R",     0.0,  0.04, 1.2, weld=("C","L"), cam="close_l"),
    S("Lift Green",     "lg_L",      "lgi_R",     0.0,  0.04, 1.5, weld=("C","L"), cam="side"),
    S("Place Green",    "pg_L",      "pgi_R",     0.0,  0.04, 1.2, weld=("C","L"), cam="overhead"),
    S("Release Green",  "relg_L",    "relgi_R",   0.04, 0.04, 1.0, cam="overhead"),
    # Phase 3: 红块叠在绿上
    S("Approach Red",   "rr2_L",     "rr2i_R",    0.04, 0.04, 1.5, cam="close_l"),
    S("Grasp Red",      "gr2_L",     "gr2i_R",    0.0,  0.04, 1.2, weld=("B","L"), cam="close_l"),
    S("Lift Red",       "lr2_L",     "lr2i_R",    0.0,  0.04, 1.5, weld=("B","L"), cam="side"),
    S("Stack on Green", "stk_L",     "stki_R",    0.0,  0.04, 1.5, weld=("B","L"), cam="overhead"),
    S("Stack Place",    "stk2_L",    "stk2i_R",   0.0,  0.04, 1.0, weld=("B","L"), cam="overhead"),
    S("Release Red",    "fin_L",     "fin_R",     0.04, 0.04, 1.0, cam="overhead"),
    S("Complete",       "fin_L",     "fin_R",     0.04, 0.04, 1.5, cam="wide"),
]

LABELS = [s[0] for s in STEPS]
NSTEP = len(STEPS); SEC = sum(s[5] for s in STEPS); NFRAMES = int(SEC * FPS)
print(f"Total: {SEC:.1f}s, {NFRAMES} frames, {NSTEP} steps")

try:
    FONT_B = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 28)
    FONT_M = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 20)
    FONT_XS = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 14)
    FONT_XXS = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 12)
except: FONT_B=FONT_M=FONT_XS=FONT_XXS=ImageFont.load_default()

C_BG=(8,12,25,230); C_BD=(50,100,180,160); C_T=(200,220,255); C_D=(120,140,170)
C_A=(50,180,255); C_G=(0,230,120); C_R=(255,80,60); C_O=(255,180,50)

CAM_D = {"wide":(30,-25,5.0,0.1), "close_l":(20,-35,3.2,0.05), "close_r":(40,-35,3.2,0.05),
         "overhead":(0,-80,4.0,0.2), "side":(90,-20,4.5,0.1), "front":(0,-30,4.5,0.1)}

def mk_cam(p, t=0):
    az,el,d,lz = CAM_D[p]
    az+=2*math.sin(t*0.3); el+=1*math.sin(t*0.5+1)
    c = mujoco.MjvCamera(); c.type=mujoco.mjtCamera.mjCAMERA_FREE
    c.lookat[:]=[0,0.1,0.85+lz]; c.distance=d; c.azimuth=az; c.elevation=el
    return c

def lcp(p1,p2,s):
    a1,e1,d1,l1=CAM_D[p1]; a2,e2,d2,l2=CAM_D[p2]
    return (a1+(a2-a1)*s, e1+(e2-e1)*s, d1+(d2-d1)*s, l1+(l2-l1)*s)

def overlay(si, t, fL, fR, gL, gR, fh, err=False):
    img = Image.new("RGBA", (W,H), (0,0,0,0)); draw = ImageDraw.Draw(img)
    fw,fh_h=400,min(NSTEP*20+60,380)
    draw.rounded_rectangle((15,15,15+fw,15+fh_h), radius=10, fill=C_BG, outline=C_BD, width=1)
    draw.text((25,22), "TASK SEQUENCE", fill=C_A, font=FONT_M)
    draw.line((25,48,15+fw-10,48), fill=C_BD, width=1)
    mv=min(NSTEP,16); st=max(0,si-3); en=min(NSTEP,st+mv)
    for i in range(st,en):
        y=58+(i-st)*20
        if y+18>15+fh_h-5: break
        if i==si:
            draw.rounded_rectangle((25,y-2,15+fw-10,y+16), radius=4, fill=(50,100,180,80))
            draw.text((30,y), f"▶ {LABELS[i]}", fill=C_G, font=FONT_XS)
        elif i<si: draw.text((30,y), f"✓ {LABELS[i]}", fill=C_D, font=FONT_XS)
        else: draw.text((30,y), f"  {LABELS[i]}", fill=(70,80,100), font=FONT_XS)
    
    pw,ph=350,120; px,py=15,H-15-ph
    draw.rounded_rectangle((px,py,px+pw,py+ph), radius=10, fill=C_BG, outline=C_BD, width=1)
    draw.text((px+10,py+8), "FORCE", fill=C_A, font=FONT_M)
    for s2,fx,fv,fg in [("L",px+10,fL,gL),("R",px+180,fR,gR)]:
        draw.text((fx,py+35), s2, fill=C_D, font=FONT_XS)
        by=py+52; bw,bh=130,14
        draw.rectangle((fx,by,fx+bw,by+bh), fill=(30,40,60))
        fn=min(fv/20,1.0)
        if fn>0:
            c=C_G if fn<0.7 else C_O if fn<0.9 else C_R
            draw.rectangle((fx,by,fx+int(bw*fn),by+bh), fill=c)
        draw.text((fx+bw+5,by), f"{fv:.1f}N", fill=C_T, font=FONT_XS)
        draw.text((fx,py+70), "GRIP" if fg else "OPEN", fill=C_G if fg else C_O, font=FONT_XS)
    pid=fL>0.5 or fR>0.5
    draw.text((px+10,py+90), "PID:", fill=C_D, font=FONT_XS)
    draw.text((px+45,py+90), "ON" if pid else "OFF", fill=C_G if pid else C_O, font=FONT_XS)
    draw.text((px+100,py+90), "Ctrl:", fill=C_D, font=FONT_XS)
    draw.text((px+145,py+90), "AUTO", fill=C_G, font=FONT_XS)
    
    sn=LABELS[si]
    if err: sn+=" ⚠"
    tw=500; tx=(W-tw)//2
    draw.rounded_rectangle((tx,H-55,tx+tw,H-12), radius=10, fill=C_BG, outline=C_R if err else C_BD, width=2)
    draw.text((tx+20,H-46), sn, fill=C_R if err else (255,255,255), font=FONT_B)
    
    pw2,ph2=280,45; px2,py2=W-pw2-15,H-15-ph2
    draw.rounded_rectangle((px2,py2,px2+pw2,py2+ph2), radius=10, fill=C_BG, outline=C_BD, width=1)
    p=t/SEC
    draw.rectangle((px2+15,py2+12,px2+pw2-15,py2+25), fill=(30,40,60))
    draw.rectangle((px2+15,py2+12,px2+15+int((pw2-30)*p),py2+25), fill=C_A)
    draw.text((px2+pw2//2-20,py2+28), f"{p*100:.0f}%", fill=C_T, font=FONT_XS)
    draw.text((px2+15,py2+1), f"{t:.1f}s / {SEC:.1f}s", fill=C_D, font=FONT_XXS)
    
    sw,sh=250,80; sx,sy=W-sw-15,15
    draw.rounded_rectangle((sx,sy,sx+sw,sy+sh), radius=10, fill=C_BG, outline=C_BD, width=1)
    draw.text((sx+10,sy+5), "STATUS", fill=C_A, font=FONT_XS)
    draw.text((sx+10,sy+22), "MuJoCo", fill=C_D, font=FONT_XXS)
    draw.text((sx+90,sy+22), "ON", fill=C_G, font=FONT_XXS)
    draw.text((sx+10,sy+38), "Ctrl", fill=C_D, font=FONT_XXS)
    draw.text((sx+90,sy+38), "AUTO", fill=C_G, font=FONT_XXS)
    draw.text((sx+10,sy+54), "Arm", fill=C_D, font=FONT_XXS)
    draw.text((sx+90,sy+54), "SYNC", fill=C_G, font=FONT_XXS)
    draw.text((sx+10,sy+68), "Phys", fill=C_D, font=FONT_XXS)
    draw.text((sx+90,sy+68), "1kHz", fill=C_A, font=FONT_XXS)
    
    return img

# ===== 真闭环力控：读取touch sensor =====
def read_touch_forces(data, model):
    """读取4个touch sensor的真实接触力"""
    forces = {}
    for i in range(model.nsensor):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, i)
        adr = model.sensor_adr[i]
        forces[name] = abs(data.sensordata[adr])
    # 左臂合力 = 两个finger的力之和
    fL = forces.get("touch_L_f1", 0) + forces.get("touch_L_f2", 0)
    # 右臂合力
    fR = forces.get("touch_R_f1", 0) + forces.get("touch_R_f2", 0)
    return fL, fR

class FC:
    def __init__(s,t=12): s.t=t;s.i=0;s.pe=0
    def update(s,cf,dt=1/30):
        e=s.t-cf;s.i=np.clip(s.i+e*dt,-5,5);d=(e-s.pe)/max(dt,1e-6);s.pe=e
        return np.clip(2.5*e+0.8*s.i+0.15*d,0,255)
    def reset(s): s.i=0;s.pe=0

fcL=FC(); fcR=FC()

print("\nRendering v16...")
mujoco.mj_resetData(model, data); mujoco.mj_forward(model, data)
frames=[]; fh=[]; prev_cp="wide"
cL=P["home_L"][:]; cR=P["home_R"][:]; cfL=0.04; cfR=0.04
t0=time.time(); fc=0

for si,(name,tL,tR,fl,fr,dur,weld_info,cp) in enumerate(STEPS):
    nf=int(dur*FPS); sL,sR,sfL,sfR=cL[:],cR[:],cfL,cfR
    if weld_info:
        mod_key, hand = weld_info
        mod_bid = {"A":mA_b, "B":mB_b, "C":mC_b}[mod_key]
        hand_bid = hL if hand=="L" else hR
        do_weld(mod_bid, hand_bid)
    elif not weld_info and si > 0:
        # 检查上一步是否有weld，这步没有 → 释放
        pass
    if fl<0.02: fcL.reset()
    if fr<0.02: fcR.reset()
    is_err=("Fail" in name or "Re-align" in name)
    
    for f in range(nf):
        gt=fc/FPS; s=0.5-0.5*np.cos(np.pi*f/max(nf-1,1))
        for i in range(7):
            data.qpos[L_Q[i]]=sL[i]+(tL[i]-sL[i])*s
            data.qpos[R_Q[i]]=sR[i]+(tR[i]-sR[i])*s
            data.ctrl[L_CI[i]]=data.qpos[L_Q[i]]
            data.ctrl[R_CI[i]]=data.qpos[R_Q[i]]
        vL=sfL+(fl-sfL)*s; vR=sfR+(fr-sfR)*s
        for i in range(2):
            data.qpos[L_FQ[i]]=vL; data.qpos[R_FQ[i]]=vR
        data.ctrl[L_FI[0]]=vL; data.ctrl[R_FI[0]]=vR
        mujoco.mj_forward(model,data); apply_welds(); mujoco.mj_forward(model,data)
        # 关键：让mj_step推进物理，使重力对自由体生效
        mujoco.mj_step(model,data)
        # 重新设置机器人关节位置（覆盖mj_step的偏移）
        for i in range(7):
            data.qpos[L_Q[i]]=sL[i]+(tL[i]-sL[i])*s
            data.qpos[R_Q[i]]=sR[i]+(tR[i]-sR[i])*s
        mujoco.mj_forward(model,data)
        
        # 真闭环力控：读取touch sensor真实接触力
        fLv, fRv = read_touch_forces(data, model)
        
        # PID闭环：根据真力调节夹爪
        if fl < 0.02:  # 左臂应该在抓
            if fLv > 0.5:
                # 有力接触 → PID调节
                output = fcL.update(fLv)
                # 输出到夹爪ctrl（0-255映射到0-0.04）
                vL = output / 255.0 * 0.04
            else:
                fcL.reset()
                vL = sfL + (fl - sfL) * s  # 无接触，用原始位置
        if fr < 0.02:  # 右臂应该在抓
            if fRv > 0.5:
                output = fcR.update(fRv)
                vR = output / 255.0 * 0.04
            else:
                fcR.reset()
                vR = sfR + (fr - sfR) * s
        
        # 真力已经在上面计算了（fLv, fRv来自touch sensor）
        fh.append(fLv+fRv)
        if len(fh)>300: fh=fh[-300:]
        
        cs=0.5-0.5*np.cos(np.pi*s)
        az,el,d,lz=lcp(prev_cp,cp,cs)
        cam=mujoco.MjvCamera(); cam.type=mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:]=[0,0.1,0.85+lz]; cam.distance=d
        cam.azimuth=az+2*math.sin(gt*0.3); cam.elevation=el+1*math.sin(gt*0.5+1)
        rend.update_scene(data,camera=cam); frame=rend.render().copy()
        ov=overlay(si,gt,fLv,fRv,vL<0.01,vR<0.01,fh,err=is_err)
        bf=Image.fromarray(frame).convert("RGBA")
        bf=Image.alpha_composite(bf,ov)
        frames.append(np.array(bf.convert("RGB"))); fc+=1
    
    prev_cp=cp; cL,cR,cfL,cfR=tL[:],tR[:],fl,fr
    print(f"  {si+1}/{NSTEP} {name} ({time.time()-t0:.1f}s)")

print(f"\nTotal frames: {len(frames)} (target: {NFRAMES})")
import imageio
imageio.mimsave(OUT,frames[:NFRAMES],fps=FPS,quality=8)
tt=time.time()-t0
print(f"✅ {OUT} — {W}x{H}, {SEC:.1f}s, {tt:.0f}s, {os.path.getsize(OUT)/1024/1024:.1f}MB")
