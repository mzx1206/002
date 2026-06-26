import streamlit as st
import folium
from streamlit_folium import folium_static, st_folium
from folium import plugins
from folium.plugins import Draw
import random
import time
import math
import json
import os
from datetime import datetime
import pandas as pd
import threading
from streamlit_autorefresh import st_autorefresh

# ==================== 配置常量 ====================
SCHOOL_CENTER = [118.7490, 32.2340]
A_DFT = [118.746956, 32.232945]
B_DFT = [118.751589, 32.235204]
SAT_URL = "https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}"
VEC_URL = "https://webrd02.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}"
ATTR = "高德地图"
CONFIG_FILE = "obstacle_config.json"
BASE_SPEED_MPS = 5.0
HEARTBEAT_INTERVAL = 3

# ==================== 坐标转换 ====================
def gcj2wgs(lng, lat):
    if abs(lng) < 72 or abs(lng) > 138 or abs(lat) < 0.8 or abs(lat) > 56: return lng, lat
    dlat = -100 + 2*lng + 3*lat + 0.2*lat*lat + 0.1*lng*lat + 0.2*math.sqrt(abs(lng))
    dlat += (20*math.sin(6*lng*math.pi)+20*math.sin(2*lng*math.pi))*2/3
    dlat += (20*math.sin(lat*math.pi)+40*math.sin(lat/3*math.pi))*2/3
    dlat += (160*math.sin(lat/12*math.pi)+320*math.sin(lat*math.pi/30))*2/3
    dlng = 300 + lng + 2*lat + 0.1*lng*lng + 0.1*lng*lat + 0.1*math.sqrt(abs(lng))
    dlng += (20*math.sin(6*lng*math.pi)+20*math.sin(2*lng*math.pi))*2/3
    dlng += (20*math.sin(lng*math.pi)+40*math.sin(lng/3*math.pi))*2/3
    dlng += (150*math.sin(lng/12*math.pi)+300*math.sin(lng/30*math.pi))*2/3
    rad = lat/180*math.pi
    magic = 1 - 0.00669342162296594323 * math.sin(rad)**2
    sqrtmagic = math.sqrt(magic)
    dlat = dlat * 180 / ((6378245.0*(1-0.00669342162296594323))/(magic*sqrtmagic)*math.pi)
    dlng = dlng * 180 / (6378245.0/sqrtmagic*math.cos(rad)*math.pi)
    return lng-dlng, lat-dlat

def wgs2gcj(lng, lat):
    if abs(lng) < 72 or abs(lng) > 138 or abs(lat) < 0.8 or abs(lat) > 56: return lng, lat
    dlat = -100 + 2*lng + 3*lat + 0.2*lat*lat + 0.1*lng*lat + 0.2*math.sqrt(abs(lng))
    dlat += (20*math.sin(6*lng*math.pi)+20*math.sin(2*lng*math.pi))*2/3
    dlat += (20*math.sin(lat*math.pi)+40*math.sin(lat/3*math.pi))*2/3
    dlat += (160*math.sin(lat/12*math.pi)+320*math.sin(lat*math.pi/30))*2/3
    dlng = 300 + lng + 2*lat + 0.1*lng*lng + 0.1*lng*lat + 0.1*math.sqrt(abs(lng))
    dlng += (20*math.sin(6*lng*math.pi)+20*math.sin(2*lng*math.pi))*2/3
    dlng += (20*math.sin(lng*math.pi)+40*math.sin(lng/3*math.pi))*2/3
    dlng += (150*math.sin(lng/12*math.pi)+300*math.sin(lng/30*math.pi))*2/3
    rad = lat/180*math.pi
    magic = 1 - 0.00669342162296594323 * math.sin(rad)**2
    sqrtmagic = math.sqrt(magic)
    dlat = dlat * 180 / ((6378245.0*(1-0.00669342162296594323))/(magic*sqrtmagic)*math.pi)
    dlng = dlng * 180 / (6378245.0/sqrtmagic*math.cos(rad)*math.pi)
    return lng+dlng, lat+dlat

# ==================== 几何辅助 ====================
def dist(p1, p2):
    return math.hypot(p1[0]-p2[0], p1[1]-p2[1])

def point_in_poly(p, poly):
    x,y = p; inside=False
    for i in range(len(poly)):
        x1,y1 = poly[i]; x2,y2 = poly[(i+1)%len(poly)]
        if ((y1>y)!=(y2>y)) and (x<(x2-x1)*(y-y1)/(y2-y1)+x1): inside=not inside
    return inside

def lines_intersect(a,b,c,d):
    def ccw(A,B,C): return (C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0])
    return ccw(a,c,d)!=ccw(b,c,d) and ccw(a,b,c)!=ccw(a,b,d)

def line_cross_poly(p1,p2,poly):
    if point_in_poly(p1,poly) or point_in_poly(p2,poly): return True
    for i in range(len(poly)):
        if lines_intersect(p1,p2,poly[i],poly[(i+1)%len(poly)]): return True
    return False

def seg_to_poly_dist(p1, p2, poly):
    min_d = float('inf')
    for pt in poly:
        t = ((pt[0]-p1[0])*(p2[0]-p1[0]) + (pt[1]-p1[1])*(p2[1]-p1[1])) / (dist(p1,p2)**2+1e-9)
        t = max(0,min(1,t))
        proj = (p1[0]+t*(p2[0]-p1[0]), p1[1]+t*(p2[1]-p1[1]))
        d = dist(pt,proj)
        if d < min_d: min_d = d
    for i in range(len(poly)):
        p3,p4 = poly[i], poly[(i+1)%len(poly)]
        for t in range(11):
            pt = (p3[0]+(p4[0]-p3[0])*t/10, p3[1]+(p4[1]-p3[1])*t/10)
            d = dist(pt, (p1[0],p1[1]))
            if d < min_d: min_d = d
    return min_d * 111000

def should_avoid(obs, h):
    return h <= obs.get('height',20)

def path_safe(p1,p2,obs,rad_m,h):
    for o in obs:
        if not should_avoid(o,h): continue
        poly = o.get('polygon',[])
        if len(poly)<3: continue
        if line_cross_poly(p1,p2,poly): return False
        if seg_to_poly_dist(p1,p2,poly) < rad_m-0.1: return False
    return True

# ==================== 绕行生成 ====================
def gen_bypass(A,B,obs,rad_m,h,side='left'):
    avoid = [o for o in obs if should_avoid(o,h)]
    if not avoid: return [A,B]
    mx,my = (A[0]+B[0])/2, (A[1]+B[1])/2
    dx,dy = B[0]-A[0], B[1]-A[1]
    L = math.hypot(dx,dy)
    if L==0: return [A,B]
    ux,uy = dx/L, dy/L
    px,py = -uy, ux
    if side=='right': px,py = uy,-ux
    deg_m = 1/111000
    for attempt in range(1,31):
        off_m = rad_m*2*attempt
        off_deg = off_m*deg_m
        wp = (mx+px*off_deg, my+py*off_deg)
        if path_safe(A,wp,avoid,rad_m,h) and path_safe(wp,B,avoid,rad_m,h):
            return [A,wp,B]
    pts = [p for o in avoid for p in o.get('polygon',[])]
    if pts:
        cx = sum(p[0] for p in pts)/len(pts); cy = sum(p[1] for p in pts)/len(pts)
        far = max(pts, key=lambda p: dist((cx,cy),p))
        dx,dy = far[0]-cx, far[1]-cy
        L2 = math.hypot(dx,dy)
        if L2>0: dx,dy = dx/L2, dy/L2
        else: dx,dy = 1,0
        wp = (far[0]+dx*rad_m*15*deg_m, far[1]+dy*rad_m*15*deg_m)
        return [A,wp,B]
    return [A,B]

def plan_single_segment(A,B,obs,h,rad,strat):
    avoid = [o for o in obs if should_avoid(o,h)]
    straight = not any(line_cross_poly(A,B,o['polygon']) for o in avoid)
    if straight: return [A,B]
    if strat in ('left','right'):
        return gen_bypass(A,B,obs,rad,h,strat)
    else:
        left=gen_bypass(A,B,obs,rad,h,'left')
        right=gen_bypass(A,B,obs,rad,h,'right')
        if left and right:
            len_left = sum(dist(left[i],left[i+1]) for i in range(len(left)-1))
            len_right = sum(dist(right[i],right[i+1]) for i in range(len(right)-1))
            return left if len_left <= len_right else right
        return left or right or [A,B]

def plan_full_path(waypoints, obs, h, rad, strat):
    full = []
    for i in range(len(waypoints)-1):
        seg = plan_single_segment(waypoints[i], waypoints[i+1], obs, h, rad, strat)
        if i == 0:
            full.extend(seg)
        else:
            full.extend(seg[1:])
    return full

# ==================== 辅助函数 ====================
def point_to_seg_meters(p, a, b):
    ap = (p[0]-a[0], p[1]-a[1])
    ab = (b[0]-a[0], b[1]-a[1])
    t = (ap[0]*ab[0] + ap[1]*ab[1]) / (ab[0]*ab[0] + ab[1]*ab[1] + 1e-9)
    t = max(0, min(1, t))
    proj = (a[0] + t*ab[0], a[1] + t*ab[1])
    return math.hypot(p[0]-proj[0], p[1]-proj[1]) * 111000

def check_safety_radius(drone_pos, obstacles, flight_alt, safe_radius):
    if not drone_pos:
        return True, None, None
    min_dist = float('inf')
    danger_name = None
    for obs in obstacles:
        if obs.get('height',20) > flight_alt:
            poly = obs.get('polygon',[])
            if poly:
                for i in range(len(poly)):
                    p1 = poly[i]; p2 = poly[(i+1)%len(poly)]
                    d = point_to_seg_meters(drone_pos, p1, p2)
                    if d < min_dist:
                        min_dist = d
                        danger_name = obs.get('name','障碍物')
    if min_dist < safe_radius:
        return False, min_dist, danger_name
    return True, min_dist if min_dist!=float('inf') else None, None

# ==================== 通信日志管理 ====================
def add_comm_log(direction: str, message: str, details: dict = None):
    """添加通信日志条目
    direction: "GCS→OBC", "OBC→FCU", "FCU→OBC", "OBC→GCS", "OBC内部"
    message: 简短描述
    details: 可选附加信息字典
    """
    if 'comm_logs' not in st.session_state:
        st.session_state.comm_logs = []
    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    log_entry = {
        "timestamp": timestamp,
        "direction": direction,
        "message": message,
        "details": details or {}
    }
    st.session_state.comm_logs.insert(0, log_entry)
    if len(st.session_state.comm_logs) > 200:
        st.session_state.comm_logs = st.session_state.comm_logs[:200]

# ==================== 心跳模拟器（增加通信日志） ====================
class HeartbeatSim:
    def __init__(self, start):
        self.hist = []
        self.pos = list(start)
        self.path = [list(start)]
        self.idx = 0
        self.sim = False
        self.is_paused = False
        self.alt = 50
        self.spd = 50
        self.prog = 0
        self.total = 0
        self.trav = 0
        self.start_time = None
        self.elapsed = 0
        self.safety_violation = False
        self.last_update_time = None
        self.last_reported_wp = -1

    def set_path(self, path, alt, spd):
        self.path = [list(p) for p in path]
        self.idx = 0
        self.pos = list(path[0])
        self.alt = alt
        self.spd = spd
        self.sim = True
        self.is_paused = False
        self.prog = 0
        self.trav = 0
        self.total = sum(dist(self.path[i], self.path[i+1]) for i in range(len(self.path)-1))
        self.start_time = time.time()
        self.last_update_time = time.time()
        self.elapsed = 0
        self.safety_violation = False
        self.hist = []
        self.last_reported_wp = -1

    def reset(self):
        if self.path:
            self.pos = list(self.path[0])
            self.idx = 0
            self.sim = False
            self.is_paused = False
            self.prog = 0
            self.trav = 0
            self.start_time = None
            self.last_update_time = None
            self.elapsed = 0
            self.safety_violation = False
            self.hist = []
            self.last_reported_wp = -1

    def do_pause(self):
        self.is_paused = True

    def do_resume(self):
        self.is_paused = False

    def stop(self):
        self.sim = False
        self.is_paused = False
        self.start_time = None
        self.last_update_time = None

    def update(self, obstacles_gcj, safe_radius, dt):
        if not self.sim or self.is_paused:
            return self._hb(obstacles_gcj, safe_radius)

        # 类型安全
        if not isinstance(self.pos, list):
            self.pos = list(self.pos)
        if len(self.pos) != 2:
            self.pos = [0.0, 0.0]
        for i in range(2):
            try:
                self.pos[i] = float(self.pos[i])
            except (TypeError, ValueError):
                self.pos[i] = 0.0

        if self.start_time:
            self.elapsed += dt

        if self.idx < len(self.path) - 1:
            tar = self.path[self.idx + 1]
            if not isinstance(tar, (list, tuple)) or len(tar) < 2:
                tar = [0.0, 0.0]
            dx = float(tar[0]) - self.pos[0]
            dy = float(tar[1]) - self.pos[1]
            distance_to_target = math.hypot(dx, dy)

            speed_mps = 0.5 + (self.spd / 100) * 4.5
            step_m = speed_mps * dt
            step_deg = step_m / 111000.0

            old_idx = self.idx

            if distance_to_target <= step_deg:
                self.trav += distance_to_target
                self.pos = list(tar)
                self.idx += 1
            else:
                if distance_to_target > 1e-12:
                    ratio = step_deg / distance_to_target
                    self.pos[0] += dx * ratio
                    self.pos[1] += dy * ratio
                    self.trav += step_deg
                else:
                    self.trav += distance_to_target
                    self.pos = list(tar)
                    self.idx += 1

            # 航点到达日志
            if self.idx > old_idx and self.idx <= len(self.path) - 1:
                wp_number = self.idx
                if wp_number != self.last_reported_wp:
                    self.last_reported_wp = wp_number
                    add_comm_log("FCU→OBC→GCS", f"WP_REACHED #{wp_number}",
                                 {"wp_index": wp_number, "position": self.pos})

            if self.total > 0:
                self.prog = min(1.0, self.trav / self.total)

            if self.idx >= len(self.path) - 1 and not self.sim:
                add_comm_log("FCU→OBC→GCS", "MISSION_COMPLETE",
                             {"final_position": self.pos, "total_distance": self.total})
        else:
            self.sim = False
            self.prog = 1.0

        hb_data = self._hb(obstacles_gcj, safe_radius)
        self.hist.insert(0, hb_data)
        if len(self.hist) > 1000:
            self.hist = self.hist[:1000]

        return hb_data

    def _hb(self, obstacles_gcj, safe_radius):
        if self.sim and not self.is_paused:
            speed = round(0.5 + (self.spd / 100) * 4.5, 1)
        else:
            speed = 0

        if self.sim and not self.is_paused and self.idx < len(self.path) - 1:
            remaining_in_path = 0.0
            remaining_in_path += dist(self.pos, self.path[self.idx + 1])
            for i in range(self.idx + 1, len(self.path) - 1):
                remaining_in_path += dist(self.path[i], self.path[i + 1])
            remaining_dist = remaining_in_path * 111000
        else:
            remaining_dist = max(0, self.total - self.trav) * 111000

        safe, min_d, danger = check_safety_radius(self.pos, obstacles_gcj, self.alt, safe_radius)
        self.safety_violation = not safe

        battery = max(0, 100 - int(self.prog * 100))

        if speed > 0 and remaining_dist > 0:
            eta_sec = remaining_dist / speed
            if eta_sec < 60:
                remain_str = f"{eta_sec:.0f}秒"
            else:
                minutes = int(eta_sec // 60)
                seconds = int(eta_sec % 60)
                remain_str = f"{minutes:02d}:{seconds:02d}"
        else:
            remain_str = "00:00"

        voltage = 22.2 + random.uniform(-0.5, 0.5)
        satellites = random.randint(8, 14)
        delay = round(random.uniform(10, 50), 1) if self.sim else 0
        loss = round(random.uniform(0, 0.2), 1) if self.sim else 0
        arrived = not self.sim and self.prog >= 1.0

        return {
            "timestamp": datetime.now().strftime("%H:%M:%S"),
            "lng": self.pos[0],
            "lat": self.pos[1],
            "altitude": self.alt + random.randint(-5, 5) if self.sim else random.randint(0, 10),
            "speed": speed,
            "progress": self.prog,
            "total": self.total,
            "traveled": self.trav,
            "current_wp": f"{self.idx + 1}/{len(self.path)}",
            "remain": remain_str,
            "battery": battery,
            "elapsed": self.elapsed,
            "delay_ms": delay,
            "loss_percent": loss,
            "simulating": self.sim,
            "paused": self.is_paused,
            "flight_time": self.elapsed,
            "voltage": voltage,
            "satellites": satellites,
            "arrived": arrived,
            "safety_violation": self.safety_violation,
            "remaining_distance": remaining_dist
        }

# ==================== 障碍物缓存 ====================
def save_cache():
    if 'saved' not in st.session_state: st.session_state.saved = []
    import copy
    st.session_state.saved = copy.deepcopy(st.session_state.obs)
    st.success(f"保存 {len(st.session_state.obs)} 个障碍物")

def load_cache():
    if 'saved' in st.session_state and st.session_state.saved:
        st.session_state.obs = st.session_state.saved
        st.success(f"加载 {len(st.session_state.obs)} 个障碍物")
        return True
    st.warning("无缓存")
    return False

# ==================== 安全半径可视化 ====================
def add_safety(m, obs, rad, h):
    for o in obs:
        if should_avoid(o,h):
            for pt in o.get('polygon',[]):
                folium.Circle([pt[1],pt[0]], rad, color='orange', fill=True, fill_opacity=0.2, popup=f"安全区{rad}m").add_to(m)

# ==================== 地图生成 ====================
def make_map(center, waypoints, obs, hist, full_path, maptype, rad, h, drone_pos=None):
    tiles = SAT_URL if maptype=='satellite' else VEC_URL
    m = folium.Map(location=[center[1],center[0]], zoom_start=16, tiles=tiles, attr=ATTR)
    Draw(export=True, draw_options={'polygon':{'allowIntersection':False,'showArea':True}}).add_to(m)
    add_safety(m, obs, rad, h)

    for i,o in enumerate(obs):
        coords=o.get('polygon',[])
        if len(coords)>=3:
            color = 'red' if o.get('height',20) > h else 'orange'
            folium.Polygon([[c[1],c[0]] for c in coords], color=color, weight=3, fill=True, fill_opacity=0.4,
                          popup=f"{o.get('name',f'障碍物{i+1}')}\n高度:{o.get('height',20)}m").add_to(m)

    for idx, wp in enumerate(waypoints):
        color = 'green' if idx==0 else ('red' if idx==len(waypoints)-1 else 'blue')
        folium.Marker([wp[1],wp[0]], popup=f"航点{idx+1}", icon=folium.Icon(color=color)).add_to(m)

    if full_path and len(full_path)>1:
        folium.PolyLine([[p[1],p[0]] for p in full_path], color='green', weight=5, opacity=0.9, popup="完整避障航线").add_to(m)
        for p in full_path[1:-1]: folium.CircleMarker([p[1],p[0]], 3, color='green', fill=True).add_to(m)

    if len(waypoints) > 1:
        straight_line = [[wp[1], wp[0]] for wp in waypoints]
        folium.PolyLine(straight_line, color='gray', weight=2, dash_array='5,5', popup="航点连线").add_to(m)

    if hist:
        trail = [[p[1],p[0]] for p in hist[-30:] if len(p)==2]
        if len(trail)>1: folium.PolyLine(trail, color='orange', weight=2).add_to(m)

    if drone_pos:
        folium.Circle([drone_pos[1],drone_pos[0]], rad, color='blue', fill=True, fill_opacity=0.2, popup="安全区").add_to(m)
        folium.Marker([drone_pos[1],drone_pos[0]], popup="无人机", icon=folium.Icon(color='red', icon='plane', prefix='fa')).add_to(m)

    return m

# ==================== 通信页面组件（修复 KeyError 问题） ====================
def show_communication_page():
    st.header("📡 通信链路监控与日志")

    # 拓扑图 (使用HTML/CSS模拟)
    st.markdown("""
    <style>
    .topology {
        display: flex;
        justify-content: space-around;
        align-items: center;
        background: #f0f2f6;
        padding: 20px;
        border-radius: 10px;
        margin: 10px 0;
    }
    .node {
        text-align: center;
        background: white;
        padding: 10px;
        border-radius: 8px;
        box-shadow: 0 2px 5px rgba(0,0,0,0.1);
        min-width: 150px;
    }
    .node h4 { margin: 0; color: #1f77b4; }
    .node p { margin: 5px 0; font-size: 12px; }
    .arrow { font-size: 24px; color: #2ca02c; }
    .status { color: green; font-weight: bold; }
    </style>
    """, unsafe_allow_html=True)

    col1, col2, col3 = st.columns([1, 0.2, 1])
    with col1:
        st.markdown("""
        <div class="node">
            <h4>🖥️ GCS 地面站</h4>
            <p>192.168.1.100</p>
            <p>UDP:14550 <span class="status">● 已连接</span></p>
        </div>
        """, unsafe_allow_html=True)
    with col2:
        st.markdown("<div class='arrow'>➡️</div>", unsafe_allow_html=True)
    with col3:
        st.markdown("""
        <div class="node">
            <h4>💻 OBC 机载计算机</h4>
            <p>Raspberry Pi 4</p>
            <p>MAVLink <span class="status">● 已连接</span></p>
        </div>
        """, unsafe_allow_html=True)

    st.markdown("<div style='text-align:center; margin:10px 0;'>⬇️</div>", unsafe_allow_html=True)

    col4, col5, col6 = st.columns([1, 0.2, 1])
    with col4:
        st.markdown("""
        <div class="node">
            <h4>⚙️ FCU 飞控</h4>
            <p>PX4 / ArduPilot</p>
            <p>MAVLink <span class="status">● 已连接</span></p>
        </div>
        """, unsafe_allow_html=True)
    with col5:
        st.markdown("<div class='arrow'>⬆️⬇️</div>", unsafe_allow_html=True)
    with col6:
        st.markdown("""
        <div class="node">
            <h4>🔁 双向通信</h4>
            <p>GCS ↔ OBC ↔ FCU</p>
        </div>
        """, unsafe_allow_html=True)

    st.markdown("---")

    # 链路统计
    st.subheader("📊 链路统计")
    if st.session_state.hb and st.session_state.hb.hist:
        last_hb = st.session_state.hb.hist[0]
        delay = last_hb.get('delay_ms', 25)
        loss = last_hb.get('loss_percent', 0.1)
    else:
        delay = 25
        loss = 0.1
    col_a, col_b, col_c, col_d = st.columns(4)
    col_a.metric("GCS → OBC", "正常", delta="")
    col_b.metric("OBC → FCU", "正常", delta="")
    col_c.metric("延迟", f"~{delay} ms", delta="")
    col_d.metric("丢包率", f"{loss}%", delta="")

    st.markdown("---")

    # 通信日志
    st.subheader("📜 通信日志")
    if 'comm_logs' not in st.session_state:
        st.session_state.comm_logs = []

    if st.session_state.comm_logs:
        log_data = []
        for log in st.session_state.comm_logs:
            # 安全获取字段，避免 KeyError
            timestamp = log.get('timestamp', '')
            direction = log.get('direction', '')
            message = log.get('message', '')
            details = log.get('details', {})
            details_str = ""
            if details:
                details_str = " | ".join([f"{k}: {v}" for k, v in details.items()])
            log_data.append({
                "时间": timestamp,
                "方向": direction,
                "消息": message,
                "详情": details_str
            })
        df = pd.DataFrame(log_data)
        st.dataframe(df, use_container_width=True)
    else:
        st.info("暂无通信日志，请开始飞行任务。")

# ==================== 主程序 ====================
def main():
    st.set_page_config(layout="wide")
    st.title("🏫 无人机地面站系统 - 航点飞行（最终版）")

    # 初始化状态
    if 'waypoints' not in st.session_state: st.session_state.waypoints = [A_DFT[:], B_DFT[:]]
    if 'obs' not in st.session_state: st.session_state.obs = []
    if 'hb' not in st.session_state: st.session_state.hb = HeartbeatSim(st.session_state.waypoints[0][:])
    if 'last_time' not in st.session_state: st.session_state.last_time = time.time()
    if 'running' not in st.session_state: st.session_state.running = False
    if 'alt' not in st.session_state: st.session_state.alt = 50
    if 'hist' not in st.session_state: st.session_state.hist = []
    if 'full_path' not in st.session_state: st.session_state.full_path = None
    if 'pending_poly' not in st.session_state: st.session_state.pending_poly = None
    if 'pending_h' not in st.session_state: st.session_state.pending_h = 20
    if 'drone_spd' not in st.session_state: st.session_state.drone_spd = 50
    if 'safe_rad' not in st.session_state: st.session_state.safe_rad = 5
    if 'sel_strat' not in st.session_state: st.session_state.sel_strat = 'best'
    if 'new_wp_lng' not in st.session_state: st.session_state.new_wp_lng = A_DFT[0]
    if 'new_wp_lat' not in st.session_state: st.session_state.new_wp_lat = A_DFT[1]
    if 'update_counter' not in st.session_state: st.session_state.update_counter = 0
    if 'comm_logs' not in st.session_state: st.session_state.comm_logs = []

    with st.sidebar:
        st.header("控制面板")
        page = st.radio("模块", ["规划", "监控", "障碍物", "通信"])
        map_type = "satellite" if st.radio("地图", ["卫星影像", "矢量街道"]) == "卫星影像" else "vector"
        st.markdown("---")
        st.subheader("无人机参数")
        st.session_state.drone_spd = st.slider("速度系数", 10, 100, st.session_state.drone_spd)
        st.session_state.safe_rad = st.number_input("安全半径(米)", 1, 30, st.session_state.safe_rad)
        st.session_state.alt = st.number_input("飞行高度(米)", 0, 200, st.session_state.alt)
        st.markdown("---")
        st.subheader("绕行策略")
        strat = st.radio("避障方式", ["最佳航线", "向左绕行", "向右绕行"])
        strat_map = {"最佳航线": "best", "向左绕行": "left", "向右绕行": "right"}
        st.session_state.sel_strat = strat_map[strat]
        st.info(f"障碍物: {len(st.session_state.obs)}")

        if st.button("刷新规划", use_container_width=True):
            with st.spinner("规划全航线中..."):
                full_path = plan_full_path(st.session_state.waypoints,
                                           st.session_state.obs,
                                           st.session_state.alt,
                                           st.session_state.safe_rad,
                                           st.session_state.sel_strat)
                st.session_state.full_path = full_path
                # 添加通信日志：航线规划完成
                add_comm_log("OBC内部", "航线规划完成",
                             {"类型": "horizontal",
                              "航点数": len(full_path),
                              "路径长度(m)": round(sum(dist(full_path[i], full_path[i+1]) for i in range(len(full_path)-1)) * 111000, 1)})

    if page == "规划":
        st.header("航线规划 - 多航点避障")
        st.info("📝 点击地图📐画多边形→设置高度→「添加障碍物」；下方可添加/删除航点（起点和终点固定）")

        col1, col2 = st.columns([1, 1.5])

        with col1:
            st.markdown("#### 🗺️ 航点管理")

            # 起点
            st.markdown("**起点**")
            col_s = st.columns(2)
            with col_s[0]:
                a_lat = st.number_input("纬度", value=st.session_state.waypoints[0][1], format="%.6f", key="a_lat")
            with col_s[1]:
                a_lng = st.number_input("经度", value=st.session_state.waypoints[0][0], format="%.6f", key="a_lng")
            if st.button("更新起点"):
                st.session_state.waypoints[0] = [a_lng, a_lat]

            # 中间航点
            st.markdown("**中间航点**")
            if len(st.session_state.waypoints) > 2:
                for i in range(1, len(st.session_state.waypoints)-1):
                    col_wp = st.columns([3, 1])
                    col_wp[0].write(f"航点{i}: ({st.session_state.waypoints[i][0]:.6f}, {st.session_state.waypoints[i][1]:.6f})")
                    if col_wp[1].button("删除", key=f"del_wp_{i}"):
                        st.session_state.waypoints.pop(i)
            else:
                st.write("暂无中间航点")

            # 添加新航点
            st.markdown("**添加新航点**")
            col_add = st.columns(2)
            with col_add[0]:
                new_lng = st.number_input("经度", value=st.session_state.new_wp_lng, format="%.6f", key="new_lng")
            with col_add[1]:
                new_lat = st.number_input("纬度", value=st.session_state.new_wp_lat, format="%.6f", key="new_lat")
            if st.button("➕ 添加航点"):
                st.session_state.waypoints.insert(-1, [new_lng, new_lat])

            # 终点
            st.markdown("**终点**")
            col_e = st.columns(2)
            with col_e[0]:
                b_lat = st.number_input("纬度", value=st.session_state.waypoints[-1][1], format="%.6f", key="b_lat")
            with col_e[1]:
                b_lng = st.number_input("经度", value=st.session_state.waypoints[-1][0], format="%.6f", key="b_lng")
            if st.button("更新终点"):
                st.session_state.waypoints[-1] = [b_lng, b_lat]

            st.markdown("---")

            # 障碍物添加
            st.markdown("#### 🏗️ 新障碍物高度")
            st.session_state.pending_h = st.number_input("高度(米)", 1, 200, st.session_state.pending_h)
            if st.button("➕ 添加障碍物"):
                if st.session_state.pending_poly and len(st.session_state.pending_poly) >= 3:
                    st.session_state.obs.append({"name": f"建筑物{len(st.session_state.obs)+1}",
                                                 "polygon": st.session_state.pending_poly,
                                                 "height": st.session_state.pending_h})
                    st.success(f"已添加，共{len(st.session_state.obs)}个")
                    st.session_state.pending_poly = None
                    st.session_state.full_path = plan_full_path(st.session_state.waypoints,
                                                                  st.session_state.obs,
                                                                  st.session_state.alt,
                                                                  st.session_state.safe_rad,
                                                                  st.session_state.sel_strat)
                else:
                    st.warning("请先在地图上画多边形")

            if st.button("🔄 重新规划路径"):
                with st.spinner("规划全航线中..."):
                    full_path = plan_full_path(st.session_state.waypoints,
                                               st.session_state.obs,
                                               st.session_state.alt,
                                               st.session_state.safe_rad,
                                               st.session_state.sel_strat)
                    st.session_state.full_path = full_path
                    add_comm_log("OBC内部", "航线规划完成",
                                 {"类型": "horizontal",
                                  "航点数": len(full_path),
                                  "路径长度(m)": round(sum(dist(full_path[i], full_path[i+1]) for i in range(len(full_path)-1)) * 111000, 1)})

            st.markdown("#### ✈️ 飞行控制")
            if st.button("▶️ 开始飞行"):
                if st.session_state.full_path is None or len(st.session_state.full_path) < 2:
                    st.warning("请先点击「刷新规划」生成完整路径")
                else:
                    st.session_state.hb.set_path(st.session_state.full_path, st.session_state.alt, st.session_state.drone_spd)
                    st.session_state.running = True
                    st.session_state.hist = []
                    st.session_state.last_time = time.time()
                    # 添加导航目标日志
                    start_wp = st.session_state.waypoints[0]
                    end_wp = st.session_state.waypoints[-1]
                    add_comm_log("GCS→OBC", "导航目标",
                                 {"起点": f"({start_wp[1]:.6f}, {start_wp[0]:.6f})",
                                  "终点": f"({end_wp[1]:.6f}, {end_wp[0]:.6f})",
                                  "目标高度(m)": st.session_state.alt})
                    st.success("飞行开始，请切换至「监控」页面")

            if st.button("⏹️ 停止飞行"):
                st.session_state.running = False
                st.session_state.hb.stop()

            st.caption(f"航线共{len(st.session_state.waypoints)}个航点")
            if st.session_state.full_path:
                st.caption(f"完整路径含{len(st.session_state.full_path)}个航段点")

        with col2:
            center = st.session_state.waypoints[0] or SCHOOL_CENTER
            if st.session_state.full_path is None:
                st.session_state.full_path = plan_full_path(st.session_state.waypoints,
                                                              st.session_state.obs,
                                                              st.session_state.alt,
                                                              st.session_state.safe_rad,
                                                              st.session_state.sel_strat)
            drone_pos = st.session_state.hb.pos if st.session_state.running else None
            m = make_map(center, st.session_state.waypoints, st.session_state.obs, st.session_state.hist,
                        st.session_state.full_path, map_type,
                        st.session_state.safe_rad, st.session_state.alt, drone_pos)
            output = st_folium(m, width=700, height=550, returned_objects=["last_active_drawing"])
            if output and output.get("last_active_drawing"):
                d = output["last_active_drawing"]
                if d and d.get("geometry", {}).get("type") == "Polygon":
                    coords = d["geometry"]["coordinates"][0]
                    if len(coords) >= 3:
                        st.session_state.pending_poly = [[p[0], p[1]] for p in coords]
                        st.success("已捕获多边形，请设置高度后点「添加障碍物」")
            st.caption("图例：绿色=避障航线 红色=障碍物 橙色=安全区 | 蓝色旗帜=中间航点")

    elif page == "监控":
        st.header("📡 飞行实时画面 - 任务执行监控")

        # 自动刷新
        st_autorefresh(interval=HEARTBEAT_INTERVAL * 1000, key="monitor_autorefresh")

        # 控制按钮
        col1, col2, col3, col4 = st.columns(4)
        with col1:
            if st.button("▶️ 开始/继续", use_container_width=True):
                if not st.session_state.running:
                    if st.session_state.full_path is None:
                        st.warning("请先在规划页面刷新规划路径")
                    else:
                        st.session_state.hb.set_path(st.session_state.full_path, st.session_state.alt, st.session_state.drone_spd)
                        st.session_state.running = True
                        st.session_state.last_time = time.time()
                else:
                    st.session_state.hb.do_resume()

        with col2:
            if st.button("⏸️ 暂停", use_container_width=True):
                if st.session_state.running:
                    st.session_state.hb.do_pause()

        with col3:
            if st.button("⏹️ 停止", use_container_width=True):
                st.session_state.running = False
                st.session_state.hb.stop()

        with col4:
            if st.button("🔄 重置", use_container_width=True):
                st.session_state.running = False
                st.session_state.hb.reset()
                st.session_state.hist = []

        st.markdown("---")

        # 自动位置更新
        if st.session_state.running and not st.session_state.hb.is_paused:
            now = time.time()
            dt = min(now - st.session_state.last_time, HEARTBEAT_INTERVAL)
            if dt > 0:
                try:
                    st.session_state.hb.update(st.session_state.obs, st.session_state.safe_rad, dt)
                    if st.session_state.hb.hist:
                        d = st.session_state.hb.hist[0]
                        st.session_state.hist.append([d['lng'], d['lat']])
                        if len(st.session_state.hist) > 200:
                            st.session_state.hist.pop(0)
                        if d['arrived']:
                            st.session_state.running = False
                            st.success("🏁 无人机已安全到达目的地！")
                    st.session_state.last_time = now
                    st.session_state.update_counter += 1
                except Exception as e:
                    st.error(f"位置更新出错: {e}")

        # 获取最新心跳数据
        if st.session_state.hb.hist:
            d = st.session_state.hb.hist[0]
        else:
            d = {"speed": 0, "progress": 0, "elapsed": 0, "remaining_distance": 0,
                 "remain": "00:00", "battery": 0, "lng": 0, "lat": 0, "paused": False,
                 "altitude": 50}

        total_waypoints = len(st.session_state.waypoints)
        current_wp_num = int(d.get('progress', 0) * total_waypoints) + 1 if total_waypoints > 0 else 0
        current_wp_num = min(current_wp_num, total_waypoints)

        if st.session_state.running:
            if d.get('paused', False):
                status_text = "⏸️ 已暂停"
                status_color = "orange"
            else:
                status_text = "✈️ 飞行中"
                status_color = "green"
        else:
            status_text = "⏹️ 已停止"
            status_color = "red"

        st.markdown(f"### 状态: <span style='color:{status_color}'>{status_text}</span>", unsafe_allow_html=True)

        st.markdown("### ✈️ 飞行进度")
        st.progress(d.get('progress', 0), text=f"进度: {d.get('progress', 0)*100:.1f}%")

        st.markdown("### 📊 实时飞行数据")
        col_a, col_b, col_c, col_d = st.columns(4)
        col_a.metric("🎯 当前航点", f"{current_wp_num}/{total_waypoints}" if total_waypoints > 0 else "0/0")
        col_b.metric("💨 飞行速度", f"{d.get('speed', 0)} m/s")
        elapsed = d.get('elapsed', 0)
        col_c.metric("⏰ 已用时间", f"{int(elapsed//60):02d}:{int(elapsed%60):02d}")
        remaining = d.get('remaining_distance', 0)
        col_d.metric("📏 剩余距离", f"{remaining:.0f} m" if remaining >= 0 else "0 m")

        col_e, col_f = st.columns(2)
        col_e.metric("🕐 预计到达", d.get('remain', '00:00'))
        col_f.metric("🔋 电量模拟", f"{d.get('battery', 0)}%")

        st.markdown("---")
        st.info(f"📍 当前位置: 经度 {d.get('lng', 0):.6f}, 纬度 {d.get('lat', 0):.6f} | 高度: {d.get('altitude', 50)}m")

        st.markdown("### 🗺️ 实时飞行地图")
        if d.get('lat', 0) != 0:
            center = [d['lat'], d['lng']]
        elif st.session_state.waypoints:
            center = [st.session_state.waypoints[0][1], st.session_state.waypoints[0][0]]
        else:
            center = [SCHOOL_CENTER[1], SCHOOL_CENTER[0]]

        m = folium.Map(location=center, zoom_start=17, tiles=VEC_URL, attr=ATTR)
        for o in st.session_state.obs:
            coords = o.get('polygon', [])
            if len(coords) >= 3:
                folium.Polygon([[c[1], c[0]] for c in coords], color='red', fill=True, fill_opacity=0.3,
                              popup=f"{o.get('name', '障碍物')}\n高度:{o.get('height', 20)}m").add_to(m)
        if st.session_state.full_path and len(st.session_state.full_path) > 1:
            folium.PolyLine([[p[1], p[0]] for p in st.session_state.full_path], color='green', weight=3, opacity=0.8,
                           popup="规划航线").add_to(m)
        for i, wp in enumerate(st.session_state.waypoints):
            color = 'green' if i == 0 else ('red' if i == len(st.session_state.waypoints)-1 else 'blue')
            folium.Marker([wp[1], wp[0]], popup=f"航点{i+1}", icon=folium.Icon(color=color)).add_to(m)
        if st.session_state.hist and len(st.session_state.hist) > 1:
            trail = [[p[1], p[0]] for p in st.session_state.hist[-50:] if len(p) == 2]
            if len(trail) > 1:
                folium.PolyLine(trail, color='orange', weight=2, opacity=0.7, popup="历史轨迹").add_to(m)
        if d.get('lat', 0) != 0:
            folium.Marker([d['lat'], d['lng']],
                         popup=f"📍 无人机\n高度:{d.get('altitude', 50)}m\n速度:{d.get('speed', 0)}m/s",
                         icon=folium.Icon(color='red', icon='plane', prefix='fa')).add_to(m)
            folium.Circle([d['lat'], d['lng']], radius=st.session_state.safe_rad,
                         color='blue', fill=True, fill_opacity=0.2, popup=f"安全区 {st.session_state.safe_rad}m").add_to(m)
        st_folium(m, width=1000, height=500, returned_objects=[])

        st.markdown("### 📋 飞行日志")
        if st.session_state.hb.hist:
            log_df = pd.DataFrame([{
                "时间": h['timestamp'],
                "飞行时间": f"{h['elapsed']:.1f}s",
                "纬度": f"{h['lat']:.6f}",
                "经度": f"{h['lng']:.6f}",
                "高度": f"{h['altitude']}m",
                "速度": f"{h['speed']}m/s",
                "进度": f"{h['progress']*100:.1f}%"
            } for h in st.session_state.hb.hist[:10]])
            st.dataframe(log_df, use_container_width=True)

        st.info(f"🔄 监控页面每 {HEARTBEAT_INTERVAL} 秒自动刷新 | 位置更新次数: {st.session_state.update_counter}")

    elif page == "障碍物":
        st.header("🏗️ 障碍物管理")
        st.info(f"当前障碍物数量: {len(st.session_state.obs)}")

        for i, obs in enumerate(st.session_state.obs):
            col1, col2, col3 = st.columns([3, 1, 1])
            col1.write(f"{obs.get('name', f'障碍物{i+1}')} - 高度: {obs.get('height', 20)}m")
            if col3.button("删除", key=f"del_obs_{i}"):
                st.session_state.obs.pop(i)

        if st.button("清空所有障碍物"):
            st.session_state.obs = []

        if st.button("💾 保存到缓存"):
            save_cache()

        if st.button("📂 从缓存加载"):
            load_cache()

        st.markdown("### 🗺️ 障碍物分布图")
        m = folium.Map(location=[SCHOOL_CENTER[1], SCHOOL_CENTER[0]], zoom_start=16, tiles=VEC_URL, attr=ATTR)
        for o in st.session_state.obs:
            coords = o.get('polygon', [])
            if len(coords) >= 3:
                folium.Polygon([[c[1], c[0]] for c in coords], color='red', fill=True, fill_opacity=0.5,
                              popup=f"{o.get('name', '障碍物')}\n高度:{o.get('height', 20)}m").add_to(m)
        folium.Marker([A_DFT[1], A_DFT[0]], popup="起点", icon=folium.Icon(color='green')).add_to(m)
        folium.Marker([B_DFT[1], B_DFT[0]], popup="终点", icon=folium.Icon(color='red')).add_to(m)
        st_folium(m, width=700, height=500, returned_objects=[])

    elif page == "通信":
        show_communication_page()

if __name__ == "__main__":
    main()
