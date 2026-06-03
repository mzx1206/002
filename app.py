import streamlit as st
import time
import random
import pandas as pd
from datetime import datetime
import numpy as np
import folium
from streamlit_folium import folium_static
from folium.plugins import Fullscreen, MeasureControl
import plotly.graph_objects as go
from geopy.distance import geodesic
import math

# 设置页面配置
st.set_page_config(
    page_title="无人机心跳监控系统 - 3D地图",
    page_icon="🚁",
    layout="wide",
    initial_sidebar_state="expanded"
)

# 自定义CSS
st.markdown("""
<style>
    .stButton > button {
        width: 100%;
        background-color: #4CAF50;
        color: white;
        font-size: 16px;
        font-weight: bold;
    }
    .metric-card {
        background-color: #f0f2f6;
        padding: 15px;
        border-radius: 10px;
        margin: 10px 0;
    }
    .info-box {
        background-color: #e3f2fd;
        padding: 15px;
        border-radius: 10px;
        border-left: 5px solid #2196f3;
        margin: 10px 0;
    }
    .stAlert {
        margin-top: 10px;
        margin-bottom: 10px;
    }
</style>
""", unsafe_allow_html=True)

# ==================== 坐标系转换模块 ====================
class CoordinateConverter:
    """坐标系转换类 - 支持WGS84、UTM、笛卡尔坐标转换"""
    
    def __init__(self, center_lat=32.118, center_lon=118.9625):
        self.center_lat = center_lat
        self.center_lon = center_lon
        self.a = 6378137.0
        self.b = 6356752.314245
        self.e = math.sqrt(1 - (self.b**2 / self.a**2))
        
    def latlon_to_meters(self, lat, lon):
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        center_lat_rad = math.radians(self.center_lat)
        center_lon_rad = math.radians(self.center_lon)
        N = self.a / math.sqrt(1 - self.e**2 * math.sin(center_lat_rad)**2)
        x = N * math.cos(center_lat_rad) * (lon_rad - center_lon_rad)
        y = N * (lat_rad - center_lat_rad)
        return x, y
    
    def meters_to_latlon(self, x, y):
        center_lat_rad = math.radians(self.center_lat)
        center_lon_rad = math.radians(self.center_lon)
        N = self.a / math.sqrt(1 - self.e**2 * math.sin(center_lat_rad)**2)
        lat_rad = center_lat_rad + y / N
        lon_rad = center_lon_rad + x / (N * math.cos(center_lat_rad))
        return math.degrees(lat_rad), math.degrees(lon_rad)
    
    def calculate_distance(self, lat1, lon1, lat2, lon2):
        return geodesic((lat1, lon1), (lat2, lon2)).meters
    
    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        lon_diff = math.radians(lon2 - lon1)
        x = math.sin(lon_diff) * math.cos(lat2_rad)
        y = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(lon_diff)
        bearing = math.degrees(math.atan2(x, y))
        return (bearing + 360) % 360

# ==================== 无人机心跳模拟器 ====================
class DroneHeartbeatSimulator:
    def __init__(self, timeout=3, loss_rate=0.1, enable_delay=True, max_delay=0.5):
        self.timeout = timeout
        self.loss_rate = loss_rate
        self.enable_delay = enable_delay
        self.max_delay = max_delay
        self.sequence = 0
        self.last_heartbeat_time = time.time()
        self.connected = True
        self.heartbeat_data = []
        self.packet_loss_count = 0
        self.timeout_count = 0
        self.running = False
        self.start_time = None
        self.last_update_time = 0
        self.drone_position = None
        self.drone_position_meters = None
        self.flight_path = []
        self.flight_path_meters = []
        self.total_flight_distance = 0.0
        
    def send_heartbeat(self):
        self.sequence += 1
        heartbeat = {
            'seq': self.sequence,
            'timestamp': time.time(),
            'datetime': datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
            'status': 'SENT',
            'drone_pos': self.drone_position,
            'drone_pos_meters': self.drone_position_meters
        }
        self.heartbeat_data.append(heartbeat)
        return heartbeat
    
    def receive_heartbeat(self, heartbeat):
        current_time = time.time()
        delay = current_time - heartbeat['timestamp']
        heartbeat['status'] = 'RECEIVED'
        heartbeat['receive_time'] = current_time
        heartbeat['delay'] = delay
        self.last_heartbeat_time = current_time
        if not self.connected:
            self.connected = True
            return True
        return False
    
    def check_timeout(self):
        current_time = time.time()
        if self.connected and (current_time - self.last_heartbeat_time) > self.timeout:
            self.connected = False
            self.timeout_count += 1
            timeout_record = {
                'seq': f'TIMEOUT_{self.timeout_count}',
                'timestamp': current_time,
                'datetime': datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                'status': 'TIMEOUT'
            }
            self.heartbeat_data.append(timeout_record)
            return True
        return False
    
    def simulate_network_condition(self):
        if random.random() < self.loss_rate:
            return False, 0
        delay = 0
        if self.enable_delay and random.random() > 0.7:
            delay = random.uniform(0, self.max_delay)
            if delay > 0:
                time.sleep(delay)
        return True, delay
    
    def update_drone_position(self, progress, pointA, pointB, obstacles, converter):
        lat = pointA['lat'] + (pointB['lat'] - pointA['lat']) * progress
        lon = pointA['lon'] + (pointB['lon'] - pointA['lon']) * progress
        x, y = converter.latlon_to_meters(lat, lon)
        
        base_height = 50
        height = base_height
        for obs in obstacles:
            obs_lat, obs_lon = obs['position']
            distance = converter.calculate_distance(lat, lon, obs_lat, obs_lon)
            if distance < obs.get('radius', 50):
                avoidance_height = obs['height'] + 15
                if avoidance_height > height:
                    height = avoidance_height
        
        self.drone_position = {'lat': lat, 'lon': lon, 'height': height, 'progress': progress}
        self.drone_position_meters = {'x': x, 'y': y, 'z': height, 'progress': progress}
        
        if len(self.flight_path_meters) > 0:
            last_pos = self.flight_path_meters[-1]
            distance = math.sqrt((x - last_pos['x'])**2 + (y - last_pos['y'])**2)
            self.total_flight_distance += distance
        
        self.flight_path.append(self.drone_position.copy())
        self.flight_path_meters.append(self.drone_position_meters.copy())
        return self.drone_position
    
    def step(self, pointA=None, pointB=None, obstacles=None, converter=None):
        if not self.running:
            return None
        current_time = time.time()
        if current_time - self.last_update_time >= 1.0:
            self.last_update_time = current_time
            if pointA and pointB and self.start_time and converter:
                elapsed = current_time - self.start_time
                duration = 60
                progress = min(1.0, elapsed / duration)
                self.update_drone_position(progress, pointA, pointB, obstacles, converter)
            
            should_send, delay = self.simulate_network_condition()
            if should_send:
                heartbeat = self.send_heartbeat()
                connection_restored = self.receive_heartbeat(heartbeat)
                return {
                    'type': 'heartbeat',
                    'seq': heartbeat['seq'],
                    'delay': heartbeat.get('delay', 0),
                    'datetime': heartbeat['datetime'],
                    'connection_restored': connection_restored,
                    'drone_pos': self.drone_position,
                    'drone_pos_meters': self.drone_position_meters
                }
            else:
                self.packet_loss_count += 1
                lost_record = {
                    'seq': self.sequence + 1,
                    'timestamp': current_time,
                    'datetime': datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                    'status': 'LOST'
                }
                self.heartbeat_data.append(lost_record)
                return {'type': 'loss', 'seq': self.sequence + 1, 'datetime': lost_record['datetime']}
        
        if self.check_timeout():
            return {'type': 'timeout', 'count': self.timeout_count, 'datetime': datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]}
        return None
    
    def start(self, pointA=None, pointB=None, converter=None):
        self.running = True
        self.start_time = time.time()
        self.last_heartbeat_time = time.time()
        self.last_update_time = time.time()
        self.flight_path = []
        self.flight_path_meters = []
        self.total_flight_distance = 0.0
        if pointA and converter:
            self.drone_position = {'lat': pointA['lat'], 'lon': pointA['lon'], 'height': 50, 'progress': 0}
            x, y = converter.latlon_to_meters(pointA['lat'], pointA['lon'])
            self.drone_position_meters = {'x': x, 'y': y, 'z': 50, 'progress': 0}
            self.flight_path.append(self.drone_position.copy())
            self.flight_path_meters.append(self.drone_position_meters.copy())
        
    def stop(self):
        self.running = False
    
    def reset(self, timeout=3, loss_rate=0.1, enable_delay=True, max_delay=0.5):
        self.__init__(timeout, loss_rate, enable_delay, max_delay)
    
    def get_statistics(self):
        total_packets = len([item for item in self.heartbeat_data 
                            if isinstance(item.get('seq'), int) and item['status'] != 'TIMEOUT'])
        received = len([item for item in self.heartbeat_data if item['status'] == 'RECEIVED'])
        stats = {
            'total_packets': total_packets,
            'received': received,
            'lost': self.packet_loss_count,
            'timeout_count': self.timeout_count,
            'success_rate': (received / total_packets * 100) if total_packets > 0 else 0,
            'total_distance': self.total_flight_distance
        }
        if received > 0:
            delays = [item['delay'] for item in self.heartbeat_data 
                     if item['status'] == 'RECEIVED' and 'delay' in item]
            if delays:
                stats['avg_delay'] = float(np.mean(delays))
                stats['max_delay'] = float(max(delays))
                stats['min_delay'] = float(min(delays))
        return stats
    
    def get_dataframe(self):
        df_data = []
        for item in self.heartbeat_data:
            if item['status'] == 'RECEIVED':
                df_data.append({'序号': item['seq'], '时间': item['datetime'], '延迟(秒)': f"{item['delay']:.3f}", '状态': '✓ 正常'})
            elif item['status'] == 'LOST':
                df_data.append({'序号': item['seq'], '时间': item['datetime'], '延迟(秒)': '-', '状态': '✗ 丢包'})
            elif item['status'] == 'TIMEOUT':
                df_data.append({'序号': item['seq'], '时间': item['datetime'], '延迟(秒)': '-', '状态': '⚠ 超时'})
        return pd.DataFrame(df_data)

# ==================== 3D地图显示模块 ====================
def create_enhanced_3d_map(converter, pointA, pointB, obstacles, drone_pos=None, flight_path=None):
    """创建增强的3D地图 - 包含地形效果"""
    
    fig = go.Figure()
    
    # 添加起点A - 3D圆柱体效果
    if pointA:
        # 起点标记
        fig.add_trace(go.Scatter3d(
            x=[pointA['lon']], y=[pointA['lat']], z=[pointA.get('height', 0)],
            mode='markers+text',
            marker=dict(size=14, color='#00ff00', symbol='circle', line=dict(width=2, color='white')),
            text=['🚁 起点 A'],
            textposition='top center',
            name='起点 A',
            hovertemplate='<b>起点 A</b><br>经度: %{x:.6f}<br>纬度: %{y:.6f}<br>高度: %{z}m<extra></extra>'
        ))
        
        # 添加起点垂直柱
        fig.add_trace(go.Scatter3d(
            x=[pointA['lon'], pointA['lon']], y=[pointA['lat'], pointA['lat']], 
            z=[0, pointA.get('height', 0) + 10],
            mode='lines', line=dict(color='#00ff00', width=6, dash='solid'),
            showlegend=False, hoverinfo='skip'
        ))
    
    # 添加终点B - 3D圆柱体效果
    if pointB:
        fig.add_trace(go.Scatter3d(
            x=[pointB['lon']], y=[pointB['lat']], z=[pointB.get('height', 0)],
            mode='markers+text',
            marker=dict(size=14, color='#ff0000', symbol='circle', line=dict(width=2, color='white')),
            text=['🏁 终点 B'],
            textposition='top center',
            name='终点 B',
            hovertemplate='<b>终点 B</b><br>经度: %{x:.6f}<br>纬度: %{y:.6f}<br>高度: %{z}m<extra></extra>'
        ))
        
        fig.add_trace(go.Scatter3d(
            x=[pointB['lon'], pointB['lon']], y=[pointB['lat'], pointB['lat']], 
            z=[0, pointB.get('height', 0) + 10],
            mode='lines', line=dict(color='#ff0000', width=6, dash='solid'),
            showlegend=False, hoverinfo='skip'
        ))
    
    # 添加障碍物 - 3D建筑物效果
    if obstacles:
        for i, obs in enumerate(obstacles):
            obs_lon, obs_lat = obs['position']
            
            # 创建建筑物的3D立方体效果
            # 使用多个点来模拟建筑物
            size = obs.get('radius', 30) / 111000  # 转换为度
            
            # 建筑物主体
            fig.add_trace(go.Scatter3d(
                x=[obs_lon, obs_lon, obs_lon+size, obs_lon+size, obs_lon],
                y=[obs_lat, obs_lat+size, obs_lat+size, obs_lat, obs_lat],
                z=[obs['height']] * 5,
                mode='lines',
                line=dict(color='#ff6600', width=3),
                showlegend=False,
                hoverinfo='skip'
            ))
            
            # 建筑物顶部标记
            fig.add_trace(go.Scatter3d(
                x=[obs_lon + size/2], y=[obs_lat + size/2], z=[obs['height']],
                mode='markers+text',
                marker=dict(size=12, color='#ff6600', symbol='square', line=dict(width=1, color='white')),
                text=[obs['name']],
                textposition='top center',
                name=f'障碍物: {obs["name"]}',
                hovertemplate=f'<b>{obs["name"]}</b><br>高度: {obs["height"]}m<br>影响半径: {obs.get("radius", 50)}m<extra></extra>'
            ))
            
            # 建筑物垂直柱
            fig.add_trace(go.Scatter3d(
                x=[obs_lon + size/2, obs_lon + size/2],
                y=[obs_lat + size/2, obs_lat + size/2],
                z=[0, obs['height']],
                mode='lines',
                line=dict(color='#ff6600', width=8),
                showlegend=False,
                hoverinfo='skip'
            ))
    
    # 添加飞行路径 - 3D轨迹线
    if flight_path and len(flight_path) > 1:
        lons = [p['lon'] for p in flight_path]
        lats = [p['lat'] for p in flight_path]
        heights = [p.get('height', 50) for p in flight_path]
        
        # 主飞行路径
        fig.add_trace(go.Scatter3d(
            x=lons, y=lats, z=heights,
            mode='lines+markers',
            line=dict(color='#0066ff', width=5),
            marker=dict(size=4, color='#0066ff', symbol='circle'),
            name='飞行轨迹',
            hovertemplate='经度: %{x:.6f}<br>纬度: %{y:.6f}<br>高度: %{z:.1f}m<extra></extra>'
        ))
        
        # 添加路径上的高度变化标记
        for i in range(0, len(flight_path), max(1, len(flight_path)//20)):
            p = flight_path[i]
            fig.add_trace(go.Scatter3d(
                x=[p['lon']], y=[p['lat']], z=[p.get('height', 50)],
                mode='markers',
                marker=dict(size=6, color='#00ccff', symbol='diamond'),
                showlegend=False,
                hovertemplate=f'路径点 {i}<br>高度: {p.get("height", 50):.1f}m<extra></extra>'
            ))
    
    # 添加无人机当前位置 - 带拖尾效果
    if drone_pos:
        # 无人机主体
        fig.add_trace(go.Scatter3d(
            x=[drone_pos['lon']], y=[drone_pos['lat']], z=[drone_pos.get('height', 50)],
            mode='markers+text',
            marker=dict(size=16, color='#ff00ff', symbol='arrow', line=dict(width=2, color='white')),
            text=['✈ 无人机'],
            textposition='top center',
            name='无人机当前位置',
            hovertemplate=f'<b>无人机实时位置</b><br>经度: {drone_pos["lon"]:.6f}<br>纬度: {drone_pos["lat"]:.6f}<br>高度: {drone_pos.get("height", 50):.1f}m<br>进度: {drone_pos.get("progress", 0)*100:.1f}%<extra></extra>'
        ))
        
        # 无人机信号范围球体效果
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        radius = 0.0003  # 约30米
        x_sphere = drone_pos['lon'] + radius * np.outer(np.cos(u), np.sin(v))
        y_sphere = drone_pos['lat'] + radius * np.outer(np.sin(u), np.sin(v))
        z_sphere = drone_pos.get('height', 50) + radius * 100 * np.outer(np.ones_like(u), np.cos(v))
        
        fig.add_trace(go.Surface(
            x=x_sphere, y=y_sphere, z=z_sphere,
            opacity=0.3, colorscale=[[0, '#ff00ff'], [1, '#ff00ff']],
            showscale=False, name='信号范围', hoverinfo='skip'
        ))
    
    # 创建地形网格（模拟地面）
    lat_grid = np.linspace(32.10, 32.13, 30)
    lon_grid = np.linspace(118.95, 118.98, 30)
    Lon, Lat = np.meshgrid(lon_grid, lat_grid)
    Z = np.zeros_like(Lon)
    
    # 添加简单的地形起伏
    for i in range(len(lat_grid)):
        for j in range(len(lon_grid)):
            Z[i, j] = 5 + 2 * np.sin((lat_grid[i] - 32.115) * 100) * np.cos((lon_grid[j] - 118.962) * 100)
    
    fig.add_trace(go.Surface(
        x=Lon, y=Lat, z=Z,
        colorscale='Viridis',
        opacity=0.5,
        name='地形',
        showscale=False,
        hoverinfo='skip'
    ))
    
    # 设置3D场景布局
    fig.update_layout(
        title={
            'text': '🚁 无人机3D飞行可视化 - 实时监控',
            'x': 0.5,
            'xanchor': 'center',
            'font': {'size': 20, 'family': 'Arial Black', 'color': '#0066ff'}
        },
        scene=dict(
            xaxis=dict(
                title='经度',
                gridcolor='lightgray',
                showbackground=True,
                backgroundcolor='rgba(200, 200, 200, 0.1)',
                tickformat='.6f'
            ),
            yaxis=dict(
                title='纬度',
                gridcolor='lightgray',
                showbackground=True,
                backgroundcolor='rgba(200, 200, 200, 0.1)',
                tickformat='.6f'
            ),
            zaxis=dict(
                title='高度 (米)',
                gridcolor='lightgray',
                showbackground=True,
                backgroundcolor='rgba(200, 200, 200, 0.1)',
                range=[0, 120]
            ),
            camera=dict(
                eye=dict(x=1.8, y=1.8, z=1.5),
                center=dict(x=0, y=0, z=0),
                up=dict(x=0, y=0, z=1)
            ),
            aspectmode='manual',
            aspectratio=dict(x=1, y=1, z=0.4)
        ),
        showlegend=True,
        legend=dict(
            x=0.01,
            y=0.99,
            bgcolor='rgba(255, 255, 255, 0.9)',
            bordercolor='black',
            borderwidth=1,
            font=dict(size=12)
        ),
        height=700,
        margin=dict(l=0, r=0, t=60, b=0),
        hovermode='closest'
    )
    
    return fig


def create_route_planning_map(converter, pointA=None, pointB=None, obstacles=None):
    """创建航线规划2D地图"""
    center_lat = converter.center_lat
    center_lon = converter.center_lon
    
    m = folium.Map(
        location=[center_lat, center_lon], 
        zoom_start=16,
        tiles='https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}',
        attr='Google Satellite',
        control_scale=True
    )
    
    folium.TileLayer('OpenStreetMap', name='道路地图', overlay=False).add_to(m)
    Fullscreen().add_to(m)
    MeasureControl(position='topleft', primary_length_unit='meters').add_to(m)
    
    if pointA:
        folium.Marker(
            [pointA['lat'], pointA['lon']],
            popup=f'<b>🚁 起点 A</b><br>({pointA["lat"]:.6f}, {pointA["lon"]:.6f})',
            icon=folium.Icon(color='green', icon='play', prefix='fa')
        ).add_to(m)
        folium.Circle([pointA['lat'], pointA['lon']], radius=15, color='green', fill=True, fill_opacity=0.3).add_to(m)
    
    if pointB:
        folium.Marker(
            [pointB['lat'], pointB['lon']],
            popup=f'<b>🏁 终点 B</b><br>({pointB["lat"]:.6f}, {pointB["lon"]:.6f})',
            icon=folium.Icon(color='red', icon='flag-checkered', prefix='fa')
        ).add_to(m)
        folium.Circle([pointB['lat'], pointB['lon']], radius=15, color='red', fill=True, fill_opacity=0.3).add_to(m)
        
        if pointA:
            distance = converter.calculate_distance(pointA['lat'], pointA['lon'], pointB['lat'], pointB['lon'])
            bearing = converter.calculate_bearing(pointA['lat'], pointA['lon'], pointB['lat'], pointB['lon'])
            folium.PolyLine(
                [[pointA['lat'], pointA['lon']], [pointB['lat'], pointB['lon']]],
                color='#0066ff', weight=3, opacity=0.7, dash_array='5, 5',
                popup=f'航线距离: {distance:.2f}米 | 方位角: {bearing:.1f}°'
            ).add_to(m)
    
    if obstacles:
        for obs in obstacles:
            obs_lat, obs_lon = obs['position']
            folium.Circle(
                [obs_lat, obs_lon],
                radius=obs.get('radius', 50),
                color='#ff6600', fill=True, fill_opacity=0.3,
                popup=f'<b>{obs["name"]}</b><br>高度: {obs["height"]}米'
            ).add_to(m)
            folium.Marker(
                [obs_lat, obs_lon],
                icon=folium.Icon(color='orange', icon='warning-sign', prefix='glyphicon'),
                popup=obs['name']
            ).add_to(m)
    
    return m


def create_flight_monitoring_map(converter, pointA, pointB, obstacles, drone_pos, flight_path):
    """创建飞行监控2D地图"""
    if drone_pos:
        center_lat, center_lon = drone_pos['lat'], drone_pos['lon']
        zoom = 18
    else:
        center_lat, center_lon = converter.center_lat, converter.center_lon
        zoom = 16
    
    m = folium.Map(
        location=[center_lat, center_lon], 
        zoom_start=zoom,
        tiles='https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}',
        attr='Google Satellite',
        control_scale=True
    )
    
    folium.TileLayer('OpenStreetMap', name='道路地图', overlay=False).add_to(m)
    Fullscreen().add_to(m)
    
    if pointA:
        folium.Marker([pointA['lat'], pointA['lon']], icon=folium.Icon(color='green', icon='play', prefix='fa')).add_to(m)
    if pointB:
        folium.Marker([pointB['lat'], pointB['lon']], icon=folium.Icon(color='red', icon='flag-checkered', prefix='fa')).add_to(m)
    if obstacles:
        for obs in obstacles:
            obs_lat, obs_lon = obs['position']
            folium.Circle([obs_lat, obs_lon], radius=obs.get('radius', 50), color='#ff6600', fill=True, fill_opacity=0.3).add_to(m)
    if flight_path and len(flight_path) > 1:
        points = [[p['lat'], p['lon']] for p in flight_path]
        folium.PolyLine(points, color='#0066ff', weight=3, opacity=0.8).add_to(m)
    if drone_pos:
        folium.Marker(
            [drone_pos['lat'], drone_pos['lon']],
            icon=folium.Icon(color='purple', icon='plane', prefix='fa'),
            popup=f'进度: {drone_pos.get("progress", 0)*100:.1f}%<br>高度: {drone_pos.get("height", 50):.1f}m'
        ).add_to(m)
        folium.Circle([drone_pos['lat'], drone_pos['lon']], radius=30, color='purple', fill=True, fill_opacity=0.2).add_to(m)
    
    return m

# ==================== 预设地点 ====================
CAMPUS_LOCATIONS = {
    '图书馆': {'lat': 32.1185, 'lon': 118.9655, 'height': 35},
    '教学楼A': {'lat': 32.1125, 'lon': 118.9585, 'height': 30},
    '教学楼B': {'lat': 32.1145, 'lon': 118.9600, 'height': 28},
    '实验楼': {'lat': 32.1155, 'lon': 118.9705, 'height': 32},
    '行政楼': {'lat': 32.1215, 'lon': 118.9670, 'height': 40},
    '学生宿舍': {'lat': 32.1245, 'lon': 118.9550, 'height': 20},
    '体育馆': {'lat': 32.1095, 'lon': 118.9625, 'height': 25},
    '食堂': {'lat': 32.1235, 'lon': 118.9635, 'height': 15}
}

# ==================== 主程序 ====================
def main():
    st.title("🚁 无人机智能监控系统 - 3D可视化")
    st.markdown("### 支持心跳监控 + 3D航线规划 + 实时坐标转换")
    st.markdown("---")
    
    # 初始化session state
    if 'converter' not in st.session_state:
        st.session_state.converter = CoordinateConverter(center_lat=32.118, center_lon=118.9625)
    
    if 'simulator' not in st.session_state:
        st.session_state.simulator = DroneHeartbeatSimulator(timeout=3, loss_rate=0.1, enable_delay=True, max_delay=0.5)
        st.session_state.simulator_running = False
        st.session_state.simulation_log = []
        st.session_state.pointA = {'lat': 32.1125, 'lon': 118.9585, 'height': 0}
        st.session_state.pointB = {'lat': 32.1245, 'lon': 118.9550, 'height': 0}
        st.session_state.obstacles = []
    
    # 侧边栏
    with st.sidebar:
        st.header("⚙️ 系统配置")
        
        with st.expander("🗺️ 坐标系设置", expanded=True):
            st.info("南京大学仙林校区")
            center_lat = st.number_input("基准点纬度", value=st.session_state.converter.center_lat, format="%.6f")
            center_lon = st.number_input("基准点经度", value=st.session_state.converter.center_lon, format="%.6f")
            if st.button("更新基准点"):
                st.session_state.converter = CoordinateConverter(center_lat, center_lon)
                st.success("✅ 已更新")
        
        with st.expander("📡 通信参数", expanded=True):
            timeout = st.slider("超时阈值 (秒)", 1.0, 10.0, 3.0, 0.5)
            loss_rate = st.slider("丢包率 (%)", 0, 50, 10) / 100
            enable_delay = st.checkbox("启用延迟模拟", value=True)
            max_delay = st.slider("最大延迟 (秒)", 0.1, 2.0, 0.5, 0.1) if enable_delay else 0.5
        
        if st.button("🔄 重置系统", use_container_width=True):
            st.session_state.simulator.reset(timeout, loss_rate, enable_delay, max_delay)
            st.session_state.simulator_running = False
            st.session_state.simulation_log = []
            st.rerun()
    
    # 主标签页
    tab1, tab2 = st.tabs(["🗺️ 航线规划 (3D)", "📡 飞行监控 (3D实时)"])
    
    # ==================== 标签页1: 航线规划 ====================
    with tab1:
        st.header("🗺️ 3D航线规划")
        
        col1, col2 = st.columns([1, 2])
        
        with col1:
            st.subheader("📍 航线设置")
            
            # 快速选择
            col_p1, col_p2 = st.columns(2)
            with col_p1:
                presetA = st.selectbox("快速起点", ["自定义"] + list(CAMPUS_LOCATIONS.keys()), key="presetA")
                if presetA != "自定义":
                    st.session_state.pointA = CAMPUS_LOCATIONS[presetA].copy()
                    st.session_state.pointA['height'] = 0
            with col_p2:
                presetB = st.selectbox("快速终点", ["自定义"] + list(CAMPUS_LOCATIONS.keys()), key="presetB")
                if presetB != "自定义":
                    st.session_state.pointB = CAMPUS_LOCATIONS[presetB].copy()
                    st.session_state.pointB['height'] = 0
            
            # 起点
            st.markdown("**🚁 起点 A**")
            ca1, ca2 = st.columns(2)
            with ca1:
                latA = st.number_input("纬度", value=st.session_state.pointA['lat'], format="%.6f", key="latA")
            with ca2:
                lonA = st.number_input("经度", value=st.session_state.pointA['lon'], format="%.6f", key="lonA")
            heightA = st.number_input("高度(米)", value=st.session_state.pointA.get('height', 0), key="heightA")
            
            # 终点
            st.markdown("**🏁 终点 B**")
            cb1, cb2 = st.columns(2)
            with cb1:
                latB = st.number_input("纬度", value=st.session_state.pointB['lat'], format="%.6f", key="latB")
            with cb2:
                lonB = st.number_input("经度", value=st.session_state.pointB['lon'], format="%.6f", key="lonB")
            heightB = st.number_input("高度(米)", value=st.session_state.pointB.get('height', 0), key="heightB")
            
            # 障碍物
            st.markdown("**🏗️ 障碍物**")
            num_obs = st.number_input("数量", 0, 6, len(st.session_state.obstacles), key="num_obs")
            obstacles = []
            for i in range(num_obs):
                with st.expander(f"障碍物{i+1}"):
                    name = st.text_input(f"名称", f"建筑{i+1}", key=f"obs_name_{i}")
                    o1, o2 = st.columns(2)
                    with o1:
                        olat = st.number_input(f"纬度", 32.10, 32.13, 32.115 + i*0.003, format="%.6f", key=f"obs_lat_{i}")
                    with o2:
                        olon = st.number_input(f"经度", 118.95, 118.98, 118.962 + i*0.004, format="%.6f", key=f"obs_lon_{i}")
                    oheight = st.slider(f"高度(米)", 10, 80, 40 + i*10, key=f"obs_height_{i}")
                    oradius = st.slider(f"半径(米)", 30, 100, 50, key=f"obs_radius_{i}")
                    obstacles.append({'name': name, 'position': (olat, olon), 'height': oheight, 'radius': oradius})
            
            if st.button("✈️ 应用航线", use_container_width=True):
                st.session_state.pointA = {'lat': latA, 'lon': lonA, 'height': heightA}
                st.session_state.pointB = {'lat': latB, 'lon': lonB, 'height': heightB}
                st.session_state.obstacles = obstacles
                dist = st.session_state.converter.calculate_distance(latA, lonA, latB, lonB)
                st.success(f"✅ 航线已保存！距离: {dist:.1f}米 | 预计: {dist/10:.1f}秒")
        
        with col2:
            # 显示3D规划地图
            st.subheader("🗺️ 3D航线规划图")
            fig_3d_plan = create_enhanced_3d_map(
                st.session_state.converter,
                st.session_state.pointA,
                st.session_state.pointB,
                st.session_state.obstacles,
                drone_pos=None,
                flight_path=None
            )
            st.plotly_chart(fig_3d_plan, use_container_width=True)
            
            # 坐标系转换工具
            with st.expander("📐 坐标转换工具"):
                test_lat = st.number_input("测试纬度", 32.10, 32.13, 32.118, format="%.6f")
                test_lon = st.number_input("测试经度", 118.95, 118.98, 118.9625, format="%.6f")
                if st.button("转换"):
                    x, y = st.session_state.converter.latlon_to_meters(test_lat, test_lon)
                    st.code(f"米制坐标: X={x:.2f}m, Y={y:.2f}m")
                    lat2, lon2 = st.session_state.converter.meters_to_latlon(x, y)
                    st.info(f"反向验证: ({lat2:.6f}, {lon2:.6f})")
    
    # ==================== 标签页2: 飞行监控 ====================
    with tab2:
        st.header("📡 3D实时飞行监控")
        
        # 控制按钮
        c1, c2, c3 = st.columns(3)
        with c1:
            if not st.session_state.simulator_running:
                if st.button("▶️ 开始飞行", use_container_width=True):
                    st.session_state.simulator.reset(timeout, loss_rate, enable_delay, max_delay)
                    st.session_state.simulator.start(st.session_state.pointA, st.session_state.pointB, st.session_state.converter)
                    st.session_state.simulator_running = True
                    st.session_state.simulation_log = []
                    st.session_state.simulation_start_time = time.time()
                    st.rerun()
            else:
                if st.button("⏹️ 停止飞行", use_container_width=True):
                    st.session_state.simulator.stop()
                    st.session_state.simulator_running = False
                    st.rerun()
        with c2:
            if st.button("🗑️ 清空日志", use_container_width=True):
                st.session_state.simulation_log = []
                st.rerun()
        
        # 状态指标
        st.markdown("---")
        col1, col2, col3, col4, col5, col6 = st.columns(6)
        with col1:
            status = "🟢 正常" if st.session_state.simulator.connected else "🔴 超时"
            st.metric("连接状态", status)
        with col2:
            st.metric("心跳序号", st.session_state.simulator.sequence)
        with col3:
            st.metric("丢包数", st.session_state.simulator.packet_loss_count)
        with col4:
            st.metric("超时次数", st.session_state.simulator.timeout_count)
        with col5:
            if st.session_state.simulator.sequence > 0:
                rate = (st.session_state.simulator.sequence - st.session_state.simulator.packet_loss_count) / st.session_state.simulator.sequence * 100
                st.metric("成功率", f"{rate:.1f}%")
            else:
                st.metric("成功率", "0%")
        with col6:
            st.metric("飞行距离", f"{st.session_state.simulator.total_flight_distance:.1f}m")
        
        # 运行模拟
        if st.session_state.simulator_running:
            current_time = time.time()
            elapsed = current_time - st.session_state.simulation_start_time
            duration = 60
            
            if elapsed > duration:
                st.session_state.simulator.stop()
                st.session_state.simulator_running = False
                st.success("✅ 飞行完成！")
                st.balloons()
            else:
                progress = elapsed / duration
                st.progress(progress, text=f"✈ 进度: {progress*100:.1f}% ({elapsed:.0f}/{duration}秒)")
                
                result = st.session_state.simulator.step(
                    st.session_state.pointA, st.session_state.pointB,
                    st.session_state.obstacles, st.session_state.converter
                )
                
                if result:
                    if result['type'] == 'heartbeat':
                        msg = f"{'🟢 恢复' if result.get('connection_restored') else '✅ 心跳'} {result['seq']}: {result['delay']:.3f}s"
                        st.session_state.simulation_log.insert(0, f"[{result['datetime']}] {msg}")
                    elif result['type'] == 'loss':
                        st.session_state.simulation_log.insert(0, f"[{result['datetime']}] ❌ 丢包 {result['seq']}")
                    elif result['type'] == 'timeout':
                        st.session_state.simulation_log.insert(0, f"[{result['datetime']}] ⚠️ 超时 #{result['count']}")
                
                st.session_state.simulation_log = st.session_state.simulation_log[:100]
                time.sleep(0.05)
                st.rerun()
        
        # 3D地图显示
        st.subheader("🎯 3D实时飞行视图")
        fig_3d_live = create_enhanced_3d_map(
            st.session_state.converter,
            st.session_state.pointA,
            st.session_state.pointB,
            st.session_state.obstacles,
            st.session_state.simulator.drone_position,
            st.session_state.simulator.flight_path
        )
        st.plotly_chart(fig_3d_live, use_container_width=True)
        
        # 心跳数据
        st.markdown("---")
        tab_h1, tab_h2, tab_h3 = st.tabs(["📝 实时日志", "💓 心跳数据表", "📊 统计分析"])
        
        with tab_h1:
            if st.session_state.simulation_log:
                st.code("\n".join(st.session_state.simulation_log[:20]), language="log")
            else:
                st.info("等待飞行开始...")
        
        with tab_h2:
            df = st.session_state.simulator.get_dataframe()
            if not df.empty:
                st.dataframe(df, use_container_width=True, height=300)
                csv = df.to_csv(index=False)
                st.download_button("📥 下载数据", csv, "heartbeat.csv", "text/csv")
            else:
                st.info("暂无数据")
        
        with tab_h3:
            if st.session_state.simulator.sequence > 0:
                stats = st.session_state.simulator.get_statistics()
                col_s1, col_s2 = st.columns(2)
                with col_s1:
                    st.metric("总包数", stats['total_packets'])
                    st.metric("成功率", f"{stats['success_rate']:.1f}%")
                    st.metric("总距离", f"{stats['total_distance']:.1f}m")
                with col_s2:
                    if 'avg_delay' in stats:
                        st.metric("平均延迟", f"{stats['avg_delay']:.3f}s")
                        st.metric("最大延迟", f"{stats['max_delay']:.3f}s")
                        st.metric("最小延迟", f"{stats['min_delay']:.3f}s")
                
                if stats['success_rate'] >= 95:
                    st.success("🟢 通信质量: 优秀")
                elif stats['success_rate'] >= 85:
                    st.info("🟡 通信质量: 良好")
                elif stats['success_rate'] >= 70:
                    st.warning("🟠 通信质量: 一般")
                else:
                    st.error("🔴 通信质量: 较差")
            else:
                st.info("暂无统计数据")


if __name__ == "__main__":
    main()
