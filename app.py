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
    page_title="无人机心跳监控系统 - 双界面",
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
    .warning-box {
        background-color: #fff3e0;
        padding: 15px;
        border-radius: 10px;
        border-left: 5px solid #ff9800;
        margin: 10px 0;
    }
    .success-box {
        background-color: #e8f5e9;
        padding: 15px;
        border-radius: 10px;
        border-left: 5px solid #4caf50;
        margin: 10px 0;
    }
</style>
""", unsafe_allow_html=True)

# ==================== 坐标系转换模块 ====================
class CoordinateConverter:
    """坐标系转换类 - 支持WGS84、UTM、笛卡尔坐标转换"""
    
    def __init__(self, center_lat=32.118, center_lon=118.9625):
        """
        初始化坐标系转换器
        :param center_lat: 中心点纬度（度）
        :param center_lon: 中心点经度（度）
        """
        self.center_lat = center_lat
        self.center_lon = center_lon
        # WGS84椭球参数
        self.a = 6378137.0  # 长半轴（米）
        self.b = 6356752.314245  # 短半轴（米）
        self.e = math.sqrt(1 - (self.b**2 / self.a**2))  # 偏心率
        
    def latlon_to_meters(self, lat, lon):
        """
        将经纬度转换为米制坐标（基于中心点的局部坐标系）
        :param lat: 纬度（度）
        :param lon: 经度（度）
        :return: (x, y) 以米为单位的坐标
        """
        # 将度转换为弧度
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        center_lat_rad = math.radians(self.center_lat)
        center_lon_rad = math.radians(self.center_lon)
        
        # 计算子午圈曲率半径
        N = self.a / math.sqrt(1 - self.e**2 * math.sin(center_lat_rad)**2)
        
        # 计算X、Y方向的距离（米）
        x = N * math.cos(center_lat_rad) * (lon_rad - center_lon_rad)
        y = N * (lat_rad - center_lat_rad)
        
        return x, y
    
    def meters_to_latlon(self, x, y):
        """
        将米制坐标转换为经纬度
        :param x: X坐标（米）
        :param y: Y坐标（米）
        :return: (lat, lon) 经纬度（度）
        """
        center_lat_rad = math.radians(self.center_lat)
        center_lon_rad = math.radians(self.center_lon)
        
        # 计算子午圈曲率半径
        N = self.a / math.sqrt(1 - self.e**2 * math.sin(center_lat_rad)**2)
        
        # 计算经纬度
        lat_rad = center_lat_rad + y / N
        lon_rad = center_lon_rad + x / (N * math.cos(center_lat_rad))
        
        # 转换为度
        lat = math.degrees(lat_rad)
        lon = math.degrees(lon_rad)
        
        return lat, lon
    
    def calculate_distance(self, lat1, lon1, lat2, lon2):
        """
        计算两点之间的距离（米）- 使用Haversine公式
        """
        return geodesic((lat1, lon1), (lat2, lon2)).meters
    
    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        """
        计算从点1到点2的方位角（度）
        """
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        lon_diff = math.radians(lon2 - lon1)
        
        x = math.sin(lon_diff) * math.cos(lat2_rad)
        y = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(lon_diff)
        
        bearing = math.atan2(x, y)
        bearing = math.degrees(bearing)
        bearing = (bearing + 360) % 360
        
        return bearing

# ==================== 无人机心跳模拟器 ====================
class DroneHeartbeatSimulator:
    """无人机心跳模拟器类"""
    
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
        self.drone_position_meters = None  # 米制坐标
        self.flight_path = []
        self.flight_path_meters = []
        self.total_flight_distance = 0
        
    def send_heartbeat(self):
        """发送心跳包"""
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
        """接收心跳包"""
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
        """检查是否超时"""
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
        """模拟网络条件"""
        if random.random() < self.loss_rate:
            return False, 0
        
        delay = 0
        if self.enable_delay and random.random() > 0.7:
            delay = random.uniform(0, self.max_delay)
            if delay > 0:
                time.sleep(delay)
        
        return True, delay
    
    def update_drone_position(self, progress, pointA, pointB, obstacles, converter):
        """根据飞行进度更新无人机位置"""
        # 经纬度坐标
        lat = pointA['lat'] + (pointB['lat'] - pointA['lat']) * progress
        lon = pointA['lon'] + (pointB['lon'] - pointA['lon']) * progress
        
        # 转换为米制坐标
        x, y = converter.latlon_to_meters(lat, lon)
        xA, yA = converter.latlon_to_meters(pointA['lat'], pointA['lon'])
        xB, yB = converter.latlon_to_meters(pointB['lat'], pointB['lon'])
        
        # 计算高度（考虑障碍物）
        base_height = 50
        height = base_height
        
        # 检查是否靠近障碍物
        for obs in obstacles:
            obs_lat, obs_lon = obs['position']
            distance = converter.calculate_distance(lat, lon, obs_lat, obs_lon)
            if distance < obs.get('radius', 50):
                avoidance_height = obs['height'] + 15
                if avoidance_height > height:
                    height = avoidance_height
        
        self.drone_position = {'lat': lat, 'lon': lon, 'height': height, 'progress': progress}
        self.drone_position_meters = {'x': x, 'y': y, 'z': height, 'progress': progress}
        
        # 计算飞行距离
        if self.flight_path_meters:
            last_x, last_y = self.flight_path_meters[-1]['x'], self.flight_path_meters[-1]['y']
            distance = math.sqrt((x - last_x)**2 + (y - last_y)**2)
            self.total_flight_distance += distance
        
        self.flight_path.append(self.drone_position.copy())
        self.flight_path_meters.append(self.drone_position_meters.copy())
        
        return self.drone_position
    
    def step(self, pointA=None, pointB=None, obstacles=None, converter=None):
        """执行一步模拟"""
        if not self.running:
            return None
        
        current_time = time.time()
        
        if current_time - self.last_update_time >= 1.0:
            self.last_update_time = current_time
            
            # 更新无人机位置
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
                return {
                    'type': 'loss',
                    'seq': self.sequence + 1,
                    'datetime': lost_record['datetime']
                }
        
        if self.check_timeout():
            return {
                'type': 'timeout',
                'count': self.timeout_count,
                'datetime': datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            }
        
        return None
    
    def start(self, pointA=None, pointB=None, converter=None):
        """开始模拟"""
        self.running = True
        self.start_time = time.time()
        self.last_heartbeat_time = time.time()
        self.last_update_time = time.time()
        self.flight_path = []
        self.flight_path_meters = []
        self.total_flight_distance = 0
        if pointA and converter:
            self.drone_position = {'lat': pointA['lat'], 'lon': pointA['lon'], 'height': 50, 'progress': 0}
            x, y = converter.latlon_to_meters(pointA['lat'], pointA['lon'])
            self.drone_position_meters = {'x': x, 'y': y, 'z': 50, 'progress': 0}
            self.flight_path.append(self.drone_position.copy())
            self.flight_path_meters.append(self.drone_position_meters.copy())
        
    def stop(self):
        """停止模拟"""
        self.running = False
    
    def reset(self, timeout=3, loss_rate=0.1, enable_delay=True, max_delay=0.5):
        """重置模拟器"""
        self.__init__(timeout, loss_rate, enable_delay, max_delay)
    
    def get_statistics(self):
        """获取统计信息"""
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
        """获取数据框"""
        df_data = []
        for item in self.heartbeat_data:
            if item['status'] == 'RECEIVED':
                df_data.append({
                    '序号': item['seq'],
                    '时间': item['datetime'],
                    '延迟(秒)': f"{item['delay']:.3f}",
                    '状态': '✓ 正常'
                })
            elif item['status'] == 'LOST':
                df_data.append({
                    '序号': item['seq'],
                    '时间': item['datetime'],
                    '延迟(秒)': '-',
                    '状态': '✗ 丢包'
                })
            elif item['status'] == 'TIMEOUT':
                df_data.append({
                    '序号': item['seq'],
                    '时间': item['datetime'],
                    '延迟(秒)': '-',
                    '状态': '⚠ 超时'
                })
        return pd.DataFrame(df_data)

# ==================== 地图显示模块 ====================
def create_route_planning_map(converter, pointA=None, pointB=None, obstacles=None):
    """创建航线规划地图 - 2D高清地图"""
    # 默认中心点
    center_lat = converter.center_lat
    center_lon = converter.center_lon
    
    # 使用高分辨率卫星图层
    m = folium.Map(
        location=[center_lat, center_lon], 
        zoom_start=17,
        tiles='https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}',
        attr='Google Satellite',
        control_scale=True
    )
    
    # 添加多个地图图层
    folium.TileLayer(
        'https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}',
        attr='Google Hybrid',
        name='Google混合地图',
        overlay=False
    ).add_to(m)
    
    folium.TileLayer(
        'OpenStreetMap',
        name='OpenStreetMap',
        overlay=False
    ).add_to(m)
    
    # 添加全屏和测量工具
    Fullscreen().add_to(m)
    MeasureControl(position='topleft', primary_length_unit='meters').add_to(m)
    
    # 显示坐标系信息
    coord_info_html = f'''
    <div style="position: fixed; top: 10px; right: 10px; z-index: 1000; background-color: rgba(0,0,0,0.8); 
                color: white; padding: 10px; border-radius: 5px; font-size: 12px;">
        <b>🗺️ 坐标系信息</b><br>
        基准点: ({converter.center_lat:.6f}, {converter.center_lon:.6f})<br>
        投影: 局部切平面投影<br>
        单位: 米 (WGS84椭球)
    </div>
    '''
    m.get_root().html.add_child(folium.Element(coord_info_html))
    
    # 添加A点
    if pointA:
        xA, yA = converter.latlon_to_meters(pointA['lat'], pointA['lon'])
        folium.Marker(
            [pointA['lat'], pointA['lon']],
            popup=f"""
            <div style="width: 250px;">
                <b>🚁 起点 A</b><br>
                <hr>
                <b>经纬度:</b> ({pointA['lat']:.6f}, {pointA['lon']:.6f})<br>
                <b>米制坐标:</b> (X: {xA:.2f}m, Y: {yA:.2f}m)<br>
                <b>高度:</b> {pointA.get('height', 0)}米
            </div>
            """,
            icon=folium.Icon(color='green', icon='play', prefix='fa'),
            tooltip='起点 A'
        ).add_to(m)
        
        folium.Circle([pointA['lat'], pointA['lon']], radius=15, color='green', fill=True, fill_opacity=0.3).add_to(m)
    
    # 添加B点
    if pointB:
        xB, yB = converter.latlon_to_meters(pointB['lat'], pointB['lon'])
        folium.Marker(
            [pointB['lat'], pointB['lon']],
            popup=f"""
            <div style="width: 250px;">
                <b>🏁 终点 B</b><br>
                <hr>
                <b>经纬度:</b> ({pointB['lat']:.6f}, {pointB['lon']:.6f})<br>
                <b>米制坐标:</b> (X: {xB:.2f}m, Y: {yB:.2f}m)<br>
                <b>高度:</b> {pointB.get('height', 0)}米
            </div>
            """,
            icon=folium.Icon(color='red', icon='flag-checkered', prefix='fa'),
            tooltip='终点 B'
        ).add_to(m)
        
        folium.Circle([pointB['lat'], pointB['lon']], radius=15, color='red', fill=True, fill_opacity=0.3).add_to(m)
        
        # 计算并显示航线信息
        distance = converter.calculate_distance(pointA['lat'], pointA['lon'], pointB['lat'], pointB['lon'])
        bearing = converter.calculate_bearing(pointA['lat'], pointA['lon'], pointB['lat'], pointB['lon'])
        
        # 添加航线
        folium.PolyLine(
            [[pointA['lat'], pointA['lon']], [pointB['lat'], pointB['lon']]],
            color='#0066ff', weight=3, opacity=0.7, dash_array='5, 5',
            popup=f'航线距离: {distance:.2f}米 | 方位角: {bearing:.1f}°'
        ).add_to(m)
        
        # 显示航线信息面板
        route_info = f'''
        <div style="position: fixed; bottom: 50px; left: 50px; z-index: 1000; background-color: rgba(255,255,255,0.95); 
                    padding: 12px; border: 2px solid #0066ff; border-radius: 8px; font-size: 12px; min-width: 200px;">
            <b><span style="font-size: 14px;">✈ 航线规划信息</span></b><br>
            <hr style="margin: 5px 0;">
            <b>起点 A:</b> ({pointA['lat']:.6f}, {pointA['lon']:.6f})<br>
            <b>终点 B:</b> ({pointB['lat']:.6f}, {pointB['lon']:.6f})<br>
            <b>航线距离:</b> {distance:.2f} 米<br>
            <b>方位角:</b> {bearing:.1f}°<br>
            <b>预计飞行时间:</b> {distance/10:.1f} 秒 (速度10m/s)
        </div>
        '''
        m.get_root().html.add_child(folium.Element(route_info))
    
    # 添加障碍物
    if obstacles:
        for obs in obstacles:
            obs_lat, obs_lon = obs['position']
            x_obs, y_obs = converter.latlon_to_meters(obs_lat, obs_lon)
            
            folium.Circle(
                [obs_lat, obs_lon],
                radius=obs.get('radius', 50),
                color='#ff6600',
                fill=True,
                fill_opacity=0.4,
                popup=f"""
                <div style="width: 220px;">
                    <b>🏢 {obs['name']}</b><br>
                    <hr>
                    <b>经纬度:</b> ({obs_lat:.6f}, {obs_lon:.6f})<br>
                    <b>米制坐标:</b> (X: {x_obs:.2f}m, Y: {y_obs:.2f}m)<br>
                    <b>高度:</b> {obs['height']}米<br>
                    <b>影响半径:</b> {obs.get('radius', 50)}米
                </div>
                """,
                tooltip=f'⚠️ {obs["name"]}'
            ).add_to(m)
            
            folium.Marker(
                [obs_lat, obs_lon],
                icon=folium.Icon(color='orange', icon='warning-sign', prefix='glyphicon'),
                popup=obs['name']
            ).add_to(m)
    
    # 添加网格（米制坐标）
    for i in range(-500, 501, 100):
        # 经线方向
        lat1, lon1 = converter.meters_to_latlon(i, -500)
        lat2, lon2 = converter.meters_to_latlon(i, 500)
        folium.PolyLine([[lat1, lon1], [lat2, lon2]], color='gray', weight=1, opacity=0.3).add_to(m)
        
        # 纬线方向
        lat1, lon1 = converter.meters_to_latlon(-500, i)
        lat2, lon2 = converter.meters_to_latlon(500, i)
        folium.PolyLine([[lat1, lon1], [lat2, lon2]], color='gray', weight=1, opacity=0.3).add_to(m)
    
    return m


def create_flight_monitoring_map(converter, pointA, pointB, obstacles, drone_pos, flight_path):
    """创建飞行监控地图 - 2D高清地图（跟随无人机）"""
    if drone_pos:
        center_lat = drone_pos['lat']
        center_lon = drone_pos['lon']
        zoom = 18
    else:
        center_lat = converter.center_lat
        center_lon = converter.center_lon
        zoom = 17
    
    m = folium.Map(
        location=[center_lat, center_lon], 
        zoom_start=zoom,
        tiles='https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}',
        attr='Google Satellite',
        control_scale=True
    )
    
    folium.TileLayer('OpenStreetMap', name='OpenStreetMap', overlay=False).add_to(m)
    Fullscreen().add_to(m)
    
    # 添加A点
    if pointA:
        folium.Marker(
            [pointA['lat'], pointA['lon']],
            popup=f'<b>起点 A</b><br>({pointA["lat"]:.6f}, {pointA["lon"]:.6f})',
            icon=folium.Icon(color='green', icon='play', prefix='fa')
        ).add_to(m)
    
    # 添加B点
    if pointB:
        folium.Marker(
            [pointB['lat'], pointB['lon']],
            popup=f'<b>终点 B</b><br>({pointB["lat"]:.6f}, {pointB["lon"]:.6f})',
            icon=folium.Icon(color='red', icon='flag-checkered', prefix='fa')
        ).add_to(m)
    
    # 添加障碍物
    if obstacles:
        for obs in obstacles:
            obs_lat, obs_lon = obs['position']
            folium.Circle(
                [obs_lat, obs_lon],
                radius=obs.get('radius', 50),
                color='#ff6600',
                fill=True,
                fill_opacity=0.3,
                popup=f'<b>{obs["name"]}</b><br>高度: {obs["height"]}米'
            ).add_to(m)
    
    # 添加飞行路径
    if flight_path and len(flight_path) > 1:
        points = [[p['lat'], p['lon']] for p in flight_path]
        folium.PolyLine(points, color='#0066ff', weight=3, opacity=0.8, popup='飞行轨迹').add_to(m)
    
    # 添加无人机当前位置
    if drone_pos:
        x, y = converter.latlon_to_meters(drone_pos['lat'], drone_pos['lon'])
        folium.Marker(
            [drone_pos['lat'], drone_pos['lon']],
            popup=f"""
            <div style="width: 250px;">
                <b>✈ 无人机当前位置</b><br>
                <hr>
                <b>经纬度:</b> ({drone_pos['lat']:.6f}, {drone_pos['lon']:.6f})<br>
                <b>米制坐标:</b> (X: {x:.2f}m, Y: {y:.2f}m)<br>
                <b>高度:</b> {drone_pos.get('height', 50):.1f}米<br>
                <b>进度:</b> {drone_pos.get('progress', 0)*100:.1f}%
            </div>
            """,
            icon=folium.Icon(color='purple', icon='plane', prefix='fa'),
            tooltip='无人机'
        ).add_to(m)
        
        folium.Circle([drone_pos['lat'], drone_pos['lon']], radius=30, color='purple', fill=True, fill_opacity=0.2).add_to(m)
    
    # 添加图例
    legend_html = '''
    <div style="position: fixed; bottom: 20px; right: 20px; z-index: 1000; background-color: rgba(255,255,255,0.95); 
                padding: 10px; border: 2px solid #ccc; border-radius: 5px; font-size: 11px;">
        <b>图例</b><br>
        <span style="color: green;">●</span> 起点 A<br>
        <span style="color: red;">●</span> 终点 B<br>
        <span style="color: orange;">●</span> 障碍物<br>
        <span style="color: blue;">━</span> 飞行路径<br>
        <span style="color: purple;">✈</span> 无人机
    </div>
    '''
    m.get_root().html.add_child(folium.Element(legend_html))
    
    return m


def create_3d_flight_path(converter, pointA, pointB, obstacles, flight_path, drone_pos=None):
    """创建3D飞行路径图"""
    fig = go.Figure()
    
    # 添加A点
    if pointA:
        fig.add_trace(go.Scatter3d(
            x=[pointA['lon']], y=[pointA['lat']], z=[pointA.get('height', 0)],
            mode='markers+text', marker=dict(size=10, color='green'),
            text=['🚁 起点 A'], textposition='top center', name='起点 A'
        ))
    
    # 添加B点
    if pointB:
        fig.add_trace(go.Scatter3d(
            x=[pointB['lon']], y=[pointB['lat']], z=[pointB.get('height', 0)],
            mode='markers+text', marker=dict(size=10, color='red'),
            text=['🏁 终点 B'], textposition='top center', name='终点 B'
        ))
    
    # 添加障碍物
    if obstacles:
        for obs in obstacles:
            obs_lon, obs_lat = obs['position']
            fig.add_trace(go.Scatter3d(
                x=[obs_lon], y=[obs_lat], z=[obs['height']],
                mode='markers+text', marker=dict(size=12, color='orange', symbol='cube'),
                text=[obs['name']], textposition='top center', name=f'障碍物: {obs["name"]}'
            ))
            
            # 垂直柱
            fig.add_trace(go.Scatter3d(
                x=[obs_lon, obs_lon], y=[obs_lat, obs_lat], z=[0, obs['height']],
                mode='lines', line=dict(color='orange', width=5), showlegend=False
            ))
    
    # 添加飞行路径
    if flight_path and len(flight_path) > 1:
        lons = [p['lon'] for p in flight_path]
        lats = [p['lat'] for p in flight_path]
        heights = [p.get('height', 50) for p in flight_path]
        
        fig.add_trace(go.Scatter3d(
            x=lons, y=lats, z=heights,
            mode='lines+markers', line=dict(color='#0066ff', width=4),
            marker=dict(size=3, color='#0066ff'), name='飞行路径'
        ))
    
    # 添加无人机当前位置
    if drone_pos:
        fig.add_trace(go.Scatter3d(
            x=[drone_pos['lon']], y=[drone_pos['lat']], z=[drone_pos.get('height', 50)],
            mode='markers+text', marker=dict(size=12, color='purple', symbol='arrow'),
            text=['✈ 无人机'], textposition='top center', name='无人机当前位置'
        ))
    
    fig.update_layout(
        title='🚁 3D飞行路径可视化',
        scene=dict(
            xaxis_title='经度', yaxis_title='纬度', zaxis_title='高度 (米)',
            camera=dict(eye=dict(x=1.5, y=1.5, z=1.2))
        ),
        height=600, showlegend=True
    )
    
    return fig


# ==================== 主程序 ====================
def main():
    st.title("🚁 无人机智能监控系统")
    st.markdown("### 支持心跳监控 + 航线规划 + 坐标系转换")
    st.markdown("---")
    
    # 初始化坐标系转换器（南京大学仙林校区中心点）
    if 'converter' not in st.session_state:
        st.session_state.converter = CoordinateConverter(center_lat=32.118, center_lon=118.9625)
    
    # 侧边栏配置
    with st.sidebar:
        st.header("⚙️ 系统配置")
        
        # 坐标系设置
        with st.expander("🗺️ 坐标系设置", expanded=True):
            st.info(f"当前基准点: 南京大学仙林校区")
            center_lat = st.number_input("基准点纬度", value=32.118, format="%.6f", key="center_lat")
            center_lon = st.number_input("基准点经度", value=118.9625, format="%.6f", key="center_lon")
            
            if st.button("更新坐标系基准点"):
                st.session_state.converter = CoordinateConverter(center_lat, center_lon)
                st.success(f"坐标系已更新！")
        
        # 通信参数设置
        with st.expander("📡 通信参数", expanded=True):
            timeout = st.slider("超时阈值 (秒)", 1.0, 10.0, 3.0, 0.5)
            loss_rate = st.slider("丢包率 (%)", 0, 50, 10) / 100
            enable_delay = st.checkbox("启用延迟模拟", value=True)
            max_delay = st.slider("最大延迟 (秒)", 0.1, 2.0, 0.5, 0.1) if enable_delay else 0.5
        
        # 重置按钮
        if st.button("🔄 重置系统", use_container_width=True):
            if 'simulator' in st.session_state:
                st.session_state.simulator.reset(timeout, loss_rate, enable_delay, max_delay)
            st.session_state.simulator_running = False
            st.session_state.simulation_log = []
            st.rerun()
    
    # 初始化模拟器
    if 'simulator' not in st.session_state:
        st.session_state.simulator = DroneHeartbeatSimulator(timeout, loss_rate, enable_delay, max_delay)
        st.session_state.simulator_running = False
        st.session_state.simulation_log = []
        st.session_state.pointA = {'lat': 32.1125, 'lon': 118.9585, 'height': 0}
        st.session_state.pointB = {'lat': 32.1245, 'lon': 118.9550, 'height': 0}
        st.session_state.obstacles = []
    
    # 创建两个主要标签页
    tab1, tab2 = st.tabs(["🗺️ 航线规划", "📡 飞行监控"])
    
    # ==================== 标签页1: 航线规划 ====================
    with tab1:
        st.header("🗺️ 航线规划与地图显示")
        st.markdown("在此界面规划飞行航线，设置起点、终点和障碍物")
        
        col1, col2 = st.columns([1, 2])
        
        with col1:
            st.subheader("📍 航线设置")
            
            # 起点设置
            st.markdown("#### 🚁 起点 A")
            col_a1, col_a2 = st.columns(2)
            with col_a1:
                pointA_lat = st.number_input("纬度", value=st.session_state.pointA['lat'], format="%.6f", key="plan_latA")
            with col_a2:
                pointA_lon = st.number_input("经度", value=st.session_state.pointA['lon'], format="%.6f", key="plan_lonA")
            pointA_height = st.number_input("高度 (米)", value=0, key="plan_heightA")
            
            # 终点设置
            st.markdown("#### 🏁 终点 B")
            col_b1, col_b2 = st.columns(2)
            with col_b1:
                pointB_lat = st.number_input("纬度", value=st.session_state.pointB['lat'], format="%.6f", key="plan_latB")
            with col_b2:
                pointB_lon = st.number_input("经度", value=st.session_state.pointB['lon'], format="%.6f", key="plan_lonB")
            pointB_height = st.number_input("高度 (米)", value=0, key="plan_heightB")
            
            # 障碍物设置
            st.markdown("#### 🏗️ 障碍物")
            num_obstacles = st.number_input("障碍物数量", min_value=0, max_value=8, value=len(st.session_state.obstacles), key="plan_num_obs")
            
            obstacles = []
            for i in range(num_obstacles):
                with st.expander(f"障碍物 {i+1}"):
                    obs_name = st.text_input(f"名称", value=f"建筑物{i+1}", key=f"plan_obs_name_{i}")
                    col_o1, col_o2 = st.columns(2)
                    with col_o1:
                        obs_lat = st.number_input(f"纬度", value=32.115 + i*0.003, format="%.6f", key=f"plan_obs_lat_{i}")
                    with col_o2:
                        obs_lon = st.number_input(f"经度", value=118.962 + i*0.004, format="%.6f", key=f"plan_obs_lon_{i}")
                    obs_height = st.slider(f"高度 (米)", 10, 80, 40 + i*10, key=f"plan_obs_height_{i}")
                    obs_radius = st.slider(f"影响半径 (米)", 30, 100, 50, key=f"plan_obs_radius_{i}")
                    
                    obstacles.append({
                        'name': obs_name,
                        'position': (obs_lat, obs_lon),
                        'height': obs_height,
                        'radius': obs_radius
                    })
            
            # 应用航线按钮
            if st.button("✈️ 应用航线规划", use_container_width=True):
                st.session_state.pointA = {'lat': pointA_lat, 'lon': pointA_lon, 'height': pointA_height}
                st.session_state.pointB = {'lat': pointB_lat, 'lon': pointB_lon, 'height': pointB_height}
                st.session_state.obstacles = obstacles
                st.success("✅ 航线规划已保存！请切换到「飞行监控」标签页开始飞行")
                
                # 计算并显示航线信息
                distance = st.session_state.converter.calculate_distance(pointA_lat, pointA_lon, pointB_lat, pointB_lon)
                st.info(f"📏 航线距离: {distance:.2f} 米 | 预计飞行时间: {distance/10:.1f} 秒")
        
        with col2:
            # 显示航线规划地图
            st.subheader("🗺️ 航线规划地图")
            route_map = create_route_planning_map(
                st.session_state.converter,
                st.session_state.pointA,
                st.session_state.pointB,
                st.session_state.obstacles
            )
            folium_static(route_map, width=None, height=600)
            
            # 显示坐标系转换示例
            with st.expander("📐 坐标系转换示例"):
                st.markdown("**WGS84经纬度 → 米制坐标转换**")
                
                test_lat = st.number_input("测试纬度", value=32.118, format="%.6f", key="test_lat")
                test_lon = st.number_input("测试经度", value=118.9625, format="%.6f", key="test_lon")
                
                if st.button("转换", key="convert_btn"):
                    x, y = st.session_state.converter.latlon_to_meters(test_lat, test_lon)
                    st.success(f"经纬度 ({test_lat:.6f}, {test_lon:.6f}) → 米制坐标 (X: {x:.2f}m, Y: {y:.2f}m)")
                    
                    # 反向转换验证
                    lat2, lon2 = st.session_state.converter.meters_to_latlon(x, y)
                    st.info(f"反向转换验证: ({lat2:.6f}, {lon2:.6f})")
    
    # ==================== 标签页2: 飞行监控 ====================
    with tab2:
        st.header("📡 飞行监控与心跳数据显示")
        st.markdown("实时监控无人机飞行状态和心跳包通信")
        
        # 控制按钮
        col_ctrl1, col_ctrl2, col_ctrl3, col_ctrl4 = st.columns(4)
        
        with col_ctrl1:
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
        
        with col_ctrl2:
            if st.button("📊 查看报告", use_container_width=True):
                st.session_state.show_report = True
        
        with col_ctrl3:
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
            elapsed_time = current_time - st.session_state.simulation_start_time
            duration = 60  # 飞行时长
            
            if elapsed_time > duration:
                st.session_state.simulator.stop()
                st.session_state.simulator_running = False
                st.success("✅ 飞行完成！无人机已抵达终点")
                st.balloons()
            else:
                progress = elapsed_time / duration
                st.progress(progress, text=f"✈ 飞行进度: {progress*100:.1f}% ({elapsed_time:.0f}/{duration} 秒)")
                
                result = st.session_state.simulator.step(
                    st.session_state.pointA,
                    st.session_state.pointB,
                    st.session_state.obstacles,
                    st.session_state.converter
                )
                
                if result:
                    if result['type'] == 'heartbeat':
                        if result.get('connection_restored'):
                            msg = f"🟢 连接恢复 - 心跳 {result['seq']}: 延迟 {result['delay']:.3f}s"
                        else:
                            msg = f"✅ 心跳 {result['seq']}: 延迟 {result['delay']:.3f}s"
                        st.session_state.simulation_log.insert(0, f"[{result['datetime']}] {msg}")
                    elif result['type'] == 'loss':
                        msg = f"❌ 丢包 - 第{result['seq']}号心跳丢失"
                        st.session_state.simulation_log.insert(0, f"[{result['datetime']}] {msg}")
                    elif result['type'] == 'timeout':
                        msg = f"⚠️ 超时告警 - 第{result['count']}次连接超时"
                        st.session_state.simulation_log.insert(0, f"[{result['datetime']}] {msg}")
                
                st.session_state.simulation_log = st.session_state.simulation_log[:100]
                time.sleep(0.05)
                st.rerun()
        
        # 地图和数据显示
        map_tab, heartbeat_tab, stats_tab = st.tabs(["🗺️ 飞行地图", "💓 心跳数据", "📊 统计分析"])
        
        with map_tab:
            # 显示2D和3D地图
            sub_tab1, sub_tab2 = st.tabs(["🌍 2D 监控地图", "🎯 3D 飞行路径"])
            
            with sub_tab1:
                monitor_map = create_flight_monitoring_map(
                    st.session_state.converter,
                    st.session_state.pointA,
                    st.session_state.pointB,
                    st.session_state.obstacles,
                    st.session_state.simulator.drone_position,
                    st.session_state.simulator.flight_path
                )
                folium_static(monitor_map, width=None, height=500)
            
            with sub_tab2:
                fig_3d = create_3d_flight_path(
                    st.session_state.converter,
                    st.session_state.pointA,
                    st.session_state.pointB,
                    st.session_state.obstacles,
                    st.session_state.simulator.flight_path,
                    st.session_state.simulator.drone_position
                )
                st.plotly_chart(fig_3d, use_container_width=True)
        
        with heartbeat_tab:
            st.subheader("💓 心跳数据记录")
            
            # 实时日志
            st.markdown("#### 📝 实时事件日志")
            if st.session_state.simulation_log:
                st.code("\n".join(st.session_state.simulation_log[:20]), language="log")
            else:
                st.info("等待飞行开始...")
            
            # 数据表格
            st.markdown("#### 📋 详细数据表格")
            df = st.session_state.simulator.get_dataframe()
            if not df.empty:
                st.dataframe(df, use_container_width=True, height=300)
                
                csv = df.to_csv(index=False)
                st.download_button("📥 下载心跳数据", csv, "heartbeat_data.csv", "text/csv")
            else:
                st.info("暂无数据")
        
        with stats_tab:
            if st.session_state.simulator.sequence > 0:
                stats = st.session_state.simulator.get_statistics()
                
                col_s1, col_s2 = st.columns(2)
                
                with col_s1:
                    st.markdown("#### 📈 通信统计")
                    st.metric("总心跳包数", stats['total_packets'])
                    st.metric("成功接收", f"{stats['received']} ({stats['success_rate']:.1f}%)")
                    st.metric("丢包数", f"{stats['lost']} ({stats['lost']/stats['total_packets']*100:.1f}%)")
                    st.metric("超时次数", stats['timeout_count'])
                    st.metric("总飞行距离", f"{stats['total_distance']:.2f} 米")
                
                with col_s2:
                    if 'avg_delay' in stats:
                        st.markdown("#### ⏱️ 延迟统计")
                        st.metric("平均延迟", f"{stats['avg_delay']:.3f} 秒")
                        st.metric("最大延迟", f"{stats['max_delay']:.3f} 秒")
                        st.metric("最小延迟", f"{stats['min_delay']:.3f} 秒")
                
                # 质量评估
                st.markdown("#### 🎯 连接质量评估")
                if stats['success_rate'] >= 95:
                    st.success("🟢 优秀 - 通信质量非常好")
                elif stats['success_rate'] >= 85:
                    st.info("🟡 良好 - 通信质量正常")
                elif stats['success_rate'] >= 70:
                    st.warning("🟠 一般 - 建议检查网络")
                else:
                    st.error("🔴 较差 - 需要优化网络连接")
                
                # 无人机当前位置信息
                if st.session_state.simulator.drone_position:
                    st.markdown("#### 📍 无人机实时位置")
                    drone_pos = st.session_state.simulator.drone_position
                    x, y = st.session_state.converter.latlon_to_meters(drone_pos['lat'], drone_pos['lon'])
                    
                    col_pos1, col_pos2 = st.columns(2)
                    with col_pos1:
                        st.markdown(f"**经纬度坐标:**")
                        st.code(f"纬度: {drone_pos['lat']:.6f}\n经度: {drone_pos['lon']:.6f}")
                    with col_pos2:
                        st.markdown(f"**米制坐标 (局部坐标系):**")
                        st.code(f"X: {x:.2f} 米\nY: {y:.2f} 米\nZ: {drone_pos.get('height', 50):.1f} 米")
            else:
                st.info("暂无统计数据，请开始飞行")


if __name__ == "__main__":
    main()
