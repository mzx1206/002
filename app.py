import streamlit as st
import time
import random
import pandas as pd
from datetime import datetime
import numpy as np
import folium
from streamlit_folium import folium_static
from folium.plugins import MarkerCluster
import plotly.graph_objects as go
from geopy.distance import geodesic

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
    .heartbeat-normal {
        color: #00cc00;
        font-weight: bold;
    }
    .heartbeat-warning {
        color: #ffa500;
        font-weight: bold;
    }
    .heartbeat-error {
        color: #ff4b4b;
        font-weight: bold;
    }
    .info-box {
        background-color: #e3f2fd;
        padding: 15px;
        border-radius: 10px;
        border-left: 5px solid #2196f3;
        margin: 10px 0;
    }
</style>
""", unsafe_allow_html=True)

# 中国某大学校园的坐标范围（示例：南京大学仙林校区）
CAMPUS_BOUNDS = {
    'min_lat': 32.100, 'max_lat': 32.130,
    'min_lon': 118.950, 'max_lon': 118.980,
    'center_lat': 32.115, 'center_lon': 118.965
}

# 预设的校园关键地点
CAMPUS_LOCATIONS = {
    '图书馆': {'lat': 32.118, 'lon': 118.965, 'height': 25},
    '教学楼A': {'lat': 32.112, 'lon': 118.958, 'height': 30},
    '教学楼B': {'lat': 32.120, 'lon': 118.960, 'height': 30},
    '实验楼': {'lat': 32.115, 'lon': 118.970, 'height': 28},
    '行政楼': {'lat': 32.122, 'lon': 118.968, 'height': 35},
    '学生宿舍': {'lat': 32.125, 'lon': 118.955, 'height': 20},
    '体育馆': {'lat': 32.108, 'lon': 118.962, 'height': 22},
    '食堂': {'lat': 32.124, 'lon': 118.963, 'height': 18}
}

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
        self.flight_path = []  # 存储飞行路径
        
    def send_heartbeat(self):
        """发送心跳包"""
        self.sequence += 1
        heartbeat = {
            'seq': self.sequence,
            'timestamp': time.time(),
            'datetime': datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
            'status': 'SENT',
            'drone_pos': self.drone_position if self.drone_position else None
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
    
    def update_drone_position(self, progress, pointA, pointB, obstacles):
        """根据飞行进度更新无人机位置"""
        lat = pointA['lat'] + (pointB['lat'] - pointA['lat']) * progress
        lon = pointA['lon'] + (pointB['lon'] - pointA['lon']) * progress
        
        # 计算高度（考虑障碍物）
        base_height = 50  # 基础飞行高度（米）
        height = base_height
        
        # 检查是否靠近障碍物，增加高度
        for obs in obstacles:
            obs_lat, obs_lon = obs['position']
            distance = geodesic((lat, lon), (obs_lat, obs_lon)).meters
            if distance < 50:  # 50米范围内
                height = max(height, obs['height'] + 10)  # 飞越障碍物
        
        self.drone_position = {'lat': lat, 'lon': lon, 'height': height, 'progress': progress}
        self.flight_path.append(self.drone_position.copy())
        
        return self.drone_position
    
    def step(self, pointA=None, pointB=None, obstacles=None):
        """执行一步模拟"""
        if not self.running:
            return None
        
        current_time = time.time()
        
        if current_time - self.last_update_time >= 1.0:
            self.last_update_time = current_time
            
            # 更新无人机位置
            if pointA and pointB and self.start_time:
                elapsed = current_time - self.start_time
                duration = 60  # 60秒完成飞行
                progress = min(1.0, elapsed / duration)
                self.update_drone_position(progress, pointA, pointB, obstacles)
            
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
                    'drone_pos': self.drone_position
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
    
    def start(self, pointA=None, pointB=None):
        """开始模拟"""
        self.running = True
        self.start_time = time.time()
        self.last_heartbeat_time = time.time()
        self.last_update_time = time.time()
        self.flight_path = []
        if pointA:
            self.drone_position = {'lat': pointA['lat'], 'lon': pointA['lon'], 'height': 50, 'progress': 0}
            self.flight_path.append(self.drone_position.copy())
        
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
            'success_rate': (received / total_packets * 100) if total_packets > 0 else 0
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


def create_3d_map(pointA, pointB, obstacles, drone_pos=None, flight_path=None):
    """创建3D地图"""
    fig = go.Figure()
    
    # 添加A点
    fig.add_trace(go.Scatter3d(
        x=[pointA['lon']],
        y=[pointA['lat']],
        z=[pointA.get('height', 0)],
        mode='markers+text',
        marker=dict(size=10, color='green', symbol='circle'),
        text=['A点'],
        textposition='top center',
        name='起点 A',
        hovertemplate='起点A<br>经度: %{x:.4f}<br>纬度: %{y:.4f}<br>高度: %{z}m<extra></extra>'
    ))
    
    # 添加B点
    fig.add_trace(go.Scatter3d(
        x=[pointB['lon']],
        y=[pointB['lat']],
        z=[pointB.get('height', 0)],
        mode='markers+text',
        marker=dict(size=10, color='red', symbol='circle'),
        text=['B点'],
        textposition='top center',
        name='终点 B',
        hovertemplate='终点B<br>经度: %{x:.4f}<br>纬度: %{y:.4f}<br>高度: %{z}m<extra></extra>'
    ))
    
    # 添加障碍物
    for i, obs in enumerate(obstacles):
        obs_lon, obs_lat = obs['position']
        fig.add_trace(go.Scatter3d(
            x=[obs_lon],
            y=[obs_lat],
            z=[0],
            mode='markers+text',
            marker=dict(size=15, color='orange', symbol='cube'),
            text=[obs['name']],
            textposition='top center',
            name=f'障碍物: {obs["name"]}',
            hovertemplate=f'{obs["name"]}<br>高度: {obs["height"]}m<br>经度: {obs_lon:.4f}<br>纬度: {obs_lat:.4f}<extra></extra>'
        ))
        
        # 添加障碍物的垂直柱状表示
        fig.add_trace(go.Scatter3d(
            x=[obs_lon, obs_lon],
            y=[obs_lat, obs_lat],
            z=[0, obs['height']],
            mode='lines',
            line=dict(color='orange', width=5),
            showlegend=False,
            hoverinfo='skip'
        ))
    
    # 添加飞行路径
    if flight_path and len(flight_path) > 1:
        lons = [p['lon'] for p in flight_path]
        lats = [p['lat'] for p in flight_path]
        heights = [p.get('height', 50) for p in flight_path]
        
        fig.add_trace(go.Scatter3d(
            x=lons,
            y=lats,
            z=heights,
            mode='lines+markers',
            line=dict(color='blue', width=4),
            marker=dict(size=3, color='blue'),
            name='飞行路径',
            hovertemplate='经度: %{x:.4f}<br>纬度: %{y:.4f}<br>高度: %{z:.1f}m<extra></extra>'
        ))
    
    # 添加无人机当前位置
    if drone_pos:
        fig.add_trace(go.Scatter3d(
            x=[drone_pos['lon']],
            y=[drone_pos['lat']],
            z=[drone_pos.get('height', 50)],
            mode='markers+text',
            marker=dict(size=12, color='purple', symbol='arrow', line=dict(width=2, color='black')),
            text=['✈ 无人机'],
            textposition='top center',
            name='无人机当前位置',
            hovertemplate=f'无人机<br>经度: {drone_pos["lon"]:.4f}<br>纬度: {drone_pos["lat"]:.4f}<br>高度: {drone_pos.get("height", 50):.1f}m<br>进度: {drone_pos.get("progress", 0)*100:.1f}%<extra></extra>'
        ))
    
    # 设置地图布局
    fig.update_layout(
        title={
            'text': '🚁 无人机飞行路径3D地图',
            'x': 0.5,
            'xanchor': 'center',
            'font': {'size': 20, 'family': 'Arial Black'}
        },
        scene=dict(
            xaxis=dict(title='经度', gridcolor='lightgray', showbackground=True, backgroundcolor='white'),
            yaxis=dict(title='纬度', gridcolor='lightgray', showbackground=True, backgroundcolor='white'),
            zaxis=dict(title='高度 (米)', gridcolor='lightgray', showbackground=True, backgroundcolor='white', range=[0, 120]),
            camera=dict(
                eye=dict(x=1.5, y=1.5, z=1.2)
            ),
            aspectmode='manual',
            aspectratio=dict(x=1, y=1, z=0.5)
        ),
        showlegend=True,
        legend=dict(
            x=0.01,
            y=0.99,
            bgcolor='rgba(255, 255, 255, 0.8)',
            bordercolor='black',
            borderwidth=1
        ),
        height=600,
        margin=dict(l=0, r=0, t=50, b=0)
    )
    
    return fig


def create_2d_map(pointA, pointB, obstacles, drone_pos=None, flight_path=None):
    """创建2D地图（Folium）"""
    # 计算地图中心点
    center_lat = (pointA['lat'] + pointB['lat']) / 2
    center_lon = (pointA['lon'] + pointB['lon']) / 2
    
    # 创建地图
    m = folium.Map(location=[center_lat, center_lon], zoom_start=16, control_scale=True)
    
    # 添加A点标记
    folium.Marker(
        [pointA['lat'], pointA['lon']],
        popup='<b>起点 A</b><br>飞行起点',
        icon=folium.Icon(color='green', icon='play', prefix='fa'),
        tooltip='起点 A'
    ).add_to(m)
    
    # 添加B点标记
    folium.Marker(
        [pointB['lat'], pointB['lon']],
        popup='<b>终点 B</b><br>飞行终点',
        icon=folium.Icon(color='red', icon='flag-checkered', prefix='fa'),
        tooltip='终点 B'
    ).add_to(m)
    
    # 添加障碍物
    for obs in obstacles:
        # 创建圆形区域表示障碍物影响范围
        folium.Circle(
            [obs['position'][1], obs['position'][0]],
            radius=obs.get('radius', 50),
            color='orange',
            fill=True,
            fill_opacity=0.3,
            popup=f'<b>{obs["name"]}</b><br>高度: {obs["height"]}米<br>影响半径: {obs.get("radius", 50)}米',
            tooltip=obs['name']
        ).add_to(m)
        
        folium.Marker(
            [obs['position'][1], obs['position'][0]],
            popup=f'<b>{obs["name"]}</b><br>高度: {obs["height"]}米',
            icon=folium.Icon(color='orange', icon='warning-sign', prefix='glyphicon'),
            tooltip=obs['name']
        ).add_to(m)
    
    # 添加飞行路径
    if flight_path and len(flight_path) > 1:
        points = [[p['lat'], p['lon']] for p in flight_path]
        folium.PolyLine(
            points,
            color='blue',
            weight=3,
            opacity=0.8,
            popup='飞行路径',
            tooltip='飞行轨迹'
        ).add_to(m)
        
        # 添加方向箭头
        for i in range(0, len(points)-1, max(1, len(points)//10)):
            folium.plugins.PolyLineTextPath(
                folium.PolyLine(points[i:i+2]),
                '▶',
                repeat=True,
                offset=5,
                attributes={'fill': 'blue', 'font-size': '12'}
            ).add_to(m)
    
    # 添加无人机当前位置
    if drone_pos:
        folium.Marker(
            [drone_pos['lat'], drone_pos['lon']],
            popup=f'<b>无人机当前位置</b><br>进度: {drone_pos.get("progress", 0)*100:.1f}%<br>高度: {drone_pos.get("height", 50):.1f}米',
            icon=folium.Icon(color='purple', icon='plane', prefix='fa'),
            tooltip='无人机'
        ).add_to(m)
    
    # 添加图例
    legend_html = '''
    <div style="position: fixed; bottom: 50px; right: 50px; z-index: 1000; background-color: white; padding: 10px; border: 2px solid grey; border-radius: 5px;">
        <p><b>图例</b></p>
        <p><span style="color: green;">●</span> 起点 A</p>
        <p><span style="color: red;">●</span> 终点 B</p>
        <p><span style="color: orange;">●</span> 障碍物</p>
        <p><span style="color: blue;">━</span> 飞行路径</p>
        <p><span style="color: purple;">✈</span> 无人机当前位置</p>
    </div>
    '''
    m.get_root().html.add_child(folium.Element(legend_html))
    
    return m


def main():
    st.title("🚁 无人机心跳监控系统 - 3D地图导航")
    st.markdown("实时监控无人机飞行状态，支持3D地图显示和障碍物规避")
    st.markdown("---")
    
    # 侧边栏配置
    with st.sidebar:
        st.header("⚙️ 飞行路径配置")
        
        st.subheader("📍 起点 A")
        col1, col2 = st.columns(2)
        with col1:
            pointA_lat = st.number_input("纬度", min_value=32.10, max_value=32.13, value=32.112, format="%.6f")
        with col2:
            pointA_lon = st.number_input("经度", min_value=118.95, max_value=118.98, value=118.958, format="%.6f")
        
        # 预设地点快速选择
        presetA = st.selectbox("快速选择预设地点", ["自定义"] + list(CAMPUS_LOCATIONS.keys()), key="presetA")
        if presetA != "自定义":
            pointA_lat = CAMPUS_LOCATIONS[presetA]['lat']
            pointA_lon = CAMPUS_LOCATIONS[presetA]['lon']
        
        pointA_height = st.number_input("A点高度 (米)", min_value=0, max_value=100, value=0)
        
        st.markdown("---")
        
        st.subheader("📍 终点 B")
        col3, col4 = st.columns(2)
        with col3:
            pointB_lat = st.number_input("纬度", min_value=32.10, max_value=32.13, value=32.125, format="%.6f", key="latB")
        with col4:
            pointB_lon = st.number_input("经度", min_value=118.95, max_value=118.98, value=118.955, format="%.6f", key="lonB")
        
        presetB = st.selectbox("快速选择预设地点", ["自定义"] + list(CAMPUS_LOCATIONS.keys()), key="presetB")
        if presetB != "自定义":
            pointB_lat = CAMPUS_LOCATIONS[presetB]['lat']
            pointB_lon = CAMPUS_LOCATIONS[presetB]['lon']
        
        pointB_height = st.number_input("B点高度 (米)", min_value=0, max_value=100, value=0)
        
        st.markdown("---")
        
        st.subheader("🏗️ 障碍物配置")
        
        # 预设障碍物
        obstacles = []
        num_obstacles = st.slider("障碍物数量", 1, 8, 3)
        
        for i in range(num_obstacles):
            with st.expander(f"障碍物 {i+1}"):
                obs_name = st.text_input(f"名称", value=f"建筑物{i+1}", key=f"obs_name_{i}")
                
                col5, col6 = st.columns(2)
                with col5:
                    obs_lat = st.number_input(f"纬度", min_value=32.10, max_value=32.13, 
                                             value=32.115 + i*0.003, format="%.6f", key=f"obs_lat_{i}")
                with col6:
                    obs_lon = st.number_input(f"经度", min_value=118.95, max_value=118.98, 
                                             value=118.962 + i*0.004, format="%.6f", key=f"obs_lon_{i}")
                
                obs_height = st.slider(f"高度 (米)", min_value=10, max_value=100, value=40 + i*10, key=f"obs_height_{i}")
                obs_radius = st.slider(f"影响半径 (米)", min_value=20, max_value=100, value=50, key=f"obs_radius_{i}")
                
                obstacles.append({
                    'name': obs_name,
                    'position': (obs_lon, obs_lat),
                    'height': obs_height,
                    'radius': obs_radius
                })
        
        st.markdown("---")
        
        st.header("⚙️ 通信参数")
        
        timeout = st.slider("超时阈值 (秒)", 1.0, 10.0, 3.0, 0.5)
        loss_rate = st.slider("丢包率 (%)", 0, 50, 10) / 100
        enable_delay = st.checkbox("启用延迟模拟", value=True)
        max_delay = st.slider("最大延迟 (秒)", 0.1, 2.0, 0.5, 0.1) if enable_delay else 0.5
        
        st.markdown("---")
        duration = st.number_input("飞行时长 (秒)", min_value=10, max_value=300, value=60, step=5)
        
        st.markdown("---")
        
        col1, col2 = st.columns(2)
        with col1:
            if st.button("▶️ 开始飞行", use_container_width=True):
                pointA = {'lat': pointA_lat, 'lon': pointA_lon, 'height': pointA_height}
                pointB = {'lat': pointB_lat, 'lon': pointB_lon, 'height': pointB_height}
                
                if 'simulator' in st.session_state:
                    st.session_state.simulator.reset(timeout, loss_rate, enable_delay, max_delay)
                else:
                    st.session_state.simulator = DroneHeartbeatSimulator(timeout, loss_rate, enable_delay, max_delay)
                
                st.session_state.simulator.start(pointA, pointB)
                st.session_state.simulator_running = True
                st.session_state.simulation_log = []
                st.session_state.simulation_start_time = time.time()
                st.session_state.pointA = pointA
                st.session_state.pointB = pointB
                st.session_state.obstacles = obstacles
                st.rerun()
        
        with col2:
            if st.button("⏹️ 停止飞行", use_container_width=True):
                if 'simulator' in st.session_state:
                    st.session_state.simulator.stop()
                st.session_state.simulator_running = False
                st.rerun()
        
        if st.button("🔄 重置系统", use_container_width=True):
            if 'simulator' in st.session_state:
                st.session_state.simulator.reset(timeout, loss_rate, enable_delay, max_delay)
            st.session_state.simulator_running = False
            st.session_state.simulation_log = []
            st.rerun()
    
    # 初始化
    if 'simulator' not in st.session_state:
        st.session_state.simulator = DroneHeartbeatSimulator(timeout, loss_rate, enable_delay, max_delay)
        st.session_state.simulator_running = False
        st.session_state.simulation_log = []
        st.session_state.pointA = {'lat': 32.112, 'lon': 118.958, 'height': 0}
        st.session_state.pointB = {'lat': 32.125, 'lon': 118.955, 'height': 0}
        st.session_state.obstacles = []
    
    # 运行模拟
    if st.session_state.simulator_running:
        current_time = time.time()
        elapsed_time = current_time - st.session_state.simulation_start_time
        
        if elapsed_time > duration:
            st.session_state.simulator.stop()
            st.session_state.simulator_running = False
            st.success(f"✅ 飞行完成！无人机已抵达终点")
        else:
            progress = elapsed_time / duration
            st.progress(progress, text=f"飞行进度: {progress*100:.1f}% ({elapsed_time:.0f}/{duration} 秒)")
            
            result = st.session_state.simulator.step(
                st.session_state.pointA, 
                st.session_state.pointB,
                st.session_state.obstacles
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
    
    # 显示状态指标
    col1, col2, col3, col4, col5 = st.columns(5)
    
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
    
    # 3D地图显示
    st.markdown("### 🗺️ 3D飞行地图")
    
    # 创建2D和3D地图的选项卡
    map_tab1, map_tab2 = st.tabs(["🌍 3D 地图", "🗺️ 2D 地图"])
    
    with map_tab1:
        if st.session_state.pointA and st.session_state.pointB:
            fig = create_3d_map(
                st.session_state.pointA,
                st.session_state.pointB,
                st.session_state.obstacles,
                st.session_state.simulator.drone_position,
                st.session_state.simulator.flight_path
            )
            st.plotly_chart(fig, use_container_width=True)
    
    with map_tab2:
        if st.session_state.pointA and st.session_state.pointB:
            m = create_2d_map(
                st.session_state.pointA,
                st.session_state.pointB,
                st.session_state.obstacles,
                st.session_state.simulator.drone_position,
                st.session_state.simulator.flight_path
            )
            folium_static(m, width=None, height=500)
    
    # 飞行信息
    if st.session_state.simulator.drone_position:
        st.markdown("### 📊 实时飞行信息")
        col1, col2, col3, col4 = st.columns(4)
        with col1:
            st.metric("当前位置进度", f"{st.session_state.simulator.drone_position.get('progress', 0)*100:.1f}%")
        with col2:
            st.metric("当前高度", f"{st.session_state.simulator.drone_position.get('height', 50):.1f} 米")
        with col3:
            # 计算到终点的距离
            if 'progress' in st.session_state.simulator.drone_position:
                remaining = (1 - st.session_state.simulator.drone_position['progress']) * 100
                st.metric("剩余距离", f"{remaining:.1f}%")
        with col4:
            st.metric("飞行状态", "飞行中" if st.session_state.simulator_running else "已停止")
    
    # 日志区域
    st.markdown("### 📝 实时事件日志")
    if st.session_state.simulation_log:
        st.code("\n".join(st.session_state.simulation_log[:15]), language="log")
    else:
        st.info("等待飞行开始...")
    
    # 数据显示
    st.markdown("---")
    tab1, tab2 = st.tabs(["📊 统计报告", "📋 数据表格"])
    
    with tab1:
        if st.session_state.simulator.sequence > 0:
            stats = st.session_state.simulator.get_statistics()
            
            col1, col2 = st.columns(2)
            with col1:
                st.metric("总包数", stats['total_packets'])
                st.metric("成功接收", f"{stats['received']} ({stats['success_rate']:.1f}%)")
                st.metric("丢包率", f"{stats['lost']/stats['total_packets']*100:.1f}%")
            
            with col2:
                if 'avg_delay' in stats:
                    st.metric("平均延迟", f"{stats['avg_delay']:.3f}秒")
                    st.metric("最大延迟", f"{stats['max_delay']:.3f}秒")
                    st.metric("最小延迟", f"{stats['min_delay']:.3f}秒")
            
            # 质量评估
            if stats['success_rate'] >= 95:
                st.success("🟢 优秀 - 通信质量非常好")
            elif stats['success_rate'] >= 85:
                st.info("🟡 良好 - 通信质量正常")
            elif stats['success_rate'] >= 70:
                st.warning("🟠 一般 - 建议检查网络")
            else:
                st.error("🔴 较差 - 需要优化网络")
        else:
            st.info("暂无数据")
    
    with tab2:
        df = st.session_state.simulator.get_dataframe()
        if not df.empty:
            st.dataframe(df, use_container_width=True, height=400)
            
            csv = df.to_csv(index=False)
            st.download_button("📥 下载数据", csv, "heartbeat_data.csv", "text/csv")
        else:
            st.info("暂无数据")


if __name__ == "__main__":
    main()
