import streamlit as st
import time
import random
import pandas as pd
from datetime import datetime
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import numpy as np

# 设置页面配置
st.set_page_config(
    page_title="无人机心跳监控系统",
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
    .warning-text {
        color: #ff4b4b;
        font-weight: bold;
    }
    .success-text {
        color: #00cc00;
        font-weight: bold;
    }
    .info-box {
        background-color: #e3f2fd;
        padding: 15px;
        border-radius: 10px;
        border-left: 5px solid #2196f3;
    }
</style>
""", unsafe_allow_html=True)

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
        
    def send_heartbeat(self):
        """发送心跳包"""
        self.sequence += 1
        heartbeat = {
            'seq': self.sequence,
            'timestamp': time.time(),
            'datetime': datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
            'status': 'SENT'
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
        
        # 更新连接状态
        if not self.connected:
            self.connected = True
            return True  # 连接恢复
        
        return False  # 连接未变化
    
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
                'status': 'TIMEOUT',
                'delay': -1
            }
            self.heartbeat_data.append(timeout_record)
            return True  # 发生超时
        return False  # 无超时
    
    def simulate_network_condition(self):
        """模拟网络条件"""
        # 模拟丢包
        if random.random() < self.loss_rate:
            return False, 0
        
        # 模拟延迟
        delay = 0
        if self.enable_delay and random.random() > 0.7:
            delay = random.uniform(0, self.max_delay)
            if delay > 0:
                time.sleep(delay)
        
        return True, delay
    
    def step(self):
        """执行一步模拟"""
        if not self.running:
            return None
        
        current_time = time.time()
        
        # 每秒发送一次心跳
        if current_time - self.last_update_time >= 1.0:
            self.last_update_time = current_time
            should_send, delay = self.simulate_network_condition()
            
            if should_send:
                heartbeat = self.send_heartbeat()
                connection_restored = self.receive_heartbeat(heartbeat)
                return {
                    'type': 'heartbeat',
                    'seq': heartbeat['seq'],
                    'delay': heartbeat.get('delay', 0),
                    'datetime': heartbeat['datetime'],
                    'connection_restored': connection_restored
                }
            else:
                self.packet_loss_count += 1
                lost_record = {
                    'seq': self.sequence + 1,
                    'timestamp': current_time,
                    'datetime': datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                    'status': 'LOST',
                    'delay': -1
                }
                self.heartbeat_data.append(lost_record)
                return {
                    'type': 'loss',
                    'seq': self.sequence + 1,
                    'datetime': lost_record['datetime']
                }
        
        # 检查超时
        if self.check_timeout():
            return {
                'type': 'timeout',
                'count': self.timeout_count,
                'datetime': datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            }
        
        return None
    
    def start(self):
        """开始模拟"""
        self.running = True
        self.start_time = time.time()
        self.last_heartbeat_time = time.time()
        self.last_update_time = time.time()
        
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
                stats['std_delay'] = float(np.std(delays))
        
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
                    '状态': '✓ 正常',
                    '状态码': 1
                })
            elif item['status'] == 'LOST':
                df_data.append({
                    '序号': item['seq'],
                    '时间': item['datetime'],
                    '延迟(秒)': '-',
                    '状态': '✗ 丢包',
                    '状态码': 0
                })
            elif item['status'] == 'TIMEOUT':
                df_data.append({
                    '序号': item['seq'],
                    '时间': item['datetime'],
                    '延迟(秒)': '-',
                    '状态': '⚠ 超时',
                    '状态码': -1
                })
        return pd.DataFrame(df_data)


def create_visualization(data, timeout):
    """使用Plotly创建可视化图表"""
    if not data:
        return None
    
    # 准备数据
    sequences = []
    delays = []
    lost_seqs = []
    timeout_events = []
    
    for item in data:
        if item['status'] == 'RECEIVED' and isinstance(item.get('seq'), int):
            sequences.append(item['seq'])
            delays.append(item['delay'])
        elif item['status'] == 'LOST' and isinstance(item.get('seq'), int):
            lost_seqs.append(item['seq'])
        elif item['status'] == 'TIMEOUT':
            timeout_events.append(item)
    
    if not sequences:
        return None
    
    # 创建子图
    fig = make_subplots(
        rows=2, cols=1,
        subplot_titles=('📈 心跳包延迟变化', '🔗 连接状态监控'),
        vertical_spacing=0.12,
        row_heights=[0.6, 0.4]
    )
    
    # 1. 延迟图
    fig.add_trace(
        go.Scatter(
            x=sequences,
            y=delays,
            mode='lines+markers',
            name='心跳延迟',
            line=dict(color='#1f77b4', width=2),
            marker=dict(size=8, color='#1f77b4', symbol='circle'),
            hovertemplate='序号: %{x}<br>延迟: %{y:.3f}秒<extra></extra>'
        ),
        row=1, col=1
    )
    
    # 添加超时阈值线
    fig.add_hline(
        y=timeout, 
        line_dash="dash", 
        line_color="red",
        line_width=2,
        annotation_text=f"⚠ 超时阈值 ({timeout}秒)",
        annotation_position="top right",
        row=1, col=1
    )
    
    # 添加丢包标记
    if lost_seqs:
        fig.add_trace(
            go.Scatter(
                x=lost_seqs,
                y=[max(delays) * 0.9] * len(lost_seqs),
                mode='markers',
                name='丢包事件',
                marker=dict(size=12, color='orange', symbol='x', line=dict(width=2)),
                hovertemplate='序号: %{x}<br>状态: 丢包<extra></extra>'
            ),
            row=1, col=1
        )
    
    # 2. 连接状态图
    status_data = []
    for item in data:
        if item['status'] == 'RECEIVED' and isinstance(item.get('seq'), int):
            status_data.append((item['seq'], 1, '正常'))
        elif item['status'] == 'LOST' and isinstance(item.get('seq'), int):
            status_data.append((item['seq'], 0, '丢包'))
    
    if status_data:
        status_seqs, status_vals, status_texts = zip(*status_data)
        colors = ['#00cc00' if v == 1 else '#ffa500' for v in status_vals]
        
        fig.add_trace(
            go.Scatter(
                x=status_seqs,
                y=status_vals,
                mode='markers',
                name='连接状态',
                marker=dict(size=10, color=colors, symbol='circle', line=dict(width=1)),
                text=status_texts,
                hovertemplate='序号: %{x}<br>状态: %{text}<extra></extra>'
            ),
            row=2, col=1
        )
    
    # 添加超时事件标记
    if timeout_events:
        timeout_y = [1.2] * len(timeout_events)
        fig.add_trace(
            go.Scatter(
                x=list(range(1, len(timeout_events) + 1)),
                y=timeout_y,
                mode='markers',
                name='超时事件',
                marker=dict(size=15, color='red', symbol='triangle-down'),
                text=[f"超时 #{i+1}" for i in range(len(timeout_events))],
                hovertemplate='%{text}<br>时间: %{customdata}<extra></extra>',
                customdata=[item['datetime'] for item in timeout_events]
            ),
            row=2, col=1
        )
    
    # 更新布局
    fig.update_xaxes(title_text="心跳序号", row=1, col=1, showgrid=True, gridwidth=1, gridcolor='lightgray')
    fig.update_yaxes(title_text="延迟 (秒)", row=1, col=1, showgrid=True, gridwidth=1, gridcolor='lightgray')
    fig.update_xaxes(title_text="心跳序号", row=2, col=1, showgrid=True, gridwidth=1, gridcolor='lightgray')
    fig.update_yaxes(
        title_text="状态", 
        ticktext=['丢包', '正常'], 
        tickvals=[0, 1],
        range=[-0.2, 1.4],
        row=2, col=1
    )
    
    fig.update_layout(
        height=600,
        showlegend=True,
        legend=dict(
            yanchor="top",
            y=0.99,
            xanchor="left",
            x=0.01,
            bgcolor="rgba(255, 255, 255, 0.8)",
            bordercolor="black",
            borderwidth=1
        ),
        hovermode='closest',
        plot_bgcolor='white',
        paper_bgcolor='white'
    )
    
    return fig


def main():
    st.title("🚁 无人机心跳监控系统")
    st.markdown("实时监控无人机与地面站的心跳通信状态")
    st.markdown("---")
    
    # 侧边栏配置
    with st.sidebar:
        st.header("⚙️ 系统配置")
        
        timeout = st.slider("⏱️ 超时阈值 (秒)", 1.0, 10.0, 3.0, 0.5, 
                           help="超过此时间未收到心跳包将触发超时告警")
        
        loss_rate = st.slider("📊 丢包率 (%)", 0, 50, 10, 
                             help="模拟网络丢包的概率")
        
        enable_delay = st.checkbox("⏰ 启用延迟模拟", value=True,
                                   help="模拟网络传输延迟")
        
        if enable_delay:
            max_delay = st.slider("最大延迟 (秒)", 0.1, 2.0, 0.5, 0.1,
                                 help="模拟的最大网络延迟时间")
        else:
            max_delay = 0.5
        
        st.markdown("---")
        
        duration = st.number_input("🕐 运行时长 (秒)", min_value=5, max_value=300, value=60, step=5,
                                  help="模拟运行的总时长")
        
        st.markdown("---")
        
        # 控制按钮
        col1, col2 = st.columns(2)
        
        with col1:
            if st.button("▶️ 开始", use_container_width=True):
                if 'simulator' in st.session_state:
                    st.session_state.simulator.reset(timeout, loss_rate/100, enable_delay, max_delay)
                else:
                    st.session_state.simulator = DroneHeartbeatSimulator(
                        timeout=timeout,
                        loss_rate=loss_rate/100,
                        enable_delay=enable_delay,
                        max_delay=max_delay
                    )
                st.session_state.simulator.start()
                st.session_state.simulator_running = True
                st.session_state.simulation_log = []
                st.session_state.simulation_start_time = time.time()
                st.rerun()
        
        with col2:
            if st.button("⏹️ 停止", use_container_width=True):
                if 'simulator' in st.session_state:
                    st.session_state.simulator.stop()
                st.session_state.simulator_running = False
                st.rerun()
        
        st.markdown("---")
        
        if st.button("🔄 重置系统", use_container_width=True):
            if 'simulator' in st.session_state:
                st.session_state.simulator.reset(timeout, loss_rate/100, enable_delay, max_delay)
            st.session_state.simulator_running = False
            st.session_state.simulation_log = []
            st.rerun()
        
        st.markdown("---")
        st.info("💡 **使用说明**\n\n"
                "1. 配置系统参数\n"
                "2. 点击'开始'按钮\n"
                "3. 实时查看监控数据\n"
                "4. 可随时停止或重置")
    
    # 初始化session state
    if 'simulator' not in st.session_state:
        st.session_state.simulator = DroneHeartbeatSimulator(
            timeout=timeout,
            loss_rate=loss_rate/100,
            enable_delay=enable_delay,
            max_delay=max_delay
        )
        st.session_state.simulator_running = False
        st.session_state.simulation_log = []
    
    # 显示实时状态
    if st.session_state.simulator_running:
        # 检查是否超时
        current_time = time.time()
        elapsed_time = current_time - st.session_state.simulation_start_time
        
        if elapsed_time > duration:
            st.session_state.simulator.stop()
            st.session_state.simulator_running = False
            st.success(f"✅ 模拟完成！运行时长 {duration} 秒")
        else:
            # 显示进度条
            progress = elapsed_time / duration
            st.progress(progress, text=f"模拟进度: {elapsed_time:.0f}/{duration} 秒")
            
            # 执行一步模拟
            result = st.session_state.simulator.step()
            
            if result:
                if result['type'] == 'heartbeat':
                    if result.get('connection_restored'):
                        log_msg = f"🟢 连接恢复 - 心跳 {result['seq']}: 延迟 {result['delay']:.3f}s"
                    else:
                        log_msg = f"✅ 心跳 {result['seq']}: 延迟 {result['delay']:.3f}s"
                    st.session_state.simulation_log.insert(0, f"[{result['datetime']}] {log_msg}")
                elif result['type'] == 'loss':
                    log_msg = f"❌ 丢包 - 第{result['seq']}号心跳丢失"
                    st.session_state.simulation_log.insert(0, f"[{result['datetime']}] {log_msg}")
                elif result['type'] == 'timeout':
                    log_msg = f"⚠️ 超时告警 - 第{result['count']}次连接超时"
                    st.session_state.simulation_log.insert(0, f"[{result['datetime']}] {log_msg}")
            
            # 限制日志长度
            st.session_state.simulation_log = st.session_state.simulation_log[:100]
            
            # 自动刷新
            time.sleep(0.05)
            st.rerun()
    
    # 显示状态指标
    st.markdown("### 📊 实时监控指标")
    
    col1, col2, col3, col4, col5 = st.columns(5)
    
    with col1:
        if st.session_state.simulator.connected:
            st.metric("🔗 连接状态", "正常", delta="在线", delta_color="normal")
        else:
            st.metric("🔗 连接状态", "超时", delta="离线", delta_color="inverse")
    
    with col2:
        st.metric("📨 心跳序号", st.session_state.simulator.sequence)
    
    with col3:
        st.metric("📤 丢包数量", st.session_state.simulator.packet_loss_count)
    
    with col4:
        st.metric("⚠️ 超时次数", st.session_state.simulator.timeout_count)
    
    with col5:
        if st.session_state.simulator.sequence > 0:
            success_rate = (st.session_state.simulator.sequence - st.session_state.simulator.packet_loss_count) / st.session_state.simulator.sequence * 100
            st.metric("📈 成功率", f"{success_rate:.1f}%")
        else:
            st.metric("📈 成功率", "0%")
    
    # 日志区域
    st.markdown("### 📝 实时事件日志")
    log_container = st.container()
    
    with log_container:
        if st.session_state.simulation_log:
            log_text = "\n".join(st.session_state.simulation_log[:20])
            st.code(log_text, language="log")
        else:
            st.info("等待模拟开始...")
    
    # 数据显示区域
    st.markdown("---")
    tab1, tab2, tab3 = st.tabs(["📈 可视化分析", "📊 统计报告", "📋 数据表格"])
    
    with tab1:
        data = st.session_state.simulator.heartbeat_data
        if data:
            fig = create_visualization(data, timeout)
            if fig:
                st.plotly_chart(fig, use_container_width=True)
            else:
                st.warning("数据不足，无法生成图表")
        else:
            st.info("暂无数据，请开始模拟")
    
    with tab2:
        if st.session_state.simulator.sequence > 0:
            stats = st.session_state.simulator.get_statistics()
            
            col1, col2 = st.columns(2)
            
            with col1:
                st.markdown("#### 📈 通信统计")
                st.metric("总心跳包数", stats['total_packets'])
                st.metric("成功接收", f"{stats['received']} ({stats['success_rate']:.1f}%)")
                st.metric("丢包数", f"{stats['lost']} ({stats['lost']/stats['total_packets']*100:.1f}%)")
                st.metric("超时次数", stats['timeout_count'])
            
            with col2:
                if 'avg_delay' in stats:
                    st.markdown("#### ⏱️ 延迟统计")
                    st.metric("平均延迟", f"{stats['avg_delay']:.3f} 秒")
                    st.metric("最大延迟", f"{stats['max_delay']:.3f} 秒")
                    st.metric("最小延迟", f"{stats['min_delay']:.3f} 秒")
                    st.metric("延迟标准差", f"{stats['std_delay']:.3f} 秒")
            
            # 质量评估
            st.markdown("#### 🎯 连接质量评估")
            quality_col1, quality_col2 = st.columns([1, 3])
            
            with quality_col1:
                if stats['success_rate'] >= 95:
                    st.markdown("# 🟢")
                    quality = "优秀"
                    color = "success"
                elif stats['success_rate'] >= 85:
                    st.markdown("# 🟡")
                    quality = "良好"
                    color = "info"
                elif stats['success_rate'] >= 70:
                    st.markdown("# 🟠")
                    quality = "一般"
                    color = "warning"
                else:
                    st.markdown("# 🔴")
                    quality = "较差"
                    color = "error"
            
            with quality_col2:
                if color == "success":
                    st.success(f"**{quality}** - 通信质量非常好，系统运行稳定")
                elif color == "info":
                    st.info(f"**{quality}** - 通信质量正常，可以接受")
                elif color == "warning":
                    st.warning(f"**{quality}** - 通信质量一般，建议检查网络")
                else:
                    st.error(f"**{quality}** - 通信质量较差，需要优化网络连接")
        else:
            st.info("暂无数据，请开始模拟")
    
    with tab3:
        df = st.session_state.simulator.get_dataframe()
        if not df.empty:
            st.dataframe(df, use_container_width=True, height=400)
            
            # 导出功能
            col1, col2 = st.columns(2)
            with col1:
                csv = df.to_csv(index=False)
                st.download_button(
                    label="📥 下载 CSV 文件",
                    data=csv,
                    file_name=f"heartbeat_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
                    mime="text/csv",
                    use_container_width=True
                )
            
            with col2:
                # 统计摘要
                st.markdown(f"**数据摘要:** 共 {len(df)} 条记录")
                st.markdown(f"**正常:** {len(df[df['状态'] == '✓ 正常'])} 条")
                st.markdown(f"**丢包:** {len(df[df['状态'] == '✗ 丢包'])} 条")
                st.markdown(f"**超时:** {len(df[df['状态'] == '⚠ 超时'])} 条")
        else:
            st.info("暂无数据，请开始模拟")


if __name__ == "__main__":
    main()
