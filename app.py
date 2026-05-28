#!/usr/bin/env python3
"""
无人机心跳模拟系统
模拟无人机与地面站之间的心跳包通信，支持丢包、延迟模拟和超时检测
"""

import time
import threading
import random
import argparse
import signal
import sys
from datetime import datetime
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle


class DroneHeartbeatSimulator:
    """无人机心跳模拟器类"""
    
    def __init__(self, timeout=3, loss_rate=0.1, enable_delay=True, max_delay=0.5):
        """
        初始化无人机心跳模拟器
        
        Args:
            timeout: 超时阈值（秒）
            loss_rate: 丢包率 (0-1)
            enable_delay: 是否启用延迟模拟
            max_delay: 最大延迟时间（秒）
        """
        self.timeout = timeout
        self.loss_rate = loss_rate
        self.enable_delay = enable_delay
        self.max_delay = max_delay
        self.sequence = 0
        self.last_heartbeat_time = time.time()
        self.connected = True
        self.heartbeat_data = []  # 存储所有心跳数据
        self.received_heartbeats = deque(maxlen=100)  # 存储最近的心跳记录
        self.running = True
        self.packet_loss_count = 0
        self.timeout_count = 0
        
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
        print(f"[发送] 序号: {heartbeat['seq']:4d}, 时间: {heartbeat['datetime']}")
        return heartbeat
    
    def receive_heartbeat(self, heartbeat):
        """接收心跳包"""
        current_time = time.time()
        delay = current_time - heartbeat['timestamp']
        
        heartbeat['status'] = 'RECEIVED'
        heartbeat['receive_time'] = current_time
        heartbeat['delay'] = delay
        
        self.received_heartbeats.append(heartbeat)
        self.last_heartbeat_time = current_time
        
        # 更新连接状态
        if not self.connected:
            self.connected = True
            print(f"\n[系统] ✓ 连接已恢复！")
        
        print(f"[接收] 序号: {heartbeat['seq']:4d}, 延迟: {delay:.3f}秒, 时间: {heartbeat['datetime']}")
    
    def check_timeout(self):
        """检查是否超时"""
        current_time = time.time()
        if self.connected and (current_time - self.last_heartbeat_time) > self.timeout:
            self.connected = False
            self.timeout_count += 1
            print(f"\n[告警] ✗ 连接超时！{self.timeout}秒未收到心跳包 (超时次数: {self.timeout_count})")
            # 添加超时记录
            timeout_record = {
                'seq': f'TIMEOUT_{self.timeout_count}',
                'timestamp': current_time,
                'datetime': datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                'status': 'TIMEOUT'
            }
            self.heartbeat_data.append(timeout_record)
        return self.connected
    
    def simulate_network_condition(self):
        """模拟网络条件（丢包和延迟）"""
        # 模拟丢包
        if random.random() < self.loss_rate:
            return False, 0  # 丢包
        
        # 模拟延迟
        delay = 0
        if self.enable_delay and random.random() > 0.7:  # 30%的概率产生延迟
            delay = random.uniform(0, self.max_delay)
            if delay > 0:
                time.sleep(delay)
        
        return True, delay
    
    def run(self, duration=60):
        """
        运行心跳模拟
        
        Args:
            duration: 运行时长（秒），0表示无限运行
        """
        print("=" * 70)
        print("🚁 无人机心跳模拟器启动")
        print("=" * 70)
        print(f"⏱️  超时阈值: {self.timeout}秒")
        print(f"📊 丢包率: {self.loss_rate * 100}%")
        print(f"⏰ 延迟模拟: {'启用' if self.enable_delay else '禁用'} (最大延迟: {self.max_delay}秒)")
        print(f"🕐 模拟时长: {duration if duration > 0 else '无限运行'}秒")
        print("=" * 70)
        print("\n提示: 按 Ctrl+C 提前结束模拟\n")
        
        start_time = time.time()
        last_send_time = 0
        
        try:
            while self.running and (duration == 0 or (time.time() - start_time) < duration):
                current_time = time.time()
                
                # 每秒发送一次心跳
                if current_time - last_send_time >= 1.0:
                    # 模拟网络条件
                    should_send, delay = self.simulate_network_condition()
                    
                    if should_send:
                        heartbeat = self.send_heartbeat()
                        self.receive_heartbeat(heartbeat)
                    else:
                        self.packet_loss_count += 1
                        print(f"[丢包] ✗ 第{self.sequence + 1}号心跳包丢失 (总丢包: {self.packet_loss_count})")
                        # 记录丢包事件
                        self.heartbeat_data.append({
                            'seq': self.sequence + 1,
                            'timestamp': current_time,
                            'datetime': datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                            'status': 'LOST',
                            'delay': -1
                        })
                    
                    last_send_time = current_time
                
                # 检查超时
                self.check_timeout()
                
                # 短暂休眠，避免CPU占用过高
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\n\n⚠️  收到中断信号，正在停止模拟...")
        
        print("\n" + "=" * 70)
        print("模拟结束")
        print("=" * 70)
    
    def stop(self):
        """停止模拟"""
        self.running = False
    
    def get_data_for_visualization(self):
        """获取用于可视化的数据"""
        data = []
        for item in self.heartbeat_data:
            if item['status'] == 'RECEIVED' and 'delay' in item:
                data.append({
                    'seq': item['seq'],
                    'delay': item['delay'],
                    'timestamp': item['timestamp'],
                    'datetime': item['datetime'],
                    'status': item['status']
                })
            elif item['status'] in ['LOST', 'TIMEOUT']:
                data.append({
                    'seq': item['seq'] if isinstance(item['seq'], int) else -1,
                    'delay': -1,
                    'timestamp': item['timestamp'],
                    'datetime': item['datetime'],
                    'status': item['status']
                })
        return data
    
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
                stats['avg_delay'] = sum(delays) / len(delays)
                stats['max_delay'] = max(delays)
                stats['min_delay'] = min(delays)
                stats['std_delay'] = (sum((d - stats['avg_delay'])**2 for d in delays) / len(delays))**0.5
        
        return stats


def visualize_heartbeat_data(data, timeout=3, save_figure=False):
    """
    可视化心跳数据
    
    Args:
        data: 心跳数据列表
        timeout: 超时阈值
        save_figure: 是否保存图表到文件
    """
    if not data:
        print("没有数据可供可视化")
        return
    
    # 提取数据
    sequences = []
    delays = []
    status_colors = []
    lost_sequences = []
    timeout_markers = []
    
    for item in data:
        if item['status'] == 'RECEIVED' and item['seq'] > 0:
            sequences.append(item['seq'])
            delays.append(item['delay'])
            status_colors.append('green')
        elif item['status'] == 'LOST' and item['seq'] > 0:
            lost_sequences.append(item['seq'])
            delays.append(None)
        elif item['status'] == 'TIMEOUT':
            timeout_markers.append(item)
    
    # 创建图表
    fig = plt.figure(figsize=(14, 10))
    fig.suptitle('🚁 无人机心跳监控系统', fontsize=16, fontweight='bold')
    
    # 子图1：延迟变化
    ax1 = plt.subplot(3, 1, 1)
    if sequences and delays:
        valid_indices = [i for i, d in enumerate(delays) if d is not None]
        if valid_indices:
            valid_seqs = [sequences[i] for i in valid_indices]
            valid_delays = [delays[i] for i in valid_indices]
            ax1.plot(valid_seqs, valid_delays, 'o-', color='#1f77b4', 
                    markersize=6, linewidth=1.5, alpha=0.7, label='心跳延迟')
            ax1.axhline(y=timeout, color='red', linestyle='--', linewidth=2, 
                       label=f'超时阈值 ({timeout}s)')
            
            # 添加丢包标记
            if lost_sequences:
                ax1.scatter(lost_sequences, [0]*len(lost_sequences), 
                          color='orange', s=100, marker='x', zorder=5,
                          label='丢包事件')
            
            ax1.set_xlabel('心跳序号', fontsize=11)
            ax1.set_ylabel('延迟 (秒)', fontsize=11)
            ax1.set_title('心跳包延迟变化图', fontsize=12, fontweight='bold')
            ax1.grid(True, alpha=0.3, linestyle='--')
            ax1.legend(loc='upper left', fontsize=10)
            
            # 添加统计信息框
            avg_delay = sum(valid_delays) / len(valid_delays)
            max_delay = max(valid_delays)
            min_delay = min(valid_delays)
            stats_text = f'平均延迟: {avg_delay:.3f}s | 最大延迟: {max_delay:.3f}s | 最小延迟: {min_delay:.3f}s'
            ax1.text(0.02, 0.98, stats_text, transform=ax1.transAxes, 
                    fontsize=9, verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # 子图2：连接状态
    ax2 = plt.subplot(3, 1, 2)
    status_values = []
    status_indices = []
    
    for i, item in enumerate(data):
        if item['status'] == 'RECEIVED' and item['seq'] > 0:
            status_values.append(1)
            status_indices.append(item['seq'])
        elif item['status'] == 'LOST' and item['seq'] > 0:
            status_values.append(0)
            status_indices.append(item['seq'])
    
    if status_indices:
        ax2.plot(status_indices, status_values, 'o', markersize=6, alpha=0.7)
        ax2.set_ylim(-0.5, 1.5)
        ax2.set_yticks([0, 1])
        ax2.set_yticklabels(['丢包', '正常'], fontsize=10)
        ax2.set_xlabel('心跳序号', fontsize=11)
        ax2.set_ylabel('连接状态', fontsize=11)
        ax2.set_title('连接状态变化图', fontsize=12, fontweight='bold')
        ax2.grid(True, alpha=0.3, linestyle='--')
        
        # 添加颜色区域
        ax2.axhspan(0.5, 1.5, alpha=0.2, color='green', label='正常')
        ax2.axhspan(-0.5, 0.5, alpha=0.2, color='orange', label='丢包')
        ax2.legend(loc='upper left', fontsize=10)
    
    # 子图3：超时事件时间线
    ax3 = plt.subplot(3, 1, 3)
    if timeout_markers:
        timeout_times = [item['timestamp'] for item in timeout_markers]
        timeout_nums = range(1, len(timeout_markers) + 1)
        ax3.stem(timeout_nums, [1]*len(timeout_markers), basefmt=" ", 
                linefmt='r-', markerfmt='ro', label='超时事件')
        ax3.set_xlabel('超时事件序号', fontsize=11)
        ax3.set_ylabel('事件', fontsize=11)
        ax3.set_title('超时事件记录', fontsize=12, fontweight='bold')
        ax3.set_yticks([0, 1])
        ax3.set_yticklabels(['', '超时'])
        ax3.legend(loc='upper left', fontsize=10)
        ax3.grid(True, alpha=0.3, axis='x')
        
        # 添加时间信息
        for i, (t, num) in enumerate(zip(timeout_times, timeout_nums)):
            time_str = datetime.fromtimestamp(t).strftime('%H:%M:%S')
            ax3.annotate(f'{time_str}', xy=(num, 1), xytext=(5, 5),
                        textcoords='offset points', fontsize=8, rotation=45)
    else:
        ax3.text(0.5, 0.5, '无超时事件', ha='center', va='center',
                transform=ax3.transAxes, fontsize=14, color='green')
        ax3.set_title('超时事件记录 - 无超时', fontsize=12, fontweight='bold')
    
    plt.tight_layout()
    
    if save_figure:
        plt.savefig('heartbeat_visualization.png', dpi=150, bbox_inches='tight')
        print("\n图表已保存到: heartbeat_visualization.png")
    
    plt.show()


def generate_summary_report(data, stats):
    """生成数据摘要报告"""
    print("\n" + "=" * 70)
    print("📊 数据摘要报告")
    print("=" * 70)
    
    print(f"\n📈 通信统计:")
    print(f"  总心跳包数: {stats['total_packets']}")
    print(f"  成功接收: {stats['received']} ({stats['success_rate']:.1f}%)")
    print(f"  丢包数: {stats['lost']} ({stats['lost']/stats['total_packets']*100:.1f}%)")
    print(f"  超时次数: {stats['timeout_count']}")
    
    if 'avg_delay' in stats:
        print(f"\n⏱️  延迟统计:")
        print(f"  平均延迟: {stats['avg_delay']:.3f}秒")
        print(f"  最大延迟: {stats['max_delay']:.3f}秒")
        print(f"  最小延迟: {stats['min_delay']:.3f}秒")
        print(f"  标准差: {stats['std_delay']:.3f}秒")
    
    # 连接质量评估
    print(f"\n🎯 连接质量评估:")
    if stats['success_rate'] >= 95:
        quality = "优秀"
        color = "🟢"
    elif stats['success_rate'] >= 85:
        quality = "良好"
        color = "🟡"
    elif stats['success_rate'] >= 70:
        quality = "一般"
        color = "🟠"
    else:
        quality = "较差"
        color = "🔴"
    
    print(f"  {color} {quality} (成功率: {stats['success_rate']:.1f}%)")
    
    if stats['timeout_count'] > 0:
        print(f"\n⚠️  建议: 检测到{stats['timeout_count']}次超时，请检查网络连接质量")
    
    print("=" * 70)
    
    # 保存数据到文件
    with open('heartbeat_data.txt', 'w', encoding='utf-8') as f:
        f.write("=" * 70 + "\n")
        f.write("无人机心跳数据记录\n")
        f.write("=" * 70 + "\n\n")
        
        for item in data:
            if item['status'] == 'RECEIVED':
                f.write(f"[{item['datetime']}] 序号: {item['seq']:4d} | "
                       f"延迟: {item['delay']:.3f}s | 状态: ✓ 正常\n")
            elif item['status'] == 'LOST':
                f.write(f"[{item['datetime']}] 序号: {item['seq']:4d} | "
                       f"状态: ✗ 丢包\n")
            elif item['status'] == 'TIMEOUT':
                f.write(f"[{item['datetime']}] {item['seq']} | 状态: ⚠ 超时\n")
        
        f.write("\n" + "=" * 70 + "\n")
        f.write("统计信息:\n")
        f.write(f"  总包数: {stats['total_packets']}\n")
        f.write(f"  成功率: {stats['success_rate']:.1f}%\n")
        if 'avg_delay' in stats:
            f.write(f"  平均延迟: {stats['avg_delay']:.3f}s\n")
        f.write("=" * 70 + "\n")
    
    print(f"\n💾 详细数据已保存到: heartbeat_data.txt")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description='无人机心跳模拟系统',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python drone_heartbeat.py                    # 默认配置运行60秒
  python drone_heartbeat.py -d 120             # 运行120秒
  python drone_heartbeat.py -l 0.2             # 设置20%丢包率
  python drone_heartbeat.py -t 5               # 设置5秒超时阈值
  python drone_heartbeat.py --no-delay         # 禁用延迟模拟
  python drone_heartbeat.py -d 0               # 无限运行直到手动停止
        """
    )
    
    parser.add_argument('-d', '--duration', type=int, default=60,
                       help='模拟运行时长（秒），0表示无限运行 (默认: 60)')
    parser.add_argument('-t', '--timeout', type=float, default=3.0,
                       help='超时阈值（秒） (默认: 3.0)')
    parser.add_argument('-l', '--loss-rate', type=float, default=0.1,
                       help='丢包率 (0-1) (默认: 0.1)')
    parser.add_argument('--max-delay', type=float, default=0.5,
                       help='最大延迟时间（秒） (默认: 0.5)')
    parser.add_argument('--no-delay', action='store_true',
                       help='禁用延迟模拟')
    parser.add_argument('--no-viz', action='store_true',
                       help='不显示可视化图表')
    parser.add_argument('--save-fig', action='store_true',
                       help='保存可视化图表到文件')
    
    args = parser.parse_args()
    
    # 验证参数
    if not 0 <= args.loss_rate <= 1:
        print("错误: 丢包率必须在0-1之间")
        sys.exit(1)
    
    if args.timeout <= 0:
        print("错误: 超时阈值必须大于0")
        sys.exit(1)
    
    # 创建模拟器
    simulator = DroneHeartbeatSimulator(
        timeout=args.timeout,
        loss_rate=args.loss_rate,
        enable_delay=not args.no_delay,
        max_delay=args.max_delay
    )
    
    # 运行模拟
    simulator.run(duration=args.duration)
    
    # 获取数据和统计
    data = simulator.get_data_for_visualization()
    stats = simulator.get_statistics()
    
    # 生成报告
    generate_summary_report(data, stats)
    
    # 可视化（如果需要）
    if not args.no_viz and data:
        try:
            visualize_heartbeat_data(data, timeout=args.timeout, save_figure=args.save_fig)
        except Exception as e:
            print(f"\n可视化失败: {e}")
            print("请确保已安装matplotlib: pip install matplotlib")
    
    print("\n✅ 程序执行完成")


if __name__ == "__main__":
    main()
