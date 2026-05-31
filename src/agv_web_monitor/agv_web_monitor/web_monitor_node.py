"""
Web监控面板节点
订阅ROS2话题，通过WebSocket推送实时数据到浏览器
"""

import json
import threading
import os
from http.server import HTTPServer, SimpleHTTPRequestHandler

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from agv_interfaces.msg import AGVStatus, FleetStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path


class WebMonitorNode(Node):
    """AGV Web监控面板"""

    def __init__(self):
        super().__init__('web_monitor')

        # 参数
        self.declare_parameter('port', 8080)
        self.declare_parameter('agv_ids', ['agv_001', 'agv_002'])
        self.declare_parameter('template_dir', '')

        self.port = self.get_parameter('port').value
        self.agv_ids = self.get_parameter('agv_ids').value

        # 数据存储
        self.agv_data = {}
        for agv_id in self.agv_ids:
            self.agv_data[agv_id] = {
                'x': 0.0, 'y': 0.0, 'theta': 0.0,
                'linear_velocity': 0.0, 'angular_velocity': 0.0,
                'battery_level': 100.0, 'status': 0,
                'current_task_id': '', 'path': []
            }

        self.fleet_info = {
            'active_tasks': 0,
            'pending_tasks': 0
        }

        # WebSocket客户端列表
        self.ws_clients = []
        self.ws_lock = threading.Lock()

        # 订阅器
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        for agv_id in self.agv_ids:
            self.create_subscription(
                AGVStatus, f'/{agv_id}/status',
                lambda msg, aid=agv_id: self.status_callback(aid, msg), qos)

            self.create_subscription(
                Path, f'/{agv_id}/planned_path',
                lambda msg, aid=agv_id: self.path_callback(aid, msg), qos)

        self.create_subscription(
            FleetStatus, '/fleet_status',
            self.fleet_callback, qos)

        # 启动HTTP服务器
        self.template_dir = self.get_parameter('template_dir').value
        if not self.template_dir:
            self.template_dir = os.path.join(
                os.path.dirname(os.path.abspath(__file__)), '..', 'templates')

        self.http_thread = threading.Thread(target=self.run_http_server, daemon=True)
        self.http_thread.start()

        # 定时广播数据
        self.broadcast_timer = self.create_timer(0.5, self.broadcast_data)

        self.get_logger().info(f'Web监控面板启动: http://localhost:{self.port}')

    def status_callback(self, agv_id, msg):
        """AGV状态回调"""
        self.agv_data[agv_id].update({
            'x': round(msg.pose.pose.pose.position.x, 3),
            'y': round(msg.pose.pose.pose.position.y, 3),
            'linear_velocity': round(msg.linear_velocity, 3),
            'angular_velocity': round(msg.angular_velocity, 3),
            'battery_level': round(msg.battery_level, 1),
            'status': msg.status,
            'current_task_id': msg.current_task_id,
        })

    def path_callback(self, agv_id, msg):
        """路径回调"""
        path_points = []
        for pose in msg.poses:
            path_points.append({
                'x': round(pose.pose.position.x, 3),
                'y': round(pose.pose.position.y, 3)
            })
        self.agv_data[agv_id]['path'] = path_points

    def fleet_callback(self, msg):
        """车队状态回调"""
        self.fleet_info['active_tasks'] = msg.active_task_count
        self.fleet_info['pending_tasks'] = msg.pending_task_count

    def broadcast_data(self):
        """广播数据到所有WebSocket客户端"""
        data = {
            'type': 'update',
            'agvs': self.agv_data,
            'fleet': self.fleet_info
        }
        message = json.dumps(data)

        with self.ws_lock:
            disconnected = []
            for ws in self.ws_clients:
                try:
                    ws.send_message(message)
                except Exception:
                    disconnected.append(ws)
            for ws in disconnected:
                self.ws_clients.remove(ws)

    def run_http_server(self):
        """运行HTTP服务器"""
        template_dir = self.template_dir

        class Handler(SimpleHTTPRequestHandler):
            def do_GET(self):
                if self.path == '/' or self.path == '/index.html':
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/html; charset=utf-8')
                    self.end_headers()
                    html_path = os.path.join(template_dir, 'dashboard.html')
                    with open(html_path, 'rb') as f:
                        self.wfile.write(f.read())
                elif self.path == '/api/data':
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.end_headers()
                    data = {
                        'agvs': self.server.node.agv_data,
                        'fleet': self.server.node.fleet_info
                    }
                    self.wfile.write(json.dumps(data).encode())
                else:
                    self.send_response(404)
                    self.end_headers()

            def log_message(self, format, *args):
                pass  # 静默HTTP日志

        server = HTTPServer(('0.0.0.0', self.port), Handler)
        server.node = self
        server.serve_forever()


def main(args=None):
    rclpy.init(args=args)
    node = WebMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
