#!/usr/bin/env python3
import math
import numpy as np
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from tf2_ros import Buffer, TransformListener
import threading
import websocket
import json

# --- CINEMÁTICA Y DINÁMICAS ORIGINALES ---
DH_TABLE =[
    (0.0,    0.2435,   0.0,       0.0),
    (0.0,    0.0,     -np.pi/2,   0.0),
    (0.2002, 0.0,      0.0,       0.0),
    (0.087,  0.22761, -np.pi/2,   0.0),
    (0.0,    0.0,      np.pi/2,   0.0),
    (0.0,    0.0625,  -np.pi/2,   0.0),
]

def compute_fk(q):
    T = np.eye(4)
    for i in range(6):
        a, d, alpha, offset = DH_TABLE[i]
        th = q[i] + offset
        A = np.array([
            [np.cos(th), -np.sin(th)*np.cos(alpha),  np.sin(th)*np.sin(alpha), a*np.cos(th)],
            [np.sin(th),  np.cos(th)*np.cos(alpha), -np.cos(th)*np.sin(alpha), a*np.sin(th)],
            [0.0,         np.sin(alpha),            np.cos(alpha),            d],
            [0.0,         0.0,                      0.0,                      1.0]
        ])
        T = T @ A
    return T[:3, 3]

def compute_jacobian(q, eps=1e-5):
    J = np.zeros((3, 6))
    for i in range(6):
        q_f = q.copy(); q_f[i] += eps
        q_b = q.copy(); q_b[i] -= eps
        J[:, i] = (compute_fk(q_f) - compute_fk(q_b)) / (2 * eps)
    return J

def compute_dynamics(q, qd):
    M = np.eye(6)
    m_eff =[2.5, 3.0, 1.5, 0.5, 0.3, 0.1]
    L2, L3 = 0.2002, 0.22761
    m3, m4 = 0.953, 1.284
    
    c2, c3 = np.cos(q[1]), np.cos(q[2])
    M[0, 0] = m_eff[0] + (m3 * L2**2 + m4 * (L2**2 + L3**2 + 2*L2*L3*c3)) * c2**2
    M[1, 1] = m_eff[1] + m3 * L2**2 + m4 * (L2**2 + L3**2 + 2*L2*L3*c3)
    M[2, 2] = m_eff[2] + m4 * L3**2
    for i in range(3, 6): M[i, i] = m_eff[i]
    M[1, 2] = M[2, 1] = m4 * L2 * L3 * c3
    M += 0.01 * np.eye(6)

    Cqd = np.zeros(6)
    h = m4 * L2 * L3 * np.sin(q[2])
    Cqd[1] = -h * qd[2] * (2*qd[1] + qd[2])
    Cqd[2] = h * qd[1]**2

    G = np.zeros(6)
    G[1] = -(1.166 + m3 + m4) * 9.81 * L2 * c2 - m4 * 9.81 * L3 * np.cos(q[1] + q[2])
    G[2] = -m4 * 9.81 * L3 * np.cos(q[1] + q[2])

    F = np.array([0.5, 0.5, 0.3, 0.2, 0.1, 0.05]) * qd
    return M, Cqd, G, F

class ShadowControllers:
    def __init__(self, dt):
        self.dt = dt
        self.limit = 10.0
        
        self.kp_pid = np.diag([40.0, 40.0, 40.0, 30.0, 25.0, 20.0])
        self.kd_pid = np.diag([14.0, 14.0, 14.0, 10.0, 8.0, 6.0])
        self.ki_pid = np.diag([5.0, 5.0, 5.0, 3.0, 2.0, 1.0])
        self.e_int = np.zeros(6)

        self.kp_ctc = np.diag([80.0, 80.0, 80.0, 60.0, 50.0, 40.0])
        self.kd_ctc = np.diag([28.0, 28.0, 28.0, 20.0, 16.0, 12.0])

    def compute_pid(self, q, qd, q_r, qd_r):
        e = q - q_r
        ed = qd - qd_r
        self.e_int = np.clip(self.e_int + e * self.dt, -1.5, 1.5)
        tau = -(self.kp_pid @ e + self.kd_pid @ ed + self.ki_pid @ self.e_int)
        return np.clip(tau, -self.limit, self.limit)

    def compute_ctc(self, q, qd, q_r, qd_r, qdd_r):
        e = q - q_r
        ed = qd - qd_r
        M, Cqd, G, F = compute_dynamics(q, qd)
        
        v = qdd_r - (self.kp_ctc @ e) - (self.kd_ctc @ ed)
        tau = M @ v + Cqd + G + F + 5.5 * np.tanh((2.5 * e + ed)/0.002)
        return np.clip(tau, -self.limit, self.limit)

# --- NODO ROS 2 DEL ESCLAVO ---
class SlaveControllerNode(Node):
    def __init__(self):
        super().__init__('xarm_slave_controller')
        
        self.mode = "pid" # Puedes cambiar a "ctc"
        self.hz = 50 
        self.dt = 1.0 / self.hz
        self.controllers = ShadowControllers(self.dt)
        
        # Variables de estado
        self.q_measured = np.zeros(6)
        self.qd_measured = np.zeros(6)
        self.q_ik_state = None
        
        # Variables del Maestro
        self.p_r = None
        self.prev_p_r = None
        
        self.sub_joint = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.pub_twist = self.create_publisher(TwistStamped, '/servo_server/delta_twist_cmds', 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Iniciar WebSocket Client en hilo separado
        self.ws_thread = threading.Thread(target=self.start_websocket)
        self.ws_thread.daemon = True
        self.ws_thread.start()
        
        self.timer = self.create_timer(self.dt, self.control_tick)
        self.get_logger().info("Esclavo iniciado con PID/CTC. Esperando al Maestro...")

    def joint_cb(self, msg):
        self.q_measured = np.array(msg.position[:6])
        self.qd_measured = np.array(msg.velocity[:6])
        if self.q_ik_state is None:
            self.q_ik_state = self.q_measured.copy()

    def _read_ee_pose(self):
        try:
            t = self.tf_buffer.lookup_transform("link_base", "link_eef", rclpy.time.Time())
            return np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        except Exception:
            return compute_fk(self.q_measured) 

    def start_websocket(self):
        ws_url = "ws://127.0.0.1:8765"
        ws_app = websocket.WebSocketApp(ws_url, on_message=self.on_ws_message)
        ws_app.run_forever()

    def on_ws_message(self, ws, message):
        data = json.loads(message)
        self.p_r = np.array([data['x'], data['y'], data['z']])

    def control_tick(self):
        if self.p_r is None or self.q_ik_state is None:
            return

        # 1. Calcular velocidad deseada (pd_r) del maestro en tiempo real
        if self.prev_p_r is None:
            self.prev_p_r = self.p_r.copy()
            
        pd_r = (self.p_r - self.prev_p_r) / self.dt
        self.prev_p_r = self.p_r.copy()

        # 2. IK en tiempo real para obtener q_r y qd_r (los objetivos del PID)
        W = np.diag([1.0, 1.0, 2.5])
        J_ik = compute_jacobian(self.q_ik_state)
        Jw = W @ J_ik
        J_inv = Jw.T @ np.linalg.inv(Jw @ Jw.T + 0.015**2 * np.eye(3))
        
        err_ik = self.p_r - compute_fk(self.q_ik_state)
        v_cmd_ik = pd_r + 14.0 * err_ik
        
        qd_null = (np.eye(6) - J_inv @ Jw) @ (-1.5 * self.q_ik_state)
        qd_r = J_inv @ v_cmd_ik + qd_null
        
        self.q_ik_state += qd_r * self.dt
        q_r = self.q_ik_state

        # 3. Tus Controladores Originales (calculan tau)
        qdd_r = np.zeros(6) # Aproximación para tiempo real
        if self.mode == "ctc":
            tau = self.controllers.compute_ctc(self.q_measured, self.qd_measured, q_r, qd_r, qdd_r)
        else:
            tau = self.controllers.compute_pid(self.q_measured, self.qd_measured, q_r, qd_r)

        # 4. Cálculo de Twist y publicación (IDÉNTICO a tu código original)
        p_meas = self._read_ee_pose()
        e_task = self.p_r - p_meas
        J = compute_jacobian(self.q_measured)
        
        v_cmd = 5.0 * e_task + 0.1 * (pd_r - J @ self.qd_measured) + pd_r
        v_cmd = np.clip(v_cmd, -0.15, 0.15) 
        
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "link_base"
        cmd.twist.linear.x = float(v_cmd[0])
        cmd.twist.linear.y = float(v_cmd[1])
        cmd.twist.linear.z = float(v_cmd[2])
        self.pub_twist.publish(cmd)

        # Log
        err_mm = np.linalg.norm(e_task) * 1000
        self.get_logger().info(f"Modo: {self.mode.upper()} | Error Espacial: {err_mm:.2f} mm", throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = SlaveControllerNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node()

if __name__ == '__main__':
    main()
