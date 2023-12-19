#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# 1.导入消息类型JointState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
import threading,time,serial,struct
import numpy as np
from queue import Queue
from pymavlink import mavutil
import serial.tools.list_ports
from typing import Iterable,Any
from scipy.spatial.transform import Rotation

class Servogroup():
    def __init__(self,servo_num:int=4) -> None:

        offset = np.array((775, 1155, 400, 700),dtype=int)
        reverse = np.array((-1,1,1,1),dtype=int)

        self.mutex = threading.Lock()
        self.servo_num =servo_num
        self.offset = offset[:self.servo_num]
        self.reverse = reverse[:self.servo_num]
        self.position = np.zeros(self.servo_num,dtype=int) 
    
    def updatePos(self,pos:Iterable,angle = False) -> None:
        pos = np.array(pos)
        if pos.shape != (self.servo_num,):
            raise IndexError()
        with self.mutex:
            if angle:
                self.position =np.array(pos/0.24+self.offset,dtype=int)
            else:
                self.position = pos
    
    def getPos(self):
        with self.mutex:
            return self.position
    def getPos_r(self):
        with self.mutex:
            return (self.position-self.offset)*0.24*np.pi/180
            

class Mpu():
    
    def __init__(self,dt:int=0,orientation:np.ndarray=np.eye(3)) -> None:
        
        self.R0 = Rotation.from_matrix(orientation)
        self.R = None
        self.mutex = threading.Lock()
        
    def get_quat(self) -> np.ndarray:
        try:
            with self.mutex:
                return self.R.as_quat()
        except AttributeError:
            print('self.R is None')

    def get_euler(self,seq:str) -> np.ndarray:
        try:
            with self.mutex:
                return self.R.as_euler(seq)
        except AttributeError:
            print('self.R is None')
            

    def update_from_quat(self,quat:Iterable) -> None:
        quaternion = np.array(quat,dtype=np.float64)
        with self.mutex:
            self.R = self.R0 * Rotation.from_quat(quaternion)
            #self.get_logger().info(f'1___{str(self.get_quat())})')
        

class ControlArmNode(Node):
    def __init__(self,name):
        super().__init__(name)

        self.joint_states_publisher_ = self.create_publisher(JointState,"joint_states", 10)
        self.tf_publisher = TransformBroadcaster(self)
        self._static_tf_pub('mpu_base','base_link',Rotation.from_euler('zyx',(np.pi/4,np.pi/2,0)))
        self.joint_states = JointState()
        self.servos = Servogroup(4)
        self.mpu = Mpu()
        

        self._joint_state_msg_init()         
        if self._serial_init():
            exit()

        
        self.pub_rate = self.create_rate(10)
        self.thread_pub = threading.Thread(target=self._thread_pub)
        self.thread_serial = threading.Thread(target=self._thread_serial)
        self.thread_pub.setDaemon(True)
        self.thread_serial.setDaemon(True)
        self.thread_serial.start()
        self.thread_pub.start()

    def _thread_serial(self):
        
        while rclpy.ok():
            msg = self.mav_connection.recv_match(blocking = True)
            msg_type = msg.get_type()
            if msg_type == 'BAD_DATA':
                continue
            elif msg_type == 'ATTITUDE_QUATERNION':
                
                #self.get_logger().info(f'{quat},{type(quat)}')
                self.mpu.update_from_quat((msg.q1,msg.q2,msg.q3,msg.q4))
            elif msg_type == 'COMMAND_LONG':
                self.servos.updatePos((msg.param1,msg.param2,msg.param3,msg.param4))
            #self.get_logger().info(f'got {msg.get_type()}')

        
    def _thread_pub(self):
        
        while rclpy.ok():
        
            # delta_time =  time.time()-last_update_time
            # last_update_time = time.time()
            self.joint_states.header.stamp = self.get_clock().now().to_msg()
            #self.get_logger().info(f'{pos}')
            for i,pos in enumerate(self.servos.getPos_r()):
                self.joint_states.position[i] = 0
            
            #self.get_logger().info(f"2___{self.mpu.get_euler('xyz')}")
            
            if (angles :=self.mpu.get_euler('xyz')) is not None:
                self.get_logger().info(f'3___{angles}')
                yaw,pitch,row = angles
                
                r  = Rotation.from_euler('xyz',(-row,pitch,0))
                self.tf_publisher.sendTransform(self.quat_to_TF(
                    'world', 'mpu_base', 0, 0, 0, r.as_quat()))
            
            self.joint_states_publisher_.publish(self.joint_states)
            self.get_logger().info('send{str(self.joint_states.position)}')
        
            self.pub_rate.sleep()
            
    def quat_to_TF(self, A: str, B: str, tx: float, ty: float, tz: float, q: list):
        if len(q) != 4:
            raise IndexError('Quaternion must be a list of 4 elements')

        T = TransformStamped()
        T.header.stamp = self.get_clock().now().to_msg()
        T.header.frame_id = A
        T.child_frame_id = B
        T.transform.translation.x = float(tx)
        T.transform.translation.y = float(ty)
        T.transform.translation.z = float(tz)
        T.transform.rotation.x = q[0]
        T.transform.rotation.y = q[1]
        T.transform.rotation.z = q[2]
        T.transform.rotation.w = q[3]
        return T
    
    def _joint_state_msg_init(self) -> None:
        self.get_logger().info('publisher start')
        self.joint_states.name = ['joint1','joint2','joint3','joint4','joint5','joint6']
        self.joint_states.position = np.zeros(6,dtype=float).tolist()
        self.joint_states.velocity = []
        #self.joint_states.name = ['joint1']
        #self.joint_states.position = [10.0]
        #self.joint_states.velocity = [0.0]
        self.joint_states.effort = []
        self.joint_states.header.frame_id = ""
    
    def _static_tf_pub(self,A:str='world1',B:str='world2',R:Rotation=Rotation.from_matrix(np.eye(3))):
        static_tf_publisher = StaticTransformBroadcaster(self)
        static_tf_publisher.sendTransform(
            self.quat_to_TF(A,B, 0, 0, 0, R.as_quat()))

    def _serial_init(self) -> bool:
        open_cnt = 0
        while open_cnt<30:
            try:
                ports = list(serial.tools.list_ports.comports())
                self.mav_connection = mavutil.mavlink_connection(ports[0][0],baud=115200,dialect='common') 
                self.get_logger().info(f'serial success in {ports[0][0]}!')
                break
            except IndexError:
                self.get_logger().fatal(f'No device found !\n\n\n')
                time.sleep(1)
                open_cnt +=1
        return bool(open_cnt)
# class RotateWheelNode(Node):
#     def __init__(self,name):
#         super().__init__(name)
#         self.get_logger().info(f"node {name} init..")
#         # 创建并初始化发布者成员属性pub_joint_states_
#         self.joint_states_publisher_ = self.create_publisher(JointState,"joint_states", 10) 
#         # 初始化数据
#         self._init_joint_states()
#         self.pub_rate = self.create_rate(30)
#         self.thread_ = threading.Thread(target=self._thread_pub)
#         self.thread_.start()

    
#     def _init_joint_states(self):
#         # 初始左右轮子的速度
#         self.joint_speeds = [0.0,0.0]
#         self.joint_states = JointState()
#         self.joint_states.header.stamp = self.get_clock().now().to_msg()
#         self.joint_states.header.frame_id = ""
#         # 关节名称
#         self.joint_states.name = ['left_wheel_joint','right_wheel_joint']
#         # 关节的位置
#         self.joint_states.position = [0.0,0.0]
#         # 关节速度
#         self.joint_states.velocity = self.joint_speeds
#         # 力 
#         self.joint_states.effort = []

#     def update_speed(self,speeds):
#         self.joint_speeds = speeds

#     def _thread_pub(self):
#         last_update_time = time.time()
#         while rclpy.ok():
#             delta_time =  time.time()-last_update_time
#             last_update_time = time.time()
#             # 更新位置
#             self.joint_states.position[0]  += delta_time*self.joint_states.velocity[0]
#             self.joint_states.position[1]  += delta_time*self.joint_states.velocity[1]
#             # 更新速度
#             self.joint_states.velocity = self.joint_speeds
#             # 更新 header
#             self.joint_states.header.stamp = self.get_clock().now().to_msg()
#             # 发布关节数据
#             self.joint_states_publisher_.publish(self.joint_states)
#             self.pub_rate.sleep()

def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = ControlArmNode("control_node")  # 新建一个节点
    
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy
