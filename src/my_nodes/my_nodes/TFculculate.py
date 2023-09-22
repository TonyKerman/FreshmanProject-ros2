
import sys
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64MultiArray, Float64MultiArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
import numpy as np
import scipy
from scipy.spatial.transform import Rotation

from .ModelData import arm_length

# DH变换


def DH_transform(alpha, a, d, theta):
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    t = np.array([
        [ct, -st, 0, a],
        [st*ca, ct*ca, -sa, -sa*d],
        [st*sa, ct*sa, ca, ca*d],
        [0, 0, 0, 1]
    ])
    return t


class TF_Publisher(Node):
    def __init__(self):
        super().__init__('tf_publisher')
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.timer2 = self.create_timer(3, self.timer2_callback)
        self.mpu_subscriber = self.create_subscription(
            Int64MultiArray, 'mpu_data', self.mpu_callback, 10)
        self.servo_subscriber = self.create_subscription(
            Float64MultiArray, 'servo_data', self.servo_callback, 10)
        self.tf_publisher = TransformBroadcaster(self)
        static_tf_publisher = StaticTransformBroadcaster(self)
        self.mpu_q = np.array([0, 0, 0, 1], dtype=float)
        self.servoAngles = [0, 0, 0, 0]

        r = Rotation.from_euler('y', -np.pi/4)
        static_tf_publisher.sendTransform(
            self.quat_to_TF('base0', 'base1', 0, 0, 0, r.as_quat()))
        self.servoAngles = [0, 0, 0, 0]

    def timer_callback(self):
        self.tf_publisher.sendTransform(self.quat_to_TF(
            'world', 'base0', 0, 0, 0, self.mpu_q))
        self.tf_publisher.sendTransform(self.DH_to_TF(
            'base1', 'S1', 0, 0, 0, (self.servoAngles[0])/180*np.pi))
        self.tf_publisher.sendTransform(self.DH_to_TF(
            'S1', 'S2', 0, 0.17, 0, -(self.servoAngles[1])/180*np.pi))
        self.tf_publisher.sendTransform(self.DH_to_TF(
            'S2', 'S3A', np.pi, 0.14, 0, -(self.servoAngles[2])/180*np.pi))
        self.tf_publisher.sendTransform(
            self.DH_to_TF('S3A', 'S3B', 0, 0, 0, np.pi/2))
        self.tf_publisher.sendTransform(self.DH_to_TF(
            'S3B', 'S4', np.pi/2, 0.03, 0, (self.servoAngles[3])/180*np.pi))
        #self.get_logger().info('Publishing Transform')

    def timer2_callback(self):
        self.get_logger().info('Publishing Transform')

    def mpu_callback(self, msg):
        # 当串口输出xy角度弧度时
        ruler = np.array([0, -msg.data[0]/65535, msg.data[1]/65535])
        R = Rotation.from_euler('zyx', ruler, degrees=False)
        self.mpu_q = R.as_quat()
        # 当串口是四元数时
        # def mpu_decode(quats):
        #     q30 = float(1073741824.0)
        #     # q0  = quats[0]/q30
        #     # q1  = quats[1]/q30
        #     # q2  = quats[2]/q30
        #     # q3  = quats[3]/q30
        #     q0  = quats[0]
        #     q1  = quats[1]
        #     q2  = quats[2]
        #     q3  = quats[3]
        #     angle = np.array([0,0,0])
        #     angle[0] = np.arcsin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3   # pitch
        #     angle[1]= np.arctan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;    # roll
        #     angle[2]= np.arctan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3;    #yaw
        #     return angle
        # self.mpu_q = msg.data

    def servo_callback(self, msg):
        self.servoAngles = msg.data
        pass

    def DH_to_TF(self, A: str, B: str, alpha, a, d, theta):
        m = DH_transform(alpha, a, d, theta)
        T = TransformStamped()
        T.header.stamp = self.get_clock().now().to_msg()
        T.header.frame_id = A
        T.child_frame_id = B
        T.transform.translation.x = m[0][3]
        T.transform.translation.y = m[1][3]
        T.transform.translation.z = m[2][3]
        r = Rotation.from_matrix(m[:3, :3])
        q = r.as_quat()
        T.transform.rotation.w = q[3]
        T.transform.rotation.x = q[0]
        T.transform.rotation.y = q[1]
        T.transform.rotation.z = q[2]
        # print(T.transform.rotation.x,'   ',T.transform.rotation.y,'   ',T.transform.rotation.z,'   ',T.transform.rotation.w)
        return T

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


pose = ['waiting', 'aiming']
# def control():
#     state = pose[0]
#     if state == 'waiting':
#         angles = [-90,170,0,0]
#     if state == 'aiming':
#         #solve
#         # h/cos(a0)=l1*cos(a1)+l2*sin(-a2+a1-pi/2),
#         # l1*sin(a1)=x+l2*cos(-a2+a1-pi/2),
#         # a3=a1-a2-pi/2,a0=pi/6
#         #  for a1,a2,a3
#         a1 = 0
#         a2 = 0
#         pass


def main(args=None):
    rclpy.init(args=args)
    tf_publisher = TF_Publisher()
    rclpy.spin(tf_publisher)
    tf_publisher.destroy_node()
    rclpy.shutdown()


# 平移变换
# def translation(qx, qy, qz):
#     Dq = np.array([[1, 0, 0, qx],
#                    [0, 1, 0, qy],
#                    [0, 0, 1, qz],
#                    [0, 0, 0, 1]], dtype=float)
#     return Dq


# # 旋转变换npa
# def rotation(cls, angles):
#     r = Rotation.from_euler(cls, angles, degrees=False)
#     R = r.as_matrix()
#     R = np.vstack((R, np.array([0, 0, 0])))
#     R = np.hstack((R, np.array([[0], [0], [0], [1]])))
#     return R
# #一般变换(有问题，只能表示180度，所以使用scipy中Rotation代替)
# def transform(Dm,Rm):
#     if(Dm.shape !=(4,4) or Rm.shape!=(3,3)):
#         raise IndexError("Wrong Matrix size!")
#     temp = np.append(Rm,np.reshape(Dm[:3,3],(3,1)),axis=1)
#     temp = np.append(temp,[[0.,0.,0.,1.]],axis=0)
#     return temp
# def TFtransform_from_Matrix(timestamp,A:str,B:str,m:np.ndarray):
#     if m.shape != (4, 4):
#         raise IndexError('The matrix shape is not 4x4')
#     T = TransformStamped()
#     T.header.stamp = timestamp
#     T.header.frame_id = A
#     T.child_frame_id = B
#     T.transform.translation.x = m[0][3]
#     T.transform.translation.y = m[1][3]
#     T.transform.translation.z = m[2][3]
#     w = 0.5*np.sqrt(1+np.square(m[0][0])+np.square(m[1][1])+np.square(m[2][2]))
#     T.transform.rotation.w = w
#     T.transform.rotation.x = (m[2][1]-m[1][2])/(4*w)
#     T.transform.rotation.y = (m[0][2]-m[2][0])/(4*w)
#     T.transform.rotation.z = (m[1][0]-m[0][1])/(4*w)
#     #print(T.transform.rotation.x,'   ',T.transform.rotation.y,'   ',T.transform.rotation.z,'   ',T.transform.rotation.w)
#     return T
