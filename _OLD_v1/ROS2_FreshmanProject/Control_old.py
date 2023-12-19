import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np
from struct import pack,calcsize
from time import sleep
from scipy.spatial.transform import Rotation

from .ModelData import angleToPos,arm_length
#http://wed.xjx100.cn/news/7599.html?action=onClick


class Controller(Node):
    def __init__(self):
        super().__init__("controller")
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)
        self.serialMsg_publisher = self.create_publisher(Int16MultiArray,'serial_Msg',3)
        #self.declare_parameter("target_angles",[0,0,0,0])
        self.target_angles = np.zeros(3,dtype=float)
        self.i = 0
        self.a =0
        self.s =0
        

    def timer_callback(self):
        try:
            pos =list(angleToPos(self.target_angles))
            msg =Int16MultiArray()
            msg.data = [round(a) for a in pos]

            self.serialMsg_publisher.publish(msg)
            self.get_logger().info('target: "%s"' % str(msg.data))
        except TransformException as ex:
            print(f'can not find{ex}')
        #self.target_angles = self.get_parameter("target_angles").value
        self.test1()
        #self.vertical_stabilize()

    def vertical_stabilize(self):
        now = rclpy.time.Time()
        T = self.tf_buffer.lookup_transform('world','base0',now)
        quats = [T.transform.rotation.w,T.transform.rotation.x,
                  T.transform.rotation.y,T.transform.rotation.z]
        R = Rotation.from_quat(quats)
        axangles = R.as_euler('zyx',degrees=False)
        theta1 = np.arccos(1.5*arm_length[0]/(arm_length[0]+arm_length[1]))/np.pi*180-axangles[0]/np.pi*180
        theta2 = 2*theta1
        theta3 =90-theta1
        theta4 =-axangles[1]/np.pi*180
        
        self.target_angles=np.array([theta1,theta2,theta3,theta4],dtype=float)
        #self.get_logger().info('target_angles: "%s"' % str(self.target_angles))



    def test1(self):
        if self.s == 0:
            self.a=-15
            self.s = 1
        else:
            self.a=15
            self.s = 0
        self.target_angles = np.array([0,45,0,self.a])
        #self.get_logger().info("change pose%s" %  
        #                           str(self.target_angles))
    def test2(self):
        L = arm_length
        x = np.linspace(L[0]-L[1],L[0]+L[1],10)
        #单位 度
        theta1 = np.arccos(x/(L[0]+L[1]))/np.pi*180
        theta2 = 2*theta1
        theta3 =90-theta1
        self.target_angles = np.ararry([theta1[self.i],theta2[self.i],theta3[self.i]])
        if self.i<5:
            self.get_logger().info("change pose%s" %  
                                    str(self.target_angles))
            self.i+=1
        else:
            self.i=5



def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()



