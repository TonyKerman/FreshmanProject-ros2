import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Byte
from std_msgs.msg import Int64MultiArray, Float64MultiArray,Int16MultiArray
import serial
import serial.tools.list_ports
import queue
import threading
from time import sleep
from .ModelData import servos_bis
from struct import pack


class SerialNode(Node):

    def __init__(self):
        super().__init__('serial_node')
        self.mpuPublisher = self.create_publisher(
            Int64MultiArray, 'mpu_data', 10)
        self.servoPublisher = self.create_publisher(
            Float64MultiArray, 'servo_data', 10)
        self.serialMsg_subscriber = self.create_subscription(
            Int16MultiArray, 'serial_Msg', self.serial_callback, 3)

        # open the serial port
        is_open = False
        while(not is_open):
            try:
                # change this to your serial port and baud rate
                self.ports = list(serial.tools.list_ports.comports())
                self.serial_port = serial.Serial(self.ports[0][0], 115200,timeout=0.5)
                is_open = True
                self.get_logger().info('Serial node has been started')
            except IndexError:
                self.get_logger().fatal('No serial port foundn\n\n\n')
                sleep(1)
            except OSError:
                self.get_logger().fatal('port busy\n\n\n\n')
                sleep(1)
        # create queues to store data
        self.serMsg_queue = queue.Queue()
        # create a thread to read and write the serial
        self.read_thread = threading.Thread(target=self.serial_thread)
        self.read_thread.setDaemon(True)
        self.read_thread.start()
        self.get_logger().info('Serial node has been started')

    def serial_thread(self):
        while True:
            # check and reopen the serial port
            if not self.serial_port.is_open:
                try:
                    # change this to your serial port and baud rate
                    self.ports = list(serial.tools.list_ports.comports())
                    self.serial_port = serial.Serial(self.ports[0][0], 115200,timeout=0.1)
                    is_open = True
                    self.get_logger().info('Serial node has been started')
                except IndexError:
                    self.get_logger().fatal('No serial port foundn\n\n\n')
                    sleep(1)
                    continue
                except OSError:
                    self.get_logger().fatal('port busy\n\n\n\n')
                    sleep(1)
                    continue
            self.write_serial()     
            self.read_serial()
                
                
    # 读串口数据帧，并根据帧类型选择解码
    # 注意，发送给解码函数时已经去掉校验位
    def read_serial(self):
        byte = self.serial_port.read(1)
        # decode the byte as UTF-8
        if byte == b'\x55' and self.serial_port.read(1) == b'\x55':
            massageType = self.serial_port.read(1)
            l = self.serial_port.read(1).hex()
            length = int(l, 16)
            data = b''
            for i in range(length):
                data += (self.serial_port.read(1))
            if massageType == b'\x33':
                data = self.mDecode_MpuMsg(data)
                msg = Int64MultiArray()
                msg.data = data
                self.mpuPublisher.publish(msg)
                self.get_logger().info('mpuData: "%s"' % msg.data)
            elif massageType == b'\x11':
                data = self.mDecode_ServoMsg(data)
                msg = Float64MultiArray()
                msg.data = data
                self.servoPublisher.publish(msg)
                self.get_logger().info('ServoData: "%s"' % msg.data)
                return 0
        else:
            #self.get_logger().info("%s" % byte.decode(encoding='utf-8'))
            return -1

    def write_serial(self):
        # self.serial_port.write(b'\x55\x55\x01\x02\x03\x04')
        # self.get_logger().info("send a cmd")
        # sleep(0.5)
        if not self.serMsg_queue.empty():
            self.serial_port.write(self.serMsg_queue.get())
            self.get_logger().info("send a cmd")

    def sendmsg(self,data):
        msg=b'\x55\x55'
        msgtype=b'\x01'
        buf=b''
        for b in data:
            if b<32767:
                buf+=pack('>H',b)
        #print(buf)
        msglen=pack(">B",len(buf))
        msg+=msgtype
        msg+=msglen
        msg+=buf
        #print(msg)
        #self.get_logger().info("decode a cmd")
        return msg
    def serial_callback(self, msg):
        #self.get_logger().info(msg.data)
        self.serMsg_queue.put(self.sendmsg(msg.data))
    
    def mDecode_MpuMsg(self, rawData):
        val = []
        datalen = 4
        for i in range(0, len(rawData), datalen):
            val.append(int.from_bytes(
                rawData[i:i+datalen], byteorder='big', signed=True))
        return val

    def mDecode_ServoMsg(self, rawData):
        # l = [rawData[i:i+2] for i in range(0,len(rawData),2)]
        # for i in range(0,len(l),2):
        #     val.append(int(l[i]+l[i+1],16))
        val = []
        datalen = 2
        for i in range(0, len(rawData), datalen):
            val.append(int.from_bytes(
                rawData[i:i+datalen], byteorder='big', signed=False))
        n = 4

        def to_angle(pos, bis) -> float:
            return (pos - bis)*0.24
        angles = [to_angle(val[i], servos_bis[i]) for i in range(n)]
        #print(angles[0], angles[1], angles[2])
        return angles
    
    
        
# def publish_data(self):

    #     while not self.data_queue.empty(): # check if the queue is not empty
    #         data = self.data_queue.get() # get the data from the queue
    #         msg = String()
    #         msg.data = data
    #         self.publisher.publish(msg) # publish the data as a string message
    #         self.get_logger().info('Publishing: "%s"' % msg.data)
    #     else:
    #         pass


def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()
    while rclpy.ok():
        # spin once to handle callbacks
        rclpy.spin_once(serial_node, timeout_sec=0)
        
    serial_node.read_thread.join()
    serial_node.destroy_node()
    rclpy.shutdown()
    exit()
