from scipy.spatial.transform import Rotation
import threading
from typing import Iterable,Any
import numpy as np
from sensor_msgs.msg import Imu

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

    def get_euler(self) -> np.ndarray:
        try:
            with self.mutex:
                return self.R.as_euler('xyz')
        except AttributeError:
            print('self.R is None')
            

    def update_from_quat(self,quat:Iterable) -> None:
        quaternion = np.array(quat,dtype=np.float64)
        with self.mutex:
            self.R = self.R0 * Rotation.from_quat(quaternion)
            #self.get_logger().info(f'1___{str(self.get_quat())})')
        
