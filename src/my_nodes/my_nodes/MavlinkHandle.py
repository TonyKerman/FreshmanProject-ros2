import rclpy
from rclpy.node import Node
from pymavlink import mavutil
import serial.tools.list_ports
import numpy as np
from typing import Iterable,Any

class MavlinkHandler(Node):
    def __init__(self):
        pass