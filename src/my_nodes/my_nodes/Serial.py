import rclpy
from rclpy.node import Node
from my_interfaces.msg import input_msg, output_msg

import serial
import serial.tools.list_ports
import queue
import threading
from time import sleep
from .....ROS2_FreshmanProject.ROS2_FreshmanProject.ModelData import servos_bis
from struct import pack