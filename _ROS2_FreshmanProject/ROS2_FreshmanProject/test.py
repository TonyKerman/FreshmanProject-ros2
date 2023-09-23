import serial
import serial.tools.list_ports
from time import sleep

ports = list(serial.tools.list_ports.comports())
serial_port = serial.Serial(ports[0][0], 115200,timeout=0.01)
count=0
while(True):
    sleep(0.1)
    count+=1
    serial_port.write(b'\x55\x55\x01\x04\x11\x22\x33\x44')
    print('read:',serial_port.read(1),' ',count)
    serial_port.reset_input_buffer()