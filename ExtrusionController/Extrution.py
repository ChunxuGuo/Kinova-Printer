# -*- coding: utf-8 -*-
import serial
import time

serialPort = "COM5"
baudRate = 9600
ser = serial.Serial(serialPort, baudRate, timeout=0.5)
print("Port=%s ，Baudrate=%d" % (serialPort, baudRate))

# demo1=b"0"#将0转换为ASCII码方便发送
# demo2=b"1"#同理
while 1:
    ser.write(b"t")
    time.sleep(3)
