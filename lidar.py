#!/usr/bin/env python
import serial
import struct

def packets(ser):
  buffer = b""
  while True:
    bytes_to_read = ser.inWaiting()
    buffer += ser.read(bytes_to_read)
    print('going', buffer)
    if len(buffer) >= 22:
      idx = buffer.find(b'\0xFA')
      buffer = buffer[idx:]
      packet = buffer[:22]
      buffer = buffer[22:]
      yield packet

ser = serial.Serial('/dev/tty.usbserial', 115200, timeout=2, xonxoff=False, rtscts=False, dsrdtr=False) #Tried with and without the last 3 parameters, and also at 1Mbps, same happens.
ser.flushInput()
ser.flushOutput()
for packet in packets(ser):
  print(packet)
