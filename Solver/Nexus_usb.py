#!/usr/bin/python3

# Program to implement the interface between a Nexus DSC Pro and
# a Nexus eFinder (electronic finder)
# Copyright (C) 2026 Keith Venables.
# 13 March 2026

import serial
import time
import socket
import subprocess

class Nexus:
    def __init__(self,auxD,baudR):
        self.auxDevice = auxD
        self.baudRate = baudR
        try:
            self.ser = serial.Serial(self.auxDevice,baudrate=self.baudRate, xonxoff=True, write_timeout=1)
            time.sleep(0.1)
            self.ser.write(b':ID=eFinderLite#')
            print ("Connected to Nexus DSC via USB")
        except:
            print('failed to open usb to Nexus DSC')

    def setState(self):
        try:
            self.ser.write(b":P#")
            time.sleep(0.1)
            p = str(self.ser.read(self.ser.in_waiting).decode("cp1253")).strip("#")
            if p[0] == "L":
                self.ser.write(b":U#")
            self.ser.write(b":P#")
            time.sleep(0.1)
            res = str(self.ser.read(self.ser.in_waiting).decode('cp1253')).strip("#")
            print("Connected to Nexus in", res)
        except:
            print('failed to open usb to Nexus DSC')
    def write(self, txt: str):
        self.ser.write(bytes(txt.encode("cp1253")))
        print("sent", txt, "to Nexus")

    def writeBytes(self,txt,byt):
        print('byte array length',len(byt))
        self.ser.write(bytes(txt.encode('cp1253')))
        time.sleep(0.01)
        for i in range (0,1024,32):
            pkt = byt[i:i+32]
            self.ser.write(pkt)
            time.sleep(0.01)

    def get(self, txt: str):
        self.ser.write(bytes(txt.encode('cp1253')))
        time.sleep(0.05)
        res = str(self.ser.read(self.ser.in_waiting).decode('cp1253')).strip("#")
        return res
    
    def scan(self):
        try:
            if self.ser.in_waiting > 0:
                a = str(self.ser.read(self.ser.in_waiting).decode('cp1253'))
                return a
        except:
            self.ser = serial.Serial(self.auxDevice,baudrate=self.baudRate, xonxoff=True, write_timeout=1)
            if self.ser.is_open:
                txt = ':ID=eFinderLite#'
                self.ser.write(bytes(txt.encode('cp1253')))


