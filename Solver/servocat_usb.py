import serial
import time
import subprocess

class ServoCat:
    """The ServoCat utility class"""

    def __init__(self,auxD,baudR):
        self.auxDevice = auxD
        self.baudRate = baudR
        try:
            self.ser = serial.Serial(self.auxDevice,baudrate=self.baudRate, write_timeout=1)
            print('Aux USB opened')
            self.auxOpen = True
        except:
            print ("cant open Aux USB")
            self.auxOpen = False
    
    def isAuxOpen(self):
        return self.auxOpen
        
    def write(self, txt: str):
        self.ser.write(bytes(txt.encode("cp1253")))
        print("sent", txt, "Aux usb")

    def scan(self):
        try:
            if self.ser.in_waiting > 0:
                time.sleep(0.1)
                a = str(self.ser.read(self.ser.in_waiting).decode("ascii"))
                return a
        except:
            try:
                self.ser = serial.Serial(self.auxDevice,baudrate=self.baudRate)
            except:
                self.auxOpen = False
                print ('Aux communications lost')