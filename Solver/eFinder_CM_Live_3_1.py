#!/usr/bin/python3

# Program to implement an eFinder (electronic finder)
# Copyright (C) 2025 Keith Venables.
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
from __future__ import annotations
import os
import re
import sys
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--test", help="Run in test mode", action="store_true")
args = parser.parse_args()
if args.test:
    print("Running in test mode")

os.system('sudo pigpiod')

import math
import socket
from threading import Thread
import pigpio

led = pigpio.pi()
led.hardware_PWM(18,4,500000)

switch = pigpio.pi()
switch.set_mode(17, pigpio.INPUT)
switch.set_pull_up_down(17, pigpio.PUD_UP)

from pathlib import Path
home_path = str(Path.home())

version = "3.1"

print ('eFinder Live','Version '+ version)
print ('Loading program')
import time
from datetime import datetime, timezone
from PIL import Image, ImageDraw, ImageFont, ImageEnhance, ImageOps
import numpy as np
import RPICamera_Nexus_4
import Coordinates_wifi_2
import tetra3
import csv
from io import BytesIO
import subprocess

#from skyfield.api import load
from rig_plate_solve import make_rig_plate_solve
from astro_time import ObserverLocation
from telescope_pointing import TelescopePointingEstimator, get_pointing_estimate
from bno085_interface import open_bno085_software_i2c, read_imu_sample
from test_plate_solver import make_synthetic_plate_solve, LastSampleCache
coordinates = Coordinates_wifi_2.Coordinates()
if args.test:
    if os.path.exists("/dev/ttyGS0"):
        nexusDevice = "/dev/ttyGS0"
        nexusBaudrate = 115200
    else:
        nexusDevice = "/dev/ttyAMA3"
        nexusBaudrate = 115200
    import Nexus_usb
    nexus = Nexus_usb.Nexus(nexusDevice,nexusBaudrate)
    nexus.setState()
    from rig_plate_solve import make_rig_plate_solve

t3 = tetra3.Tetra3("/home/efinder/Solver/t3_fov14_mag8.npz")

expInc = 0.1 # sets how much exposure changes when using handpad adjust (seconds)
gainInc = 5 # ditto for gain
offset_flag = False
offset_str = "0,0"
solve = False
testMode = args.test # True = bench test with a synthetic solver, no sky needed
stars = peak = '0'
capArray = np.zeros((760,960),dtype=np.uint8)
hotspot = False
keep = False
solved_radec = 0,0
fnt = ImageFont.truetype("/home/efinder/Solver/text.ttf",16)
frame = 0
loop = True
lock = False
DIAGNOSE_FREEZE = True  # True = log every plate-solve attempt and how long it took, to help diagnose occasional freezes during a slew
USE_GAME_ROTATION_VECTOR = True
MAX_RATE_DEG_S = 0.2 #float(param["MAX_RATE_DEG_S"])
MIN_SETTLE_S = 0.4

ts = load.timescale()

def logWrite(err):
    with open("/var/www/html/eFinderLiveLog.txt", "a") as h:
        h.write(err+'\n')

def configRecover():
    global fault
    try:
        if os.path.exists(home_path + "/Solver/backup.config") == True:
            os.system('sudo cp /home/efinder/Solver/backup.config /home/efinder/Solver/eFinder.config')
            os.system("sudo chmod a+rwx /home/efinder/Solver/eFinder.config")
            logWrite("New default eFinder.config had to be created")
        else:
            fault = fault + "1"
            logWrite("backup.config needed but not found")
    except Exception as error:
        fault = fault + "1"
        logWrite(str(error))

def serveWifi(): # serve WiFi port
    global solved_radec, lock, addr, param, keep, frame, Lat, Long
    print ('starting wifi server')
    host = ''
    port = 4060
    backlog = 50
    size = 1024
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((host,port))
    s.listen(backlog)
    raStr = decStr = ""
    timeOffset = '0'
    timeStr = '23:00:00'
    try:
        while True:
            try:
                client, address = s.accept()
                while True:
                    data = client.recv(size)
                    if not data:
                        break
                    if data:
                        pkt = data.decode("utf-8","ignore")
                        time.sleep(0.02)
                        a = pkt.split('#')
                        #print(a)
                        raPacket = coordinates.hh2dms(solved_radec[0]/15)+'#'
                        decPacket = coordinates.dd2aligndms(solved_radec[1])+'#'
                        for x in a:
                            if x != '':
                                #print (x)
                                if x == ':GR':
                                    client.send(bytes(raPacket.encode('ascii')))
                                elif x == ':GD':
                                    client.send(bytes(decPacket.encode('ascii')))
                                elif x[1:3] == 'St':
                                    client.send(b'1')
                                    LatStr = x[3:].split('*')
                                    Lat = int(LatStr[0]) + int(LatStr[1])/60 # Latitude as decimal degrees North +Ve
                                    param["Lat"] = str(Lat)
                                    save_param()
                                elif x[1:3] == 'Sg':
                                    client.send(b'1')
                                    LongStr = x[3:].split('*')
                                    Long = -(int(LongStr[0]) + int(LongStr[1])/60) # Longitude as decimal degrees West -ve
                                    param["Long"] = str(Long)
                                    save_param()
                                elif x[1:3] == 'SG':
                                    client.send(b'1')
                                    timeOffset = x[3:]
                                elif x[1:3] == 'SL':
                                    client.send(b'1')
                                    timeStr = x[3:]
                                elif x[1:3] == 'SC':
                                    client.send(b'Updating Planetary Data#                              #')
                                    print('dateSet',timeOffset,timeStr,x[3:])
                                    coordinates.dateSet(timeOffset,timeStr,x[3:])
                                    lock = True
                                    coordinates.setup_precess()
                                    
                                elif x[1:3] == 'RG': # set minimum exp/gain
                                    selectExp(0.1,10)
                                elif x[1:3] == 'RC': # set med exp/gain
                                    selectExp(0.1,20)
                                elif x[1:3] == 'RM': # set high exp/gain
                                    selectExp(0.2,20)
                                elif x[1:3] == 'RS': # set very high exp/gain
                                    selectExp(0.5,30)
                                elif x[1:3] == 'Sr': # target RA
                                    raStr = x[3:]
                                    client.send(b'1')
                                elif x[1:3] == 'Sd': # target Dec
                                    decStr = x[3:]
                                    client.send(b'1')
                                elif x[1:3] == 'Ms':
                                    adjExp(-1)
                                elif x[1:3] == 'Mn':
                                    adjExp(1)
                                elif x[1:3] == 'Me' or x[1:3] == 'Mw':
                                    keep = True
                                    frame = 0
                                elif x[1:3] == 'CM': # do offset
                                    client.send(b'0')
                                    measure_offset()
                                elif x[-1] == 'Q':
                                    print('Stop saving images')
                                    keep = False
                                    frame = 0
            except Exception as error:
                print (error)
                print ('re-starting Device wifi server')
                with open("fred.txt", "a") as h:
                    h.write(str(error)+'\n')
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                s.bind((host,port))
                s.listen(backlog)
    except Exception as error:
        print('outer error,',error)
        pass
        with open("/var/www/html/eFinderLiveLog.txt", "a") as h:
            h.write(str(error)+'\n')

def selectExp(e,g):
    camera.set(e,g)
    param['Exposure'] = str(e)
    param['Gain'] = str(g)
    save_param()

def pixel2dxdy(pix_x, pix_y):  # converts a pixel position, into a delta angular offset from the image centre
    deg_x = (float(pix_x) - cam[0]/2) * cam[2] / 3600  # in degrees
    deg_y = (cam[1]/2 - float(pix_y)) * cam[2] / 3600
    return (deg_x, deg_y)

def dxdy2pixel(dx, dy): # converts offsets in arcseconds to pixel position
    pix_x = dx * 3600 / cam[2] + cam[0]/2
    pix_y = cam[1]/2 - dy * 3600 / cam[2]
    return (pix_x, pix_y)

def capture():
    global capArray
    hemi = 'N'
    if testMode == True:
        if offset_flag == False:
            test = True
            offset = False
        else:
            test = False
            offset = True
    else:
        test = False
        offset = False
    capArray = camera.capture(test,offset,hemi)
    return capArray

def solveImage(img):
    global offset_flag, solve, eTime, firstStar, solution, cam, stars, peak
    start_time = time.time()
    print ("Started solving")
    np_image = np.asarray(img, dtype=np.float32)
    centroids = t3.get_centroids_from_image(np_image)
    print ('centroids',len(centroids),'   peak',np.max(np_image))
    if len(centroids) < 15:
        print ("Bad image","only "+ str(len(centroids))," centroids")
        solve = False
        if keep:
            txt = "Bad image - " + str(len(centroids)) + " stars" + "    Exp = "+str(param['Exposure'])+ 's.   Gain = ' + str(param["Gain"])
            saveImage(img,txt)
        return (0,0,0,solve)
    stars = ('%4d' % (len(centroids)))
    peak = ('%3d' % (np.max(np_image)))
    solution = t3.solve_from_centroids(
        centroids,
        (760,960),
        fov_estimate=13.4, 
        target_pixel=target_px,
        distortion = 0.0,
        return_matches=True
    )
    #print ('solution',solution)
    elapsed_time = time.time() - start_time
    eTime = ('%2.2f' % (elapsed_time * 1000)).zfill(5)
    print ('solve time',eTime,'ms')
    if solution['RA'] == None:
        print ("Not Solved",stars + " stars")
        if keep:
            txt = "Not Solved - " + stars + " stars" + "    Exp = "+str(param['Exposure'])+ 's.   Gain = ' + str(param["Gain"])
            saveImage(img,txt)
        solve = False
        return (0,0,0,solve)
    firstStar = centroids[0]
    ra = solution['RA_target'][0]
    dec = solution['Dec_target'][0]
    print ('J2000',coordinates.hh2dms(ra/15),coordinates.dd2aligndms(dec))
    ra,dec = coordinates.precess(ra,dec)
    if keep:
        txt = "Peak = "+ str(np.max(np_image)) + "   Stars = "+ str(len(centroids)) + "    Exp = "+str(param['Exposure'])+ 's.   Gain = ' + str(param["Gain"])
        saveImage(img,txt)
    #solved_radec = ra,dec
    print ('JNow',coordinates.hh2dms(solved_radec[0]/15),coordinates.dd2aligndms(solved_radec[1]))
    solve = True
    return (ra,dec,solution['Roll'],solve)

def saveImage(array,txt):
    global frame, keep
    frame +=1
    start = time.time()
    img = Image.fromarray(array)
    img2 = ImageEnhance.Contrast(img).enhance(5)
    img2 = img2.rotate(angle=180)
    img3 = ImageDraw.Draw(img2)
    txt = txt + "      Frame " + str(frame)
    print(txt)
    img3.text((70,5), txt, font = fnt, fill='white')
    img2 = ImageOps.expand(img2,border=5,fill='red')
    img2 = img2.save('/home/efinder/Solver/images/capture.jpg')
    print ('save %5.3f secs' % (time.time()-start))
    if frame > 500:
        keep = False
        frame = 0


def measure_offset():
    global offset_str, offset_flag, offset, param, target_px
    offset_flag = True
    print ("started capture")
    solveImage(capture())
    if solve == False:
        print ("solve failed")
        offset_flag = False
        return ("fail")
    tempExp = param['Exposure']
    while float(peak) > 255:
        tempExp = tempExp * 0.75
        camera.set(tempExp, param["Gain"])
        solveImage(capture())
    if solve == False:
        print ("solve failed")
        offset_flag = False
        return ("fail")
    scope_x = firstStar[1]
    scope_y = firstStar[0]
    offset = firstStar
    target_px = np.array([offset], dtype=np.float64)
    d_x, d_y = pixel2dxdy(scope_x, scope_y)
    param["d_x"] = "{: .2f}".format(float(60 * d_x))
    param["d_y"] = "{: .2f}".format(float(60 * d_y))
    save_param()
    offset_str = ('%1.3f,%1.3f' % (d_x,d_y))
    hipId = str(solution['matched_catID'][0])
    name = secondname = ""
    with open(home_path+'/Solver/starnames.csv') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            for row in reader:
                nam = row[0].strip()
                hip = row[1]
                if str(row[1]) == str(solution['matched_catID'][0]):
                    hipId = hip
                    name = nam
                    if len(row[2].strip())==0:
                        secondname = ""
                    else:
                        secondname = " ("+row[2].strip()+")"
                    break       
    print (name + ', HIP ' + hipId)
    offset_flag = False
    return(name+secondname+',HIP'+hipId+','+offset_str)

def go_solve():
    solveImage(capture())
    if solve == True:
        print ("Solved")
        return('1')
    else:
        print ("Not Solved")
        return('0')

def reset_offset():
    global param, offset, offset_str,target_px
    param["d_x"] = 0
    param["d_y"] = 0
    offset = (cam[1]/2, cam[0]/2) # default centre of the image
    target_px = np.array([offset], dtype=np.float64)
    offset_str = ('%1.3f,%1.3f' % (float(param["d_x"])/60, float(param["d_y"])/60))
    save_param()
    return('1') 

def save_param():
    with open("/home/efinder/Solver/eFinder.config", "w") as h:
        for key, value in param.items():
            h.write("%s:%s\n" % (key, value))

def adjExp(i): #manual
    global param
    param['Exposure'] = ('%.1f' % (float(param['Exposure']) + i*expInc))
    if float(param['Exposure']) < 0:
        param['Exposure'] = '0.1'
    exp = ('%.1f' % float((param['Exposure'])))
    save_param()
    camera.set(float(param["Exposure"]),param["Gain"])
    return str(exp)

def adjGain(i): #manual
    global param
    param['Gain'] = ('%.1f' % (float(param['Gain']) + i*gainInc))
    if float(param['Gain']) < 0:
        param['Gain'] = '5'
    elif float(param['Gain']) > 50:
        param['Gain'] = '50'
    gain = ('%3.1f' % (float(param['Gain'])))
    camera.set(float(param["Exposure"]),param["Gain"])
    save_param()
    return str(gain)

def setExp(a):
    global param
    param["Exposure"] = float(a)
    save_param()
    camera.set(float(a),param["Gain"])
    return '1'

def getAutoExp():
    expAuto = float(param["Exposure"])
    camera.set(expAuto,param["Gain"])
    np_image = capture()
    while True:
        pk = np.max(np_image)
        centroids = t3.get_centroids_from_image(np_image)
        print ('%4d %s   %3d %s ' % (len(centroids),"stars",pk,"peak signal"))
        if len(centroids) < 20:
            expAuto = expAuto * 2
            camera.set(expAuto,param["Gain"])
            capture()
        elif len(centroids) > 50 and pk > 250:
            expAuto = int((expAuto / 2) * 10)/10
            camera.set(expAuto,param["Gain"])
            capture()
        else:
            break
    return (str(expAuto))
            

def flipTestMode(mode):
    global testMode
    testMode = mode
    return('1')

def array_to_bytes(x: np.ndarray) -> bytes:
    np_bytes = BytesIO()
    np.save(np_bytes, x, allow_pickle=True)
    return np_bytes.getvalue()[128:]

def checkWifiCon(con):
    cmd = ['nmcli'] + ['con'] + ['show'] + ['--active']
    result = subprocess.run(cmd,capture_output=True,text=True)
    if con in str(result.stdout):
        return True
    else:
        return False

def checkWifiConExist(con):
    cmd = ['nmcli'] + ['con'] + ['show']
    result = subprocess.run(cmd,capture_output=True,text=True)
    if con in str(result.stdout):
        return True
    else:
        return False

def checkWifi(con):
    cmd = ['nmcli'] + ['radio'] + ['wifi']
    result = subprocess.run(cmd,capture_output=True,text=True)
    if con in str(result.stdout):
        return True
    else:
        return False
    
def setWifi(msg):
    global hotspot
    if msg == "":
        if checkWifi('enabled'):
            reply = '1'
        else:
            reply = '0'
        if checkWifiCon('Hotspot'):
            reply = reply + '1'
        else:
            reply = reply + '0'
        return reply
    elif msg == "0":
        os.system('sudo nmcli conn up preconfigured')
        hotspot = False
        return '1'
    elif msg == '1':
        os.system('sudo nmcli con up Hotspot')
        hotspot = True
        return '1'
    else:
        return '0'

def flipWifi():
    global led, hotspot, led_duty_cycle, loop
    loop = False
    led.hardware_PWM(18,4,500000)
    if checkWifiCon('Hotspot'):
        try:
            os.system('sudo nmcli conn up preconfigured')
            hotspot = False
        except Exception as error:
            logWrite(str(error))
            print (error)
    else:
        os.system('sudo nmcli con up Hotspot')
        hotspot = True
    time.sleep(2)
    led.hardware_PWM(18,200,led_duty_cycle)
    loop = True
    
def wifiOnOff(msg):
    if msg == '0':
        os.system('sudo nmcli radio wifi off')
        print('wifi off')
    else:
        os.system('sudo nmcli radio wifi on')
        print('wifi on')
    return "1"

def createDefaultHotspot():
    import uuid
    ssid = 'efinder'+(hex(uuid.getnode())[-4:])
    pswd = "12345678"
    os.system("sudo nmcli dev wifi hotspot ssid '"+ ssid +"' password '" + pswd + "'")

def configHotspotWifi(msg):
    global hotspot
    try:
        os.system("sudo nmcli con delete Hotspot")
    except Exception as error:
        logWrite(str(error))
    sid,pswd = msg.split(" ")
    os.system("sudo nmcli dev wifi hotspot ssid '"+ sid +"' password '" + pswd + "'")
    hotspot = True
    return '1'

def configInfraWifi(msg):
    global hotspot
    try:
        os.system("sudo nmcli con delete preconfigured")
    except Exception as error:
        logWrite(str(error))
    sid,pswd = msg.split(" ")
    os.system("sudo nmcli device wifi")
    os.system("sudo nmcli c add type wifi con-name 'preconfigured' ifname wlan0 ssid '" + sid + "'")
    os.system("sudo nmcli c modify 'preconfigured' wifi-sec.key-mgmt wpa-psk wifi-sec.psk '" + pswd + "'")
    hotspot = False
    return '1'

def setLED(b):
    global param,led
    led_duty_cycle = int(b) * 10000
    led.hardware_PWM(18,200,led_duty_cycle)
    param["LED"] = int(b)
    save_param()
    return ('1')
    
def startImage(j):
    global keep, frame
    if j == '1':
        print('Started saving images')
        keep = True
        frame = 0
    else:
        print('Stop saving images')
        keep = False
    return '1'

def readConfig():
    global param
    with open(home_path + "/Solver/eFinder.config") as h:
        for line in h:
            line = line.strip("\n").split(":")
            try:
                param[line[0]] = str(line[1])
            except:
                pass
        print(param,len(param))

def read_imu():
    """Wraps the driver read into an IMUSample with a UTC timestamp.
    Your Pi's clock is GPS-disciplined to UTC, so datetime.now(timezone.utc)
    is trustworthy here without any extra correction.

    Also caches the reading (harmless in normal use) so that, in
    TEST_MODE, the synthetic solver can reuse this exact sample instead
    of taking its own separate reading -- see test_plate_solver.py's
    LastSampleCache for why that consistency matters.
    """
    sample = _last_imu_cache.update(read_imu_sample(bno))

    if DIAGNOSE_FREEZE:
        global _diag_last_raw_quat
        quat = (sample.x, sample.y, sample.z, sample.w)
        if quat == _diag_last_raw_quat:
            # Two consecutive reads with the EXACT SAME raw quaternion --
            # i.e. the driver handed back a stale/cached report rather
            # than a fresh one since the last call. This would make the
            # rate check briefly read ~0 deg/s even during real motion --
            # see the module docstring's "DIAGNOSING..." section.
            print("  [diag] *** duplicate raw quaternion -- IMU read returned a stale report ***")
        _diag_last_raw_quat = quat

    return sample

def efinder_plate_solve():
    """ It should return a plain 4-tuple
    (ra_deg, dec_deg, roll_deg, solved)
    """
    return solveImage(capture())

def read_rig():
    #time.sleep(0.2)  # give the camera a moment to finish its exposure
    rastr = nexus.get(":GR#").split(":")
    decstr = re.split(r"[:*]", nexus.get(":GD#"))
    ra = (float(rastr[0]) + float(rastr[1]) / 60 + float(rastr[2]) / 3600) * 15 # convert to decimal degrees
    dec = math.copysign(abs(abs(float(decstr[0])) + float(decstr[1]) / 60 + float(decstr[2]) / 3600),float(decstr[0]))
    Rad = math.pi / 180
    t = ts.now()
    LST = t.gmst + Long / 15  # as decimal hours
    LSTd = LST * 15
    LHA = (LSTd - ra + 360) - ((int)((LSTd - ra + 360) / 360)) * 360
    x = math.cos(LHA * Rad) * math.cos(dec * Rad)
    y = math.sin(LHA * Rad) * math.cos(dec * Rad)
    z = math.sin(dec * Rad)
    xhor = x * math.cos((90 - Lat) * Rad) - z * math.sin((90 - Lat) * Rad)
    yhor = y
    zhor = x * math.sin((90 - Lat) * Rad) + z * math.cos((90 - Lat) * Rad)
    az = math.atan2(yhor, xhor) * (180 / math.pi) + 180
    alt = math.asin(zhor) * (180 / math.pi)
    tick = datetime.now(timezone.utc)
    print ('-------------------------------------')
    print ("read_rig = RA:%.2f Dec:%.2f Alt:%.2f Az:%.2f datetime:%s" % (ra,dec,alt,az, tick))
    return ra, dec, alt, az, tick

# main code starts here

with open("/var/www/html/eFinderLiveLog.txt", "w") as h:
    h.write('eFinder Live errors on boot:\n')

param = dict()

try:
    if os.path.exists(home_path + "/Solver/eFinder.config") == False:
        print('eFinder.config not found')
        configRecover()

    readConfig()
    try:
        if len(param) != float(param["check"]):
            print("bad config file")
            configRecover()
            print("New copy made from backup")
            readConfig()
    except:
        print("check parameter missing in config file")
        configRecover()
        print("New copy made from backup")
        readConfig()
         
except Exception as error:
    logWrite(str(error))
    print(str(error))


pimodel = subprocess.run(["cat","/sys/firmware/devicetree/base/model"],capture_output=True, text=True)
modelStr = str(pimodel.stdout)
pwmInvert = 0
altAngle = "none"

try:
    camera = RPICamera_Nexus_4.RPICamera()
    camera.set(float(param["Exposure"]),param["Gain"])
except Exception as error:
    logWrite(str(error))

cam = (960,760,50.8,13.5)   

pix_x, pix_y = dxdy2pixel(float(param["d_x"])/60, float(param["d_y"])/60)
offset_str = ('%1.3f,%1.3f' % (float(param["d_x"])/60, float(param["d_y"])/60))

offset = (pix_y, pix_x) 

target_px = np.array([offset], dtype=np.float64)
print('offset',target_px)
np_image = np.zeros((760,960),dtype=np.float32)

wifiloop = Thread(target=serveWifi)
wifiloop.start()
print ('Waiting for SkySafari to connect')
while lock == False:
    time.sleep(0.2)
print ('SkySafari connected, starting eFinder Live',Lat,Long)

pwmInvert = 1000000

bno = open_bno085_software_i2c(bus_number=3)
site = ObserverLocation(latitude_deg=Lat, longitude_deg=Long, elevation_m=0.0)
estimator = TelescopePointingEstimator(site, body_boresight=np.array([0.0, 0.0, -1.0]))
_last_imu_cache = LastSampleCache()  # only actually used in TEST_MODE -- see below
_diag_last_raw_quat = None  # (x, y, z, w) of the previous read_imu() call, for staleness detection
_underlying_plate_solve= make_rig_plate_solve(read_rig, site) if testMode else efinder_plate_solve
print('Running with IMU')

if DIAGNOSE_FREEZE:
    def plate_solve():
        """Diagnostic wrapper -- logs every solve ATTEMPT (i.e. every call
        that got past get_pointing_estimate()'s rate gate) with the rate
        that let it through and how long the call actually took, so a
        freeze in the console/SkySafari can be matched, after the fact,
        against a specific slow solve call. See the module docstring's
        "DIAGNOSING THE OCCASIONAL FREEZE DURING A SLEW" section."""
        rate = estimator.angular_rate_deg_s()
        rate_str = f"{rate:.3f} deg/s" if rate is not None else "unknown (fewer than 2 IMU samples yet)"
        t_start = time.monotonic()
        print(f"  [diag] solve ATTEMPT starting -- last measured rate: {rate_str}")

        result = _underlying_plate_solve()

        elapsed = time.monotonic() - t_start
        solved = result[3]
        print(f"  [diag] solve ATTEMPT finished in {elapsed:.2f}s -- solved={solved}")
        if elapsed > 1.0:
            print(f"  [diag] *** this attempt took {elapsed:.2f}s -- likely cause of a visible freeze ***")
        return result
else:
    plate_solve = _underlying_plate_solve

now = datetime.now(timezone.utc)
print("Pi UTC currently set to: " + now.strftime("%Y-%m-%d %H:%M:%S"))

hotspot = False
try:
    if param['wifi'].lower() != 'hotspot':
        hotspot = False
        print('efinder.config says no Hotspot')
except Exception as error:
    logWrite(str(error))

if hotspot == True:
    if not checkWifiConExist('Hotspot'):
        print ('Creating a default Hotspot Wifi')
        createDefaultHotspot()
    setWifi('1')
else:
    print('joining infrastructure wifi')
    setWifi('0')

led_duty_cycle = int(float(param["LED"])) * 10000
led.hardware_PWM(18,200,abs(pwmInvert - led_duty_cycle))

loop = True
a = True

was_mount_calibrated = False
print('plate-solve',plate_solve, type(plate_solve))
while True:
    _tick_start = time.monotonic() if DIAGNOSE_FREEZE else None

    if switch.read(17) == 0:
        flipWifi()

    if offset_flag == False and loop == True:
        led.hardware_PWM(18,200,a * abs(pwmInvert - led_duty_cycle))

        estimate = get_pointing_estimate(
            estimator, plate_solve, read_imu, 
            max_rate_deg_s=MAX_RATE_DEG_S, min_settle_s=MIN_SETTLE_S)

        if DIAGNOSE_FREEZE:
            _tick_elapsed = time.monotonic() - _tick_start
            if _tick_elapsed > 1.0:
                # Catches a slow tick for ANY reason, not just a slow solve
                # attempt (e.g. an I2C hiccup on the IMU read itself) -- the
                # solve-attempt log above will already explain it if that
                # was the cause; if this fires with no matching solve-ATTEMPT
                # line just before it, the cause is elsewhere (e.g. the IMU
                # read), which narrows things down just as usefully.
                print(f"  [diag] *** whole tick took {_tick_elapsed:.2f}s ***")

        if estimator.mount_calibrated and not was_mount_calibrated:
                    print(f"\n*** {estimator.mount_calibration_status.describe()} ***\n")
                    was_mount_calibrated = True
                    
        if estimate is None:
            # Only happens before the very first successful solve: there's
            # no offset yet, so an IMU-only estimate isn't possible.
            print(f"Not yet ready for IMU estimates -- {estimator.mount_calibration_status.describe()}")
        elif estimate.source == "plate_solve":
            solved_radec = (estimate.ra_deg, estimate.dec_deg)
            print(
                f"[solved]  RA={estimate.ra_deg:.4f}  Dec={estimate.dec_deg:.4f}  "
                f"roll={estimate.position_angle_deg:.2f} -- {estimator.mount_calibration_status.describe()}"
            )
        else:
            age = f"{estimate.offset_age_s:.1f}s" if estimate.offset_age_s is not None else "?"
            solved_radec = (estimate.ra_deg, estimate.dec_deg)
            print(
                f"[imu]     RA={estimate.ra_deg:.4f}  Dec={estimate.dec_deg:.4f}  "
                f"(offset age {age}{', STALE' if estimate.is_stale else ''})"
            )


        
        a = not a

    time.sleep(0.05) 
