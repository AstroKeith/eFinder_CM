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


print ('Camera Test')
print ('Loading program')
import time
from PIL import Image, ImageDraw, ImageFont, ImageEnhance, ImageOps
import numpy as np
import Camera_Nexus
import argparse

parser = argparse.ArgumentParser()
parser.add_argument(
    "--exp",
    dest="exp",
    type=float,
    default=0.1,
    help="Fixed camera exposure time in seconds"
    )
parser.add_argument(
    "--gain",
    dest="gn",
    type=float,
    default=5,
    help="Fixed camera gain"
    )

capArray = np.zeros((760,960),dtype=np.uint8)
np_image = np.zeros((760,960),dtype=np.float32)
fnt = ImageFont.truetype("/home/efinder/Solver/text.ttf",16)

def capture():
    global capArray
    capArray = camera.capture(False,False,'N')
    return capArray

def saveImage(array,txt):
    img = Image.fromarray(array)
    img2 = ImageEnhance.Contrast(img).enhance(5)
    img2 = img2.rotate(angle=180)
    img3 = ImageDraw.Draw(img2)
    txt = txt
    img3.text((70,5), txt, font = fnt, fill='white')
    img2 = ImageOps.expand(img2,border=5,fill='red')
    img2 = img2.save('/home/efinder/Solver/images/capture.jpg')

# main code starts here

args = parser.parse_args()

camera = Camera_Nexus.Camera("IMX335")
#camera = Camera_Nexus.Camera("RPI")
camera.set(args.exp,args.gn)
print ('Image can be viewed at "efinder.local/view.php"')
txt = 'Exp: ' + str(args.exp) + '  Gain: ' + str(args.gn)
try:
    while True:
        saveImage(capture(),txt)
        time.sleep(0.5)
except KeyboardInterrupt:
    print('stopping')
try:
    exit()
except Exception as e:
    print(f"Error exiting: {e}")
