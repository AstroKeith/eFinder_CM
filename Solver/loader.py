#!/usr/bin/env python3
import os
import sys
import subprocess
import time

os.system('sudo pigpiod')

import pigpio

switch = pigpio.pi()
switch.set_mode(17, pigpio.INPUT)
switch.set_pull_up_down(17, pigpio.PUD_UP)
led = pigpio.pi()

if switch.read(17) == 0: # need to start as Mini
    print ('Starting as eFinder Mini')
    led.hardware_PWM(18,10,500000)
    time.sleep(2)
    led.hardware_PWM(18,200,0)
    subprocess.Popen(["/home/efinder/venv-efinder/bin/python","/home/efinder/Solver/eFinder_mini.py"])
    sys.exit(0)

led.hardware_PWM(18,10,500000)
time.sleep(2)
led.hardware_PWM(18,200,0)

reboot_flag = False
filename = '/home/efinder/uploads/efinderUpdatePro.zip'
runfile = '/home/efinder/uploads/update.py'

if os.path.isfile(filename):
	led.hardware_PWM(18,10,500000) # blink led very fast to indicate update in progress
	subprocess.run(['sudo', 'chmod', 'a+rwx', '-R', '/var/www/html/'], check=False) # ensure web server can write to uploads folder for updates and logs
	try:
		#print("Following files found to be installed/updated")
		#subprocess.run(['unzip', '-v', filename], check=True)
		print('Starting update')
		os.system('sudo chown efinder:efinder "efinderUpdatePro.zip"')
		result = subprocess.run(['sudo', 'unzip', '-d', '/', '-o', filename],
		                        capture_output=True, text=True)
		if result.returncode != 0:
			raise RuntimeError(f"unzip failed (exit {result.returncode}):\n{result.stderr}")
		print(result.stdout)
		subprocess.run(['sudo', 'sync'], check=True)
		subprocess.run(['sudo', 'rm', filename], check=True)
		print('All files updated and zip file deleted')
	except Exception as ex:
		print(f"An unexpected error occurred: {ex}")
		with open("eFinderLoader.txt", "w") as h:
			h.write(str(ex))
		try:
			subprocess.run(['sudo', 'rm', filename], check=False) # remove the zip file to prevent repeated errors on reboot
		except Exception as ex2:
			print(f"An unexpected error occurred while trying to remove the zip file: {ex2}")
	reboot_flag = True
	led.hardware_PWM(18,200,0) # turn off LED

if os.path.isfile(runfile):
	led.hardware_PWM(18,10,500000) # blink led very fast to indicate update in progress
	print('Running update script')
	try:
		subprocess.run(["/home/efinder/venv-efinder/bin/python",runfile])
		print('Update script completed successfully')
	except Exception as ex:
		print(f"An unexpected error occurred while running the update script: {ex}")
		with open("eFinderRunfile.txt", "w") as h:
			h.write(str(ex))
	os.system('sudo rm ' +runfile) # remove the update script to prevent repeated runs on reboot
	led.hardware_PWM(18,200,0) # turn off LED
	reboot_flag = True

if reboot_flag:
	os.system('sudo killall pigpiod')
	os.system('sudo systemctl reboot')

else:
	print('no zip file or update.py found')
	subprocess.Popen(["/home/efinder/venv-efinder/bin/python","/home/efinder/Solver/eFinder.py"])
	sys.exit(0)