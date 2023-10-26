from dronekit import connect, VehicleMode, APIException, LocationGlobalRelative
from pymavlink import mavutil
import time
import numpy as np
from config import *
from util import *
import RPi.GPIO as GPIO
import sys
import argparse

###############################################################################
# Functions
###############################################################################

start_time = time.time()
target_location = LocationGlobalRelative(TARGET_LAT, TARGET_LON, 0)
# Connection string for Pixhawk (adjust based on your setup)
connection_string = '/dev/serial0'  # Replace with your connection string





GPIO.setmode(GPIO.BCM)
GPIO.setup(BURN_PIN, GPIO.OUT)
GPIO.output(BURN_PIN, GPIO.LOW)
raw_input('Press enter to burn')
GPIO.output(BURN_PIN, GPIO.HIGH)
time.sleep(5)
GPIO.output(BURN_PIN, GPIO.LOW)
GPIO.cleanup()

BURN_PIN 				= 27				# pin with burn relay
BURN_ALTITUDE 			= 1 			    # altitude at which to burn
BURN_TIME_ABOVE 		= 5					# time above burn alt before ignite
BURN_TIME_BELOW			= 3					# time below burn alt after ignite
TARGET_LAT	 			= 42.3455724		# desired landing latitude
TARGET_LON				= -71.2100037		# desired landing longitude
TARGET_ALT 				= 0 				# meters above sea level
LOOP_DELAY				= 0.5				# time in seconds to delay within loops
MIN_SATS				= 5	


def mytime():
	return time.time() - start_time

def exit(status):
	if vehicle is not None: vehicle.close()
	GPIO.cleanup()
	sys.exit(status)

def print_status():
	print('Time: {:.2f}s\tMode: {}\tSats: {}\tAlt: {}m\tLoc: ({}, {})\tdist: {:.3f}km.').format(
		mytime(),
		vehicle.mode.name,
		vehicle.gps_0.satellites_visible,
		vehicle.location.global_relative_frame.alt,
		vehicle.location.global_frame.lat,
		vehicle.location.global_frame.lon,
		straight_line_dist(target_location, vehicle.location.global_frame))

###############################################################################
# Setup
##############################################################################
GPIO.setmode(GPIO.BCM)
GPIO.setup(BURN_PIN, GPIO.OUT)
GPIO.output(BURN_PIN, GPIO.LOW)
print('GPIO enabled on pin {}').format(BURN_PIN)
print ('Burst altitude set to {} meters\n').format(BURN_ALTITUDE)



vehicle = None
try:
    vehicle = connect(connection_string, baud=57600, wait_ready=True)
except Exception as e:
    print(f"Failed to connect to Pixhawk: {e}")
    exit(1)

vehicle.mode = VehicleMode("MANUAL")

while not vehicle.mode.name == "MANUAL":
	print('Waiting for MANUAL mode.')
	time.sleep(1)

while vehicle.gps_0.satellites_visible < MIN_SATS:
	print('{} satellites aquired. Waiting for {}.').format(vehicle.gps_0.satellites_visible, MIN_SATS)
	time.sleep(1)

vehicle.armed = True

while not vehicle.armed:
	print('Waiting for arm.')
	time.sleep(1)

print('Downloading commands...')
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()
print('Done!')

print_status()
print('\n################# READY FOR FLIGHT #################\n')

###############################################################################
# Ascent
###############################################################################

# maintain a circular buffer of altitude
alt_buffer_len 	= int(60/LOOP_DELAY)
alt_buffer 		= np.ones([alt_buffer_len]) * vehicle.location.global_relative_frame.alt
alt_buffer_ind 	= 0

prev_time_below_burn_alt = mytime()
while True:
	if alt_buffer_ind%2 == 0: print_status()	# print status once every second

	alt = vehicle.location.global_relative_frame.alt
	alt_buffer[alt_buffer_ind] = alt
	alt_buffer_ind += 1
	alt_buffer_ind = alt_buffer_ind % alt_buffer_len
	alt_diff = alt - alt_buffer[alt_buffer_ind]
	if (alt_diff < -50):
		print('WARNING: descended {}m in 60 seconds. Disconnecting.').format(alt_diff)
		break

	if alt < BURN_ALTITUDE: 
		prev_time_below_burn_alt = mytime()
	else: 
		time_above = mytime() - prev_time_below_burn_alt
		print('Above {}m for {} seconds').format(BURN_ALTITUDE, time_above)
		if time_above > BURN_TIME_ABOVE:
			break

	time.sleep(LOOP_DELAY)

print_status()
print('\n################# REACHED ALTITUDE: BURN STARTED #################\n')


###############################################################################
# Burn
###############################################################################

GPIO.output(BURN_PIN, GPIO.HIGH)


# vehicle.mode = VehicleMode("GUIDED")
vehicle.simple_goto(target_location)
cmds.next = 0
vehicle.mode = VehicleMode("AUTO")

prev_time_above_burn_alt = mytime()
while True:
	alt = vehicle.location.global_relative_frame.alt
	if alt > BURN_ALTITUDE: 
		prev_time_above_burn_alt = mytime()
	else: 
		time_below = mytime() - prev_time_above_burn_alt
		print('Below {}m for {} seconds').format(BURN_ALTITUDE, time_below)
		if time_below > BURN_TIME_BELOW:
			break

	time.sleep(LOOP_DELAY)

GPIO.output(BURN_PIN, GPIO.LOW)

print_status()
print('\n################# VEHICLE DISCONNECTED: BURN STOPPED #################\n')

###############################################################################
# Descent
###############################################################################

while True:
	print_status()
	time.sleep(1)