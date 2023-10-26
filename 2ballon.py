import time
import RPi.GPIO as GPIO
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative
import numpy as np

# Delay before launching the script (in seconds)
delay_before_launch = 60  # Adjust as needed

# Connection string for Pixhawk (adjust based on your setup)
connection_string = '/dev/serial0'  # Replace with your connection string

# Coordinates for the RTL location (replace with desired values)
rtl_lat = 12.3456  # Latitude in decimal degrees
rtl_lon = 78.9012  # Longitude in decimal degrees
rtl_altitude = 70  # RTL altitude in meters

start_time = time.time()
target_location = LocationGlobalRelative(rtl_lat, rtl_lon, 0)

def mytime():
	return time.time() - start_time

def print_status():
	print('Time: {:.2f}s\tMode: {}\tSats: {}\tAlt: {}m\tLoc: ({}, {})\tdist: {:.3f}km.').format(
		mytime(),
		vehicle.mode.name,
		vehicle.gps_0.satellites_visible,
		vehicle.location.global_relative_frame.alt,
		vehicle.location.global_frame.lat,
		vehicle.location.global_frame.lon,
		straight_line_dist(target_location, vehicle.location.global_frame))

# Servo pin configuration (adjust as needed)
servo_pin = 17  # Raspberry Pi GPIO pin (adjust as needed)

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)
GPIO.output(servo_pin, GPIO.LOW)

# Connect to the Pixhawk flight controller
vehicle = None
try:
    vehicle = connect(connection_string, baud=57600, wait_ready=True)
except Exception as e:
    print(f"Failed to connect to Pixhawk: {e}")
    exit(1)

# Delay before executing the mission
print(f"Waiting for {delay_before_launch} seconds before starting the mission...")
time.sleep(delay_before_launch)

# Perform pre-arm checks
while not vehicle.is_armable:
    print("Waiting for vehicle to initialize...")
    time.sleep(1)

# Arm the vehicle
vehicle.mode = VehicleMode("MANUAL")

while not vehicle.mode.name == "MANUAL":
    vehicle.channels.overrides['2'] = np.random.randint(1300, 1700)
    vehicle.channels.overrides['1'] = np.random.randint(1300, 1700)
    time.sleep(0.2)
    print('waiting for MANUAL mode')
    time.sleep(1)

while vehicle.gps_0.satellites_visible < 5:
	print('{} satellites aquired. Waiting for {}.').format(vehicle.gps_0.satellites_visible, 5)
	time.sleep(1)

while not vehicle.gps_0.satellites_visible < 5:
     print('gps good to go')
     
vehicle.armed = True

while not vehicle.armed:
    print("Arming the vehicle...")
    time.sleep(1)
    

# Listen to altitude and trigger servo at 300 meters
target_altitude = 1  # meters
servo_trigger_duration = 40  # seconds

alt_buffer_len 	= int(60/0.5)
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

	if alt < target_altitude: 
		prev_time_below_burn_alt = mytime()
	else: 
		time_above = mytime() - prev_time_below_burn_alt
		print('Above {}m for {} seconds').format(target_altitude, time_above)
		if time_above > 5:
			break

	time.sleep(0.5)

print_status()

print('REACHED ALTITUDE: BURN STARTED')



GPIO.output(servo_pin, GPIO.HIGH)


# vehicle.mode = VehicleMode("GUIDED")
vehicle.simple_goto(target_location)
vehicle.mode = VehicleMode("AUTO")

prev_time_above_burn_alt = mytime()
while True:
	alt = vehicle.location.global_relative_frame.alt
	if alt > target_altitude: 
		prev_time_above_burn_alt = mytime()
	else: 
		time_below = mytime() - prev_time_above_burn_alt
		print('Below {}m for {} seconds'.format(target_altitude, time_below))
		if time_below > 3:
			break

	time.sleep(0.5)

GPIO.output(servo_pin, GPIO.LOW)

print_status()
print('VEHICLE DISCONNECTED: BURN STOPPED')
GPIO.cleanup()  # Cleanup GPIO pins when finished


# Switch to RTL mode to specific lat-long
rtl_location = (rtl_lat, rtl_lon)
# vehicle.parameters['RTL_LOCATION'] = rtl_location
vehicle.mode = VehicleMode("RTL")

while True:
	print('initiating RTL mode')

# Close the connection
# vehicle.close()