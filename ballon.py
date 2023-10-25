import time
import RPi.GPIO as GPIO
from pymavlink import mavutil
from dronekit import connect, VehicleMode

# Delay before launching the script (in seconds)
delay_before_launch = 60  # Adjust as needed

# Connection string for Pixhawk (adjust based on your setup)
connection_string = '/dev/serial0'  # Replace with your connection string

# Coordinates for the RTL location (replace with desired values)
rtl_lat = 12.3456  # Latitude in decimal degrees
rtl_lon = 78.9012  # Longitude in decimal degrees
rtl_altitude = 70  # RTL altitude in meters

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
while vehicle.is_armable:
    print("Waiting for vehicle to initialize...")
    time.sleep(1)

# Arm the vehicle
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

while not vehicle.armed:
    print("Arming the vehicle...")
    time.sleep(1)

# Listen to altitude and trigger servo at 300 meters
target_altitude = 300  # meters
servo_trigger_duration = 40  # seconds

while True:
    current_altitude = vehicle.location.global_relative_frame.alt
    if current_altitude >= target_altitude:
        print(f"Reached target altitude of {target_altitude} meters. Triggering servo...")
        GPIO.output(servo_pin, GPIO.HIGH)  # Activate the servo
        time.sleep(servo_trigger_duration)
        GPIO.output(servo_pin, GPIO.LOW)  # Deactivate the servo
        break
    time.sleep(1)

# Switch to RTL mode to specific lat-long
rtl_location = (rtl_lat, rtl_lon, rtl_altitude)
vehicle.parameters['RTL_LOCATION'] = rtl_location
vehicle.mode = VehicleMode("RTL")

# Close the connection
vehicle.close()
GPIO.cleanup()  # Cleanup GPIO pins when finished
