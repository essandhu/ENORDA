import cv2
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import dronekit_sitl
import time

# Configure SITL environment
sitl = dronekit_sitl.start_default()
#connectionString = sitl.connection_string()

# Used to arm the drone
def armDrone():
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    print("Vehicle initialised")

    # Check if GPS is functioning
    while vehicle.gps_0.fix_type < 2:  # Ensure GPS is ready
        print(" Waiting for GPS to initialise...", vehicle.gps_0.fix_type)
        time.sleep(1)
    print("Copter GPS Ready")

    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.mode.name=='GUIDED' and not vehicle.armed:
        print(" Getting ready to take off ...")
        time.sleep(1)
    print("Vehicle armed and in GUIDED mode!")

# Used to take off the drone to a specific altitude
def takeoffDrone(targetAltitude):
    print("Taking off!")
    vehicle.simple_takeoff(targetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= targetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

# Function to move the drone to a specific location (global relative)
def goto_location(lat, lon, alt):
    target_location = LocationGlobalRelative(lat, lon, alt)
    vehicle.simple_goto(target_location)
    while True:
        current_location = vehicle.location.global_relative_frame
        distance = current_location.distance_to(target_location)
        if distance < 1:  # Adjust this threshold as needed
            break

# Placeholder for ArUco marker detection
def detect_aruco_markers(image):
    # Define the ArUco dictionary and parameters
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters_create()

    # Detect ArUco markers in the image
    corners, ids, _ = cv2.aruco.detectMarkers(image, aruco_dict, parameters=parameters)
    return corners, ids

# Used to begin searching process for challenge 1
def challenge1Search():
    print("Starting Challenge 1 Search")

    # Define field dimensions (in yards)
    field_width = 50
    field_height = 50

    # Define grid parameters
    grid_size = 5  # Divide the field into a 5x5 grid
    grid_step_x = field_width / grid_size
    grid_step_y = field_height / grid_size

    # Initialize drone's position at the bottom-left corner
    drone_x = 0
    drone_y = 0

    while True:
        # Move the drone to the current grid cell
        goto_location(drone_x, drone_y, 10)  # Assuming altitude of 10 meters

        # Capture an image from the drone's camera
        # needs to be added

        # Detect ArUco markers in the captured image
        corners, ids = detect_aruco_markers(frame)

        # Store detected ArUco markers and their positions
        if ids is not None:
            for i in range(len(ids)):
                marker_id = ids[i][0]
                marker_corners = corners[i][0]
                # Store marker_id and marker_corners in your data structure

        # Move the drone approximately 15 yards to the right
        drone_x += 15  # Adjust this distance as needed
        if drone_x >= field_width:
            # If at the right edge of the field, move forward and reset drone_x
            drone_x = 0
            drone_y += grid_step_y
            if drone_y >= field_height:
                break  # Break the loop if the end of the field is reached



'''MAIN'''
# Connect to the vehicle (in this case SITL simulator)
vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)

# Obtain vehicle attributes for testing
print("Autopilot Firmware version: %s" % vehicle.version)
print("Autopilot capabilities (supports ftp): %s" % vehicle.capabilities.ftp)
print("Global Location: %s" % vehicle.location.global_frame)
print("Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
print("Local Location: %s" % vehicle.location.local_frame)    #NED
print("Attitude: %s" % vehicle.attitude)
print("Velocity: %s" % vehicle.velocity)
print("GPS: %s" % vehicle.gps_0)
print("Groundspeed: %s" % vehicle.groundspeed)
print("Airspeed: %s" % vehicle.airspeed)
print("Gimbal status: %s" % vehicle.gimbal)
print("Battery: %s" % vehicle.battery)
print("EKF OK?: %s" % vehicle.ekf_ok)
print("Last Heartbeat: %s" % vehicle.last_heartbeat)
print("Rangefinder: %s" % vehicle.rangefinder)
print("Rangefinder distance: %s" % vehicle.rangefinder.distance)
print("Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
print("Heading: %s" % vehicle.heading)
print("Is Armable?: %s" % vehicle.is_armable)
print("System status: %s" % vehicle.system_status.state)
print("Mode: %s" % vehicle.mode.name)    # settable
print("Armed: %s" % vehicle.armed)    # settable

# Arm the drone
armDrone()

# Take off to 10m
takeoffDrone(10)

# Begin challenge 1 search
challenge1Search()

# Close vehicle object when finished running script
vehicle.mode = VehicleMode("LAND")
vehicle.close()

# Close the simulator
sitl.stop()