from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import numpy as np
import time
import argparse
from pymavlink import mavutil
from GetEnemies import GetEnemy

# Used to connect to copter with args from command line
def connectMyCopter():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = args.connect  # Gives value after --connect; the IP address

    if not connection_string:  # If the connection string is empty; none provided
        # Create a SITL drone instance instead of launching one beforehand
        import dronekit_sitl
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()

    vehicle = connect(connection_string, wait_ready=True)
    return vehicle

# Used to arm the drone
def armDrone():
    while vehicle.is_armable == False:  # While the drone hasn't been armed
        print("Waiting for drone to become armable")
        time.sleep(1)  # Wait one second before checking if drone is armable
    print("The drone is now armable")

    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != 'GUIDED':  # While drone is not in guided mode
        print("The drone is not in guided mode yet")
        time.sleep(1)  # Wait one second before checking if drone is in guided mode
    print("The drone is now in guided mode")

    vehicle.armed = True
    while vehicle.armed == False:  # While the vehicle has not been armed
        print("Waiting for drone to arm")
        time.sleep(1)  # Wait one second before checking if drone has been armed
    print("The drone has now been armed")

    # Check if GPS is functioning
    while vehicle.gps_0.fix_type < 2:  # Ensure GPS is ready
        print(" Waiting for GPS to initialise...", vehicle.gps_0.fix_type)
        time.sleep(1)
    print("Copter GPS Ready")

# Used to take off the drone to a specific altitude
def takeoffDrone(targetAltitude):
    print("Taking off!")
    vehicle.simple_takeoff(targetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    # after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
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

    # Get the initial GPS coordinates of the drone
    initial_latitude = vehicle.location.global_frame.lat
    initial_longitude = vehicle.location.global_frame.lon

    # Initialize the drone's current position in yards
    drone_x = 0
    drone_y = 0

    # Create instance of GetEnemy class for market detection
    aruco_detector = GetEnemy()
    aruco_detector.__init__()

    # Set to keep track of detected enemy markers
    detected_markers = set()

    # Main loop for the search
    while True:
        # Move the drone to the current grid cell
        goto_location(initial_latitude + (drone_x / 1.0936 / 111111), initial_longitude + (drone_y / (1.0936 / np.cos(np.radians(initial_latitude))) / 111111), 6)  # Assuming altitude of 6 meters

        # Detect ArUco markers in the captured image using the GetEnemy class
        updated, closest_enemy_id, min_enemy_distance = aruco_detector.getClosestEnemy()

        # If a marker has been detected
        if updated and closest_enemy_id is not None:
            # Check if the marker has not alrady been detected in the current session
            if closest_enemy_id not in detected_markers:
                # Move the drone to the location of the closest enemy ArUco marker
                vector_x, vector_y = aruco_detector.getVectorToMarker(closest_enemy_id)
                goto_location(drone_x + vector_x, drone_y + vector_y, 4) 

                # Pause for about 5 seconds at the marker location (for water blast)
                time.sleep(5)

                # Add the marker to the set of detected markers for this session
                detected_markers.add(closest_enemy_id)

        # Move the drone along the x-axis
        drone_x += grid_step_x 
        if drone_x >= field_width:
            # If at the right edge of the field, move forward and reset drone_x
            drone_x = 0
            drone_y += grid_step_y
            if drone_y >= field_height:
                break  # Break the loop if the end of the field is reached

    # Close the camera
    aruco_detector.close()

'''MAIN'''
# Connect to the vehiclem (simulator)
#vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)
#vehicle = connect('/dev/ttyTHS2', wait_ready=True, baud=1500000)

# Connect to copter for testing
print("Connecting to Drone")
vehicle = connectMyCopter()
print("Connected!")

# Arm the drone
armDrone()

# Take off to 6m
takeoffDrone(6)

# Begin challenge 1 search
challenge1Search()
print("Challenge 1 Search Complete")

# Close vehicle object when finished running script
vehicle.mode = VehicleMode("LAND")
vehicle.close()