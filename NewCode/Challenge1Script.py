from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import numpy as np
import time
import argparse
import sys
from geopy.distance import geodesic # For calculating distance between two GPS coordinates
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
def arm_drone(vehicle):
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
def takeoff_drone(vehicle, targetAltitude):
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

# Used to land the drone, prints height every second while descending
def land_drone(vehicle):
    print("Setting copter into LAND mode")
    vehicle.mode = VehicleMode('LAND')
    while vehicle.mode != 'LAND':
        time.sleep(1)
    print("Initiating landing now...")

    while vehicle.armed:  # While the drone has not landed
        currDroneHeight = vehicle.location.global_relative_frame.alt
        print("Current drone elevation: ", currDroneHeight)
        time.sleep(1)
	
    print("The copter has landed!")

# Function to move the drone to a specific location (global relative)
def goto_location(vehicle, lat, lon, alt):
    target_location = LocationGlobalRelative(lat, lon, alt)
    vehicle.simple_goto(target_location)
    while True:
        current_location = vehicle.location.global_relative_frame
        distance = current_location.distance_to(target_location)
        if distance < 1:  # Adjust this threshold as needed
            break

"""
    Read bounding box coordinates from a CSV file.
    Args:
        file_path (str): Path to the CSV file containing bounding box coordinates.

    Returns:
        list: Bounding box coordinates as a list [lon_min, lat_min, lon_max, lat_max].
    """
def read_bbox_from_csv(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
        bbox = [float(coordinate) for coordinate in lines[0].strip().split(',')]
        return bbox

"""
Calculate a grid within the bounding box with specified minimum distances between rows and columns.
Args:
    bbox (list): List of 4 coordinates defining the bounding box [lon_min, lat_min, lon_max, lat_max].
    min_row_dist (float): Minimum distance in meters between rows.
    min_column_dist (float): Minimum distance in meters between columns.

Returns:
    grid_points: Array of waypoints representing the grid points within the bounding box.
"""
def calculate_search_grid(bbox, min_row_dist, min_column_dist):
    # Determine grid parameters based on input waypoints
    lon_min, lat_min, lon_max, lat_max = bbox
    total_width = geodesic((lat_min, lon_min), (lat_min, lon_max)).meters
    total_height = geodesic((lat_min, lon_min), (lat_max, lon_min)).meters
    num_rows = int(total_height // min_row_dist) + 1
    num_columns = int(total_width // min_column_dist) + 1
    grid_points = np.zeros((num_rows * num_columns, 2))

    # Calculate grid points
    for row in range(num_rows):
        for col in range(num_columns):
            lat = lat_min + (row * min_row_dist / 111111)
            lon = lon_min + (col * min_column_dist / (111111 * np.cos(np.radians(lat))))
            grid_points[row * num_columns + col] = [lat, lon]

    return grid_points

# Used to begin searching process for challenge 1
def challenge1Search(vehicle):
    print("Starting Challenge 1 Search")

    # Path to CSV file containing bounding box coordinates
    input_file = sys.argv[1] 

    # Read bounding box coordinates from the CSV file
    bbox = read_bbox_from_csv(input_file)

    # Define the minimum distances between rows and columns (adjust based on challenge and search area)
    min_row_dist = 1.25  # Minimum spacing in meters between grid rows
    min_column_dist = 3  # Minimum spacing in meters between grid columns

    # Calculate the search grid
    grid_points = calculate_search_grid(bbox, min_row_dist, min_column_dist)

    # Create instance of GetEnemy class for marker detection
    aruco_detector = GetEnemy()
    aruco_detector.__init__()

    # Set to keep track of detected enemy markers
    detected_markers = set()

    # Main loop for the search
    for point in grid_points:
        # Move the drone to the current grid cell
        lat, lon = point
        goto_location(vehicle, lat, lon, 7)  # Assuming an altitude of 7 meters

        # Detect ArUco markers in the captured image using the GetEnemy class
        updated, closest_enemy_id, min_enemy_distance = aruco_detector.getClosestEnemy()

        # If a marker has been detected
        if updated and closest_enemy_id is not None:
            # Check if the marker has not alrady been detected in the current session
            if closest_enemy_id not in detected_markers:
                # Log the detected marker
                print("Detected marker with ID: ", closest_enemy_id, " at distance: ", min_enemy_distance, " meters")

                # Move the drone to the location of the closest enemy ArUco marker
                vector_x, vector_y = aruco_detector.getVectorToMarker(closest_enemy_id)
                goto_location(lon + vector_x, lat + vector_y, 4) 

                # Pause for about 5 seconds at the marker location (for water blast)
                time.sleep(5)

                # Check for hit on ground vehicle
                    # if hit: log successful hit and continue search
                    # else: wait a few seconds and try again

                # Add the marker to the set of detected markers for this session
                detected_markers.add(closest_enemy_id)

    # Close the camera
    aruco_detector.close()

'''MAIN'''
# Connect to the vehiclem (simulator)
#vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)

# Connect to copter for testing
print("Connecting to Drone")
copter = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)
print("Connected!")

# Arm the drone
arm_drone(copter)

# Take off to 7m
takeoff_drone(copter, 7)

# Begin challenge 1 search
challenge1Search(copter)
print("Challenge 1 Search Complete")

# Close vehicle object when finished running script
land_drone(copter)
copter.close()