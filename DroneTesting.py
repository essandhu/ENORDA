from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import dronekit_sitl
import time
from GetEnemies import GetEnemy

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

    # Initialize drone's position at the bottom-left corner
    drone_x = 0
    drone_y = 0

    # Create instance of GetEnemy class for market detection
    aruco_detector = GetEnemy()
    aruco_detector.__init__()

    # Main loop for the search
    while True:
        # Move the drone to the current grid cell
        goto_location(drone_x, drone_y, 10)  # Assuming altitude of 10 meters

        # Detect ArUco markers in the captured image using the GetEnemy class
        updated, closest_enemy_id, min_enemy_distance = aruco_detector.getClosestEnemy()

        if updated and closest_enemy_id is not None:
            # Move the drone to the location of the closest enemy ArUco marker
            vector_x, vector_y = aruco_detector.getVectorToMarker(closest_enemy_id)
            goto_location(drone_x + vector_x, drone_y + vector_y, 10)  # Move to marker location

            # Pause for about 5 seconds at the marker location
            time.sleep(5)

        # Move the drone along the x-axis
        drone_x += grid_step_x 
        if drone_x >= field_width:
            # If at the right edge of the field, move forward and reset drone_x
            drone_x = 0
            drone_y += grid_step_y
            if drone_y >= field_height:
                break  # Break the loop if the end of the field is reached

'''MAIN'''
# Connect to the vehicle (in this case SITL simulator)
vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)

# Connect to copter for testing
#print("Connecting to Drone")
#vehicle = connect('/dev/ttyTHS2', wait_ready=True, baud=1500000)
#print("Connected")

# Arm the drone
armDrone()

# Take off to 10m
takeoffDrone(10)

# Begin challenge 1 search
challenge1Search()
print("Challenge 1 Search Complete")

# Close vehicle object when finished running script
vehicle.mode = VehicleMode("LAND")
vehicle.close()

# Close the simulator
sitl.stop()