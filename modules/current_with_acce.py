import subprocess
import time
import math

# Initial values
acceleration_y = 0.2  # constant acceleration
publish_interval = 0.05  # time in seconds between publishes
start_time = time.time()  # record the start time

# Function to publish the current velocity
def publish_velocity(velocity_y):
    # Use subprocess to call the ign command
    command = [
        "ign", "topic", "-t", "/ocean_current", 
        "-m", "ignition.msgs.Vector3d", 
        "-p", f"x: {velocity_y}, y: 0, z: 0"
    ]
    subprocess.run(command)

# Function to publish the current velocity
def publish_velocity_2(alpha, time):
    # Use subprocess to call the ign command
    command = [
        "ign", "topic", "-t", "/ocean_current", 
        "-m", "ignition.msgs.Vector3d", 
        "-p",
        f"x: {round(alpha*math.sin(2*math.pi*time/10.0), 4)},"
        f" y: {round(alpha*math.cos(2*math.pi*time/10.0), 4)},"
        f" z: {alpha}"
    ]
    subprocess.run(command)

# Main loop to periodically publish velocity
try:
    while True:
        elapsed_time = time.time() - start_time  # calculate elapsed time
        velocity_y = acceleration_y * elapsed_time  # update velocity based on elapsed time
        publish_velocity_2(0.2, elapsed_time)
        time.sleep(publish_interval)  # wait before publishing again
except KeyboardInterrupt:
    print("Script terminated by user.")
    publish_velocity(0)
