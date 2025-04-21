import rclpy
import os
import csv
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseArray
import matplotlib
import numpy #pip install "numpy<2"
import matplotlib.pyplot as plt
import threading
import time

print("Matplotlib version:", matplotlib.__version__)
print("NumPy version:", numpy.__version__)

class ImuListener(Node):

    def __init__(self):
        super().__init__('imu_listener')
        
        # self.subscription_imu = self.create_subscription(
        #     Imu,
        #     '/imu',
        #     self.listener_callback_imu,
        #     10)

        self.subscription_pose = self.create_subscription(
            PoseArray,
            '/world/fish_world/dynamic_pose/info',
            self.listener_callback_pose,
            10)
        self.pos_times = []

        self.orientations = {'x': [], 'y': [], 'z': [], 'w': []}
        self.angular_velocities = {'x': [], 'y': [], 'z': []}
        self.positions = {'x': [], 'y': [], 'z': []}
        self.linear_velocities = {'x': [], 'y': [], 'z': []}

        # self.linear_accelerations = {'x': [], 'y': [], 'z': []}

    # def listener_callback_imu(self, msg):
    #     current_time = time.time()
    #     self.times.append(current_time)

    #     self.linear_accelerations['x'].append(msg.linear_acceleration.x)
    #     self.linear_accelerations['y'].append(msg.linear_acceleration.y)
    #     self.linear_accelerations['z'].append(msg.linear_acceleration.z)

    def listener_callback_pose(self, msg):
        current_time = time.time()
        self.pos_times.append(current_time)

        pose = msg.poses[0]
        self.positions['x'].append(pose.position.x)
        self.positions['y'].append(pose.position.y)
        self.positions['z'].append(pose.position.z)

        self.orientations['x'].append(pose.orientation.x)
        self.orientations['y'].append(pose.orientation.y)
        self.orientations['z'].append(pose.orientation.z)
        self.orientations['w'].append(pose.orientation.w)

        low_pass_alpha = 0.1
        derivative_num = 10

        if len(self.positions['x']) > derivative_num:
            dt = current_time - self.pos_times[-1-derivative_num]
            dx = self.positions['x'][-1] - self.positions['x'][-1-derivative_num]
            dy = self.positions['y'][-1] - self.positions['y'][-1-derivative_num]
            dz = self.positions['z'][-1] - self.positions['z'][-1-derivative_num]
            vx = dx / dt * low_pass_alpha + self.linear_velocities['x'][-1] * (1 - low_pass_alpha)
            vy = dy / dt * low_pass_alpha + self.linear_velocities['y'][-1] * (1 - low_pass_alpha)
            vz = dz / dt * low_pass_alpha + self.linear_velocities['z'][-1] * (1 - low_pass_alpha)
            self.linear_velocities['x'].append(vx)
            self.linear_velocities['y'].append(vy)
            self.linear_velocities['z'].append(vz)

            # Calculating angular velocities
            dt = current_time - self.pos_times[-1-derivative_num]
            d_orientation_x = self.orientations['x'][-1] - self.orientations['x'][-1-derivative_num]
            d_orientation_y = self.orientations['y'][-1] - self.orientations['y'][-1-derivative_num]
            d_orientation_z = self.orientations['z'][-1] - self.orientations['z'][-1-derivative_num]
            angular_velocity_x = d_orientation_x / dt * low_pass_alpha + self.angular_velocities['x'][-1] * (1 - low_pass_alpha)
            angular_velocity_y = d_orientation_y / dt * low_pass_alpha + self.angular_velocities['y'][-1] * (1 - low_pass_alpha)
            angular_velocity_z = d_orientation_z / dt * low_pass_alpha + self.angular_velocities['z'][-1] * (1 - low_pass_alpha)
            self.angular_velocities['x'].append(angular_velocity_x)
            self.angular_velocities['y'].append(angular_velocity_y)
            self.angular_velocities['z'].append(angular_velocity_z)
        else:
            self.linear_velocities['x'].append(0.0)
            self.linear_velocities['y'].append(0.0)
            self.linear_velocities['z'].append(0.0)
            self.angular_velocities['x'].append(0.0)
            self.angular_velocities['y'].append(0.0)
            self.angular_velocities['z'].append(0.0)

def get_new_file_path(base_path):
    if not os.path.exists(base_path):
        return base_path
    counter = 1
    while True:
        new_path = f"{os.path.splitext(base_path)[0]}_{counter}.csv"
        if not os.path.exists(new_path):
            return new_path
        counter += 1

def plot_data(node):
    plt.ion()
    fig, axs = plt.subplots(2, 2, figsize=(10, 15))
    
    base_file_path = 'data.csv'
    file_path = get_new_file_path(base_file_path)

    # Wait until at least one pose message has been received
    while rclpy.ok() and not node.pos_times:
        time.sleep(0.1)

    # Open the CSV file in append mode
    with open(file_path, mode='a', newline='') as file:
        writer = csv.writer(file)
        
        # Write the header only if the file is empty
        if file.tell() == 0:
            writer.writerow([
                'Time (seconds)', 
                'Orientation X', 'Orientation Y', 'Orientation Z', 
                'Angular Velocity X', 'Angular Velocity Y', 'Angular Velocity Z',
                'Position X', 'Position Y', 'Position Z',
                'Linear Velocity X', 'Linear Velocity Y', 'Linear Velocity Z'
            ])

        while rclpy.ok():
            times_seconds = node.pos_times
            start_time = times_seconds[0]
            times_seconds = [(t - start_time) for t in times_seconds]

            min_length = min(len(times_seconds), 
                             len(node.orientations['x']), len(node.orientations['y']), len(node.orientations['z']), 
                             len(node.angular_velocities['x']), len(node.angular_velocities['y']), len(node.angular_velocities['z']), 
                             len(node.positions['x']), len(node.positions['y']), len(node.positions['z']),
                             len(node.linear_velocities['x']), len(node.linear_velocities['y']), len(node.linear_velocities['z']))

            orientations = {key: val[:min_length] for key, val in node.orientations.items()}
            angular_velocities = {key: val[:min_length] for key, val in node.angular_velocities.items()}
            positions = {key: val[:min_length] for key, val in node.positions.items()}
            linear_velocities = {key: val[:min_length] for key, val in node.linear_velocities.items()}
            times_seconds = times_seconds[:min_length]

            axs[0, 0].clear()
            axs[0, 0].plot(times_seconds, orientations['x'], label='Orientation X')
            axs[0, 0].plot(times_seconds, orientations['y'], label='Orientation Y')
            axs[0, 0].plot(times_seconds, orientations['z'], label='Orientation Z')
            axs[0, 0].set_ylabel('Orientation')
            axs[0, 0].legend(loc='upper right')
            axs[0, 0].set_xlabel('Time (seconds)')

            axs[0, 1].clear()
            axs[0, 1].plot(times_seconds, positions['x'], label='Position X')
            axs[0, 1].plot(times_seconds, positions['y'], label='Position Y')
            axs[0, 1].plot(times_seconds, positions['z'], label='Position Z')
            axs[0, 1].set_ylabel('Position')
            axs[0, 1].legend(loc='upper right')
            axs[0, 1].set_xlabel('Time (seconds)')

            writer.writerow([
                times_seconds[-1], 
                orientations['x'][-1], orientations['y'][-1], orientations['z'][-1],
                angular_velocities['x'][-1], angular_velocities['y'][-1], angular_velocities['z'][-1],
                positions['x'][-1], positions['y'][-1], positions['z'][-1],
                linear_velocities['x'][-1], linear_velocities['y'][-1], linear_velocities['z'][-1]
            ])

            axs[1, 0].clear()
            axs[1, 0].plot(times_seconds, angular_velocities['x'], label='Angular Velocity X')
            axs[1, 0].plot(times_seconds, angular_velocities['y'], label='Angular Velocity Y')
            axs[1, 0].plot(times_seconds, angular_velocities['z'], label='Angular Velocity Z')
            axs[1, 0].set_ylabel('Angular Velocity')
            axs[1, 0].legend(loc='upper right')
            axs[1, 0].set_xlabel('Time (seconds)')

            axs[1, 1].clear()
            axs[1, 1].plot(times_seconds, linear_velocities['x'], label='Velocity X')
            axs[1, 1].plot(times_seconds, linear_velocities['y'], label='Velocity Y')
            axs[1, 1].plot(times_seconds, linear_velocities['z'], label='Velocity Z')
            axs[1, 1].set_ylabel('Linear Velocity')
            axs[1, 1].legend(loc='upper right')
            axs[1, 1].set_xlabel('Time (seconds)')

            plt.draw()
            plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    imu_listener = ImuListener()

    # Start ROS spinning in a separate thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(imu_listener,))
    ros_thread.start()

    try:
        # Run plotting in main thread (to avoid GUI issues)
        plot_data(imu_listener)
    except KeyboardInterrupt:
        print("Exiting...")

    # Shutdown
    imu_listener.destroy_node()
    rclpy.shutdown()
    ros_thread.join()

if __name__ == '__main__':
    main()
