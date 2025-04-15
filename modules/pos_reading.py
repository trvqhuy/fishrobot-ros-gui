import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import matplotlib.pyplot as plt

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
        self.subscription = self.create_subscription(
            Pose,
            '/world/fish_world/dynamic_pose/info',  # Change this to your actual topic name
            self.pose_callback,
            10)
        self.subscription  # prevent unused variable warning

    def pose_callback(self, msg):
        # if msg.name == "fish_body":
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z
        self.plot_position(x, y, z)

    def plot_position(self, x, y, z):
        plt.scatter(x, y)
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.title('Position of fish_body')
        plt.draw()
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)

    pose_subscriber = PoseSubscriber()

    rclpy.spin(pose_subscriber)

    pose_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
