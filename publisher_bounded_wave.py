import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import numpy as np

LINK_NUMBER = 50

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_fin_01_dof_1 = [
            self.create_publisher(
                Float64, '/joints/fin_1_%d/cmd_pos' % (i + 1), 10)
            for i in range(LINK_NUMBER)
        ]
        self.publisher_fin_01_dof_2 = [
            self.create_publisher(
                Float64, '/joints/fin_1_%d_circular/cmd_pos' % (i + 1), 10)
            for i in range(LINK_NUMBER)
        ]
        self.publisher_fin_02_dof_1 = [
            self.create_publisher(
                Float64, '/joints/fin_2_%d/cmd_pos' % (i + 1), 10)
            for i in range(LINK_NUMBER)
        ]
        self.publisher_fin_02_dof_2 = [
            self.create_publisher(
                Float64, '/joints/fin_2_%d_circular/cmd_pos' % (i + 1), 10)
            for i in range(LINK_NUMBER)
        ]

        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0

    def procedure_01(self):
        # Bounded wave parameters
        L = LINK_NUMBER - 1  # Total length of the range
        x = np.linspace(0, L, LINK_NUMBER)  # x values
        A = 1  # Maximum amplitude of the quadratic bound
        alpha = 4 * A / L**2  # Quadratic coefficient for symmetry
        t = self.i * self.timer_period  # Time factor

        # Quadratic bounds
        upper_bound = -alpha * (x - L / 2)**2 + A
        lower_bound = -upper_bound

        # Compute bounded wave
        sine_wave = np.sin(2 * np.pi * (x / 30 - 0.1 * t))  # Time-shifted sine wave
        bounded_wave = sine_wave * upper_bound  # Apply bounds

        # Compute circular joints based on the bounded wave with a phase shift
        circular_joint = np.sin(
            2 * np.pi * (x / 30 - 0.1 * t) - np.pi / 2
        ) * upper_bound  # Circular wave shifted by π/2

        for iter, (wave_value, circular_value) in enumerate(zip(bounded_wave, circular_joint)):
            msg1 = Float64()
            msg1.data = wave_value  # Bounded wave value for DOF_1

            msg1_c = Float64()
            msg1_c.data = circular_value  # Circular wave value for DOF_1_CIRCULAR

            msg2 = Float64()
            msg2.data = -wave_value  # Bounded wave value for DOF_1

            msg2_c = Float64()
            msg2_c.data = circular_value  # Circular wave value for DOF_1_CIRCULAR

            # Publish messages
            self.publisher_fin_01_dof_1[iter].publish(msg1_c)
            self.publisher_fin_01_dof_2[iter].publish(msg1)
            self.publisher_fin_02_dof_1[iter].publish(msg2_c)
            self.publisher_fin_02_dof_2[iter].publish(msg2)

            # Log for debugging
            self.get_logger().info(
                f'Publishing joint {iter + 1}: DOF1={msg1.data}, DOF1_CIRCULAR={msg1_c.data}'
            )

        self.i += 1


    def timer_callback(self):
        self.procedure_01()


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
