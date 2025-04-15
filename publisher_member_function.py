
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from random import random
import numpy as np

LINK_NUMBER = 50

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_fin_01_dof_1 = [
            self.create_publisher(
                Float64, '/joints/fin_1_%d/cmd_pos' %(i+1), 10)
            for i in range(LINK_NUMBER)
        ]
        self.publisher_fin_01_dof_2 = [
            self.create_publisher(
                Float64, '/joints/fin_1_%d_circular/cmd_pos' %(i+1), 10)
            for i in range(LINK_NUMBER)
        ]
        self.publisher_fin_02_dof_1 = [
            self.create_publisher(
                Float64, '/joints/fin_2_%d/cmd_pos' %(i+1), 10)
            for i in range(LINK_NUMBER)
        ]
        self.publisher_fin_02_dof_2 = [
            self.create_publisher(
                Float64, '/joints/fin_2_%d_circular/cmd_pos' %(i+1), 10)
            for i in range(LINK_NUMBER)
        ]

        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0


    def procedure_01(self):
        forward_movement = True
        spin_movement = False

        fre_1 = 1
        fre_2 = 1
        scale_factor = 1

        wave_num = 1
        max_amplitude = 30 * (np.pi / 180.0) / 2.0
        c_max_amplitude = max_amplitude * (wave_num + 1)/2

        forward_sf = True

        shifted_amplitude = 60 * (np.pi / 180.0)
        shifted_frequency = 0
        shifted_upward = True

        publisher_list = (
            self.publisher_fin_01_dof_1
            if forward_movement else
            self.publisher_fin_01_dof_1[:-1])

        total_num = len(self.publisher_fin_01_dof_1)

        if spin_movement:
            for iter, _ in enumerate(publisher_list):
                current_amp = max_amplitude * (
                    scale_factor
                    + (total_num-iter if forward_sf else iter)
                    * (1-scale_factor)
                    / total_num
                )

                wave_function_01 = current_amp * np.sin(
                    2 * np.pi * fre_1 * self.timer_period * self.i
                    + 2 * np.pi * ((total_num-iter) * wave_num / total_num)
                ) + abs(shifted_amplitude) * (-1 if shifted_upward else 1)

                wave_function_02 = current_amp * np.sin(
                    2 * np.pi * fre_2 * self.timer_period * self.i
                    + 2 * np.pi * (iter * wave_num / total_num)
                ) + abs(shifted_amplitude) * (-1 if shifted_upward else 1)

                circular_joint_01 = c_max_amplitude * np.sin(
                    2 * np.pi * fre_1 * self.timer_period * self.i
                    + 2 * np.pi * ((total_num-iter) * wave_num / total_num)
                    + np.pi/2
                )

                circular_joint_02 = c_max_amplitude * np.sin(
                    2 * np.pi * fre_1 * self.timer_period * self.i
                    + 2 * np.pi * (iter * wave_num / total_num)
                    - np.pi/2
                )

                sine_func_1 = wave_function_01
                sine_func_2 = wave_function_02
                c_sine_func_1 = circular_joint_01
                c_sine_func_2 = circular_joint_02

                msg1 = Float64()
                msg1.data = sine_func_1
                msg1_c = Float64()
                msg1_c.data = c_sine_func_1

                msg2 = Float64()
                msg2.data = -sine_func_2
                msg2_c = Float64()
                msg2_c.data = c_sine_func_2

                self.publisher_fin_01_dof_1[iter].publish(msg1_c)
                self.publisher_fin_01_dof_2[iter].publish(msg1)
                
                self.publisher_fin_02_dof_1[iter].publish(msg2_c)
                self.publisher_fin_02_dof_2[iter].publish(msg2)

        else:
            for iter, _ in enumerate(publisher_list):
                current_amp = max_amplitude * (
                    scale_factor
                    + (total_num-iter if forward_sf else iter)
                    * (1-scale_factor)
                    / total_num
                )

                wave_function_01 = current_amp * np.sin(
                    2 * np.pi * fre_1 * self.timer_period * self.i
                    + 2 * np.pi * ((iter) * wave_num / total_num)
                ) + abs(shifted_amplitude) * (-1 if shifted_upward else 1)

                wave_function_02 = current_amp * np.sin(
                    2 * np.pi * fre_2 * self.timer_period * self.i
                    + 2 * np.pi * (iter * wave_num / total_num)
                ) + abs(shifted_amplitude) * (-1 if shifted_upward else 1)

                circular_joint_01 = c_max_amplitude * np.sin(
                    2 * np.pi * fre_1 * self.timer_period * self.i
                    + 2 * np.pi * ((iter) * wave_num / total_num)
                    - np.pi/2
                )

                circular_joint_02 = c_max_amplitude * np.sin(
                    2 * np.pi * fre_1 * self.timer_period * self.i
                    + 2 * np.pi * (iter * wave_num / total_num)
                    - np.pi/2
                )

                sine_func_1 = wave_function_01
                sine_func_2 = wave_function_02
                c_sine_func_1 = circular_joint_01
                c_sine_func_2 = circular_joint_02

                msg1 = Float64()
                msg1.data = sine_func_1
                msg1_c = Float64()
                msg1_c.data = c_sine_func_1

                msg2 = Float64()
                msg2.data = -sine_func_2
                msg2_c = Float64()
                msg2_c.data = c_sine_func_2

                self.publisher_fin_01_dof_1[iter].publish(msg1_c)
                self.publisher_fin_01_dof_2[iter].publish(msg1)
                
                self.publisher_fin_02_dof_1[iter].publish(msg2_c)
                self.publisher_fin_02_dof_2[iter].publish(msg2)

        self.i += 1
        self.get_logger().info('Publishing: "%s"' % msg1.data)

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
