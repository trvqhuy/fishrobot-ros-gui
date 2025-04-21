import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import numpy as np
import json

CONFIG_FILE = "config/gui_config.json"

def load_motion_config():
    try:
        with open(CONFIG_FILE) as f:
            params = json.load(f)
            return {
                "link_number": int(params.get("link_number", 50)),
                "wave_type": params.get("wave_type", "sine"),
                "wave_mode": params.get("wave_mode", "traveling"),
                "wave_number": float(params.get("wave_number", 1.0)),
                "membrane_1": {
                    "frequency": float(params["membrane_1"].get("frequency", 1.0)),
                    "base_amplitude": float(params["membrane_1"].get("base_amplitude", 30.0)) * np.pi / 180.0,
                    "shifted_amplitude": float(params["membrane_1"].get("shifted_amplitude", 0.0)) * np.pi / 180.0,
                    "scale_factor": float(params["membrane_1"].get("scale_factor", 1.0)),
                    "forward_sf": bool(params["membrane_1"].get("forward_sf", True)),
                    "phase_offset": float(params["membrane_1"].get("phase_offset", 0.0)) * np.pi / 180.0
                },
                "membrane_2": {
                    "frequency": float(params["membrane_2"].get("frequency", 1.0)),
                    "base_amplitude": float(params["membrane_2"].get("base_amplitude", 30.0)) * np.pi / 180.0,
                    "shifted_amplitude": float(params["membrane_2"].get("shifted_amplitude", 0.0)) * np.pi / 180.0,
                    "scale_factor": float(params["membrane_2"].get("scale_factor", 1.0)),
                    "forward_sf": bool(params["membrane_2"].get("forward_sf", True)),
                    "phase_offset": float(params["membrane_2"].get("phase_offset", 0.0)) * np.pi / 180.0
                }
            }
    except Exception as e:
        print("⚠️ Failed to load motion config:", e)
        return {}

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.params = load_motion_config()
        self.link_number = self.params["link_number"]

        self.publisher_fin_01_dof_1 = [self.create_publisher(Float64, f'/joints/fin_1_{i+1}/cmd_pos', 10) for i in range(self.link_number)]
        self.publisher_fin_01_dof_2 = [self.create_publisher(Float64, f'/joints/fin_1_{i+1}_circular/cmd_pos', 10) for i in range(self.link_number)]
        self.publisher_fin_02_dof_1 = [self.create_publisher(Float64, f'/joints/fin_2_{i+1}/cmd_pos', 10) for i in range(self.link_number)]
        self.publisher_fin_02_dof_2 = [self.create_publisher(Float64, f'/joints/fin_2_{i+1}_circular/cmd_pos', 10) for i in range(self.link_number)]

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0

    def bounded_wave(self, x, t, alpha, A, freq, wave_num, shifted_amp, phase_offset, mode):
        shift = abs(shifted_amp)
        if mode == "standing":
            temporal = np.sin(2 * np.pi * freq * t + phase_offset)
            circ_temporal = np.cos(2 * np.pi * freq * t + phase_offset)
            envelope = -alpha * (x - self.link_number / 2) ** 2 + A
            return temporal * envelope + shift, circ_temporal * envelope
        else:
            phase = 2 * np.pi * (x * wave_num / self.link_number - freq * t) + phase_offset
            circ_phase = phase - np.pi / 2
            envelope = -alpha * (x - self.link_number / 2) ** 2 + A
            return np.sin(phase) * envelope + shift, np.sin(circ_phase) * envelope

    def execute(self):
        p = self.params
        x = np.linspace(0, self.link_number - 1, self.link_number)
        t = self.i * self.timer_period

        wave_type = p["wave_type"]
        wave_mode = p["wave_mode"]
        wave_num = p["wave_number"]
        m1 = p["membrane_1"]
        m2 = p["membrane_2"]

        if wave_type == "elliptic":
            alpha1 = 4 * m1["base_amplitude"] / ((self.link_number - 1) ** 2)
            alpha2 = 4 * m2["base_amplitude"] / ((self.link_number - 1) ** 2)

            wave1, circ1 = self.bounded_wave(x, t, alpha1, m1["base_amplitude"], m1["frequency"], wave_num, m1["shifted_amplitude"], m1["phase_offset"], wave_mode)
            wave2, circ2 = self.bounded_wave(x, t, alpha2, m2["base_amplitude"], m2["frequency"], wave_num, m2["shifted_amplitude"], m2["phase_offset"], wave_mode)

            for i in range(self.link_number):
                self.publisher_fin_01_dof_1[i].publish(Float64(data=circ1[i]))
                self.publisher_fin_01_dof_2[i].publish(Float64(data=wave1[i]))
                self.publisher_fin_02_dof_1[i].publish(Float64(data=circ2[i]))
                self.publisher_fin_02_dof_2[i].publish(Float64(data=-wave2[i]))
        else:
            circ_amp1 = m1["base_amplitude"] * (wave_num + 1) / 2.0
            circ_amp2 = m2["base_amplitude"] * (wave_num + 1) / 2.0

            for i in range(self.link_number):
                idx1 = (self.link_number - i) if m1["forward_sf"] else i
                idx2 = (self.link_number - i) if m2["forward_sf"] else i

                amp1 = m1["base_amplitude"] * (m1["scale_factor"] + idx1 * (1 - m1["scale_factor"]) / self.link_number)
                amp2 = m2["base_amplitude"] * (m2["scale_factor"] + idx2 * (1 - m2["scale_factor"]) / self.link_number)

                phi1 = 2 * np.pi * m1["frequency"] * t + 2 * np.pi * (i * wave_num / self.link_number) + m1["phase_offset"]
                phi2 = 2 * np.pi * m2["frequency"] * t + 2 * np.pi * (i * wave_num / self.link_number) + m2["phase_offset"]

                if wave_mode == "standing":
                    phi1 = 2 * np.pi * (i * wave_num / self.link_number) + m1["phase_offset"]
                    phi2 = 2 * np.pi * (i * wave_num / self.link_number) + m2["phase_offset"]
                    wave1 = amp1 * np.sin(phi1) - m1["shifted_amplitude"] * np.sin(2 * np.pi * m1["frequency"] * t)
                    wave2 = amp2 * np.sin(phi2) - m2["shifted_amplitude"] * np.sin(2 * np.pi * m2["frequency"] * t)
                    circ1 = circ_amp1 * np.sin(phi1 - np.pi / 2)
                    circ2 = circ_amp2 * np.sin(phi2 - np.pi / 2)
                else:
                    wave1 = amp1 * np.sin(phi1) - m1["shifted_amplitude"]
                    wave2 = amp2 * np.sin(phi2) - m2["shifted_amplitude"]
                    circ1 = circ_amp1 * np.sin(phi1 - np.pi / 2)
                    circ2 = circ_amp2 * np.sin(phi2 - np.pi / 2)

                self.publisher_fin_01_dof_1[i].publish(Float64(data=circ1))
                self.publisher_fin_01_dof_2[i].publish(Float64(data=wave1))
                self.publisher_fin_02_dof_1[i].publish(Float64(data=circ2))
                self.publisher_fin_02_dof_2[i].publish(Float64(data=-wave2))

        self.i += 1

    def timer_callback(self):
        self.execute()

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
