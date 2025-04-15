import numpy as np
import matplotlib.pyplot as plt

# Parameters for plotting bounded wave
LINK_NUMBER = 50  # Total number of joints
timer_period = 0.1  # Time interval in seconds
i = 10  # Arbitrary time index for demonstration

# Bounded wave parameters
L = LINK_NUMBER - 1  # Total length of the range
x = np.linspace(0, L, LINK_NUMBER)  # x values
A = 1  # Maximum amplitude of the quadratic bound
alpha = 4 * A / L**2  # Quadratic coefficient for symmetry
t = i * timer_period  # Time factor

# Quadratic bounds
upper_bound = -alpha * (x - L / 2)**2 + A
lower_bound = -upper_bound

# Compute bounded wave
sine_wave = np.sin(2 * np.pi * (x / 30 - t))  # Time-shifted sine wave
bounded_wave = sine_wave * upper_bound  # Apply bounds

# Plot the bounded wave
plt.figure(figsize=(12, 6))
plt.plot(x, upper_bound, label="Upper Bound", linestyle="--", color="red")
plt.plot(x, lower_bound, label="Lower Bound", linestyle="--", color="red")
plt.plot(x, bounded_wave, label="Bounded Wave", color="blue")
plt.fill_between(x, lower_bound, upper_bound, color="gray", alpha=0.3, label="Bounding Region")
plt.axhline(0, color="black", linewidth=0.5)
plt.title("Bounded Sine Wave with Quadratic Bounds (Starts and Ends at 0)")
plt.xlabel("x (Joint Index)")
plt.ylabel("Amplitude")
plt.legend()
plt.grid(True)
plt.show()
