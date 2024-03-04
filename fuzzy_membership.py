import matplotlib.pyplot as plt
import numpy as np

# Define membership functions (example using trapezoidal for dedt)
def error_NL(e):
    a1, a2, a3, b = -300, -200, -150, -100
    return np.maximum(0, np.minimum(np.maximum(0, (e - a1) / (a2 - a1)), (e - a3) / (b - a3)))

def error_Z(e):
    a1, a2, b = -200, 0, 200
    return np.maximum(0, np.minimum((e - a1) / (a2 - a1), 1))

def error_PL(e):
    a1, a2, a3, b = 450, 500, 550, 600
    return np.maximum(0, np.minimum(np.maximum(0, (e - a1) / (a2 - a1)), (e - a3) / (b - a3)))

def dedt_NL(de):
    a1, a2, a3, b = -0.15, -0.12, -0.08, -0.05
    return np.maximum(0, np.minimum(np.maximum(0, (de - a1) / (a2 - a1)), (de - a3) / (b - a3)))

def dedt_Z(de):
    a1, a2, b = -0.01, 0, 0.01
    return np.maximum(0, np.minimum((de - a1) / (a2 - a1), 1))

def dedt_PL(de):
    a1, a2, a3, b = 0.05, 0.08, 0.12, 0.15
    return np.maximum(0, np.minimum(np.maximum(0, (de - a1) / (a2 - a1)), (de - a3) / (b - a3)))

# Calculate maximum and minimum dE/dt based on velocities and accelerations
max_velocity = 0.1  # m/s
min_velocity = 0.002  # m/s
max_acceleration = 0.005  # m/s^2
min_acceleration = -0.1  # m/s^2
dt = 0.01  # Adjust dt as needed

max_dedt = (max_velocity - min_velocity) / dt
min_dedt = (min_velocity - max_velocity) / dt

# Create sample data points for plotting with higher resolution
error_values = np.linspace(-150, 500, 100)
dedt_values = np.linspace(min_dedt, max_dedt, 200)

# Plot membership functions on separate subplots for better clarity
plt.figure(figsize=(10, 6))

plt.subplot(211)
plt.plot(error_values , error_NL(error_values), label='Negative Large')
plt.plot(error_values , error_Z(error_values), label='Zero')
plt.plot(error_values , error_PL(error_values), label='Positive Large')
plt.title('Error Membership Functions')
plt.xlabel('Error (e)')
plt.ylim(0, 1)
plt.ylabel('Membership Degree')
plt.legend()
plt.show()
