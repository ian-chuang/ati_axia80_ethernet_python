from ati_axia80_ethernet_python import ForceTorqueSensorDriver
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

# Initialize variables for plotting
num_points = 100  # Number of points to display on the plot
wrench_data = np.zeros((num_points, 6))

def update_plot(frame):
    global wrench_data

    # Get latest wrench data
    wrench = driver.get_wrench()
    
    # Shift data to the left and append new data
    wrench_data[:-1] = wrench_data[1:]
    wrench_data[-1] = wrench

    # Update plot data for each axis
    for i, line in enumerate(lines):
        line.set_ydata(wrench_data[:, i])

    return lines

if __name__ == "__main__":
    sensor_ip = "172.22.22.3"
    driver = ForceTorqueSensorDriver(sensor_ip)

    # Setup the plot
    fig, ax = plt.subplots(2, 1, figsize=(10, 6))
    
    time_steps = np.linspace(-num_points, 0, num_points)

    # Force plot (fx, fy, fz)
    ax[0].set_title("Force (N)")
    ax[0].set_xlim(-num_points, 0)
    ax[0].set_ylim(-50, 50)  # Adjust as needed
    ax[0].set_ylabel("Force (N)")
    ax[0].grid()

    # Torque plot (tx, ty, tz)
    ax[1].set_title("Torque (Nm)")
    ax[1].set_xlim(-num_points, 0)
    ax[1].set_ylim(-5, 5)  # Adjust as needed
    ax[1].set_xlabel("Time (arbitrary units)")
    ax[1].set_ylabel("Torque (Nm)")
    ax[1].grid()

    # Initialize lines for fx, fy, fz, tx, ty, tz
    colors = ["r", "g", "b", "c", "m", "y"]
    lines = [ax[0].plot(time_steps, wrench_data[:, i], color=colors[i])[0] for i in range(3)] + \
            [ax[1].plot(time_steps, wrench_data[:, i + 3], color=colors[i + 3])[0] for i in range(3)]

    try:
        driver.start()

        # Create the animation
        ani = FuncAnimation(fig, update_plot, interval=50)
        plt.show()

    except KeyboardInterrupt:
        print("Stopping the driver...")
    finally:
        driver.stop()
