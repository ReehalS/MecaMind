import re
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import ttk

# Function to parse the log file
def parse_log(file_path):
    x_accel = []
    y_accel = []
    x_vel = []
    y_vel = []
    rotation_input = []
    rotation_output = []
    direction_input = []
    direction_output = []
    move_direction = []
    force_fl = []
    force_bl = []
    force_fr = []
    force_br = []

    # Regular expression pattern to match the log line format
    pattern = re.compile(
        r"X Acceleration: ([-+]?\d*\.\d+|\d+), Y Acceleration: ([-+]?\d*\.\d+|\d+). "
        r"X Velocity: ([-+]?\d*\.\d+|\d+), Y Velocity: ([-+]?\d*\.\d+|\d+). "
        r"Rotation Input: ([-+]?\d*\.\d+|\d+). Rotation Output: ([-+]?\d*\.\d+|\d+). "
        r"Direction Input: ([-+]?\d*\.\d+|\d+). Direction Output: ([-+]?\d*\.\d+|\d+). "
        r"Move Direction: ([-+]?\d*\.\d+|\d+). "
        r"Force FL: ([-+]?\d*\.\d+|\d+), Force BL: ([-+]?\d*\.\d+|\d+), Force FR: ([-+]?\d*\.\d+|\d+), Force BR: ([-+]?\d*\.\d+|\d+)"
    )

    with open(file_path, 'r') as file:
        next(file)
        for line in file:
            match = pattern.match(line)
            if match:
                x_accel.append(float(match.group(1)))
                y_accel.append(float(match.group(2)))
                x_vel.append(float(match.group(3)))
                y_vel.append(float(match.group(4)))
                rotation_input.append(float(match.group(5)))
                rotation_output.append(float(match.group(6)))
                direction_input.append(float(match.group(7)))
                direction_output.append(float(match.group(8)))
                move_direction.append(float(match.group(9)))
                force_fl.append(float(match.group(10)))
                force_bl.append(float(match.group(11)))
                force_fr.append(float(match.group(12)))
                force_br.append(float(match.group(13)))

    return (x_accel, y_accel, x_vel, y_vel, rotation_input, rotation_output, direction_input,
            direction_output, move_direction, force_fl, force_bl, force_fr, force_br)

# Function to plot the data
def plot_data(x_accel, y_accel, x_vel, y_vel, rotation_input, rotation_output, direction_input,
              direction_output, move_direction, force_fl, force_bl, force_fr, force_br):
    plt.figure(figsize=(15, 18))

    # Plot X and Y Acceleration
    plt.subplot(8, 1, 1)
    plt.plot(x_accel, label='X Acceleration', color='blue')
    plt.plot(y_accel, label='Y Acceleration', color='orange')
    plt.title('X and Y Acceleration')
    plt.xlabel('Time')
    plt.ylabel('Acceleration')
    plt.legend()

    # Plot X and Y Velocity
    plt.subplot(8, 1, 2)
    plt.plot(x_vel, label='X Velocity', color='blue')
    plt.plot(y_vel, label='Y Velocity', color='orange')
    plt.title('X and Y Velocity')
    plt.xlabel('Time')
    plt.ylabel('Velocity')
    plt.legend()

    # Plot Rotation Input and Output
    plt.subplot(8, 1, 3)
    plt.plot(rotation_input, label='Rotation Input', color='blue')
    plt.plot(rotation_output, label='Rotation Output', color='orange')
    plt.title('Rotation Input and Output')
    plt.xlabel('Time')
    plt.ylabel('Value')
    plt.legend()

    # Plot Direction Input and Output
    plt.subplot(8, 1, 4)
    plt.plot(direction_input, label='Direction Input', color='blue')
    plt.plot(direction_output, label='Direction Output', color='orange')
    plt.title('Direction Input and Output')
    plt.xlabel('Time')
    plt.ylabel('Value')
    plt.legend()

    # Plot Move Direction
    plt.subplot(8, 1, 5)
    plt.plot(move_direction, label='Move Direction', color='blue')
    plt.title('Move Direction')
    plt.xlabel('Time')
    plt.ylabel('Value')
    plt.legend()

    # Plot left motor forces (FL and BL)
    plt.subplot(8, 1, 6)
    plt.plot(force_fl, label='Force FL', color='red')
    plt.plot(force_bl, label='Force BL', color='green')
    plt.title('Left Motor Forces')
    plt.xlabel('Time')
    plt.ylabel('Force')
    plt.legend()

    # Plot right motor forces (FR and BR)
    plt.subplot(8, 1, 7)
    plt.plot(force_fr, label='Force FR', color='purple')
    plt.plot(force_br, label='Force BR', color='brown')
    plt.title('Right Motor Forces')
    plt.xlabel('Time')
    plt.ylabel('Force')
    plt.legend()

    # Combined plot for all forces
    plt.subplot(8, 1, 8)
    plt.plot(force_fl, label='Force FL', color='red')
    plt.plot(force_bl, label='Force BL', color='green')
    plt.plot(force_fr, label='Force FR', color='purple')
    plt.plot(force_br, label='Force BR', color='brown')
    plt.title('Combined Motor Forces')
    plt.xlabel('Time')
    plt.ylabel('Force')
    plt.legend()

    plt.tight_layout()
    return fig

# Function to create a fullscreen scrollable Tkinter window with Matplotlib
def create_scrollable_window(fig):
    root = tk.Tk()
    root.title("Force Plots")

    # Set the window to fullscreen
    root.attributes('-fullscreen', True)
    root.bind('<F11>', lambda event: root.attributes('-fullscreen', True))  # Bind F11 to toggle fullscreen
    root.bind('<Escape>', lambda event: root.attributes('-fullscreen', False))  # Bind Escape to exit fullscreen

    canvas = tk.Canvas(root)
    scrollbar = ttk.Scrollbar(root, orient="vertical", command=canvas.yview)
    scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
    canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    canvas.configure(yscrollcommand=scrollbar.set)

    plot_frame = ttk.Frame(canvas)
    canvas.create_window((0, 0), window=plot_frame, anchor="nw")

    # Embed the Matplotlib figure into the Tkinter window
    canvas_figure = FigureCanvasTkAgg(fig, master=plot_frame)
    canvas_figure.draw()
    canvas_figure.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    plot_frame.update_idletasks()
    canvas.config(scrollregion=canvas.bbox("all"))

    root.mainloop()

# Main script
if __name__ == "__main__":
    log_file_path = 'C:/Users/sunny/OneDrive/Documents/GitHub/MecaMind/viewFunctions/log.txt'  # Path to log file
    data = parse_log(log_file_path)
    plot_data(*data)
