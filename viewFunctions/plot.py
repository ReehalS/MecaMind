import re
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import ttk

# Function to parse the log file
def parse_log(file_path):
    input_data = []
    output_data = []
    actual_direction = []
    direction_error = []
    direction_output = []
    force_fl = []
    force_bl = []
    force_fr = []
    force_br = []

    pattern = re.compile(r"Input: ([-+]?\d*\.\d+|\d+). Rotation Output: ([-+]?\d*\.\d+|\d+). Actual Direction: ([-+]?\d*\.\d+|\d+). Direction Error: ([-+]?\d*\.\d+|\d+). Direction Output: ([-+]?\d*\.\d+|\d+). Force FL: ([-+]?\d*\.\d+|\d+), Force BL: ([-+]?\d*\.\d+|\d+), Force FR: ([-+]?\d*\.\d+|\d+), Force BR: ([-+]?\d*\.\d+|\d+)")

    with open(file_path, 'r') as file:
        next(file)
        for line in file:
            if "Done" in line:
                continue
            match = pattern.match(line)
            if match:
                input_data.append(float(match.group(1)))
                output_data.append(float(match.group(2)))
                actual_direction.append(float(match.group(3)))
                direction_error.append(float(match.group(4)))
                direction_output.append(float(match.group(5)))
                force_fl.append(float(match.group(6)))
                force_bl.append(float(match.group(7)))
                force_fr.append(float(match.group(8)))
                force_br.append(float(match.group(9)))

    return input_data, output_data, actual_direction, direction_error, direction_output, force_fl, force_bl, force_fr, force_br

# Function to plot the data
def plot_data(input_data, output_data, actual_direction, direction_error, direction_output, force_fl, force_bl, force_fr, force_br):
    fig, axs = plt.subplots(8, 1, figsize=(12, 24), sharex=True)

    axs[0].plot(input_data, label='Sensor Input', color='blue')
    axs[0].plot(output_data, label='PID Output', color='orange')
    axs[0].set_title('Rotation Input and Output Data')
    axs[0].set_ylabel('Value')
    axs[0].legend()

    axs[1].plot(actual_direction, label='Actual Direction', color='magenta')
    axs[1].set_title('Actual Direction')
    axs[1].set_ylabel('Degrees')
    axs[1].legend()

    axs[2].plot(direction_error, label='Direction Error', color='cyan')
    axs[2].set_title('Direction Error')
    axs[2].set_ylabel('Degrees')
    axs[2].legend()

    axs[3].plot(direction_output, label='Direction Output', color='yellow')
    axs[3].set_title('Direction Output')
    axs[3].set_ylabel('Value')
    axs[3].legend()

    axs[4].plot(force_fl, label='Force FL', color='red')
    axs[4].plot(force_bl, label='Force BL', color='green')
    axs[4].set_title('Left Motor Forces')
    axs[4].set_ylabel('Force')
    axs[4].legend()

    axs[5].plot(force_fr, label='Force FR', color='purple')
    axs[5].plot(force_br, label='Force BR', color='brown')
    axs[5].set_title('Right Motor Forces')
    axs[5].set_ylabel('Force')
    axs[5].legend()

    left_motor_sum = [fl + bl for fl, bl in zip(force_fl, force_bl)]
    right_motor_sum = [fr + br for fr, br in zip(force_fr, force_br)]
    force_diff = [l - r for l, r in zip(left_motor_sum, right_motor_sum)]
    axs[6].plot(force_diff, label='Force Difference', color='black')
    axs[6].set_title('Force Difference Sides Left-Right')
    axs[6].set_ylabel('Force')
    axs[6].legend()

    diagonal_sum1 = [fl + br for fl, br in zip(force_fl, force_br)]
    diagonal_sum2 = [fr + bl for fr, bl in zip(force_fr, force_bl)]
    force_diff_diagonal = [d1 - d2 for d1, d2 in zip(diagonal_sum1, diagonal_sum2)]
    axs[7].plot(force_diff_diagonal, label='Force Difference Diagonal', color='blue')
    axs[7].set_title('Force Difference Diagonal (FL+BR) - (FR+BL)   ')
    axs[7].set_ylabel('Force')
    axs[7].legend()

    for ax in axs:
        ax.set_xlabel('Time')

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
    input_data, output_data, actual_direction, direction_error, direction_output, force_fl, force_bl, force_fr, force_br = parse_log(log_file_path)
    fig = plot_data(input_data, output_data, actual_direction, direction_error, direction_output, force_fl, force_bl, force_fr, force_br)
    create_scrollable_window(fig)
