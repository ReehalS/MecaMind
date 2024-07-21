import re
import matplotlib.pyplot as plt

# Function to parse the log file
def parse_log(file_path):
    input_data = []
    output_data = []
    force_fl = []
    force_bl = []
    force_fr = []
    force_br = []

    # Regular expression pattern to match the log line format
    pattern = re.compile(r"Input: ([-+]?\d*\.\d+|\d+). Output: ([-+]?\d*\.\d+|\d+). Force FL: ([-+]?\d*\.\d+|\d+), Force BL: ([-+]?\d*\.\d+|\d+), Force FR: ([-+]?\d*\.\d+|\d+), Force BR: ([-+]?\d*\.\d+|\d+)")

    with open(file_path, 'r') as file:
        # Skip the first line
        next(file)
        for line in file:
            if "Done" in line:
                continue
            match = pattern.match(line)
            if match:
                input_data.append(float(match.group(1)))
                output_data.append(float(match.group(2)))
                force_fl.append(float(match.group(3)))
                force_bl.append(float(match.group(4)))
                force_fr.append(float(match.group(5)))
                force_br.append(float(match.group(6)))

    return input_data, output_data, force_fl, force_bl, force_fr, force_br

# Function to plot the data
def plot_data(input_data, output_data, force_fl, force_bl, force_fr, force_br):
    plt.figure(figsize=(12, 10))

    # Plot input and output data
    plt.subplot(5, 1, 1)
    plt.plot(input_data, label='Input', color='blue')
    plt.plot(output_data, label='Output', color='orange')
    plt.title('Input and Output Data')
    plt.xlabel('Time')
    plt.ylabel('Value')
    plt.legend()

    # Plot left motor forces (FL and BL)
    plt.subplot(5, 1, 2)
    plt.plot(force_fl, label='Force FL', color='red')
    plt.plot(force_bl, label='Force BL', color='green')
    plt.title('Left Motor Forces')
    plt.xlabel('Time')
    plt.ylabel('Force')
    plt.legend()

    # Plot right motor forces (FR and BR)
    plt.subplot(5, 1, 3)
    plt.plot(force_fr, label='Force FR', color='purple')
    plt.plot(force_br, label='Force BR', color='brown')
    plt.title('Right Motor Forces')
    plt.xlabel('Time')
    plt.ylabel('Force')
    plt.legend()

    # Combined plot for all forces
    plt.subplot(5, 1, 4)
    plt.plot(force_fl, label='Force FL', color='red')
    plt.plot(force_bl, label='Force BL', color='green')
    plt.plot(force_fr, label='Force FR', color='purple')
    plt.plot(force_br, label='Force BR', color='brown')
    plt.title('Combined Motor Forces')
    plt.xlabel('Time')
    plt.ylabel('Force')
    plt.legend()

    plt.subplot(5, 1, 5)
    # add left forces and subtract sum of right forces to get total force
    total_force = [(force_fl[i] + force_bl[i] - force_fr[i] - force_br[i])/2 for i in range(len(force_fl))]
    plt.plot(total_force, label='Total Force', color='blue') 
    plt.xlabel('Time')
    plt.ylabel('Force')
    plt.title('Total Force')
    plt.legend()

    plt.tight_layout()
    plt.show()

# Main script
if __name__ == "__main__":
    log_file_path = 'C:/Users/sunny/OneDrive/Documents/GitHub/MecaMind/viewFunctions/log.txt'  # Path to log file
    input_data, output_data, force_fl, force_bl, force_fr, force_br = parse_log(log_file_path)
    plot_data(input_data, output_data, force_fl, force_bl, force_fr, force_br)
