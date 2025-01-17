import tkinter as tk
from tkinter import ttk
import json  # Assuming the parameters are stored in JSON files
import os
import radicalsdk.radar.config_v1 as cfg_tools
import webbrowser
import math
import subprocess
import threading

# This defines a GUI which can be used to quickly visualize data, great for a demo setup

# Equation sources for chirp configurations: 
# https://www.ti.com/lit/an/swra553a/swra553a.pdf?ts=1732092771975&ref_url=https%253A%252F%252Fwww.google.com%252F

# Speed of light
c = 299792458 # m/s
ros_node_running = False


# Maximum range equation
def max_range(IF_max, freq_slope):
    return (IF_max*c)/(2*freq_slope)

# Range resolution
def range_res(chirp_bw):
    return c/(2*chirp_bw)

# Unambiguous max velocity
def max_velocity(total_time, wavelength):
    return wavelength/(4*total_time)

# Velocity resolution
def velocity_res(max_velocity, num_chirps):
    return (2*max_velocity)/num_chirps

# Note that angular resolution has a range depending on the angle of arrival
def angle_res(wavelength, max_distance, num_rx, num_tx):
    numerator = wavelength*180
    denominator = max_distance*num_rx*num_tx*math.pi
    return numerator/denominator


# Function to load radar configuration, calculate characteristics
def load_configuration(config):
    config_file = os.path.join(r'./src/configs/', f'{config}.cfg')
    config_dict = cfg_tools.read_radar_params(config_file)

    profiles = config_dict['profiles'][0]

    # Calculate values used in equations
    IF_max = 0.8*profiles['adcSampleRate']
    chirp_slope = profiles['freqSlopeConst']
    chirp_bw = (chirp_slope*(profiles['adcSamples']/profiles['adcSampleRate']))
    total_chirp_time = profiles['idle']+profiles['rampEndTime']
    wavelength = c/(4*profiles['start_frequency'])
    num_chirps_per_frame = config_dict['numChirps']/len(config_dict['chirps']) # Need to divide by number of antennas here
    num_rx = config_dict['numLanes']
    num_tx = config_dict['numTx']

    # Calculate relevant detection characteristic
    max_range_val = max_range(IF_max,profiles['freqSlopeConst'])
    range_res_val = range_res(chirp_bw)
    max_velocity_val = max_velocity(total_chirp_time, wavelength)
    velocity_res_val = velocity_res(max_velocity_val, num_chirps_per_frame)
    angle_res_val = angle_res(wavelength, max_range_val, num_rx, num_tx)

    # Display characteristics
    max_range_label.config(text=f"Max Range: {max_range_val:.4f} meters")
    range_resolution_label.config(text=f"Range Resolution: {range_res_val:.4f} meters")
    max_velocity_label.config(text=f"Max Velocity: {max_velocity_val: .5f} m/s")
    velocity_resolution_label.config(text=f"Velocity Resolution: {velocity_res_val: .4f} m/s")
    angle_resolution_label.config(text=f"Angle Resolution(at max distance, straight-on): {angle_res_val: .4f} degrees")
        
def open_web_tool():
    url = "https://dev.ti.com/gallery/view/mmwave/mmWaveSensingEstimator/ver/2.4.0/" 
    webbrowser.open(url)

def colcon_build():
    # do colcon build 
    command = "colcon build --packages-select cpp_mmwavec"
    result = subprocess.run(command, shell=True, capture_output=True, text=True)
    print(f"Command Output: {result.stdout}")
    print(f"Command Error: {result.stderr}")
    print("Building package")

def access_serial_port():
    # do sudo chmod (access to serial port)
    command2 = 'echo "radar" | sudo -S chmod 666 /dev/ttyACM0'
    result = subprocess.run(command2, shell=True, capture_output=True, text=True)
    print(f"Command Output: {result.stdout}")
    print(f"Command Error: {result.stderr}")
    if ('cannot access' in result.stderr):
        print("Try restarting the radar system")
    print("Accessing serial port")

# run visualization script with config
def viz_script(config_info):
    # visualization_script = os.path.join(r'./src/configs/', f'./{visualization_options_dict[viz_dropdown.get()]}.cfg')
    args = [
        '/bin/python3', f'{visualization_options_dict[viz_dropdown.get()]}',  # Replace with the actual path to the script
        '--samples_per_chirp', f'{config_info[2]}',  # Example argument
        '--n_receivers', f'{config_info[0]}',  # Example argument
        '--n_tx', f'{config_info[1]}',  # Example argument
        '--n_chirps_per_frame', f'{config_info[3]}',  # Example argument
    ]
    result = subprocess.run(args, shell=False, capture_output=False, text=True)
    # print(f"Command Output: {result.stdout}")
    # print(f"Command Error: {result.stderr}")
    print("Starting visualization")

# ros2 run with config
def ros_node_run(config_name):
    # do sudo chmod (access to serial port)
    command = f'ros2 run cpp_mmwavec mmwave {config_name}'
    result = subprocess.run(command, shell=True, capture_output=True, text=True)
    # print(f"Command Output: {result.stdout}")
    # print(f"Command Error: {result.stderr}")
    print("Running ROS node")


def run_radar_and_viz():
    global ros_node_running
    # Get info about config needed for chirp configuration and visualization
    config_name = config_dropdown.get()
    config_file = os.path.join(r'./src/configs/', f'{config_name}.cfg')
    config_dict = cfg_tools.read_radar_params(config_file)
    config_profiles = config_dict['profiles'][0]
    num_chirps_per_frame = config_dict['numChirps']/len(config_dict['chirps']) # Need to divide by number of antennas here
    
    colcon_build()
    access_serial_port()

    # num_rx, num_tx, adc_samples, chirps_per_frame. This will be passed as an argument into the ROS 2 subscriber/visualization node
    radar_info = [config_dict['numLanes'], config_dict['numTx'], config_profiles['adcSamples'], int(num_chirps_per_frame)]
    
    ros_thread = threading.Thread(target=ros_node_run, args=(config_name,))
    viz_thread = threading.Thread(target=viz_script, args=(radar_info,))

    if ros_node_running==False:
        ros_thread.start()
        ros_node_running=True
    viz_thread.start()
  
    


# Create the main window
root = tk.Tk()
root.title("FMCW Radar Chirp Configuration")

# Create and place the dropdown for selecting the use case
tk.Label(root, text="Select Radar Use Case").grid(row=0, column=0, padx=10, pady=5)

# Dropdown options (use case categories)
config_files = os.listdir(r'./src/configs')
config_options = [i.split('.')[0] for i in config_files if os.path.isfile(os.path.join('./src/configs', i))]

config_dropdown = ttk.Combobox(root, values=config_options)
config_dropdown.grid(row=0, column=1, padx=10, pady=5)
config_dropdown.set(config_options[0])

# Button to load the selected configuration and update the parameters
load_button = tk.Button(root, text="View Configuration", command=lambda: load_configuration(config_dropdown.get()))
load_button.grid(row=1, column=0, columnspan=2, pady=10)

# Labels to display the radar parameters
max_range_label = tk.Label(root, text="Max Range: N/A")
max_range_label.grid(row=2, column=0, padx=10, pady=5, columnspan=2)

range_resolution_label = tk.Label(root, text="Range Resolution: N/A")
range_resolution_label.grid(row=3, column=0, padx=10, pady=5, columnspan=2)

max_velocity_label = tk.Label(root, text="Max Velocity: N/A")
max_velocity_label.grid(row=4, column=0, padx=10, pady=5, columnspan=2)

velocity_resolution_label = tk.Label(root, text="Velocity Resolution: N/A")
velocity_resolution_label.grid(row=5, column=0, padx=10, pady=5, columnspan=2)

angle_resolution_label = tk.Label(root, text="Angle Resolution(at max distance, straight-on): N/A")
angle_resolution_label.grid(row=6, column=0, padx=10, pady=5, columnspan=2)

# Button to open the external web tool for custom configurations
web_tool_button = tk.Button(root, text="Open Web Tool for Custom Configurations", command=open_web_tool)
web_tool_button.grid(row=7, column=0, columnspan=2, pady=10)


visualization_options = ['Range-azimuth Beamforming', 'Range-azimuth FFT', 'Range-Doppler', 'Range']
visualization_options_dict = {'Range-azimuth Beamforming':'/home/radar/dev_ws/src/cpp_mmwavec/scripts/2d_capon.py', 
                              'Range-azimuth FFT':'/home/radar/dev_ws/src/cpp_mmwavec/scripts/2d_fft.py',
                               'Range-Doppler':'/home/radar/dev_ws/src/cpp_mmwavec/scripts/doppler.py', 
                               'Range':'/home/radar/dev_ws/src/cpp_mmwavec/scripts/range.py'}

tk.Label(root, text="Select Visualization").grid(row=8, column=0, padx=10, pady=5)
viz_dropdown = ttk.Combobox(root, values=visualization_options)
viz_dropdown.grid(row=8, column=1, padx=10, pady=5)
viz_dropdown.set(visualization_options[0])


# "Go" button to run the radar configuration and visualizations
go_button = tk.Button(root, text="Go", command=run_radar_and_viz)
go_button.grid(row=12, column=0, columnspan=2, pady=20)

# Run the Tkinter event loop
root.mainloop()