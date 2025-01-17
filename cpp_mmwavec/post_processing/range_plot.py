import rosbag2_py
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import numpy as np
import matplotlib.pyplot as plt
from radicalsdk.radar.config_v1 import read_radar_params
from radicalsdk.radar.v1 import RadarFrame
import mmwave.dsp


"""
This file is an example of how to process a recorded ROS 2 bag
"""

# Reshape function as before
def reshape_frame(data, samples_per_chirp, n_receivers, n_tdm, n_chirps_per_frame):
    data = np.array(data)
    
    _data = data.reshape(-1, 8)
    _data = _data[:, :4] + 1j * _data[:, 4:]
    _data = _data.reshape(n_chirps_per_frame, samples_per_chirp, n_receivers)

    # Deinterleave if there's TDM
    if n_tdm > 1:
        _data_i = [_data[i::n_tdm, :, :] for i in range(n_tdm)]
        _data = np.concatenate(_data_i, axis=-1)
    
    return _data

def fft_process(adc_samples):
        fft_range = np.fft.fft(adc_samples, axis=1)
        fft_range_doppler = np.fft.fft(fft_range, axis=0)
        fft_range_azi = np.fft.fft(fft_range_doppler, n=32, axis=2)
        
        fft_mag = np.fft.fftshift(np.log(np.abs(fft_range_doppler[:, :, 0])), axes=0)
        return fft_range


# Function to extract, reshape, process FFT, and plot the result
def extract_reshape_and_process_fft(bag_file, samples_per_chirp, n_receivers, n_tdm, n_chirps_per_frame):
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_file,
        storage_id='sqlite3'
    )
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}
    
    i=0
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        
        # Assuming msg contains the radar data you want to reshape
        radar_data = list(msg.data)  # Adjust this based on your actual message structure
        
        # Reshape the radar data (this is the radar cube)
        reshaped_data = reshape_frame(radar_data, samples_per_chirp, n_receivers, n_tdm, n_chirps_per_frame)
        
        # # Apply FFT processing
        fft_magnitude = fft_process(reshaped_data)
        i+=1    
        # Plot the FFT magnitude every 200 frames
        if i%200==0:
            plt.imshow(fft_magnitude, aspect='auto', cmap='viridis')
            plt.title(f'FFT Magnitude for {topic}')
            plt.xlabel('Range Bins')
            plt.ylabel('Doppler Bins')
            plt.colorbar(label='Magnitude (dB)')
            plt.show()

if __name__ == '__main__':
    # Update these parameters based on your radar configuration
    samples_per_chirp = 256
    n_receivers = 4
    n_tdm = 1
    n_chirps_per_frame = 128
    
    extract_reshape_and_process_fft('/home/radar/dev_ws/rosbag_august/testing2/testing2_0.db3', samples_per_chirp, n_receivers, n_tdm, n_chirps_per_frame)