#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Int16MultiArray
import pprint
from scipy.ndimage import uniform_filter
import argparse


VERBOSE = True

class MmwaveFFTviz(Node):
    def __init__(self, frame_kwargs=None):
        super().__init__('mmwave_fftviz')
        self.subscription = self.create_subscription(
            Int16MultiArray,
            'radar_data',
            self.callback,
            10)
        self.subscription  # prevent unused variable warning
        if frame_kwargs is not None:
            self.frame_kwargs = frame_kwargs
        

        if VERBOSE:
            self.get_logger().info('Subscribed to mmwave radar_data')

        fft_mag_shape = (32,256)
        self.fft_mag = np.zeros(fft_mag_shape, dtype=np.float32)
        self.pp = pprint.PrettyPrinter(width=100)

        
        self.plot = 'xy'
        plt.ion()

        if self.plot == 'xy':
            self.fig, self.ax = plt.subplots()  # Set up the Cartesian plot

        elif self.plot=='polar':
            # Set up the plot
            self.fig, self.ax = plt.subplots()
            self.cax = self.ax.imshow(self.fft_mag, aspect='auto', origin='lower', cmap='viridis')
            self.fig.colorbar(self.cax)  # Add colorbar once       

    
    def apply_window(self, data, window_type='blackman', axis=-1):
        """
        Apply a window function to the data along the specified axis.
        """
        if window_type == 'hamming':
            window = np.hamming(data.shape[axis])
        elif window_type == 'hann':
            window = np.hanning(data.shape[axis])
        elif window_type == 'blackman':
            window = np.blackman(data.shape[axis])
        else:
            raise ValueError(f"Unsupported window type: {window_type}")
        
        window = window.reshape(-1 if axis == 0 else 1, -1 if axis == 1 else 1, -1 if axis == 2 else 1)
        return data * window

    def fft_process(self, adc_samples):
        # Apply windowing to the data cube
        # adc_samples = self.apply_window(adc_samples, axis=1)  # Windowing along range
        # adc_samples = self.apply_window(adc_samples, axis=0)  # Windowing along Doppler
        # adc_samples = self.apply_window(adc_samples, axis=2)  # Windowing along angle

        fft_range = np.fft.fft(adc_samples, axis=1)
        fft_range_doppler = np.fft.fft(fft_range, axis=0)

        
        # perform FFT over azimuth 
        az_fft = np.fft.fftshift(np.fft.fft(fft_range_doppler,n=32, axis=2), axes=2)

        # Compute Power Spectrum
        az_power = np.abs(az_fft)**2
        # Setting to true onyl detects moving objects
        only_moving = True
        if only_moving:
            final = (np.sum(az_power[2:,:,:], axis=0))
        else:
            final = (np.sum(az_power[:,:,:], axis=0))

        cfarred = self.ca_cfar_detection_2d_optimized(final, 35, 5, 35) # Changing these values changes the threshold for detected objects
        
        return final

    def ca_cfar_detection_2d_optimized(self, data, num_train, num_guard, threshold_factor):
        num_rows, num_cols = data.shape

        # Initialize the detection mask
        detection_mask = np.zeros((num_rows, num_cols), dtype=bool)

        # Create a window size that includes both training and guard cells
        window_size = num_train + num_guard

        # Compute the noise estimate using uniform filtering
        noise_estimate = uniform_filter(data, size=window_size, mode='reflect')  # Use 'reflect' to handle edges better

        # Calculate the number of cells in the window
        total_cells = window_size * window_size
        guard_cells = num_guard * num_guard
        cells_in_window = total_cells - guard_cells

        # Adjust noise estimate to account for guard cells
        noise_estimate = noise_estimate * (cells_in_window / total_cells)

        # Compute the threshold by multiplying the noise estimate by the threshold factor
        threshold = noise_estimate * threshold_factor

        # Perform detection
        detection_mask = data > threshold

        return detection_mask
    
    def callback(self, data):
        if not self.frame_kwargs:
            self.set_radar_cfg()

        adc_samples = self.reshape_frame(data.data)

        # Update the FFT magnitude and frequency for plotting
        self.fft_mag = self.fft_process(adc_samples)
        self.update_plot()

    def update_plot(self):
        # Clear the axes for the next frame
        if self.plot=='xy':
            # Get range and angles for the data
            num_range_bins, num_angles = self.fft_mag.shape

            # Assume angles range from -90 to 90 degrees
            angles = np.linspace(-90, 90, num_angles)
            # Assume range bins are equally spaced
            range_bins = np.arange(num_range_bins)

            # Convert to radians for trigonometric functions
            angles_rad = np.deg2rad(angles)

            # Create a meshgrid of range and angles
            range_grid, angle_grid = np.meshgrid(range_bins, angles_rad, indexing='ij')

            # Convert polar coordinates to Cartesian (x, y)
            x = range_grid * np.cos(angle_grid)
            y = range_grid * np.sin(angle_grid)

            # Clear the axes for the next frame
            self.ax.clear()

            # Use pcolormesh for a continuous color plot
            # You need to transpose fft_mag to match the x and y shape
            pcm = self.ax.pcolormesh(y, x, self.fft_mag, cmap='viridis', shading='auto')

        
            plt.draw()


        # Update the plot with the new FFT data
        if self.plot=='polar':    
                    
            # Update the image data
            self.cax.set_data(self.fft_mag)
            # self.cax.set_clim(vmin=0, vmax=1.5)  # Set color bar minimum to 0
            self.cax.autoscale()

            # Set y-axis label
            self.ax.set_ylabel('Range bin')
            self.ax.set_xlabel('Angle bin')


            plt.draw()

        plt.pause(0.01)

    def reshape_frame(self, data):
        samples_per_chirp=self.frame_kwargs['samples_per_chirp']
        n_receivers=self.frame_kwargs['n_receivers']
        n_tdm = self.frame_kwargs['n_tx']
        n_chirps_per_frame = self.frame_kwargs["n_chirps_per_frame"]
        data = np.array(data)
        
        _data = data.reshape(-1, 8)
        _data = _data[:, :4] + 1j * _data[:, 4:]
        _data = _data.reshape(n_chirps_per_frame, samples_per_chirp, n_receivers)

        #deinterleve if theres TDM
        if n_tdm > 1:
            _data_i = [_data[i::n_tdm, :, :] for i in range(n_tdm)]
            _data = np.concatenate(_data_i, axis=-1)
        
        return _data

def parse_arguments():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description='Initialize Mmwave FFT Visualization with command line parameters.')
    parser.add_argument('--samples_per_chirp', type=int, default=64, help='Number of samples per chirp')
    parser.add_argument('--n_receivers', type=int, default=4, help='Number of receivers')
    parser.add_argument('--n_tx', type=int, default=2, help='Number of tx')
    parser.add_argument('--n_chirps_per_frame', type=int, default=128, help='Number of chirps per frame')

    args = parser.parse_args()

    # Convert parsed arguments into a dictionary
    frame_kwargs = {
        'samples_per_chirp': args.samples_per_chirp,
        'n_receivers': args.n_receivers,
        'n_tx': args.n_tx,
        'n_chirps_per_frame': args.n_chirps_per_frame,
    }


    return frame_kwargs

def main(args=None):
    rclpy.init(args=args)

    frame_kwargs = parse_arguments()

    fft_viz = MmwaveFFTviz(frame_kwargs=frame_kwargs)
    
    try:
        while rclpy.ok():
            rclpy.spin_once(fft_viz, timeout_sec=0.01)
            fft_viz.update_plot()

    except KeyboardInterrupt:
        pass
    finally:
        fft_viz.destroy_node()
        rclpy.shutdown()

    plt.show()

if __name__ == '__main__':
    main()