#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Int16MultiArray
import pprint
from scipy.ndimage import uniform_filter


VERBOSE = True

class MmwaveFFTviz(Node):
    def __init__(self):
        super().__init__('mmwave_fftviz')
        self.subscription = self.create_subscription(
            Int16MultiArray,
            'radar_data',
            self.callback,
            10)
        self.subscription  # prevent unused variable warning

        if VERBOSE:
            self.get_logger().info('Subscribed to mmwave radar_data')

        self.frame_kwargs = None
        fft_mag_shape = (32,256)
        self.fft_mag = np.zeros(fft_mag_shape, dtype=np.float32)
        self.pp = pprint.PrettyPrinter(width=100)

        
        self.plot = 'xy'


        if self.plot == 'polar':
            # Set up the plot
            self.fig, self.ax = plt.subplots()
            self.cax = self.ax.imshow(self.fft_mag, aspect='auto', origin='lower', cmap='viridis')
            self.fig.colorbar(self.cax)  # Add colorbar once
        elif self.plot == 'xy':
            self.fig, self.ax = plt.subplots()  # Set up the Cartesian plot
        

    def set_radar_cfg(self):
        # Configuration for radar parameters
        self.frame_kwargs = {
            'samples_per_chirp': 256,
            'n_receivers': 4,
            'n_tdm': 3,
            'n_chirps_per_frame': 60,
        }
                # speed of wave propagation
        # c = 299792458 # m/s

        # # compute ADC sample period T_c in msec
        # # adc_sample_period = 1 / adc_sample_rate * num_adc_samples # msec
        # adc_sample_period = 1 / 9499 * 304 # msec


        # # next compute the Bandwidth in GHz
        # bandwidth = adc_sample_period * 100 # GHz

        # # Coompute range resolution in meters
        # self.range_resolution = c / (2 * (bandwidth * 1e9)) # meters

        # self.max_range = self.range_resolution * 304
        # print(f'MAX RANGE: {self.max_range}')

        if VERBOSE:
            self.get_logger().info(str(self.frame_kwargs))
    
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
        range_cube = np.fft.fft(adc_samples, axis=1)
        range_doppler = np.fft.fft(range_cube, axis=0)
        
        steering_vector = compute_steering_vector(num_ant=12, angle_res=3, angle_rng=90)
        n_range_bins = range_doppler.shape[1]
        n_angles = steering_vector.shape[0]

        range_azimuth = np.zeros((n_range_bins, n_angles), dtype=np.float32)

        # Process each range bin
        for i in range(range_doppler.shape[1]):
            range_bin_data = range_doppler[:, i, :] # start at second doppler bin to remove stationary objects
            capon_spectrum = aoa_capon(range_bin_data, steering_vector)
            range_azimuth[i, :] = capon_spectrum


        cfarred = self.ca_cfar_detection_2d_optimized(range_azimuth, 30, 5, 25)

        return range_azimuth
        

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

        adc_samples = reshape_frame(data.data, **self.frame_kwargs)

        # Update the FFT magnitude and frequency for plotting
        self.fft_mag = self.fft_process(adc_samples)
        self.update_plot()

    def update_plot(self):
        # Clear the axes for the next frame
        if self.plot=='polar':
                   
            self.cax.set_data(self.fft_mag)
            self.cax.set_clim(vmin=0, vmax=1)  # Set color bar minimum to 0
            # self.cax.autoscale()

            # Set y-axis label
            self.ax.set_ylabel('Range bin')

            plt.draw()


            # Update the plot with the new FFT data
        elif self.plot == 'xy':
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

        plt.pause(0.01)

def reshape_frame(data, samples_per_chirp, n_receivers, n_tdm, n_chirps_per_frame):
    data = np.array(data)
    
    _data = data.reshape(-1, 8)
    _data = _data[:, :4] + 1j * _data[:, 4:]
    _data = _data.reshape(n_chirps_per_frame, samples_per_chirp, n_receivers)

    #deinterleve if theres TDM
    if n_tdm > 1:
        _data_i = [_data[i::n_tdm, :, :] for i in range(n_tdm)]
        _data = np.concatenate(_data_i, axis=-1)
    
    return _data

def forward_backward_avg(Rxx):
    """ Performs forward backward averaging on the given input square matrix
    Args:
        Rxx (ndarray): A 2D-Array square matrix containing the covariance matrix for the given input data
    Returns:
        R_fb (ndarray): The 2D-Array square matrix containing the forward backward averaged covariance matrix
    """
    assert np.size(Rxx, 0) == np.size(Rxx, 1)

    # --> Calculation
    M = np.size(Rxx, 0)  # Find number of antenna elements
    Rxx = np.matrix(Rxx)  # Cast np.ndarray as a np.matrix

    # Create exchange matrix
    J = np.eye(M)  # Generates an identity matrix with row/col size M
    J = np.fliplr(J)  # Flips the identity matrix left right
    J = np.matrix(J)  # Cast np.ndarray as a np.matrix

    R_fb = 0.5 * (Rxx + J * np.conjugate(Rxx) * J)

    return np.array(R_fb)


def aoa_capon(x, a):
    """ Computes Capon Spectrum
        Inputs:
            x - output of 1D range FFT (num_velocities, num_antennas)
            a - steering vector (num_angles, num_antennas)
        Outputs:
            capon_spectrum - Computed Capon Spectrum (num_angles)
    """
    num_antennas = x.shape[1]  # Should be 12

    # Perturbation matrix
    p = np.eye(num_antennas) * 1e-9

    # Compute covariance matrix
    Rxx = np.cov(x, rowvar=False)  # Covariance matrix, should be (12, 12)
    Rxx = forward_backward_avg(Rxx)

    # Verify the shape of Rxx

    Rxx_inv = np.linalg.inv(Rxx + p)

    capon_spec = np.reciprocal(np.einsum('ij,ij->i', a.conj(), (Rxx_inv @ a.T).T))

    return capon_spec



def compute_steering_vector(num_ant, angle_res, angle_rng):
    """ Computes array of Steering Vectors for a desired angluar range
        and resolution. **This is a special function that only computes the
        steering vectors along a 1D linear axis.**
        Inputs:
            angle_res - angle resolution in degrees
            angle_rng - single sided angle range
            num_ant - number of virtual antennas
        Output:
            steering_vectors
    """
    # get number of steering vectors based on desired angle range and resolution
    num_vec = (2 * angle_rng / angle_res + 1)
    num_vec = int(round(num_vec))

    # convert to radians
    angle_rng = angle_rng*np.pi/180
    angle_res = angle_res*np.pi/180

    # compute steering vectors
    steering_vectors = np.zeros((num_vec, num_ant), dtype=np.complex64)
    for k in range(num_vec):
        for m in range(num_ant):
            steering_vectors[k, m] = np.exp(-1j*np.pi*m
                                            *np.sin(-angle_rng + k*angle_res))
            
    return steering_vectors


def main(args=None):
    rclpy.init(args=args)

    fft_viz = MmwaveFFTviz()

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
