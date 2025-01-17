#!/usr/bin/env python3
import argparse
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import sys
import threading
import pprint
from std_msgs.msg import Int16MultiArray
from playsound import playsound

VERBOSE = True

class FrameBuffer():
    # DoubleBuffer for visualization
    def __init__(self, framesize=(256, 256)):
        self.buff = [np.zeros(framesize),
                     np.zeros(framesize),
                    ]

        self.dirty = True
        self.frontbuff = 0
        self.open = False

    def write_frame(self, new_frame):
        backbuff = (self.frontbuff + 1) % 2
        self.buff[backbuff] = new_frame
        self.frontbuff = backbuff
        self.dirty = True

    def get_frame(self):
        d = self.dirty
        self.dirty = False
        return d, self.buff[self.frontbuff]

def normalize_and_color(data, max_val=0, cmap=None):
    if max_val <= 0:
        max_val = np.max(data)

    img = (data / max_val * 255).astype(np.uint8)
    if cmap:
        img = cv2.applyColorMap(img, cmap)
    return img

def imshow_thread(framebuffer, max_val=18.0, cmap=cv2.COLORMAP_WINTER):
    cv2.namedWindow('fft_viz', cv2.WINDOW_FREERATIO)

    while True:
        update, img = framebuffer.get_frame()
        if update:
            img = normalize_and_color(img, max_val, cmap)
            cv2.imshow('fft_viz', img)
        cv2.waitKey(1)

        if cv2.getWindowProperty('fft_viz', cv2.WND_PROP_VISIBLE) > 1:
            framebuffer.open = True

        if cv2.getWindowProperty('fft_viz', cv2.WND_PROP_VISIBLE) < 1 and framebuffer.open==True:
            print("Window closed, stopping visualization thread.")
            framebuffer.open = False
            break

    cv2.destroyAllWindows()

def reshape_frame(data, samples_per_chirp, n_receivers, n_tdm, n_chirps_per_frame):
    data = np.array(data)
    
    _data = data.reshape(-1, 8)
    _data = _data[:, :4] + 1j * _data[:, 4:]
    _data = _data.reshape(n_chirps_per_frame, samples_per_chirp, n_receivers)

    if n_tdm > 1:
        _data_i = [_data[i::n_tdm, :, :] for i in range(n_tdm)]
        _data = np.concatenate(_data_i, axis=-1)
    
    return _data

class MmwaveFFTviz(Node):
    def __init__(self, fb, frame_kwargs=None):
        super().__init__('mmwave_fftviz')
        if frame_kwargs == None:
            print("No config parameters given")
            return
        self.frame_kwargs = frame_kwargs
        self.subscription = self.create_subscription(
            Int16MultiArray,
            'radar_data',
            self.callback,
            10)
        self.subscription  # prevent unused variable warning

        if VERBOSE:
            self.get_logger().info('Subscribed to mmwave radar_data')

        self.windowCreated = False
        self.fb = fb
        self.pp = pprint.PrettyPrinter(width=100)

    def set_radar_cfg(self):

        c = 299792458  # speed of wave propagation in m/s
        adc_sample_period = 1 / 9499 * self.frame_kwargs["samples_per_chirp"]  # msec
        bandwidth = adc_sample_period * 100  # GHz
        self.range_resolution = c / (2 * (bandwidth * 1e9))  # meters
        self.max_range = self.range_resolution * self.frame_kwargs["samples_per_chirp"]

        if VERBOSE:
            self.get_logger().info(str(self.frame_kwargs))

    def apply_window(self, data, window_type='blackman', axis=-1):
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
        adc_samples = self.apply_window(adc_samples, axis=1)
        adc_samples = self.apply_window(adc_samples, axis=0)

        fft_range = np.fft.fft(adc_samples, axis=1)
        fft_range_doppler = np.fft.fft(fft_range, axis=0)

        fft_mag = np.fft.fftshift(np.log(np.abs(fft_range_doppler[:, :, 0])), axes=0)

        if (fft_mag[0, 15:30] > 10).any():
            playsound('/home/radar/dev_ws/src/cpp_mmwavec/scripts/warning.wav')

        return fft_mag

    def callback(self, data):
        if not self.frame_kwargs:
            self.set_radar_cfg()

        adc_samples = reshape_frame(data.data, **self.frame_kwargs)
        fft_mag = self.fft_process(adc_samples)
        self.fb.write_frame(fft_mag)

def parse_arguments():
    parser = argparse.ArgumentParser(description='Initialize Mmwave FFT Visualization with command line parameters.')
    parser.add_argument('--samples_per_chirp', type=int, default=256, help='Number of samples per chirp')
    parser.add_argument('--n_receivers', type=int, default=4, help='Number of receivers')
    parser.add_argument('--n_tx', type=int, default=3, help='Number of TDM (Time Division Multiplexing) channels')
    parser.add_argument('--n_chirps_per_frame', type=int, default=120, help='Number of chirps per frame')

    args = parser.parse_args()

    # Convert parsed arguments into a dictionary
    frame_kwargs = {
        'samples_per_chirp': args.samples_per_chirp,
        'n_receivers': args.n_receivers,
        'n_tdm': args.n_tx,
        'n_chirps_per_frame': args.n_chirps_per_frame,
    }

    return frame_kwargs

def main(args=None):
    rclpy.init(args=args)

    frame_kwargs = parse_arguments()
    fb = FrameBuffer()
    fft_viz = MmwaveFFTviz(fb, frame_kwargs=frame_kwargs)

    ui_thread = threading.Thread(target=imshow_thread, args=(fb,))
    ui_thread.setDaemon(True)
    ui_thread.start()

    rclpy.spin(fft_viz)

    fft_viz.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
