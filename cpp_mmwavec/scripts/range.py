#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import sys
import threading
import pprint
from std_msgs.msg import Int16MultiArray

VERBOSE = True

class FrameBuffer():
    #DoubleBuffer for for vis
    def __init__(self, framesize=(256,256)):
        self.buff = [np.zeros(framesize),
                     np.zeros(framesize),
                    ]

        self.dirty = True
        self.frontbuff = 0

    def write_frame(self, new_frame):
        backbuff = (self.frontbuff+1)%2
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

    img = (data/max_val * 255).astype(np.uint8)
    if cmap:
        img = cv2.applyColorMap(img, cmap)
    return img


def imshow_thread(framebuffer, max_val=18.0, cmap=cv2.COLORMAP_WINTER):
    #k = cv2.waitKey(1)

    cv2.namedWindow('fft_viz', cv2.WINDOW_FREERATIO)

    ax = 2

    while True:
        update, img = framebuffer.get_frame()
        if update:
            img = normalize_and_color(img, max_val, cmap)
            cv2.imshow('fft_viz', img)
        cv2.waitKey(1)

    cv2.destroyAllWindows()


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

class MmwaveFFTviz(Node):
    def __init__(self, fb):
        super().__init__('mmwave_fftviz')
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
        self.frame_kwargs = None
        self.pp = pprint.PrettyPrinter(width=100)

    def set_radar_cfg(self):
        # params = self.get_parameters('')
        # params_dict = {}
        # for param in params:
        #     params_dict[param.name] = param.value
        # self.get_logger().info("Radar Params: %r" % params_dict)

        # samples_per_chirp = params_dict['adcSamples']
        # n_tdm = len(params_dict['chirps'])
        # n_rx = params_dict['rx1'] + params_dict['rx2'] + params_dict['rx3'] + params_dict['rx4']
        # n_chirps_per_frame = params_dict['numChirps']

        self.frame_kwargs = {
            'samples_per_chirp': 256,
            'n_receivers': 4,
            'n_tdm': 3,
            'n_chirps_per_frame': 60,
        }

        if VERBOSE:
            self.get_logger().info(str(self.frame_kwargs))

    def apply_window(self, data, window_type='hann', axis=-1):
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
        adc_samples = self.apply_window(adc_samples, axis=1)  # Windowing along range
        # adc_samples = self.apply_window(adc_samples, axis=0)  # Windowing along Doppler
        # adc_samples = self.apply_window(adc_samples, axis=2)  # Windowing along angle

        fft_range = np.fft.fft(adc_samples, axis=1)
        fft_range_doppler = np.fft.fft(fft_range, axis=0)

        
        fft_mag = np.fft.fftshift(np.log(np.abs(fft_range[:, :, 0])), axes=0)
        return fft_mag
    
    

    def callback(self, data):
        if not self.frame_kwargs:
            self.set_radar_cfg()

        adc_samples = reshape_frame(data.data,
                                    **self.frame_kwargs
                                   )

        
        fft_mag = self.fft_process(adc_samples)

        self.fb.write_frame(fft_mag)



def main(args=None):
    rclpy.init(args=args)

    fb = FrameBuffer()
    fft_viz = MmwaveFFTviz(fb)

    ui_thread = threading.Thread(target=imshow_thread, args=(fb,))
    ui_thread.setDaemon(True)
    ui_thread.start()

    rclpy.spin(fft_viz)

    fft_viz.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
