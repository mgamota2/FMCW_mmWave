# FMCW mmWave
ROS 2 Package (C++) for TI IWR1443BOOST and TI DCA1000EVM FMCW mmWave Radar System

This project is based off of the work done for the RaDICaL SDK: 
https://moodoki.github.io/radical_sdk/
https://publish.illinois.edu/radicaldata/

This has only been tested on Ubuntu 22.04.5

## Getting started
This repository includes the ROS 2 package written in C++ with real time visualization scripts written in Python. There is also an example script for working with prerecorded data in the .db3 format, which is the format for ROS 2 bags.

I recommend reading the ROS 2 documentation to set up your ROS 2 environment and workspace, then adding the package to the workspace within the src folder.

## How to use
Once ROS 2 has been configured and you have added the cpp_mmwavec package files to the src of your ROS 2 workspace, follow the below instructions.

The ROS 2 package is defined as "cpp_mmwavec" in the package.xml file, but this can be changed.

1. Open terminal

2. To build the ROS 2 package use `colcon build --packages-select cpp_mmwavec`

3. Access to the serial port must be granted, in my setup this meant `sudo chmod 666 /dev/ttyACM0`

4. Finally `ros2 run cpp_mmwavec mmwave [config name]`

Regarding the config name, if the config is "short_range.cfg" , the config name is "short_range"

## Real-time Visualizations
To use one of the sample real-time visualizations, open a new terminal tab or window

## Recording data
To record data, open a new terminal tab or window use `ros2 bag record /radar_data -o [path_to_save]`. This will create a folder at the path specified and create a metadata.yml file and a .db3 file with all of the data from the radar.


## Additional resources
Fundamentals of mmWave: https://www.ti.com/lit/wp/spyy005a/spyy005a.pdf?ts=1727119340010&ref_url=https%253A%252F%252Fwww.google.com%252F

FMCW mmWave: https://www.ti.com/content/dam/videos/external-videos/zh-tw/2/3816841626001/5415203482001.mp4/subassets/mmwaveSensing-FMCW-offlineviewing_0.pdf


RaDICaL Paper: https://ieeexplore.ieee.org/document/9361086


