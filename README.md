# M5Stack_ECGModule_ros_driver

This repository contains ros driver for [M5Stack ECG Module13.2 (AD8232)](https://shop.m5stack.com/products/ecg-module13-2-ad8232-with-cables-and-pads)

## Requirements

- M5Stack Fire
- [M5Stack ECG Module13.2 (AD8232)](https://shop.m5stack.com/products/ecg-module13-2-ad8232-with-cables-and-pads) 
- platformio

## How to build and burn

```bash
cd firmware
cd lib
rosrun rosserial_arduino make_libraries.py .
cd ..
pio run -t upload --upload-port <port to M5Stack Fire>
```

## How to use

Please see demo.launch for examples

```bash
roslaunch M5Stack_ECGModule_ros_driver demo.launch
```
