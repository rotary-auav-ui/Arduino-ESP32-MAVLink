# MAVLink CPP Library

This library is a C++ implementation of the MAVLink protocol. It is designed to be used on the ESP32 microcontroller. It is a work in progress and is not yet complete. The library is designed to be used to test changes before it is commited to the [ESP32-MAVLink](https://github.com/rotary-auav-ui/ESP32-MAVLink) main branch. The testing is done on the Gazebo PX4 SITL simulator. Connection to the PX4 SITL is done using UDP instead of UART via telemetry.

## Installation

Clone the PX4 Autopilot Library and run the PX4 SITL simulator on Gazebo
```
git clone -b v1.13.2 --recurse-submodules https://github.com/PX4/PX4-Autopilot
cd ./PX4-Autopilot
make px4_sitl gazebo
```

Clone the MAVLink CPP library & compile
```
git clone -b UDP-SITL https://github.com/rotary-auav-ui/mavlink-c-library.git
cd mavlink-c-library
g++ -pthread ./mavlink_commands.cpp ./mavlink_udp.cpp -o ./mavlink_udp
```

Run on a seperate terminal with gazebo px4 sitl open
```
./mavlink_udp
```

## Usage
Usage is similar to the [main branch](https://github.com/rotary-auav-ui/ESP32-MAVLink). However, std::thread is used for read_data() and send_heartbeat looping.

