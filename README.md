## MAVLink CPP Library

### PX4 SITL Testing via UDP

Clone the PX4 Autopilot Library
```
git clone --recurse-submodules https://github.com/PX4/PX4-Autopilot
cd PX4-Autopilot
make px4_sitl gazebo
```

Clone the MAVLink CPP library & compile
```
git clone -b UDP-SITL https://github.com/rotary-auav-ui/mavlink-c-library.git
cd mavlink-c-library
g++ -fno-stack-protector -pthread -g ./mavlink_commands.cpp ./mavlink_udp.cpp -o ./mavlink_udp
```

Run on seperate terminal with gazebo px4 sitl open
```
./mavlink_udp
```