# vectornav

A minimal driver for VectorNav IMUs

## Setup

```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone git@github.com:ntnu-arl/vectornav.git
cd ..
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build
```

## Usage

### Setting up custom udev rules

```bash
cd catkin_ws/src
sudo cp vectornav/udev/* /etc/udev/rules.d/
sudo udevadm control --reload
```

Note: The USB latency timer should be set to 1 for the IMU to avoid the [bunching up of IMU messages](https://github.com/ntnu-arl/vectornav/issues/5). The udev rules automatically does this for you. 

### Running the node

```bash
cd catkin_ws
source devel/setup.bash
roslaunch vectornav vectornav.launch
```

## Additional Documentation

![Coordinate frame](images/vn100_coordinate_system.png)

The default output from the VN100 is in the NED coordinate system.

![IMU subsystems](images/imu_subsystems.png)

For any questions, please contact

- [Nikhil Khedekar](mailto:nikhil.v.khedekar@ntnu.no)