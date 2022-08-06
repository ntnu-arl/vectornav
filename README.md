# vectornav

A minimal driver for VectorNav IMUs

[![Formatting (pre-commit)](https://github.com/ntnu-arl/vectornav/actions/workflows/format.yaml/badge.svg?branch=main)](https://github.com/ntnu-arl/vectornav/actions/workflows/format.yaml?query=branch%3Amain)

## Setup

### Setup dependancies

```bash
sudo apt-get install libspdlog-dev
```

### Setup build

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

1. Use `udevadm info --attribute-walk /dev/ttyUSB0` (change `/dev/ttyUSB0` to the port that the converter is connected to) to find the values for your FTDI converter. Typically, `idProduct`, `idVendor` and `serial` should be enough.
2. Edit the `99-vn100.rules` file and replace the values with the ones from your converter.
3. Edit the `SYMLINK` line in the `99-vn100.rules` file and replace the value with the name of the device that you want to use.
4. Copy the `99-vn100.rules` file to `/etc/udev/rules.d/`.

```bash
cd catkin_ws/src
sudo cp vectornav/udev/* /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
```

5. Add your user to the `dialout` group with `sudo usermod -a -G dialout $USER`.
6. Restart the computer.

Note:

1. The USB latency timer should be set to 1 for the IMU to avoid the [bunching up of IMU messages](https://github.com/ntnu-arl/vectornav/issues/5). The udev rules automatically does this for you.
2. You can use `udevadm test $(udevadm info --query=path --name=/dev/ttyUSB0)` to test whether the new rule works as expected. If the rule works, it should show you the attributes for the converter with the rule that was triggered.
3. You may need to unplug and replug the IMU to make the udev rule take effect.
4. If the udev rules do not load, you may need to restart the computer. This only needs to be done once during the setup.

### Setting up the parameters

Modify the `config/vn100_params.yaml` per your requirements.

### Running the node

```bash
cd catkin_ws
source devel/setup.bash
roslaunch vectornav vectornav.launch
```

### Compatibility

This driver has been tested on the following systems:

- USB:
  - Pop OS 20.04 x86-64
  - Ubuntu 20.04 arm64

## Additional Documentation

![Coordinate frame](images/vn100_coordinate_system.png)

The default output from the VN100 is in the NED coordinate system.

![IMU subsystems](images/imu_subsystems.png)

For any questions, please contact

- [Nikhil Khedekar](mailto:nikhil.v.khedekar@ntnu.no)
- [Morten Nissov](mailto:morten.nissov@ntnu.no)
