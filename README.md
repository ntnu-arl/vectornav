# vectornav_driver

[![Formatting (pre-commit)](https://github.com/ntnu-arl/vectornav/actions/workflows/format.yaml/badge.svg?event=push)](https://github.com/ntnu-arl/vectornav/actions/workflows/format.yaml)
[![ROS Noetic](https://github.com/ntnu-arl/vectornav/actions/workflows/build_noetic.yaml/badge.svg?=event=push)](https://github.com/ntnu-arl/vectornav/actions/workflows/build_noetic.yaml)

A minimal driver for VectorNav IMUs

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

Usage of the VN100 via a USB connection requires an [FTDI TTL converter](https://no.rs-online.com/web/p/communication-wireless-development-tools/0429284). The converter should be connected to the VN100 as shown below

Pin Number on VN100 | Label on VN100 | Label on FTDI TTL Converter
--- | --- | ---
1 | VCC | VCC
5 | GND | GND
8 | TX2_TTL | RX
9 | RX2_TTL | TX

The complete pin assignments for the VN100 can be found in the [user manual](https://www.vectornav.com/resources/user-manuals)

Note that the default output from the VN100 is in the NED coordinate system.

Note that the `filter` topics are bias compensated (with the onboard filter) whereas the `imu` topics are not.

Topic | Message Type | Description
--- | --- | ---
`imu/data` | `sensor_msgs/Imu` | Raw IMU data
`imu/mag` | `sensor_msgs/MagneticField` | Raw magnetometer data
`filter/data` | `sensor_msgs/Imu` | IMU data from the onboard filter
`filter/mag` | `sensor_msgs/MagneticField` | Magnetometer data from the onboard filter
`temperature` | `sensor_msgs/Temperature` | Temperature of the IMU
`pressure` | `sensor_msgs/FluidPressure` | Pressure measured by the IMU
`sync_out_stamp` | `std_msgs/Header` | Approximate timestamp of the sync_out signal

Service Name | Service Type | Description
--- | --- | ---
`reset` | `std_srvs/Empty` | Resets the IMU

### Setting up custom udev rules

1. Use `udevadm info --attribute-walk /dev/ttyUSB0` (change `/dev/ttyUSB0` to the port that the converter is connected to) to find the values for your FTDI converter. Typically, `idProduct`, `idVendor` and `serial` should be enough.
2. Edit the `vectornav_driver/udev/99-vn100.rules` file and replace the values with the ones from your converter.
3. Edit the `SYMLINK` line in the `vectornav_driver/udev/99-vn100.rules` file and replace the value with the name of the device that you want to use.
4. Copy the `vectornav_driver/udev/99-vn100.rules` file to `/etc/udev/rules.d/`.

    ```bash
    cd catkin_ws/src
    sudo cp vectornav/vectornav_driver/udev/* /etc/udev/rules.d/
    sudo udevadm control --reload-rules && udevadm trigger
    ```

5. Add your user to the `dialout` group with `sudo usermod -a -G dialout $USER`.
6. Restart the computer.

Note:

1. The USB latency timer should be set to 1 for the IMU to avoid the bunching up of IMU messages. The udev rule automatically does this for you.
2. You can use `udevadm test $(udevadm info --query=path --name=/dev/ttyUSB0)` to test whether the new rule works as expected. If the rule works, it should show you the attributes for the converter with the rule that was triggered.
3. You may need to unplug and replug the IMU to make the udev rule take effect.
4. If the udev rules do not load, you may need to restart the computer. This only needs to be done once during the setup.

### Setting up the parameters

Modify the `vectornav_driver/config/vn100_params.yaml` per your requirements.

### Running the node

```bash
cd catkin_ws
source devel/setup.bash
roslaunch vectornav_driver vectornav_driver.launch
```

### Compatibility

This driver has been tested on the following systems:

- VN100 Rugged - USB:
  - Pop OS 20.04 x86-64
  - Ubuntu 20.04 arm64
  - Ubuntu 20.04 x86-64

#### Known Issues

The Seeed Studio A603 carrier board for the NVIDIA Orin NX has an issue with the kernel drivers for `usbserial` on some versions (CHECK IF YOUR VERSION WORKS WITHOUT THIS FIX FIRST). Essentially, instead of using the `usbserial.ko` for the board which are modified by Seeed, the default `usbserial.ko` is used. This causes the FTDI driver to not work as expected. The solution is to move the `usbserial.ko` file which causes the new file from Seeed Studio to be used.

```bash
cd /lib/modules/5.10.120-tegra/kernel/drivers/usb/serial/
sudo mv usbserial.ko usbserial.ko.bk
```

## Additional Documentation

- [VN100 Datasheet](https://www.vectornav.com/docs/default-source/datasheets/vn-100-datasheet-rev2.pdf?sfvrsn=8e35fd12_10)
- [VN100 User Manual](https://www.vectornav.com/resources/user-manuals/vn-100-user-manual)
- [Inertial Navigation Primer](https://www.vectornav.com/resources/inertial-navigation-primer)
- Time Synchronization Application Note: Contact VectorNav for access

For any questions, please contact

- [Nikhil Khedekar](mailto:nikhil.v.khedekar@ntnu.no)
- [Morten Nissov](mailto:morten.nissov@ntnu.no)
- [Kostas Alexis](mailto:konstantinos.alexis@ntnu.no)
