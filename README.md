# ROS2 WITH CFI(Control Flow Integrity)

## PROJECT OVERVIEW
  - Simulate control-flow hijacking vulnerability for message callback system of ROS2
  - Implement low-volume, efficient CFI(Control-Flow Integrity) mechanism and program

### Attack Scenario: ROS2 Callback Hijacking
  - Overwrite subscription callback function pointer of ROS2 with buffer overflow
  - WHY?
    - Target core mechanism of ROS2, Publisher & Subscriber Model
    - Callback function pointers use indirect calls, making them CFI protect friendly
    - Simulating is clearn and easy to understand
    - Attacks that can be fatal in real robot systems
   
### How CFI Implemented: Runtime Hook-Based CFI
  - It is implemented by runtime hooking using LD_PRELOAD
    - Hooking the callback registration/call function of ROS2
    - Whitelist management of legitimate callback function
    - Verify the address at the time of callback call
    - Ending the program in case of abnormal control flow detection

## RUNNING PROJECT

### SYSTEM SETTING

> Ubuntu 22.04 Jammy Jellyfish

``` bash
sudo apt update && sudo apt upgrade -y
```

``` bash
sudo apt install software-properties-common curl gnupg2 lsb-release -y
```
- `software-properties-common`: easy to manage APT repo
- `gnuog2`: for verifying digital signature

``` bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
```

- ROS2 GPG key download and install
- Add ROS2 repository to APT repos

### ROS2 HUMBLE

``` bash
sudo apt install ros-humble-desktop -y

sudo apt install ros-dev-tools -y
# for ROS2 core, development tools and other dependency
```

```bash
source /opt/ros/humble/setup.bash

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
# You can turn on ROS2 environment and for automatically active environment, add this code to bashrc

source ~/.bashrc
```

```bash
mkdir -p ~/ros2_cfi_project/src
cd ~/ros2_cfi_project

colcon build
# in here, we build workspace

source install/setup.bash
# sourcing workspace's environment
```

``` bash
sudo apt install gdb valgrind -y
```
- Additionally, You can install Debugging Tool for future
