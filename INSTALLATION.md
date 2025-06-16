# Unicorn Race Stack Installation
Before getting started, please note that the Unicorn Race Stack uses `Cartographer` and most of its stacks are written primarily in `Python`. Therefore, we strongly recommend using an `Intel NUC` for optimal performance. Additionally, if you are using a recent NUC model, please be aware that `Ubuntu 20.04 is not supported`. We recommend installing a `compatible Ubuntu version` supported by **your NUC** and performing the installation via ***Docker***.

## Prerequisites 

A 1/10 scale F1TENTH vehicle is used as the hardware platform. If you do not have one prepared, please refer to the [official F1TENTH instructions](https://f1tenth.org/build).

Additionally, the following extra devices were used to improve performance and monitor the system state.

- [blink](https://blink1.thingm.com/)
- 3DM-GX5-25 imu
                                                                      
To deploy or develop using Docker, make sure you have both `Docker` and `Docker Compose` installed. 
And make sure to have docker accessible **without** sudo, follow the official docker post-installation steps [here](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user).

  - [Docker / Docker Compose]((https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository).)

For your reference, the Unicorn Race Stack uses `ROS Noetic`.

<!-- **Note**:
Be sure to have included the sourcing lines in your `~/.bashrc` file, in order to properly setup ROS in every terminal you open. 
The two lines to be added are (if you are using `bash`)
```
source /opt/ros/noetic/setup.bash
source <path to your catkin_ws>/devel/setup.bash
``` -->

## Clone the Repo

Before cloning, make sure to create the appropriate directory.
```
mkdir ~/unicorn_ws && cd ~/unicorn_ws

mkdir -p cache/noetic/build cache/noetic/devel cache/noetic/logs
```

Recursive clone the main repository together with it's submodules!
```bash
git clone --recurse-submodules https://github.com/HMCL-UNIST/UNICORN.git && cd UNICORN
```

## udev Rules Setup
To ensure that the USB-connected devices are recognized consistently and accessible without requiring manual reconfiguration, a set of custom udev rules must be installed.

For installation of custom udev rules, run the script with root privileges:

```bash
# for Blink1
sudo ./.devcontainer/.install_utils/udev_rules_blink1.sh

# for IMU
sudo ./.devcontainer/.install_utils/udev_rules_imu.sh

# for VESC
sudo ./.devcontainer/.install_utils/udev_rules_vesc.sh
```
Then check if the devices are correctly linked:

```bash
# for IMU
ls -l /dev/IMU

# for VESC
ls -l /dev/VESC
```

## Build with docker

First, build the base docker image with `docker compose`:
```bash
docker compose build base_x86
```

Then export the needed environment variables and build the simulator container:
```bash
export UID=$(id -u)
export GID=$(id -g)
docker compose build nuc
```

Then Open the workspace from VS Code:
```bash
code ~/unicorn_ws
```

In the VS Code window, press `Ctrl + Shift + P` to open the command palette. From the banner that appears, select `Dev Containers: Rebuild and Reopen in Container`. This will allow VS Code to automatically connect to the container.
By installing the `Remote Development` extension from the `Extensions` tab in VS Code, you will be able to easily reconnect to previously created containers in the future.

## How to use GUI applications with the container
To have more information on how to use GUI applications with the container, please refer to the [GUI applications documentation](./.docker_utils/README_GUI.md).

