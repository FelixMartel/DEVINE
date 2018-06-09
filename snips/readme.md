Snips ROS Package
================================

Specialized ROS package for the integration 

## Dependenies

1. [ROS](https://github.com/FelixMartel/IRL-1/blob/devine_install/INSTALL.md)
2. paho-mqtt -> `sudo -H pip install paho-mqtt`

## Installation
1. [Setup snips](https://github.com/snipsco/snips-platform-documentation/wiki/1.-Setup-the-Snips-Voice-Platform) for Debian/amd64 

        If you are running on Ubuntu, make sure you edit the `/etc/apt/sources.list.d/snips.list` to point to `jessie`.
        deb https://debian.snips.ai/jessie stable main

2. Run the install script

    `./install_package.bash`

    or 

    `./install_package.zsh`

3. Put DEVINE's assistant. from [Installing an assistant](https://github.com/snipsco/snips-platform-documentation/wiki/2.-Create-an-assistant-using-an-existing-bundle#step-2-download-your-assistant):
    ```
    sudo rm -r /usr/share/snips/assistant
    sudo cp -R assistant /usr/share/snips/assistant
    ```

3. Build the module using catkin\_make:
```
roscd
cd ..
catkin_make
```

## Usage
Run in seperate shells:
```bash
roscore #starts ROS master
rosrun snips snips.py #runs snips node
rostopic echo /answer #listens to the answers
rostopic pub /question std_msgs/String "Is the object blue ?" #ask away!
```