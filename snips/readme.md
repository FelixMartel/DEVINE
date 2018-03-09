Snips ROS Package
================================

Specialized ROS package for the integration 

## Installation
1. [Setup snips](https://github.com/snipsco/snips-platform-documentation/wiki/1.-Setup-the-Snips-Voice-Platform) for Ubuntu 16.04
2. Download the simple yes-no assistant
3. Run the install script `./install_package.bash` 

## Usage
```bash
roscore #start ROS master
rosrun snips snips.py #run snips node
rostopic echo /snips_answer #listen to the answers
rostopic pub /snipsAsk std_msgs/String "Is the object blue ?" #ask away !
``` 
