Snips ROS Package
================================

Specialized ROS package for the integration 

## Installation
# 1.Installing Snips Voice Platform

Run the following commands in the terminal to install Snips Voice Platform

```
sudo apt-get update
sudo apt-get install -y dirmngr
sudo bash -c  'echo "deb https://debian.snips.ai/jessie stable main" > /etc/apt/sources.list.d/snips.list'
sudo apt-key adv --keyserver pgp.mit.edu --recv-keys F727C778CCB0A455

sudo apt-get update
sudo apt-get install -y snips-platform-voice
```

# 2.Configuration
This section shows you how to configure the audio input and output used by Snips.

Step 1. List your devices
Audio output

List all playback devices:

```
$ aplay -l
** List of PLAYBACK Hardware Devices **
card 0: Intel [HDA Intel], device 0: AD1984A Analog [AD1984A Analog]
 Subdevices: 1/1
 Subdevice #0: subdevice #0
card 0: Intel [HDA Intel], device 2: AD1984A Alt Analog [AD1984A Alt Analog]
 Subdevices: 1/1
 Subdevice #0: subdevice #0
card 1: CODEC [USB audio CODEC], device 0: USB Audio [USB Audio]
 Subdevices: 1/1
 Subdevice #0: subdevice #0
```

In this case, the audio playback device desired is card 0 / device 0 ( hw:0,0 ).

Audio input

List all audio capture devices:

```
$ arecord -l
**** List of CAPTURE Hardware Devices ****
card 1: Device [USB PnP Sound Device], device 0: USB Audio [USB Audio]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
```

In this case, the audio capture device is card 1 / device 0 ( hw:1,0 ).

Step 2. Select an input and output
Navigate to /etc/asound.conf and update the file accordingly.

```
$ sudo nano /etc/asound.conf

pcm.!default {
  type asym
   playback.pcm {
     type plug
     slave.pcm "hw:0,0"
   }
   capture.pcm {
     type plug
     slave.pcm "hw:1,0"
   }
}
```

Step 3. Adjust input and output volume
This commands allows you to set up the sound cards:

```
$ alsamixer
```

The sound card that regulates the input volume is the one of the mic. Use F6 to select this card and F4 to play with the db gain.

Step 4. Troubleshoot your audio setup (Optional)
To run this test, you need to stop snips-audio-server as it takes hold of the microphone:

```
sudo systemctl stop snips-audio-server
```

Execute the following command, speak into your microphone, and hit CTRL-C when you are done speaking.

```
$ arecord -f cd out.wav
Recording WAVE 'out.wav' : Signed 16 bit Little Endian, Rate 44100 Hz, Stereo
^CAborted by signal Interrupt...
```

Play the recorded sound to validate that your microphone and speaker are working. If not, you might want to play again with the db gain using alsamixer.

```
$ aplay out.wav
Playing WAVE 'out.wav' : Signed 16 bit Little Endian, Rate 44100 Hz, Stereo
```

Don't forget to restart the audio server:

```
sudo systemctl start snips-audio-server
```

# 3.Assistant
Go to snips.ai and download a premade or custom Assistant. Once downloaded, extract the assistant folder to /usr/share/snips/assistant via the following command:

```
sudo mv ./assistant /usr/share/snips/assistant
```

You will also need to restart Snips to take the new assistant into account, which is achieved by issuing the command
sudo systemctl restart "snips*"

# 4.Install Snips-Watch
You can install snips-watch to monitor the bus:

```
sudo apt-get install snips-watch
```

Then you can start snips-watch to know about the successful interactions. You also use one or more -v (snips-watch -vvv) to see more details during the various stages of the conversation.

## Run the install script `./install_package.bash` 

## Build the module using catkin_make:
```bash
roscd
cd ..
catkin_make
```

## Usage
```bash
roscore #start ROS master
rosrun snips snips.py #run snips node
rostopic echo /snips_answer #listen to the answers
rostopic pub /snips_ask std_msgs/String "Is the object blue ?" #ask away !
``` 

## Troubleshooting
# Snips not firing responses

If Snips is receiving data correctly but is having trouble mapping it to its Intent, try redownloading the correct assistant (or see the folder assistant on the drive).

