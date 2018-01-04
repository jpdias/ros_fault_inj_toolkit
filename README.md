#  ROS Fault Injection Toolkit

## Scope

As of today, robotics is widespread in the world, ranging from different applications and use cases. One of most trending topics is autonomous driving, with companies like Google and Tesla investing into it. This system highly depends on sensing the environment in order to make decisions and move correctly in, e.g., roads or industrial floors. One of the most used sensors is cameras. The images collected from cameras are processed at different levels until a final decision is taken on the car next move (e.g. turn or break). 

One of the used mechanisms for the development of such systems is the *Conde_Simulator*  (ROS with Gazebo). This system allows the development of autonomous vehicles and deploys the *same* system on a real robot/car. However, in real use cases, there are high error rates on sensors readings, including camera images (e.g. freezes) and the simulators do not cover these problematics. Additionally, and from another viewpoint, with the advent of cloud computing and cloud robotics, there are situations when the robot depends on external modules or information in order to act, a condition which raises security concerns.

As so, there is the lack of a way to intentionally inject payloads (by the means of messages) in the system as a way to test its reliability, fault-tolerance, and, even further, attack mitigation and fail-safes. Within this scope, we propose the development of a toolkit in such way that it allows one to intentionally analyze, modify and inject information into the *Conde_Simulator* system.

## Setup

### Software Dependencies

- ROS distro: [ROS Indigo Full](http://wiki.ros.org/indigo)
- Linux version: [Linux Lubuntu 14.04 LTS](https://lubuntu.net/)
- Gazebo version: [Gazebo 2.2.3](http://gazebosim.org/)
- Conde-simulator version: [2017-09-07](https://bitbucket.org/ee09115/conde_simulator_student)
- Python version: [Python 2.7.14](https://www.python.org/)

### Using the Toolkit

#### Environment Setup

1. Launch _conde-simulator_ world simulator.
 ```bash
 $ roslaunch conde_world main.launch 
 ```
2.  Launch _conde-simulator_ camera tracking. Launch files available [here](conde_simualtor). 
 ```bash
 $ roslaunch conde_tracking run_fake.launch
 ``` 
 

3. Make the robot move forward.
 ```bash 
 $ ./move_scripts/samplemove.sh
 ``` 

4. Launch the toolkit with the fault to inject.
 ```bash
 $ python script.py -f FAULTTYPE 
 ```

#### Toolkit Help

```bash
usage: ros-inj-tool.py [-h] -f FAULTTYPE

ROS queue fault-injection toolkit

optional arguments:
  -h, --help            show this help message and exit
  -f FAULTTYPE, --faulttype FAULTTYPE

      FAULTTYPE: RANDOM, SLOW, FREEZE, INJECTPAYLOAD, CAMERASTORE, PARTIALLOSS
      
        RANDOM: Shuffles the camera feed images bytes
        SLOW: Introduces a slowness in the camara image stream
        FREEZE: Freezes a camara feed for n interations
        INJECTPAYLOAD: Injects personalized images into the queue
        CAMERASTORE: Stores all images in camera feeds
        PARTIALLOSS: Loss of part of the image

        -- Additional configuration in config.ini file.
```

A documented sample of the configuration file is available: [config.ini](config.ini)

## Institutions

![INESC TEC](/resources/inesc.png)
![FEUP](/resources/feup.png)

## Authors

[João Pedro Dias](http://jpdias.me)