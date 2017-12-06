#  ROS Fault Injection ToolkitROS Fault Injection Toolkit

## Scope

As of today, robotics is widespread in the world, ranging from different applications and use cases. One of most trending topics is autonomous driving, with companies like Google and Tesla investing into it. This system highly depends on sensing the environment in order to make decisions and move correctly in, e.g., roads or industrial floors. One of the most used sensors is cameras. The images collected from cameras are processed at different levels until a final decision is taken on the car next move (e.g. turn or break). 

One of the used mechanisms for the development of such systems is the *Conde_Simulator*  (ROS with Gazebo). This system allows the development of autonomous vehicles and deploys the *same* system on a real robot/car. However, in real use cases, there are high error rates on sensors readings, including camera images (e.g. freezes) and the simulators do not cover these problematics. Additionally, and from another viewpoint, with the advent of cloud computing and cloud robotics, there are situations when the robot depends on external modules or information in order to act, a condition which raises security concerns.

As so, there is the lack of a way to intentionally inject payloads (by the means of messages) in the system as a way to test its reliability, fault-tolerance, and, even further, attack mitigation and fail-safes. Within this scope, we propose the development of a toolkit in such way that it allows one to intentionally analyze, modify and inject information into the *Conde_Simulator* system.

## Goals

### Primary Goals

- GUI & CLI modes
- Explore messages between modules as the system is running (eavesdropping)
- Freeze messages (keep sending the same reading)
- Inject messages
- Message queue flood attack
 
### Secundary Goals

- Intercept & Replay messages
- Explore hardening solutions to increase security and fault-tolerance (e.g. encryption)


## Deliverables

### First Component (#3)

- State of the art in robotics simulation systems in the scope of security, fault-tolerance & testing
  - Focus on autonumous driving
- Tools available
- Evaluation scenarios
- Tool Requirements & Functionalities mock-up

### Second Component (#4)

- Article
- Tool covering *at least* the primary goals
- Evaluation use scenarios

## Authors

[João Pedro Dias](http://jpdias.me)