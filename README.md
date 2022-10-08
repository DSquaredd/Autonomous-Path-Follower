# Autonomous-Path-Follower
Program to simulate autonomous path following robot using Coppelia Sim. 

Part of a class project for Introduction to Autonomous Systems. 

OBJECTIVE 

To program the robot to detect and follow the provided track and complete a full lap
even when encountering gaps. This requires writing a program that reads three vision
sensors, and, depending on the state of the vision sensors, adjusts the motor velocities
to either make the robot turn or align itself with the track.

![image](https://user-images.githubusercontent.com/115327300/194683358-367eadb1-acd7-44db-917e-53f2dfac4095.png)

Challenges

Because we are controlling a non-holonomic mobile robot, the left and
right joints need to individually have their velocities set to 0 in order to
brake. The order in which the wheel joint velocities are adjusted can affect
the robot’s trajectory. Even though the instructions to stop both wheels
occur one after the other, the small delay between the execution of the
instructions can affect the robot’s trajectory. Similarly, the delay of
function calls have a similar effect on the robot’s trajectory.

The starting position of the robot affected the sensors that would be
detecting the track. At the beginning of the project, depending on the
robot’s position and the speed in which it curved, the robot had the
possibility of leaving the track or skipping part of it. Also, because the
robot’s position was changed, the sensors obtaining any information about
the track were also modified. The idea of moving its position was to analyze 
different situations that the robot may face in another moment,
searching and storing the information given by the sensors, and how the
robot interacted with the gaps.

RESULTS

We wrote a program that read three vision
sensors, and, depending on the state of the vision sensors, adjusts the motor velocities
to either make the robot turn or align itself with the track. 
As a result the robot was able to follow the path and turn around at the origin to follow the path in the opposite direction. 
Completion time was 1 min and 40 Seconds, 3 min was the maximum time so simulation was successful. 
