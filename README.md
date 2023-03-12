# Arduino-PID-controller-robot

Arduino script which controls a robot equipped with 5 IR sensors located at the bottom. The purpose of the script is to enable the robot to follow a black line using PID controllers.

The IR sensors detect the contrast between the black line and the surface it is on. The PID controller is a feedback control mechanism that continuously calculates the error between the robot's position and the desired position (the black line) and adjusts the robot's movement accordingly. The PID controller takes into account the proportional, integral, and derivative factors to minimize the error and ensure that the robot stays on the black line.

Robot used for this script:
- Model: AlphaBot
- Brand: WaveShare
- Board: Arduino UNO
