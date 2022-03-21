# Integrated-Design-Project

*This is a project for 2nd-year engineers in the University of Cambridge.*

Scenario:
Many robot applications require the ability of a robot to carry out a task to a high degree of precision. In this task
you are required to build a robot to demonstrate the collection, detection and delivery of blocks to a high degree of
precision in a structured environment.
Packages will be randomly positioned within a box, they must be sorted by colour and then accurately placed on the
target areas.

![image](https://user-images.githubusercontent.com/94389344/155006604-c28d0354-6b6b-496e-a3ca-8a9cd7bffe59.png)


Log:
Feb.18-20: 
Discussed the pros & cons and biggest challenges for each algorithm, made the choice of main algorithms 
Discussed with ME subteam about the choice and position of wheels. 
Discussed with EE subteam about the choice and position of sensors. 
Made the decision to use the line following algorithm to solve the task. 
Did some research and had some basic ideas of coding.

Feb.21: 
Built basic infrastructure of software code. 
Discussed with EE subteam and implemented the interface between hardware and software (i.e. coding in Arduino; control input and output of electronics). 
Implemented moving and turning algorithm based on motor control, line following algorithm based on PID control (with the error referred to as the difference between desired value and true value of line sensors).

Feb.22: 
Without a robot, runned the code on the Arduino IDE and debugged it. 
Discussed with EE subteam, and found out that the output of line sensors can be digital or analog depending on the circuit. Compared to digital output, analog output could give a more precise error value, which could be good for a more precise PID control. However after doing experiments with EE subteam we found that the analog data feedback fluctuated a lot and may not be a good choice for used to determine the PID error. Hence we provided codes of both analog and digital read versions and wished to compare the difference the next day.

Feb.23: 
EE subteam adopted the idea of technicians and determined to use the circuit that gives digital output. We discussed that with EE subteam and adapted our codes.
Tried to add an intersection detection algorithm based on data output from line sensors . After testing the line sensors, we found that sometimes there could be errors in feedback data. To solve this problem, we used queue data structure to construct a stable intersection detection algorithm. (The idea was that the output data [0 / 1]  is popped into the queue at a constant rate, and an intersection is detected if the number of 1s in the queue meets a threshold.)  

Feb.24: 
Adapted the code with the motor control library Adafruit_DCMotor 
Determined the main loop logic of navigation & line detection algorithm
Discussed with ME subteam about design of board chassis.

Feb.26:
Built the block detection algorithm to measure the distance and orientation of the block from the robot based on distance sensors assuming ideal conditions.

Feb.28:
Discussed with EE subteam about testing distance sensors. Found out that the feedback from distance sensor fluctuated a lot (Distance - Voltage diagram is not linear; small voltage diff indicates large diff in distance; the voltage feedbacks always fluctuated in a large range)
Ideas about adapting Block detection algorithm. For measurements of orientation, we still tried to use distance sensors to do that, but took imprecision into consideration, the idea was to use queue data structure and concept of “average”or “threshold” to solve the problem. For measurements of distance, due to large variation in distance sensor, we abandoned the idea using distance sensor, and used colour sensor instead (idea is that the colour sensor only works in very short range, the steep change in the feedback of colour sensor indicates that the robot is within a very close range of the block, and hence can be used to determine the distance once we have the orientation.)
Optimised the logic of algorithms. Hardcoded with combined movements and detection algorithms to perform specific tasks.

Mar.1:
Integrated the projects. Runned the robot. Debugged, fine-tuned.

Mar.2:
Refitted wheel, distance sensor, motor etc.

Mar.3:
Testing. Changed the motor.

Mar.4:
Testing

Mar.5:
Meeting with the team. Design of the grabber. Implemented …

Mar.7:
Testing with colour.
Add stray protection algorithm based on L2R2 line sensors.

Mar.8:
Debug. Optimise the logics. Add a button to control the initialization of the program.

