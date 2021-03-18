# Traffic_Sign_Detection_and_Auto_Lane_Keeping

The project aims for cross-computer communication of traffic-sign detection and autonomous lane following. The following sub-tasks were attempted to achieve the goal. 
There are two MATLAB scripts used in this project, viz. lane_detection.m, and traffic_sign_detection.m.
The Lane Detection script is detecting the lane lines and predicting the steering angle of the vehicle for HMI representation from a pre-recorded camera feed of an F1/10th vehicle. While, the Traffic-Sign Detection script is used to detect traffic signs. Once a stop sign is detected, the HMI part of the code indicates on the screen that the vehicle should pull the brakes. 
UDP/IP protocol is used for communication between the two scripts. The two scripts can run on two seperate machines or on a single machine as well.




![Layout](https://github.com/anshums/Traffic_Sign_Detection_and_Auto_Lane_Keeping/blob/main/Layout.png)

The above picture shows the layout Software Architecture.



![ADTPrjctGIF](https://github.com/anshums/Traffic_Sign_Detection_and_Auto_Lane_Keeping/blob/main/ADTPrjctGIF.gif)

The final outcome.
