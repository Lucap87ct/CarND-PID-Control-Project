# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

![](record.gif)

The Project
---

In this project a PID controller is implemented to control the steering angle of the self-driving car depending on the current displacement error from the center of the lane (CTE or Cross-Track error), with the Udacity driving simulator.
On the other side the vehicle speed is set constant.

The three components of the PID controller have the following effects.

The Proportional component is acting directly on the current displacement error.
During the tuning, it was observed that if it's too high the vehicle will correct too harshly and we will have an unstable response. On the other side, if it's too low the settling time will be to low and the vehicle will go out of the track.

The Integral component is acting on the steady-state behavior and can correct for steering bias, such as a constant mis-orientation of the wheels. During the tuning, it was observed that if it's too high we will have overshoot.

The Derivative component is acting on the difference between the current error and the previous error. It was observed that when it is too high, we have overshoot and instability. If it's too low, the vehicle will react too slowly and it will drive off the track.

Parameter tuning
---

It was observed that the vehicle is very sensitive to small variations, so P, I and D components were tuned manually in order to achieve staying inside the track.
I also implemented and played with the twiddle algorithm in order to optimize the parameter, but no appreciatable results were obtained. Therefore the manually tuned parameters were kept.
Final values are:
* P = 0.1
* I = 0.01
* D = 2.5

Code guidelines
---

* The code for the communication with the simulator is inside src/main.cpp. Also the manually tuned PID parameters are there.
* The PID algorithm is inside src/PID.h and src/PID.cpp. The twiddle implementation is in PID.cpp in lines 60 to 123.

Reflection
---

All in all, the vehicle is able to stay in the track without touching lines or unsafe surfaces. The behavior is still not optimal in curves where there are some oscillations that could not be compensated by PID parameter tuning. A more advanced control strategy using preview of the curve as well as knowledge of the vehicle model could help improve the behavior.
