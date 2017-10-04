# MPC Project
The goal is to minimize the cost function.

We have two 'actuator' variables (items we control)

* Acceleration ( -1 to 1 )
* Steering Angle ( -25 to 25 )

We have 6 'state' variables

* X position
* Y position
* Psi orientation
* Velocity
* Cross Track Error
* Orientation Error (epsi)

I optimized N and dt over MANY tests, I recorded the notable ones at the top of MPC.cpp, also pasting below. I optimized it at 40 MPH, and then did a couple extra tests to attempt to pass at 55 MPH all the way around and got values of N = 11 and dt = .09 to complete the track. 

* N = 25, dt=.05, slow to process
* N = 13, dt=.1, works well
* N = 13, dt=.15, Looks really far out, rough around sharp corners
* N = 13, dt=.1, works well
* N = 7, dt=.1, doesnt look far enough ahead
* N = 7, dt=.15, does not get a good formed polynomial
* N = 9, dt=.15, Does not work well around sharp corners
* N = 10, dt=.1, works well
* N = 11, dt=.11, works well, did not complete course at 55mph
* N = 11, dt=.09, worked best so far, DID complete course at 55mph

My update functions came from the Mind The Line Lesson:  [Mind The Line](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/338b458f-7ebf-449c-9ad1-611eb933b076/concepts/ee21948d-7fad-4821-b61c-0d69bbfcc425)

fg[0] holds the cost value, and we asssign higher cost to cte and epsi because we want the car to stay on the track at all costs.

Finally in main.cpp we use the information from MPC to update the steering angle and throttle accordingly, after mapping the polynomial to the track.
