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


### Tuning dt and N
I optimized N and dt over MANY tests, I recorded the notable ones at the top of MPC.cpp, also pasting below. I optimized it at 40 MPH, and got values of N = 11 and dt = .09 to complete the track. These worked BEST in my experience with my costs, but I ended up going with N = 10 and dt = .10, because the dt=.10 meshes well with our 100 ms 'latency' so i can easily include the latency in MPC. If we look at too many waypoints (N), it can slow down our calculations too much. If we look too far into the future (high N and dt), we risk the polynomial not being fit properly to too complex of a curve.  If the dt is too small, we need more waypoints to get down the road and again, we run into complex curves not fitting to a polynomial well. If dt is too high, you can miss details in the track, and again risk not getting a polynomial to fit properly. 

* N = 25, dt=.05, slow to process
* N = 13, dt=.1, works well
* N = 13, dt=.15, Looks really far out, rough around sharp corners
* N = 13, dt=.1, works well
* N = 7, dt=.1, doesnt look far enough ahead
* N = 7, dt=.15, does not get a good formed polynomial
* N = 9, dt=.15, Does not work well around sharp corners
* N = 10, dt=.1, works well
* N = 11, dt=.11, works well
* N = 11, dt=.09, works well
* N = 10, dt=.1, Used this in the end to make latency 'delay' easier as dt=.1 is equivalent
//to 100ms for our delay in processing.

My update functions came from the Mind The Line Lesson:  [Mind The Line](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/338b458f-7ebf-449c-9ad1-611eb933b076/concepts/ee21948d-7fad-4821-b61c-0d69bbfcc425)
### Costs
fg[0] holds the cost value, and we asssign higher cost to cte and epsi because we want the car to stay on the track at all costs.

```
const int cte_cost = 15;
const int epsi_cost = 15;
const int v_cost = 1;
const int delta_cost = 5;
const int a_cost = 1;
const int delta_d_cost = 1000;
const int delta_a_cost = 5;
```

### Waypoints from cars pov

```
for( int i = 0; i < ptsx.size(); i++ ) {
  double x = ptsx[i] - px;
  double y = ptsy[i] - py;
  ptsx[i] = ( x * cos( 0 - psi ) - y * sin( 0 - psi ) );
  ptsy[i] = ( x * sin( 0 - psi ) + y * cos( 0 - psi ) );
}
```

### Latency 
I skip the first indece of the variable upper and lower limit loops and feed it the current throttle / steering angle for the skipped index. 

```
for ( int i = 0; i < delta_start; i++ ) {
  	vars_lowerbound[i] = -1.0e19;
  	vars_upperbound[i] = 1.0e19;
  }
  //Bound the upper and lower bounds to current steering angle for the length of our latency.
  for ( int i = delta_start; i < delta_start + latency; i++) {
  	vars_lowerbound[i] = steering_angle;
  	vars_upperbound[i] = steering_angle;
  }
  for ( int i = delta_start + latency; i < a_start; i++ ) {
  	vars_lowerbound[i] = -0.4363;
  	vars_upperbound[i] = 0.4363;
  }
  //Bound the upper and lower bounds to current throttle for the length of our latency.
  for ( int i = a_start; i < a_start + latency; i++) {
  	vars_lowerbound[i] = throttle;
  	vars_upperbound[i] = throttle;
  }
  for ( int i = a_start + latency; i < n_vars; i++ ) {
  	vars_lowerbound[i] = -1.0;
  	vars_upperbound[i] = 1.0;
  }
```
Finally in main.cpp we use the information from MPC to update the steering angle and throttle accordingly, after mapping the polynomial to the track.
