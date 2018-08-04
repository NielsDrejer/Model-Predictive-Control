## Udacity Self-Driving Car Engineer Nanodegree, Term 2, Project 5: Model Predictive Control

The goal of this project is to implement Model Predictive Control to drive a car around a simulated track.

The car and track simulator is the one from the Behavioral cloning project of term 1, this time running in the term 2 simulator application.

The Model Predictive Control algorithm receives telemetry data in the form of way points, current position, car angle and velocity from the simulator. Based on these it shall calculate steering and acceleration and send this back to the simulator and keep the car on track.
One main challenge is a 100 ms latency, which the algorithm must be able to cope with.

Seed code for the project is provided, and the task is to complete this. Seed code is taken from here:

https://github.com/udacity/CarND-MPC-Project

## Notes about Installation

I am using a Mac for development.

The uWebSockets were already installed in connection with project 1 of term 2.

The additionally required package ipopt and cppad could not be installed via the provided instructions, so I ended up using the helpful instructions from this link:

https://discussions.udacity.com/t/installing-ipopt-and-cppad-on-mac-solution-with-docker/508316

to run the mpc binary inside a Docker container, leaving the simulator on my Mac. I assume that my code will run directly on other systems which have the required libraries installed, but have not been able to verify this.

Inside the docker container I could directly execute the instructions:

1. mkdir build
2. cd build
3. cmake ..
4. make all

After that everything worked as expected. I used the Sublime text editor and make on the command line in the docker container to build.

Please note that I only submit the src files, but not the build folder. Please follow the 4 steps above to test my code.

## Rubric Points

### Compilation

#### Your code should compile  

Calling 'make all' on the command line compiles the code without warnings.

### Implementation

#### The Model

The code in the mpc.cpp FGEval class is based on the kinematic model taught in Lesson 19.

The model state includes the following 6 parameters:

* The vehicles x coordinate.
* The vehicles y coordinate.
* The vehicles orientation angle psi.
* The vehicles velocity v.
* The cross track error, measuring the difference between the wanted and the actual vehicle trajectory
* The psi error, measuring the difference between wanted and actual vehicle orientation.

Further the model incorporates actuator values for throttle a and steering angle delta. The throttle value models both gas and brake and is thus a value between -1 and 1. The steering angle is limited to a value between -25 and +25 degrees, to model the steering of a real car.

The kinematic model is summarized in the following equations for calculating the state at time t+1, based on the state at time t and the actuator values:

x_[t+1] = x[t] + v[t] \* cos(psi[t]) \* dt

y_[t+1] = y[t] + v[t] \* sin(psi[t]) \* dt

psi_[t+1] = psi[t] + (v[t] / Lf) \* delta[t] * dt

v_[t+1] = v[t] + a[t] \* dt

cte[t+1] = f(x[t]) - y[t] + v[t] \* sin(epsi[t]) \* dt

epsi[t+1] = psi[t] - psides[t] + v[t] \* (delta[t] / Lf) \* dt

#### Timestep Length and Elapsed Duration (N & dt)

I chose these values to be N = 10 and dt = 0.1, based on suggestions in the project Slack channel.

I tried different other values such as 25 and 0.05, and found that the solution quickly produced very instable behavior. The whole system thus appears to be extremely dependent on these values, and "incorrect" values lead to dramatic malfunction. This is in my humble opinion a very negative side to the model predictive control method.

#### Polynomial Fitting and MPC Preprocessing

Also based on suggestions in the project Slack channel I changed the way points to be relative to the car perspective, instead of their global perspective.

This has the effect that the vehicle x,y position becomes 0,0 and its orientation angle becomes 0 for the polynomial fitting of the waypoints.

Another suggestion from the Slack channel and the project hints without which the controller does not work is to weight the elements of the cost function. The main change here is to put (much) more weight to the cte and epsi parts of the cost function. I multiply these values with 3000, and found this via trial and error.

Originally I used only the cost elements from the lesson, but ended up adding (thanks to Slack channel hint) an element which is the product of velocity and delta. This leads to a much more calm behavior of the car as changes of steering and speed are made smaller. This element is also weighted heavily in the total cost by multiplying it with 500.

All the weights I used were found by trial and error. I did not really go into this, but I assume a more systematic look at the actual sizes of the individual parts of the cost function would deliver better reasons as to why for example it is necessary to multiply the cte with a value of several thousand to get the system to behave.

#### Model Predictive Control with Latency

Without handling for dealing with latency the model will not work. In only found a way of dealing with this in the project Slack channel. The method chosen was to apply the actuations 1 time step later (see mpc.cpp line 117-121) to account for the fact that they physically also don't take immediate effect.

I think this explains the sensitivity of dt mentioned above. The chosen dt of 0.1 matches the 100 ms latency, and this works quite well. Different values lead to instable behavior of the controller.

### Simulation

#### The vehicle must successfully drive a lap around the track

The vehicle did that. I even increased the reference speed from 40 to 60 just for the fun of it. I did not experiment further with how fast the car can go. I observe that the vehicle goes fairly slow around corners with the current settings.

### Reflection

Without the various hints, mainly from the project Slack channel, I would not have been able to get the project to work. I find it a little bit problematic that absolute requirements are not taught clearly in the lessons, and also personally unsatisfying to have to rely on hints from others like that to get the whole thing to work.

I hope this project is deemed to obey the Honor Code of Udacity although I had to use help from other to make it.
